#include "fmi_dp_mock.hpp"

#include <cppfmu_cs.hpp>
#include <zeabuz/common/utilities/enums.hpp>

#include "data_types.hpp"

FmuDpMock::FmuDpMock(const std::string& instanceName) : modelName(instanceName)
{
    initializeValueReferences();
    m_validControlModes["STANDBY"] = DPMock::ControlMode::STANDBY;
    m_validControlModes["DP"] = DPMock::ControlMode::DP;
    m_validControlModes["JOYSTICK"] = DPMock::ControlMode::JOYSTICK;
    m_validControlModes["AUTOPILOT"] = DPMock::ControlMode::AUTOPILOT;

    m_ptrDPController = std::make_unique<DPMock::DPController>();

    bool_signals[m_refValues.enableReadyForAutonomy] = false;
    bool_signals[m_refValues.enableAutonomyMode] = false;

    string_signals[m_refValues.desiredControlMode] = "STANDBY";

    real_signals[m_refValues.measurements.northEastHeading.x] = 0.0;
    real_signals[m_refValues.measurements.northEastHeading.y] = 0.0;
    real_signals[m_refValues.measurements.northEastHeading.z] = 0.0;

    real_signals[m_refValues.measurements.nu.x] = 0.0;
    real_signals[m_refValues.measurements.nu.y] = 0.0;
    real_signals[m_refValues.measurements.nu.z] = 10.0;

    real_signals[m_refValues.refDpManual.northEastHeading.x] = 0.0;
    real_signals[m_refValues.refDpManual.northEastHeading.y] = 0.0;
    real_signals[m_refValues.refDpManual.northEastHeading.z] = 0.0;

    real_signals[m_refValues.refDpManual.northEastHeadingVel.x] = 0.0;
    real_signals[m_refValues.refDpManual.northEastHeadingVel.y] = 0.0;
    real_signals[m_refValues.refDpManual.northEastHeadingVel.z] = 0.0;

    real_signals[m_refValues.refDpManual.northEastHeadingAccel.x] = 0.0;
    real_signals[m_refValues.refDpManual.northEastHeadingAccel.y] = 0.0;
    real_signals[m_refValues.refDpManual.northEastHeadingAccel.z] = 0.0;

    real_signals[m_refValues.refDpManual.surge_integrator_mode] = 0.0;
    real_signals[m_refValues.refDpManual.sway_integrator_mode] = 0.0;
    real_signals[m_refValues.refDpManual.heading_integrator_mode] = 0.0;

    real_signals[m_refValues.refDpAutonomy.northEastHeading.x] = 0.0;
    real_signals[m_refValues.refDpAutonomy.northEastHeading.y] = 0.0;
    real_signals[m_refValues.refDpAutonomy.northEastHeading.z] = 0.0;

    real_signals[m_refValues.refDpAutonomy.northEastHeadingVel.x] = 0.0;
    real_signals[m_refValues.refDpAutonomy.northEastHeadingVel.y] = 0.0;
    real_signals[m_refValues.refDpAutonomy.northEastHeadingVel.z] = 0.0;

    real_signals[m_refValues.refDpAutonomy.northEastHeadingAccel.x] = 0.0;
    real_signals[m_refValues.refDpAutonomy.northEastHeadingAccel.y] = 0.0;
    real_signals[m_refValues.refDpAutonomy.northEastHeadingAccel.z] = 0.0;

    real_signals[m_refValues.refDpAutonomy.surge_integrator_mode] = 0;
    real_signals[m_refValues.refDpAutonomy.sway_integrator_mode] = 0;
    real_signals[m_refValues.refDpAutonomy.heading_integrator_mode] = 0;

    real_signals[m_refValues.refForcesManual.x] = 0.0;
    real_signals[m_refValues.refForcesManual.y] = 0.0;
    real_signals[m_refValues.refForcesManual.z] = 0.0;

    real_signals[m_refValues.refForcesAutonomy.x] = 0.0;
    real_signals[m_refValues.refForcesAutonomy.y] = 0.0;
    real_signals[m_refValues.refForcesAutonomy.z] = 0.0;

    real_signals[m_refValues.parameters.Kp.x] = 0.0;
    real_signals[m_refValues.parameters.Kp.y] = 0.0;
    real_signals[m_refValues.parameters.Kp.z] = 0.0;

    real_signals[m_refValues.parameters.Ki.x] = 0.0;
    real_signals[m_refValues.parameters.Ki.y] = 0.0;
    real_signals[m_refValues.parameters.Ki.z] = 0.0;

    real_signals[m_refValues.parameters.Kd.x] = 0.0;
    real_signals[m_refValues.parameters.Kd.y] = 0.0;
    real_signals[m_refValues.parameters.Kd.z] = 0.0;

    real_signals[m_refValues.parameters.GeneralizedForcesSaturation.x] = 0.0;
    real_signals[m_refValues.parameters.GeneralizedForcesSaturation.y] = 0.0;
    real_signals[m_refValues.parameters.GeneralizedForcesSaturation.z] = 0.0;

    real_signals[m_refValues.parameters.IntegralActionSaturation.x] = 0.0;
    real_signals[m_refValues.parameters.IntegralActionSaturation.y] = 0.0;
    real_signals[m_refValues.parameters.IntegralActionSaturation.z] = 0.0;

    real_signals[m_refValues.parameters.MassMatrixDiagonal.x] = 0.0;
    real_signals[m_refValues.parameters.MassMatrixDiagonal.y] = 0.0;
    real_signals[m_refValues.parameters.MassMatrixDiagonal.z] = 0.0;

    real_signals[m_refValues.parameters.DampingMatrixDiagonal.x] = 0.0;
    real_signals[m_refValues.parameters.DampingMatrixDiagonal.y] = 0.0;
    real_signals[m_refValues.parameters.DampingMatrixDiagonal.z] = 0.0;

    real_signals[m_refValues.parameters.MaxEffort.x] = 0.0;
    real_signals[m_refValues.parameters.MaxEffort.y] = 0.0;
    real_signals[m_refValues.parameters.MaxEffort.z] = 0.0;

    // Outputs
    bool_signals[m_refValues.outputs.readyForAutonomyEnabled] = false;
    bool_signals[m_refValues.outputs.autonomyModeEnabled] = false;
    string_signals[m_refValues.outputs.controlMode] = "STANDBY";

    real_signals[m_refValues.outputs.generalizedForces.x] = 0.0;
    real_signals[m_refValues.outputs.generalizedForces.y] = 0.0;
    real_signals[m_refValues.outputs.generalizedForces.z] = 0.0;

    real_signals[m_refValues.outputs.refDPActive.northEastHeading.x] = 0.0;
    real_signals[m_refValues.outputs.refDPActive.northEastHeading.y] = 0.0;
    real_signals[m_refValues.outputs.refDPActive.northEastHeading.z] = 0.0;

    real_signals[m_refValues.outputs.refDPActive.northEastHeadingVel.x] = 0.0;
    real_signals[m_refValues.outputs.refDPActive.northEastHeadingVel.y] = 0.0;
    real_signals[m_refValues.outputs.refDPActive.northEastHeadingVel.z] = 0.0;

    real_signals[m_refValues.outputs.refDPActive.northEastHeadingAccel.x] = 0.0;
    real_signals[m_refValues.outputs.refDPActive.northEastHeadingAccel.y] = 0.0;
    real_signals[m_refValues.outputs.refDPActive.northEastHeadingAccel.z] = 0.0;

    real_signals[m_refValues.outputs.refDPActive.surge_integrator_mode] = 0.0;
    real_signals[m_refValues.outputs.refDPActive.sway_integrator_mode] = 0.0;
    real_signals[m_refValues.outputs.refDPActive.heading_integrator_mode] = 0.0;

    // autopilot inputs
    real_signals[m_refValues.autopilotInputsAutonomy.heading_ref] = 0.0;
    real_signals[m_refValues.autopilotInputsAutonomy.yaw_rate_ref] = 0.0;
    real_signals[m_refValues.autopilotInputsAutonomy.yaw_accel_ref] = 0.0;
    real_signals[m_refValues.autopilotInputsAutonomy.surge_speed_ref] = 0.0;
    real_signals[m_refValues.autopilotInputsAutonomy.surge_accel_ref] = 0.0;
    real_signals[m_refValues.autopilotInputsAutonomy.surge_integrator_mode] = 0;
    real_signals[m_refValues.autopilotInputsAutonomy.heading_integrator_mode] = 0;

    real_signals[m_refValues.autopilotInputsManual.heading_ref] = 0.0;
    real_signals[m_refValues.autopilotInputsManual.yaw_rate_ref] = 0.0;
    real_signals[m_refValues.autopilotInputsManual.yaw_accel_ref] = 0.0;
    real_signals[m_refValues.autopilotInputsManual.surge_speed_ref] = 0.0;
    real_signals[m_refValues.autopilotInputsManual.surge_accel_ref] = 0.0;
    real_signals[m_refValues.autopilotInputsManual.surge_integrator_mode] = 0;
    real_signals[m_refValues.autopilotInputsManual.heading_integrator_mode] = 0;

    // autopilot parameters
    real_signals[m_refValues.parameters.autopilot.heading_pid.Kp] = 0.0;
    real_signals[m_refValues.parameters.autopilot.heading_pid.Ki] = 0.0;
    real_signals[m_refValues.parameters.autopilot.heading_pid.Kd] = 0.0;
    real_signals[m_refValues.parameters.autopilot.surge_speed_pid.Kp] = 0.0;
    real_signals[m_refValues.parameters.autopilot.surge_speed_pid.Ki] = 0.0;
    real_signals[m_refValues.parameters.autopilot.surge_speed_pid.Kd] = 0.0;

    // autopilot outputs
    real_signals[m_refValues.outputs.autopilot_active_inputs.heading_ref] = 0.0;
    real_signals[m_refValues.outputs.autopilot_active_inputs.yaw_rate_ref] = 0.0;
    real_signals[m_refValues.outputs.autopilot_active_inputs.yaw_accel_ref] = 0.0;
    real_signals[m_refValues.outputs.autopilot_active_inputs.surge_speed_ref] = 0.0;
    real_signals[m_refValues.outputs.autopilot_active_inputs.surge_accel_ref] = 0.0;
    real_signals[m_refValues.outputs.autopilot_active_inputs.surge_integrator_mode] = 0;
    real_signals[m_refValues.outputs.autopilot_active_inputs.heading_integrator_mode] = 0;
}
void FmuDpMock::initializeValueReferences()
{
    m_refValues.enableReadyForAutonomy = 0;
    m_refValues.enableAutonomyMode = 1;
    m_refValues.desiredControlMode = 2;

    m_refValues.measurements.northEastHeading.x = 3;
    m_refValues.measurements.northEastHeading.y = 4;
    m_refValues.measurements.northEastHeading.z = 5;

    m_refValues.measurements.nu.x = 6;
    m_refValues.measurements.nu.y = 7;
    m_refValues.measurements.nu.z = 8;

    m_refValues.refDpManual.northEastHeading.x = 9;
    m_refValues.refDpManual.northEastHeading.y = 10;
    m_refValues.refDpManual.northEastHeading.z = 11;

    m_refValues.refDpManual.northEastHeadingVel.x = 12;
    m_refValues.refDpManual.northEastHeadingVel.y = 13;
    m_refValues.refDpManual.northEastHeadingVel.z = 14;

    m_refValues.refDpManual.northEastHeadingAccel.x = 15;
    m_refValues.refDpManual.northEastHeadingAccel.y = 16;
    m_refValues.refDpManual.northEastHeadingAccel.z = 17;

    m_refValues.refDpManual.surge_integrator_mode = 90;
    m_refValues.refDpManual.sway_integrator_mode = 91;
    m_refValues.refDpManual.heading_integrator_mode = 92;

    m_refValues.refDpAutonomy.northEastHeading.x = 18;
    m_refValues.refDpAutonomy.northEastHeading.y = 19;
    m_refValues.refDpAutonomy.northEastHeading.z = 20;

    m_refValues.refDpAutonomy.northEastHeadingVel.x = 21;
    m_refValues.refDpAutonomy.northEastHeadingVel.y = 22;
    m_refValues.refDpAutonomy.northEastHeadingVel.z = 23;

    m_refValues.refDpAutonomy.northEastHeadingAccel.x = 24;
    m_refValues.refDpAutonomy.northEastHeadingAccel.y = 25;
    m_refValues.refDpAutonomy.northEastHeadingAccel.z = 26;

    m_refValues.refDpAutonomy.surge_integrator_mode = 93;
    m_refValues.refDpAutonomy.sway_integrator_mode = 94;
    m_refValues.refDpAutonomy.heading_integrator_mode = 95;

    m_refValues.refForcesManual.x = 27;
    m_refValues.refForcesManual.y = 28;
    m_refValues.refForcesManual.z = 29;

    m_refValues.refForcesAutonomy.x = 30;
    m_refValues.refForcesAutonomy.y = 31;
    m_refValues.refForcesAutonomy.z = 32;

    // parameters
    m_refValues.parameters.Kp.x = 33;
    m_refValues.parameters.Kp.y = 34;
    m_refValues.parameters.Kp.z = 35;

    m_refValues.parameters.Ki.x = 36;
    m_refValues.parameters.Ki.y = 37;
    m_refValues.parameters.Ki.z = 38;

    m_refValues.parameters.Kd.x = 39;
    m_refValues.parameters.Kd.y = 40;
    m_refValues.parameters.Kd.z = 41;

    m_refValues.parameters.GeneralizedForcesSaturation.x = 48;
    m_refValues.parameters.GeneralizedForcesSaturation.y = 49;
    m_refValues.parameters.GeneralizedForcesSaturation.z = 50;

    m_refValues.parameters.IntegralActionSaturation.x = 51;
    m_refValues.parameters.IntegralActionSaturation.y = 52;
    m_refValues.parameters.IntegralActionSaturation.z = 53;

    m_refValues.parameters.MassMatrixDiagonal.x = 54;
    m_refValues.parameters.MassMatrixDiagonal.y = 55;
    m_refValues.parameters.MassMatrixDiagonal.z = 56;

    m_refValues.parameters.DampingMatrixDiagonal.x = 57;
    m_refValues.parameters.DampingMatrixDiagonal.y = 58;
    m_refValues.parameters.DampingMatrixDiagonal.z = 59;

    m_refValues.parameters.MaxEffort.x = 105;
    m_refValues.parameters.MaxEffort.y = 106;
    m_refValues.parameters.MaxEffort.z = 107;

    // Outputs
    m_refValues.outputs.readyForAutonomyEnabled = 42;
    m_refValues.outputs.autonomyModeEnabled = 43;
    m_refValues.outputs.controlMode = 44;

    m_refValues.outputs.generalizedForces.x = 45;
    m_refValues.outputs.generalizedForces.y = 46;
    m_refValues.outputs.generalizedForces.z = 47;

    m_refValues.outputs.refDPActive.northEastHeading.x = 60;
    m_refValues.outputs.refDPActive.northEastHeading.y = 61;
    m_refValues.outputs.refDPActive.northEastHeading.z = 62;

    m_refValues.outputs.refDPActive.northEastHeadingVel.x = 63;
    m_refValues.outputs.refDPActive.northEastHeadingVel.y = 64;
    m_refValues.outputs.refDPActive.northEastHeadingVel.z = 65;

    m_refValues.outputs.refDPActive.northEastHeadingAccel.x = 66;
    m_refValues.outputs.refDPActive.northEastHeadingAccel.y = 67;
    m_refValues.outputs.refDPActive.northEastHeadingAccel.z = 68;

    m_refValues.outputs.refDPActive.surge_integrator_mode = 96;
    m_refValues.outputs.refDPActive.sway_integrator_mode = 97;
    m_refValues.outputs.refDPActive.heading_integrator_mode = 98;

    // autopilot inputs
    m_refValues.autopilotInputsAutonomy.heading_ref = 69;
    m_refValues.autopilotInputsAutonomy.yaw_rate_ref = 70;
    m_refValues.autopilotInputsAutonomy.yaw_accel_ref = 71;
    m_refValues.autopilotInputsAutonomy.surge_speed_ref = 72;
    m_refValues.autopilotInputsAutonomy.surge_accel_ref = 73;
    m_refValues.autopilotInputsAutonomy.surge_integrator_mode = 99;
    m_refValues.autopilotInputsAutonomy.heading_integrator_mode = 100;

    m_refValues.autopilotInputsManual.heading_ref = 74;
    m_refValues.autopilotInputsManual.yaw_rate_ref = 75;
    m_refValues.autopilotInputsManual.yaw_accel_ref = 76;
    m_refValues.autopilotInputsManual.surge_speed_ref = 77;
    m_refValues.autopilotInputsManual.surge_accel_ref = 78;
    m_refValues.autopilotInputsManual.surge_integrator_mode = 101;
    m_refValues.autopilotInputsManual.heading_integrator_mode = 102;

    // autopilot parameters
    m_refValues.parameters.autopilot.heading_pid.Kp = 79;
    m_refValues.parameters.autopilot.heading_pid.Ki = 80;
    m_refValues.parameters.autopilot.heading_pid.Kd = 81;

    m_refValues.parameters.autopilot.surge_speed_pid.Kp = 82;
    m_refValues.parameters.autopilot.surge_speed_pid.Ki = 83;
    m_refValues.parameters.autopilot.surge_speed_pid.Kd = 84;

    // autopilot outputs
    m_refValues.outputs.autopilot_active_inputs.heading_ref = 85;
    m_refValues.outputs.autopilot_active_inputs.yaw_rate_ref = 86;
    m_refValues.outputs.autopilot_active_inputs.yaw_accel_ref = 87;
    m_refValues.outputs.autopilot_active_inputs.surge_speed_ref = 88;
    m_refValues.outputs.autopilot_active_inputs.surge_accel_ref = 89;
    m_refValues.outputs.autopilot_active_inputs.surge_integrator_mode = 103;
    m_refValues.outputs.autopilot_active_inputs.heading_integrator_mode = 104;
}

ValueReferences FmuDpMock::getValueReferences()
{
    return m_refValues;
}

void FmuDpMock::Reset()
{
}

void FmuDpMock::ExitInitializationMode()
{
}

void FmuDpMock::SetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIString value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        string_signals.at(vr[i]) = value[i];
        if (string_signals.find(vr[i]) == string_signals.end()) {
            std::stringstream errorMessage;
            errorMessage << "Error in SetString. Invalid string_signals reference value: " << vr[i];
            throw std::invalid_argument(errorMessage.str());
        }
        else {
            string_signals.at(vr[i]) = value[i];
        }
    }
}

void FmuDpMock::SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        if (real_signals.find(vr[i]) == real_signals.end()) {
            std::stringstream errorMessage;
            errorMessage << "Error in SetReal. Invalid real_signals reference value: " << vr[i];
            throw std::invalid_argument(errorMessage.str());
        }
        else {
            real_signals.at(vr[i]) = value[i];
        }
    }
}

void FmuDpMock::SetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIBoolean value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        if (bool_signals.find(vr[i]) == bool_signals.end()) {
            std::stringstream errorMessage;
            errorMessage << "Error in SetBoolean. Invalid bool_signals reference value: " << vr[i];
            throw std::invalid_argument(errorMessage.str());
        }
        else {
            bool_signals.at(vr[i]) = value[i];
        }
    }
};

void FmuDpMock::GetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIString value[]) const
{
    for (std::size_t i = 0; i < nvr; ++i) {
        if (string_signals.find(vr[i]) == string_signals.end()) {
            std::stringstream errorMessage;
            errorMessage << "Error in GetString. Invalid string_signals reference value: " << vr[i];
            throw std::invalid_argument(errorMessage.str());
        }
        else {
            value[i] = string_signals.at(vr[i]).c_str();
        }
    }
}

void FmuDpMock::GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const
{
    for (std::size_t i = 0; i < nvr; ++i) {
        if (real_signals.find(vr[i]) == real_signals.end()) {
            std::stringstream errorMessage;
            errorMessage << "Error in GetReal. Invalid real_signals reference value: " << vr[i];
            throw std::invalid_argument(errorMessage.str());
        }
        else {
            value[i] = real_signals.at(vr[i]);
        }
    }
}

void FmuDpMock::GetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIBoolean value[]) const
{
    for (std::size_t i = 0; i < nvr; ++i) {
        if (bool_signals.find(vr[i]) == bool_signals.end()) {
            std::stringstream errorMessage;
            errorMessage << "Error in GetBoolean. Invalid bool_signals reference value: " << vr[i];
            throw std::invalid_argument(errorMessage.str());
        }
        else {
            value[i] = bool_signals.at(vr[i]);
        }
    }
}

bool FmuDpMock::DoStep(cppfmu::FMIReal /*currentCommunicationPoint*/, cppfmu::FMIReal dt,
                       cppfmu::FMIBoolean /*newStep*/, cppfmu::FMIReal& /*endOfStep*/)
{
    updateActiveInputs(dt);
    updateActiveParameters();

    m_ptrDPController->configure(m_activeDPParameters);
    m_ptrDPController->step(m_activeDPInputs);
    updateOutputs();
    return true;
}

void FmuDpMock::updateActiveInputs(double dt)
{
    m_activeDPInputs.enable_autonomy_mode = bool_signals.at(m_refValues.enableAutonomyMode);
    m_activeDPInputs.enable_ready_for_autonomy = bool_signals.at(m_refValues.enableReadyForAutonomy);
    m_activeDPInputs.step_size = std::chrono::duration<double>(dt);
    m_activeDPInputs.desired_control_mode = parseControlMode(string_signals.at(m_refValues.desiredControlMode));

    m_activeDPInputs.measurements.north_east_heading = extract3dArray(m_refValues.measurements.northEastHeading);
    m_activeDPInputs.measurements.nu = extract3dArray(m_refValues.measurements.nu);

    m_activeDPInputs.dp_ref_manual.north_east_heading = extract3dArray(m_refValues.refDpManual.northEastHeading);
    m_activeDPInputs.dp_ref_manual.north_east_heading_vel = extract3dArray(m_refValues.refDpManual.northEastHeadingVel);
    m_activeDPInputs.dp_ref_manual.north_east_heading_accel =
        extract3dArray(m_refValues.refDpManual.northEastHeadingAccel);

    m_activeDPInputs.dp_ref_autonomy.north_east_heading = extract3dArray(m_refValues.refDpAutonomy.northEastHeading);
    m_activeDPInputs.dp_ref_autonomy.north_east_heading_vel =
        extract3dArray(m_refValues.refDpAutonomy.northEastHeadingVel);
    m_activeDPInputs.dp_ref_autonomy.north_east_heading_accel =
        extract3dArray(m_refValues.refDpAutonomy.northEastHeadingAccel);

    using type = DPMock::IntegrationMode;
    auto cast = [&](auto& real) {
        return zeabuz::common::utilities::enums::enum_cast<type>(static_cast<int16_t>(real));
    };
    m_activeDPInputs.dp_ref_autonomy.surge_integrator_mode =
        cast(real_signals.at(m_refValues.refDpAutonomy.surge_integrator_mode));
    m_activeDPInputs.dp_ref_autonomy.sway_integrator_mode =
        cast(real_signals.at(m_refValues.refDpAutonomy.sway_integrator_mode));
    m_activeDPInputs.dp_ref_autonomy.heading_integrator_mode =
        cast(real_signals.at(m_refValues.refDpAutonomy.heading_integrator_mode));

    m_activeDPInputs.forces_ref_manual = extract3dArray(m_refValues.refForcesManual);
    m_activeDPInputs.forces_ref_autonomy = extract3dArray(m_refValues.refForcesAutonomy);

    m_activeDPInputs.autopilot_inputs_manual.heading_ref =
        real_signals.at(m_refValues.autopilotInputsManual.heading_ref);
    m_activeDPInputs.autopilot_inputs_manual.yaw_rate_ref =
        real_signals.at(m_refValues.autopilotInputsManual.yaw_rate_ref);
    m_activeDPInputs.autopilot_inputs_manual.yaw_accel_ref =
        real_signals.at(m_refValues.autopilotInputsManual.yaw_accel_ref);
    m_activeDPInputs.autopilot_inputs_manual.surge_speed_ref =
        real_signals.at(m_refValues.autopilotInputsManual.surge_speed_ref);
    m_activeDPInputs.autopilot_inputs_manual.surge_accel_ref =
        real_signals.at(m_refValues.autopilotInputsManual.surge_accel_ref);

    m_activeDPInputs.autopilot_inputs_autonomy.heading_ref =
        real_signals.at(m_refValues.autopilotInputsAutonomy.heading_ref);
    m_activeDPInputs.autopilot_inputs_autonomy.yaw_rate_ref =
        real_signals.at(m_refValues.autopilotInputsAutonomy.yaw_rate_ref);
    m_activeDPInputs.autopilot_inputs_autonomy.yaw_accel_ref =
        real_signals.at(m_refValues.autopilotInputsAutonomy.yaw_accel_ref);
    m_activeDPInputs.autopilot_inputs_autonomy.surge_speed_ref =
        real_signals.at(m_refValues.autopilotInputsAutonomy.surge_speed_ref);
    m_activeDPInputs.autopilot_inputs_autonomy.surge_accel_ref =
        real_signals.at(m_refValues.autopilotInputsAutonomy.surge_accel_ref);

    using type = DPMock::IntegrationMode;
    m_activeDPInputs.autopilot_inputs_autonomy.surge_integrator_mode =
        cast(real_signals.at(m_refValues.autopilotInputsAutonomy.surge_integrator_mode));
    m_activeDPInputs.autopilot_inputs_autonomy.heading_integrator_mode =
        cast(real_signals.at(m_refValues.autopilotInputsAutonomy.heading_integrator_mode));
    m_activeDPInputs.autopilot_inputs_manual.surge_integrator_mode =
        cast(real_signals.at(m_refValues.autopilotInputsManual.surge_integrator_mode));
    m_activeDPInputs.autopilot_inputs_manual.heading_integrator_mode =
        cast(real_signals.at(m_refValues.autopilotInputsManual.heading_integrator_mode));
}

void FmuDpMock::updateActiveParameters()
{
    m_activeDPParameters.Kp = extract3dArray(m_refValues.parameters.Kp);
    m_activeDPParameters.Ki = extract3dArray(m_refValues.parameters.Ki);
    m_activeDPParameters.Kd = extract3dArray(m_refValues.parameters.Kd);
    m_activeDPParameters.generalized_forces_saturation =
        extract3dArray(m_refValues.parameters.GeneralizedForcesSaturation);
    m_activeDPParameters.integral_action_saturation = extract3dArray(m_refValues.parameters.IntegralActionSaturation);

    Eigen::Vector3d diagM = extract3dArray(m_refValues.parameters.MassMatrixDiagonal);
    Eigen::Vector3d diagD = extract3dArray(m_refValues.parameters.DampingMatrixDiagonal);
    m_activeDPParameters.mass_matrix = diagM.asDiagonal();
    m_activeDPParameters.damping_matrix = diagD.asDiagonal();
    m_activeDPParameters.max_effort = extract3dArray(m_refValues.parameters.MaxEffort);
    m_activeDPParameters.autopilot.pid_heading.Kp = real_signals.at(m_refValues.parameters.autopilot.heading_pid.Kp);
    m_activeDPParameters.autopilot.pid_heading.Ki = real_signals.at(m_refValues.parameters.autopilot.heading_pid.Ki);
    m_activeDPParameters.autopilot.pid_heading.Kd = real_signals.at(m_refValues.parameters.autopilot.heading_pid.Kd);

    m_activeDPParameters.autopilot.pid_surge.Kp = real_signals.at(m_refValues.parameters.autopilot.surge_speed_pid.Kp);
    m_activeDPParameters.autopilot.pid_surge.Ki = real_signals.at(m_refValues.parameters.autopilot.surge_speed_pid.Ki);
    m_activeDPParameters.autopilot.pid_surge.Kd = real_signals.at(m_refValues.parameters.autopilot.surge_speed_pid.Kd);
}

void FmuDpMock::updateOutputs()
{
    m_activeDPOutputs = m_ptrDPController->get_outputs();
    bool_signals[m_refValues.outputs.readyForAutonomyEnabled] = m_activeDPOutputs.ready_for_autonomy_enabled;
    bool_signals[m_refValues.outputs.autonomyModeEnabled] = m_activeDPOutputs.autonomy_mode_enabled;
    string_signals[m_refValues.outputs.controlMode] = parseControlMode(m_activeDPOutputs.control_mode);

    real_signals[m_refValues.outputs.generalizedForces.x] = m_activeDPOutputs.generalized_forces(0);
    real_signals[m_refValues.outputs.generalizedForces.y] = m_activeDPOutputs.generalized_forces(1);
    real_signals[m_refValues.outputs.generalizedForces.z] = m_activeDPOutputs.generalized_forces(2);

    real_signals[m_refValues.outputs.refDPActive.northEastHeading.x] =
        m_activeDPOutputs.dp_ref_active.north_east_heading.x();
    real_signals[m_refValues.outputs.refDPActive.northEastHeading.y] =
        m_activeDPOutputs.dp_ref_active.north_east_heading.y();
    real_signals[m_refValues.outputs.refDPActive.northEastHeading.z] =
        m_activeDPOutputs.dp_ref_active.north_east_heading.z();

    real_signals[m_refValues.outputs.refDPActive.northEastHeadingVel.x] =
        m_activeDPOutputs.dp_ref_active.north_east_heading_vel.x();
    real_signals[m_refValues.outputs.refDPActive.northEastHeadingVel.y] =
        m_activeDPOutputs.dp_ref_active.north_east_heading_vel.y();
    real_signals[m_refValues.outputs.refDPActive.northEastHeadingVel.z] =
        m_activeDPOutputs.dp_ref_active.north_east_heading_vel.z();

    real_signals[m_refValues.outputs.refDPActive.northEastHeadingAccel.x] =
        m_activeDPOutputs.dp_ref_active.north_east_heading_accel.x();
    real_signals[m_refValues.outputs.refDPActive.northEastHeadingAccel.y] =
        m_activeDPOutputs.dp_ref_active.north_east_heading_accel.y();
    real_signals[m_refValues.outputs.refDPActive.northEastHeadingAccel.z] =
        m_activeDPOutputs.dp_ref_active.north_east_heading_accel.z();
    real_signals[m_refValues.outputs.refDPActive.heading_integrator_mode] =
        zeabuz::common::utilities::enums::get_underlying_value(m_activeDPOutputs.dp_ref_active.surge_integrator_mode);
    real_signals[m_refValues.outputs.refDPActive.surge_integrator_mode] =
        zeabuz::common::utilities::enums::get_underlying_value(m_activeDPOutputs.dp_ref_active.sway_integrator_mode);
    real_signals[m_refValues.outputs.refDPActive.heading_integrator_mode] =
        zeabuz::common::utilities::enums::get_underlying_value(m_activeDPOutputs.dp_ref_active.heading_integrator_mode);

    real_signals[m_refValues.outputs.autopilot_active_inputs.heading_ref] =
        m_activeDPOutputs.active_autopilot_inputs.heading_ref;
    real_signals[m_refValues.outputs.autopilot_active_inputs.yaw_rate_ref] =
        m_activeDPOutputs.active_autopilot_inputs.yaw_rate_ref;
    real_signals[m_refValues.outputs.autopilot_active_inputs.yaw_accel_ref] =
        m_activeDPOutputs.active_autopilot_inputs.yaw_accel_ref;
    real_signals[m_refValues.outputs.autopilot_active_inputs.surge_speed_ref] =
        m_activeDPOutputs.active_autopilot_inputs.surge_speed_ref;
    real_signals[m_refValues.outputs.autopilot_active_inputs.surge_accel_ref] =
        m_activeDPOutputs.active_autopilot_inputs.surge_accel_ref;
    real_signals[m_refValues.outputs.autopilot_active_inputs.surge_integrator_mode] =
        zeabuz::common::utilities::enums::get_underlying_value(
            m_activeDPOutputs.active_autopilot_inputs.surge_integrator_mode);
    real_signals[m_refValues.outputs.autopilot_active_inputs.heading_integrator_mode] =
        zeabuz::common::utilities::enums::get_underlying_value(
            m_activeDPOutputs.active_autopilot_inputs.heading_integrator_mode);
}

Eigen::Vector3d FmuDpMock::extract3dArray(const Vector3ValueRef& references)
{
    auto x = real_signals.at(references.x);
    auto y = real_signals.at(references.y);
    auto z = real_signals.at(references.z);
    return {x, y, z};
}

DPMock::ControlMode FmuDpMock::parseControlMode(const std::string& controlMode)
{
    if (m_validControlModes.find(controlMode) == m_validControlModes.end()) {
        std::ostringstream msg_stream;
        msg_stream << "Invalid controller mode. Available options: {STANDBY; JOYSTICK; DP; AUTOPILOT}. Got ";
        msg_stream << controlMode;
        throw std::invalid_argument(msg_stream.str());
    }
    else {
        return m_validControlModes.at(controlMode);
    }
}

std::string FmuDpMock::parseControlMode(const DPMock::ControlMode& controlMode)
{
    auto it = std::find_if(
        m_validControlModes.begin(), m_validControlModes.end(),
        [&controlMode](const std::pair<std::string, DPMock::ControlMode>& p) { return p.second == controlMode; });
    if (it == m_validControlModes.end()) {
        throw std::invalid_argument("Invalid controller mode. Available options: {STANDBY; JOYSTICK; DP}");
    }
    else {
        return it->first;
    }
}

DPMock::DPInputs FmuDpMock::getActiveDPInputs() const
{
    return m_activeDPInputs;
}

DPMock::DPParameters FmuDpMock::getActiveDPParameters() const
{
    return m_activeDPParameters;
}

DPMock::DPOutputs FmuDpMock::getActiveDPOutputs() const
{
    return m_activeDPOutputs;
}

size_t FmuDpMock::getNumberStringValues()
{
    return string_signals.size();
}
size_t FmuDpMock::getNumberBooleanValues()
{
    return bool_signals.size();
}

size_t FmuDpMock::getNumberRealValues()
{
    return real_signals.size();
}

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString instanceName, cppfmu::FMIString /*fmuGUID*/, cppfmu::FMIString /*fmuResourceLocation*/,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger /*logger*/)
{
    // if (strcmp(fmuGUID, FMU_UUID) != 0) {
    //    throw std::runtime_error("FMU GUID mismatch");
    // }
    return cppfmu::AllocateUnique<FmuDpMock>(memory, instanceName);
}
