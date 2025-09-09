#include <fmi2Functions.h>
#include <gtest/gtest.h>

#include <zeabuz/common/utilities/enums.hpp>

#include "cppfmu_common.hpp"
#include "fmi_dp_mock/data_types.hpp"
#include "fmi_dp_mock/fmi_dp_mock.hpp"

using zeabuz::common::utilities::enums::enum_cast;

class FMUDpMockUnitTests : public ::testing::Test
{
   public:
    FMUDpMockUnitTests()
    {
    }
    void SetUp() override
    {
    }
};

TEST_F(FMUDpMockUnitTests, T01_InstanciateFMUObject_NoErrors)
{
    const std::string name = "DP";
    auto fmu = std::make_unique<FmuDpMock>(name);
    fmu->Reset();
    fmu->EnterInitializationMode();
    fmu->ExitInitializationMode();
}

TEST_F(FMUDpMockUnitTests, T02_VerifyValueRefMap_AllValuesAreUnique)
{
    auto fmu = std::make_unique<FmuDpMock>("DPMocker");
    auto refValueInputMap = fmu->getValueReferences();

    std::vector<cppfmu::FMIValueReference> refValueList;
    refValueList.push_back(refValueInputMap.enableReadyForAutonomy);
    refValueList.push_back(refValueInputMap.enableAutonomyMode);
    refValueList.push_back(refValueInputMap.desiredControlMode);

    refValueList.push_back(refValueInputMap.measurements.northEastHeading.x);
    refValueList.push_back(refValueInputMap.measurements.northEastHeading.y);
    refValueList.push_back(refValueInputMap.measurements.northEastHeading.z);

    refValueList.push_back(refValueInputMap.measurements.nu.x);
    refValueList.push_back(refValueInputMap.measurements.nu.y);
    refValueList.push_back(refValueInputMap.measurements.nu.z);

    refValueList.push_back(refValueInputMap.refDpManual.northEastHeading.x);
    refValueList.push_back(refValueInputMap.refDpManual.northEastHeading.y);
    refValueList.push_back(refValueInputMap.refDpManual.northEastHeading.z);

    refValueList.push_back(refValueInputMap.refDpManual.northEastHeadingVel.x);
    refValueList.push_back(refValueInputMap.refDpManual.northEastHeadingVel.y);
    refValueList.push_back(refValueInputMap.refDpManual.northEastHeadingVel.z);

    refValueList.push_back(refValueInputMap.refDpManual.northEastHeadingAccel.x);
    refValueList.push_back(refValueInputMap.refDpManual.northEastHeadingAccel.y);
    refValueList.push_back(refValueInputMap.refDpManual.northEastHeadingAccel.z);

    refValueList.push_back(refValueInputMap.refDpManual.surge_integrator_mode);
    refValueList.push_back(refValueInputMap.refDpManual.sway_integrator_mode);
    refValueList.push_back(refValueInputMap.refDpManual.heading_integrator_mode);

    refValueList.push_back(refValueInputMap.refDpAutonomy.northEastHeading.x);
    refValueList.push_back(refValueInputMap.refDpAutonomy.northEastHeading.y);
    refValueList.push_back(refValueInputMap.refDpAutonomy.northEastHeading.z);

    refValueList.push_back(refValueInputMap.refDpAutonomy.northEastHeadingVel.x);
    refValueList.push_back(refValueInputMap.refDpAutonomy.northEastHeadingVel.y);
    refValueList.push_back(refValueInputMap.refDpAutonomy.northEastHeadingVel.z);

    refValueList.push_back(refValueInputMap.refDpAutonomy.northEastHeadingAccel.x);
    refValueList.push_back(refValueInputMap.refDpAutonomy.northEastHeadingAccel.y);
    refValueList.push_back(refValueInputMap.refDpAutonomy.northEastHeadingAccel.z);

    refValueList.push_back(refValueInputMap.refDpAutonomy.surge_integrator_mode);
    refValueList.push_back(refValueInputMap.refDpAutonomy.sway_integrator_mode);
    refValueList.push_back(refValueInputMap.refDpAutonomy.heading_integrator_mode);

    refValueList.push_back(refValueInputMap.refForcesManual.x);
    refValueList.push_back(refValueInputMap.refForcesManual.y);
    refValueList.push_back(refValueInputMap.refForcesManual.z);

    refValueList.push_back(refValueInputMap.refForcesAutonomy.x);
    refValueList.push_back(refValueInputMap.refForcesAutonomy.y);
    refValueList.push_back(refValueInputMap.refForcesAutonomy.z);

    refValueList.push_back(refValueInputMap.parameters.Kp.x);
    refValueList.push_back(refValueInputMap.parameters.Kp.y);
    refValueList.push_back(refValueInputMap.parameters.Kp.z);

    refValueList.push_back(refValueInputMap.parameters.Ki.x);
    refValueList.push_back(refValueInputMap.parameters.Ki.y);
    refValueList.push_back(refValueInputMap.parameters.Ki.z);

    refValueList.push_back(refValueInputMap.parameters.Kd.x);
    refValueList.push_back(refValueInputMap.parameters.Kd.y);
    refValueList.push_back(refValueInputMap.parameters.Kd.z);

    refValueList.push_back(refValueInputMap.parameters.GeneralizedForcesSaturation.x);
    refValueList.push_back(refValueInputMap.parameters.GeneralizedForcesSaturation.y);
    refValueList.push_back(refValueInputMap.parameters.GeneralizedForcesSaturation.z);

    refValueList.push_back(refValueInputMap.parameters.IntegralActionSaturation.x);
    refValueList.push_back(refValueInputMap.parameters.IntegralActionSaturation.y);
    refValueList.push_back(refValueInputMap.parameters.IntegralActionSaturation.z);

    refValueList.push_back(refValueInputMap.parameters.MassMatrixDiagonal.x);
    refValueList.push_back(refValueInputMap.parameters.MassMatrixDiagonal.y);
    refValueList.push_back(refValueInputMap.parameters.MassMatrixDiagonal.z);

    refValueList.push_back(refValueInputMap.parameters.DampingMatrixDiagonal.x);
    refValueList.push_back(refValueInputMap.parameters.DampingMatrixDiagonal.y);
    refValueList.push_back(refValueInputMap.parameters.DampingMatrixDiagonal.z);

    refValueList.push_back(refValueInputMap.parameters.MaxEffort.x);
    refValueList.push_back(refValueInputMap.parameters.MaxEffort.y);
    refValueList.push_back(refValueInputMap.parameters.MaxEffort.z);

    refValueList.push_back(refValueInputMap.outputs.readyForAutonomyEnabled);
    refValueList.push_back(refValueInputMap.outputs.autonomyModeEnabled);
    refValueList.push_back(refValueInputMap.outputs.controlMode);

    refValueList.push_back(refValueInputMap.outputs.generalizedForces.x);
    refValueList.push_back(refValueInputMap.outputs.generalizedForces.y);
    refValueList.push_back(refValueInputMap.outputs.generalizedForces.z);

    refValueList.push_back(refValueInputMap.outputs.refDPActive.northEastHeading.x);
    refValueList.push_back(refValueInputMap.outputs.refDPActive.northEastHeading.y);
    refValueList.push_back(refValueInputMap.outputs.refDPActive.northEastHeading.z);

    refValueList.push_back(refValueInputMap.outputs.refDPActive.northEastHeadingVel.x);
    refValueList.push_back(refValueInputMap.outputs.refDPActive.northEastHeadingVel.y);
    refValueList.push_back(refValueInputMap.outputs.refDPActive.northEastHeadingVel.z);

    refValueList.push_back(refValueInputMap.outputs.refDPActive.northEastHeadingAccel.x);
    refValueList.push_back(refValueInputMap.outputs.refDPActive.northEastHeadingAccel.y);
    refValueList.push_back(refValueInputMap.outputs.refDPActive.northEastHeadingAccel.z);

    refValueList.push_back(refValueInputMap.outputs.refDPActive.surge_integrator_mode);
    refValueList.push_back(refValueInputMap.outputs.refDPActive.sway_integrator_mode);
    refValueList.push_back(refValueInputMap.outputs.refDPActive.heading_integrator_mode);

    refValueList.push_back(refValueInputMap.autopilotInputsAutonomy.heading_ref);
    refValueList.push_back(refValueInputMap.autopilotInputsAutonomy.yaw_rate_ref);
    refValueList.push_back(refValueInputMap.autopilotInputsAutonomy.yaw_accel_ref);
    refValueList.push_back(refValueInputMap.autopilotInputsAutonomy.surge_speed_ref);
    refValueList.push_back(refValueInputMap.autopilotInputsAutonomy.surge_accel_ref);
    refValueList.push_back(refValueInputMap.autopilotInputsAutonomy.surge_integrator_mode);
    refValueList.push_back(refValueInputMap.autopilotInputsAutonomy.heading_integrator_mode);

    refValueList.push_back(refValueInputMap.autopilotInputsManual.heading_ref);
    refValueList.push_back(refValueInputMap.autopilotInputsManual.yaw_rate_ref);
    refValueList.push_back(refValueInputMap.autopilotInputsManual.yaw_accel_ref);
    refValueList.push_back(refValueInputMap.autopilotInputsManual.surge_speed_ref);
    refValueList.push_back(refValueInputMap.autopilotInputsManual.surge_accel_ref);
    refValueList.push_back(refValueInputMap.autopilotInputsManual.surge_integrator_mode);
    refValueList.push_back(refValueInputMap.autopilotInputsManual.heading_integrator_mode);

    refValueList.push_back(refValueInputMap.parameters.autopilot.heading_pid.Kp);
    refValueList.push_back(refValueInputMap.parameters.autopilot.heading_pid.Ki);
    refValueList.push_back(refValueInputMap.parameters.autopilot.heading_pid.Kd);
    refValueList.push_back(refValueInputMap.parameters.autopilot.surge_speed_pid.Kp);
    refValueList.push_back(refValueInputMap.parameters.autopilot.surge_speed_pid.Ki);
    refValueList.push_back(refValueInputMap.parameters.autopilot.surge_speed_pid.Kd);

    refValueList.push_back(refValueInputMap.outputs.autopilot_active_inputs.heading_ref);
    refValueList.push_back(refValueInputMap.outputs.autopilot_active_inputs.yaw_rate_ref);
    refValueList.push_back(refValueInputMap.outputs.autopilot_active_inputs.yaw_accel_ref);
    refValueList.push_back(refValueInputMap.outputs.autopilot_active_inputs.surge_speed_ref);
    refValueList.push_back(refValueInputMap.outputs.autopilot_active_inputs.surge_accel_ref);
    refValueList.push_back(refValueInputMap.outputs.autopilot_active_inputs.surge_integrator_mode);
    refValueList.push_back(refValueInputMap.outputs.autopilot_active_inputs.heading_integrator_mode);

    auto numElements = refValueList.size();
    auto numUniqueElements = std::set<cppfmu::FMIValueReference>(refValueList.begin(), refValueList.end()).size();
    EXPECT_EQ(numElements, numUniqueElements);

    auto expectedNumOfElements =
        sizeof(ValueReferences) / sizeof(cppfmu::FMIValueReference);  // NOTE: this might be wrong due to padding to
                                                                      // satisfy memory alighment constraints

    EXPECT_EQ(numElements, expectedNumOfElements);

    auto numBooleans = fmu->getNumberBooleanValues();
    auto numStrings = fmu->getNumberStringValues();
    auto numReals = fmu->getNumberRealValues();
    auto totalNumElements = numBooleans + numStrings + numReals;
    EXPECT_EQ(totalNumElements,
              expectedNumOfElements);  // This will fail if not all signals have been initialized in the fmu constructor
}

TEST_F(FMUDpMockUnitTests, T03_VerifyInputMapping)
{
    auto fmu = std::make_unique<FmuDpMock>("DPMocker");
    auto refValueInputMap = fmu->getValueReferences();

    cppfmu::FMIReal currentCommunicationPoint = 0;
    cppfmu::FMIReal dt = 0.05;
    cppfmu::FMIBoolean newStep = false;
    cppfmu::FMIReal endOfStep = dt;

    cppfmu::FMIValueReference vrBool[2] = {refValueInputMap.enableReadyForAutonomy,
                                           refValueInputMap.enableAutonomyMode};
    std::size_t nvrBool = 2;
    const cppfmu::FMIBoolean valueBool[2] = {true, false};
    fmu->SetBoolean(vrBool, nvrBool, valueBool);

    cppfmu::FMIValueReference vrString[1] = {refValueInputMap.desiredControlMode};
    std::size_t nvrString = 1;
    const cppfmu::FMIString valueString[1] = {"DP"};
    fmu->SetString(vrString, nvrString, valueString);

    constexpr size_t number_of_reals = 33;
    std::size_t nvrReal = number_of_reals;
    cppfmu::FMIValueReference vrReal[number_of_reals] = {refValueInputMap.refDpAutonomy.northEastHeading.x,
                                                         refValueInputMap.refDpAutonomy.northEastHeading.y,
                                                         refValueInputMap.refDpAutonomy.northEastHeading.z,

                                                         refValueInputMap.refDpAutonomy.northEastHeadingVel.x,
                                                         refValueInputMap.refDpAutonomy.northEastHeadingVel.y,
                                                         refValueInputMap.refDpAutonomy.northEastHeadingVel.z,

                                                         refValueInputMap.refDpAutonomy.northEastHeadingAccel.x,
                                                         refValueInputMap.refDpAutonomy.northEastHeadingAccel.y,
                                                         refValueInputMap.refDpAutonomy.northEastHeadingAccel.z,

                                                         refValueInputMap.refDpManual.northEastHeading.x,
                                                         refValueInputMap.refDpManual.northEastHeading.y,
                                                         refValueInputMap.refDpManual.northEastHeading.z,

                                                         refValueInputMap.refDpManual.northEastHeadingVel.x,
                                                         refValueInputMap.refDpManual.northEastHeadingVel.y,
                                                         refValueInputMap.refDpManual.northEastHeadingVel.z,

                                                         refValueInputMap.refDpManual.northEastHeadingAccel.x,
                                                         refValueInputMap.refDpManual.northEastHeadingAccel.y,
                                                         refValueInputMap.refDpManual.northEastHeadingAccel.z,

                                                         refValueInputMap.measurements.northEastHeading.x,
                                                         refValueInputMap.measurements.northEastHeading.y,
                                                         refValueInputMap.measurements.northEastHeading.z,

                                                         refValueInputMap.measurements.nu.x,
                                                         refValueInputMap.measurements.nu.y,
                                                         refValueInputMap.measurements.nu.z,

                                                         refValueInputMap.refForcesManual.x,
                                                         refValueInputMap.refForcesManual.y,
                                                         refValueInputMap.refForcesManual.z,

                                                         refValueInputMap.refForcesAutonomy.x,
                                                         refValueInputMap.refForcesAutonomy.y,
                                                         refValueInputMap.refForcesAutonomy.z,

                                                         refValueInputMap.refDpAutonomy.surge_integrator_mode,
                                                         refValueInputMap.refDpAutonomy.sway_integrator_mode,
                                                         refValueInputMap.refDpAutonomy.heading_integrator_mode};
    const cppfmu::FMIReal valueReal[number_of_reals] = {
        1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17,
        18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 1,  2,  0};  // set unique values for every signal

    fmu->SetReal(vrReal, nvrReal, valueReal);

    fmu->DoStep(currentCommunicationPoint, dt, newStep, endOfStep);
    DPMock::DPInputs activeInputs = fmu->getActiveDPInputs();

    EXPECT_TRUE(activeInputs.enable_ready_for_autonomy);
    EXPECT_FALSE(activeInputs.enable_autonomy_mode);
    EXPECT_EQ(activeInputs.desired_control_mode, DPMock::ControlMode::DP);

    EXPECT_EQ(activeInputs.dp_ref_autonomy.north_east_heading(0), valueReal[0]);
    EXPECT_EQ(activeInputs.dp_ref_autonomy.north_east_heading(1), valueReal[1]);
    EXPECT_EQ(activeInputs.dp_ref_autonomy.north_east_heading(2), valueReal[2]);

    EXPECT_EQ(activeInputs.dp_ref_autonomy.north_east_heading_vel(0), valueReal[3]);
    EXPECT_EQ(activeInputs.dp_ref_autonomy.north_east_heading_vel(1), valueReal[4]);
    EXPECT_EQ(activeInputs.dp_ref_autonomy.north_east_heading_vel(2), valueReal[5]);

    EXPECT_EQ(activeInputs.dp_ref_autonomy.north_east_heading_accel(0), valueReal[6]);
    EXPECT_EQ(activeInputs.dp_ref_autonomy.north_east_heading_accel(1), valueReal[7]);
    EXPECT_EQ(activeInputs.dp_ref_autonomy.north_east_heading_accel(2), valueReal[8]);

    using type = DPMock::IntegrationMode;
    auto cast = [&](const auto& real) { return enum_cast<type>(static_cast<int16_t>(real)); };
    EXPECT_EQ(activeInputs.dp_ref_autonomy.surge_integrator_mode, cast(valueReal[30]));
    EXPECT_EQ(activeInputs.dp_ref_autonomy.sway_integrator_mode, cast(valueReal[31]));
    EXPECT_EQ(activeInputs.dp_ref_autonomy.heading_integrator_mode, cast(valueReal[32]));

    EXPECT_EQ(activeInputs.dp_ref_manual.north_east_heading(0), valueReal[9]);
    EXPECT_EQ(activeInputs.dp_ref_manual.north_east_heading(1), valueReal[10]);
    EXPECT_EQ(activeInputs.dp_ref_manual.north_east_heading(2), valueReal[11]);

    EXPECT_EQ(activeInputs.dp_ref_manual.north_east_heading_vel(0), valueReal[12]);
    EXPECT_EQ(activeInputs.dp_ref_manual.north_east_heading_vel(1), valueReal[13]);
    EXPECT_EQ(activeInputs.dp_ref_manual.north_east_heading_vel(2), valueReal[14]);

    EXPECT_EQ(activeInputs.dp_ref_manual.north_east_heading_accel(0), valueReal[15]);
    EXPECT_EQ(activeInputs.dp_ref_manual.north_east_heading_accel(1), valueReal[16]);
    EXPECT_EQ(activeInputs.dp_ref_manual.north_east_heading_accel(2), valueReal[17]);

    EXPECT_EQ(activeInputs.measurements.north_east_heading(0), valueReal[18]);
    EXPECT_EQ(activeInputs.measurements.north_east_heading(1), valueReal[19]);
    EXPECT_EQ(activeInputs.measurements.north_east_heading(2), valueReal[20]);

    EXPECT_EQ(activeInputs.measurements.nu(0), valueReal[21]);
    EXPECT_EQ(activeInputs.measurements.nu(1), valueReal[22]);
    EXPECT_EQ(activeInputs.measurements.nu(2), valueReal[23]);

    EXPECT_EQ(activeInputs.forces_ref_manual(0), valueReal[24]);
    EXPECT_EQ(activeInputs.forces_ref_manual(1), valueReal[25]);
    EXPECT_EQ(activeInputs.forces_ref_manual(2), valueReal[26]);

    EXPECT_EQ(activeInputs.forces_ref_autonomy(0), valueReal[27]);
    EXPECT_EQ(activeInputs.forces_ref_autonomy(1), valueReal[28]);
    EXPECT_EQ(activeInputs.forces_ref_autonomy(2), valueReal[29]);

    cppfmu::FMIReal valueRealReturn[number_of_reals];
    fmu->GetReal(vrReal, nvrReal, valueRealReturn);
    for (size_t i = 0; i < nvrReal; i++) {
        EXPECT_EQ(valueRealReturn[i], valueReal[i]);
    }

    cppfmu::FMIBoolean valueBoolReturn[2];
    fmu->GetBoolean(vrBool, nvrBool, valueBoolReturn);
    for (size_t i = 0; i < nvrBool; i++) {
        EXPECT_EQ(valueBoolReturn[i], valueBool[i]);
    }

    cppfmu::FMIString valueStringReturn[1];
    fmu->GetString(vrString, nvrString, valueStringReturn);
    for (size_t i = 0; i < nvrString; i++) {
        EXPECT_EQ(std::string(valueString[i]), std::string(valueStringReturn[i]));
    }
}

TEST_F(FMUDpMockUnitTests, T04_VerifyParameterMapping)
{
    auto fmu = std::make_unique<FmuDpMock>("DPMocker");
    auto refValueInputMap = fmu->getValueReferences();

    cppfmu::FMIReal currentCommunicationPoint = 0;
    cppfmu::FMIReal dt = 0.05;
    cppfmu::FMIBoolean newStep = false;
    cppfmu::FMIReal endOfStep = dt;

    std::size_t nvrReal = 30;
    cppfmu::FMIValueReference vrReal[30] = {
        refValueInputMap.parameters.Kp.x,
        refValueInputMap.parameters.Kp.y,
        refValueInputMap.parameters.Kp.z,

        refValueInputMap.parameters.Ki.x,
        refValueInputMap.parameters.Ki.y,
        refValueInputMap.parameters.Ki.z,

        refValueInputMap.parameters.Kd.x,
        refValueInputMap.parameters.Kd.y,
        refValueInputMap.parameters.Kd.z,

        refValueInputMap.parameters.GeneralizedForcesSaturation.x,
        refValueInputMap.parameters.GeneralizedForcesSaturation.y,
        refValueInputMap.parameters.GeneralizedForcesSaturation.z,

        refValueInputMap.parameters.IntegralActionSaturation.x,
        refValueInputMap.parameters.IntegralActionSaturation.y,
        refValueInputMap.parameters.IntegralActionSaturation.z,

        refValueInputMap.parameters.DampingMatrixDiagonal.x,
        refValueInputMap.parameters.DampingMatrixDiagonal.y,
        refValueInputMap.parameters.DampingMatrixDiagonal.z,

        refValueInputMap.parameters.MassMatrixDiagonal.x,
        refValueInputMap.parameters.MassMatrixDiagonal.y,
        refValueInputMap.parameters.MassMatrixDiagonal.z,

        refValueInputMap.parameters.MaxEffort.x,
        refValueInputMap.parameters.MaxEffort.y,
        refValueInputMap.parameters.MaxEffort.z,

        refValueInputMap.parameters.autopilot.heading_pid.Kp,
        refValueInputMap.parameters.autopilot.heading_pid.Ki,
        refValueInputMap.parameters.autopilot.heading_pid.Kd,

        refValueInputMap.parameters.autopilot.surge_speed_pid.Kp,
        refValueInputMap.parameters.autopilot.surge_speed_pid.Ki,
        refValueInputMap.parameters.autopilot.surge_speed_pid.Kd,

    };

    const cppfmu::FMIReal valueReal[30] = {
        0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13,
        14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29};  // set unique values for every signal

    fmu->SetReal(vrReal, nvrReal, valueReal);
    fmu->DoStep(currentCommunicationPoint, dt, newStep, endOfStep);
    DPMock::DPParameters activeParameters = fmu->getActiveDPParameters();

    EXPECT_EQ(activeParameters.Kp(0), valueReal[0]);
    EXPECT_EQ(activeParameters.Kp(1), valueReal[1]);
    EXPECT_EQ(activeParameters.Kp(2), valueReal[2]);

    EXPECT_EQ(activeParameters.Ki(0), valueReal[3]);
    EXPECT_EQ(activeParameters.Ki(1), valueReal[4]);
    EXPECT_EQ(activeParameters.Ki(2), valueReal[5]);

    EXPECT_EQ(activeParameters.Kd(0), valueReal[6]);
    EXPECT_EQ(activeParameters.Kd(1), valueReal[7]);
    EXPECT_EQ(activeParameters.Kd(2), valueReal[8]);

    EXPECT_EQ(activeParameters.generalized_forces_saturation(0), valueReal[9]);
    EXPECT_EQ(activeParameters.generalized_forces_saturation(1), valueReal[10]);
    EXPECT_EQ(activeParameters.generalized_forces_saturation(2), valueReal[11]);

    EXPECT_EQ(activeParameters.integral_action_saturation(0), valueReal[12]);
    EXPECT_EQ(activeParameters.integral_action_saturation(1), valueReal[13]);
    EXPECT_EQ(activeParameters.integral_action_saturation(2), valueReal[14]);

    EXPECT_EQ(activeParameters.damping_matrix(0, 0), valueReal[15]);
    EXPECT_EQ(activeParameters.damping_matrix(1, 1), valueReal[16]);
    EXPECT_EQ(activeParameters.damping_matrix(2, 2), valueReal[17]);

    EXPECT_EQ(activeParameters.mass_matrix(0, 0), valueReal[18]);
    EXPECT_EQ(activeParameters.mass_matrix(1, 1), valueReal[19]);
    EXPECT_EQ(activeParameters.mass_matrix(2, 2), valueReal[20]);

    EXPECT_EQ(activeParameters.max_effort(0), valueReal[21]);
    EXPECT_EQ(activeParameters.max_effort(1), valueReal[22]);
    EXPECT_EQ(activeParameters.max_effort(2), valueReal[23]);


    EXPECT_EQ(activeParameters.autopilot.pid_heading.Kp, valueReal[24]);
    EXPECT_EQ(activeParameters.autopilot.pid_heading.Ki, valueReal[25]);
    EXPECT_EQ(activeParameters.autopilot.pid_heading.Kd, valueReal[26]);

    EXPECT_EQ(activeParameters.autopilot.pid_surge.Kp, valueReal[27]);
    EXPECT_EQ(activeParameters.autopilot.pid_surge.Ki, valueReal[28]);
    EXPECT_EQ(activeParameters.autopilot.pid_surge.Kd, valueReal[29]);

    cppfmu::FMIReal valueRealReturn[30];
    fmu->GetReal(vrReal, nvrReal, valueRealReturn);

    for (size_t i = 0; i < nvrReal; i++) {
        EXPECT_EQ(valueRealReturn[i], valueReal[i]);
    }
}

TEST_F(FMUDpMockUnitTests, T05_VerifyOutputs)
{
    auto fmu = std::make_unique<FmuDpMock>("DPMocker");
    auto refValueInputMap = fmu->getValueReferences();

    cppfmu::FMIReal currentCommunicationPoint = 0;
    cppfmu::FMIReal dt = 0.05;
    cppfmu::FMIBoolean newStep = false;
    cppfmu::FMIReal endOfStep = dt;

    cppfmu::FMIValueReference vrBool[2] = {refValueInputMap.enableReadyForAutonomy,
                                           refValueInputMap.enableAutonomyMode};
    std::size_t nvrBool = 2;
    const cppfmu::FMIBoolean valueBool[2] = {true, false};
    fmu->SetBoolean(vrBool, nvrBool, valueBool);

    cppfmu::FMIValueReference vrString[1] = {refValueInputMap.desiredControlMode};
    std::size_t nvrString = 1;
    const cppfmu::FMIString valueString[1] = {"DP"};
    fmu->SetString(vrString, nvrString, valueString);

    fmu->DoStep(currentCommunicationPoint, dt, newStep, endOfStep);
    DPMock::DPOutputs outputs = fmu->getActiveDPOutputs();

    EXPECT_TRUE(outputs.ready_for_autonomy_enabled);
    EXPECT_FALSE(outputs.autonomy_mode_enabled);
    EXPECT_EQ(outputs.control_mode, DPMock::ControlMode::DP);

    cppfmu::FMIValueReference vrBoolOut[2] = {refValueInputMap.outputs.readyForAutonomyEnabled,
                                              refValueInputMap.outputs.autonomyModeEnabled};
    std::size_t nvrBoolOut = 2;
    cppfmu::FMIBoolean valueBoolOut[2];
    fmu->GetBoolean(vrBoolOut, nvrBoolOut, valueBoolOut);

    cppfmu::FMIValueReference vrStringOut[1] = {refValueInputMap.outputs.controlMode};
    std::size_t nvrStringOut = 1;
    cppfmu::FMIString valueStringOut[1];
    fmu->GetString(vrStringOut, nvrStringOut, valueStringOut);

    std::size_t nvrReal = 3;
    cppfmu::FMIValueReference vrReal[3] = {
        refValueInputMap.outputs.generalizedForces.x,
        refValueInputMap.outputs.generalizedForces.y,
        refValueInputMap.outputs.generalizedForces.z,
    };
    cppfmu::FMIReal valueRealOut[3];
    fmu->GetReal(vrReal, nvrReal, valueRealOut);

    EXPECT_EQ(valueBoolOut[0], outputs.ready_for_autonomy_enabled);
    EXPECT_EQ(valueBoolOut[1], outputs.autonomy_mode_enabled);
    EXPECT_EQ(valueRealOut[0], outputs.generalized_forces(0));
    EXPECT_EQ(valueRealOut[1], outputs.generalized_forces(1));
    EXPECT_EQ(valueRealOut[2], outputs.generalized_forces(2));
}

TEST_F(FMUDpMockUnitTests, T06_VerifyGeneralizedForceOutput)
{
    auto fmu = std::make_unique<FmuDpMock>("DPMocker");
    auto refValueInputMap = fmu->getValueReferences();

    cppfmu::FMIReal currentCommunicationPoint = 0;
    cppfmu::FMIReal dt = 0.05;
    cppfmu::FMIBoolean newStep = false;
    cppfmu::FMIReal endOfStep = dt;

    cppfmu::FMIValueReference vrBool[2] = {refValueInputMap.enableReadyForAutonomy,
                                           refValueInputMap.enableAutonomyMode};
    std::size_t nvrBool = 2;
    const cppfmu::FMIBoolean valueBool[2] = {false, false};
    fmu->SetBoolean(vrBool, nvrBool, valueBool);

    cppfmu::FMIValueReference vrString[1] = {refValueInputMap.desiredControlMode};
    std::size_t nvrString = 1;
    const cppfmu::FMIString valueString[1] = {"JOYSTICK"};
    fmu->SetString(vrString, nvrString, valueString);

    std::size_t nvrReal = 6;
    cppfmu::FMIValueReference vrReal[6] = {refValueInputMap.refForcesManual.x,
                                           refValueInputMap.refForcesManual.y,
                                           refValueInputMap.refForcesManual.z,
                                           refValueInputMap.parameters.MaxEffort.x,
                                           refValueInputMap.parameters.MaxEffort.y,
                                           refValueInputMap.parameters.MaxEffort.z,};
    const cppfmu::FMIReal valueReal[6] = {10, 20, 30, 1000, 1000, 1000};
    fmu->SetReal(vrReal, nvrReal, valueReal);

    fmu->DoStep(currentCommunicationPoint, dt, newStep, endOfStep);
    DPMock::DPOutputs outputs = fmu->getActiveDPOutputs();

    EXPECT_EQ(outputs.generalized_forces(0), valueReal[0] * 10);
    EXPECT_EQ(outputs.generalized_forces(1), valueReal[1] * 10);
    EXPECT_EQ(outputs.generalized_forces(2), valueReal[2] * 10);

    cppfmu::FMIValueReference vrRealOut[3] = {
        refValueInputMap.outputs.generalizedForces.x,
        refValueInputMap.outputs.generalizedForces.y,
        refValueInputMap.outputs.generalizedForces.z,
    };
    cppfmu::FMIReal valueRealOut[3];
    fmu->GetReal(vrRealOut, nvrReal, valueRealOut);

    EXPECT_EQ(valueRealOut[0], outputs.generalized_forces(0));
    EXPECT_EQ(valueRealOut[1], outputs.generalized_forces(1));
    EXPECT_EQ(valueRealOut[2], outputs.generalized_forces(2));
}

TEST_F(FMUDpMockUnitTests, T07_VerifyGeneralizedForceOutput_StandbyMode)
{
    auto fmu = std::make_unique<FmuDpMock>("DPMocker");
    auto refValueInputMap = fmu->getValueReferences();

    cppfmu::FMIReal currentCommunicationPoint = 0;
    cppfmu::FMIReal dt = 0.05;
    cppfmu::FMIBoolean newStep = false;
    cppfmu::FMIReal endOfStep = dt;

    cppfmu::FMIValueReference vrString[1] = {refValueInputMap.desiredControlMode};
    std::size_t nvrString = 1;
    const cppfmu::FMIString valueString[1] = {"STANDBY"};
    fmu->SetString(vrString, nvrString, valueString);

    fmu->DoStep(currentCommunicationPoint, dt, newStep, endOfStep);
    DPMock::DPOutputs outputs = fmu->getActiveDPOutputs();

    EXPECT_EQ(outputs.control_mode, DPMock::ControlMode::STANDBY);
    EXPECT_EQ(outputs.generalized_forces(0), 0.0);
    EXPECT_EQ(outputs.generalized_forces(1), 0.0);
    EXPECT_EQ(outputs.generalized_forces(2), 0.0);
}

TEST_F(FMUDpMockUnitTests, T08_VerifyActiveDPOutputsMapping)
{
    auto fmu = std::make_unique<FmuDpMock>("DPMocker");
    auto refValueInputMap = fmu->getValueReferences();

    cppfmu::FMIReal currentCommunicationPoint = 0;
    cppfmu::FMIReal dt = 0.05;
    cppfmu::FMIBoolean newStep = false;
    cppfmu::FMIReal endOfStep = dt;

    cppfmu::FMIValueReference vrBool[2] = {refValueInputMap.enableReadyForAutonomy,
                                           refValueInputMap.enableAutonomyMode};
    std::size_t nvrBool = 2;
    const cppfmu::FMIBoolean valueBool[2] = {true, true};
    fmu->SetBoolean(vrBool, nvrBool, valueBool);

    cppfmu::FMIValueReference vrString[1] = {refValueInputMap.desiredControlMode};
    std::size_t nvrString = 1;
    const cppfmu::FMIString valueString[1] = {"DP"};
    fmu->SetString(vrString, nvrString, valueString);

    fmu->DoStep(currentCommunicationPoint, dt, newStep, endOfStep);
    DPMock::DPOutputs outputs = fmu->getActiveDPOutputs();

    std::size_t nvrReal = 9;
    cppfmu::FMIValueReference vrReal[9] = {
        refValueInputMap.refDpAutonomy.northEastHeading.x,      refValueInputMap.refDpAutonomy.northEastHeading.y,
        refValueInputMap.refDpAutonomy.northEastHeading.z,

        refValueInputMap.refDpAutonomy.northEastHeadingVel.x,   refValueInputMap.refDpAutonomy.northEastHeadingVel.y,
        refValueInputMap.refDpAutonomy.northEastHeadingVel.z,

        refValueInputMap.refDpAutonomy.northEastHeadingAccel.x, refValueInputMap.refDpAutonomy.northEastHeadingAccel.y,
        refValueInputMap.refDpAutonomy.northEastHeadingAccel.z,
    };
    const cppfmu::FMIReal valueReal[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};  // set unique values for every signal

    fmu->SetReal(vrReal, nvrReal, valueReal);
    //
    fmu->DoStep(currentCommunicationPoint, dt, newStep, endOfStep);

    std::size_t nvrRealout = 9;
    cppfmu::FMIValueReference vrRealOut[9] = {
        refValueInputMap.outputs.refDPActive.northEastHeading.x,
        refValueInputMap.outputs.refDPActive.northEastHeading.y,
        refValueInputMap.outputs.refDPActive.northEastHeading.z,

        refValueInputMap.outputs.refDPActive.northEastHeadingVel.x,
        refValueInputMap.outputs.refDPActive.northEastHeadingVel.y,
        refValueInputMap.outputs.refDPActive.northEastHeadingVel.z,

        refValueInputMap.outputs.refDPActive.northEastHeadingAccel.x,
        refValueInputMap.outputs.refDPActive.northEastHeadingAccel.y,
        refValueInputMap.outputs.refDPActive.northEastHeadingAccel.z,
    };
    cppfmu::FMIReal valueRealOut[9];
    fmu->GetReal(vrRealOut, nvrRealout, valueRealOut);

    outputs = fmu->getActiveDPOutputs();
    // auto test = outputs.dp_ref_active.north_east_heading_accel.x();
    EXPECT_EQ(valueRealOut[0], outputs.dp_ref_active.north_east_heading.x());
    EXPECT_EQ(valueRealOut[1], outputs.dp_ref_active.north_east_heading.y());
    EXPECT_EQ(valueRealOut[2], outputs.dp_ref_active.north_east_heading.z());

    EXPECT_EQ(valueRealOut[3], outputs.dp_ref_active.north_east_heading_vel.x());
    EXPECT_EQ(valueRealOut[4], outputs.dp_ref_active.north_east_heading_vel.y());
    EXPECT_EQ(valueRealOut[5], outputs.dp_ref_active.north_east_heading_vel.z());

    EXPECT_EQ(valueRealOut[6], outputs.dp_ref_active.north_east_heading_accel.x());
    EXPECT_EQ(valueRealOut[7], outputs.dp_ref_active.north_east_heading_accel.y());
    EXPECT_EQ(valueRealOut[8], outputs.dp_ref_active.north_east_heading_accel.z());
}

TEST_F(FMUDpMockUnitTests, T09_VerifyInputInitialization_NoExceptionThrown)
{
    auto fmu = std::make_unique<FmuDpMock>("DPMocker");
    auto refValueInputMap = fmu->getValueReferences();

    // cppfmu::FMIReal currentCommunicationPoint = 0;
    // cppfmu::FMIReal dt = 0.05;
    // cppfmu::FMIBoolean newStep = false;
    // cppfmu::FMIReal endOfStep = dt;

    cppfmu::FMIValueReference vrBool[2] = {refValueInputMap.enableReadyForAutonomy,
                                           refValueInputMap.enableAutonomyMode};
    std::size_t nvrBool = 2;
    // const cppfmu::FMIBoolean valueBool[2] = {true, false};
    cppfmu::FMIValueReference vrString[1] = {refValueInputMap.desiredControlMode};
    std::size_t nvrString = 1;

    std::size_t nvrReal = 30;
    cppfmu::FMIValueReference vrReal[30] = {refValueInputMap.refDpAutonomy.northEastHeading.x,
                                            refValueInputMap.refDpAutonomy.northEastHeading.y,
                                            refValueInputMap.refDpAutonomy.northEastHeading.z,

                                            refValueInputMap.refDpAutonomy.northEastHeadingVel.x,
                                            refValueInputMap.refDpAutonomy.northEastHeadingVel.y,
                                            refValueInputMap.refDpAutonomy.northEastHeadingVel.z,

                                            refValueInputMap.refDpAutonomy.northEastHeadingAccel.x,
                                            refValueInputMap.refDpAutonomy.northEastHeadingAccel.y,
                                            refValueInputMap.refDpAutonomy.northEastHeadingAccel.z,

                                            refValueInputMap.refDpManual.northEastHeading.x,
                                            refValueInputMap.refDpManual.northEastHeading.y,
                                            refValueInputMap.refDpManual.northEastHeading.z,

                                            refValueInputMap.refDpManual.northEastHeadingVel.x,
                                            refValueInputMap.refDpManual.northEastHeadingVel.y,
                                            refValueInputMap.refDpManual.northEastHeadingVel.z,

                                            refValueInputMap.refDpManual.northEastHeadingAccel.x,
                                            refValueInputMap.refDpManual.northEastHeadingAccel.y,
                                            refValueInputMap.refDpManual.northEastHeadingAccel.z,

                                            refValueInputMap.measurements.northEastHeading.x,
                                            refValueInputMap.measurements.northEastHeading.y,
                                            refValueInputMap.measurements.northEastHeading.z,

                                            refValueInputMap.measurements.nu.x,
                                            refValueInputMap.measurements.nu.y,
                                            refValueInputMap.measurements.nu.z,

                                            refValueInputMap.refForcesManual.x,
                                            refValueInputMap.refForcesManual.y,
                                            refValueInputMap.refForcesManual.z,

                                            refValueInputMap.refForcesAutonomy.x,
                                            refValueInputMap.refForcesAutonomy.y,
                                            refValueInputMap.refForcesAutonomy.z};
    // const cppfmu::FMIReal valueReal[30] = {
    //   1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
    //    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30};  // set unique values for every signal

    // Try to read values before setting our stepping.  An exception will be thrown if signal map has not been
    // initialized in the fmu constructor
    cppfmu::FMIReal valueRealReturn[30];
    fmu->GetReal(vrReal, nvrReal, valueRealReturn);

    cppfmu::FMIBoolean valueBoolReturn[2];
    fmu->GetBoolean(vrBool, nvrBool, valueBoolReturn);

    cppfmu::FMIString valueStringReturn[1];
    fmu->GetString(vrString, nvrString, valueStringReturn);
}

TEST_F(FMUDpMockUnitTests, T10_VerifyParameterInitialization_ExpectNoException)
{
    auto fmu = std::make_unique<FmuDpMock>("DPMocker");
    auto refValueInputMap = fmu->getValueReferences();

    // cppfmu::FMIReal currentCommunicationPoint = 0;
    // cppfmu::FMIReal dt = 0.05;
    // cppfmu::FMIBoolean newStep = false;
    // cppfmu::FMIReal endOfStep = dt;

    std::size_t nvrReal = 24;
    cppfmu::FMIValueReference vrReal[24] = {
        refValueInputMap.parameters.Kp.x,
        refValueInputMap.parameters.Kp.y,
        refValueInputMap.parameters.Kp.z,

        refValueInputMap.parameters.Ki.x,
        refValueInputMap.parameters.Ki.y,
        refValueInputMap.parameters.Ki.z,

        refValueInputMap.parameters.Kd.x,
        refValueInputMap.parameters.Kd.y,
        refValueInputMap.parameters.Kd.z,

        refValueInputMap.parameters.GeneralizedForcesSaturation.x,
        refValueInputMap.parameters.GeneralizedForcesSaturation.y,
        refValueInputMap.parameters.GeneralizedForcesSaturation.z,

        refValueInputMap.parameters.IntegralActionSaturation.x,
        refValueInputMap.parameters.IntegralActionSaturation.y,
        refValueInputMap.parameters.IntegralActionSaturation.z,

        refValueInputMap.parameters.DampingMatrixDiagonal.x,
        refValueInputMap.parameters.DampingMatrixDiagonal.y,
        refValueInputMap.parameters.DampingMatrixDiagonal.z,

        refValueInputMap.parameters.MassMatrixDiagonal.x,
        refValueInputMap.parameters.MassMatrixDiagonal.y,
        refValueInputMap.parameters.MassMatrixDiagonal.z,

        refValueInputMap.parameters.MaxEffort.x,
        refValueInputMap.parameters.MaxEffort.y,
        refValueInputMap.parameters.MaxEffort.z,

    };
    // Try to read values before setting our stepping.  An exception will be thrown if signal map has not been
    // initialized in the fmu constructor
    cppfmu::FMIReal valueRealReturn[24];
    fmu->GetReal(vrReal, nvrReal, valueRealReturn);
}

TEST_F(FMUDpMockUnitTests, T11_VerifyOutputInitialization_ExpectNoException)
{
    auto fmu = std::make_unique<FmuDpMock>("DPMocker");
    auto refValueInputMap = fmu->getValueReferences();

    cppfmu::FMIValueReference vrBoolOut[2] = {refValueInputMap.outputs.readyForAutonomyEnabled,
                                              refValueInputMap.outputs.autonomyModeEnabled};
    std::size_t nvrBoolOut = 2;
    cppfmu::FMIBoolean valueBoolOut[2];
    fmu->GetBoolean(vrBoolOut, nvrBoolOut, valueBoolOut);

    cppfmu::FMIValueReference vrStringOut[1] = {refValueInputMap.outputs.controlMode};
    std::size_t nvrStringOut = 1;
    cppfmu::FMIString valueStringOut[1];
    fmu->GetString(vrStringOut, nvrStringOut, valueStringOut);

    std::size_t nvrReal = 12;
    cppfmu::FMIValueReference vrReal[12] = {
        refValueInputMap.outputs.generalizedForces.x,
        refValueInputMap.outputs.generalizedForces.y,
        refValueInputMap.outputs.generalizedForces.z,
        refValueInputMap.outputs.refDPActive.northEastHeading.x,
        refValueInputMap.outputs.refDPActive.northEastHeading.y,
        refValueInputMap.outputs.refDPActive.northEastHeading.z,

        refValueInputMap.outputs.refDPActive.northEastHeadingVel.x,
        refValueInputMap.outputs.refDPActive.northEastHeadingVel.y,
        refValueInputMap.outputs.refDPActive.northEastHeadingVel.z,

        refValueInputMap.outputs.refDPActive.northEastHeadingAccel.x,
        refValueInputMap.outputs.refDPActive.northEastHeadingAccel.y,
        refValueInputMap.outputs.refDPActive.northEastHeadingAccel.z,
    };
    cppfmu::FMIReal valueRealOut[12];
    fmu->GetReal(vrReal, nvrReal, valueRealOut);
}

TEST_F(FMUDpMockUnitTests, T12_VerifyActiveAutopilotOutputsMapping_AutonomyEnabled)
{
    auto fmu = std::make_unique<FmuDpMock>("DPMocker");
    auto refValueInputMap = fmu->getValueReferences();

    cppfmu::FMIReal currentCommunicationPoint = 0;
    cppfmu::FMIReal dt = 0.05;
    cppfmu::FMIBoolean newStep = false;
    cppfmu::FMIReal endOfStep = dt;

    cppfmu::FMIValueReference vrBool[2] = {refValueInputMap.enableReadyForAutonomy,
                                           refValueInputMap.enableAutonomyMode};
    std::size_t nvrBool = 2;
    const cppfmu::FMIBoolean valueBool[2] = {true, true};
    fmu->SetBoolean(vrBool, nvrBool, valueBool);

    cppfmu::FMIValueReference vrString[1] = {refValueInputMap.desiredControlMode};
    std::size_t nvrString = 1;
    const cppfmu::FMIString valueString[1] = {"AUTOPILOT"};
    fmu->SetString(vrString, nvrString, valueString);

    fmu->DoStep(currentCommunicationPoint, dt, newStep, endOfStep);
    DPMock::DPOutputs outputs = fmu->getActiveDPOutputs();

    std::size_t nvrReal = 5;
    cppfmu::FMIValueReference vrReal[5] = {
        refValueInputMap.autopilotInputsAutonomy.heading_ref,
        refValueInputMap.autopilotInputsAutonomy.yaw_rate_ref,
        refValueInputMap.autopilotInputsAutonomy.yaw_accel_ref,
        refValueInputMap.autopilotInputsAutonomy.surge_speed_ref,
        refValueInputMap.autopilotInputsAutonomy.surge_accel_ref,

    };
    const cppfmu::FMIReal valueReal[9] = {1, 2, 3, 4, 5};  // set unique values for every signal

    fmu->SetReal(vrReal, nvrReal, valueReal);

    fmu->DoStep(currentCommunicationPoint, dt, newStep, endOfStep);

    std::size_t nvrRealout = 5;
    cppfmu::FMIValueReference vrRealOut[5] = {

        refValueInputMap.outputs.autopilot_active_inputs.heading_ref,
        refValueInputMap.outputs.autopilot_active_inputs.yaw_rate_ref,
        refValueInputMap.outputs.autopilot_active_inputs.yaw_accel_ref,
        refValueInputMap.outputs.autopilot_active_inputs.surge_speed_ref,
        refValueInputMap.outputs.autopilot_active_inputs.surge_accel_ref,

    };
    cppfmu::FMIReal valueRealOut[5];
    fmu->GetReal(vrRealOut, nvrRealout, valueRealOut);

    outputs = fmu->getActiveDPOutputs();

    EXPECT_EQ(valueRealOut[0], outputs.active_autopilot_inputs.heading_ref);
    EXPECT_EQ(valueRealOut[1], outputs.active_autopilot_inputs.yaw_rate_ref);
    EXPECT_EQ(valueRealOut[2], outputs.active_autopilot_inputs.yaw_accel_ref);
    EXPECT_EQ(valueRealOut[3], outputs.active_autopilot_inputs.surge_speed_ref);
    EXPECT_EQ(valueRealOut[4], outputs.active_autopilot_inputs.surge_accel_ref);

    EXPECT_EQ(valueReal[0], valueRealOut[0]);
    EXPECT_EQ(valueReal[1], valueRealOut[1]);
    EXPECT_EQ(valueReal[2], valueRealOut[2]);
    EXPECT_EQ(valueReal[3], valueRealOut[3]);
    EXPECT_EQ(valueReal[4], valueRealOut[4]);
}

TEST_F(FMUDpMockUnitTests, T13_VerifyActiveAutopilotOutputsMapping_AutonomyDisabled)
{
    auto fmu = std::make_unique<FmuDpMock>("DPMocker");
    auto refValueInputMap = fmu->getValueReferences();

    cppfmu::FMIReal currentCommunicationPoint = 0;
    cppfmu::FMIReal dt = 0.05;
    cppfmu::FMIBoolean newStep = false;
    cppfmu::FMIReal endOfStep = dt;

    cppfmu::FMIValueReference vrBool[2] = {refValueInputMap.enableReadyForAutonomy,
                                           refValueInputMap.enableAutonomyMode};
    std::size_t nvrBool = 2;
    const cppfmu::FMIBoolean valueBool[2] = {false, false};
    fmu->SetBoolean(vrBool, nvrBool, valueBool);

    cppfmu::FMIValueReference vrString[1] = {refValueInputMap.desiredControlMode};
    std::size_t nvrString = 1;
    const cppfmu::FMIString valueString[1] = {"AUTOPILOT"};
    fmu->SetString(vrString, nvrString, valueString);

    fmu->DoStep(currentCommunicationPoint, dt, newStep, endOfStep);
    DPMock::DPOutputs outputs = fmu->getActiveDPOutputs();

    std::size_t nvrReal = 5;
    cppfmu::FMIValueReference vrReal[5] = {
        refValueInputMap.autopilotInputsManual.heading_ref,     refValueInputMap.autopilotInputsManual.yaw_rate_ref,
        refValueInputMap.autopilotInputsManual.yaw_accel_ref,   refValueInputMap.autopilotInputsManual.surge_speed_ref,
        refValueInputMap.autopilotInputsManual.surge_accel_ref,

    };
    const cppfmu::FMIReal valueReal[9] = {1, 2, 3, 4, 5};  // set unique values for every signal

    fmu->SetReal(vrReal, nvrReal, valueReal);

    fmu->DoStep(currentCommunicationPoint, dt, newStep, endOfStep);

    std::size_t nvrRealout = 5;
    cppfmu::FMIValueReference vrRealOut[5] = {

        refValueInputMap.outputs.autopilot_active_inputs.heading_ref,
        refValueInputMap.outputs.autopilot_active_inputs.yaw_rate_ref,
        refValueInputMap.outputs.autopilot_active_inputs.yaw_accel_ref,
        refValueInputMap.outputs.autopilot_active_inputs.surge_speed_ref,
        refValueInputMap.outputs.autopilot_active_inputs.surge_accel_ref,

    };
    cppfmu::FMIReal valueRealOut[5];
    fmu->GetReal(vrRealOut, nvrRealout, valueRealOut);

    outputs = fmu->getActiveDPOutputs();

    EXPECT_EQ(valueRealOut[0], outputs.active_autopilot_inputs.heading_ref);
    EXPECT_EQ(valueRealOut[1], outputs.active_autopilot_inputs.yaw_rate_ref);
    EXPECT_EQ(valueRealOut[2], outputs.active_autopilot_inputs.yaw_accel_ref);
    EXPECT_EQ(valueRealOut[3], outputs.active_autopilot_inputs.surge_speed_ref);
    EXPECT_EQ(valueRealOut[4], outputs.active_autopilot_inputs.surge_accel_ref);

    EXPECT_EQ(valueReal[0], valueRealOut[0]);
    EXPECT_EQ(valueReal[1], valueRealOut[1]);
    EXPECT_EQ(valueReal[2], valueRealOut[2]);
    EXPECT_EQ(valueReal[3], valueRealOut[3]);
    EXPECT_EQ(valueReal[4], valueRealOut[4]);
}