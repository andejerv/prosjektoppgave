#ifndef DYNAMIC_POSITIONING_CPP_FMI_DP_MOCK_HPP
#define DYNAMIC_POSITIONING_CPP_FMI_DP_MOCK_HPP

#include <climits>
#include <cppfmu_cs.hpp>
#include <unordered_map>

#include "dynamic_positioning.hpp"

struct Vector3ValueRef
{
    Vector3ValueRef() : x(UINT_MAX), y(UINT_MAX), z(UINT_MAX){};
    cppfmu::FMIValueReference x;
    cppfmu::FMIValueReference y;
    cppfmu::FMIValueReference z;
};

struct DPReferenceValueRef
{
    Vector3ValueRef northEastHeading;
    Vector3ValueRef northEastHeadingVel;
    Vector3ValueRef northEastHeadingAccel;
    cppfmu::FMIValueReference surge_integrator_mode;
    cppfmu::FMIValueReference sway_integrator_mode;
    cppfmu::FMIValueReference heading_integrator_mode;
};

struct MeasurementsValueRef
{
    Vector3ValueRef northEastHeading;
    Vector3ValueRef nu;
};

struct PIDRef
{
    cppfmu::FMIValueReference Kp;
    cppfmu::FMIValueReference Ki;
    cppfmu::FMIValueReference Kd;
};
struct AutopilotParametersRef
{
    PIDRef heading_pid;
    PIDRef surge_speed_pid;
};

struct ParamValueRef
{
    Vector3ValueRef Kp;
    Vector3ValueRef Ki;
    Vector3ValueRef Kd;
    Vector3ValueRef GeneralizedForcesSaturation;
    Vector3ValueRef IntegralActionSaturation;
    Vector3ValueRef MassMatrixDiagonal;
    Vector3ValueRef DampingMatrixDiagonal;
    Vector3ValueRef MaxEffort;
    AutopilotParametersRef autopilot;
};

struct AutopilotInputsRef
{
    cppfmu::FMIValueReference surge_speed_ref;
    cppfmu::FMIValueReference surge_accel_ref;
    cppfmu::FMIValueReference heading_ref;
    cppfmu::FMIValueReference yaw_rate_ref;
    cppfmu::FMIValueReference yaw_accel_ref;
    cppfmu::FMIValueReference surge_integrator_mode;
    cppfmu::FMIValueReference heading_integrator_mode;
};
struct OutputValueRef
{
    Vector3ValueRef generalizedForces;
    cppfmu::FMIValueReference controlMode;
    cppfmu::FMIValueReference readyForAutonomyEnabled;
    cppfmu::FMIValueReference autonomyModeEnabled;
    DPReferenceValueRef refDPActive;
    AutopilotInputsRef autopilot_active_inputs;
};

struct ValueReferences
{
    // booleans
    cppfmu::FMIValueReference enableReadyForAutonomy;
    cppfmu::FMIValueReference enableAutonomyMode;
    // strings
    cppfmu::FMIValueReference desiredControlMode;
    // reals
    DPReferenceValueRef refDpAutonomy;
    Vector3ValueRef refForcesAutonomy;

    DPReferenceValueRef refDpManual;
    Vector3ValueRef refForcesManual;

    AutopilotInputsRef autopilotInputsManual;
    AutopilotInputsRef autopilotInputsAutonomy;

    MeasurementsValueRef measurements;

    ParamValueRef parameters;

    OutputValueRef outputs;
};

class FmuDpMock : public cppfmu::SlaveInstance
{
   public:
    FmuDpMock(const std::string& instanceName);
    FmuDpMock() = default;

    void Reset() override;

    void ExitInitializationMode() override;

    void SetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIString value[]) override;

    void SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[]) override;

    void SetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIBoolean value[]) override;

    void GetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIString value[]) const override;

    void GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const override;

    void GetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIBoolean value[]) const override;

    bool DoStep(cppfmu::FMIReal /*currentCommunicationPoint*/, cppfmu::FMIReal /*dt*/, cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal& /*endOfStep*/) override;

    ValueReferences getValueReferences();

   private:
    const std::string modelName;
    std::unique_ptr<DPMock::DPController> m_ptrDPController;
    DPMock::DPInputs m_activeDPInputs;
    DPMock::DPParameters m_activeDPParameters;
    DPMock::DPOutputs m_activeDPOutputs;

   public:
    DPMock::DPParameters getActiveDPParameters() const;
    DPMock::DPInputs getActiveDPInputs() const;
    DPMock::DPOutputs getActiveDPOutputs() const;
    size_t getNumberStringValues();
    size_t getNumberBooleanValues();
    size_t getNumberRealValues();

   private:
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIReal> real_signals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIBoolean> bool_signals;
    std::unordered_map<cppfmu::FMIValueReference, std::string> string_signals;

    std::unordered_map<std::string, DPMock::ControlMode> m_validControlModes;

    ValueReferences m_refValues;

    DPMock::ControlMode parseControlMode(const std::string& controlMode);

    Eigen::Vector3d extract3dArray(const Vector3ValueRef& references);

    void initializeValueReferences();

    void updateActiveParameters();

    void updateActiveInputs(double dt);

    void updateOutputs();

    std::string parseControlMode(const DPMock::ControlMode& controlMode);

    bool m_prev_enabled_autonomy_mode{false};
};

#endif  // DYNAMIC_POSITIONING_CPP_FMI_DP_MOCK_HPP
