#ifndef DYNAMIC_POSITIONING_CPP_DATATYPES_HPP
#define DYNAMIC_POSITIONING_CPP_DATATYPES_HPP

#include <Eigen/Dense>
#include <chrono>

namespace DPMock
{

enum class ControlMode { JOYSTICK = 0, DP = 1, STANDBY = 2, AUTOPILOT = 3 };
enum class IntegrationMode : std::int16_t { FREEZE = 0, ENABLE = 1, AUTOMATIC = 2 };
using GeneralizedForces = Eigen::Vector3d;

struct DPReference
{
    Eigen::Vector3d north_east_heading;
    Eigen::Vector3d north_east_heading_vel;
    Eigen::Vector3d north_east_heading_accel;
    IntegrationMode surge_integrator_mode{};
    IntegrationMode sway_integrator_mode{};
    IntegrationMode heading_integrator_mode{};
    DPReference() = default;

    DPReference(Eigen::Vector3d ref_pose, Eigen::Vector3d ref_vel, Eigen::Vector3d ref_accel)
    : north_east_heading(ref_pose), north_east_heading_vel(ref_vel), north_east_heading_accel(ref_accel){};

    // TODO: consider wrapYawAngle in the equal comparison
    inline bool operator==(const DPReference& rhs) const
    {
        bool equal = (north_east_heading == rhs.north_east_heading) &&
                     (north_east_heading_vel == rhs.north_east_heading_vel) &&
                     (north_east_heading_accel == rhs.north_east_heading_accel) &&
                     (surge_integrator_mode == rhs.surge_integrator_mode) &&
                     (sway_integrator_mode == rhs.sway_integrator_mode) &&
                     (heading_integrator_mode == rhs.heading_integrator_mode);
        return equal;
    }

    inline bool operator!=(const DPReference& rhs) const
    {
        return !(*this == rhs);
    }
};

struct Measurements
{
    Measurements() = default;
    Eigen::Vector3d north_east_heading;
    Eigen::Vector3d nu;
    Measurements(Eigen::Vector3d meas_pose, Eigen::Vector3d meas_vel) : north_east_heading(meas_pose), nu(meas_vel){};
};

struct AutopilotInputs
{
    double surge_speed_ref;
    double surge_accel_ref;

    double heading_ref;
    double yaw_rate_ref;
    double yaw_accel_ref;

    IntegrationMode surge_integrator_mode;
    IntegrationMode heading_integrator_mode;

    inline bool operator==(const AutopilotInputs& rhs) const
    {
        bool equal = (surge_speed_ref == rhs.surge_speed_ref) && (surge_accel_ref == rhs.surge_accel_ref) &&
                     (heading_ref == rhs.heading_ref) && (yaw_rate_ref == rhs.yaw_rate_ref) &&
                     (yaw_accel_ref == rhs.yaw_accel_ref) && (surge_integrator_mode == rhs.surge_integrator_mode) &&
                     (heading_integrator_mode == rhs.heading_integrator_mode);
        return equal;
    }

    inline bool operator!=(const AutopilotInputs& rhs) const
    {
        return !(*this == rhs);
    }
};

struct DPInputs
{
    bool enable_ready_for_autonomy;
    bool enable_autonomy_mode;
    ControlMode desired_control_mode;

    DPReference dp_ref_autonomy;
    GeneralizedForces forces_ref_autonomy;

    DPReference dp_ref_manual;
    GeneralizedForces forces_ref_manual;

    Measurements measurements;
    std::chrono::duration<double> step_size{-1.0};

    AutopilotInputs autopilot_inputs_manual;
    AutopilotInputs autopilot_inputs_autonomy;
};

struct DPOutputs
{
    GeneralizedForces generalized_forces;
    ControlMode control_mode;
    bool ready_for_autonomy_enabled;
    bool autonomy_mode_enabled;
    DPReference dp_ref_active;
    AutopilotInputs active_autopilot_inputs;
};

struct PIDParam
{
    double Kp{0};
    double Ki{0};
    double Kd{0};
};

struct AutopilotParameters
{
    PIDParam pid_heading{};
    PIDParam pid_surge{};
};

struct DPParameters
{
    Eigen::Vector3d Ki;
    Eigen::Vector3d Kp;
    Eigen::Vector3d Kd;
    Eigen::Vector3d generalized_forces_saturation;
    Eigen::Vector3d integral_action_saturation;
    Eigen::Matrix3d mass_matrix;
    Eigen::Matrix3d damping_matrix;
    Eigen::Vector3d max_effort;
    AutopilotParameters autopilot{};
    DPParameters()
    {
        Ki = {0, 0, 0};
        Kp = {0, 0, 0};
        Kd = {0, 0, 0};
        generalized_forces_saturation = {1e20, 1e20, 1e20};
        integral_action_saturation = {1e20, 1e20, 1e20};
        mass_matrix = Eigen::Matrix3d::Zero(3, 3);
        damping_matrix = Eigen::Matrix3d::Zero(3, 3);
        max_effort = {0, 0, 0};
    }

    inline bool operator==(const DPParameters& rhs) const
    {
        bool equal = (Ki == rhs.Ki) && (Kp == rhs.Kp) && (Kd == rhs.Kd);
        return equal;
    }

    inline bool operator!=(const DPParameters& rhs) const
    {
        return !(*this == rhs);
    }
};
}  // namespace DPMock

#endif  // DYNAMIC_POSITIONING_CPP_DATATYPES_HPP
