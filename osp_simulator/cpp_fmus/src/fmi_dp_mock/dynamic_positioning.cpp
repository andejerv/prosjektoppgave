#include "dynamic_positioning.hpp"

#include <zeabuz/common/utilities/math.hpp>
#include <zeabuz/common/utilities/string.hpp>

#include "data_types.hpp"

using zeabuz::common::utilities::math::heading_to_rotation_matrix;
using zeabuz::common::utilities::math::inf2pipi;

namespace DPMock
{

DPController::DPController()
: m_generalized_forces({0, 0, 0})
, m_integral_action({0, 0, 0})
, m_proportional_action({0, 0, 0})
, m_feedForward_action({0, 0, 0})
, m_ready_for_autonomy_status(false)
, m_autonomy_mode_enabled(false)
, m_control_mode(ControlMode::STANDBY)
, m_prev_control_mode(ControlMode::STANDBY)
, m_active_autopilot_inputs()
, m_prev_autopilot_manual_inputs()
{
    m_active_config.mass_matrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    m_active_config.damping_matrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}

void DPController::step(const DPInputs& inputs)
{
    set_system_mode(inputs);
    check_step_size(inputs.step_size.count());
    set_control_mode(inputs.desired_control_mode);
    bool has_ctrl_mode_changed = m_control_mode != inputs.desired_control_mode;

    if (has_ctrl_mode_changed)  // don't want to keep old integral action
    {
        m_integral_action = {0, 0, 0};
        m_head_ctrl_integral_action = 0.0;
        m_surge_speed_ctrl_integral_action = 0.0;
    }

    switch (m_control_mode) {
        case ControlMode::STANDBY:
        {
            execute_standby_mode();
            break;
        }
        case ControlMode::JOYSTICK:
        {
            execute_joystick_mode(inputs);
            break;
        }
        case ControlMode::DP:
        {
            execute_dp_mode(inputs);
            break;
        }
        case ControlMode::AUTOPILOT:
        {
            execute_autopilot_mode(inputs);
            break;
        }
        default:
        {
            throw std::invalid_argument(
                "Invalid controller mode. Available options: {STANDBY; JOYSTICK; DP; AUTOPILOT}");
        }
    }
}

void DPController::execute_standby_mode()
{
    m_generalized_forces = {0, 0, 0};
}

void DPController::execute_joystick_mode(const DPInputs& inputs)
{
    Eigen::Vector3d input_force_ref{};
    if (m_autonomy_mode_enabled) {
        input_force_ref = inputs.forces_ref_autonomy;
    }
    else {
        input_force_ref = inputs.forces_ref_manual;
    }
    m_generalized_forces(0) = static_cast<double>(input_force_ref(0)) / 100.0 * m_active_config.max_effort(0);
    m_generalized_forces(1) = static_cast<double>(input_force_ref(1)) / 100.0 * m_active_config.max_effort(1);
    m_generalized_forces(2) = static_cast<double>(input_force_ref(2)) / 100.0 * m_active_config.max_effort(2);
}

void DPController::execute_dp_mode(const DPInputs& inputs)
{
    bool autonomy_mode_changed_to_false = (m_autonomy_mode_enabled == false) && (m_prev_autonomy_mode == true);

    if (m_autonomy_mode_enabled) {
        m_active_dp_ref = inputs.dp_ref_autonomy;
    }  // when disabling autonomy and changing to DP mode at the same time, set DP setpoints to current measurement
    else if (autonomy_mode_changed_to_false && (m_prev_control_mode != ControlMode::DP)) {
        m_active_dp_ref.north_east_heading = inputs.measurements.north_east_heading;
        m_active_dp_ref.north_east_heading_accel = {0, 0, 0};
        m_active_dp_ref.north_east_heading_vel = {0, 0, 0};
    }
    else if (inputs.dp_ref_manual != m_prev_manual_dp_ref) {
        m_active_dp_ref = inputs.dp_ref_manual;
    }
    else {
        // dp not update
    }
    m_prev_manual_dp_ref = inputs.dp_ref_manual;

    double dt = inputs.step_size.count();
    Eigen::Vector3d eta_error = inputs.measurements.north_east_heading - m_active_dp_ref.north_east_heading;
    eta_error(2) = inf2pipi(eta_error(2));
    Eigen::Matrix3d rot_mat = heading_to_rotation_matrix(inputs.measurements.north_east_heading(2));  // body to NED

    Eigen::Vector3d nu_ref = rot_mat.transpose() * m_active_dp_ref.north_east_heading_vel;
    Eigen::Vector3d nu_ref_dot = rot_mat.transpose() * m_active_dp_ref.north_east_heading_accel;

    Eigen::Vector3d nu_error = inputs.measurements.nu - nu_ref;

    m_proportional_action = -m_active_config.Kp.array() * (rot_mat.transpose() * eta_error).array();
    m_derivative_action = -m_active_config.Kd.array() * nu_error.array();

    m_feedForward_action = m_active_config.mass_matrix * nu_ref_dot + m_active_config.damping_matrix * nu_ref;

    if (m_active_dp_ref.surge_integrator_mode != DPMock::IntegrationMode::FREEZE) {
        m_integral_action(0) = m_integral_action(0) - dt * m_active_config.Ki(0) * eta_error(0);
    }
    if (m_active_dp_ref.sway_integrator_mode != DPMock::IntegrationMode::FREEZE) {
        m_integral_action(1) = m_integral_action(1) - dt * m_active_config.Ki(1) * eta_error(1);
    }
    if (m_active_dp_ref.heading_integrator_mode != DPMock::IntegrationMode::FREEZE) {
        m_integral_action(2) = m_integral_action(2) - dt * m_active_config.Ki(2) * eta_error(2);
    }

    // prevent integral windup
    m_integral_action = m_integral_action.cwiseMin(m_active_config.integral_action_saturation)
                            .cwiseMax(-m_active_config.integral_action_saturation);

    Eigen::Vector3d tau_pid_body =
        rot_mat.transpose() * m_integral_action + m_proportional_action + m_derivative_action + m_feedForward_action;
    m_generalized_forces = tau_pid_body.cwiseMin(m_active_config.generalized_forces_saturation)
                               .cwiseMax(-m_active_config.generalized_forces_saturation);
}

void DPController::execute_autopilot_mode(const DPInputs& inputs)
{
    if (m_autonomy_mode_enabled) {
        m_active_autopilot_inputs = inputs.autopilot_inputs_autonomy;
    }
    else {
        if (inputs.autopilot_inputs_manual != m_prev_autopilot_manual_inputs) {
            m_active_autopilot_inputs = inputs.autopilot_inputs_manual;
        }
    }
    m_prev_autopilot_manual_inputs = inputs.autopilot_inputs_manual;

    double yaw_moment = update_heading_controller(inputs);
    double surge_force = update_surge_speed_controller(inputs);
    m_generalized_forces = {surge_force, 0, yaw_moment};
}

double DPController::update_heading_controller(const DPInputs& inputs)
{
    double dt = inputs.step_size.count();
    double head_error = inputs.measurements.north_east_heading(2) - m_active_autopilot_inputs.heading_ref;

    head_error = inf2pipi(head_error);

    double kp = m_active_config.autopilot.pid_heading.Kp;
    double ki = m_active_config.autopilot.pid_heading.Ki;
    double kd = m_active_config.autopilot.pid_heading.Kd;

    double head_proportional_action = -kp * head_error;

    double head_integrator_action = 0.0;
    if (m_active_autopilot_inputs.heading_integrator_mode != DPMock::IntegrationMode::FREEZE) {
        head_integrator_action = dt * ki * head_error;
    }
    m_head_ctrl_integral_action = m_head_ctrl_integral_action - head_integrator_action;

    // prevent integral windup
    double i_max = m_active_config.integral_action_saturation(2);
    m_head_ctrl_integral_action = std::min(std::max(m_head_ctrl_integral_action, -i_max), i_max);

    double yaw_rate = inputs.measurements.nu(2);
    double error_yaw_rate = yaw_rate - m_active_autopilot_inputs.yaw_rate_ref;
    double head_derivative_action = -kd * error_yaw_rate;

    double m33 = m_active_config.mass_matrix(2, 2);
    double d33 = m_active_config.damping_matrix(2, 2);
    double feedfoward = m33 * m_active_autopilot_inputs.yaw_accel_ref + d33 * m_active_autopilot_inputs.yaw_rate_ref;

    double yaw_moment = feedfoward + m_head_ctrl_integral_action + head_proportional_action + head_derivative_action;
    double max_moment = m_active_config.generalized_forces_saturation(2);
    yaw_moment = std::min(std::max(yaw_moment, -max_moment), max_moment);
    return yaw_moment;
}
double DPController::update_surge_speed_controller(const DPInputs& inputs)
{
    double dt = inputs.step_size.count();
    double surge_speed_error = inputs.measurements.nu(0) - m_active_autopilot_inputs.surge_speed_ref;

    double kp = m_active_config.autopilot.pid_surge.Kp;
    double ki = m_active_config.autopilot.pid_surge.Ki;
    // double Kd = m_active_config.autopilot.pid_surge.Kd;

    double m11 = m_active_config.mass_matrix(0, 0);
    double d11 = m_active_config.damping_matrix(0, 0);

    double feedfoward =
        m11 * m_active_autopilot_inputs.surge_accel_ref + d11 * m_active_autopilot_inputs.surge_speed_ref;

    auto surge_integral_action = 0.0;
    if (m_active_autopilot_inputs.surge_integrator_mode != DPMock::IntegrationMode::FREEZE) {
        surge_integral_action = dt * ki * surge_speed_error;
    }
    m_surge_speed_ctrl_integral_action = m_surge_speed_ctrl_integral_action - surge_integral_action;

    // prevent integral windup
    double i_max = m_active_config.integral_action_saturation(0);
    m_surge_speed_ctrl_integral_action = std::min(std::max(m_surge_speed_ctrl_integral_action, -i_max), i_max);

    double proportional_action = kp * surge_speed_error;

    double surge_force = feedfoward - proportional_action + m_surge_speed_ctrl_integral_action;
    double max_surge_force = m_active_config.generalized_forces_saturation(0);

    // Saturate surge force and don't use active force for reducing speed
    if (m_active_autopilot_inputs.surge_speed_ref >= 0.0) {  // Forward speed
        surge_force = std::min(std::max(surge_force, 0.0), max_surge_force);
    }
    else {  // Backward speed
        surge_force = std::min(std::max(surge_force, -max_surge_force), 0.0);
    }

    return surge_force;
}

void DPController::configure(const DPParameters& param)
{
    m_active_config.Kp = param.Kp;
    m_active_config.Ki = param.Ki;
    m_active_config.Kd = param.Kd;
    m_active_config.generalized_forces_saturation = param.generalized_forces_saturation;
    m_active_config.integral_action_saturation = param.integral_action_saturation;
    m_active_config.mass_matrix = param.mass_matrix;
    m_active_config.damping_matrix = param.damping_matrix;
    m_active_config.max_effort = param.max_effort;
    m_active_config.autopilot = param.autopilot;
}

void DPController::set_control_mode(const ControlMode mode)
{
    m_prev_control_mode = m_control_mode;
    m_control_mode = mode;
}

bool DPController::get_ready_for_autonomy_status() const
{
    return m_ready_for_autonomy_status;
}

bool DPController::get_autonomy_mode_enabled() const
{
    return m_autonomy_mode_enabled;
}

ControlMode DPController::get_control_mode() const
{
    return m_control_mode;
}

void DPController::set_system_mode(const DPInputs& inputs)
{
    m_prev_autonomy_mode = m_autonomy_mode_enabled;
    m_ready_for_autonomy_status = inputs.enable_ready_for_autonomy;
    m_autonomy_mode_enabled = m_ready_for_autonomy_status && inputs.enable_autonomy_mode;
}

Eigen::Vector3d DPController::get_generalized_forces() const
{
    return m_generalized_forces;
}

DPOutputs DPController::get_outputs() const
{
    DPOutputs outputs;
    outputs.control_mode = get_control_mode();
    outputs.autonomy_mode_enabled = get_autonomy_mode_enabled();
    outputs.ready_for_autonomy_enabled = get_ready_for_autonomy_status();
    outputs.generalized_forces = get_generalized_forces();
    outputs.dp_ref_active = get_active_dp_ref();
    outputs.active_autopilot_inputs = get_active_autopilot_inputs();
    return outputs;
}

AutopilotInputs DPController::get_active_autopilot_inputs() const
{
    return m_active_autopilot_inputs;
}

DPReference DPController::get_active_dp_ref() const
{
    return m_active_dp_ref;
}

Eigen::Vector3d DPController::get_integral_action() const
{
    return m_integral_action;
}

Eigen::Vector3d DPController::get_proportional_action() const
{
    return m_proportional_action;
}

Eigen::Vector3d DPController::get_derivative_action() const
{
    return m_derivative_action;
}

Eigen::Vector3d DPController::get_feedforward_action() const
{
    return m_feedForward_action;
}

DPParameters DPController::get_active_config() const
{
    return m_active_config;
}

void DPController::check_step_size(const double step_size)
{
    if (step_size < 0.0) {
        std::stringstream error_message;
        error_message << "Invalid step size. expected stepSize>=0. Got " << step_size;
        throw std::invalid_argument(error_message.str());
    }
}

double DPController::get_heading_ctrl_integral_action() const
{
    return m_head_ctrl_integral_action;
}

double DPController::get_surge_speed_ctrl_integral_action() const
{
    return m_surge_speed_ctrl_integral_action;
}

}  // namespace DPMock
