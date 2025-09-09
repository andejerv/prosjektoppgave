#ifndef DYNAMIC_POSITIONING_CPP_DYNAMIC_POSITIONING_HPP
#define DYNAMIC_POSITIONING_CPP_DYNAMIC_POSITIONING_HPP

#include "data_types.hpp"

namespace DPMock
{

class DPController
{
   public:
    DPController();

    void step(const DPInputs& inputs);

    void configure(const DPParameters& param);

    DPOutputs get_outputs() const;

    DPReference get_active_dp_ref() const;

    Eigen::Vector3d get_integral_action() const;

    Eigen::Vector3d get_proportional_action() const;

    Eigen::Vector3d get_feedforward_action() const;

    Eigen::Vector3d get_derivative_action() const;

    DPParameters get_active_config() const;

    double get_heading_ctrl_integral_action() const;

    double get_surge_speed_ctrl_integral_action() const;

   private:
    Eigen::Vector3d m_generalized_forces;
    Eigen::Vector3d m_integral_action;
    Eigen::Vector3d m_proportional_action;
    Eigen::Vector3d m_derivative_action;
    Eigen::Vector3d m_feedForward_action;

    double m_head_ctrl_integral_action{0};
    double m_surge_speed_ctrl_integral_action{0};

    bool m_ready_for_autonomy_status;
    bool m_autonomy_mode_enabled;
    bool m_prev_autonomy_mode{false};
    ControlMode m_control_mode;
    ControlMode m_prev_control_mode;

    DPReference m_active_dp_ref;
    AutopilotInputs m_active_autopilot_inputs;

    DPParameters m_active_config;
    DPReference m_prev_manual_dp_ref;
    AutopilotInputs m_prev_autopilot_manual_inputs;

    void set_control_mode(const ControlMode mode);

    bool get_autonomy_mode_enabled() const;

    bool get_ready_for_autonomy_status() const;

    ControlMode get_control_mode() const;

    Eigen::Vector3d get_generalized_forces() const;

    void execute_joystick_mode(const DPInputs& inputs);

    void execute_dp_mode(const DPInputs& inputs);

    void execute_autopilot_mode(const DPInputs& inputs);

    void check_step_size(const double step_size);

    void set_system_mode(const DPInputs& inputs);

    void execute_standby_mode();

    double update_heading_controller(const DPInputs& inputs);
    double update_surge_speed_controller(const DPInputs& inputs);
    AutopilotInputs get_active_autopilot_inputs() const;
};
}  // namespace DPMock

#endif
