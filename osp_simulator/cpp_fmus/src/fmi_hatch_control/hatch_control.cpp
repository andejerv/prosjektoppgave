#include "fmi_hatch_control/hatch_control.hpp"

#include <algorithm>
#include <iostream>
#include <zeabuz/common/utilities/enums.hpp>

#include "spdlog/spdlog.h"

HatchControl::HatchControl(const double hatch_timeout, const double start_position)
: m_hatch_timeout(hatch_timeout), m_hatch_position(start_position)
{
    determine_initial_state();
    setup_command_callbacks();
}

void HatchControl::determine_initial_state()
{
    if (m_hatch_position == 100.0) {
        m_current_hatch_state = HatchTypes::State::STATE_CLOSED;
        m_hatch_sensor_status = HatchTypes::HatchSensorStates::DI_CLOSED;
    }
    else if (m_hatch_position == 0.0) {
        m_current_hatch_state = HatchTypes::State::STATE_OPEN;
        m_hatch_sensor_status = HatchTypes::HatchSensorStates::DI_OPENED;
    }
    else {
        m_current_hatch_state = HatchTypes::State::STATE_UNDEFINED;
        m_hatch_sensor_status = HatchTypes::HatchSensorStates::UNDEFINED;
    }
}

void HatchControl::setup_command_callbacks()
{
    m_execute_functions_map.emplace(HatchTypes::Command::COMMAND_OPEN, &HatchControl::command_open);
    m_execute_functions_map.emplace(HatchTypes::Command::COMMAND_CLOSE, &HatchControl::command_close);
    m_execute_functions_map.emplace(HatchTypes::Command::COMMAND_50, &HatchControl::command_50);
    m_execute_functions_map.emplace(HatchTypes::Command::COMMAND_STOP, &HatchControl::command_stop);
    m_execute_functions_map.emplace(HatchTypes::Command::COMMAND_RESET_FAULT, &HatchControl::command_reset_fault);
    m_hatch_execute_function = m_execute_functions_map.at(HatchTypes::Command::COMMAND_STOP);
}

void HatchControl::set_emergency_stop_state(const bool emergency_stop_active)
{
    if (m_hatch_sensor_status != HatchTypes::HatchSensorStates::DI_EM_STOP) {
        m_hatch_sensor_previous_status = m_hatch_sensor_status.load();
    }
    m_hatch_sensor_status =
        (emergency_stop_active == true) ? HatchTypes::HatchSensorStates::DI_EM_STOP : m_hatch_sensor_previous_status;
}

bool HatchControl::check_hatch_ready_for_action(const HatchTypes::Command command) const
{
    if (m_hatch_sensor_status != HatchTypes::HatchSensorStates::DI_EM_STOP) {
        if ((command == HatchTypes::Command::COMMAND_RESET_FAULT) ||
            (m_current_hatch_fault == HatchTypes::Fault::FAULT_NO_FAULT)) {
            return true;
        }
    }
    return false;
}

void HatchControl::hatch_command_callback(const HatchTypes::Command command)
{
    if (check_hatch_ready_for_action(command)) {
        set_hatch_action(command);
    }
    else {
        spdlog::error("Hatch not in state to receive action. Check emergency stop and fault states for hatches.");
    }
}

void HatchControl::set_hatch_action(const HatchTypes::Command command)
{
    try {
        m_hatch_command_time_spent = 0.0;
        m_hatch_execute_function = m_execute_functions_map.at(command);
        m_current_hatch_command = command;
    }
    catch (const std::out_of_range& e) {
        std::ostringstream oss;
        oss << __FUNCTION__ << ": " << __LINE__ << " - Unknown hatch command "
            << zeabuz::common::utilities::enums::get_underlying_value(command) << ", no action is valid.";
        throw std::out_of_range(oss.str());
    }
}

HatchTypes::HatchStatus HatchControl::get_hatch_status() const
{
    auto status = HatchTypes::HatchStatus{};
    status.state = m_current_hatch_state;
    status.action = m_current_hatch_action;
    status.fault = m_current_hatch_fault;
    return status;
}

void HatchControl::command_open()
{
    if (m_hatch_sensor_status != HatchTypes::HatchSensorStates::DI_OPENED) {
        m_current_hatch_action = HatchTypes::Action::ACTION_OPENING;
        m_current_hatch_state = HatchTypes::State::STATE_UNDEFINED;
        perform_stepping_of_hatch_position(-m_hatch_step_size);
        if (check_hatch_position(0.0)) {
            m_hatch_sensor_status = HatchTypes::HatchSensorStates::DI_OPENED;
            m_current_hatch_action = HatchTypes::Action::ACTION_IDLE;
            m_current_hatch_state = HatchTypes::State::STATE_OPEN;
        }
        else if (check_time_spent_performing_action()) {
            m_current_hatch_fault = HatchTypes::Fault::FAULT_OPEN_TIMEOUT;
        }
    }
}

void HatchControl::command_close()
{
    if (m_hatch_sensor_status != HatchTypes::HatchSensorStates::DI_CLOSED) {
        m_current_hatch_action = HatchTypes::Action::ACTION_CLOSING;
        m_current_hatch_state = HatchTypes::State::STATE_UNDEFINED;
        perform_stepping_of_hatch_position(m_hatch_step_size);
        if (check_hatch_position(100.0)) {
            m_hatch_sensor_status = HatchTypes::HatchSensorStates::DI_CLOSED;
            m_current_hatch_action = HatchTypes::Action::ACTION_IDLE;
            m_current_hatch_state = HatchTypes::State::STATE_CLOSED;
        }
        else if (check_time_spent_performing_action()) {
            m_current_hatch_fault = HatchTypes::Fault::FAULT_CLOSE_TIMEOUT;
        }
    }
}

void HatchControl::command_reset_fault()
{
    m_current_hatch_fault = HatchTypes::Fault::FAULT_NO_FAULT;
    m_current_hatch_action = HatchTypes::Action::ACTION_IDLE;
}

void HatchControl::command_50()
{
    if (m_hatch_sensor_status != HatchTypes::HatchSensorStates::DI_50) {
        switch (m_hatch_sensor_status) {
            case HatchTypes::HatchSensorStates::DI_OPENED:
            {
                command_50_close();
                break;
            }
            case HatchTypes::HatchSensorStates::DI_CLOSED:
            {
                command_50_open();
                break;
            }
            default:
            {
                // Invalid state of hatch for 50% command, do nothing
                std::stringstream oss;
                oss << "Tried performing command_50 action when hatch is not in either end positions. No action taken.";
                std::cerr << oss.str() << std::endl;
                break;
            }
        }
    }
}

void HatchControl::command_50_close()
{
    m_current_hatch_action = HatchTypes::Action::ACTION_50_CLOSING;
    m_current_hatch_state = HatchTypes::State::STATE_UNDEFINED;
    perform_stepping_of_hatch_position(m_hatch_step_size);
    if (check_hatch_position(50.0)) {
        m_hatch_sensor_status = HatchTypes::HatchSensorStates::DI_50;
        m_current_hatch_action = HatchTypes::Action::ACTION_IDLE;
        m_current_hatch_state = HatchTypes::State::STATE_50;
    }
    else if (check_time_spent_performing_action()) {
        m_current_hatch_fault = HatchTypes::Fault::FAULT_50_CLOSING_TIMEOUT;
    }
}

void HatchControl::command_50_open()
{
    m_current_hatch_action = HatchTypes::Action::ACTION_50_OPENING;
    m_current_hatch_state = HatchTypes::State::STATE_UNDEFINED;
    perform_stepping_of_hatch_position(-m_hatch_step_size);
    if (check_hatch_position(50.0)) {
        m_hatch_sensor_status = HatchTypes::HatchSensorStates::DI_50;
        m_current_hatch_action = HatchTypes::Action::ACTION_IDLE;
        m_current_hatch_state = HatchTypes::State::STATE_50;
    }
    else if (check_time_spent_performing_action()) {
        m_current_hatch_fault = HatchTypes::Fault::FAULT_50_OPENING_TIMEOUT;
    }
}

void HatchControl::command_stop()
{
    if (m_current_hatch_fault == HatchTypes::Fault::FAULT_NO_FAULT) {
        m_current_hatch_action = HatchTypes::Action::ACTION_IDLE;
    }
}

void HatchControl::perform_stepping_of_hatch_position(const double step_size)
{
    m_hatch_position = m_hatch_position + step_size;
    // Restrict position to 0-100%
    m_hatch_position = std::max(std::min(m_hatch_position.load(), 100.0), 0.0);
}

bool HatchControl::check_time_spent_performing_action() const
{
    return (m_hatch_command_time_spent > m_hatch_timeout) ? true : false;
}

bool HatchControl::check_hatch_position(const double target_position)
{
    if (std::abs(m_hatch_position.load() - target_position) < M_HATCH_POSITION_SUCCESS_THRESHOLD) {
        m_hatch_position = target_position;
        return true;
    }
    return false;
}

void HatchControl::execute(const double time_step)
{
    if (check_hatch_ready_for_action(m_current_hatch_command)) {
        calculate_hatch_step_size(time_step);
        m_hatch_execute_function(*this);
        iterate_time_taken_by_action(time_step);
    }
}

void HatchControl::iterate_time_taken_by_action(const double time_step)
{
    m_hatch_command_time_spent = m_hatch_command_time_spent + time_step;
}

void HatchControl::calculate_hatch_step_size(const double time_step)
{
    m_hatch_step_size = 100.0 * time_step / (m_hatch_timeout - 1.0);
}

HatchTypes::HatchSensorStates HatchControl::get_hatch_sensor_status() const
{
    return m_hatch_sensor_status;
}

double HatchControl::get_hatch_position() const
{
    return m_hatch_position;
}
