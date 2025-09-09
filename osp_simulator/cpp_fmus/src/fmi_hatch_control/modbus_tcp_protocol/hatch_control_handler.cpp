

#include "hatch_control_handler.hpp"

#include <bitset>
#include <mutex>
#include <stdexcept>
#include <zeabuz/common/io/modbus_tcp_server.hpp>
#include <zeabuz/common/utilities/pretty_class_macro.hpp>

#include "fmi_hatch_control/hatch_control.hpp"
#include "spdlog/spdlog.h"

using HatchTypes::Action;
using HatchTypes::Fault;
using HatchTypes::State;
using HatchFore = ModbusTcpProtocol::Signals::HatchFore;
using HatchAft = ModbusTcpProtocol::Signals::HatchAft;
using ModbusTcpProtocol::HatchControlHandler;

using namespace zeabuz::common::io::modbus_tcp;

HatchControlHandler::HatchControlHandler(std::shared_ptr<Server> modbus_config,
                                         std::unordered_map<uint16_t, std::shared_ptr<HatchControl>> hatch_controllers)
: m_modbus_server(modbus_config), m_hatch_controllers(hatch_controllers)
{
}

std::unordered_map<uint16_t, double> HatchControlHandler::get_hatch_positions() const
{
    auto positions = std::unordered_map<uint16_t, double>{};
    for (const auto& controller : m_hatch_controllers) {
        auto position = controller.second->get_hatch_position();
        positions.emplace(controller.first, position);
    }
    return positions;
}

void HatchControlHandler::hatch_command_cb(const HatchTypes::HatchCommand& command)
{
    if (m_hatch_controllers.find(command.hatch_id) != m_hatch_controllers.end()) {
        m_hatch_controllers.at(command.hatch_id)->hatch_command_callback(command.command);
    }
    else {
        spdlog::error("{} Hatch_id: {} is not a valid ID, check that correct hatch id is being used",
                      ZB_PRETTY_FUNCTION, command.hatch_id);
    }
}

HatchTypes::HatchStatuses HatchControlHandler::get_hatch_feedback() const
{
    auto statuses = HatchTypes::HatchStatuses{};
    statuses.auto_manual_control = m_auto_manual_mode;
    for (const auto& controller : m_hatch_controllers) {
        auto hatch_status = controller.second->get_hatch_status();
        hatch_status.hatch_id = controller.first;
        statuses.hatch_statuses.push_back(hatch_status);
    }
    return statuses;
}

void HatchControlHandler::execute(const double time_step, const bool emergency_stop_active, const bool auto_manual_mode)
{
    update_hatch_command();
    m_auto_manual_mode = auto_manual_mode;
    m_emergency_stop_state.store(emergency_stop_active);
    for (auto& controller : m_hatch_controllers) {
        controller.second->set_emergency_stop_state(emergency_stop_active);
        controller.second->execute(time_step);
    }
    update_modbus_server();
}

void HatchControlHandler::update_modbus_server()
{
    auto status = get_hatch_feedback();
    if (m_hatch_statuses != status) {
        auto register_value = status_to_bits(status);
        std::lock_guard<std::mutex> lock{m_server_mutex};
        m_modbus_server->set_input_register(0, register_value);
    }
    m_hatch_statuses = status;
}

void HatchControlHandler::update_hatch_command()
{
    m_server_mutex.lock();
    auto reg_value = m_modbus_server->get_register(0);
    m_server_mutex.unlock();
    try {
        // Hatch driver will send a reset command as 0, so do nothing for 0 values
        if (reg_value > 0) {
            auto command = value_to_command(reg_value);
            if (command != m_prev_command) {
                hatch_command_cb(command);
            }
            m_prev_command = command;
        }
    }
    catch (const std::runtime_error& e) {
        spdlog::error("{}, received invalid command from client, {}", ZB_PRETTY_FUNCTION, e.what());
    }
}

HatchTypes::HatchCommand ModbusTcpProtocol::value_to_command(const uint16_t value)
{
    using ModbusTcpProtocol::Signals;
    auto command = HatchTypes::HatchCommand{};
    auto bits = std::bitset<16>{value};
    bool is_hatch_fore =
        bits.test(HatchFore::COMMAND_OPEN) || bits.test(HatchFore::COMMAND_CLOSE) || bits.test(HatchFore::COMMAND_STOP);
    bool is_hatch_aft =
        bits.test(HatchAft::COMMAND_OPEN) || bits.test(HatchAft::COMMAND_CLOSE) || bits.test(HatchAft::COMMAND_STOP);
    if (is_hatch_aft || is_hatch_fore) {
        if (is_hatch_fore) {
            command.hatch_id = HatchControlHandler::HATCH_ID_FORE;
        }
        else if (is_hatch_aft) {
            command.hatch_id = HatchControlHandler::HATCH_ID_AFT;
        }

        if (bits.test(HatchFore::COMMAND_OPEN) || bits.test(HatchAft::COMMAND_OPEN)) {
            command.command = HatchTypes::Command::COMMAND_OPEN;
        }
        else if (bits.test(HatchFore::COMMAND_CLOSE) || bits.test(HatchAft::COMMAND_CLOSE)) {
            command.command = HatchTypes::Command::COMMAND_CLOSE;
        }
        else if (bits.test(HatchFore::COMMAND_STOP) || bits.test(HatchAft::COMMAND_STOP)) {
            command.command = HatchTypes::Command::COMMAND_STOP;
        }
        else {
            std::ostringstream oss{};
            oss << ZB_PRETTY_FUNCTION << " Received invalid hatch command for ModbusTcpServer";
            throw std::runtime_error(oss.str());
        }
    }
    return command;
}

uint16_t ModbusTcpProtocol::status_to_bits(const HatchTypes::HatchStatuses& status)
{
    auto modbus_state = std::bitset<16>{};
    modbus_state.set(Signals::AUTO_MANUAL_CONTROL, status.auto_manual_control == true ? 1 : 0);

    for (const auto& hatch : status.hatch_statuses) {
        if (hatch.hatch_id == HatchControlHandler::HATCH_ID_FORE) {
            modbus_state.set(HatchFore::ACTION_OPENING, hatch.action == Action::ACTION_OPENING ? 1 : 0);
            modbus_state.set(HatchFore::ACTION_CLOSING, hatch.action == Action::ACTION_CLOSING ? 1 : 0);
            modbus_state.set(HatchFore::STATE_OPEN, hatch.state == State::STATE_OPEN ? 0 : 1);
            modbus_state.set(HatchFore::STATE_CLOSE, hatch.state == State::STATE_CLOSED ? 0 : 1);
            bool fore_timeout =
                (hatch.fault == Fault::FAULT_OPEN_TIMEOUT) || (hatch.fault == Fault::FAULT_CLOSE_TIMEOUT);
            modbus_state.set(HatchFore::FAULT_TIMEOUT, fore_timeout ? 1 : 0);
            modbus_state.set(HatchFore::FAULT_GENERAL, hatch.fault == Fault::FAULT_GENERAL_FAULT ? 1 : 0);
        }
        else if (hatch.hatch_id == HatchControlHandler::HATCH_ID_AFT) {
            modbus_state.set(HatchAft::ACTION_OPENING, hatch.action == Action::ACTION_OPENING ? 1 : 0);
            modbus_state.set(HatchAft::ACTION_CLOSING, hatch.action == Action::ACTION_CLOSING ? 1 : 0);
            modbus_state.set(HatchAft::STATE_OPEN, hatch.state == State::STATE_OPEN ? 0 : 1);
            modbus_state.set(HatchAft::STATE_CLOSE, hatch.state == State::STATE_CLOSED ? 0 : 1);
            bool aft_timeout =
                (hatch.fault == Fault::FAULT_OPEN_TIMEOUT) || (hatch.fault == Fault::FAULT_CLOSE_TIMEOUT);
            modbus_state.set(HatchAft::FAULT_TIMEOUT, aft_timeout ? 1 : 0);
            modbus_state.set(HatchAft::FAULT_GENERAL, hatch.fault == Fault::FAULT_GENERAL_FAULT ? 1 : 0);
        }
        else {
            spdlog::error("Hatch Status is not valid for provided Hatch ID. Provided: `{}`, valid: `[{},{}]`",
                          hatch.hatch_id, HatchControlHandler::HATCH_ID_FORE, HatchControlHandler::HATCH_ID_AFT);
        }
    }
    return static_cast<uint16_t>(modbus_state.to_ulong());
}
