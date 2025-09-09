#include "hatch_control_handler.hpp"

#include <stdexcept>
#include <zeabuz/common/io/message_handler.hpp>
#include <zeabuz/common/io/messages/custom/wago_hatch.hpp>
#include <zeabuz/common/io/messages/custom/wago_hatch_fb.hpp>
#include <zeabuz/common/utilities/enums.hpp>

#include "fmi_hatch_control/hatch_control.hpp"

using NMEAProtocol::HatchControlHandler;

using namespace zeabuz::common::io;
using namespace zeabuz::common::utilities::enums;

HatchControlHandler::HatchControlHandler(std::shared_ptr<message_handlers::NMEA0183MessageHandler> message_handler,
                                         std::unordered_map<uint16_t, std::shared_ptr<HatchControl>> hatch_controllers)
: m_message_handler(message_handler), m_hatch_controllers(hatch_controllers)
{
}

void HatchControlHandler::initialize_fmu()
{
    m_message_handler->subscribe_to_message(&HatchControlHandler::hatch_command_cb, this);
    m_message_handler->run();
}

void HatchControlHandler::hatch_command_cb(const messages::custom::WAGOHatchMessage& message)
{
    if (m_hatch_controllers.find(message.data.hatch_id) != m_hatch_controllers.end()) {
        using type = HatchTypes::Command;
        auto command = enum_convert<type>(message.data.command);
        m_hatch_controllers.at(message.data.hatch_id)->hatch_command_callback(command);
    }
    else {
        spdlog::error("{} Hatch_id {} is not a valid ID, check that correct hatch id is being used.",
                      ZB_PRETTY_FUNCTION, message.data.hatch_id);
    }
}

void HatchControlHandler::execute(const double time_step, const bool emergency_stop_active)
{
    m_emergency_stop_state.store(emergency_stop_active);
    for (auto& controller : m_hatch_controllers) {
        controller.second->set_emergency_stop_state(emergency_stop_active);
        controller.second->execute(time_step);
    }
}

messages::custom::WagoHatchFbMessage HatchControlHandler::get_hatch_feedback() const
{
    using state_type = messages::custom::WagoHatchFbMessageStruct::State;
    using action_type = messages::custom::WagoHatchFbMessageStruct::Action;
    using fault_type = messages::custom::WagoHatchFbMessageStruct::Fault;
    std::vector<messages::custom::WagoHatchFbMessageStruct::HatchStatus> hatch_statuses{};
    for (const auto& controller : m_hatch_controllers) {
        auto hatch_status = controller.second->get_hatch_status();
        auto hatch_id = controller.first;
        auto state = enum_convert<state_type>(hatch_status.state);
        auto action = enum_convert<action_type>(hatch_status.action);
        auto fault = enum_convert<fault_type>(hatch_status.fault);
        hatch_statuses.push_back({hatch_id, state, action, fault});
    }
    messages::custom::WagoHatchFbMessage msg;
    msg.data.emergency_state = m_emergency_stop_state.load();
    msg.data.hatch_statuses = hatch_statuses;
    return msg;
}
void HatchControlHandler::send_hatch_feedback()
{
    auto msg = get_hatch_feedback();
    m_message_handler->send_message(msg);
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