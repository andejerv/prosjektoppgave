#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

#include <zeabuz/common/io/messages/custom/wago_hatch.hpp>
#include "fmi_hatch_control/hatch_control.hpp"
#include "fmi_hatch_control/modbus_tcp_protocol/hatch_control_handler.hpp"
#include "fmi_hatch_control/nmea_protocol/hatch_control_handler.hpp"
#include "zeabuz/common/io/message_handler.hpp"
#include "zeabuz/common/io/message_handler_descriptions.hpp"
#include <zeabuz/common/utilities/enums.hpp>



using namespace zeabuz::common::io;
using zeabuz::common::io::messages::custom::WagoHatchFbMessageStruct;
using zeabuz::common::io::messages::custom::WAGOHatchMessageStruct;
using zeabuz::common::io::messages::custom::WagoHatchFbMessage;
using hatch_command = zeabuz::common::io::messages::custom::WAGOHatchMessageStruct::HatchCommand;
using zeabuz::common::utilities::enums::enum_convert;

class NmeaHatchControlTests : public ::testing::Test
{
   public:
    std::shared_ptr<HatchControl> m_hatch_control_fore;
    std::shared_ptr<HatchControl> m_hatch_control_aft;
    std::shared_ptr<NMEAProtocol::HatchControlHandler> m_hatch_control_handler;
    std::unordered_map<uint16_t, std::shared_ptr<HatchControl>> m_hatch_control_map;
    uint16_t m_hatch_command_timeout{5};
    double m_dt{0.05};
    double m_full_action_just_incomplete_time{};
    double m_half_action_just_incomplete_time{};
    uint16_t m_hatch_id_fore{1};
    uint16_t m_hatch_id_aft{2};

   public:
    NmeaHatchControlTests()
    {
        auto message_handler = std::make_shared<message_handlers::NMEA0183MessageHandler>();

        m_hatch_control_fore = std::make_shared<HatchControl>(m_hatch_command_timeout, 100.0);
        m_hatch_control_aft = std::make_shared<HatchControl>(m_hatch_command_timeout, 100.0);

        m_hatch_control_map.emplace(static_cast<uint16_t>(m_hatch_id_fore), m_hatch_control_fore);
        m_hatch_control_map.emplace(static_cast<uint16_t>(m_hatch_id_aft), m_hatch_control_aft);
        m_hatch_control_handler =
            std::make_unique<NMEAProtocol::HatchControlHandler>(message_handler, m_hatch_control_map);
        calculate_hatch_move_time();
    }

    WagoHatchFbMessageStruct::HatchStatus find_hatch_status(
        const uint16_t hatch_id, const std::vector<WagoHatchFbMessageStruct::HatchStatus>& statuses)
    {
        auto find_criteria = [hatch_id](const WagoHatchFbMessageStruct::HatchStatus status) -> bool {
            return (status.hatch_id == hatch_id) ? true : false;
        };
        auto hatch_status = std::find_if(statuses.begin(), statuses.end(), find_criteria);
        if (hatch_status == statuses.end()) {
            throw std::out_of_range("Hatch Id not found in vector");
        }
        return *hatch_status;
    }

    void go_to_hatch_state(const uint16_t hatch_id, const hatch_command command, const bool emergency_stop_active)
    {
        m_hatch_control_map.at(hatch_id)->hatch_command_callback(
            enum_convert<HatchTypes::Command>(command));
        for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
            m_hatch_control_handler->execute(m_dt, emergency_stop_active);
        }
    }

    void force_timeout_state(const uint16_t hatch_id, const hatch_command command)
    {
        m_hatch_control_map.at(hatch_id)->hatch_command_callback(
            enum_convert<HatchTypes::Command>(command));
        m_hatch_control_handler->execute(m_dt, false);
        for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
            m_hatch_control_map.at(hatch_id)->iterate_time_taken_by_action(m_dt);
        }
        m_hatch_control_handler->execute(m_dt, false);
    }

    void calculate_hatch_move_time()
    {
        m_full_action_just_incomplete_time = m_hatch_command_timeout - (1.0 + 4 * m_dt);
        m_half_action_just_incomplete_time = m_hatch_command_timeout / 2.0 - (1.0 + 4 * m_dt);
    }
};

TEST_F(NmeaHatchControlTests, T01_command_callback_validity)
{
    EXPECT_NO_THROW(m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_50)));
    EXPECT_NO_THROW(m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_CLOSE)));
    EXPECT_NO_THROW(m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_OPEN)));
    EXPECT_NO_THROW(m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_STOP)));
    EXPECT_NO_THROW(m_hatch_control_fore->hatch_command_callback(enum_convert<HatchTypes::Command>(
        WAGOHatchMessageStruct::HatchCommand::COMMAND_RESET_FAULT)));
    EXPECT_THROW(m_hatch_control_fore->hatch_command_callback(
                     enum_convert<HatchTypes::Command>(static_cast<hatch_command>(10))),
                 std::invalid_argument);
}

TEST_F(NmeaHatchControlTests, T01_command_emergency_stop)
{
    go_to_hatch_state(1, WAGOHatchMessageStruct::HatchCommand::COMMAND_OPEN, true);

    auto msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(msg.data.emergency_state, true);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_CLOSED);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_IDLE);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
}

TEST_F(NmeaHatchControlTests, T01_command_stop)
{
    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_OPEN));
    m_hatch_control_handler->execute(m_dt, false);

    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_STOP));
    m_hatch_control_handler->execute(m_dt, false);

    auto msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_IDLE);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
}

TEST_F(NmeaHatchControlTests, T01_command_stop_restart)
{
    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_OPEN));
    m_hatch_control_handler->execute(m_dt, false);

    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_STOP));
    m_hatch_control_handler->execute(m_dt, false);

    go_to_hatch_state(1, WAGOHatchMessageStruct::HatchCommand::COMMAND_OPEN, false);

    auto msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_OPEN);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_IDLE);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
}

TEST_F(NmeaHatchControlTests, T01_command_force_timeout)
{
    force_timeout_state(1, WAGOHatchMessageStruct::HatchCommand::COMMAND_OPEN);
    auto msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_OPENING);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_OPEN_TIMEOUT);
}

TEST_F(NmeaHatchControlTests, T01_command_fault_reset)
{
    force_timeout_state(1, WAGOHatchMessageStruct::HatchCommand::COMMAND_OPEN);

    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_RESET_FAULT));
    m_hatch_control_handler->execute(m_dt, false);

    auto msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_IDLE);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);

    go_to_hatch_state(1, WAGOHatchMessageStruct::HatchCommand::COMMAND_OPEN, false);
    msg = m_hatch_control_handler->get_hatch_feedback();
    hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_OPEN);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_IDLE);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
}

TEST_F(NmeaHatchControlTests, T02a_command_hatch_open)
{
    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_OPEN));
    WagoHatchFbMessage msg{};
    for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
        m_hatch_control_handler->execute(m_dt, false);
        if (i <= m_full_action_just_incomplete_time) {
            // Expect Hatch to be in undefined position while opening with no faults
            msg = m_hatch_control_handler->get_hatch_feedback();
            auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
            EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
            EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_OPENING);
            EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
        }
    }

    msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_OPEN);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_IDLE);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
}

TEST_F(NmeaHatchControlTests, T02b_command_hatch_close)
{
    go_to_hatch_state(1, WAGOHatchMessageStruct::HatchCommand::COMMAND_OPEN, false);
    EXPECT_EQ(m_hatch_control_fore->get_hatch_sensor_status(), HatchTypes::HatchSensorStates::DI_OPENED);

    // Starting from Open position
    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_CLOSE));
    WagoHatchFbMessage msg{};
    for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
        m_hatch_control_handler->execute(m_dt, false);
        if (i <= m_full_action_just_incomplete_time) {
            // Expect Hatch to be in undefined position while closing with no faults
            msg = m_hatch_control_handler->get_hatch_feedback();
            auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
            EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
            EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_CLOSING);
            EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
        }
    }

    msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_CLOSED);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_IDLE);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
}

TEST_F(NmeaHatchControlTests, T02c_command_hatch_from_close_to_50)
{
    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_50));
    WagoHatchFbMessage msg{};
    for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
        m_hatch_control_handler->execute(m_dt, false);
        if (i <= m_half_action_just_incomplete_time) {
            // Expect Hatch to be in undefined position while opening with no faults
            msg = m_hatch_control_handler->get_hatch_feedback();
            auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
            EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
            EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_50_OPENING);
            EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
        }
    }

    msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_50);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_IDLE);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
}

TEST_F(NmeaHatchControlTests, T02d_command_hatch_from_50_to_close)
{
    go_to_hatch_state(1, WAGOHatchMessageStruct::HatchCommand::COMMAND_50, false);
    EXPECT_EQ(m_hatch_control_fore->get_hatch_sensor_status(), HatchTypes::HatchSensorStates::DI_50);

    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_CLOSE));
    WagoHatchFbMessage msg{};
    for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
        m_hatch_control_handler->execute(m_dt, false);
        if (i <= m_half_action_just_incomplete_time) {
            // Expect Hatch to be in undefined position while closing with no faults
            msg = m_hatch_control_handler->get_hatch_feedback();
            auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
            EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
            EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_CLOSING);
            EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
        }
    }

    msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_CLOSED);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_IDLE);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
}

TEST_F(NmeaHatchControlTests, T02e_command_hatch_from_50_to_open)
{
    go_to_hatch_state(1, WAGOHatchMessageStruct::HatchCommand::COMMAND_50, false);
    EXPECT_EQ(m_hatch_control_fore->get_hatch_sensor_status(), HatchTypes::HatchSensorStates::DI_50);

    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_OPEN));
    WagoHatchFbMessage msg{};
    for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
        m_hatch_control_handler->execute(m_dt, false);
        if (i <= m_half_action_just_incomplete_time) {
            // Expect Hatch to be in undefined position while opening with no faults
            msg = m_hatch_control_handler->get_hatch_feedback();
            auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
            EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
            EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_OPENING);
            EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
        }
    }

    msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_OPEN);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_IDLE);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
}

TEST_F(NmeaHatchControlTests, T02f_command_hatch_from_open_to_50)
{
    go_to_hatch_state(1, WAGOHatchMessageStruct::HatchCommand::COMMAND_OPEN, false);
    EXPECT_EQ(m_hatch_control_fore->get_hatch_sensor_status(), HatchTypes::HatchSensorStates::DI_OPENED);

    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_50));
    WagoHatchFbMessage msg{};
    for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
        m_hatch_control_handler->execute(m_dt, false);
        if (i <= m_half_action_just_incomplete_time) {
            // Expect Hatch to be in undefined position while closing with no faults
            msg = m_hatch_control_handler->get_hatch_feedback();
            auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
            EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
            EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_50_CLOSING);
            EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
        }
    }

    msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_50);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_IDLE);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
}

TEST_F(NmeaHatchControlTests, T03a_command_hatch_open_timeout)
{
    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_OPEN));
    WagoHatchFbMessage msg{};
    for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
        if (i < m_full_action_just_incomplete_time) {
            m_hatch_control_handler->execute(m_dt, false);
            // Expect Hatch to be in undefined position while opening with no faults
            msg = m_hatch_control_handler->get_hatch_feedback();
            auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
            EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
            EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_OPENING);
            EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
        }
        else {
            m_hatch_control_fore->iterate_time_taken_by_action(m_dt);
        }
    }
    m_hatch_control_handler->execute(m_dt, false);

    msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_OPENING);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_OPEN_TIMEOUT);
}

TEST_F(NmeaHatchControlTests, T03b_command_hatch_close_timeout)
{
    go_to_hatch_state(1, WAGOHatchMessageStruct::HatchCommand::COMMAND_OPEN, false);
    EXPECT_EQ(m_hatch_control_fore->get_hatch_sensor_status(), HatchTypes::HatchSensorStates::DI_OPENED);

    // Starting from Open position
    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_CLOSE));
    WagoHatchFbMessage msg{};
    for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
        if (i <= m_full_action_just_incomplete_time) {
            // Expect Hatch to be in undefined position while closing with no faults
            m_hatch_control_handler->execute(m_dt, false);
            msg = m_hatch_control_handler->get_hatch_feedback();
            auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
            EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
            EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_CLOSING);
            EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
        }
        else {
            m_hatch_control_fore->iterate_time_taken_by_action(m_dt);
        }
    }
    m_hatch_control_handler->execute(m_dt, false);

    msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_CLOSING);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_CLOSE_TIMEOUT);
}

TEST_F(NmeaHatchControlTests, T03c_command_hatch_from_close_to_50_timeout)
{
    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_50));
    WagoHatchFbMessage msg{};
    for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
        if (i <= m_half_action_just_incomplete_time) {
            // Expect Hatch to be in undefined position while opening with no faults
            m_hatch_control_handler->execute(m_dt, false);
            msg = m_hatch_control_handler->get_hatch_feedback();
            auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
            EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
            EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_50_OPENING);
            EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
        }
        else {
            m_hatch_control_fore->iterate_time_taken_by_action(m_dt);
        }
    }
    m_hatch_control_handler->execute(m_dt, false);

    msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_50_OPENING);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_50_OPENING_TIMEOUT);
}

TEST_F(NmeaHatchControlTests, T03d_command_hatch_from_50_to_close_timeout)
{
    go_to_hatch_state(1, WAGOHatchMessageStruct::HatchCommand::COMMAND_50, false);
    EXPECT_EQ(m_hatch_control_fore->get_hatch_sensor_status(), HatchTypes::HatchSensorStates::DI_50);

    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_CLOSE));
    WagoHatchFbMessage msg{};
    for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
        if (i <= m_half_action_just_incomplete_time) {
            // Expect Hatch to be in undefined position while closing with no faults
            m_hatch_control_handler->execute(m_dt, false);
            msg = m_hatch_control_handler->get_hatch_feedback();
            auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
            EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
            EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_CLOSING);
            EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
        }
        else {
            m_hatch_control_fore->iterate_time_taken_by_action(m_dt);
        }
    }
    m_hatch_control_handler->execute(m_dt, false);

    msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_CLOSING);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_CLOSE_TIMEOUT);
}

TEST_F(NmeaHatchControlTests, T03e_command_hatch_from_50_to_open_timeout)
{
    go_to_hatch_state(1, WAGOHatchMessageStruct::HatchCommand::COMMAND_50, false);
    EXPECT_EQ(m_hatch_control_fore->get_hatch_sensor_status(), HatchTypes::HatchSensorStates::DI_50);

    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_OPEN));
    WagoHatchFbMessage msg{};
    for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
        if (i <= m_half_action_just_incomplete_time) {
            // Expect Hatch to be in undefined position while opening with no faults
            m_hatch_control_handler->execute(m_dt, false);
            msg = m_hatch_control_handler->get_hatch_feedback();
            auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
            EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
            EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_OPENING);
            EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
        }
        else {
            m_hatch_control_fore->iterate_time_taken_by_action(m_dt);
        }
    }
    m_hatch_control_handler->execute(m_dt, false);

    msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_OPENING);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_OPEN_TIMEOUT);
}

TEST_F(NmeaHatchControlTests, T03f_command_hatch_from_open_to_50_timeout)
{
    go_to_hatch_state(1, WAGOHatchMessageStruct::HatchCommand::COMMAND_OPEN, false);
    EXPECT_EQ(m_hatch_control_fore->get_hatch_sensor_status(), HatchTypes::HatchSensorStates::DI_OPENED);

    m_hatch_control_fore->hatch_command_callback(
        enum_convert<HatchTypes::Command>(hatch_command::COMMAND_50));
    WagoHatchFbMessage msg{};
    for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
        if (i <= m_half_action_just_incomplete_time) {
            // Expect Hatch to be in undefined position while closing with no faults
            m_hatch_control_handler->execute(m_dt, false);
            msg = m_hatch_control_handler->get_hatch_feedback();
            auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
            EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
            EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_50_CLOSING);
            EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_NO_FAULT);
        }
        else {
            m_hatch_control_fore->iterate_time_taken_by_action(m_dt);
        }
    }
    m_hatch_control_handler->execute(m_dt, false);

    msg = m_hatch_control_handler->get_hatch_feedback();
    auto hatch_status = find_hatch_status(1, msg.data.hatch_statuses);
    EXPECT_EQ(hatch_status.hatch_state, WagoHatchFbMessageStruct::State::STATE_UNDEFINED);
    EXPECT_EQ(hatch_status.hatch_action, WagoHatchFbMessageStruct::Action::ACTION_50_CLOSING);
    EXPECT_EQ(hatch_status.hatch_fault, WagoHatchFbMessageStruct::Fault::FAULT_50_CLOSING_TIMEOUT);
}