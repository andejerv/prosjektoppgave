#include <gtest/gtest.h>

#include <bitset>
#include <memory>
#include <thread>
#include <zeabuz/common/io/message_handler.hpp>
#include <zeabuz/common/io/modbus_tcp_client.hpp>
#include <zeabuz/common/io/udp_connection.hpp>

#include "fmi_hatch_control/hatch_control.hpp"
#include "fmi_hatch_control/modbus_tcp_protocol/hatch_control_handler.hpp"

using ModbusTcpProtocol::Mapping;
using ModbusTcpProtocol::Signals;
using HatchFore = ModbusTcpProtocol::Signals::HatchFore;
using HatchAft = ModbusTcpProtocol::Signals::HatchAft;
using namespace std::chrono_literals;

using namespace zeabuz::common::io::modbus_tcp;
class ModbusTcpbHatchControlTests : public ::testing::Test
{
   public:
    std::thread m_server_thread;
    std::shared_ptr<Server> m_server;
    std::shared_ptr<Client> m_client;
    std::shared_ptr<HatchControl> m_hatch_control_fore;
    std::shared_ptr<HatchControl> m_hatch_control_aft;
    std::unique_ptr<ModbusTcpProtocol::HatchControlHandler> m_hatch_control_handler;
    std::unordered_map<uint16_t, std::shared_ptr<HatchControl>> m_hatch_control_map;
    uint16_t m_hatch_command_timeout{5};
    double m_dt{0.05};
    double m_full_action_just_incomplete_time{};
    double m_half_action_just_incomplete_time{};
    uint16_t m_hatch_id_fore{1};
    uint16_t m_hatch_id_aft{2};

   public:
    ModbusTcpbHatchControlTests()
    {
        start_modbus_server();
        m_hatch_control_fore = std::make_shared<HatchControl>(m_hatch_command_timeout, 100.0);
        m_hatch_control_aft = std::make_shared<HatchControl>(m_hatch_command_timeout, 100.0);

        m_hatch_control_map.emplace(static_cast<uint16_t>(m_hatch_id_fore), m_hatch_control_fore);
        m_hatch_control_map.emplace(static_cast<uint16_t>(m_hatch_id_aft), m_hatch_control_aft);
        m_hatch_control_handler =
            std::make_unique<ModbusTcpProtocol::HatchControlHandler>(m_server, m_hatch_control_map);
        calculate_hatch_move_time();
    }
    ~ModbusTcpbHatchControlTests()
    {
        m_server->shutdown();
    }
    void start_modbus_server()
    {
        auto mapping = ModbusMapping{
            Mapping::COIL_ADDRESS_WRITE,    Mapping::COIL_WRITE_NUM_BITS,    Mapping::COIL_ADDRESS_READ,
            Mapping::COIL_READ_NUM_BITS,    Mapping::REGISTER_ADDRESS_WRITE, Mapping::REGISTER_WRITE_NUM_REGS,
            Mapping::REGISTER_ADDRESS_READ, Mapping::REGISTER_READ_NUM_REGS,
        };

        auto modbus_config = ServerConfiguration{};
        modbus_config.ip = "127.0.0.1";
        modbus_config.port = 43245;
        modbus_config.mapping = mapping;
        modbus_config.start_listen_on_init = true;
        modbus_config.debug_enable = false;
        m_server = std::make_shared<Server>(modbus_config);
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
        auto config_client = ClientConfiguration{};
        config_client.ip = "127.0.0.1";
        config_client.port = 43245;
        config_client.connect_on_init = true;
        config_client.wait_for_connection_on_init = true;
        m_client = std::make_unique<Client>(config_client);
    }
    void go_to_hatch_state(const bool emergency_stop_active = false, const bool auto_manual_mode = false)
    {
        for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
            m_hatch_control_handler->execute(m_dt, emergency_stop_active, auto_manual_mode);
        }
    }

    void force_timeout_state(const uint16_t hatch_id)
    {
        m_hatch_control_handler->execute(m_dt, false, false);
        for (double i = 0; i < m_hatch_command_timeout; i += m_dt) {
            m_hatch_control_map.at(hatch_id)->iterate_time_taken_by_action(m_dt);
        }
        m_hatch_control_handler->execute(m_dt, false, false);
    }

    void calculate_hatch_move_time()
    {
        m_full_action_just_incomplete_time = m_hatch_command_timeout - (1.0 + 4 * m_dt);
        m_half_action_just_incomplete_time = m_hatch_command_timeout / 2.0 - (1.0 + 4 * m_dt);
    }

    uint16_t get_command_value(const uint16_t command)
    {
        auto cmd = std::bitset<16>{};
        cmd.set(command);
        return static_cast<uint16_t>(cmd.to_ulong());
    }
};

TEST_F(ModbusTcpbHatchControlTests, T01_command_open)
{
    m_client->write_register(ModbusTcpProtocol::Mapping::REGISTER_ADDRESS_WRITE,
                             get_command_value(HatchFore::COMMAND_OPEN));
    go_to_hatch_state();
    auto hatch_status = m_hatch_control_fore->get_hatch_status();
    EXPECT_EQ(hatch_status.state, HatchTypes::State::STATE_OPEN);
    EXPECT_EQ(hatch_status.action, HatchTypes::Action::ACTION_IDLE);
    EXPECT_EQ(hatch_status.fault, HatchTypes::Fault::FAULT_NO_FAULT);
}

TEST_F(ModbusTcpbHatchControlTests, T02_command_stop)
{
    m_client->write_register(ModbusTcpProtocol::Mapping::REGISTER_ADDRESS_WRITE,
                             get_command_value(HatchFore::COMMAND_OPEN));
    m_hatch_control_handler->execute(m_dt, false, false);
    m_client->write_register(ModbusTcpProtocol::Mapping::REGISTER_ADDRESS_WRITE,
                             get_command_value(HatchFore::COMMAND_STOP));
    m_hatch_control_handler->execute(m_dt, false, false);
    auto hatch_status = m_hatch_control_fore->get_hatch_status();
    EXPECT_EQ(hatch_status.state, HatchTypes::State::STATE_UNDEFINED);
    EXPECT_EQ(hatch_status.action, HatchTypes::Action::ACTION_IDLE);
    EXPECT_EQ(hatch_status.fault, HatchTypes::Fault::FAULT_NO_FAULT);
}

TEST_F(ModbusTcpbHatchControlTests, T03_command_close)
{
    m_client->write_register(ModbusTcpProtocol::Mapping::REGISTER_ADDRESS_WRITE,
                             get_command_value(HatchFore::COMMAND_OPEN));
    go_to_hatch_state();
    m_client->write_register(ModbusTcpProtocol::Mapping::REGISTER_ADDRESS_WRITE,
                             get_command_value(HatchFore::COMMAND_CLOSE));
    go_to_hatch_state();
    auto hatch_status = m_hatch_control_fore->get_hatch_status();
    EXPECT_EQ(hatch_status.state, HatchTypes::State::STATE_CLOSED);
    EXPECT_EQ(hatch_status.action, HatchTypes::Action::ACTION_IDLE);
    EXPECT_EQ(hatch_status.fault, HatchTypes::Fault::FAULT_NO_FAULT);
}

TEST_F(ModbusTcpbHatchControlTests, T04_status_to_bits)
{
    using namespace ModbusTcpProtocol;
    auto status = HatchTypes::HatchStatuses{};
    status.auto_manual_control = true;
    auto status_fore = HatchTypes::HatchStatus{};
    auto status_aft = HatchTypes::HatchStatus{};

    status_fore.hatch_id = HatchControlHandler::HATCH_ID_FORE;
    status_fore.action = HatchTypes::Action::ACTION_OPENING;
    status_fore.state = HatchTypes::State::STATE_UNDEFINED;
    status_fore.fault = HatchTypes::Fault::FAULT_NO_FAULT;
    status.hatch_statuses.push_back(status_fore);

    status_aft.hatch_id = HatchControlHandler::HATCH_ID_AFT;
    status_aft.action = HatchTypes::Action::ACTION_IDLE;
    status_aft.state = HatchTypes::State::STATE_OPEN;
    status_aft.fault = HatchTypes::Fault::FAULT_GENERAL_FAULT;
    status.hatch_statuses.push_back(status_aft);

    auto value = status_to_bits(status);
    auto bits = std::bitset<16>{value};

    EXPECT_TRUE(bits.test(Signals::AUTO_MANUAL_CONTROL));
    EXPECT_TRUE(bits.test(HatchFore::ACTION_OPENING));
    EXPECT_FALSE(bits.test(HatchFore::ACTION_CLOSING));
    EXPECT_TRUE(bits.test(HatchFore::STATE_OPEN));
    EXPECT_TRUE(bits.test(HatchFore::STATE_CLOSE));
    EXPECT_FALSE(bits.test(HatchFore::FAULT_TIMEOUT));
    EXPECT_FALSE(bits.test(HatchFore::FAULT_GENERAL));

    EXPECT_FALSE(bits.test(HatchAft::ACTION_OPENING));
    EXPECT_FALSE(bits.test(HatchAft::ACTION_CLOSING));
    EXPECT_FALSE(bits.test(HatchAft::STATE_OPEN));
    EXPECT_TRUE(bits.test(HatchAft::STATE_CLOSE));
    EXPECT_FALSE(bits.test(HatchAft::FAULT_TIMEOUT));
    EXPECT_TRUE(bits.test(HatchAft::FAULT_GENERAL));
}

TEST_F(ModbusTcpbHatchControlTests, T04_bits_to_command)
{
    using namespace ModbusTcpProtocol;
    auto bits = std::bitset<16>{};
    bits.set(HatchFore::COMMAND_OPEN);
    auto command = value_to_command(static_cast<uint16_t>(bits.to_ulong()));
    EXPECT_EQ(command.hatch_id, 1);
    EXPECT_EQ(command.command, HatchTypes::Command::COMMAND_OPEN);

    auto bits2 = std::bitset<16>{};
    bits2.set(HatchAft::COMMAND_CLOSE);
    command = value_to_command(static_cast<uint16_t>(bits2.to_ulong()));
    EXPECT_EQ(command.hatch_id, 2);
    EXPECT_EQ(command.command, HatchTypes::Command::COMMAND_CLOSE);

    auto bits3 = std::bitset<16>{};
    bits3.set(HatchAft::COMMAND_STOP);
    command = value_to_command(static_cast<uint16_t>(bits3.to_ulong()));
    EXPECT_EQ(command.hatch_id, 2);
    EXPECT_EQ(command.command, HatchTypes::Command::COMMAND_STOP);
}
