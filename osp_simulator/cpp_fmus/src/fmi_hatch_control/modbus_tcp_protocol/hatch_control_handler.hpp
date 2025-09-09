#ifndef MODBUS_TCP_HATCH_CONTROL_HANDLER_HPP
#define MODBUS_TCP_HATCH_CONTROL_HANDLER_HPP

#include <atomic>
#include <memory>
#include <zeabuz/common/io/modbus_tcp_server.hpp>

#include "fmi_hatch_control/hatch_control.hpp"

namespace ModbusTcpProtocol
{
struct Mapping
{
    static constexpr uint16_t COIL_ADDRESS_READ = 0;
    static constexpr uint16_t COIL_READ_NUM_BITS = 0;
    static constexpr uint16_t COIL_ADDRESS_WRITE = 0;
    static constexpr uint16_t COIL_WRITE_NUM_BITS = 0;
    static constexpr uint16_t REGISTER_ADDRESS_READ = 200;
    static constexpr uint16_t REGISTER_READ_NUM_REGS = 1;
    static constexpr uint16_t REGISTER_ADDRESS_WRITE = 32000;
    static constexpr uint16_t REGISTER_WRITE_NUM_REGS = 1;
};
struct Signals
{
    static constexpr uint16_t AUTO_MANUAL_CONTROL = 0;
    static constexpr uint16_t LIVE_HEARTBEAT = 1;
    struct HatchFore
    {
        static constexpr uint16_t HATCH_ID = 1;
        static constexpr uint16_t COMMAND_OPEN = 2;
        static constexpr uint16_t COMMAND_CLOSE = 3;
        static constexpr uint16_t COMMAND_STOP = 4;
        static constexpr uint16_t ACTION_OPENING = 1;
        static constexpr uint16_t ACTION_CLOSING = 2;
        static constexpr uint16_t STATE_OPEN = 3;
        static constexpr uint16_t STATE_CLOSE = 4;
        static constexpr uint16_t FAULT_TIMEOUT = 5;
        static constexpr uint16_t FAULT_GENERAL = 6;
        static constexpr uint16_t IS_STOPPED = 13;
    };
    struct HatchAft
    {
        static constexpr uint16_t HATCH_ID = 2;
        static constexpr uint16_t COMMAND_OPEN = 5;
        static constexpr uint16_t COMMAND_CLOSE = 6;
        static constexpr uint16_t COMMAND_STOP = 7;
        static constexpr uint16_t ACTION_OPENING = 7;
        static constexpr uint16_t ACTION_CLOSING = 8;
        static constexpr uint16_t STATE_OPEN = 9;
        static constexpr uint16_t STATE_CLOSE = 10;
        static constexpr uint16_t FAULT_TIMEOUT = 11;
        static constexpr uint16_t FAULT_GENERAL = 12;
        static constexpr uint16_t IS_STOPPED = 14;
    };
};

HatchTypes::HatchCommand value_to_command(const uint16_t value);
uint16_t status_to_bits(const HatchTypes::HatchStatuses& status);

class HatchControlHandler
{
   public:
    static constexpr uint16_t HATCH_ID_FORE = 1;
    static constexpr uint16_t HATCH_ID_AFT = 2;

    HatchControlHandler(std::shared_ptr<zeabuz::common::io::modbus_tcp::Server> m_modbus_server,
                        std::unordered_map<uint16_t, std::shared_ptr<HatchControl>> hatch_controllers);

    void execute(const double time_step, const bool emergency_stop_active, const bool auto_manual_mode);
    std::unordered_map<uint16_t, double> get_hatch_positions() const;

    HatchTypes::HatchStatuses get_hatch_feedback() const;

   private:
    void update_hatch_command();
    void update_modbus_server();
    void hatch_command_cb(const HatchTypes::HatchCommand& message);
    void send_hatch_feedback();

    std::shared_ptr<zeabuz::common::io::modbus_tcp::Server> m_modbus_server;
    std::unordered_map<uint16_t, std::shared_ptr<HatchControl>> m_hatch_controllers;
    std::mutex m_server_mutex{};
    HatchTypes::HatchStatuses m_hatch_statuses{};
    HatchTypes::HatchCommand m_prev_command{};
    std::atomic<bool> m_emergency_stop_state{false};
    std::atomic<bool> m_auto_manual_mode{false};
};
}  // namespace ModbusTcpProtocol
#endif