#ifndef NMEA_HATCH_CONTROL_HANDLER_HPP
#define NMEA_HATCH_CONTROL_HANDLER_HPP

#include <atomic>
#include <memory>
#include <type_traits>
#include <zeabuz/common/io/message_handler.hpp>
#include <zeabuz/common/io/messages/custom/wago_hatch.hpp>
#include <zeabuz/common/io/messages/custom/wago_hatch_fb.hpp>

#include "fmi_hatch_control/hatch_control.hpp"

namespace NMEAProtocol
{
class HatchControlHandler
{
   public:
    enum class State : std::uint16_t { STATE_UNDEFINED = 0, STATE_OPEN = 1, STATE_CLOSED = 2, STATE_50 = 3 };

    enum class Action : std::uint16_t {
        ACTION_IDLE = 0,
        ACTION_OPENING = 1,     // Hatch moving to open state
        ACTION_CLOSING = 2,     // Hatch moving to closed state
        ACTION_50_CLOSING = 3,  // Hatch moving to 50% state from open state
        ACTION_50_OPENING = 4   // Hatch moving to 50% state from closed state
    };

    enum class Fault : std::uint16_t {
        FAULT_NO_FAULT = 0,            // No fault active
        FAULT_OPEN_TIMEOUT = 1,        // Hatch did not open in time
        FAULT_CLOSE_TIMEOUT = 2,       // Hatch did not close in time
        FAULT_50_CLOSING_TIMEOUT = 3,  // Hatch did not reach 50 % from open state in time
        FAULT_50_OPENING_TIMEOUT = 4   // Hatch did not reach 50 % from closed state in time
    };
    enum class HatchSensorStates : std::uint16_t { DI_OPENED = 0, DI_CLOSED = 1, DI_50 = 2, DI_EM_STOP = 3 };

    HatchControlHandler(std::shared_ptr<zeabuz::common::io::message_handlers::NMEA0183MessageHandler> message_handler,
                        std::unordered_map<uint16_t, std::shared_ptr<HatchControl>> hatch_controllers);

    void initialize_fmu();
    void hatch_command_cb(const zeabuz::common::io::messages::custom::WAGOHatchMessage& message);
    void execute(const double time_step, const bool emergency_stop_active);
    void send_hatch_feedback();
    zeabuz::common::io::messages::custom::WagoHatchFbMessage get_hatch_feedback() const;
    std::unordered_map<uint16_t, double> get_hatch_positions() const;

   private:
    std::atomic<bool> m_emergency_stop_state{false};
    std::shared_ptr<zeabuz::common::io::message_handlers::NMEA0183MessageHandler> m_message_handler;
    std::unordered_map<uint16_t, std::shared_ptr<HatchControl>> m_hatch_controllers;
};
}  // namespace NMEAProtocol
#endif