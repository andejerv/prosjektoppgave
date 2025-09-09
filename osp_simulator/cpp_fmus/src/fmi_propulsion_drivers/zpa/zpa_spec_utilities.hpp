#ifndef FMI_PROPULSION_DRIVERS_MODBUS_TCP_INTERFACE_ZPA_SPEC_UTILITIES_HPP
#define FMI_PROPULSION_DRIVERS_MODBUS_TCP_INTERFACE_ZPA_SPEC_UTILITIES_HPP

#include <chrono>
#include <zeabuz/common/utilities/math.hpp>

#include "fmi_propulsion_drivers/zpa/model_signals.hpp"

inline uint16_t wrapped_rad_to_zpa_deg(const double angle_rad)
{
    const auto angle_wrapped_0_2pi = std::fmod(angle_rad + 2 * M_PI, 2 * M_PI);
    const auto angle_deg_in_range = std::round(zeabuz::common::utilities::math::to_deg(angle_wrapped_0_2pi) * 10);
    const auto angle_deg_fb = static_cast<uint16_t>(static_cast<int16_t>(angle_deg_in_range));
    return angle_deg_fb;
}

inline double zpa_deg_to_wrapped_rad(const uint16_t angle_deg)
{
    using zeabuz::common::utilities::math::inf2pipi;
    using zeabuz::common::utilities::math::to_rad;
    return inf2pipi(to_rad(static_cast<double>(static_cast<int16_t>(angle_deg)) / 10.0));
}

struct PreviousReadyForAutonomyState
{
    bool button_signal{};
    bool ready_for_autonomy{};
};

struct ReadyForAutonomySequenceResult
{
    bool ready_for_autonomy_allowed{};
    bool ready_for_autonomy_button_available_allowed{};
};

struct InternalFaultAutonomySequenceResult
{
    bool ready_for_autonomy_allowed{};
    bool ready_for_autonomy_button_available_allowed{};
};

struct LCPHeartbeatState
{
    double last_heartbeat_time{};
    uint16_t heartbeat_counter{};
};

struct AutonomySystemHeartbeatState
{
    double last_heartbeat_time{};
    uint16_t heartbeat_counter{};
};

struct AutonomyHeartbeatSequenceResult
{
    AutonomySystemHeartbeatState heartbeat_state{};
    bool ready_for_autonomy_allowed{};
    bool ready_for_autonomy_button_available_allowed{};
};

inline ReadyForAutonomySequenceResult ready_for_autonomy_sequence(const ModelInputs& inputs,
                                                                  const PreviousReadyForAutonomyState& previous_state)
{
    // Sequence for setting ready for autonomy
    // 1.  Autonomy system available (“autonomy_available” = True)
    // 2. Set the levers (AL-P and AL-S) in neutral position (angle and RPM) (Is controllable by the user through the
    // input signal lcp_ready_for_autonomy_condition (defaults to true), and simulates the levers being
    // in neutral position
    // 3. The LCP-P and LCP-S makes the “Ready for autonomy button” available by setting the
    //    “Ready for autonomy button available” signal to True.
    // 4. Press the “Ready for autonomy button” button on the Autonomy HMI
    // 5. The “Ready for autonomy button” signal is sent to LCP-P/LCP-S.
    // 6. LCP-P and LCP-S sets the “Ready for autonomy” flag to be read by the autonomy system.

    // ready_for_autonomy_button is a signal pulse. Check if rising edge of signal pulse is detected  that
    // signals button is pressed
    auto button_pressed = false;
    if (inputs.modbus_tcp_server.ready_for_autonomy_button == true && previous_state.button_signal == false) {
        button_pressed = true;
    }

    const bool autonomy_available = inputs.modbus_tcp_server.autonomy_available;
    const bool lcp_condition = inputs.fmu.lcp_ready_for_autonomy_condition;
    auto result = ReadyForAutonomySequenceResult{};
    if (autonomy_available == false) {
        result.ready_for_autonomy_button_available_allowed = false;
        result.ready_for_autonomy_allowed = false;
    }
    else if (lcp_condition == false) {
        result.ready_for_autonomy_button_available_allowed = false;
        result.ready_for_autonomy_allowed = false;
    }
    else if ((autonomy_available == true) && (lcp_condition == true)) {
        result.ready_for_autonomy_button_available_allowed = true;
        result.ready_for_autonomy_allowed = button_pressed ? true : previous_state.ready_for_autonomy;
    }
    else {
        // Safe guard, both to false
        result.ready_for_autonomy_button_available_allowed = false;
        result.ready_for_autonomy_allowed = false;
    }
    return result;
}

inline AutonomyHeartbeatSequenceResult autonomy_heartbeat_sequence(
    const ModelInputs& inputs, const AutonomySystemHeartbeatState& autonomy_heartbeat_state,
    const double currentCommunicationPoint, const std::chrono::milliseconds& heartbeat_timeout)
{
    // The autonomy system stops sending data to LCP-P and/or LCP-S
    // 1. A Watchdog in LCP-P and/or LCP-S detect that the autonomy system stops sending data (Heartbeat)
    // 2. “Ready for autonomy” flag is set to False (if it was True)
    // 3. The VMS HMI should alarm the operator
    // 4. RPM/angle commands is read from the levers (which are in the neutral position)

    auto heartbeat_state = autonomy_heartbeat_state;
    if (autonomy_heartbeat_state.heartbeat_counter != inputs.modbus_tcp_server.autonomy_system_heartbeat) {
        heartbeat_state.last_heartbeat_time = currentCommunicationPoint;
        heartbeat_state.heartbeat_counter = inputs.modbus_tcp_server.autonomy_system_heartbeat;
    }

    auto result = AutonomyHeartbeatSequenceResult{};
    result.heartbeat_state = heartbeat_state;
    if ((currentCommunicationPoint - heartbeat_state.last_heartbeat_time) >
        std::chrono::duration<double>(heartbeat_timeout).count()) {
        result.ready_for_autonomy_allowed = false;
        result.ready_for_autonomy_button_available_allowed = false;
        return result;
    }
    else {
        result.ready_for_autonomy_allowed = true;
        result.ready_for_autonomy_button_available_allowed = true;
        return result;
    }
}

inline ModelOutputs get_model_output_signals(const ModelInputs& inputs,
                                             const ReadyForAutonomySequenceResult& ready_for_autonomy_sequence_result,
                                             const AutonomyHeartbeatSequenceResult& autonomy_heartbeat_sequence_result,
                                             const LCPHeartbeatState& lcp_heartbeat)
{
    const auto ready_for_autonomy = ready_for_autonomy_sequence_result.ready_for_autonomy_allowed &&
                                    autonomy_heartbeat_sequence_result.ready_for_autonomy_allowed;
    const auto ready_for_autonomy_button_available =
        ready_for_autonomy_sequence_result.ready_for_autonomy_button_available_allowed &&
        autonomy_heartbeat_sequence_result.ready_for_autonomy_button_available_allowed;
    auto outputs = ModelOutputs{};
    outputs.fmu.autonomy_available = inputs.modbus_tcp_server.autonomy_available;
    outputs.fmu.ready_for_autonomy = ready_for_autonomy;
    outputs.fmu.ready_for_autonomy_button_available = ready_for_autonomy_button_available;
    outputs.fmu.ready_for_autonomy_button = inputs.modbus_tcp_server.ready_for_autonomy_button;
    outputs.fmu.autonomy_system_heartbeat = inputs.modbus_tcp_server.autonomy_system_heartbeat;
    outputs.fmu.lcp_heartbeat = lcp_heartbeat.heartbeat_counter;

    outputs.fmu.rpm_cmd = 0;
    outputs.fmu.angle_cmd = 0;
    if (ready_for_autonomy) {
        // Ready for autonomy is true, set commands to propulsion and steering
        auto rpm = static_cast<double>(static_cast<int16_t>(inputs.modbus_tcp_server.rpm_cmd));
        outputs.fmu.rpm_cmd = rpm / inputs.parameters.rpm_scaling_factor;
        const auto wrapped_angle = zpa_deg_to_wrapped_rad(inputs.modbus_tcp_server.angle_cmd);
        outputs.fmu.angle_cmd = wrapped_angle;
    }

    // Feedback signals
    const auto steering_ready = inputs.fmu.steering_drive_ready;
    const auto steering_running = inputs.fmu.steering_drive_running;
    const auto steering_fault = inputs.fmu.steering_drive_fault;
    const auto steering_status = steering_fault << 2 | steering_running << 1 | steering_ready;

    const auto propulsion_ready = inputs.fmu.propulsion_drive_ready;
    const auto propulsion_running = inputs.fmu.propulsion_drive_running;
    const auto propulsion_fault = inputs.fmu.propulsion_drive_fault;
    const auto propulsion_status = propulsion_fault << 2 | propulsion_running << 1 | propulsion_ready;

    const auto propeller_rpm = inputs.fmu.propeller_rpm;
    const auto rpm_double = std::round(propeller_rpm * inputs.parameters.rpm_scaling_factor);
    const auto rpm_unsigned = static_cast<uint16_t>(static_cast<uint16_t>(rpm_double));
    const auto angle_rad = inputs.fmu.angle_rad;
    const auto angle_deg_fb = wrapped_rad_to_zpa_deg(angle_rad);
    outputs.modbus_tcp_server.angle_fb = angle_deg_fb;
    outputs.modbus_tcp_server.rpm_fb = rpm_unsigned;
    outputs.modbus_tcp_server.steering_status = steering_status;
    outputs.modbus_tcp_server.propulsion_status = propulsion_status;
    outputs.modbus_tcp_server.ready_for_autonomy = ready_for_autonomy;
    outputs.modbus_tcp_server.ready_for_autonomy_button_available = ready_for_autonomy_button_available;

    return outputs;
}

inline LCPHeartbeatState update_lcp_heartbeat(const LCPHeartbeatState& state, const double current_time,
                                              const std::chrono::milliseconds& heartbeat_time_step,
                                              const uint16_t heartbeat_wraparound,
                                              const bool stop_lcp_heartbeat_user_override)
{
    auto heartbeat_state = state;
    const auto time_since_last_lcp_increment = current_time - heartbeat_state.last_heartbeat_time;
    if (time_since_last_lcp_increment >= std::chrono::duration<double>(heartbeat_time_step).count() &&
        stop_lcp_heartbeat_user_override == false) {
        auto new_lcp_heartbeat_counter = heartbeat_state.heartbeat_counter + 1;
        if (new_lcp_heartbeat_counter > heartbeat_wraparound) {
            new_lcp_heartbeat_counter = 1;
        }
        heartbeat_state.heartbeat_counter = new_lcp_heartbeat_counter;
        heartbeat_state.last_heartbeat_time = current_time;
    }
    return heartbeat_state;
}

#endif
