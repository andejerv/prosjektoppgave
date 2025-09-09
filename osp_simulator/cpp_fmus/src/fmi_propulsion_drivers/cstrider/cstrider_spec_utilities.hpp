#ifndef CSTRIDER_CSTRIDER_SPEC_UTILITIES_HPP
#define CSTRIDER_CSTRIDER_SPEC_UTILITIES_HPP

#include <chrono>
#include <zeabuz/common/utilities/math.hpp>

#include "fmi_propulsion_drivers/cstrider/model_signals.hpp"

struct Parameters
{
    static constexpr uint16_t HEARTBEAT_WRAPAROUND = 60;
    static constexpr std::chrono::milliseconds PROPULSION_SYSTEM_HEARTBEAT_INCREMENT_TIMESTEP{1000};
    static constexpr std::chrono::milliseconds ZEABUZ_SYSTEM_HEARTBEAT_TIMEOUT{2000};

    struct PropulsionRange
    {
        static constexpr int16_t MIN_STEERING_ANGLE = 0;
        static constexpr int16_t MAX_STEERING_ANGLE = 3599;
        static constexpr int16_t MIN_PROPULSION_PERCENT = -1000;
        static constexpr int16_t MAX_PROPULSION_PERCENT = 1000;
    };

    struct PropulsorInfo
    {
        // https://gateway.epropulsion.uk/api/download.php?api_key=f915b2c8-2962-49b5-89d1-09528e265295&file=docs/manuals/Navy-6.0-Evo2024-User-Manual-EN_DE-V1.0-20230911.pdf
        static constexpr double MAX_POWER = 6e3;
        static constexpr double MAX_RPM = 1700.0;
    };
};

inline uint16_t wrapped_rad_to_cstrider_deg(const double angle_rad)
{
    const auto angle_wrapped_0_2pi = std::fmod(angle_rad + 2 * M_PI, 2 * M_PI);
    const auto angle_deg_in_range = std::round(zeabuz::common::utilities::math::to_deg(angle_wrapped_0_2pi) * 10);
    const auto angle_deg_fb = static_cast<uint16_t>(static_cast<int16_t>(angle_deg_in_range));
    return angle_deg_fb;
}

inline double cstrider_deg_to_wrapped_rad(const uint16_t angle_deg)
{
    using zeabuz::common::utilities::math::inf2pipi;
    using zeabuz::common::utilities::math::to_rad;
    return inf2pipi(to_rad(static_cast<double>(static_cast<int16_t>(angle_deg)) / 10.0));
}

inline double propulsion_command_to_propeller_rpm(const uint16_t propulsion_command)
{
    constexpr auto scaling_factor = 10.0;  // Scale factor to convert from modbus representation of percent
    auto thrust_percent = static_cast<double>(static_cast<int16_t>(propulsion_command)) / scaling_factor;

    constexpr auto exponent_coefficient = 3.0;
    auto thrust_power = (thrust_percent / 100.0) * Parameters::PropulsorInfo::MAX_POWER;
    auto power_coefficient =
        Parameters::PropulsorInfo::MAX_POWER / std::pow(Parameters::PropulsorInfo::MAX_RPM, exponent_coefficient);
    auto propeller_rpm = std::pow(std::abs(thrust_power) / power_coefficient, 1.0 / exponent_coefficient);
    if (thrust_power < 0.0) {
        return -propeller_rpm;
    }
    return propeller_rpm;
}

inline uint16_t propeller_rpm_to_propulsion_feedback(const double propeller_rpm)
{
    constexpr auto exponent_coefficient = 3.0;
    auto power_coefficient =
        Parameters::PropulsorInfo::MAX_POWER / std::pow(Parameters::PropulsorInfo::MAX_RPM, exponent_coefficient);
    auto power = power_coefficient * std::pow(propeller_rpm, exponent_coefficient);
    auto power_percent = (power / Parameters::PropulsorInfo::MAX_POWER) * 100.0;

    constexpr auto scaling_factor = 10.0;  // Scale factor to convert to modbus representation of percent
    auto thrust_scaled = static_cast<int16_t>(power_percent * scaling_factor);
    const auto thrust_clamped = std::clamp(thrust_scaled, Parameters::PropulsionRange::MIN_PROPULSION_PERCENT,
                                           Parameters::PropulsionRange::MAX_PROPULSION_PERCENT);
    const auto propulsion_feedback = static_cast<uint16_t>(thrust_clamped);

    return propulsion_feedback;
}

struct PropulsionSystemHeartbeatState
{
    double last_heartbeat_time{};
    uint16_t heartbeat_counter{};
};

struct ZeabuzSystemHeartbeatState
{
    double last_heartbeat_time{};
    uint16_t heartbeat_counter{};
};

struct ZeabuzSystemHeartbeatSequenceResult
{
    ZeabuzSystemHeartbeatState heartbeat_state{};
    bool ready_for_zeabuz_system_allowed{};
};

inline ZeabuzSystemHeartbeatSequenceResult zeabuz_system_heartbeat_sequence(
    const ModelInputs& inputs, const ZeabuzSystemHeartbeatState& zeabuz_system_heartbeat_state,
    const double currentCommunicationPoint, const std::chrono::milliseconds& heartbeat_timeout)
{
    // The autonomy system stops sending data
    // 1. A Watchdog detects that the zeabuz system stops sending data (Heartbeat)
    // 2. “Ready for Zeabuz system” flag is set to False (if it was True)
    // 3. The VMS HMI should alarm the operator
    // 4. RPM/angle commands is read from the levers (which are in the neutral position)

    auto heartbeat_state = zeabuz_system_heartbeat_state;
    if (zeabuz_system_heartbeat_state.heartbeat_counter != inputs.modbus_tcp_server.zeabuz_system_heartbeat) {
        heartbeat_state.last_heartbeat_time = currentCommunicationPoint;
        heartbeat_state.heartbeat_counter = inputs.modbus_tcp_server.zeabuz_system_heartbeat;
    }

    auto result = ZeabuzSystemHeartbeatSequenceResult{};
    result.heartbeat_state = heartbeat_state;
    if ((currentCommunicationPoint - heartbeat_state.last_heartbeat_time) >
        std::chrono::duration<double>(heartbeat_timeout).count()) {
        result.ready_for_zeabuz_system_allowed = false;
        return result;
    }
    else {
        result.ready_for_zeabuz_system_allowed = true;
        return result;
    }
}

inline ModelOutputs get_model_output_signals(
    const ModelInputs& inputs, const ZeabuzSystemHeartbeatSequenceResult& zeabuz_system_heartbeat_sequence_result,
    const PropulsionSystemHeartbeatState& propulsion_system_heartbeat)
{
    auto outputs = ModelOutputs{};
    auto ready_for_zeabuz_system = inputs.modbus_tcp_server.zeabuz_system_available &&
                                   inputs.fmu.ready_for_zeabuz_system &&
                                   zeabuz_system_heartbeat_sequence_result.ready_for_zeabuz_system_allowed;

    outputs.fmu.ready_for_zeabuz_system = ready_for_zeabuz_system;
    outputs.fmu.zeabuz_system_available = inputs.modbus_tcp_server.zeabuz_system_available;
    outputs.fmu.zeabuz_system_heartbeat = inputs.modbus_tcp_server.zeabuz_system_heartbeat;
    outputs.fmu.propulsion_system_heartbeat = propulsion_system_heartbeat.heartbeat_counter;

    outputs.fmu.stb_rpm_cmd = 0;
    outputs.fmu.port_rpm_cmd = 0;
    outputs.fmu.stb_angle_cmd = 0;

    if (ready_for_zeabuz_system) {
        // Ready for Zeabuz system is true, set commands to propulsion and steering
        outputs.fmu.stb_rpm_cmd = propulsion_command_to_propeller_rpm(inputs.modbus_tcp_server.stb_percent_thrust_cmd);
        outputs.fmu.port_rpm_cmd =
            propulsion_command_to_propeller_rpm(inputs.modbus_tcp_server.port_percent_thrust_cmd);
        const auto wrapped_angle = cstrider_deg_to_wrapped_rad(inputs.modbus_tcp_server.stb_angle_cmd);
        outputs.fmu.stb_angle_cmd = wrapped_angle;
        outputs.fmu.port_angle_cmd = 0.0;
    }

    // Feedback signals
    const auto stb_steering_ready = inputs.fmu.stb_steering_ready;

    const auto stb_propulsion_ready = inputs.fmu.stb_propulsion_ready;
    const auto port_propulsion_ready = inputs.fmu.port_propulsion_ready;

    const auto stb_thrust_percent = propeller_rpm_to_propulsion_feedback(inputs.fmu.stb_propeller_rpm);
    const auto port_thrust_percent = propeller_rpm_to_propulsion_feedback(inputs.fmu.port_propeller_rpm);
    const auto stb_angle_rad = inputs.fmu.stb_angle_rad;
    const auto stb_angle_deg_fb = wrapped_rad_to_cstrider_deg(stb_angle_rad);
    outputs.modbus_tcp_server.stb_angle_fb = stb_angle_deg_fb;
    outputs.modbus_tcp_server.stb_thrust_percent_fb = stb_thrust_percent;
    outputs.modbus_tcp_server.port_thrust_percent_fb = port_thrust_percent;
    outputs.modbus_tcp_server.stb_steering_ready = stb_steering_ready;
    outputs.modbus_tcp_server.stb_propulsion_ready = stb_propulsion_ready;
    outputs.modbus_tcp_server.port_propulsion_ready = port_propulsion_ready;
    outputs.modbus_tcp_server.ready_for_zeabuz_system = ready_for_zeabuz_system;

    return outputs;
}

inline PropulsionSystemHeartbeatState update_propulsion_system_heartbeat(
    const PropulsionSystemHeartbeatState& state, const double current_time,
    const std::chrono::milliseconds& heartbeat_time_step, const uint16_t heartbeat_wraparound,
    const bool stop_lcp_heartbeat_user_override)
{
    auto heartbeat_state = state;
    const auto time_since_last_heartbeat_increment = current_time - heartbeat_state.last_heartbeat_time;
    if (time_since_last_heartbeat_increment >= std::chrono::duration<double>(heartbeat_time_step).count() &&
        stop_lcp_heartbeat_user_override == false) {
        auto new_heartbeat_counter = heartbeat_state.heartbeat_counter + 1;
        if (new_heartbeat_counter >= heartbeat_wraparound) {
            new_heartbeat_counter = 0;
        }
        heartbeat_state.heartbeat_counter = new_heartbeat_counter;
        heartbeat_state.last_heartbeat_time = current_time;
    }
    return heartbeat_state;
}

#endif /* CSTRIDER_CSTRIDER_SPEC_UTILITIES_HPP */
