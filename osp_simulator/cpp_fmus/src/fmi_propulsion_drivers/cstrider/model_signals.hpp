#ifndef FMI_PROPULSION_DRIVERS_CSTRIDER_MODEL_SIGNALS_HPP
#define FMI_PROPULSION_DRIVERS_CSTRIDER_MODEL_SIGNALS_HPP

#include <cstdint>

struct ModelInputs
{
    struct ModbusTcpServerOutput
    {
        bool zeabuz_system_available{};
        uint16_t zeabuz_system_heartbeat{};
        uint16_t stb_percent_thrust_cmd{};
        uint16_t port_percent_thrust_cmd{};
        uint16_t stb_angle_cmd{};
    };
    struct FmuInputs
    {
        bool stb_steering_ready{};
        bool stb_propulsion_ready{};
        bool port_propulsion_ready{};
        double stb_propeller_rpm{};
        double port_propeller_rpm{};
        double stb_angle_rad{};
        double port_angle_rad{};

        // User provided overrides
        bool ready_for_zeabuz_system{};
        bool stop_propulsion_system_heartbeat_user_override{};
    };

    ModbusTcpServerOutput modbus_tcp_server{};
    FmuInputs fmu{};
};

struct ModelOutputs
{
    struct ModbusTcpServerInput
    {
        bool ready_for_zeabuz_system{};
        uint16_t propulsion_system_heartbeat{};
        uint16_t stb_propulsion_ready{};
        uint16_t port_propulsion_ready{};
        uint16_t stb_steering_ready{};
        uint16_t stb_thrust_percent_fb{};
        uint16_t port_thrust_percent_fb{};
        uint16_t stb_angle_fb{};
    };

    struct FmuOutputs
    {
        bool zeabuz_system_available{};
        bool ready_for_zeabuz_system{};
        double stb_rpm_cmd{};
        double port_rpm_cmd{};
        double stb_angle_cmd{};
        double port_angle_cmd{};
        uint16_t zeabuz_system_heartbeat{};
        uint16_t propulsion_system_heartbeat{};
    };

    ModbusTcpServerInput modbus_tcp_server{};
    FmuOutputs fmu{};
};

#endif