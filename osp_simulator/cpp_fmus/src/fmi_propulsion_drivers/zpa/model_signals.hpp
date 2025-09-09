#ifndef FMI_PROPULSION_DRIVERS_MODBUS_TCP_INTERFACE_FMU_SIGNALS_HPP
#define FMI_PROPULSION_DRIVERS_MODBUS_TCP_INTERFACE_FMU_SIGNALS_HPP

#include <cstdint>

struct ModelInputs
{
    struct ModbusTcpServerOutput
    {
        bool autonomy_available{};
        bool ready_for_autonomy_button{};
        uint16_t autonomy_system_heartbeat{};
        uint16_t rpm_cmd{};
        uint16_t angle_cmd{};
    };
    struct FmuInputs
    {
        bool steering_drive_ready{};
        bool steering_drive_running{};
        bool steering_drive_fault{};
        bool propulsion_drive_ready{};
        bool propulsion_drive_running{};
        bool propulsion_drive_fault{};
        double propeller_rpm{};
        double angle_rad{};

        // User provided overrides
        bool lcp_ready_for_autonomy_condition{};
        bool stop_lcp_heartbeat_user_override{};
    };

    struct ParameterInputs
    {
        double rpm_scaling_factor{1.0};
    };

    ModbusTcpServerOutput modbus_tcp_server{};
    FmuInputs fmu{};
    ParameterInputs parameters{};
};

struct ModelOutputs
{
    struct ModbusTcpServerInput
    {
        bool ready_for_autonomy{};
        bool ready_for_autonomy_button_available{};
        uint16_t lcp_heartbeat{};
        uint16_t propulsion_status{};
        uint16_t steering_status{};
        uint16_t rpm_fb{};
        uint16_t angle_fb{};
    };

    struct FmuOutputs
    {
        bool autonomy_available{};
        bool ready_for_autonomy_button{};
        bool ready_for_autonomy{};
        bool ready_for_autonomy_button_available{};
        double rpm_cmd{};
        double angle_cmd{};
        uint16_t autonomy_system_heartbeat{};
        uint16_t lcp_heartbeat{};
    };

    ModbusTcpServerInput modbus_tcp_server{};
    FmuOutputs fmu{};
};

#endif