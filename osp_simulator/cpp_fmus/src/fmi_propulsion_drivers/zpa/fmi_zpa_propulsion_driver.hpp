
#include <chrono>
#include <cstdint>
#include <iostream>

#include "fmi_propulsion_drivers/zpa/zpa_spec_utilities.hpp"
#ifdef WIN32
#define NOMINMAX
#include <SDKDDKVer.h>
#endif

#include <cppfmu_cs.hpp>
#include <iostream>
#include <string>
#include <zeabuz/common/io/modbus_tcp_server.hpp>
#include <zeabuz/common/utilities/math.hpp>

#include "fmi_propulsion_drivers/zpa/model_signals.hpp"

struct Parameters
{
    static constexpr uint16_t HEARTBEAT_WRAPAROUND = 60;
    static constexpr std::chrono::milliseconds LCP_HEARTBEAT_INCREMENT_TIMESTEP{1000};
    static constexpr std::chrono::milliseconds AUTONOMY_HEARTBEAT_TIMEOUT{2000};
};

constexpr int16_t get_addr(const uint16_t register_addr, const uint16_t base_addr)
{
    return register_addr - base_addr;
}

struct ParametersValueRef
{
    cppfmu::FMIValueReference modbus_server_port;
    cppfmu::FMIValueReference modbus_server_ip;
    cppfmu::FMIValueReference holding_register_base_address;
    cppfmu::FMIValueReference num_holding_registers;
    cppfmu::FMIValueReference autonomy_system_heartbeat_signal_address;
    cppfmu::FMIValueReference autonomy_available_signal_address;
    cppfmu::FMIValueReference propulsion_rpm_command_signal_address;
    cppfmu::FMIValueReference steering_degree_command_signal_address;
    cppfmu::FMIValueReference ready_for_autonomy_button_signal_address;
    cppfmu::FMIValueReference lcp_heartbeat_signal_address;
    cppfmu::FMIValueReference ready_for_autonomy_signal_address;
    cppfmu::FMIValueReference propulsion_drive_status_signal_address;
    cppfmu::FMIValueReference steering_drive_status_signal_address;
    cppfmu::FMIValueReference propulsion_rpm_feedback_signal_address;
    cppfmu::FMIValueReference steering_degree_feedback_signal_address;
    cppfmu::FMIValueReference ready_for_autonomy_button_available_signal_address;
    cppfmu::FMIValueReference modbus_debug_enable;
    cppfmu::FMIValueReference start_listen_on_init;
    cppfmu::FMIValueReference rpm_scaling_factor;
};

struct OutputValueRef
{
    cppfmu::FMIValueReference ready_for_autonomy;
    cppfmu::FMIValueReference lcp_heartbeat;
    cppfmu::FMIValueReference autonomy_system_heartbeat;
    cppfmu::FMIValueReference autonomy_available;
    cppfmu::FMIValueReference rpm_cmd;
    cppfmu::FMIValueReference angle_cmd;
    cppfmu::FMIValueReference ready_for_autonomy_button;
    cppfmu::FMIValueReference ready_for_autonomy_button_available;
};

struct InputValueRefs
{
    cppfmu::FMIValueReference propulsion_drive_ready;
    cppfmu::FMIValueReference propulsion_drive_running;
    cppfmu::FMIValueReference propulsion_drive_fault;

    cppfmu::FMIValueReference steering_drive_ready;
    cppfmu::FMIValueReference steering_drive_running;
    cppfmu::FMIValueReference steering_drive_fault;

    cppfmu::FMIValueReference rpm_fb;
    cppfmu::FMIValueReference angle_fb;

    cppfmu::FMIValueReference lcp_ready_for_autonomy_condition;
    cppfmu::FMIValueReference stop_lcp_heartbeat_user_override;
};

struct ValueReferences
{
    ParametersValueRef parameters;
    OutputValueRef output;
    InputValueRefs input;
};

class FmuZpaPropulsionInterface : public cppfmu::SlaveInstance
{
   public:
    explicit FmuZpaPropulsionInterface(cppfmu::FMIString fmu_resource_location);
    void initialize_value_references();
    void ExitInitializationMode() override;

    ModelInputs get_model_inputs();
    void set_model_output_signals(const ModelOutputs& outputs);
    bool DoStep(cppfmu::FMIReal currentCommunicationPoint, cppfmu::FMIReal dt, cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal& /*endOfStep*/) override;

    void SetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIString value[]) override;
    void SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[]) override;
    void SetInteger(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIInteger value[]) override;
    void SetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIBoolean value[]) override;
    void GetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIString value[]) const override;
    void GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const override;
    void GetInteger(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIInteger value[]) const override;
    void GetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIBoolean value[]) const override;

   private:
    std::shared_ptr<zeabuz::common::io::modbus_tcp::Server> m_modbus_server;
    std::string m_resource_location;

    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIReal> m_RealSignals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIInteger> m_IntegerSignals;
    std::unordered_map<cppfmu::FMIValueReference, std::string> m_StringSignals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIBoolean> m_BooleanSignals;

    ValueReferences m_ValueRefs{};
    AutonomySystemHeartbeatState m_autonomy_heartbeat_state{};
    LCPHeartbeatState m_lcp_heartbeat_state{};
    PreviousReadyForAutonomyState m_previous_ready_for_autonomy_state{};
};