
#include <chrono>
#include <cstdint>
#include <iostream>

#include "fmi_propulsion_drivers/cstrider/cstrider_spec_utilities.hpp"
#ifdef WIN32
#define NOMINMAX
#include <SDKDDKVer.h>
#endif

#include <cppfmu_cs.hpp>
#include <iostream>
#include <string>
#include <zeabuz/common/io/modbus_tcp_server.hpp>
#include <zeabuz/common/utilities/math.hpp>

#include "fmi_propulsion_drivers/cstrider/model_signals.hpp"

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
    cppfmu::FMIValueReference zeabuz_system_heartbeat_signal_address;
    cppfmu::FMIValueReference zeabuz_system_available_signal_address;
    cppfmu::FMIValueReference stb_propulsion_command_signal_address;
    cppfmu::FMIValueReference stb_steering_command_signal_address;
    cppfmu::FMIValueReference port_propulsion_command_signal_address;
    cppfmu::FMIValueReference propulsion_system_heartbeat_signal_address;
    cppfmu::FMIValueReference ready_for_zeabuz_system_signal_address;
    cppfmu::FMIValueReference stb_propulsion_ready_signal_address;
    cppfmu::FMIValueReference stb_propulsion_feedback_signal_address;
    cppfmu::FMIValueReference stb_steering_ready_signal_address;
    cppfmu::FMIValueReference stb_steering_feedback_signal_address;
    cppfmu::FMIValueReference port_propulsion_ready_signal_address;
    cppfmu::FMIValueReference port_propulsion_feedback_signal_address;
    cppfmu::FMIValueReference modbus_debug_enable;
    cppfmu::FMIValueReference start_listen_on_init;
};

struct OutputValueRef
{
    cppfmu::FMIValueReference stb_rpm_cmd;
    cppfmu::FMIValueReference port_rpm_cmd;
    cppfmu::FMIValueReference stb_angle_cmd;
    cppfmu::FMIValueReference port_angle_cmd;
    cppfmu::FMIValueReference zeabuz_system_available;
    cppfmu::FMIValueReference zeabuz_system_heartbeat;
    cppfmu::FMIValueReference propulsion_system_heartbeat;
    cppfmu::FMIValueReference ready_for_zeabuz_system;
};

struct InputValueRefs
{
    cppfmu::FMIValueReference stb_rpm_fb;
    cppfmu::FMIValueReference port_rpm_fb;
    cppfmu::FMIValueReference stb_angle_fb;
    cppfmu::FMIValueReference port_angle_fb;
    cppfmu::FMIValueReference stb_steering_ready;
    cppfmu::FMIValueReference stb_propulsion_ready;
    cppfmu::FMIValueReference port_propulsion_ready;
    cppfmu::FMIValueReference ready_for_zeabuz_system;
    cppfmu::FMIValueReference stop_propulsion_system_heartbeat_user_override;
};

struct ValueReferences
{
    ParametersValueRef parameters;
    OutputValueRef output;
    InputValueRefs input;
};

class FmucstriderPropulsionInterface : public cppfmu::SlaveInstance
{
   public:
    explicit FmucstriderPropulsionInterface(cppfmu::FMIString fmu_resource_location);
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
    ZeabuzSystemHeartbeatState m_zeabuz_system_heartbeat_state{};
    PropulsionSystemHeartbeatState m_propulsion_system_heartbeat_state{};
};