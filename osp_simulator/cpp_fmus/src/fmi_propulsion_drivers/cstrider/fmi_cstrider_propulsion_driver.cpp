
#include <chrono>
#include <cstdint>
#include <iostream>

#include "fmt/core.h"
#ifdef WIN32
#define NOMINMAX
#include <SDKDDKVer.h>
#endif

#include <cppfmu_cs.hpp>
#include <iostream>
#include <string>
#include <zeabuz/common/io/modbus_tcp_server.hpp>
#include <zeabuz/common/utilities/math.hpp>

#include "fmi_propulsion_drivers/cstrider/cstrider_spec_utilities.hpp"
#include "fmi_propulsion_drivers/cstrider/fmi_cstrider_propulsion_driver.hpp"
#include "fmi_propulsion_drivers/cstrider/model_signals.hpp"

using namespace zeabuz::common::io::modbus_tcp;

FmucstriderPropulsionInterface::FmucstriderPropulsionInterface(cppfmu::FMIString fmu_resource_location)
: m_resource_location(fmu_resource_location)
{
    spdlog::set_level(spdlog::level::info);
    initialize_value_references();
    m_IntegerSignals[m_ValueRefs.parameters.modbus_server_port] = 0;
    m_StringSignals[m_ValueRefs.parameters.modbus_server_ip] = "";
    m_IntegerSignals[m_ValueRefs.parameters.holding_register_base_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.num_holding_registers] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.zeabuz_system_heartbeat_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.zeabuz_system_available_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.stb_propulsion_command_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.stb_steering_command_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.port_propulsion_command_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.propulsion_system_heartbeat_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.ready_for_zeabuz_system_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.stb_propulsion_ready_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.stb_propulsion_feedback_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.stb_steering_ready_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.stb_steering_feedback_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.port_propulsion_ready_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.port_propulsion_feedback_signal_address] = 0;
    m_BooleanSignals[m_ValueRefs.parameters.modbus_debug_enable] = false;
    m_BooleanSignals[m_ValueRefs.parameters.start_listen_on_init] = true;

    m_RealSignals[m_ValueRefs.output.stb_rpm_cmd] = 0.0;
    m_RealSignals[m_ValueRefs.output.port_rpm_cmd] = 0.0;
    m_RealSignals[m_ValueRefs.output.stb_angle_cmd] = 0.0;
    m_RealSignals[m_ValueRefs.output.port_angle_cmd] = 0.0;
    m_BooleanSignals[m_ValueRefs.output.zeabuz_system_available] = false;
    m_IntegerSignals[m_ValueRefs.output.zeabuz_system_heartbeat] = 0;
    m_IntegerSignals[m_ValueRefs.output.propulsion_system_heartbeat] = false;
    m_BooleanSignals[m_ValueRefs.output.ready_for_zeabuz_system] = false;

    m_RealSignals[m_ValueRefs.input.stb_rpm_fb] = 0.0;
    m_RealSignals[m_ValueRefs.input.port_rpm_fb] = 0.0;
    m_RealSignals[m_ValueRefs.input.stb_angle_fb] = 0.0;
    m_RealSignals[m_ValueRefs.input.port_angle_fb] = 0.0;
    m_BooleanSignals[m_ValueRefs.input.stb_steering_ready] = true;
    m_BooleanSignals[m_ValueRefs.input.stb_propulsion_ready] = true;
    m_BooleanSignals[m_ValueRefs.input.port_propulsion_ready] = true;
    m_BooleanSignals[m_ValueRefs.input.ready_for_zeabuz_system] = true;
    m_BooleanSignals[m_ValueRefs.input.stop_propulsion_system_heartbeat_user_override] = false;
}

void FmucstriderPropulsionInterface::initialize_value_references()
{
    // parameters
    m_ValueRefs.parameters.modbus_server_port = 1;
    m_ValueRefs.parameters.modbus_server_ip = 2;
    m_ValueRefs.parameters.holding_register_base_address = 3;
    m_ValueRefs.parameters.num_holding_registers = 4;
    m_ValueRefs.parameters.zeabuz_system_heartbeat_signal_address = 5;
    m_ValueRefs.parameters.zeabuz_system_available_signal_address = 6;
    m_ValueRefs.parameters.stb_propulsion_command_signal_address = 7;
    m_ValueRefs.parameters.stb_steering_command_signal_address = 8;
    m_ValueRefs.parameters.port_propulsion_command_signal_address = 9;
    m_ValueRefs.parameters.propulsion_system_heartbeat_signal_address = 10;
    m_ValueRefs.parameters.ready_for_zeabuz_system_signal_address = 11;
    m_ValueRefs.parameters.stb_propulsion_ready_signal_address = 12;
    m_ValueRefs.parameters.stb_propulsion_feedback_signal_address = 13;
    m_ValueRefs.parameters.stb_steering_ready_signal_address = 14;
    m_ValueRefs.parameters.stb_steering_feedback_signal_address = 15;
    m_ValueRefs.parameters.port_propulsion_ready_signal_address = 16;
    m_ValueRefs.parameters.port_propulsion_feedback_signal_address = 17;
    m_ValueRefs.parameters.modbus_debug_enable = 18;
    m_ValueRefs.parameters.start_listen_on_init = 19;

    // output
    m_ValueRefs.output.stb_rpm_cmd = 20;
    m_ValueRefs.output.port_rpm_cmd = 21;
    m_ValueRefs.output.stb_angle_cmd = 22;
    m_ValueRefs.output.port_angle_cmd = 23;
    m_ValueRefs.output.zeabuz_system_available = 24;
    m_ValueRefs.output.zeabuz_system_heartbeat = 25;
    m_ValueRefs.output.propulsion_system_heartbeat = 26;
    m_ValueRefs.output.ready_for_zeabuz_system = 27;

    // Input
    m_ValueRefs.input.stb_rpm_fb = 30;
    m_ValueRefs.input.port_rpm_fb = 31;
    m_ValueRefs.input.stb_angle_fb = 32;
    m_ValueRefs.input.port_angle_fb = 33;
    m_ValueRefs.input.stb_steering_ready = 34;
    m_ValueRefs.input.stb_propulsion_ready = 35;
    m_ValueRefs.input.port_propulsion_ready = 36;
    m_ValueRefs.input.ready_for_zeabuz_system = 37;
    m_ValueRefs.input.stop_propulsion_system_heartbeat_user_override = 38;
}

void FmucstriderPropulsionInterface::ExitInitializationMode()
{
    auto modbus_mapping =
        ModbusMapping{0,
                      0,
                      0,
                      0,
                      static_cast<uint16_t>(m_IntegerSignals[m_ValueRefs.parameters.holding_register_base_address]),
                      static_cast<uint16_t>(m_IntegerSignals[m_ValueRefs.parameters.num_holding_registers]),
                      0,
                      0};

    auto modbus_config = ServerConfiguration{};
    modbus_config.ip = m_StringSignals[m_ValueRefs.parameters.modbus_server_ip];
    modbus_config.port = static_cast<uint16_t>(m_IntegerSignals[m_ValueRefs.parameters.modbus_server_port]);
    modbus_config.mapping = modbus_mapping;
    modbus_config.debug_enable = m_BooleanSignals[m_ValueRefs.parameters.modbus_debug_enable];
    modbus_config.start_listen_on_init = m_BooleanSignals[m_ValueRefs.parameters.start_listen_on_init];
    m_modbus_server = std::make_shared<Server>(modbus_config);
}

ModelInputs FmucstriderPropulsionInterface::get_model_inputs()
{
    const auto base_address_holding_registers = m_IntegerSignals[m_ValueRefs.parameters.holding_register_base_address];
    // Command Signals
    ModelInputs inputs{};

    inputs.modbus_tcp_server.zeabuz_system_heartbeat = m_modbus_server->get_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.zeabuz_system_heartbeat_signal_address],
                 base_address_holding_registers));

    inputs.modbus_tcp_server.zeabuz_system_available = m_modbus_server->get_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.zeabuz_system_available_signal_address],
                 base_address_holding_registers));

    inputs.modbus_tcp_server.stb_percent_thrust_cmd = m_modbus_server->get_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.stb_propulsion_command_signal_address],
                 base_address_holding_registers));
    inputs.modbus_tcp_server.port_percent_thrust_cmd = m_modbus_server->get_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.port_propulsion_command_signal_address],
                 base_address_holding_registers));
    inputs.modbus_tcp_server.stb_angle_cmd = m_modbus_server->get_register(get_addr(
        m_IntegerSignals[m_ValueRefs.parameters.stb_steering_command_signal_address], base_address_holding_registers));

    inputs.fmu.stb_steering_ready = m_BooleanSignals[m_ValueRefs.input.stb_steering_ready];
    inputs.fmu.stb_propulsion_ready = m_BooleanSignals[m_ValueRefs.input.stb_propulsion_ready];
    inputs.fmu.port_propulsion_ready = m_BooleanSignals[m_ValueRefs.input.port_propulsion_ready];
    inputs.fmu.stb_propeller_rpm = m_RealSignals[m_ValueRefs.input.stb_rpm_fb];
    inputs.fmu.port_propeller_rpm = m_RealSignals[m_ValueRefs.input.port_rpm_fb];
    inputs.fmu.stb_angle_rad = m_RealSignals[m_ValueRefs.input.stb_angle_fb];
    inputs.fmu.port_angle_rad = m_RealSignals[m_ValueRefs.input.port_angle_fb];

    inputs.fmu.ready_for_zeabuz_system = m_BooleanSignals[m_ValueRefs.input.ready_for_zeabuz_system];
    inputs.fmu.stop_propulsion_system_heartbeat_user_override =
        m_BooleanSignals[m_ValueRefs.input.stop_propulsion_system_heartbeat_user_override];

    return inputs;
}

void FmucstriderPropulsionInterface::set_model_output_signals(const ModelOutputs& outputs)
{
    const auto base_address_holding_registers = m_IntegerSignals[m_ValueRefs.parameters.holding_register_base_address];
    m_modbus_server->set_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.propulsion_system_heartbeat_signal_address],
                 base_address_holding_registers),
        m_propulsion_system_heartbeat_state.heartbeat_counter);
    m_modbus_server->set_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.stb_propulsion_feedback_signal_address],
                 base_address_holding_registers),
        outputs.modbus_tcp_server.stb_thrust_percent_fb);
    m_modbus_server->set_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.port_propulsion_feedback_signal_address],
                 base_address_holding_registers),
        outputs.modbus_tcp_server.port_thrust_percent_fb);
    m_modbus_server->set_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.stb_steering_feedback_signal_address],
                 base_address_holding_registers),
        outputs.modbus_tcp_server.stb_angle_fb);
    m_modbus_server->set_register(get_addr(m_IntegerSignals[m_ValueRefs.parameters.stb_steering_ready_signal_address],
                                           base_address_holding_registers),
                                  outputs.modbus_tcp_server.stb_steering_ready);
    m_modbus_server->set_register(get_addr(m_IntegerSignals[m_ValueRefs.parameters.stb_propulsion_ready_signal_address],
                                           base_address_holding_registers),
                                  outputs.modbus_tcp_server.stb_propulsion_ready);
    m_modbus_server->set_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.port_propulsion_ready_signal_address],
                 base_address_holding_registers),
        outputs.modbus_tcp_server.port_propulsion_ready);
    m_modbus_server->set_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.ready_for_zeabuz_system_signal_address],
                 base_address_holding_registers),
        outputs.modbus_tcp_server.ready_for_zeabuz_system);

    m_IntegerSignals[m_ValueRefs.output.propulsion_system_heartbeat] =
        m_propulsion_system_heartbeat_state.heartbeat_counter;
    m_IntegerSignals[m_ValueRefs.output.zeabuz_system_heartbeat] = outputs.fmu.zeabuz_system_heartbeat;
    m_BooleanSignals[m_ValueRefs.output.zeabuz_system_available] = outputs.fmu.zeabuz_system_available;
    m_RealSignals[m_ValueRefs.output.stb_rpm_cmd] = outputs.fmu.stb_rpm_cmd;
    m_RealSignals[m_ValueRefs.output.port_rpm_cmd] = outputs.fmu.port_rpm_cmd;
    m_RealSignals[m_ValueRefs.output.stb_angle_cmd] = outputs.fmu.stb_angle_cmd;
    m_RealSignals[m_ValueRefs.output.port_angle_cmd] = outputs.fmu.port_angle_cmd;
    m_BooleanSignals[m_ValueRefs.output.ready_for_zeabuz_system] = outputs.fmu.ready_for_zeabuz_system;
}

bool FmucstriderPropulsionInterface::DoStep(cppfmu::FMIReal currentCommunicationPoint, cppfmu::FMIReal dt,
                                            cppfmu::FMIBoolean /*newStep*/, cppfmu::FMIReal& /*endOfStep*/)
{
    using zeabuz::common::utilities::math::inf2pipi;
    using zeabuz::common::utilities::math::to_deg;
    using zeabuz::common::utilities::math::to_rad;

    const auto inputs = get_model_inputs();

    const auto zeabuz_system_heartbeat_sequence_result =
        zeabuz_system_heartbeat_sequence(inputs, m_zeabuz_system_heartbeat_state, currentCommunicationPoint,
                                         Parameters::ZEABUZ_SYSTEM_HEARTBEAT_TIMEOUT);

    // Update zeabuz system heartbeat signal
    m_zeabuz_system_heartbeat_state = zeabuz_system_heartbeat_sequence_result.heartbeat_state;
    // Handle propulsion system heartbeat signal
    m_propulsion_system_heartbeat_state = update_propulsion_system_heartbeat(
        m_propulsion_system_heartbeat_state, currentCommunicationPoint,
        Parameters::PROPULSION_SYSTEM_HEARTBEAT_INCREMENT_TIMESTEP, Parameters::HEARTBEAT_WRAPAROUND,
        inputs.fmu.stop_propulsion_system_heartbeat_user_override);

    const auto outputs =
        get_model_output_signals(inputs, zeabuz_system_heartbeat_sequence_result, m_propulsion_system_heartbeat_state);
    set_model_output_signals(outputs);
    return true;
}

void FmucstriderPropulsionInterface::SetString(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                               const cppfmu::FMIString value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        m_StringSignals[vr[i]] = value[i];
    }
}

void FmucstriderPropulsionInterface::SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                             const cppfmu::FMIReal value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        m_RealSignals[vr[i]] = value[i];
    }
}

void FmucstriderPropulsionInterface::SetInteger(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                                const cppfmu::FMIInteger value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        m_IntegerSignals[vr[i]] = value[i];
    }
}

void FmucstriderPropulsionInterface::SetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                                const cppfmu::FMIBoolean value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        m_BooleanSignals[vr[i]] = value[i];
    }
}

void FmucstriderPropulsionInterface::GetString(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                               cppfmu::FMIString value[]) const
{
    for (std::size_t i = 0; i < nvr; ++i) {
        value[i] = m_StringSignals.at(vr[i]).c_str();
    }
}

void FmucstriderPropulsionInterface::GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                             cppfmu::FMIReal value[]) const
{
    for (std::size_t i = 0; i < nvr; ++i) {
        value[i] = m_RealSignals.at(vr[i]);
    }
}

void FmucstriderPropulsionInterface::GetInteger(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                                cppfmu::FMIInteger value[]) const
{
    for (std::size_t i = 0; i < nvr; ++i) {
        value[i] = m_IntegerSignals.at(vr[i]);
    }
}

void FmucstriderPropulsionInterface::GetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                                cppfmu::FMIBoolean value[]) const
{
    for (std::size_t i = 0; i < nvr; ++i) {
        value[i] = m_BooleanSignals.at(vr[i]);
    }
}

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString /*instanceName*/, cppfmu::FMIString /*fmuGUID*/, cppfmu::FMIString fmu_resource_location,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger /*logger*/)
{
    // if (std::strcmp(fmuGUID, FMU_UUID) != 0) {
    //     throw std::runtime_error("FMU GUID mismatch");
    // }
    return cppfmu::AllocateUnique<FmucstriderPropulsionInterface>(memory, fmu_resource_location);
}
