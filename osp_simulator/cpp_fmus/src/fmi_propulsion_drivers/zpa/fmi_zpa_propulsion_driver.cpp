
#include <chrono>
#include <cstdint>
#include <iostream>

#include "fmt/core.h"
#include "spdlog/spdlog.h"
#ifdef WIN32
#define NOMINMAX
#include <SDKDDKVer.h>
#endif

#include <cppfmu_cs.hpp>
#include <iostream>
#include <string>
#include <zeabuz/common/io/modbus_tcp_server.hpp>
#include <zeabuz/common/utilities/math.hpp>

#include "fmi_propulsion_drivers/zpa/fmi_zpa_propulsion_driver.hpp"
#include "fmi_propulsion_drivers/zpa/model_signals.hpp"
#include "fmi_propulsion_drivers/zpa/zpa_spec_utilities.hpp"

using namespace zeabuz::common::io::modbus_tcp;

FmuZpaPropulsionInterface::FmuZpaPropulsionInterface(cppfmu::FMIString fmu_resource_location)
: m_resource_location(fmu_resource_location)
{
    spdlog::set_level(spdlog::level::info);
    initialize_value_references();
    m_StringSignals[m_ValueRefs.parameters.modbus_server_ip] = "";
    m_IntegerSignals[m_ValueRefs.parameters.modbus_server_port] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.holding_register_base_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.num_holding_registers] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.autonomy_system_heartbeat_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.autonomy_available_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.propulsion_rpm_command_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.steering_degree_command_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.ready_for_autonomy_button_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.lcp_heartbeat_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.ready_for_autonomy_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.propulsion_drive_status_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.steering_drive_status_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.propulsion_rpm_feedback_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.steering_degree_feedback_signal_address] = 0;
    m_IntegerSignals[m_ValueRefs.parameters.ready_for_autonomy_button_available_signal_address] = 0;
    m_BooleanSignals[m_ValueRefs.parameters.modbus_debug_enable] = false;
    m_BooleanSignals[m_ValueRefs.parameters.start_listen_on_init] = true;
    m_RealSignals[m_ValueRefs.parameters.rpm_scaling_factor] = 1.0;

    m_IntegerSignals[m_ValueRefs.output.lcp_heartbeat] = 0;
    m_IntegerSignals[m_ValueRefs.output.autonomy_system_heartbeat] = 0;
    m_BooleanSignals[m_ValueRefs.output.autonomy_available] = false;
    m_BooleanSignals[m_ValueRefs.output.ready_for_autonomy_button] = false;
    m_RealSignals[m_ValueRefs.output.rpm_cmd] = 0.0;
    m_RealSignals[m_ValueRefs.output.angle_cmd] = 0.0;
    m_BooleanSignals[m_ValueRefs.output.ready_for_autonomy] = false;
    m_BooleanSignals[m_ValueRefs.output.ready_for_autonomy_button_available] = false;

    m_BooleanSignals[m_ValueRefs.input.steering_drive_ready] = true;
    m_BooleanSignals[m_ValueRefs.input.steering_drive_running] = true;
    m_BooleanSignals[m_ValueRefs.input.steering_drive_fault] = false;
    m_BooleanSignals[m_ValueRefs.input.propulsion_drive_ready] = true;
    m_BooleanSignals[m_ValueRefs.input.propulsion_drive_running] = true;
    m_BooleanSignals[m_ValueRefs.input.propulsion_drive_fault] = false;
    m_RealSignals[m_ValueRefs.input.rpm_fb] = 0.0;
    m_RealSignals[m_ValueRefs.input.angle_fb] = 0.0;
    m_BooleanSignals[m_ValueRefs.input.lcp_ready_for_autonomy_condition] = true;
    m_BooleanSignals[m_ValueRefs.input.stop_lcp_heartbeat_user_override] = false;
}

void FmuZpaPropulsionInterface::initialize_value_references()
{
    // parameters
    m_ValueRefs.parameters.modbus_server_port = 1;
    m_ValueRefs.parameters.modbus_server_ip = 2;
    m_ValueRefs.parameters.holding_register_base_address = 3;
    m_ValueRefs.parameters.num_holding_registers = 4;
    m_ValueRefs.parameters.autonomy_system_heartbeat_signal_address = 5;
    m_ValueRefs.parameters.autonomy_available_signal_address = 6;
    m_ValueRefs.parameters.propulsion_rpm_command_signal_address = 7;
    m_ValueRefs.parameters.steering_degree_command_signal_address = 8;
    m_ValueRefs.parameters.ready_for_autonomy_button_signal_address = 9;

    // Reads
    m_ValueRefs.parameters.lcp_heartbeat_signal_address = 10;
    m_ValueRefs.parameters.ready_for_autonomy_signal_address = 11;
    m_ValueRefs.parameters.propulsion_drive_status_signal_address = 12;
    m_ValueRefs.parameters.steering_drive_status_signal_address = 13;
    m_ValueRefs.parameters.propulsion_rpm_feedback_signal_address = 14;
    m_ValueRefs.parameters.steering_degree_feedback_signal_address = 15;
    m_ValueRefs.parameters.ready_for_autonomy_button_available_signal_address = 16;
    m_ValueRefs.parameters.modbus_debug_enable = 17;
    m_ValueRefs.parameters.start_listen_on_init = 18;
    m_ValueRefs.parameters.rpm_scaling_factor = 19;

    // output
    m_ValueRefs.output.rpm_cmd = 20;
    m_ValueRefs.output.angle_cmd = 21;
    m_ValueRefs.output.autonomy_available = 22;
    m_ValueRefs.output.ready_for_autonomy_button = 23;
    m_ValueRefs.output.autonomy_system_heartbeat = 24;
    m_ValueRefs.output.lcp_heartbeat = 25;
    m_ValueRefs.output.ready_for_autonomy = 26;
    m_ValueRefs.output.ready_for_autonomy_button_available = 27;

    // Input
    m_ValueRefs.input.rpm_fb = 30;
    m_ValueRefs.input.angle_fb = 31;
    m_ValueRefs.input.steering_drive_ready = 32;
    m_ValueRefs.input.steering_drive_running = 33;
    m_ValueRefs.input.steering_drive_fault = 34;
    m_ValueRefs.input.propulsion_drive_ready = 35;
    m_ValueRefs.input.propulsion_drive_running = 36;
    m_ValueRefs.input.propulsion_drive_fault = 37;
    m_ValueRefs.input.lcp_ready_for_autonomy_condition = 38;
    m_ValueRefs.input.stop_lcp_heartbeat_user_override = 39;
}

void FmuZpaPropulsionInterface::ExitInitializationMode()
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

ModelInputs FmuZpaPropulsionInterface::get_model_inputs()
{
    const auto base_address_holding_registers = m_IntegerSignals[m_ValueRefs.parameters.holding_register_base_address];
    // Command Signals
    ModelInputs inputs{};

    inputs.modbus_tcp_server.autonomy_system_heartbeat = m_modbus_server->get_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.autonomy_system_heartbeat_signal_address],
                 base_address_holding_registers));

    inputs.modbus_tcp_server.autonomy_available = m_modbus_server->get_register(get_addr(
        m_IntegerSignals[m_ValueRefs.parameters.autonomy_available_signal_address], base_address_holding_registers));

    inputs.modbus_tcp_server.ready_for_autonomy_button = m_modbus_server->get_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.ready_for_autonomy_button_signal_address],
                 base_address_holding_registers));
    inputs.modbus_tcp_server.rpm_cmd = m_modbus_server->get_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.propulsion_rpm_command_signal_address],
                 base_address_holding_registers));
    inputs.modbus_tcp_server.angle_cmd = m_modbus_server->get_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.steering_degree_command_signal_address],
                 base_address_holding_registers));

    inputs.fmu.steering_drive_ready = m_BooleanSignals[m_ValueRefs.input.steering_drive_ready];
    inputs.fmu.steering_drive_running = m_BooleanSignals[m_ValueRefs.input.steering_drive_running];
    inputs.fmu.steering_drive_fault = m_BooleanSignals[m_ValueRefs.input.steering_drive_fault];
    inputs.fmu.propulsion_drive_ready = m_BooleanSignals[m_ValueRefs.input.propulsion_drive_ready];
    inputs.fmu.propulsion_drive_running = m_BooleanSignals[m_ValueRefs.input.propulsion_drive_running];
    inputs.fmu.propulsion_drive_fault = m_BooleanSignals[m_ValueRefs.input.propulsion_drive_fault];
    inputs.fmu.propeller_rpm = m_RealSignals[m_ValueRefs.input.rpm_fb];
    inputs.fmu.angle_rad = m_RealSignals[m_ValueRefs.input.angle_fb];

    inputs.fmu.lcp_ready_for_autonomy_condition = m_BooleanSignals[m_ValueRefs.input.lcp_ready_for_autonomy_condition];
    inputs.fmu.stop_lcp_heartbeat_user_override = m_BooleanSignals[m_ValueRefs.input.stop_lcp_heartbeat_user_override];

    inputs.parameters.rpm_scaling_factor = m_RealSignals[m_ValueRefs.parameters.rpm_scaling_factor];

    return inputs;
}

void FmuZpaPropulsionInterface::set_model_output_signals(const ModelOutputs& outputs)
{
    const auto base_address_holding_registers = m_IntegerSignals[m_ValueRefs.parameters.holding_register_base_address];
    m_modbus_server->set_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.lcp_heartbeat_signal_address], base_address_holding_registers),
        m_lcp_heartbeat_state.heartbeat_counter);
    m_modbus_server->set_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.propulsion_rpm_feedback_signal_address],
                 base_address_holding_registers),
        outputs.modbus_tcp_server.rpm_fb);
    m_modbus_server->set_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.steering_degree_feedback_signal_address],
                 base_address_holding_registers),
        outputs.modbus_tcp_server.angle_fb);
    m_modbus_server->set_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.steering_drive_status_signal_address],
                 base_address_holding_registers),
        outputs.modbus_tcp_server.steering_status);
    m_modbus_server->set_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.propulsion_drive_status_signal_address],
                 base_address_holding_registers),
        outputs.modbus_tcp_server.propulsion_status);
    m_modbus_server->set_register(get_addr(m_IntegerSignals[m_ValueRefs.parameters.ready_for_autonomy_signal_address],
                                           base_address_holding_registers),
                                  outputs.modbus_tcp_server.ready_for_autonomy);
    m_modbus_server->set_register(
        get_addr(m_IntegerSignals[m_ValueRefs.parameters.ready_for_autonomy_button_available_signal_address],
                 base_address_holding_registers),
        outputs.modbus_tcp_server.ready_for_autonomy_button_available);

    m_IntegerSignals[m_ValueRefs.output.lcp_heartbeat] = m_lcp_heartbeat_state.heartbeat_counter;
    m_IntegerSignals[m_ValueRefs.output.autonomy_system_heartbeat] = outputs.fmu.autonomy_system_heartbeat;
    m_BooleanSignals[m_ValueRefs.output.autonomy_available] = outputs.fmu.autonomy_available;
    m_BooleanSignals[m_ValueRefs.output.ready_for_autonomy_button] = outputs.fmu.ready_for_autonomy_button;
    m_BooleanSignals[m_ValueRefs.output.ready_for_autonomy_button_available] =
        outputs.fmu.ready_for_autonomy_button_available;
    m_RealSignals[m_ValueRefs.output.rpm_cmd] = outputs.fmu.rpm_cmd;
    m_RealSignals[m_ValueRefs.output.angle_cmd] = outputs.fmu.angle_cmd;
    m_BooleanSignals[m_ValueRefs.output.ready_for_autonomy] = outputs.fmu.ready_for_autonomy;
}

bool FmuZpaPropulsionInterface::DoStep(cppfmu::FMIReal currentCommunicationPoint, cppfmu::FMIReal dt,
                                       cppfmu::FMIBoolean /*newStep*/, cppfmu::FMIReal& /*endOfStep*/)
{
    using zeabuz::common::utilities::math::inf2pipi;
    using zeabuz::common::utilities::math::to_deg;
    using zeabuz::common::utilities::math::to_rad;

    const auto inputs = get_model_inputs();

    const auto ready_for_autonomy_sequence_result =
        ready_for_autonomy_sequence(inputs, m_previous_ready_for_autonomy_state);
    const auto autonomy_heartbeat_sequence_result = autonomy_heartbeat_sequence(
        inputs, m_autonomy_heartbeat_state, currentCommunicationPoint, Parameters::AUTONOMY_HEARTBEAT_TIMEOUT);

    // Update autonomy heartbeat signal
    m_autonomy_heartbeat_state = autonomy_heartbeat_sequence_result.heartbeat_state;
    // Handle LCP heartbeat signal
    m_lcp_heartbeat_state = update_lcp_heartbeat(
        m_lcp_heartbeat_state, currentCommunicationPoint, Parameters::LCP_HEARTBEAT_INCREMENT_TIMESTEP,
        Parameters::HEARTBEAT_WRAPAROUND, inputs.fmu.stop_lcp_heartbeat_user_override);

    const auto outputs = get_model_output_signals(inputs, ready_for_autonomy_sequence_result,
                                                  autonomy_heartbeat_sequence_result, m_lcp_heartbeat_state);
    set_model_output_signals(outputs);

    m_previous_ready_for_autonomy_state.ready_for_autonomy = outputs.modbus_tcp_server.ready_for_autonomy;
    m_previous_ready_for_autonomy_state.button_signal = inputs.modbus_tcp_server.ready_for_autonomy_button;
    return true;
}

void FmuZpaPropulsionInterface::SetString(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                          const cppfmu::FMIString value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        m_StringSignals[vr[i]] = value[i];
    }
}

void FmuZpaPropulsionInterface::SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                        const cppfmu::FMIReal value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        m_RealSignals[vr[i]] = value[i];
    }
}

void FmuZpaPropulsionInterface::SetInteger(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                           const cppfmu::FMIInteger value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        m_IntegerSignals[vr[i]] = value[i];
    }
}

void FmuZpaPropulsionInterface::SetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                           const cppfmu::FMIBoolean value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        m_BooleanSignals[vr[i]] = value[i];
    }
}

void FmuZpaPropulsionInterface::GetString(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                          cppfmu::FMIString value[]) const
{
    for (std::size_t i = 0; i < nvr; ++i) {
        value[i] = m_StringSignals.at(vr[i]).c_str();
    }
}

void FmuZpaPropulsionInterface::GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                        cppfmu::FMIReal value[]) const
{
    for (std::size_t i = 0; i < nvr; ++i) {
        value[i] = m_RealSignals.at(vr[i]);
    }
}

void FmuZpaPropulsionInterface::GetInteger(const cppfmu::FMIValueReference vr[], std::size_t nvr,
                                           cppfmu::FMIInteger value[]) const
{
    for (std::size_t i = 0; i < nvr; ++i) {
        value[i] = m_IntegerSignals.at(vr[i]);
    }
}

void FmuZpaPropulsionInterface::GetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr,
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
    return cppfmu::AllocateUnique<FmuZpaPropulsionInterface>(memory, fmu_resource_location);
}
