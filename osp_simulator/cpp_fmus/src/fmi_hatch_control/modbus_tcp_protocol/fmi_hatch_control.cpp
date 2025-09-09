
#ifdef WIN32
#define NOMINMAX
#include <SDKDDKVer.h>
#endif

#include <cppfmu_cs.hpp>
#include <string>
#include <zeabuz/common/io/modbus_tcp_server.hpp>

#include "fmi_hatch_control/hatch_control.hpp"
#include "hatch_control_handler.hpp"

using namespace zeabuz::common::io::modbus_tcp;

struct ParametersValueRef
{
    cppfmu::FMIValueReference hatch_command_timeout;
    cppfmu::FMIValueReference hatch_id_fore;
    cppfmu::FMIValueReference hatch_id_aft;
    cppfmu::FMIValueReference emergency_stop_active;
    cppfmu::FMIValueReference modbus_server_port;
    cppfmu::FMIValueReference modbus_server_ip;
    cppfmu::FMIValueReference coil_address_write;
    cppfmu::FMIValueReference coil_write_num_bits;
    cppfmu::FMIValueReference coil_address_read;
    cppfmu::FMIValueReference coil_read_num_bits;
    cppfmu::FMIValueReference register_address_write;
    cppfmu::FMIValueReference register_write_num_regs;
    cppfmu::FMIValueReference register_address_read;
    cppfmu::FMIValueReference register_read_num_regs;
    cppfmu::FMIValueReference modbus_debug_enable;
    cppfmu::FMIValueReference hatch_start_position_fore;
    cppfmu::FMIValueReference hatch_start_position_aft;
    cppfmu::FMIValueReference auto_manual_mode;
};

struct OutputValueRef
{
    cppfmu::FMIValueReference hatch_position_fore;
    cppfmu::FMIValueReference hatch_position_aft;
};

struct ValueReferences
{
    ParametersValueRef parameters;
    OutputValueRef output;
};

class FmuHatchControl : public cppfmu::SlaveInstance
{
   public:
    explicit FmuHatchControl(cppfmu::FMIString fmu_resource_location) : m_resource_location(fmu_resource_location)
    {
        initialize_value_references();
        initialize_signal_maps();
    }

    void initialize_value_references()
    {
        // parameters
        m_ValueRefs.parameters.hatch_command_timeout = 1;
        m_ValueRefs.parameters.hatch_id_fore = 2;
        m_ValueRefs.parameters.hatch_id_aft = 3;
        m_ValueRefs.parameters.emergency_stop_active = 4;

        m_ValueRefs.parameters.modbus_server_port = 5;
        m_ValueRefs.parameters.modbus_server_ip = 6;
        m_ValueRefs.parameters.auto_manual_mode = 7;
        m_ValueRefs.parameters.coil_address_read = 8;
        m_ValueRefs.parameters.coil_read_num_bits = 9;
        m_ValueRefs.parameters.coil_address_write = 10;
        m_ValueRefs.parameters.coil_write_num_bits = 11;
        m_ValueRefs.parameters.register_address_read = 12;
        m_ValueRefs.parameters.register_read_num_regs = 13;
        m_ValueRefs.parameters.register_address_write = 14;
        m_ValueRefs.parameters.register_write_num_regs = 15;
        m_ValueRefs.parameters.modbus_debug_enable = 16;
        m_ValueRefs.parameters.hatch_start_position_fore = 17;
        m_ValueRefs.parameters.hatch_start_position_aft = 18;

        // output
        m_ValueRefs.output.hatch_position_fore = 19;
        m_ValueRefs.output.hatch_position_aft = 20;
    }

    void initialize_signal_maps()
    {
        // parameters
        m_StringSignals[m_ValueRefs.parameters.modbus_server_ip] = "127.0.0.1";
        m_RealSignals[m_ValueRefs.parameters.modbus_server_port] = 502;
        m_RealSignals[m_ValueRefs.parameters.hatch_command_timeout] = 12;
        m_RealSignals[m_ValueRefs.parameters.hatch_id_fore] = 1;
        m_RealSignals[m_ValueRefs.parameters.hatch_id_aft] = 2;
        m_BooleanSignals[m_ValueRefs.parameters.emergency_stop_active] = false;
        m_BooleanSignals[m_ValueRefs.parameters.auto_manual_mode] = true;
        m_BooleanSignals[m_ValueRefs.parameters.modbus_debug_enable] = false;
        using ModbusTcpProtocol::Mapping;
        m_RealSignals[m_ValueRefs.parameters.coil_address_write] = Mapping::COIL_ADDRESS_WRITE;
        m_RealSignals[m_ValueRefs.parameters.coil_write_num_bits] = Mapping::COIL_WRITE_NUM_BITS;
        m_RealSignals[m_ValueRefs.parameters.coil_address_read] = Mapping::COIL_ADDRESS_READ;
        m_RealSignals[m_ValueRefs.parameters.coil_read_num_bits] = Mapping::COIL_READ_NUM_BITS;
        m_RealSignals[m_ValueRefs.parameters.register_address_read] = Mapping::REGISTER_ADDRESS_WRITE;
        m_RealSignals[m_ValueRefs.parameters.register_read_num_regs] = Mapping::REGISTER_WRITE_NUM_REGS;
        m_RealSignals[m_ValueRefs.parameters.register_address_write] = Mapping::REGISTER_ADDRESS_READ;
        m_RealSignals[m_ValueRefs.parameters.register_write_num_regs] = Mapping::REGISTER_READ_NUM_REGS;
        m_RealSignals[m_ValueRefs.parameters.hatch_start_position_fore] = 100.0;
        m_RealSignals[m_ValueRefs.parameters.hatch_start_position_aft] = 100.0;

        // outputs
        m_RealSignals[m_ValueRefs.output.hatch_position_fore] = 100.0;
        m_RealSignals[m_ValueRefs.output.hatch_position_aft] = 100.0;
    }

    void ExitInitializationMode() override
    {
        auto modbus_mapping =
            ModbusMapping{static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.coil_address_write]),
                          static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.coil_write_num_bits]),
                          static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.coil_address_read]),
                          static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.coil_read_num_bits]),
                          static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.register_address_write]),
                          static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.register_write_num_regs]),
                          static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.register_address_read]),
                          static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.register_read_num_regs])};

        auto modbus_config = ServerConfiguration{};
        modbus_config.ip = m_StringSignals[m_ValueRefs.parameters.modbus_server_ip];
        modbus_config.port = static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.modbus_server_port]);
        modbus_config.mapping = modbus_mapping;
        modbus_config.debug_enable = m_BooleanSignals[m_ValueRefs.parameters.modbus_debug_enable];
        m_modbus_server = std::make_shared<Server>(modbus_config);

        m_RealSignals[m_ValueRefs.parameters.hatch_start_position_fore] =
            std::max(std::min(m_RealSignals[m_ValueRefs.parameters.hatch_start_position_fore], 100.0), 0.0);
        m_RealSignals[m_ValueRefs.parameters.hatch_start_position_aft] =
            std::max(std::min(m_RealSignals[m_ValueRefs.parameters.hatch_start_position_aft], 100.0), 0.0);
        auto hatch_handler_fore =
            std::make_shared<HatchControl>(m_RealSignals[m_ValueRefs.parameters.hatch_command_timeout],
                                           m_RealSignals[m_ValueRefs.parameters.hatch_start_position_fore]);
        auto hatch_handler_aft =
            std::make_shared<HatchControl>(m_RealSignals[m_ValueRefs.parameters.hatch_command_timeout],
                                           m_RealSignals[m_ValueRefs.parameters.hatch_start_position_aft]);

        std::unordered_map<uint16_t, std::shared_ptr<HatchControl>> hatch_control_map{};
        hatch_control_map.emplace(static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.hatch_id_fore]),
                                  hatch_handler_fore);
        hatch_control_map.emplace(static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.hatch_id_aft]),
                                  hatch_handler_aft);
        m_hatch_control_handler =
            std::make_unique<ModbusTcpProtocol::HatchControlHandler>(m_modbus_server, hatch_control_map);
    }

    void Terminate() override
    {
        m_modbus_server->shutdown();
    }

    void Reset() override
    {
        initialize_signal_maps();
        m_modbus_server->shutdown();
    }

    bool DoStep(cppfmu::FMIReal /*currentCommunicationPoint*/, cppfmu::FMIReal dt, cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal&
                /*endOfStep*/) override
    {
        m_hatch_control_handler->execute(dt, m_BooleanSignals[m_ValueRefs.parameters.emergency_stop_active],
                                         m_BooleanSignals[m_ValueRefs.parameters.auto_manual_mode]);

        auto hatch_positions = m_hatch_control_handler->get_hatch_positions();
        m_RealSignals[m_ValueRefs.output.hatch_position_fore] =
            hatch_positions.at(static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.hatch_id_fore]));
        m_RealSignals[m_ValueRefs.output.hatch_position_aft] =
            hatch_positions.at(static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.hatch_id_aft]));
        return true;
    }

    void SetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIString value[]) override
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            m_StringSignals.at(vr[i]) = value[i];
        }
    }

    void SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[]) override
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            m_RealSignals.at(vr[i]) = value[i];
        }
    }

    void SetInteger(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIInteger value[]) override
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            m_IntegerSignals.at(vr[i]) = value[i];
        }
    }

    void SetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIBoolean value[]) override
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            m_BooleanSignals.at(vr[i]) = value[i];
        }
    }

    void GetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIString value[]) const override
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            value[i] = m_StringSignals.at(vr[i]).c_str();
        }
    }

    void GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const override
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            value[i] = m_RealSignals.at(vr[i]);
        }
    }

    void GetInteger(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIInteger value[]) const override
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            value[i] = m_IntegerSignals.at(vr[i]);
        }
    }

    void GetBoolean(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIBoolean value[]) const override
    {
        for (std::size_t i = 0; i < nvr; ++i) {
            value[i] = m_BooleanSignals.at(vr[i]);
        }
    }

   private:
    std::shared_ptr<Server> m_modbus_server;
    std::unique_ptr<ModbusTcpProtocol::HatchControlHandler> m_hatch_control_handler;
    std::string m_resource_location;

    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIReal> m_RealSignals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIInteger> m_IntegerSignals;
    std::unordered_map<cppfmu::FMIValueReference, std::string> m_StringSignals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIBoolean> m_BooleanSignals;

    ValueReferences m_ValueRefs{};
};

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString /*instanceName*/, cppfmu::FMIString /*fmuGUID*/, cppfmu::FMIString fmu_resource_location,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger /*logger*/)
{
    // if (std::strcmp(fmuGUID, FMU_UUID) != 0) {
    //     throw std::runtime_error("FMU GUID mismatch");
    // }
    return cppfmu::AllocateUnique<FmuHatchControl>(memory, fmu_resource_location);
}
