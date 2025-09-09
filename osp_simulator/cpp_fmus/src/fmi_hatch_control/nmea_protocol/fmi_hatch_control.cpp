#ifdef WIN32
#include <SDKDDKVer.h>
#endif

#include <cppfmu_cs.hpp>
#include <string>
#include <zeabuz/common/io/file_dumper_t.hpp>
#include <zeabuz/common/io/udp_connection.hpp>

#include "fmi_hatch_control/hatch_control.hpp"
#include "hatch_control_handler.hpp"

using namespace zeabuz::common::io;
struct ParametersValueRef
{
    cppfmu::FMIValueReference hatch_command_timeout;
    cppfmu::FMIValueReference hatch_id_fore;
    cppfmu::FMIValueReference hatch_id_aft;
    cppfmu::FMIValueReference emergency_stop_active;
    cppfmu::FMIValueReference remote_ip;
    cppfmu::FMIValueReference local_ip;
    cppfmu::FMIValueReference command_port;
    cppfmu::FMIValueReference feedback_port;
    cppfmu::FMIValueReference dump_messages;
    cppfmu::FMIValueReference hatch_start_position_fore;
    cppfmu::FMIValueReference hatch_start_position_aft;
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
        m_ValueRefs.parameters.remote_ip = 5;
        m_ValueRefs.parameters.local_ip = 6;
        m_ValueRefs.parameters.command_port = 7;
        m_ValueRefs.parameters.feedback_port = 8;
        m_ValueRefs.parameters.dump_messages = 9;
        m_ValueRefs.parameters.hatch_start_position_fore = 12;
        m_ValueRefs.parameters.hatch_start_position_aft = 13;

        // output
        m_ValueRefs.output.hatch_position_fore = 10;
        m_ValueRefs.output.hatch_position_aft = 11;
    }

    void initialize_signal_maps()
    {
        // parameters
        m_StringSignals[m_ValueRefs.parameters.remote_ip] = "127.0.0.1";
        m_StringSignals[m_ValueRefs.parameters.local_ip] = "0.0.0.0";
        m_RealSignals[m_ValueRefs.parameters.command_port] = 2021;
        m_RealSignals[m_ValueRefs.parameters.feedback_port] = 2022;
        m_RealSignals[m_ValueRefs.parameters.hatch_command_timeout] = 12;
        m_RealSignals[m_ValueRefs.parameters.hatch_id_fore] = 1;
        m_RealSignals[m_ValueRefs.parameters.hatch_id_aft] = 2;
        m_BooleanSignals[m_ValueRefs.parameters.dump_messages] = false;
        m_BooleanSignals[m_ValueRefs.parameters.emergency_stop_active] = false;
        m_RealSignals[m_ValueRefs.parameters.hatch_start_position_fore] = 100.0;
        m_RealSignals[m_ValueRefs.parameters.hatch_start_position_aft] = 100.0;

        // outputs
        m_RealSignals[m_ValueRefs.output.hatch_position_fore] = 100.0;
        m_RealSignals[m_ValueRefs.output.hatch_position_aft] = 100.0;
    }

    void ExitInitializationMode() override
    {
        std::shared_ptr<FileDumperT<ChronoTime, ChronoTime::TimeStruct>> file_dumper;
        if (m_BooleanSignals[m_ValueRefs.parameters.dump_messages]) {
            auto chrono_time = std::make_shared<ChronoTime>();
            file_dumper = std::make_shared<FileDumperT<ChronoTime, ChronoTime::TimeStruct>>("HatchControl.txt",
                                                                                            chrono_time.get());
        }

        auto udp_connection =
            udp::Socket::setup_p2p_socket(static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.feedback_port]),
                                          m_StringSignals[m_ValueRefs.parameters.remote_ip],
                                          static_cast<uint16_t>(m_RealSignals[m_ValueRefs.parameters.command_port]),
                                          m_StringSignals[m_ValueRefs.parameters.local_ip], file_dumper);
        m_message_handler = std::make_shared<message_handlers::NMEA0183MessageHandler>(udp_connection);

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
            std::make_unique<NMEAProtocol::HatchControlHandler>(m_message_handler, hatch_control_map);
        m_hatch_control_handler->initialize_fmu();
    }

    void Terminate() override
    {
        m_message_handler->shutdown();
    }

    void Reset() override
    {
        initialize_signal_maps();
        m_message_handler->shutdown();
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

    bool DoStep(cppfmu::FMIReal /*currentCommunicationPoint*/, cppfmu::FMIReal dt, cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal& /*endOfStep*/) override
    {
        m_hatch_control_handler->execute(dt, m_BooleanSignals[m_ValueRefs.parameters.emergency_stop_active]);
        m_hatch_control_handler->send_hatch_feedback();

        auto hatch_positions = m_hatch_control_handler->get_hatch_positions();
        m_RealSignals[m_ValueRefs.output.hatch_position_fore] =
            hatch_positions.at(m_RealSignals[m_ValueRefs.parameters.hatch_id_fore]);
        m_RealSignals[m_ValueRefs.output.hatch_position_aft] =
            hatch_positions.at(m_RealSignals[m_ValueRefs.parameters.hatch_id_aft]);
        return true;
    }

   private:
    std::shared_ptr<message_handlers::NMEA0183MessageHandler> m_message_handler;
    std::unique_ptr<NMEAProtocol::HatchControlHandler> m_hatch_control_handler;
    std::string m_resource_location;

    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIReal> m_RealSignals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIInteger> m_IntegerSignals;
    std::unordered_map<cppfmu::FMIValueReference, std::string> m_StringSignals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIBoolean> m_BooleanSignals;

    ValueReferences m_ValueRefs;
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
