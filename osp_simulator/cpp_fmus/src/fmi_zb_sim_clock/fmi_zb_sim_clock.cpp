#include "zeabuz/common/io/messages/custom/zb_sim_nav.hpp"
#ifdef WIN32
#include <SDKDDKVer.h>
#endif

#include <cppfmu_cs.hpp>
#include <string>
#include <zeabuz/common/io/file_dumper_t.hpp>
#include <zeabuz/common/io/message_handler.hpp>
#include <zeabuz/common/io/messages/custom/zb_sim_clock.hpp>
#include <zeabuz/common/io/udp_connection.hpp>
#include <zeabuz/common/utilities/conversions.hpp>
#include <zeabuz/common/utilities/enums.hpp>
#include <zeabuz/common/utilities/math.hpp>
#include <zeabuz/common/utilities/pretty_class_macro.hpp>
#include <zeabuz/common/utilities/string.hpp>

using namespace zeabuz::common::io;
using zeabuz::common::io::messages::custom::SimClockMessage;

struct ParameterValueRef
{
    cppfmu::FMIValueReference remoteIp;
    cppfmu::FMIValueReference localIp;
    cppfmu::FMIValueReference remotePort;
    cppfmu::FMIValueReference localPort;
    cppfmu::FMIValueReference dump_messages;
};

struct ValueReferences
{
    ParameterValueRef parameters;
};

class FmuOspGenInterface : public cppfmu::SlaveInstance
{
   public:
    FmuOspGenInterface(cppfmu::FMIString fmuResourceLocation) : _resource_location(fmuResourceLocation)
    {
        initializeValueReferences();
        initializeSignalMaps();
    }

    void initializeValueReferences()
    {
        // Input signals
        m_ValueRefs.parameters.remoteIp = 0;
        m_ValueRefs.parameters.remotePort = 2;
        m_ValueRefs.parameters.dump_messages = 4;
    }

    void initializeSignalMaps()
    {
        // Parameters
        m_StringSignals[m_ValueRefs.parameters.remoteIp] = "127.0.0.1";
        m_StringSignals[m_ValueRefs.parameters.remotePort] = "3001";
        m_BooleanSignals[m_ValueRefs.parameters.dump_messages] = false;
    }

    void sendMessages(const double clock)
    {
        m_message_handler->send_message(createZBSimClockMessage(clock));
    }

    SimClockMessage createZBSimClockMessage(const double clock)
    {
        SimClockMessage msg{};
        msg.Clock = clock;
        return msg;
    }

    void Terminate() override
    {
        m_message_handler->shutdown();
    }

    void Reset() override
    {
        initializeSignalMaps();
        m_message_handler->shutdown();
    }

    void ExitInitializationMode() override
    {
        std::shared_ptr<FileDumperT<ChronoTime, ChronoTime::TimeStruct>> file_dumper;
        if (m_BooleanSignals[m_ValueRefs.parameters.dump_messages]) {
            auto m_ChronoTime = std::make_shared<ChronoTime>();
            file_dumper = std::make_shared<FileDumperT<ChronoTime, ChronoTime::TimeStruct>>("OSPGenInterface.txt",
                                                                                            m_ChronoTime.get());
        }

        auto udp_connection = zeabuz::common::io::udp::Socket::setup_send_socket(
            zeabuz::common::utilities::string::string_to_uint16(m_StringSignals[m_ValueRefs.parameters.remotePort]),
            m_StringSignals[m_ValueRefs.parameters.remoteIp], file_dumper);

        m_message_handler = std::make_unique<message_handlers::NMEA0183MessageHandler>(udp_connection);
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
    };

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

    bool DoStep(cppfmu::FMIReal currentCommunicationPoint, cppfmu::FMIReal /*dt*/, cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal& /*endOfStep*/) override
    {
        sendMessages(currentCommunicationPoint);
        return true;
    }

   private:
    std::string _resource_location;

    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIReal> m_RealSignals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIInteger> m_IntegerSignals;
    std::unordered_map<cppfmu::FMIValueReference, std::string> m_StringSignals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIBoolean> m_BooleanSignals;

    ValueReferences m_ValueRefs;

    std::unique_ptr<message_handlers::NMEA0183MessageHandler> m_message_handler;
};

struct component
{
    std::vector<int> codes;
};

struct mtllc
{
    int cardId;
    std::unordered_map<int, component> components;
};

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString /*instanceName*/, cppfmu::FMIString fmuGUID, cppfmu::FMIString fmuResourceLocation,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger logger)
{
    // if (std::strcmp(fmuGUID, FMU_UUID) != 0) {
    //     throw std::runtime_error("FMU GUID mismatch");
    // }
    return cppfmu::AllocateUnique<FmuOspGenInterface>(memory, fmuResourceLocation);
}