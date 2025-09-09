#ifdef WIN32
#include <SDKDDKVer.h>
#endif

#include <cppfmu_cs.hpp>
#include <string>
#include <zeabuz/common/io/file_dumper_t.hpp>
#include <zeabuz/common/io/message_handler.hpp>
#include <zeabuz/common/io/messages/custom/zb_sim_nav.hpp>
#include <zeabuz/common/io/udp_connection.hpp>
#include <zeabuz/common/utilities/pretty_class_macro.hpp>
#include <zeabuz/common/utilities/string.hpp>

using zeabuz::common::io::messages::custom::SimNavMessage;
using namespace zeabuz::common::io;

struct Vector3ValueRef
{
    Vector3ValueRef() : x(-1), y(-1), z(-1){};
    cppfmu::FMIValueReference x;
    cppfmu::FMIValueReference y;
    cppfmu::FMIValueReference z;
};

struct PoseValueRef
{
    Vector3ValueRef position;
    Vector3ValueRef orientation;
};

struct twistValueRef
{
    Vector3ValueRef linear;
    Vector3ValueRef angular;
};

struct ParameterValueRef
{
    cppfmu::FMIValueReference remoteIp;
    cppfmu::FMIValueReference localIp;
    cppfmu::FMIValueReference remotePort;
    cppfmu::FMIValueReference localPort;
    cppfmu::FMIValueReference dump_messages;
    cppfmu::FMIValueReference navigation_time_offset_ms;
};

struct ValueReferences
{
    ParameterValueRef parameters;
    twistValueRef twistBody;
    twistValueRef twistNED;
    PoseValueRef pose;
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
        m_ValueRefs.parameters.navigation_time_offset_ms = 5;

        m_ValueRefs.pose.position.x = 18;
        m_ValueRefs.pose.position.y = 19;
        m_ValueRefs.pose.position.z = 20;
        m_ValueRefs.pose.orientation.x = 21;
        m_ValueRefs.pose.orientation.y = 22;
        m_ValueRefs.pose.orientation.z = 23;

        m_ValueRefs.twistBody.linear.x = 24;
        m_ValueRefs.twistBody.linear.y = 25;
        m_ValueRefs.twistBody.linear.z = 26;
        m_ValueRefs.twistBody.angular.x = 27;
        m_ValueRefs.twistBody.angular.y = 28;
        m_ValueRefs.twistBody.angular.z = 29;

        m_ValueRefs.twistNED.linear.x = 30;
        m_ValueRefs.twistNED.linear.y = 31;
        m_ValueRefs.twistNED.linear.z = 32;
        m_ValueRefs.twistNED.angular.x = 33;
        m_ValueRefs.twistNED.angular.y = 34;
        m_ValueRefs.twistNED.angular.z = 35;
    }

    void initializeSignalMaps()
    {
        // Parameters
        m_StringSignals[m_ValueRefs.parameters.remoteIp] = "127.0.0.1";
        m_StringSignals[m_ValueRefs.parameters.remotePort] = "3000";
        m_BooleanSignals[m_ValueRefs.parameters.dump_messages] = false;
        m_RealSignals[m_ValueRefs.parameters.navigation_time_offset_ms] = 0;

        // Inputs signals
        m_RealSignals[m_ValueRefs.pose.position.x] = 0;
        m_RealSignals[m_ValueRefs.pose.position.y] = 0;
        m_RealSignals[m_ValueRefs.pose.position.z] = 0;
        m_RealSignals[m_ValueRefs.pose.orientation.x] = 0;
        m_RealSignals[m_ValueRefs.pose.orientation.y] = 0;
        m_RealSignals[m_ValueRefs.pose.orientation.z] = 0;

        m_RealSignals[m_ValueRefs.twistBody.linear.x] = 0;
        m_RealSignals[m_ValueRefs.twistBody.linear.y] = 0;
        m_RealSignals[m_ValueRefs.twistBody.linear.z] = 0;
        m_RealSignals[m_ValueRefs.twistBody.angular.x] = 0;
        m_RealSignals[m_ValueRefs.twistBody.angular.y] = 0;
        m_RealSignals[m_ValueRefs.twistBody.angular.z] = 0;

        m_RealSignals[m_ValueRefs.twistNED.linear.x] = 0;
        m_RealSignals[m_ValueRefs.twistNED.linear.y] = 0;
        m_RealSignals[m_ValueRefs.twistNED.linear.z] = 0;
        m_RealSignals[m_ValueRefs.twistNED.angular.x] = 0;
        m_RealSignals[m_ValueRefs.twistNED.angular.y] = 0;
        m_RealSignals[m_ValueRefs.twistNED.angular.z] = 0;
    }

    void sendMessages(const double clock)
    {
        m_message_handler->send_message(createZBSimNavMessage(clock));
    }

    SimNavMessage createZBSimNavMessage(const double clock)
    {
        SimNavMessage msg{};
        msg.Stamp =
            clock - static_cast<double>(m_RealSignals[m_ValueRefs.parameters.navigation_time_offset_ms] / 1000.0);
        msg.PoseNED.Position.x = m_RealSignals[m_ValueRefs.pose.position.x];
        msg.PoseNED.Position.y = m_RealSignals[m_ValueRefs.pose.position.y];
        msg.PoseNED.Position.z = m_RealSignals[m_ValueRefs.pose.position.z];
        msg.PoseNED.Orientation.x = m_RealSignals[m_ValueRefs.pose.orientation.x];
        msg.PoseNED.Orientation.y = m_RealSignals[m_ValueRefs.pose.orientation.y];
        msg.PoseNED.Orientation.z = m_RealSignals[m_ValueRefs.pose.orientation.z];

        msg.TwistNED.Linear.x = m_RealSignals[m_ValueRefs.twistNED.linear.x];
        msg.TwistNED.Linear.y = m_RealSignals[m_ValueRefs.twistNED.linear.y];
        msg.TwistNED.Linear.z = m_RealSignals[m_ValueRefs.twistNED.linear.z];
        msg.TwistNED.Angular.x = m_RealSignals[m_ValueRefs.twistNED.angular.x];
        msg.TwistNED.Angular.y = m_RealSignals[m_ValueRefs.twistNED.angular.y];
        msg.TwistNED.Angular.z = m_RealSignals[m_ValueRefs.twistNED.angular.z];

        msg.TwistBody.Linear.x = m_RealSignals[m_ValueRefs.twistBody.linear.x];
        msg.TwistBody.Linear.y = m_RealSignals[m_ValueRefs.twistBody.linear.y];
        msg.TwistBody.Linear.z = m_RealSignals[m_ValueRefs.twistBody.linear.z];
        msg.TwistBody.Angular.x = m_RealSignals[m_ValueRefs.twistBody.angular.x];
        msg.TwistBody.Angular.y = m_RealSignals[m_ValueRefs.twistBody.angular.y];
        msg.TwistBody.Angular.z = m_RealSignals[m_ValueRefs.twistBody.angular.z];
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