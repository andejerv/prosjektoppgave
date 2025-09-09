#include "zeabuz/common/io/message_handler_descriptions.hpp"
#ifdef WIN32
#include <SDKDDKVer.h>
#endif

#include <cppfmu_cs.hpp>
#include <string>
#include <zeabuz/common/io/message_handler.hpp>
#include <zeabuz/common/io/messages/custom/ultrasound_distance.hpp>
#include <zeabuz/common/io/udp_connection.hpp>
#include <zeabuz/common/utilities/string.hpp>

using zeabuz::common::utilities::string::string_to_uint16;
using namespace zeabuz::common::io;

struct UltrasonicSensorValueRef
{
    cppfmu::FMIValueReference valid;
    cppfmu::FMIValueReference meas;
};

struct UltrasonicSensorsValueRef
{
    UltrasonicSensorValueRef forePort;
    UltrasonicSensorValueRef foreStarboard;
    UltrasonicSensorValueRef aftPort;
    UltrasonicSensorValueRef aftStarboard;
};

struct InputValueRef
{
    UltrasonicSensorsValueRef ultrasonicSensors;
};

struct ParametersValueRef
{
    cppfmu::FMIValueReference remote_ip;
    cppfmu::FMIValueReference local_ip;
    cppfmu::FMIValueReference remote_port_fore_port;
    cppfmu::FMIValueReference remote_port_fore_starboard;
    cppfmu::FMIValueReference remote_port_aft_port;
    cppfmu::FMIValueReference remote_port_aft_starboard;
    cppfmu::FMIValueReference local_port_fore_port;
    cppfmu::FMIValueReference local_port_fore_starboard;
    cppfmu::FMIValueReference local_port_aft_port;
    cppfmu::FMIValueReference local_port_aft_starboard;
    cppfmu::FMIValueReference dump_messages;
};

struct ValueReferences
{
    InputValueRef input;
    ParametersValueRef parameters;
};

class FmuUltrasonicDistanceSensorCommunication : public cppfmu::SlaveInstance
{
   public:
    FmuUltrasonicDistanceSensorCommunication(cppfmu::FMIString fmuResourceLocation)
    : _resource_location(fmuResourceLocation)
    {
        initializeValueReferences();
        initializeSignalMaps();
    }

    void initializeValueReferences()
    {
        // parameters
        m_ValueRefs.parameters.remote_ip = 0;
        m_ValueRefs.parameters.local_ip = 1;
        m_ValueRefs.parameters.remote_port_fore_starboard = 2;
        m_ValueRefs.parameters.remote_port_fore_port = 3;
        m_ValueRefs.parameters.remote_port_aft_port = 4;
        m_ValueRefs.parameters.remote_port_aft_starboard = 5;
        m_ValueRefs.parameters.local_port_fore_starboard = 6;
        m_ValueRefs.parameters.local_port_fore_port = 7;
        m_ValueRefs.parameters.local_port_aft_port = 8;
        m_ValueRefs.parameters.local_port_aft_starboard = 9;
        m_ValueRefs.parameters.dump_messages = 18;

        // Input signals
        m_ValueRefs.input.ultrasonicSensors.forePort.meas = 10;
        m_ValueRefs.input.ultrasonicSensors.forePort.valid = 11;
        m_ValueRefs.input.ultrasonicSensors.foreStarboard.meas = 12;
        m_ValueRefs.input.ultrasonicSensors.foreStarboard.valid = 13;
        m_ValueRefs.input.ultrasonicSensors.aftPort.meas = 14;
        m_ValueRefs.input.ultrasonicSensors.aftPort.valid = 15;
        m_ValueRefs.input.ultrasonicSensors.aftStarboard.meas = 16;
        m_ValueRefs.input.ultrasonicSensors.aftStarboard.valid = 17;
    }

    void initializeSignalMaps()
    {
        // parameters
        m_StringSignals[m_ValueRefs.parameters.remote_ip] = "127.0.0.1";
        m_StringSignals[m_ValueRefs.parameters.local_ip] = "0.0.0.0";
        m_StringSignals[m_ValueRefs.parameters.remote_port_fore_starboard] = "2001";
        m_StringSignals[m_ValueRefs.parameters.remote_port_fore_port] = "2002";
        m_StringSignals[m_ValueRefs.parameters.remote_port_aft_port] = "2003";
        m_StringSignals[m_ValueRefs.parameters.remote_port_aft_starboard] = "2004";
        m_StringSignals[m_ValueRefs.parameters.local_port_fore_starboard] = "1996";
        m_StringSignals[m_ValueRefs.parameters.local_port_fore_port] = "1997";
        m_StringSignals[m_ValueRefs.parameters.local_port_aft_port] = "1998";
        m_StringSignals[m_ValueRefs.parameters.local_port_aft_starboard] = "1999";
        m_BooleanSignals[m_ValueRefs.parameters.dump_messages] = false;

        // Inputs signals
        m_RealSignals[m_ValueRefs.input.ultrasonicSensors.forePort.meas] = 0.0;
        m_BooleanSignals[m_ValueRefs.input.ultrasonicSensors.forePort.valid] = false;
        m_RealSignals[m_ValueRefs.input.ultrasonicSensors.foreStarboard.meas] = 0.0;
        m_BooleanSignals[m_ValueRefs.input.ultrasonicSensors.foreStarboard.valid] = false;
        m_RealSignals[m_ValueRefs.input.ultrasonicSensors.aftPort.meas] = 0.0;
        m_BooleanSignals[m_ValueRefs.input.ultrasonicSensors.aftPort.valid] = false;
        m_RealSignals[m_ValueRefs.input.ultrasonicSensors.aftStarboard.meas] = 0.0;
        m_BooleanSignals[m_ValueRefs.input.ultrasonicSensors.aftStarboard.valid] = false;
    }

    void ExitInitializationMode() override
    {
        std::shared_ptr<FileDumperT<ChronoTime, ChronoTime::TimeStruct>> file_dumper;
        if (m_BooleanSignals[m_ValueRefs.parameters.dump_messages]) {
            auto m_ChronoTime = std::make_shared<ChronoTime>();
            file_dumper = std::make_shared<FileDumperT<ChronoTime, ChronoTime::TimeStruct>>(
                "UltrasoundDistanceSensorsOSP.txt", m_ChronoTime.get());
        }

        auto udpConnectionForePort = udp::Socket::setup_p2p_socket(
            string_to_uint16(m_StringSignals[m_ValueRefs.parameters.remote_port_fore_port]),
            m_StringSignals[m_ValueRefs.parameters.remote_ip],
            string_to_uint16(m_StringSignals[m_ValueRefs.parameters.local_port_fore_port]),
            m_StringSignals[m_ValueRefs.parameters.local_ip], file_dumper);
        m_MessageHandlerForePort = std::make_unique<message_handlers::NMEA0183MessageHandler>(udpConnectionForePort);

        auto udpConnectionForeStarboard = udp::Socket::setup_p2p_socket(
            string_to_uint16(m_StringSignals[m_ValueRefs.parameters.remote_port_fore_starboard]),
            m_StringSignals[m_ValueRefs.parameters.remote_ip],
            string_to_uint16(m_StringSignals[m_ValueRefs.parameters.local_port_fore_starboard]),
            m_StringSignals[m_ValueRefs.parameters.local_ip], file_dumper);
        m_MessageHandlerForeStarboard = std::make_unique<message_handlers::NMEA0183MessageHandler>(udpConnectionForeStarboard);

        auto udpConnectionAftPort = udp::Socket::setup_p2p_socket(
            string_to_uint16(m_StringSignals[m_ValueRefs.parameters.remote_port_aft_port]),
            m_StringSignals[m_ValueRefs.parameters.remote_ip],
            string_to_uint16(m_StringSignals[m_ValueRefs.parameters.local_port_aft_port]),
            m_StringSignals[m_ValueRefs.parameters.local_ip], file_dumper);
        m_MessageHandlerAftPort = std::make_unique<message_handlers::NMEA0183MessageHandler>(udpConnectionAftPort);

        auto udpConnectionAftStarboard = udp::Socket::setup_p2p_socket(
            string_to_uint16(m_StringSignals[m_ValueRefs.parameters.remote_port_aft_starboard]),
            m_StringSignals[m_ValueRefs.parameters.remote_ip],
            string_to_uint16(m_StringSignals[m_ValueRefs.parameters.local_port_aft_starboard]),
            m_StringSignals[m_ValueRefs.parameters.local_ip], file_dumper);
        m_MessageHandlerAftStarboard = std::make_unique<message_handlers::NMEA0183MessageHandler>(udpConnectionAftStarboard);
    }

    // Output Signals
    void sendDistanceData()
    {
        m_MessageHandlerForeStarboard->send_message(createUltrasoundDistanceMessage(
            1, m_RealSignals.at(m_ValueRefs.input.ultrasonicSensors.foreStarboard.meas) * 1000.0,
            m_BooleanSignals.at(m_ValueRefs.input.ultrasonicSensors.foreStarboard.valid)));
        m_MessageHandlerForePort->send_message(createUltrasoundDistanceMessage(
            2, m_RealSignals.at(m_ValueRefs.input.ultrasonicSensors.forePort.meas) * 1000.0,
            m_BooleanSignals.at(m_ValueRefs.input.ultrasonicSensors.forePort.valid)));
        m_MessageHandlerAftPort->send_message(createUltrasoundDistanceMessage(
            3, m_RealSignals.at(m_ValueRefs.input.ultrasonicSensors.aftPort.meas) * 1000.0,
            m_BooleanSignals.at(m_ValueRefs.input.ultrasonicSensors.aftPort.valid)));
        m_MessageHandlerAftStarboard->send_message(createUltrasoundDistanceMessage(
            4, m_RealSignals.at(m_ValueRefs.input.ultrasonicSensors.aftStarboard.meas) * 1000.0,
            m_BooleanSignals.at(m_ValueRefs.input.ultrasonicSensors.aftStarboard.valid)));
    }

    messages::custom::UltrasoundDistanceMessage createUltrasoundDistanceMessage(uint16_t id, double distance,
                                                                                bool isValid)
    {
        messages::custom::UltrasoundDistanceMessage msg{};
        messages::custom::UltrasoundDistanceStruct data{};
        data.id = id;
        data.distance = distance;
        data.valid = isValid;
        data.signal_error = messages::custom::UltrasoundDistanceStruct::SignalError::SIGNAL_OK;
        msg.set_message_parameters(data);
        return msg;
    }

    void Terminate() override
    {
        m_MessageHandlerForeStarboard.reset();
        m_MessageHandlerForePort.reset();
        m_MessageHandlerAftPort.reset();
        m_MessageHandlerAftStarboard.reset();
    }

    void Reset() override
    {
        initializeSignalMaps();
        m_MessageHandlerForeStarboard.reset();
        m_MessageHandlerForePort.reset();
        m_MessageHandlerAftPort.reset();
        m_MessageHandlerAftStarboard.reset();
    }

    void SetString(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIString value[])
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

    bool DoStep(cppfmu::FMIReal currentCommunicationPoint, cppfmu::FMIReal /*dt*/, cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal& /*endOfStep*/) override
    {
        sendDistanceData();
        return true;
    }

   private:
   
    std::unique_ptr<message_handlers::NMEA0183MessageHandler> m_MessageHandlerForePort;
    std::unique_ptr<message_handlers::NMEA0183MessageHandler> m_MessageHandlerForeStarboard;
    std::unique_ptr<message_handlers::NMEA0183MessageHandler> m_MessageHandlerAftPort;
    std::unique_ptr<message_handlers::NMEA0183MessageHandler> m_MessageHandlerAftStarboard;

    std::string _resource_location;

    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIReal> m_RealSignals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIInteger> m_IntegerSignals;
    std::unordered_map<cppfmu::FMIValueReference, std::string> m_StringSignals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIBoolean> m_BooleanSignals;

    ValueReferences m_ValueRefs;
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
    return cppfmu::AllocateUnique<FmuUltrasonicDistanceSensorCommunication>(memory, fmuResourceLocation);
}
