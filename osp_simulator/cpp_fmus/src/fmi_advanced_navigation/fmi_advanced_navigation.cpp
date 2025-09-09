#include "zeabuz/common/io/message_handler_descriptions.hpp"
#ifdef WIN32
#include <SDKDDKVer.h>
#endif

#define _USE_MATH_DEFINES
#include <math.h>

#include <cppfmu_cs.hpp>
#include <string>
#include <zeabuz/common/io/file_dumper_t.hpp>
#include <zeabuz/common/io/message_handler.hpp>
#include <zeabuz/common/io/messages/custom/byte_message.hpp>
#include <zeabuz/common/io/udp_connection.hpp>
#include <zeabuz/common/utilities/conversions.hpp>
#include <zeabuz/common/utilities/math.hpp>
#include <zeabuz/common/utilities/string.hpp>

#include "adnav_lib/an_packet_protocol.h"
#include "adnav_lib/anpp_encode_decode.h"

using namespace zeabuz::common::io;
using zeabuz::common::io::messages::custom::ByteMessage;
using zeabuz::common::utilities::conversions::position_local2lla;
using zeabuz::common::utilities::math::inf2pipi;
using zeabuz::common::utilities::math::to_rad;
using zeabuz::common::utilities::string::string_to_uint16;

struct Vector3ValueRef
{
    Vector3ValueRef() : x(UINT_MAX), y(UINT_MAX), z(UINT_MAX){};
    cppfmu::FMIValueReference x;
    cppfmu::FMIValueReference y;
    cppfmu::FMIValueReference z;
};

struct PoseValueRef
{
    Vector3ValueRef position;
    Vector3ValueRef orientation;
};

struct TwistValueRef
{
    Vector3ValueRef linear;
    Vector3ValueRef angular;
};

struct AccValueRef
{
    Vector3ValueRef linear;
    Vector3ValueRef angular;
};
struct InputValueRef
{
    // pose
    PoseValueRef pose;
    TwistValueRef twist_body;
    TwistValueRef twist_ned;
    AccValueRef acceleration;
};

struct ParametersValueRef
{
    cppfmu::FMIValueReference autonomy_machine_remote_ip;
    cppfmu::FMIValueReference mt_dp_remote_ip;
    cppfmu::FMIValueReference local_ip;
    cppfmu::FMIValueReference autonomy_machine_remote_port;
    cppfmu::FMIValueReference mt_dp_remote_port;
    cppfmu::FMIValueReference latitude0;
    cppfmu::FMIValueReference longitude0;
    cppfmu::FMIValueReference height0;
    cppfmu::FMIValueReference dump_messages;
    cppfmu::FMIValueReference fix_type;
    cppfmu::FMIValueReference hdop;
    cppfmu::FMIValueReference vdop;
    cppfmu::FMIValueReference gps_satellites;
    cppfmu::FMIValueReference glonass_satellites;
    cppfmu::FMIValueReference beidou_satellites;
    cppfmu::FMIValueReference galileo_satellites;
    cppfmu::FMIValueReference sbas_satellites;
    cppfmu::FMIValueReference g_force;
    cppfmu::FMIValueReference lat_stddev;
    cppfmu::FMIValueReference lon_stddev;
    cppfmu::FMIValueReference heigth_stddev;
};

struct ValueReferences
{
    ParametersValueRef parameters;
    PoseValueRef pose;
    TwistValueRef twist_body;
    TwistValueRef twist_ned;
    AccValueRef acceleration;
};

class FmuDpInterface : public cppfmu::SlaveInstance
{
   public:
    FmuDpInterface(cppfmu::FMIString fmuResourceLocation) : _resource_location(fmuResourceLocation)
    {
        initializeValueReferences();
        initializeSignalMaps();
    }

    void initializeValueReferences()
    {
        // Input signals
        m_ValueRefs.parameters.autonomy_machine_remote_ip = 0;
        m_ValueRefs.parameters.mt_dp_remote_ip = 1;
        m_ValueRefs.parameters.local_ip = 2;
        m_ValueRefs.parameters.autonomy_machine_remote_port = 3;
        m_ValueRefs.parameters.mt_dp_remote_port = 4;
        m_ValueRefs.parameters.latitude0 = 5;
        m_ValueRefs.parameters.longitude0 = 6;
        m_ValueRefs.parameters.height0 = 7;
        m_ValueRefs.parameters.dump_messages = 8;
        m_ValueRefs.parameters.hdop = 42;
        m_ValueRefs.parameters.vdop = 43;
        m_ValueRefs.parameters.gps_satellites = 44;
        m_ValueRefs.parameters.glonass_satellites = 45;
        m_ValueRefs.parameters.beidou_satellites = 46;
        m_ValueRefs.parameters.galileo_satellites = 47;
        m_ValueRefs.parameters.sbas_satellites = 48;
        m_ValueRefs.parameters.g_force = 49;
        m_ValueRefs.parameters.lat_stddev = 50;
        m_ValueRefs.parameters.lon_stddev = 51;
        m_ValueRefs.parameters.heigth_stddev = 52;
        m_ValueRefs.parameters.fix_type = 53;

        m_ValueRefs.pose.position.x = 11;
        m_ValueRefs.pose.position.y = 12;
        m_ValueRefs.pose.position.z = 13;
        m_ValueRefs.pose.orientation.x = 14;
        m_ValueRefs.pose.orientation.y = 15;
        m_ValueRefs.pose.orientation.z = 16;

        m_ValueRefs.twist_body.linear.x = 24;
        m_ValueRefs.twist_body.linear.y = 25;
        m_ValueRefs.twist_body.linear.z = 26;
        m_ValueRefs.twist_body.angular.x = 27;
        m_ValueRefs.twist_body.angular.y = 28;
        m_ValueRefs.twist_body.angular.z = 29;

        m_ValueRefs.twist_ned.linear.x = 30;
        m_ValueRefs.twist_ned.linear.y = 31;
        m_ValueRefs.twist_ned.linear.z = 32;
        m_ValueRefs.twist_ned.angular.x = 33;
        m_ValueRefs.twist_ned.angular.y = 34;
        m_ValueRefs.twist_ned.angular.z = 35;

        m_ValueRefs.acceleration.linear.x = 36;
        m_ValueRefs.acceleration.linear.y = 37;
        m_ValueRefs.acceleration.linear.z = 38;
        m_ValueRefs.acceleration.angular.x = 39;
        m_ValueRefs.acceleration.angular.y = 40;
        m_ValueRefs.acceleration.angular.z = 41;
    }

    void initializeSignalMaps()
    {
        // Parameters
        m_StringSignals[m_ValueRefs.parameters.autonomy_machine_remote_ip] = "127.0.0.1";
        m_StringSignals[m_ValueRefs.parameters.mt_dp_remote_ip] = "127.0.0.1";
        m_StringSignals[m_ValueRefs.parameters.local_ip] = "0.0.0.0";
        m_StringSignals[m_ValueRefs.parameters.autonomy_machine_remote_port] = "16718";
        m_StringSignals[m_ValueRefs.parameters.mt_dp_remote_port] = "5097";
        m_BooleanSignals[m_ValueRefs.parameters.dump_messages] = false;
        m_RealSignals[m_ValueRefs.parameters.latitude0] = 63.4389029083;
        m_RealSignals[m_ValueRefs.parameters.longitude0] = 10.39908278;
        m_RealSignals[m_ValueRefs.parameters.height0] = 39.923;
        m_RealSignals[m_ValueRefs.parameters.dump_messages] = false;
        m_RealSignals[m_ValueRefs.parameters.hdop] = 0.0;
        m_RealSignals[m_ValueRefs.parameters.vdop] = 0.0;
        m_RealSignals[m_ValueRefs.parameters.gps_satellites] = 4;
        m_RealSignals[m_ValueRefs.parameters.glonass_satellites] = 0;
        m_RealSignals[m_ValueRefs.parameters.beidou_satellites] = 0;
        m_RealSignals[m_ValueRefs.parameters.galileo_satellites] = 0;
        m_RealSignals[m_ValueRefs.parameters.sbas_satellites] = 0;
        m_RealSignals[m_ValueRefs.parameters.g_force] = 0.0;
        m_RealSignals[m_ValueRefs.parameters.lat_stddev] = 0.0;
        m_RealSignals[m_ValueRefs.parameters.lon_stddev] = 0.0;
        m_RealSignals[m_ValueRefs.parameters.heigth_stddev] = 0.0;
        m_RealSignals[m_ValueRefs.parameters.fix_type] = 7;

        // Inputs signals
        m_RealSignals[m_ValueRefs.pose.position.x] = 0;
        m_RealSignals[m_ValueRefs.pose.position.y] = 0;
        m_RealSignals[m_ValueRefs.pose.position.z] = 0;
        m_RealSignals[m_ValueRefs.pose.orientation.x] = 0;
        m_RealSignals[m_ValueRefs.pose.orientation.y] = 0;
        m_RealSignals[m_ValueRefs.pose.orientation.z] = 0;

        m_RealSignals[m_ValueRefs.twist_body.linear.x] = 0;
        m_RealSignals[m_ValueRefs.twist_body.linear.y] = 0;
        m_RealSignals[m_ValueRefs.twist_body.linear.z] = 0;
        m_RealSignals[m_ValueRefs.twist_body.angular.x] = 0;
        m_RealSignals[m_ValueRefs.twist_body.angular.y] = 0;
        m_RealSignals[m_ValueRefs.twist_body.angular.z] = 0;

        m_RealSignals[m_ValueRefs.twist_ned.linear.x] = 0;
        m_RealSignals[m_ValueRefs.twist_ned.linear.y] = 0;
        m_RealSignals[m_ValueRefs.twist_ned.linear.z] = 0;
        m_RealSignals[m_ValueRefs.twist_ned.angular.x] = 0;
        m_RealSignals[m_ValueRefs.twist_ned.angular.y] = 0;
        m_RealSignals[m_ValueRefs.twist_ned.angular.z] = 0;

        m_RealSignals[m_ValueRefs.acceleration.linear.x] = 0;
        m_RealSignals[m_ValueRefs.acceleration.linear.y] = 0;
        m_RealSignals[m_ValueRefs.acceleration.linear.z] = 0;
        m_RealSignals[m_ValueRefs.acceleration.angular.x] = 0;
        m_RealSignals[m_ValueRefs.acceleration.angular.y] = 0;
        m_RealSignals[m_ValueRefs.acceleration.angular.z] = 0;
    }

    void ExitInitializationMode() override
    {
        std::shared_ptr<FileDumperT<ChronoTime, ChronoTime::TimeStruct>> file_dumper;
        if (m_BooleanSignals[m_ValueRefs.parameters.dump_messages]) {
            auto chrono_time = std::make_shared<ChronoTime>();
            file_dumper = std::make_shared<FileDumperT<ChronoTime, ChronoTime::TimeStruct>>("AdvancedNavigation.txt",
                                                                                            chrono_time.get());
        }

        auto autonomy_machine_ip = m_StringSignals[m_ValueRefs.parameters.autonomy_machine_remote_ip];
        auto autonomy_machine_port =
            string_to_uint16(m_StringSignals[m_ValueRefs.parameters.autonomy_machine_remote_port]);
        auto udp_connection_autonomy_machine =
            udp::Socket::setup_send_socket(autonomy_machine_port, autonomy_machine_ip, file_dumper);
        m_message_handler_autonomy_machine = std::make_shared<message_handlers::ByteMessageHandler>(udp_connection_autonomy_machine);

        auto mt_dp_ip = m_StringSignals[m_ValueRefs.parameters.mt_dp_remote_ip];
        auto mt_dp_port = string_to_uint16(m_StringSignals[m_ValueRefs.parameters.mt_dp_remote_port]);
        if (mt_dp_port != 0) {
            auto udp_connection_mt_dp = udp::Socket::setup_send_socket(mt_dp_port, mt_dp_ip, file_dumper);
            m_message_handler_mt_dp = std::make_shared<message_handlers::ByteMessageHandler>(udp_connection_mt_dp);
        }
    }

    void send_advanced_navigation_data(const double clock)
    {
        system_state_packet_t system_state_packet;
        satellites_packet_t satellites_packet;
        body_velocity_packet_t body_velocity_packet;

        create_system_state_packet(&system_state_packet, clock);
        create_satellites_packet(&satellites_packet);
        create_body_velocity_packet(&body_velocity_packet);

        auto an_packet_1 = encode_system_state_packet(&system_state_packet);
        auto an_packet_2 = encode_satellites_packet(&satellites_packet);
        auto an_packet_3 = encode_body_velocity_packet(&body_velocity_packet);

        // Encode an_packets
        an_packet_encode(an_packet_1);
        an_packet_encode(an_packet_2);
        an_packet_encode(an_packet_3);

        auto byte_msg_1 = create_byte_message(an_packet_1);
        auto byte_msg_2 = create_byte_message(an_packet_2);
        auto byte_msg_3 = create_byte_message(an_packet_3);

        m_message_handler_autonomy_machine->send_message(byte_msg_1);
        m_message_handler_autonomy_machine->send_message(byte_msg_2);
        m_message_handler_autonomy_machine->send_message(byte_msg_3);
        if (m_message_handler_mt_dp) {
            m_message_handler_mt_dp->send_message(byte_msg_1);
        }
        an_packet_free(&an_packet_1);
        an_packet_free(&an_packet_2);
        an_packet_free(&an_packet_3);
    }

    ByteMessage create_byte_message(an_packet_t* packet)
    {
        auto an_msg = ByteMessage{};
        auto size = an_packet_size(packet);
        an_msg.data.data.resize(size);
        memcpy(&an_msg.data.data[0], an_packet_pointer(packet), an_packet_size(packet));
        return an_msg;
    }

    void create_system_state_packet(system_state_packet_t* system_state_packet_ptr, const double clock)
    {
        system_state_packet_ptr->system_status.r = 0u;
        system_state_packet_ptr->system_status.b.accelerometer_over_range = 0u;
        system_state_packet_ptr->system_status.b.accelerometer_sensor_failure = 0u;
        system_state_packet_ptr->system_status.b.gnss_antenna_fault = 0u;
        system_state_packet_ptr->system_status.b.gyroscope_over_range = 0u;
        system_state_packet_ptr->system_status.b.gyroscope_sensor_failure = 0u;
        system_state_packet_ptr->system_status.b.high_voltage_alarm = 0u;
        system_state_packet_ptr->system_status.b.low_voltage_alarm = 0u;
        system_state_packet_ptr->system_status.b.magnetometer_over_range = 0u;
        system_state_packet_ptr->system_status.b.magnetometer_sensor_failure = 0u;
        system_state_packet_ptr->system_status.b.maximum_temperature_alarm = 0u;
        system_state_packet_ptr->system_status.b.minimum_temperature_alarm = 0u;
        system_state_packet_ptr->system_status.b.pressure_over_range = 0u;
        system_state_packet_ptr->system_status.b.pressure_sensor_failure = 0u;
        system_state_packet_ptr->system_status.b.serial_port_overflow_alarm = 0u;
        system_state_packet_ptr->system_status.b.system_failure = 0u;

        system_state_packet_ptr->filter_status.r = 0u;
        system_state_packet_ptr->filter_status.b.atmospheric_altitude_enabled = 1u;
        system_state_packet_ptr->filter_status.b.dual_antenna_heading_active = 1u;
        system_state_packet_ptr->filter_status.b.event1_flag = 1u;
        system_state_packet_ptr->filter_status.b.event2_flag = 1u;
        system_state_packet_ptr->filter_status.b.external_heading_active = 1u;
        system_state_packet_ptr->filter_status.b.external_position_active = 1u;
        system_state_packet_ptr->filter_status.b.external_velocity_active = 1u;
        system_state_packet_ptr->filter_status.b.gnss_fix_type =
            convert_to_adnav_fix(m_RealSignals[m_ValueRefs.parameters.fix_type]);
        system_state_packet_ptr->filter_status.b.heading_initialised = 1u;
        system_state_packet_ptr->filter_status.b.ins_filter_initialised = 1u;
        system_state_packet_ptr->filter_status.b.internal_gnss_enabled = 1u;
        system_state_packet_ptr->filter_status.b.orientation_filter_initialised = 1u;
        system_state_packet_ptr->filter_status.b.utc_time_initialised = 1u;
        system_state_packet_ptr->filter_status.b.velocity_heading_enabled = 1u;

        auto seconds = std::floor(clock);
        system_state_packet_ptr->unix_time_seconds = static_cast<uint32_t>(seconds);
        constexpr uint32_t sec_to_us_conversion_factor = 1e6;
        system_state_packet_ptr->microseconds = static_cast<uint32_t>((clock - seconds) * sec_to_us_conversion_factor);

        m_llh0 = Eigen::Vector3d{to_rad(m_RealSignals[m_ValueRefs.parameters.latitude0]),
                                 to_rad(m_RealSignals[m_ValueRefs.parameters.longitude0]),
                                 m_RealSignals[m_ValueRefs.parameters.height0]};
        auto ned =
            Eigen::Vector3d{m_RealSignals[m_ValueRefs.pose.position.x], m_RealSignals[m_ValueRefs.pose.position.y],
                            m_RealSignals[m_ValueRefs.pose.position.z]};
        auto llh = position_local2lla(ned, m_llh0);
        system_state_packet_ptr->latitude = llh(0);   // In radians
        system_state_packet_ptr->longitude = llh(1);  // In radians
        system_state_packet_ptr->height = llh(2);

        system_state_packet_ptr->velocity[0] =
            static_cast<float>(m_RealSignals[m_ValueRefs.twist_ned.linear.x]);  // North
        system_state_packet_ptr->velocity[1] =
            static_cast<float>(m_RealSignals[m_ValueRefs.twist_ned.linear.y]);  // East
        system_state_packet_ptr->velocity[2] =
            static_cast<float>(m_RealSignals[m_ValueRefs.twist_ned.linear.z]);  // Down

        system_state_packet_ptr->body_acceleration[0] =
            static_cast<float>(m_RealSignals[m_ValueRefs.acceleration.linear.x]);
        system_state_packet_ptr->body_acceleration[1] =
            static_cast<float>(m_RealSignals[m_ValueRefs.acceleration.linear.y]);
        system_state_packet_ptr->body_acceleration[2] =
            static_cast<float>(m_RealSignals[m_ValueRefs.acceleration.linear.z]);

        system_state_packet_ptr->g_force = static_cast<float>(m_RealSignals[m_ValueRefs.parameters.g_force]);
        system_state_packet_ptr->orientation[0] =
            static_cast<float>(m_RealSignals[m_ValueRefs.pose.orientation.x]);  // Roll
        system_state_packet_ptr->orientation[1] =
            static_cast<float>(m_RealSignals[m_ValueRefs.pose.orientation.y]);  // Pitch

        auto heading = m_RealSignals[m_ValueRefs.pose.orientation.z];  // Heading
        // Wrap to 0-360 degrees
        if (heading = inf2pipi(heading); heading < 0.0) {
            heading += 2.0 * M_PI;
        }
        system_state_packet_ptr->orientation[2] = static_cast<float>(heading);

        system_state_packet_ptr->angular_velocity[0] =
            static_cast<float>(m_RealSignals[m_ValueRefs.twist_ned.angular.x]);
        system_state_packet_ptr->angular_velocity[1] =
            static_cast<float>(m_RealSignals[m_ValueRefs.twist_ned.angular.y]);
        system_state_packet_ptr->angular_velocity[2] =
            static_cast<float>(m_RealSignals[m_ValueRefs.twist_ned.angular.z]);
        system_state_packet_ptr->standard_deviation[0] =
            static_cast<float>(m_RealSignals[m_ValueRefs.parameters.lat_stddev]);
        system_state_packet_ptr->standard_deviation[1] =
            static_cast<float>(m_RealSignals[m_ValueRefs.parameters.lon_stddev]);
        system_state_packet_ptr->standard_deviation[2] =
            static_cast<float>(m_RealSignals[m_ValueRefs.parameters.heigth_stddev]);
    };

    void create_satellites_packet(satellites_packet_t* satellites_packet_ptr)
    {
        satellites_packet_ptr->hdop = static_cast<float>(m_RealSignals[m_ValueRefs.parameters.hdop]);
        satellites_packet_ptr->vdop = static_cast<float>(m_RealSignals[m_ValueRefs.parameters.vdop]);
        satellites_packet_ptr->gps_satellites =
            static_cast<uint8_t>(m_RealSignals[m_ValueRefs.parameters.gps_satellites]);
        satellites_packet_ptr->glonass_satellites =
            static_cast<uint8_t>(m_RealSignals[m_ValueRefs.parameters.glonass_satellites]);
        satellites_packet_ptr->beidou_satellites =
            static_cast<uint8_t>(m_RealSignals[m_ValueRefs.parameters.beidou_satellites]);
        satellites_packet_ptr->galileo_satellites =
            static_cast<uint8_t>(m_RealSignals[m_ValueRefs.parameters.galileo_satellites]);
        satellites_packet_ptr->sbas_satellites =
            static_cast<uint8_t>(m_RealSignals[m_ValueRefs.parameters.sbas_satellites]);
    }

    void create_body_velocity_packet(body_velocity_packet_t* body_velocity_packet_ptr)
    {
        body_velocity_packet_ptr->velocity[0] = static_cast<float>(m_RealSignals[m_ValueRefs.twist_body.linear.x]);
        body_velocity_packet_ptr->velocity[1] = static_cast<float>(m_RealSignals[m_ValueRefs.twist_body.linear.y]);
        body_velocity_packet_ptr->velocity[2] = static_cast<float>(m_RealSignals[m_ValueRefs.twist_body.linear.z]);
    }

    uint16_t convert_to_adnav_fix(double adnav_fix_type)
    {
        auto fix_type = static_cast<uint16_t>(adnav_fix_type);
        if (fix_type < static_cast<uint16_t>(gnss_fix_type_e::gnss_fix_none) ||
            fix_type > static_cast<uint16_t>(gnss_fix_type_e::gnss_fix_rtk_fixed)) {
            std::ostringstream oss;
            oss << "Received unknown fix value! (" << fix_type << ")";
            std::cerr << oss.str() << std::endl;
            fix_type = static_cast<uint16_t>(gnss_fix_type_e::gnss_fix_none);
        }
        return fix_type;
    }

    void Terminate() override
    {
        m_message_handler_autonomy_machine->shutdown();
        if (m_message_handler_mt_dp) {
            m_message_handler_mt_dp->shutdown();
        }
    }

    void Reset() override
    {
        initializeSignalMaps();
        m_message_handler_autonomy_machine->shutdown();
        if (m_message_handler_mt_dp) {
            m_message_handler_mt_dp->shutdown();
        }
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

    bool DoStep(cppfmu::FMIReal currentCommunicationPoint, cppfmu::FMIReal /*dt*/, cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal&
                /*endOfStep*/) override
    {
        send_advanced_navigation_data(currentCommunicationPoint);
        return true;
    }

   private:
    Eigen::Vector3d m_llh0;
    std::shared_ptr<message_handlers::ByteMessageHandler> m_message_handler_autonomy_machine;
    std::shared_ptr<message_handlers::ByteMessageHandler> m_message_handler_mt_dp;

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
    cppfmu::FMIString /*instanceName*/, cppfmu::FMIString /*fmuGUID*/, cppfmu::FMIString fmuResourceLocation,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger /*logger*/)
{
    // if (std::strcmp(fmuGUID, FMU_UUID) != 0) {
    //     throw std::runtime_error("FMU GUID mismatch");
    // }
    return cppfmu::AllocateUnique<FmuDpInterface>(memory, fmuResourceLocation);
}
