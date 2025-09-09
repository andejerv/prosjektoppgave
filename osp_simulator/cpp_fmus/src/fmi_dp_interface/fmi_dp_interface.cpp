
#include "zeabuz/common/io/message_handler_descriptions.hpp"
#ifdef WIN32
#include <SDKDDKVer.h>
#endif

#include <cppfmu_cs.hpp>
#include <string>
#include <zeabuz/common/io/file_dumper_t.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_external_control_enable_cmd.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_mode_selector.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_vessel_data.hpp>
#include <zeabuz/common/io/udp_connection.hpp>
#include <zeabuz/common/utilities/conversions.hpp>
#include <zeabuz/common/utilities/enums.hpp>
#include <zeabuz/common/utilities/math.hpp>
#include <zeabuz/common/utilities/pretty_class_macro.hpp>
#include <zeabuz/common/utilities/string.hpp>

#include "dp_interface.hpp"

using namespace zeabuz::common::io::messages::mtdp;
using namespace zeabuz::common::io;

struct Vector3ValueRef
{
    Vector3ValueRef() : x(UINT_MAX), y(UINT_MAX), z(UINT_MAX){};
    cppfmu::FMIValueReference x;
    cppfmu::FMIValueReference y;
    cppfmu::FMIValueReference z;
};

struct DPReferenceValueRef
{
    Vector3ValueRef northEastHeading;
    Vector3ValueRef northEastHeadingVel;
    Vector3ValueRef northEastHeadingAccel;
    cppfmu::FMIValueReference surge_integrator_mode;
    cppfmu::FMIValueReference sway_integrator_mode;
    cppfmu::FMIValueReference heading_integrator_mode;
};

struct AutopilotReferenceValueRef
{
    cppfmu::FMIValueReference heading;
    cppfmu::FMIValueReference yaw_rate;
    cppfmu::FMIValueReference yaw_acc;
    cppfmu::FMIValueReference surge_speed;
    cppfmu::FMIValueReference surge_acc;
    cppfmu::FMIValueReference surge_integrator_mode;
    cppfmu::FMIValueReference heading_integrator_mode;
};

struct JoystickCommandValueRef
{
    cppfmu::FMIValueReference surge;
    cppfmu::FMIValueReference sway;
    cppfmu::FMIValueReference yaw;
};

struct OutputValueRef
{
    DPReferenceValueRef dpReference;
    AutopilotReferenceValueRef autopilotReference;
    JoystickCommandValueRef joystickCommand;
    cppfmu::FMIValueReference setControlMode;
    cppfmu::FMIValueReference autonomyEnable;
};

struct PoseValueRef
{
    Vector3ValueRef position;
    Vector3ValueRef orientation;
};

struct InputValueRef
{
    // booleams
    cppfmu::FMIValueReference readyForAutonomy;
    cppfmu::FMIValueReference autonomyEnabled;
    // strings
    cppfmu::FMIValueReference controlMode;
    // pose
    PoseValueRef pose;
};

struct ParametersValueRef
{
    cppfmu::FMIValueReference remote_ip;
    cppfmu::FMIValueReference local_ip;
    cppfmu::FMIValueReference remote_port;
    cppfmu::FMIValueReference local_port;
    cppfmu::FMIValueReference latitude0;
    cppfmu::FMIValueReference longitude0;
    cppfmu::FMIValueReference height0;
    cppfmu::FMIValueReference publish_time_step;
    cppfmu::FMIValueReference publish_dp_mode_time_step;
    cppfmu::FMIValueReference dump_messages;
};

struct ValueReferences
{
    // booleans
    InputValueRef input;
    OutputValueRef output;
    ParametersValueRef parameters;
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
        // parameters
        m_ValueRefs.parameters.remote_ip = 0;
        m_ValueRefs.parameters.local_ip = 1;
        m_ValueRefs.parameters.remote_port = 2;
        m_ValueRefs.parameters.local_port = 3;
        m_ValueRefs.parameters.latitude0 = 21;
        m_ValueRefs.parameters.longitude0 = 22;
        m_ValueRefs.parameters.height0 = 23;
        m_ValueRefs.parameters.publish_time_step = 24;
        m_ValueRefs.parameters.publish_dp_mode_time_step = 41;
        m_ValueRefs.parameters.dump_messages = 25;

        // Output Signals
        m_ValueRefs.output.autonomyEnable = 4;
        m_ValueRefs.output.setControlMode = 5;

        m_ValueRefs.output.joystickCommand.surge = 6;
        m_ValueRefs.output.joystickCommand.sway = 7;
        m_ValueRefs.output.joystickCommand.yaw = 8;

        m_ValueRefs.output.dpReference.northEastHeading.x = 9;
        m_ValueRefs.output.dpReference.northEastHeading.y = 10;
        m_ValueRefs.output.dpReference.northEastHeading.z = 11;
        m_ValueRefs.output.dpReference.northEastHeadingVel.x = 12;
        m_ValueRefs.output.dpReference.northEastHeadingVel.y = 13;
        m_ValueRefs.output.dpReference.northEastHeadingVel.z = 14;
        m_ValueRefs.output.dpReference.northEastHeadingAccel.x = 15;
        m_ValueRefs.output.dpReference.northEastHeadingAccel.y = 16;
        m_ValueRefs.output.dpReference.northEastHeadingAccel.z = 17;
        m_ValueRefs.output.dpReference.surge_integrator_mode = 36;
        m_ValueRefs.output.dpReference.sway_integrator_mode = 37;
        m_ValueRefs.output.dpReference.heading_integrator_mode = 38;

        m_ValueRefs.output.autopilotReference.heading = 31;
        m_ValueRefs.output.autopilotReference.yaw_rate = 32;
        m_ValueRefs.output.autopilotReference.yaw_acc = 33;
        m_ValueRefs.output.autopilotReference.surge_speed = 34;
        m_ValueRefs.output.autopilotReference.surge_acc = 35;
        m_ValueRefs.output.autopilotReference.surge_integrator_mode = 39;
        m_ValueRefs.output.autopilotReference.heading_integrator_mode = 40;

        // Input signals
        m_ValueRefs.input.controlMode = 18;
        m_ValueRefs.input.readyForAutonomy = 19;
        m_ValueRefs.input.autonomyEnabled = 20;

        m_ValueRefs.input.pose.position.x = 25;
        m_ValueRefs.input.pose.position.y = 26;
        m_ValueRefs.input.pose.position.z = 27;
        m_ValueRefs.input.pose.orientation.x = 28;
        m_ValueRefs.input.pose.orientation.y = 29;
        m_ValueRefs.input.pose.orientation.z = 30;
    }

    void initializeSignalMaps()
    {
        // parameters
        m_StringSignals[m_ValueRefs.parameters.remote_ip] = "127.0.0.1";
        m_StringSignals[m_ValueRefs.parameters.local_ip] = "0.0.0.0";
        m_StringSignals[m_ValueRefs.parameters.remote_port] = "5096";
        m_StringSignals[m_ValueRefs.parameters.local_port] = "5095";
        m_RealSignals[m_ValueRefs.parameters.latitude0] = 63.4389029083;
        m_RealSignals[m_ValueRefs.parameters.longitude0] = 10.39908278;
        m_RealSignals[m_ValueRefs.parameters.height0] = 39.923;
        m_RealSignals[m_ValueRefs.parameters.publish_time_step] = 1.0;
        m_RealSignals[m_ValueRefs.parameters.publish_dp_mode_time_step] = 0.1;
        m_BooleanSignals[m_ValueRefs.parameters.dump_messages] = false;

        // Input signals
        m_BooleanSignals[m_ValueRefs.input.readyForAutonomy] = false;
        m_BooleanSignals[m_ValueRefs.input.autonomyEnabled] = false;
        m_StringSignals[m_ValueRefs.input.controlMode] = "STANDBY";
        m_RealSignals[m_ValueRefs.input.pose.position.x] = 0;
        m_RealSignals[m_ValueRefs.input.pose.position.y] = 0;
        m_RealSignals[m_ValueRefs.input.pose.position.z] = 0;
        m_RealSignals[m_ValueRefs.input.pose.orientation.x] = 0;
        m_RealSignals[m_ValueRefs.input.pose.orientation.y] = 0;
        m_RealSignals[m_ValueRefs.input.pose.orientation.z] = 0;

        // Output
        m_BooleanSignals[m_ValueRefs.output.autonomyEnable] = false;
        m_StringSignals[m_ValueRefs.output.setControlMode] = "STANDBY";

        m_RealSignals[m_ValueRefs.output.joystickCommand.surge] = 0;
        m_RealSignals[m_ValueRefs.output.joystickCommand.sway] = 0;
        m_RealSignals[m_ValueRefs.output.joystickCommand.yaw] = 0;

        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeading.x] = 0;
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeading.y] = 0;
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeading.z] = 0;
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeadingVel.x] = 0;
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeadingVel.y] = 0;
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeadingVel.z] = 0;
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeadingAccel.x] = 0;
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeadingAccel.y] = 0;
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeadingAccel.z] = 0;
        m_RealSignals[m_ValueRefs.output.dpReference.surge_integrator_mode] = 0;
        m_RealSignals[m_ValueRefs.output.dpReference.sway_integrator_mode] = 0;
        m_RealSignals[m_ValueRefs.output.dpReference.heading_integrator_mode] = 0;

        m_RealSignals[m_ValueRefs.output.autopilotReference.heading] = 0;
        m_RealSignals[m_ValueRefs.output.autopilotReference.yaw_rate] = 0;
        m_RealSignals[m_ValueRefs.output.autopilotReference.yaw_acc] = 0;
        m_RealSignals[m_ValueRefs.output.autopilotReference.surge_speed] = 0;
        m_RealSignals[m_ValueRefs.output.autopilotReference.surge_acc] = 0;
        m_RealSignals[m_ValueRefs.output.autopilotReference.surge_integrator_mode] = 0;
        m_RealSignals[m_ValueRefs.output.autopilotReference.heading_integrator_mode] = 0;
    }

    void ExitInitializationMode() override
    {
        std::shared_ptr<FileDumperT<ChronoTime, ChronoTime::TimeStruct>> file_dumper;
        if (m_BooleanSignals[m_ValueRefs.parameters.dump_messages]) {
            auto chrono_time = std::make_shared<ChronoTime>();
            file_dumper = std::make_shared<FileDumperT<ChronoTime, ChronoTime::TimeStruct>>("MTDPInterfaceOSP.txt",
                                                                                            chrono_time.get());
        }

        auto udp_connection = zeabuz::common::io::udp::Socket::setup_p2p_socket(
            zeabuz::common::utilities::string::string_to_uint16(m_StringSignals[m_ValueRefs.parameters.remote_port]),
            m_StringSignals[m_ValueRefs.parameters.remote_ip],
            zeabuz::common::utilities::string::string_to_uint16(m_StringSignals[m_ValueRefs.parameters.local_port]),
            m_StringSignals[m_ValueRefs.parameters.local_ip], file_dumper);
        m_message_handler = std::make_shared<message_handlers::MTNMEAMessageHandler>(udp_connection);
        m_DpInterface = std::make_unique<MarineTechnologiesDpInterface>(m_message_handler);
        m_Llh0 = Eigen::Vector3d{m_RealSignals[m_ValueRefs.parameters.latitude0],
                                 m_RealSignals[m_ValueRefs.parameters.longitude0],
                                 m_RealSignals[m_ValueRefs.parameters.height0]};
    }

    // Output Signals
    void updateOutputSignals()
    {
        updateExternalControlInputSignals();
        updateAutopilotControlInputSignals();
        updateAutonomyEnableSignal();
        updateSetModeSignal();
        updateJoystickCommandSignals();
    }

    void updateExternalControlInputSignals()
    {
        auto externalControlInput = m_DpInterface->getReceivedMTCMDExternalControlInput().m_Object.data;
        Eigen::Vector3d llaDeg{externalControlInput.lat, externalControlInput.lon, m_Llh0(2)};
        Eigen::Vector3d lla{zeabuz::common::utilities::math::to_rad(llaDeg(0)),
                            zeabuz::common::utilities::math::to_rad(llaDeg(1)), m_Llh0(2)};
        Eigen::Vector3d lla0{zeabuz::common::utilities::math::to_rad(m_Llh0(0)),
                             zeabuz::common::utilities::math::to_rad(m_Llh0(1)), m_Llh0(2)};
        Eigen::Vector3d localNed = zeabuz::common::utilities::conversions::position_lla2local(lla, lla0);

        Eigen::Matrix3d rotMat = zeabuz::common::utilities::math::heading_to_rotation_matrix(
            zeabuz::common::utilities::math::to_rad(externalControlInput.Hdg));
        Eigen::Vector3d nu{externalControlInput.surgeVel, externalControlInput.swayVel,
                           zeabuz::common::utilities::math::to_rad(externalControlInput.yawVel)};
        Eigen::Vector3d nu_dot{externalControlInput.surgeAcc, externalControlInput.swayAcc,
                               zeabuz::common::utilities::math::to_rad(externalControlInput.yawAcc)};
        Eigen::Vector3d eta_dot = rotMat * nu;
        Eigen::Vector3d eta_ddot = rotMat * nu_dot;

        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeading.x] = localNed(0);
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeading.y] = localNed(1);
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeading.z] =
            zeabuz::common::utilities::math::to_rad(externalControlInput.Hdg);
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeadingVel.x] = eta_dot(0);
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeadingVel.y] = eta_dot(1);
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeadingVel.z] = eta_dot(2);
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeadingAccel.x] = eta_ddot(0);
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeadingAccel.y] = eta_ddot(1);
        m_RealSignals[m_ValueRefs.output.dpReference.northEastHeadingAccel.z] = eta_ddot(2);
        m_RealSignals[m_ValueRefs.output.dpReference.surge_integrator_mode] =
            zeabuz::common::utilities::enums::get_underlying_value(externalControlInput.surge_integrator_mode);
        m_RealSignals[m_ValueRefs.output.dpReference.sway_integrator_mode] =
            zeabuz::common::utilities::enums::get_underlying_value(externalControlInput.sway_integrator_mode);
        m_RealSignals[m_ValueRefs.output.dpReference.heading_integrator_mode] =
            zeabuz::common::utilities::enums::get_underlying_value(externalControlInput.heading_integrator_mode);
    }

    void updateAutopilotControlInputSignals()
    {
        auto autopilot_ref = m_DpInterface->getReceivedMTCMDAutopilotControlInput().m_Object.data;
        m_RealSignals[m_ValueRefs.output.autopilotReference.heading] =
            zeabuz::common::utilities::math::to_rad(autopilot_ref.heading_ref);
        m_RealSignals[m_ValueRefs.output.autopilotReference.yaw_rate] =
            zeabuz::common::utilities::math::to_rad(autopilot_ref.yaw_rate_ref);
        m_RealSignals[m_ValueRefs.output.autopilotReference.yaw_acc] =
            zeabuz::common::utilities::math::to_rad(autopilot_ref.yaw_acc_ref);
        m_RealSignals[m_ValueRefs.output.autopilotReference.surge_speed] = autopilot_ref.surge_speed_ref;
        m_RealSignals[m_ValueRefs.output.autopilotReference.surge_acc] = autopilot_ref.surge_acc_ref;
        m_RealSignals[m_ValueRefs.output.autopilotReference.surge_integrator_mode] =
            zeabuz::common::utilities::enums::get_underlying_value(autopilot_ref.surge_integrator_mode);
        m_RealSignals[m_ValueRefs.output.autopilotReference.heading_integrator_mode] =
            zeabuz::common::utilities::enums::get_underlying_value(autopilot_ref.heading_integrator_mode);
    }

    void updateAutonomyEnableSignal()
    {
        using cmd_mode = MTExternalControlEnableCMDStruct::AutonomyEnable;

        m_should_set_emergency_dp = false;
        auto autonomy_enabled = m_DpInterface->getReceivedMTCMDExternalControlEnableCommand().m_Object.data;
        if (autonomy_enabled.autonomyEnabled == cmd_mode::AUTONOMY_MODE_ON) {
            m_BooleanSignals[m_ValueRefs.output.autonomyEnable] = true;
        }
        else if (autonomy_enabled.autonomyEnabled == cmd_mode::AUTONOMY_MODE_OFF) {
            m_BooleanSignals[m_ValueRefs.output.autonomyEnable] = false;
        }

        else if (autonomy_enabled.autonomyEnabled == cmd_mode::EMERGENCY_DP) {
            m_should_set_emergency_dp = true;
        }
    }

    void updateSetModeSignal()
    {
        // Only accept SetMode commands when autonomy is enabled
        if (m_BooleanSignals[m_ValueRefs.input.autonomyEnabled]) {
            auto mode = std::string_view{};
            auto mode_selector = m_DpInterface->getReceivedMTCMDModeSelector().m_Object.data;
            if (mode_selector.mode == MTModeSelectorStruct::MainMode::TRANSIT) {
                if (mode_selector.subMode.transitSubMode == MTModeSelectorStruct::TransitSubMode::AUTOPILOT) {
                    mode = zeabuz::common::utilities::enums::enum_name(mode_selector.subMode.transitSubMode);
                }
                else {
                    std::stringstream errorMessage;
                    errorMessage << "Error in updateSetModeSignal. Invalid transitSubMode specified";
                    std::cerr << errorMessage.str() << std::endl;
                    mode = "STANDBY";
                }
            }
            else {
                mode = zeabuz::common::utilities::enums::enum_name(mode_selector.mode);
            }

            if (m_should_set_emergency_dp) {
                m_BooleanSignals[m_ValueRefs.output.autonomyEnable] = false;
                m_StringSignals[m_ValueRefs.output.setControlMode] = "DP";
            }
            else {
                m_StringSignals[m_ValueRefs.output.setControlMode] = mode.data();
            }
        }
    }

    void updateJoystickCommandSignals()
    {
        auto joystickCommands = m_DpInterface->getReceivedMTCMDJoystickCommand().m_Object.data;

        m_RealSignals[m_ValueRefs.output.joystickCommand.surge] = joystickCommands.Surge;
        m_RealSignals[m_ValueRefs.output.joystickCommand.sway] = joystickCommands.Sway;
        m_RealSignals[m_ValueRefs.output.joystickCommand.yaw] = joystickCommands.Yaw;
    }

    // Input Signals
    void updateInputSignals()
    {
        // Mode selector is updated individually at a higher rate in the main loop
        // updateModeSelectorSignal();
        updateExternalControlEnableFeedbackSignal();
        updateVesselDataSignal();
        updateReferenceSystemEnableDisableSignal();
        updateSensorEnableDisableSignal();
        updateThrusterSystemEnableDisableSignal();
        updateAlarmLimitsSignal();
        updateDPClassSignal();
        updateExternalControlInputSignal();
        updateGainSettingsSignal();
        updateHeadingSetpointSignal();
        updateJoystickCommandSignal();
        updateJoystickSettingsSignal();
        updatePositionSetpointSignal();
    }

    void updateExternalControlEnableFeedbackSignal()
    {
        MTExternalControlEnableFBA msg{};
        msg.m_Object.data.readyForAutonomy = m_BooleanSignals[m_ValueRefs.input.readyForAutonomy];
        using fba_mode = MTExternalControlEnableFBAStruct::AutonomyEnable;
        msg.m_Object.data.autonomyEnabled = m_BooleanSignals[m_ValueRefs.input.autonomyEnabled]
                                                ? fba_mode::AUTONOMY_MODE_ON
                                                : fba_mode::AUTONOMY_MODE_OFF;

        m_message_handler->send_message(msg);
    }

    void updateModeSelectorSignal()
    {
        using type = MTModeSelectorStruct;
        auto msg = MTModeSelectorFBA{};

        auto mode_str = m_StringSignals[m_ValueRefs.input.controlMode];
        if (mode_str == zeabuz::common::utilities::enums::enum_name(type::TransitSubMode::AUTOPILOT)) {
            msg.m_Object.data.mode = type::MainMode::TRANSIT;
            msg.m_Object.data.subMode.transitSubMode = type::TransitSubMode::AUTOPILOT;
            msg.m_Object.data.HeadingControl = true;
        }
        else {
            msg.m_Object.data.mode = zeabuz::common::utilities::enums::enum_cast<type::MainMode>(mode_str.c_str());
            if (msg.m_Object.data.mode == type::MainMode::DP) {
                msg.m_Object.data.AlongshipControl = true;
                msg.m_Object.data.AthwartshipsControl = true;
                msg.m_Object.data.HeadingControl = true;
            }
        }
        m_message_handler->send_message(msg);
    }

    void updateVesselDataSignal()
    {
        MTVesselDataFBA msg{};
        Eigen::Vector3d local_position = Eigen::Vector3d{m_RealSignals[m_ValueRefs.input.pose.position.x],
                                                         m_RealSignals[m_ValueRefs.input.pose.position.y],
                                                         m_RealSignals[m_ValueRefs.input.pose.position.z]};
        Eigen::Vector3d llh0{zeabuz::common::utilities::math::to_rad(m_Llh0(0)),
                             zeabuz::common::utilities::math::to_rad(m_Llh0(1)), m_Llh0(2)};
        Eigen::Vector3d llh = zeabuz::common::utilities::conversions::position_local2lla(local_position, llh0);
        Eigen::Vector3d llh_deg = Eigen::Vector3d{zeabuz::common::utilities::math::to_deg(llh(0)),
                                                  zeabuz::common::utilities::math::to_deg(llh(1)),
                                                  zeabuz::common::utilities::math::to_deg(llh(2))};

        // TODO: Expand as part of user story 1407 on the board, fill in all data fields
        msg.m_Object.data.Latitude = llh_deg(0);
        msg.m_Object.data.Longitude = llh_deg(1);
        msg.m_Object.data.Heading =
            zeabuz::common::utilities::math::to_deg(m_RealSignals[m_ValueRefs.input.pose.orientation.z]);
        m_message_handler->send_message(msg);
    }

    void updateReferenceSystemEnableDisableSignal()
    {
        // TODO: User story 1407, use configuration and not hardcoded values
        MTReferenceSystemEnableDisableFBA msg{};
        msg.m_Object.data.data = {{1, true}, {2, false}};
        m_message_handler->send_message(msg);
    }

    void updateSensorEnableDisableSignal()
    {
        // TODO: User story 1407, use configuration and not hardcoded values
        MTSensorEnableDisableFBA msg{};

        msg.m_Object.data.data = {{101, {MTSensorEnableDisableStruct::DeviceType::GYRO, 1, true}},
                                  {201, {MTSensorEnableDisableStruct::DeviceType::WIND, 1, true}},
                                  {401, {MTSensorEnableDisableStruct::DeviceType::SPEED, 1, true}},
                                  {501, {MTSensorEnableDisableStruct::DeviceType::ROT, 1, true}}};
        m_message_handler->send_message(msg);
    }

    void updateThrusterSystemEnableDisableSignal()
    {
        // TODO: User story 1407, use configuration and not hardcoded values
        MTThrusterEnableDisableFBA msg{};
        msg.m_Object.data.data = {{1, std::vector<bool>{true, false, false, false}},
                                  {2, std::vector<bool>{true, false, false, false}},
                                  {3, std::vector<bool>{true, false, false, false}},
                                  {4, std::vector<bool>{true, false, false, false}}};
        m_message_handler->send_message(msg);
    }

    // TODO: User story 1407, expand these. For current use the content for these messages are DONT CARE.
    // This is to keep the dp interface condition monitor happy with receiveing all messages at the given rate.
    void updateAlarmLimitsSignal()
    {
        MTAlarmLimitsFBA msg{};
        m_message_handler->send_message(msg);
    }

    void updateDPClassSignal()
    {
        MTDPClassFBA msg{};
        m_message_handler->send_message(msg);
    }

    void updateExternalControlInputSignal()
    {
        MTExternalControlInputFBA msg{};
        m_message_handler->send_message(msg);
    }

    void updateGainSettingsSignal()
    {
        MTGainSettingsFBA msg{};
        m_message_handler->send_message(msg);
    }

    void updateHeadingSetpointSignal()
    {
        MTHeadingSetpointFBA msg{};
        m_message_handler->send_message(msg);
    }

    void updateJoystickCommandSignal()
    {
        MTJoystickCommandFBA msg{};
        m_message_handler->send_message(msg);
    }

    void updateJoystickSettingsSignal()
    {
        MTJoystickSettingsFBA msg{};
        m_message_handler->send_message(msg);
    }

    void updatePositionSetpointSignal()
    {
        MTPositionSetpointFBA msg{};
        m_message_handler->send_message(msg);
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

    bool check_shall_publish(const double current_time, const double dt, const double publish_time_step)
    {
        const auto delta = dt / 2.0;
        const auto remainder = std::fmod(current_time, publish_time_step);

        if ((remainder < delta) || (remainder > publish_time_step - delta)) {
            return true;
        }
        else{
            return false;

        }
    }

    bool DoStep(cppfmu::FMIReal currentCommunicationPoint, cppfmu::FMIReal dt, cppfmu::FMIBoolean /*newStep*/,
                cppfmu::FMIReal&
                /*endOfStep*/) override
    {
        updateOutputSignals();
        if (check_shall_publish(currentCommunicationPoint, dt, m_RealSignals[m_ValueRefs.parameters.publish_dp_mode_time_step])) {
            updateModeSelectorSignal();
        }
        if (check_shall_publish(currentCommunicationPoint, dt, m_RealSignals[m_ValueRefs.parameters.publish_time_step])) {
            updateInputSignals();
        }
        return true;
    }

   private:
    Eigen::Vector3d m_Llh0;
    std::shared_ptr<message_handlers::MTNMEAMessageHandler> m_message_handler;
    std::unique_ptr<MarineTechnologiesDpInterface> m_DpInterface;

    std::string _resource_location;

    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIReal> m_RealSignals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIInteger> m_IntegerSignals;
    std::unordered_map<cppfmu::FMIValueReference, std::string> m_StringSignals;
    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIBoolean> m_BooleanSignals;

    ValueReferences m_ValueRefs;
    bool m_should_set_emergency_dp{false};
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
