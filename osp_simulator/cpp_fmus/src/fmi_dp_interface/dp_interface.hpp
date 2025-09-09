#ifndef MARINETECHNOLOGIESDPINTERFACE_HPP
#define MARINETECHNOLOGIESDPINTERFACE_HPP
#include <mutex>
#include <zeabuz/common/io/message_handler.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_alarm_limits.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_autopilot_control_input.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_dp_class.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_external_control_enable_cmd.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_external_control_enable_fba.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_external_control_input.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_gain_settings.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_heading_setpoint.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_joystick_command.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_joystick_settings.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_mode_selector.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_position_setpoint.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_reference_system_enable_disable.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_sensor_enable_disable.hpp>
#include <zeabuz/common/io/messages/mtdp/mt_thruster_enable_disable.hpp>

#include "zeabuz/common/io/message_handler_descriptions.hpp"

class MarineTechnologiesDpInterface
{
   public:
    MarineTechnologiesDpInterface(
        std::shared_ptr<zeabuz::common::io::message_handlers::MTNMEAMessageHandler> messageHandler);

    // zeabuz::common::io::messages::mtdp::MTDP CMD
    void handleMTCMDJoystickCommand(const zeabuz::common::io::messages::mtdp::MTJoystickCommandCMD& message);
    void handleMTCMDThrusterEnableDisable(
        const zeabuz::common::io::messages::mtdp::MTThrusterEnableDisableCMD& message);
    void handleMTCMDModeSelector(const zeabuz::common::io::messages::mtdp::MTModeSelectorCMD& message);
    void handleMTCMDHeadingSetpoint(const zeabuz::common::io::messages::mtdp::MTHeadingSetpointCMD& message);
    void handleMTCMDPositionSetpoint(const zeabuz::common::io::messages::mtdp::MTPositionSetpointCMD& message);
    void handleMTCMDRefSysEnableDisable(
        const zeabuz::common::io::messages::mtdp::MTReferenceSystemEnableDisableCMD& message);
    void handleMTCMDSensorEnableDisable(const zeabuz::common::io::messages::mtdp::MTSensorEnableDisableCMD& message);
    void handleMTCMDGainSettings(const zeabuz::common::io::messages::mtdp::MTGainSettingsCMD& message);
    void handleMTCMDJoystickSettings(const zeabuz::common::io::messages::mtdp::MTJoystickSettingsCMD& message);
    void handleMTCMDAlarmLimits(const zeabuz::common::io::messages::mtdp::MTAlarmLimitsCMD& message);
    void handleMTCMDDPClass(const zeabuz::common::io::messages::mtdp::MTDPClassCMD& message);
    void handleMTCMDExternalControlInput(const zeabuz::common::io::messages::mtdp::MTExternalControlInputCMD& message);
    void handleMTCMDExternalControlEnableCommand(
        const zeabuz::common::io::messages::mtdp::MTExternalControlEnableCMD& message);
    void handleMTCMDAutopilotControlInput(
        const zeabuz::common::io::messages::mtdp::MTAutopilotControlInputCMD& message);

    zeabuz::common::io::messages::mtdp::MTJoystickCommandCMD getReceivedMTCMDJoystickCommand() const;
    zeabuz::common::io::messages::mtdp::MTThrusterEnableDisableCMD getReceivedMTCMDThrusterEnableDisable() const;
    zeabuz::common::io::messages::mtdp::MTModeSelectorCMD getReceivedMTCMDModeSelector() const;
    zeabuz::common::io::messages::mtdp::MTHeadingSetpointCMD getReceivedMTCMDHeadingSetpoint() const;
    zeabuz::common::io::messages::mtdp::MTPositionSetpointCMD getReceivedMTCMDPositionSetpoint() const;
    zeabuz::common::io::messages::mtdp::MTReferenceSystemEnableDisableCMD getReceivedMTCMDRefSysEnableDisable() const;
    zeabuz::common::io::messages::mtdp::MTSensorEnableDisableCMD getReceivedMTCMDSensorEnableDisable() const;
    zeabuz::common::io::messages::mtdp::MTGainSettingsCMD getReceivedMTCMDGainSettings() const;
    zeabuz::common::io::messages::mtdp::MTJoystickSettingsCMD getReceivedMTCMDJoystickSettings() const;
    zeabuz::common::io::messages::mtdp::MTAlarmLimitsCMD getReceivedMTCMDAlarmLimits() const;
    zeabuz::common::io::messages::mtdp::MTDPClassCMD getReceivedMTCMDDPClass() const;
    zeabuz::common::io::messages::mtdp::MTExternalControlInputCMD getReceivedMTCMDExternalControlInput() const;
    zeabuz::common::io::messages::mtdp::MTExternalControlEnableCMD getReceivedMTCMDExternalControlEnableCommand() const;
    zeabuz::common::io::messages::mtdp::MTAutopilotControlInputCMD getReceivedMTCMDAutopilotControlInput() const;

   private:
    void initializeMutexes();
    void doSubscriptionToCMDMessages();

   private:
    zeabuz::common::io::messages::mtdp::MTJoystickCommandCMD m_ReceivedMTCMDJoystickCommand;
    zeabuz::common::io::messages::mtdp::MTThrusterEnableDisableCMD m_ReceivedMTCMDThrusterEnableDisable;
    zeabuz::common::io::messages::mtdp::MTModeSelectorCMD m_ReceivedMTCMDModeSelector;
    zeabuz::common::io::messages::mtdp::MTHeadingSetpointCMD m_ReceivedMTCMDHeadingSetpoint;
    zeabuz::common::io::messages::mtdp::MTPositionSetpointCMD m_ReceivedMTCMDPositionSetpoint;
    zeabuz::common::io::messages::mtdp::MTReferenceSystemEnableDisableCMD m_ReceivedMTCMDRefSysEnableDisable;
    zeabuz::common::io::messages::mtdp::MTSensorEnableDisableCMD m_ReceivedMTCMDSensorEnableDisable;
    zeabuz::common::io::messages::mtdp::MTGainSettingsCMD m_ReceivedMTCMDGainSettings;
    zeabuz::common::io::messages::mtdp::MTJoystickSettingsCMD m_ReceivedMTCMDJoystickSettings;
    zeabuz::common::io::messages::mtdp::MTAlarmLimitsCMD m_ReceivedMTCMDAlarmLimits;
    zeabuz::common::io::messages::mtdp::MTDPClassCMD m_ReceivedMTCMDDPClass;
    zeabuz::common::io::messages::mtdp::MTExternalControlInputCMD m_ReceivedMTCMDExternalControlInput;
    zeabuz::common::io::messages::mtdp::MTExternalControlEnableCMD m_ReceivedMTCMDExternalControlEnableCommand;
    zeabuz::common::io::messages::mtdp::MTAutopilotControlInputCMD m_ReceivedMTCMDAutopilotControlInput;

    mutable std::unordered_map<uint16_t, std::unique_ptr<std::mutex>> m_MessageMutex;
    std::shared_ptr<zeabuz::common::io::message_handlers::MTNMEAMessageHandler> m_MessageHandler;
};
#endif
