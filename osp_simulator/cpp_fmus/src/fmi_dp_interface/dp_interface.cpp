#include "dp_interface.hpp"

#include "zeabuz/common/io/message_handler_descriptions.hpp"
#include "zeabuz/common/io/messages/mtdp/mt_autopilot_control_input.hpp"

using zeabuz::common::io::message_handlers::MTNMEAMessageHandler;
using namespace zeabuz::common::io::messages::mtdp;

MarineTechnologiesDpInterface::MarineTechnologiesDpInterface(std::shared_ptr<MTNMEAMessageHandler> messageHandler)
: m_MessageHandler(messageHandler)
{
    initializeMutexes();
    doSubscriptionToCMDMessages();
}

void MarineTechnologiesDpInterface::initializeMutexes()
{
    m_MessageMutex.insert({MTJoystickCommandStruct::ID, std::make_unique<std::mutex>()});
    m_MessageMutex.insert({MTThrusterEnableDisableStruct::ID, std::make_unique<std::mutex>()});
    m_MessageMutex.insert({MTModeSelectorStruct::ID, std::make_unique<std::mutex>()});
    m_MessageMutex.insert({MTHeadingSetpointStruct::ID, std::make_unique<std::mutex>()});
    m_MessageMutex.insert({MTPositionSetpointStruct::ID, std::make_unique<std::mutex>()});
    m_MessageMutex.insert({MTReferenceSystemEnableDisableStruct::ID, std::make_unique<std::mutex>()});
    m_MessageMutex.insert({MTSensorEnableDisableStruct::ID, std::make_unique<std::mutex>()});
    m_MessageMutex.insert({MTGainSettingsStruct::ID, std::make_unique<std::mutex>()});
    m_MessageMutex.insert({MTJoystickSettingsStruct::ID, std::make_unique<std::mutex>()});
    m_MessageMutex.insert({MTAlarmLimitsStruct::ID, std::make_unique<std::mutex>()});
    m_MessageMutex.insert({MTDPClassStruct::ID, std::make_unique<std::mutex>()});
    m_MessageMutex.insert({MTExternalControlInputStruct::ID, std::make_unique<std::mutex>()});
    m_MessageMutex.insert({MTExternalControlEnableFBAStruct::ID, std::make_unique<std::mutex>()});
    m_MessageMutex.insert({MTExternalControlEnableCMDStruct::ID, std::make_unique<std::mutex>()});
    m_MessageMutex.insert({MTAutopilotControlInputStruct::ID, std::make_unique<std::mutex>()});
}

void MarineTechnologiesDpInterface::doSubscriptionToCMDMessages()
{
    m_MessageHandler->subscribe_to_message(&MarineTechnologiesDpInterface::handleMTCMDJoystickCommand, this);
    m_MessageHandler->subscribe_to_message(&MarineTechnologiesDpInterface::handleMTCMDThrusterEnableDisable, this);
    m_MessageHandler->subscribe_to_message(&MarineTechnologiesDpInterface::handleMTCMDModeSelector, this);
    m_MessageHandler->subscribe_to_message(&MarineTechnologiesDpInterface::handleMTCMDHeadingSetpoint, this);
    m_MessageHandler->subscribe_to_message(&MarineTechnologiesDpInterface::handleMTCMDPositionSetpoint, this);
    m_MessageHandler->subscribe_to_message(&MarineTechnologiesDpInterface::handleMTCMDRefSysEnableDisable, this);
    m_MessageHandler->subscribe_to_message(&MarineTechnologiesDpInterface::handleMTCMDSensorEnableDisable, this);
    m_MessageHandler->subscribe_to_message(&MarineTechnologiesDpInterface::handleMTCMDGainSettings, this);
    m_MessageHandler->subscribe_to_message(&MarineTechnologiesDpInterface::handleMTCMDJoystickSettings, this);
    m_MessageHandler->subscribe_to_message(&MarineTechnologiesDpInterface::handleMTCMDAlarmLimits, this);
    m_MessageHandler->subscribe_to_message(&MarineTechnologiesDpInterface::handleMTCMDDPClass, this);
    m_MessageHandler->subscribe_to_message(&MarineTechnologiesDpInterface::handleMTCMDExternalControlInput, this);
    m_MessageHandler->subscribe_to_message(&MarineTechnologiesDpInterface::handleMTCMDExternalControlEnableCommand,
                                           this);
    m_MessageHandler->subscribe_to_message(&MarineTechnologiesDpInterface::handleMTCMDAutopilotControlInput, this);
    m_MessageHandler->run();
}

//////////////////////////////////////////////////////////////////////////////////
////////////////////////////// MT COMMAND MESSAGES ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void MarineTechnologiesDpInterface::handleMTCMDJoystickCommand(const MTJoystickCommandCMD& message)
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[message.m_Object.get_mtdp_id()].get()};
    m_ReceivedMTCMDJoystickCommand = message;
}

void MarineTechnologiesDpInterface::handleMTCMDThrusterEnableDisable(const MTThrusterEnableDisableCMD& message)
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[message.m_Object.get_mtdp_id()].get()};
    m_ReceivedMTCMDThrusterEnableDisable = message;
}

void MarineTechnologiesDpInterface::handleMTCMDModeSelector(const MTModeSelectorCMD& message)
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[message.m_Object.get_mtdp_id()].get()};
    m_ReceivedMTCMDModeSelector = message;
}

void MarineTechnologiesDpInterface::handleMTCMDHeadingSetpoint(const MTHeadingSetpointCMD& message)
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[message.m_Object.get_mtdp_id()].get()};
    m_ReceivedMTCMDHeadingSetpoint = message;
}

void MarineTechnologiesDpInterface::handleMTCMDPositionSetpoint(const MTPositionSetpointCMD& message)
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[message.m_Object.get_mtdp_id()].get()};
    m_ReceivedMTCMDPositionSetpoint = message;
}

void MarineTechnologiesDpInterface::handleMTCMDRefSysEnableDisable(const MTReferenceSystemEnableDisableCMD& message)
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[message.m_Object.get_mtdp_id()].get()};
    m_ReceivedMTCMDRefSysEnableDisable = message;
}

void MarineTechnologiesDpInterface::handleMTCMDSensorEnableDisable(const MTSensorEnableDisableCMD& message)
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[message.m_Object.get_mtdp_id()].get()};
    m_ReceivedMTCMDSensorEnableDisable = message;
}

void MarineTechnologiesDpInterface::handleMTCMDGainSettings(const MTGainSettingsCMD& message)
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[message.m_Object.get_mtdp_id()].get()};
    m_ReceivedMTCMDGainSettings = message;
}

void MarineTechnologiesDpInterface::handleMTCMDJoystickSettings(const MTJoystickSettingsCMD& message)
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[message.m_Object.get_mtdp_id()].get()};
    m_ReceivedMTCMDJoystickSettings = message;
}

void MarineTechnologiesDpInterface::handleMTCMDAlarmLimits(const MTAlarmLimitsCMD& message)
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[message.m_Object.get_mtdp_id()].get()};
    m_ReceivedMTCMDAlarmLimits = message;
}

void MarineTechnologiesDpInterface::handleMTCMDDPClass(const MTDPClassCMD& message)
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[message.m_Object.get_mtdp_id()].get()};
    m_ReceivedMTCMDDPClass = message;
}

void MarineTechnologiesDpInterface::handleMTCMDExternalControlInput(const MTExternalControlInputCMD& message)
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[message.m_Object.get_mtdp_id()].get()};
    m_ReceivedMTCMDExternalControlInput = message;
}

void MarineTechnologiesDpInterface::handleMTCMDExternalControlEnableCommand(const MTExternalControlEnableCMD& message)
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[message.m_Object.get_mtdp_id()].get()};
    m_ReceivedMTCMDExternalControlEnableCommand = message;
}

void MarineTechnologiesDpInterface::handleMTCMDAutopilotControlInput(const MTAutopilotControlInputCMD& message)
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[message.m_Object.get_mtdp_id()].get()};
    m_ReceivedMTCMDAutopilotControlInput = message;
}
//////////////////////////////////////////////////////////////////////////////////
////////////////////////// GET RECEIVED CMD MESSAGES /////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
MTJoystickCommandCMD MarineTechnologiesDpInterface::getReceivedMTCMDJoystickCommand() const
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[m_ReceivedMTCMDJoystickCommand.m_Object.get_mtdp_id()].get()};
    return m_ReceivedMTCMDJoystickCommand;
}
MTThrusterEnableDisableCMD MarineTechnologiesDpInterface::getReceivedMTCMDThrusterEnableDisable() const
{
    std::scoped_lock<std::mutex> lock{
        *m_MessageMutex[m_ReceivedMTCMDThrusterEnableDisable.m_Object.get_mtdp_id()].get()};
    return m_ReceivedMTCMDThrusterEnableDisable;
}
MTModeSelectorCMD MarineTechnologiesDpInterface::getReceivedMTCMDModeSelector() const
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[m_ReceivedMTCMDModeSelector.m_Object.get_mtdp_id()].get()};
    return m_ReceivedMTCMDModeSelector;
}
MTHeadingSetpointCMD MarineTechnologiesDpInterface::getReceivedMTCMDHeadingSetpoint() const
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[m_ReceivedMTCMDHeadingSetpoint.m_Object.get_mtdp_id()].get()};
    return m_ReceivedMTCMDHeadingSetpoint;
}
MTPositionSetpointCMD MarineTechnologiesDpInterface::getReceivedMTCMDPositionSetpoint() const
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[m_ReceivedMTCMDPositionSetpoint.m_Object.get_mtdp_id()].get()};
    return m_ReceivedMTCMDPositionSetpoint;
}
MTReferenceSystemEnableDisableCMD MarineTechnologiesDpInterface::getReceivedMTCMDRefSysEnableDisable() const
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[m_ReceivedMTCMDRefSysEnableDisable.m_Object.get_mtdp_id()].get()};
    return m_ReceivedMTCMDRefSysEnableDisable;
}
MTSensorEnableDisableCMD MarineTechnologiesDpInterface::getReceivedMTCMDSensorEnableDisable() const
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[m_ReceivedMTCMDSensorEnableDisable.m_Object.get_mtdp_id()].get()};
    return m_ReceivedMTCMDSensorEnableDisable;
}
MTGainSettingsCMD MarineTechnologiesDpInterface::getReceivedMTCMDGainSettings() const
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[m_ReceivedMTCMDGainSettings.m_Object.get_mtdp_id()].get()};
    return m_ReceivedMTCMDGainSettings;
}
MTJoystickSettingsCMD MarineTechnologiesDpInterface::getReceivedMTCMDJoystickSettings() const
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[m_ReceivedMTCMDJoystickSettings.m_Object.get_mtdp_id()].get()};
    return m_ReceivedMTCMDJoystickSettings;
}
MTAlarmLimitsCMD MarineTechnologiesDpInterface::getReceivedMTCMDAlarmLimits() const
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[m_ReceivedMTCMDAlarmLimits.m_Object.get_mtdp_id()].get()};
    return m_ReceivedMTCMDAlarmLimits;
}
MTDPClassCMD MarineTechnologiesDpInterface::getReceivedMTCMDDPClass() const
{
    std::scoped_lock<std::mutex> lock{*m_MessageMutex[m_ReceivedMTCMDDPClass.m_Object.get_mtdp_id()].get()};
    return m_ReceivedMTCMDDPClass;
}
MTExternalControlInputCMD MarineTechnologiesDpInterface::getReceivedMTCMDExternalControlInput() const
{
    std::scoped_lock<std::mutex> lock{
        *m_MessageMutex[m_ReceivedMTCMDExternalControlInput.m_Object.get_mtdp_id()].get()};
    return m_ReceivedMTCMDExternalControlInput;
}
MTExternalControlEnableCMD MarineTechnologiesDpInterface::getReceivedMTCMDExternalControlEnableCommand() const
{
    std::scoped_lock<std::mutex> lock{
        *m_MessageMutex[m_ReceivedMTCMDExternalControlEnableCommand.m_Object.get_mtdp_id()].get()};
    return m_ReceivedMTCMDExternalControlEnableCommand;
}

MTAutopilotControlInputCMD MarineTechnologiesDpInterface::getReceivedMTCMDAutopilotControlInput() const
{
    std::scoped_lock<std::mutex> lock{
        *m_MessageMutex[m_ReceivedMTCMDAutopilotControlInput.m_Object.get_mtdp_id()].get()};
    return m_ReceivedMTCMDAutopilotControlInput;
}