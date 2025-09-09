#include "heading_controller_fmu.hpp"

#include <math.h>

#include <cstring>
#include <iostream>
#include <sstream>

HeadingControllerFMU::HeadingControllerFMU(const std::string& instance_name) : FMUBaseClass(instance_name)
{
    // parameters
    m_real_signals.insert(std::make_pair(m_K_P, 0.0));
    m_real_signals.insert(std::make_pair(m_K_I, 0.0));
    m_real_signals.insert(std::make_pair(m_K_D, 0.0));
    m_real_signals.insert(std::make_pair(m_MIN_ANGLE, 0.0));
    m_real_signals.insert(std::make_pair(m_MAX_ANGLE, 0.0));

    // input
    m_real_signals.insert(std::make_pair(m_TARGET_HEADING, 0.0));
    m_real_signals.insert(std::make_pair(m_ESTIMATED_HEADING, 0.0));
    m_boolean_signals.insert(std::make_pair(m_PID_ON, true));
    m_real_signals.insert(std::make_pair(m_MANUEL_ANGLE, 0.0));
    m_real_signals.insert(std::make_pair(m_GAIN_ADJUST, 1.0));

    // ouput
    m_real_signals.insert(std::make_pair(m_ANGLE, 0.0));
    m_real_signals.insert(std::make_pair(m_ERROR, 0.0));

    m_controller = std::make_unique<PIDController>(0.0, 0.0, 0.0, 0.0, 0.0);
}

void HeadingControllerFMU::ExitInitializationMode()
{
    m_controller->set_control_limits(m_real_signals.at(m_MIN_ANGLE), m_real_signals.at(m_MAX_ANGLE));

    m_controller->set_gains(m_real_signals.at(m_K_P), m_real_signals.at(m_K_I), m_real_signals.at(m_K_D));
}

bool HeadingControllerFMU::DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean newStep,
                                  cppfmu::FMIReal& endOfStep)
{
    m_controller->set_set_point(m_real_signals.at(m_TARGET_HEADING));

    m_controller->set_pid_on(m_boolean_signals.at(m_PID_ON));
    m_controller->set_fixed_output(m_real_signals.at(m_MANUEL_ANGLE));
    m_controller->set_gain_adjust_factor(m_real_signals.at(m_GAIN_ADJUST));

    m_real_signals.at(m_ANGLE) = m_controller->do_step(dt, m_real_signals.at(m_ESTIMATED_HEADING));
    m_real_signals.at(m_ERROR) = m_controller->get_error();

    return true;
}

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString instanceName, cppfmu::FMIString fmuGUID, cppfmu::FMIString fmuResourceLocation,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger logger)
{
    if (strcmp(fmuGUID, FMU_UUID) != 0) {
        throw std::runtime_error("FMU GUID mismatch");
    }

    return cppfmu::AllocateUnique<HeadingControllerFMU>(memory, instanceName);
}
