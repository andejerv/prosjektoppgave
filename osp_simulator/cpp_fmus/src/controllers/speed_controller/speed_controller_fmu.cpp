#include "speed_controller_fmu.hpp"

#include <cmath>
#include <cstring>

SpeedControllerFMU::SpeedControllerFMU(const std::string& instance_name) : FMUBaseClass(instance_name)
{
    // parameters
    m_real_signals.insert(std::make_pair(m_K_P, 0.0));
    m_real_signals.insert(std::make_pair(m_K_I, 0.0));
    m_real_signals.insert(std::make_pair(m_K_D, 0.0));
    m_real_signals.insert(std::make_pair(m_MIN_LOADING, 0.0));
    m_real_signals.insert(std::make_pair(m_MAX_LOADING, 0.0));

    // input
    m_real_signals.insert(std::make_pair(m_TARGET_SPEED, 0.0));
    m_real_signals.insert(std::make_pair(m_ESTIMATED_U, 0.0));
    m_real_signals.insert(std::make_pair(m_ESTIMATED_V, 0.0));

    // ouput
    m_real_signals.insert(std::make_pair(m_LOADING, 0.0));
    m_real_signals.insert(std::make_pair(m_ERROR, 0.0));

    m_controller = std::make_unique<PIDController>(0.0, 0.0, 0.0, 0.0, 0.0);
}

void SpeedControllerFMU::ExitInitializationMode()
{
    this->SetControllerParameters();
}

void SpeedControllerFMU::SetControllerParameters()
{
    m_controller->set_control_limits(m_real_signals.at(m_MIN_LOADING), m_real_signals.at(m_MAX_LOADING));

    m_controller->set_gains(m_real_signals.at(m_K_P), m_real_signals.at(m_K_I), m_real_signals.at(m_K_D));
}

bool SpeedControllerFMU::DoStep(cppfmu::FMIReal /*t*/, cppfmu::FMIReal dt, cppfmu::FMIBoolean /*newStep*/,
                                cppfmu::FMIReal& /*endOfStep*/)
{
    m_controller->set_set_point(m_real_signals.at(m_TARGET_SPEED));

    double u = m_real_signals.at(m_ESTIMATED_U);
    double v = m_real_signals.at(m_ESTIMATED_V);

    double estimated_speed = sqrt(std::pow(u, 2.0) + std::pow(v, 2.0));

    m_real_signals.at(m_LOADING) = m_controller->do_step(dt, estimated_speed);

    m_real_signals.at(m_ERROR) = m_controller->get_error();

    return true;
}

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString instanceName, cppfmu::FMIString fmuGUID, cppfmu::FMIString /*fmuResourceLocation*/,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger /*logger*/)
{
    if (std::strcmp(fmuGUID, FMU_UUID) != 0) {
        throw std::runtime_error("FMU GUID mismatch");
    }

    return cppfmu::AllocateUnique<SpeedControllerFMU>(memory, instanceName);
}
