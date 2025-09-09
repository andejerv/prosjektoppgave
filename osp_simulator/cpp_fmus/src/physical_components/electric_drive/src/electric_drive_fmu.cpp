#include "electric_drive_fmu.hpp"

#include <cstring>
#include <sstream>

ElectricDriveFMU::ElectricDriveFMU(const std::string& instance_name) : model_name(instance_name)
{
    // parameters
    m_real_signals.insert(std::make_pair(max_rpm_vr, 0.0));
    m_real_signals.insert(std::make_pair(time_constant_vr, 0.0));
    m_real_signals.insert(std::make_pair(max_rpm_rate_vr, 0.0));

    // input
    m_real_signals.insert(std::make_pair(rpm_command_vr, 0.0));

    // ouput
    m_real_signals.insert(std::make_pair(rpm_vr, 0.0));
}

void ElectricDriveFMU::ExitInitializationMode()
{
}

void ElectricDriveFMU::SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[])
{
    for (std::size_t i = 0; i < nvr; ++i) {
        if (m_real_signals.find(vr[i]) == m_real_signals.end()) {
            std::stringstream error_message;
            error_message << "Error in SetReal. Invalid m_real_signals reference value: " << vr[i];
            throw std::invalid_argument(error_message.str());
        }
        else {
            m_real_signals.at(vr[i]) = value[i];
        }
    }
}

void ElectricDriveFMU::GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const
{
    for (size_t i = 0; i < nvr; ++i) {
        if (m_real_signals.find(vr[i]) == m_real_signals.end()) {
            std::stringstream error_message;
            error_message << "Error in GetReal. Invalid m_real_signals reference value: " << vr[i];
            throw std::invalid_argument(error_message.str());
        }
        else {
            value[i] = m_real_signals.at(vr[i]);
        }
    }
}

bool ElectricDriveFMU::DoStep(cppfmu::FMIReal /*t*/, cppfmu::FMIReal dt, cppfmu::FMIBoolean /*new_step*/,
                              cppfmu::FMIReal& /*end_of_step*/)
{
    const auto T = m_real_signals.at(time_constant_vr);
    const auto max_rpm = m_real_signals.at(max_rpm_vr);
    const auto max_rpm_rate = m_real_signals.at(max_rpm_rate_vr);
    const auto rpm = m_real_signals.at(rpm_vr);
    const auto rpm_command = std::clamp(m_real_signals.at(rpm_command_vr), -max_rpm, max_rpm);

    // Euler integration of first order rpm dynamics with rate limits
    const auto rpm_dot = std::clamp((1.0 / T) * (rpm_command - rpm), -max_rpm_rate, max_rpm_rate);
    const double next_rpm = rpm + rpm_dot * dt;
    m_real_signals.at(rpm_vr) = std::clamp(next_rpm, -max_rpm, max_rpm);

    return true;
}

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString instance_name, cppfmu::FMIString fmu_guid, cppfmu::FMIString /*fmu_resource_location*/,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger /*logger*/)
{
    if (strcmp(fmu_guid, FMU_UUID) != 0) {
        throw std::runtime_error("FMU GUID mismatch");
    }

    return cppfmu::AllocateUnique<ElectricDriveFMU>(memory, instance_name);
}