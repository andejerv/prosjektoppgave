#include "steering_machinery_fmu.hpp"

#include <cstring>
#include <sstream>

SteeringMachineryFMU::SteeringMachineryFMU(const std::string& instance_name) : model_name(instance_name)
{
    // parameters
    m_real_signals.insert(std::make_pair(max_speed_vr, 0.0));
    m_real_signals.insert(std::make_pair(initial_angle_vr, 0.0));

    // input
    m_real_signals.insert(std::make_pair(target_angle_vr, 0.0));

    // ouput
    m_real_signals.insert(std::make_pair(angle_vr, 0.0));
}

void SteeringMachineryFMU::ExitInitializationMode()
{
    m_model = std::make_unique<SteeringMachinery>();
    m_model->set_initial_angle(m_real_signals.at(initial_angle_vr));
    m_model->set_max_speed(m_real_signals.at(max_speed_vr));
}

void SteeringMachineryFMU::SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[])
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

void SteeringMachineryFMU::GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const
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

bool SteeringMachineryFMU::DoStep(cppfmu::FMIReal /*t*/, cppfmu::FMIReal dt, cppfmu::FMIBoolean /*new_step*/,
                                  cppfmu::FMIReal& /*end_of_step*/)
{
    m_model->set_target_angle(m_real_signals.at(target_angle_vr));
    m_model->do_step(dt);
    m_real_signals.at(angle_vr) = m_model->get_angle();

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

    return cppfmu::AllocateUnique<SteeringMachineryFMU>(memory, instance_name);
}