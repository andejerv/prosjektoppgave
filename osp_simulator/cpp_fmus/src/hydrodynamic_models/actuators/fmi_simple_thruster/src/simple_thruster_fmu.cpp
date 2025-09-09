#include "simple_thruster_fmu.hpp"

#include <cstring>

SimpleThrusterFMU::SimpleThrusterFMU(const std::string& instance_name) : FMUBaseClass(instance_name)
{
    // parameters
    m_real_signals.insert(std::make_pair(m_p0_x_i, 0.0));
    m_real_signals.insert(std::make_pair(m_p0_y_i, 0.0));
    m_real_signals.insert(std::make_pair(m_orientation0_x_i, 0.0));
    m_real_signals.insert(std::make_pair(m_orientation0_y_i, 0.0));
    m_real_signals.insert(std::make_pair(m_max_thrust_i, 0.0));

    // input
    m_real_signals.insert(std::make_pair(m_angle_i, 0.0));
    m_real_signals.insert(std::make_pair(m_loading_i, 0.0));

    // ouput
    m_real_signals.insert(std::make_pair(m_force_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_force_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_yaw_i, 0.0));

    m_model = std::make_unique<SimpleThruster>();
}

void SimpleThrusterFMU::ExitInitializationMode()
{
    Vector p0;
    Vector orientation0;

    p0.x = m_real_signals.at(m_p0_x_i);
    p0.y = m_real_signals.at(m_p0_y_i);

    orientation0.x = m_real_signals.at(m_orientation0_x_i);
    orientation0.y = m_real_signals.at(m_orientation0_y_i);

    double max_thrust = m_real_signals.at(m_max_thrust_i);

    m_model->set_parameters(p0, orientation0, max_thrust);
}

bool SimpleThrusterFMU::DoStep(cppfmu::FMIReal, cppfmu::FMIReal, cppfmu::FMIBoolean, cppfmu::FMIReal&)
{
    m_model->set_angle(m_real_signals.at(m_angle_i));
    m_model->set_loading(m_real_signals.at(m_loading_i));

    ManeuveringBodyVector result = m_model->do_step();

    m_real_signals.at(m_force_surge_i) = result.surge;
    m_real_signals.at(m_force_sway_i) = result.sway;
    m_real_signals.at(m_moment_yaw_i) = result.yaw;

    return true;
}

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString instance_name, cppfmu::FMIString fmu_guid, cppfmu::FMIString /*fmu_resource_location*/,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger)
{
    if (strcmp(fmu_guid, FMU_UUID) != 0) {
        throw std::runtime_error("FMU GUID mismatch");
    }

    return cppfmu::AllocateUnique<SimpleThrusterFMU>(memory, instance_name);
}