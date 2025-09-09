#include "hull_seakeeping_fmu.hpp"

#include <cstring>

HullSeakeepingFMU::HullSeakeepingFMU(const std::string& instance_name) : FMUBaseClass(instance_name)
{
    // parameters
    m_real_signals.insert(std::make_pair(m_density_i, 1025.9));
    m_real_signals.insert(std::make_pair(m_gravity_i, 9.81));

    m_real_signals.insert(std::make_pair(m_d_heave_i, 0.0));
    m_real_signals.insert(std::make_pair(m_d_roll_i, 0.0));
    m_real_signals.insert(std::make_pair(m_d_pitch_i, 0.0));

    m_real_signals.insert(std::make_pair(m_gm_roll_i, 0.0));
    m_real_signals.insert(std::make_pair(m_gm_pitch_i, 0.0));
    m_real_signals.insert(std::make_pair(m_mass_i, 0.0));
    m_real_signals.insert(std::make_pair(m_waterplane_area_i, 0.0));

    // input
    m_real_signals.insert(std::make_pair(m_heave_i, 0.0));
    m_real_signals.insert(std::make_pair(m_roll_i, 0.0));
    m_real_signals.insert(std::make_pair(m_pitch_i, 0.0));

    m_real_signals.insert(std::make_pair(m_velocity_heave_i, 0.0));
    m_real_signals.insert(std::make_pair(m_velocity_roll_i, 0.0));
    m_real_signals.insert(std::make_pair(m_velocity_pitch_i, 0.0));

    // ouput
    m_real_signals.insert(std::make_pair(m_force_heave_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_roll_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_pitch_i, 0.0));

    m_real_signals.insert(std::make_pair(m_force_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_force_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_yaw_i, 0.0));

    m_model = std::make_unique<HullSeakeeping>();
}

void HullSeakeepingFMU::ExitInitializationMode()
{
    m_model->density = m_real_signals.at(m_density_i);
    m_model->gravity = m_real_signals.at(m_gravity_i);

    m_model->heave_damping = m_real_signals.at(m_d_heave_i);
    m_model->roll_damping = m_real_signals.at(m_d_roll_i);
    m_model->pitch_damping = m_real_signals.at(m_d_pitch_i);

    m_model->mass = m_real_signals.at(m_mass_i);
    m_model->waterplane_area = m_real_signals.at(m_waterplane_area_i);
    m_model->GM_roll = m_real_signals.at(m_gm_roll_i);
    m_model->GM_pitch = m_real_signals.at(m_gm_pitch_i);

    m_model->calc_restoring_force_coeffients();
}

bool HullSeakeepingFMU::DoStep(cppfmu::FMIReal /*t*/, cppfmu::FMIReal /*dt*/, cppfmu::FMIBoolean /*new_step*/,
                               cppfmu::FMIReal& /*end_of_step*/)
{
    double w = m_real_signals.at(m_velocity_heave_i);
    double p = m_real_signals.at(m_velocity_roll_i);
    double q = m_real_signals.at(m_velocity_pitch_i);

    double heave = m_real_signals.at(m_heave_i);
    double roll = m_real_signals.at(m_roll_i);
    double pitch = m_real_signals.at(m_pitch_i);

    m_model->set_velocity(w, p, q);
    m_model->set_displacement(heave, roll, pitch);

    m_model->calc_force();
    auto force = m_model->get_force();

    m_real_signals.at(m_force_heave_i) = force.heave;
    m_real_signals.at(m_moment_roll_i) = force.roll;
    m_real_signals.at(m_moment_pitch_i) = force.pitch;

    return true;
}

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString instance_name, cppfmu::FMIString fmu_guid, cppfmu::FMIString /*fmu_resource_location*/,
    cppfmu::FMIString /*mime_type*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger /*logger*/)
{
    if (strcmp(fmu_guid, FMU_UUID) != 0) {
        throw std::runtime_error("FMU GUID mismatch");
    }

    return cppfmu::AllocateUnique<HullSeakeepingFMU>(memory, instance_name);
}