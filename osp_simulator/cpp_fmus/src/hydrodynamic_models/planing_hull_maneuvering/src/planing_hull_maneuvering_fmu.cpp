#include "hydrodynamic_models/planing_hull_maneuvering/include/planing_hull_maneuvering_fmu.hpp"

#include <cstring>

#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

PlaningHullManeuveringFMU::PlaningHullManeuveringFMU(const std::string& instance_name) : FMUBaseClass(instance_name)
{
    // parameters
    m_real_signals.insert(std::make_pair(m_density_i, 1025.9));
    m_real_signals.insert(std::make_pair(m_kinematic_viscosity_i, 1.19e-6));

    m_real_signals.insert(std::make_pair(m_length_i, 0.0));
    m_real_signals.insert(std::make_pair(m_beam_i, 0.0));
    m_real_signals.insert(std::make_pair(m_depth_i, 0.0));
    m_real_signals.insert(std::make_pair(m_mass_i, 0.0));
    m_real_signals.insert(std::make_pair(m_wetted_surface_i, 0.0));

    m_real_signals.insert(std::make_pair(m_shape_factor_i, 0.0));
    m_real_signals.insert(std::make_pair(m_cr_m_i, 0.0));
    m_real_signals.insert(std::make_pair(m_cr_p_i, 0.0));
    m_real_signals.insert(std::make_pair(m_cd_lateral_i, 0.0));

    m_real_signals.insert(std::make_pair(m_lcg_i, 0.0));
    m_real_signals.insert(std::make_pair(m_vcg_i, 0.0));
    m_real_signals.insert(std::make_pair(m_deadrise_deg_i, 0.0));
    m_real_signals.insert(std::make_pair(m_vertical_thruster_placement_i, 0.0));

    m_real_signals.insert(std::make_pair(m_model_mixing_start_i, 3.0));
    m_real_signals.insert(std::make_pair(m_model_mixing_end_i, 4.0));

    m_real_signals.insert(std::make_pair(m_sway_linear_damping_i, 0.0));
    m_real_signals.insert(std::make_pair(m_yaw_linear_damping_i, 0.0));

    // input
    m_real_signals.insert(std::make_pair(m_velocity_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_velocity_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_velocity_yaw_i, 0.0));

    // ouput
    m_real_signals.insert(std::make_pair(m_force_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_force_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_yaw_i, 0.0));

    m_real_signals.insert(std::make_pair(m_force_heave_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_roll_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_pitch_i, 0.0));

    m_model = std::make_unique<PlaningHullManeuvering>();
}

void PlaningHullManeuveringFMU::ExitInitializationMode()
{
    m_model->density = m_real_signals.at(m_density_i);
    m_model->kinematic_viscosity = m_real_signals.at(m_kinematic_viscosity_i);

    m_model->length = m_real_signals.at(m_length_i);
    m_model->beam = m_real_signals.at(m_beam_i);
    m_model->depth = m_real_signals.at(m_depth_i);
    m_model->mass = m_real_signals.at(m_mass_i);
    m_model->wetted_surface = m_real_signals.at(m_wetted_surface_i);

    m_model->shape_factor = m_real_signals.at(m_shape_factor_i);
    m_model->CR_m = m_real_signals.at(m_cr_m_i);
    m_model->CR_p = m_real_signals.at(m_cr_p_i);
    m_model->CD_lateral = m_real_signals.at(m_cd_lateral_i);

    m_model->lcg = m_real_signals.at(m_lcg_i);
    m_model->vcg = m_real_signals.at(m_vcg_i);
    m_model->deadrise_deg = m_real_signals.at(m_deadrise_deg_i);
    m_model->vertical_thruster_placement = m_real_signals.at(m_vertical_thruster_placement_i);

    m_model->model_mixing_start = m_real_signals.at(m_model_mixing_start_i);
    m_model->model_mixing_end = m_real_signals.at(m_model_mixing_end_i);

    m_model->sway_linear_damping = m_real_signals.at(m_sway_linear_damping_i);
    m_model->yaw_linear_damping = m_real_signals.at(m_yaw_linear_damping_i);
}

bool PlaningHullManeuveringFMU::DoStep(cppfmu::FMIReal /*t*/, cppfmu::FMIReal /*dt*/, cppfmu::FMIBoolean /*new_step*/,
                                       cppfmu::FMIReal& /*end_of_step*/)
{
    double u = m_real_signals.at(m_velocity_surge_i);
    double v = m_real_signals.at(m_velocity_sway_i);
    double r = m_real_signals.at(m_velocity_yaw_i);

    m_model->do_step(u, v, r);

    ManeuveringBodyVector force = m_model->get_maneuvering_force();

    m_real_signals.at(m_force_surge_i) = force.surge;
    m_real_signals.at(m_force_sway_i) = force.sway;
    m_real_signals.at(m_moment_yaw_i) = force.yaw;

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

    return cppfmu::AllocateUnique<PlaningHullManeuveringFMU>(memory, instance_name);
}