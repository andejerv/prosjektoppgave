#include "rudder_fmu.hpp"

#include <cstring>
#include <memory>

#include "hydrodynamic_models/actuators/include/hull_interaction.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

RudderFMU::RudderFMU(const std::string& instance_name) : FMUBaseClass(instance_name)
{
    // -------------------------- Parameters -----------------------------------
    m_real_signals.insert(std::make_pair(m_density_i, 1025.9));
    m_real_signals.insert(std::make_pair(m_kinematic_viscosity_i, 1.19e-6));

    m_real_signals.insert(std::make_pair(m_area_i, 1.0));
    m_real_signals.insert(std::make_pair(m_chord_i, 1.0));

    m_real_signals.insert(std::make_pair(m_longitudinal_position_i, 0.0));

    m_real_signals.insert(std::make_pair(m_ship_length_i, 0.0));

    m_real_signals.insert(std::make_pair(m_effective_aspect_ratio_lift_i, 0.0));
    m_real_signals.insert(std::make_pair(m_effective_aspect_ratio_drag_i, 0.0));

    m_real_signals.insert(std::make_pair(m_cd_k_i, 0.0));
    m_real_signals.insert(std::make_pair(m_cd_a2_i, 0.1));

    m_real_signals.insert(std::make_pair(m_cd_stall_max_i, 1.0));
    m_real_signals.insert(std::make_pair(m_cd_stall_power_i, 1.6));

    m_real_signals.insert(std::make_pair(m_cl_stall_mean_i, 2.0));
    m_real_signals.insert(std::make_pair(m_cl_post_stall_max_i, 1.0));

    m_real_signals.insert(std::make_pair(m_w0_i, 0.0));
    m_real_signals.insert(std::make_pair(m_gamma_positive_i, 1.0));
    m_real_signals.insert(std::make_pair(m_gamma_negative_i, 1.0));

    m_real_signals.insert(std::make_pair(m_lr_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_lr_sway_i, 0.0));

    m_real_signals.insert(std::make_pair(m_tr_i, 0.0));
    m_real_signals.insert(std::make_pair(m_ah_i, 0.0));
    m_real_signals.insert(std::make_pair(m_xh_i, 0.0));

    m_boolean_signals.insert(std::make_pair(m_normal_force_mode_i, true));

    // ----------------------------- Input -------------------------------------
    m_real_signals.insert(std::make_pair(m_ship_velocity_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_ship_velocity_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_ship_velocity_yaw_i, 0.0));

    m_real_signals.insert(std::make_pair(m_angle_i, 0.0));
    m_real_signals.insert(std::make_pair(m_surge_velocity_increase_factor_i, 1.0));

    // ----------------------------- Output ------------------------------------
    m_real_signals.insert(std::make_pair(m_force_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_force_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_yaw_i, 0.0));
}

void RudderFMU::ExitInitializationMode()
{
    // Create lifting surface model
    auto lifting_surface = std::make_unique<LiftingSurface>();
    lifting_surface->set_aspect_ratio(m_real_signals.at(m_effective_aspect_ratio_lift_i),
                                      m_real_signals.at(m_effective_aspect_ratio_drag_i));
    lifting_surface->set_lift_stall_model(m_real_signals.at(m_cl_stall_mean_i),
                                          m_real_signals.at(m_cl_post_stall_max_i));
    lifting_surface->set_drag_pre_stall_model(m_real_signals.at(m_cd_k_i), m_real_signals.at(m_cd_a2_i));
    lifting_surface->set_drag_post_stall_model(m_real_signals.at(m_cd_stall_max_i),
                                               m_real_signals.at(m_cd_stall_power_i));

    // Create hull interaction model
    auto hull_interaction = std::make_unique<HullInteraction>();
    hull_interaction->ship_length = m_real_signals.at(m_ship_length_i);
    hull_interaction->w0 = m_real_signals.at(m_w0_i);
    hull_interaction->gamma_positive = m_real_signals.at(m_gamma_positive_i);
    hull_interaction->gamma_negative = m_real_signals.at(m_gamma_negative_i);
    hull_interaction->lr_surge = m_real_signals.at(m_lr_surge_i);
    hull_interaction->lr_sway = m_real_signals.at(m_lr_sway_i);
    hull_interaction->tr = m_real_signals.at(m_tr_i);
    hull_interaction->ah = m_real_signals.at(m_ah_i);
    hull_interaction->xh = m_real_signals.at(m_xh_i);

    // Construct rudder model
    m_model = std::make_unique<MMGRudder>(std::move(lifting_surface), std::move(hull_interaction));
    m_model->set_fluid_properies(m_real_signals.at(m_density_i), m_real_signals.at(m_kinematic_viscosity_i));
    m_model->set_rudder_geometry(m_real_signals.at(m_area_i), m_real_signals.at(m_chord_i));
    m_model->set_rudder_placement(m_real_signals.at(m_longitudinal_position_i), 0.0);
    if (m_boolean_signals.at(m_normal_force_mode_i)) {
        m_model->set_model_mode(ModelMode::NORMAL_FORCE);
    }
    else {
        m_model->set_model_mode(ModelMode::LIFTING_LINE);
    }
}

bool RudderFMU::DoStep(cppfmu::FMIReal /*t*/, cppfmu::FMIReal /*dt*/, cppfmu::FMIBoolean /*new_step*/,
                       cppfmu::FMIReal& /*end_of_step*/)
{
    double ship_velocity_surge = m_real_signals.at(m_ship_velocity_surge_i);
    double ship_velocity_sway = m_real_signals.at(m_ship_velocity_sway_i);
    double ship_velocity_yaw = m_real_signals.at(m_ship_velocity_yaw_i);

    double angle = m_real_signals.at(m_angle_i);
    m_model->set_surge_velocity_increase_factor(m_real_signals.at(m_surge_velocity_increase_factor_i));

    m_model->set_ship_velocity({ship_velocity_surge, ship_velocity_sway, ship_velocity_yaw});
    m_model->set_rudder_angle(angle);

    ManeuveringBodyVector result_force = m_model->get_body_force();

    m_real_signals.at(m_force_surge_i) = result_force.surge;
    m_real_signals.at(m_force_sway_i) = result_force.sway;
    m_real_signals.at(m_moment_yaw_i) = result_force.yaw;

    return true;
}

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString instance_name, cppfmu::FMIString fmu_guid, cppfmu::FMIString /*fmu_resource_location*/ /*value*/,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger /*logger*/)
{
    if (strcmp(fmu_guid, FMU_UUID) != 0) {
        throw std::runtime_error("FMU GUID mismatch");
    }

    return cppfmu::AllocateUnique<RudderFMU>(memory, instance_name);
}