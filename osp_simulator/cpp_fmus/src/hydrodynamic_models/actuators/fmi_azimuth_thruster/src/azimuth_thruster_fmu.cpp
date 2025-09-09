#include "hydrodynamic_models/actuators/fmi_azimuth_thruster/include/azimuth_thruster_fmu.hpp"

#include <cppfmu_cs.hpp>
#include <cstring>
#include <memory>
#include <zeabuz/common/utilities/enums.hpp>

#include "cppfmu_common.hpp"
#include "hydrodynamic_models/actuators/fmi_azimuth_thruster/include/azimuth_thruster.hpp"
#include "hydrodynamic_models/actuators/fmi_azimuth_thruster/include/four_quadrant_model_data.hpp"
#include "hydrodynamic_models/actuators/fmi_azimuth_thruster/include/rudder_effect.hpp"
#include "hydrodynamic_models/actuators/include/open_water_propeller_characteristics.hpp"
#include "hydrodynamic_models/actuators/include/propeller_base.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"
#include "hydrodynamic_models/hydro_utils/include/lifting_surface.hpp"

using zeabuz::common::utilities::enums::enum_cast;

AzimuthThrusterFMU::AzimuthThrusterFMU(const std::string& instance_name) : FMUBaseClass(instance_name)
{
    // -------------------------- Parameters -----------------------------------
    m_real_signals.insert(std::make_pair(m_density_i, 1025.9));
    m_real_signals.insert(std::make_pair(m_kinematic_viscosity_i, 1.19e-6));

    m_real_signals.insert(std::make_pair(m_area_i, 1.0));
    m_real_signals.insert(std::make_pair(m_chord_i, 1.0));

    m_real_signals.insert(std::make_pair(m_longitudinal_position_i, 0.0));
    m_real_signals.insert(std::make_pair(m_lateral_position_i, 0.0));

    m_real_signals.insert(std::make_pair(m_ship_length_i, 0.0));

    m_real_signals.insert(std::make_pair(m_effective_aspect_ratio_lift_i, 0.0));
    m_real_signals.insert(std::make_pair(m_effective_aspect_ratio_drag_i, 0.0));

    m_real_signals.insert(std::make_pair(m_cd_k_i, 0.0));
    m_real_signals.insert(std::make_pair(m_cd_a2_i, 0.1));

    m_real_signals.insert(std::make_pair(m_cd_stall_max_i, 1.0));
    m_real_signals.insert(std::make_pair(m_cd_stall_power_i, 1.6));

    m_real_signals.insert(std::make_pair(m_cl_stall_mean_i, 2.0));
    m_real_signals.insert(std::make_pair(m_cl_post_stall_max_i, 1.0));

    m_real_signals.insert(std::make_pair(m_tr_i, 0.0));
    m_real_signals.insert(std::make_pair(m_ah_i, 0.0));
    m_real_signals.insert(std::make_pair(m_xh_i, 0.0));

    m_real_signals.insert(std::make_pair(m_prop_diameter_i, 0.0));

    m_real_signals.insert(std::make_pair(m_wake_factor_i, 0.0));

    m_real_signals.insert(std::make_pair(m_thrust_deduction_factor_i, 0.0));

    m_string_signals.insert(std::make_pair(m_propeller_geometry_i, ""));
    m_real_signals.insert(std::make_pair(m_pitch_scaling_factor_i, 1.0));

    // ----------------------------- Input -------------------------------------
    m_real_signals.insert(std::make_pair(m_ship_velocity_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_ship_velocity_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_ship_velocity_yaw_i, 0.0));

    m_real_signals.insert(std::make_pair(m_angle_i, 0.0));
    m_real_signals.insert(std::make_pair(m_rotations_per_second_i, 0.0));

    // ----------------------------- Output ------------------------------------
    m_real_signals.insert(std::make_pair(m_force_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_force_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_yaw_i, 0.0));

    m_real_signals.insert(std::make_pair(m_thrust_i, 0.0));
    m_real_signals.insert(std::make_pair(m_torque_i, 0.0));
    m_real_signals.insert(std::make_pair(m_power_i, 0.0));

    m_real_signals.insert(std::make_pair(m_force_heave_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_roll_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_pitch_i, 0.0));
}

void AzimuthThrusterFMU::ExitInitializationMode()
{
    // Create lifting surface model for rudder effect
    auto lifting_surface = std::make_unique<LiftingSurface>();
    lifting_surface->set_aspect_ratio(m_real_signals.at(m_effective_aspect_ratio_lift_i),
                                      m_real_signals.at(m_effective_aspect_ratio_drag_i));
    lifting_surface->set_lift_stall_model(m_real_signals.at(m_cl_stall_mean_i),
                                          m_real_signals.at(m_cl_post_stall_max_i));
    lifting_surface->set_drag_pre_stall_model(m_real_signals.at(m_cd_k_i), m_real_signals.at(m_cd_a2_i));
    lifting_surface->set_drag_post_stall_model(m_real_signals.at(m_cd_stall_max_i),
                                               m_real_signals.at(m_cd_stall_power_i));

    // Create rudder effect model
    auto rudder = std::make_unique<RudderEffect>(std::move(lifting_surface));
    HullInteractionParameters hull_interaction_params{};
    hull_interaction_params.ah = m_real_signals.at(m_ah_i);
    hull_interaction_params.xh = m_real_signals.at(m_xh_i);
    hull_interaction_params.tr = m_real_signals.at(m_tr_i);
    hull_interaction_params.wake_factor = m_real_signals.at(m_wake_factor_i);
    hull_interaction_params.ship_length = m_real_signals.at(m_ship_length_i);
    rudder->set_rudder_geometry(m_real_signals.at(m_area_i), m_real_signals.at(m_chord_i));
    rudder->set_rudder_placement(m_real_signals.at(m_longitudinal_position_i), m_real_signals.at(m_lateral_position_i));
    rudder->set_fluid_properies(m_real_signals.at(m_density_i), m_real_signals.at(m_kinematic_viscosity_i));
    rudder->set_hull_interaction_params(hull_interaction_params);

    // Create propeller model
    auto propeller_geometry = enum_cast<PropellerGeometry>(m_string_signals.at(m_propeller_geometry_i));
    auto four_quadrant_model =
        generate_four_quadrant_model(propeller_geometry, m_real_signals.at(m_pitch_scaling_factor_i));
    auto propeller = std::make_unique<PropellerBase>(std::move(four_quadrant_model));
    propeller->set_propeller_diameter(m_real_signals.at(m_prop_diameter_i));
    propeller->set_placement(m_real_signals.at(m_longitudinal_position_i), m_real_signals.at(m_lateral_position_i));
    propeller->set_thrust_deduction_factor(m_real_signals.at(m_thrust_deduction_factor_i));
    propeller->set_wake_factor(m_real_signals.at(m_wake_factor_i));
    propeller->set_water_density(m_real_signals.at(m_density_i));

    // Create azimuth thruster model
    m_model = std::make_unique<AzimuthThruster>(std::move(rudder), std::move(propeller));
    m_model->set_thruster_placement(m_real_signals.at(m_longitudinal_position_i),
                                    m_real_signals.at(m_lateral_position_i));
}

bool AzimuthThrusterFMU::DoStep(cppfmu::FMIReal /*t*/, cppfmu::FMIReal /*dt*/, cppfmu::FMIBoolean /*new_step*/,
                                cppfmu::FMIReal& /*end_of_step*/)
{
    // Read inputs
    ManeuveringBodyVector ship_velocity;
    ship_velocity.surge = m_real_signals.at(m_ship_velocity_surge_i);
    ship_velocity.sway = m_real_signals.at(m_ship_velocity_sway_i);
    ship_velocity.yaw = m_real_signals.at(m_ship_velocity_yaw_i);

    double angle = m_real_signals.at(m_angle_i);
    double rotations_per_second = m_real_signals.at(m_rotations_per_second_i);

    // Update model
    m_model->set_ship_velocity(ship_velocity);
    m_model->set_propeller_speed(rotations_per_second);
    m_model->set_thruster_angle(angle);
    m_model->update_state();

    // Set outputs
    auto result_force = m_model->get_body_force();
    m_real_signals.at(m_force_surge_i) = result_force.surge;
    m_real_signals.at(m_force_sway_i) = result_force.sway;
    m_real_signals.at(m_moment_yaw_i) = result_force.yaw;

    m_real_signals.at(m_thrust_i) = m_model->get_propeller_thrust();
    m_real_signals.at(m_torque_i) = m_model->get_propeller_shaft_torque();
    m_real_signals.at(m_power_i) = m_model->get_propeller_shaft_power();

    return true;
}

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString instance_name, cppfmu::FMIString fmu_GUID, cppfmu::FMIString /*fmu_resource_location*/,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger /*logger*/)
{
    if (strcmp(fmu_GUID, FMU_UUID) != 0) {
        throw std::runtime_error("FMU GUID mismatch");
    }

    return cppfmu::AllocateUnique<AzimuthThrusterFMU>(memory, instance_name);
}
