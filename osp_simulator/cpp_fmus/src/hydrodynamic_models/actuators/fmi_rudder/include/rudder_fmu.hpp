/*----------------------------------------------------------------------------*\

FMU interface class for the rudder model

\*----------------------------------------------------------------------------*/

#ifndef RUDDER_FMU_H
#define RUDDER_FMU_H

#include "hydrodynamic_models/actuators/fmi_rudder/include/mmg_rudder.hpp"
#include "physics_fmu_base_class/physics_fmu_base_class.hpp"

class RudderFMU : public FMUBaseClass
{
   public:
    explicit RudderFMU(const std::string& instance_name);
    RudderFMU() = default;

    void ExitInitializationMode() override;

    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean new_step,
                cppfmu::FMIReal& end_of_step) override;

   private:
    std::unique_ptr<MMGRudder> m_model;

    // -------------------------- geometric parameters -------------------------
    const cppfmu::FMIValueReference m_density_i = 1;
    const cppfmu::FMIValueReference m_kinematic_viscosity_i = 2;

    const cppfmu::FMIValueReference m_area_i = 3;
    const cppfmu::FMIValueReference m_chord_i = 4;

    const cppfmu::FMIValueReference m_longitudinal_position_i = 5;

    const cppfmu::FMIValueReference m_ship_length_i = 6;

    // -------------------------- model parameters -----------------------------
    const cppfmu::FMIValueReference m_effective_aspect_ratio_lift_i = 7;
    const cppfmu::FMIValueReference m_effective_aspect_ratio_drag_i = 8;

    const cppfmu::FMIValueReference m_cd_k_i = 9;
    const cppfmu::FMIValueReference m_cd_a2_i = 10;

    const cppfmu::FMIValueReference m_cd_stall_max_i = 11;
    const cppfmu::FMIValueReference m_cd_stall_power_i = 12;

    const cppfmu::FMIValueReference m_cl_stall_mean_i = 13;
    const cppfmu::FMIValueReference m_cl_post_stall_max_i = 14;

    const cppfmu::FMIValueReference m_w0_i = 15;
    const cppfmu::FMIValueReference m_gamma_positive_i = 16;
    const cppfmu::FMIValueReference m_gamma_negative_i = 17;

    const cppfmu::FMIValueReference m_lr_surge_i = 18;
    const cppfmu::FMIValueReference m_lr_sway_i = 19;

    const cppfmu::FMIValueReference m_tr_i = 20;
    const cppfmu::FMIValueReference m_ah_i = 21;
    const cppfmu::FMIValueReference m_xh_i = 22;

    const cppfmu::FMIValueReference m_normal_force_mode_i = 23;

    // ----------------------------- Input -------------------------------------
    const cppfmu::FMIValueReference m_ship_velocity_surge_i = 101;
    const cppfmu::FMIValueReference m_ship_velocity_sway_i = 102;
    const cppfmu::FMIValueReference m_ship_velocity_yaw_i = 103;

    const cppfmu::FMIValueReference m_angle_i = 104;
    const cppfmu::FMIValueReference m_surge_velocity_increase_factor_i = 105;

    // ---------------------------- Output -------------------------------------
    const cppfmu::FMIValueReference m_force_surge_i = 201;
    const cppfmu::FMIValueReference m_force_sway_i = 202;
    const cppfmu::FMIValueReference m_moment_yaw_i = 203;
};

#endif