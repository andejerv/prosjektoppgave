/*----------------------------------------------------------------------------*\

FMU interface class for the azimuth_thruster model

\*----------------------------------------------------------------------------*/

#ifndef AzimuthThruster_FMU_H
#define AzimuthThruster_FMU_H

#include <memory>

#include "azimuth_thruster.hpp"
#include "hydrodynamic_models/actuators/include/open_water_propeller_characteristics.hpp"
#include "physics_fmu_base_class/physics_fmu_base_class.hpp"

class AzimuthThrusterFMU : public FMUBaseClass
{
   public:
    explicit AzimuthThrusterFMU(const std::string& instance_name);
    AzimuthThrusterFMU() = default;

    void ExitInitializationMode() override;

    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean new_step,
                cppfmu::FMIReal& end_of_step) override;

   private:
    std::unique_ptr<AzimuthThruster> m_model;

    // -------------------------- geometric parameters -------------------------
    const cppfmu::FMIValueReference m_density_i = 1;
    const cppfmu::FMIValueReference m_kinematic_viscosity_i = 2;

    const cppfmu::FMIValueReference m_area_i = 3;
    const cppfmu::FMIValueReference m_chord_i = 4;
    const cppfmu::FMIValueReference m_prop_diameter_i = 5;

    const cppfmu::FMIValueReference m_longitudinal_position_i = 6;
    const cppfmu::FMIValueReference m_lateral_position_i = 7;

    const cppfmu::FMIValueReference m_ship_length_i = 8;

    // -------------------------- Rudder effect parameters -----------------------------
    const cppfmu::FMIValueReference m_effective_aspect_ratio_lift_i = 9;
    const cppfmu::FMIValueReference m_effective_aspect_ratio_drag_i = 10;

    const cppfmu::FMIValueReference m_cd_k_i = 11;
    const cppfmu::FMIValueReference m_cd_a2_i = 12;

    const cppfmu::FMIValueReference m_cd_stall_max_i = 13;
    const cppfmu::FMIValueReference m_cd_stall_power_i = 14;

    const cppfmu::FMIValueReference m_cl_stall_mean_i = 15;
    const cppfmu::FMIValueReference m_cl_post_stall_max_i = 16;

    const cppfmu::FMIValueReference m_tr_i = 17;
    const cppfmu::FMIValueReference m_ah_i = 18;
    const cppfmu::FMIValueReference m_xh_i = 19;

    // -------------------------- Propeller parameters -----------------------------

    const cppfmu::FMIValueReference m_wake_factor_i = 20;
    const cppfmu::FMIValueReference m_thrust_deduction_factor_i = 21;
    const cppfmu::FMIValueReference m_propeller_geometry_i = 22;
    const cppfmu::FMIValueReference m_pitch_scaling_factor_i = 23;

    // ----------------------------- Input -------------------------------------
    const cppfmu::FMIValueReference m_ship_velocity_surge_i = 101;
    const cppfmu::FMIValueReference m_ship_velocity_sway_i = 102;
    const cppfmu::FMIValueReference m_ship_velocity_yaw_i = 103;

    const cppfmu::FMIValueReference m_rotations_per_second_i = 104;

    const cppfmu::FMIValueReference m_angle_i = 105;

    // ---------------------------- Output -------------------------------------
    const cppfmu::FMIValueReference m_force_surge_i = 201;
    const cppfmu::FMIValueReference m_force_sway_i = 202;
    const cppfmu::FMIValueReference m_moment_yaw_i = 203;

    const cppfmu::FMIValueReference m_thrust_i = 204;
    const cppfmu::FMIValueReference m_torque_i = 205;
    const cppfmu::FMIValueReference m_power_i = 206;

    const cppfmu::FMIValueReference m_force_heave_i = 207;
    const cppfmu::FMIValueReference m_moment_roll_i = 208;
    const cppfmu::FMIValueReference m_moment_pitch_i = 209;
};

#endif