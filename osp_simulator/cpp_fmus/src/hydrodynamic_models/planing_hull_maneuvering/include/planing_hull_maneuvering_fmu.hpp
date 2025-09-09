/*----------------------------------------------------------------------------*\

FMU interface class for the planing maneuvering model

\*----------------------------------------------------------------------------*/

#ifndef PLANING_HULL_MANEUVERINGFMU_H
#define PLANING_HULL_MANEUVERINGFMU_H

#include "cppfmu_cs.hpp"
#include "physics_fmu_base_class/physics_fmu_base_class.hpp"
#include "planing_hull_maneuvering.hpp"

class PlaningHullManeuveringFMU : public FMUBaseClass
{
   public:
    explicit PlaningHullManeuveringFMU(const std::string& instance_name);
    PlaningHullManeuveringFMU() = default;

    void ExitInitializationMode() override;

    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean new_step,
                cppfmu::FMIReal& end_of_step) override;

   private:
    std::unique_ptr<PlaningHullManeuvering> m_model;

    // --------------- parameters -----------------
    const cppfmu::FMIValueReference m_density_i = 1;
    const cppfmu::FMIValueReference m_kinematic_viscosity_i = 2;

    const cppfmu::FMIValueReference m_length_i = 3;
    const cppfmu::FMIValueReference m_beam_i = 4;
    const cppfmu::FMIValueReference m_depth_i = 5;
    const cppfmu::FMIValueReference m_mass_i = 6;
    const cppfmu::FMIValueReference m_wetted_surface_i = 7;

    const cppfmu::FMIValueReference m_shape_factor_i = 8;
    const cppfmu::FMIValueReference m_cr_m_i = 9;
    const cppfmu::FMIValueReference m_cr_p_i = 10;
    const cppfmu::FMIValueReference m_cd_lateral_i = 11;

    const cppfmu::FMIValueReference m_lcg_i = 12;
    const cppfmu::FMIValueReference m_vcg_i = 13;
    const cppfmu::FMIValueReference m_deadrise_deg_i = 14;
    const cppfmu::FMIValueReference m_vertical_thruster_placement_i = 15;

    const cppfmu::FMIValueReference m_model_mixing_start_i = 16;
    const cppfmu::FMIValueReference m_model_mixing_end_i = 17;

    const cppfmu::FMIValueReference m_sway_linear_damping_i = 18;
    const cppfmu::FMIValueReference m_yaw_linear_damping_i = 19;

    // --------------- input -----------------
    const cppfmu::FMIValueReference m_velocity_surge_i = 101;
    const cppfmu::FMIValueReference m_velocity_sway_i = 102;
    const cppfmu::FMIValueReference m_velocity_yaw_i = 103;

    // --------------- output -----------------
    const cppfmu::FMIValueReference m_force_surge_i = 201;
    const cppfmu::FMIValueReference m_force_sway_i = 202;
    const cppfmu::FMIValueReference m_moment_yaw_i = 203;

    const cppfmu::FMIValueReference m_force_heave_i = 204;
    const cppfmu::FMIValueReference m_moment_roll_i = 205;
    const cppfmu::FMIValueReference m_moment_pitch_i = 206;
};

#endif  // end header guard