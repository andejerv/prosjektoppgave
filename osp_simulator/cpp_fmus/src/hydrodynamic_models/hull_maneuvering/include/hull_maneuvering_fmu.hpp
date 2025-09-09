/*----------------------------------------------------------------------------*\

FMU interface class for the hull model

\*----------------------------------------------------------------------------*/

#ifndef HULL_MANEUVERINGFMU_H
#define HULL_MANEUVERINGFMU_H

#include "cppfmu_cs.hpp"
#include "hull_maneuvering.hpp"
#include "physics_fmu_base_class/physics_fmu_base_class.hpp"

class HullManeuveringFMU : public FMUBaseClass
{
   public:
    explicit HullManeuveringFMU(const std::string& instance_name);
    HullManeuveringFMU() = default;

    void ExitInitializationMode() override;

    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean new_step,
                cppfmu::FMIReal& end_of_step) override;

   private:
    std::unique_ptr<HullManeuvering> m_model;

    // --------------- parameters -----------------
    const cppfmu::FMIValueReference m_density_i = 1;
    const cppfmu::FMIValueReference m_kinematic_viscosity_i = 2;

    const cppfmu::FMIValueReference m_length_i = 3;
    const cppfmu::FMIValueReference m_depth_i = 4;
    const cppfmu::FMIValueReference m_wetted_surface_i = 5;

    const cppfmu::FMIValueReference m_shape_factor_i = 6;
    const cppfmu::FMIValueReference m_cr_m_i = 7;
    const cppfmu::FMIValueReference m_cr_p_i = 8;
    const cppfmu::FMIValueReference m_cd_lateral_i = 9;

    const cppfmu::FMIValueReference m_xvv_i = 10;
    const cppfmu::FMIValueReference m_xvr_i = 11;
    const cppfmu::FMIValueReference m_xrr_i = 12;
    const cppfmu::FMIValueReference m_xvvvv_i = 13;

    const cppfmu::FMIValueReference m_yv_i = 14;
    const cppfmu::FMIValueReference m_yr_i = 15;
    const cppfmu::FMIValueReference m_yvvv_i = 16;
    const cppfmu::FMIValueReference m_yvvr_i = 17;
    const cppfmu::FMIValueReference m_yvrr_i = 18;
    const cppfmu::FMIValueReference m_yrrr_i = 19;

    const cppfmu::FMIValueReference m_nv_i = 20;
    const cppfmu::FMIValueReference m_nr_i = 21;
    const cppfmu::FMIValueReference m_nvvv_i = 22;
    const cppfmu::FMIValueReference m_nvvr_i = 23;
    const cppfmu::FMIValueReference m_nvrr_i = 24;
    const cppfmu::FMIValueReference m_nrrr_i = 25;

    const cppfmu::FMIValueReference m_cx_low_i = 26;
    const cppfmu::FMIValueReference m_cy_low_i = 27;
    const cppfmu::FMIValueReference m_cn_low_i = 28;
    const cppfmu::FMIValueReference m_nr_low_i = 29;

    const cppfmu::FMIValueReference m_speed_limit_low_i = 30;
    const cppfmu::FMIValueReference m_speed_transfer_range_i = 31;

    const cppfmu::FMIValueReference m_low_speed_model_i = 32;
    const cppfmu::FMIValueReference m_high_speed_model_i = 33;

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