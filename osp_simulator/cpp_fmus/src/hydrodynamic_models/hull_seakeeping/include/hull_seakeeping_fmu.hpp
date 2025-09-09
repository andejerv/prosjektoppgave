/*----------------------------------------------------------------------------*\

FMU interface class for a simple hull seakeeping model based on three
uncoupled harmonic oscillators

\*----------------------------------------------------------------------------*/

#ifndef HULL_SEAKEEPINGFMU_H
#define HULL_SEAKEEPINGFMU_H

#include "cppfmu_cs.hpp"
#include "hull_seakeeping.hpp"
#include "physics_fmu_base_class/physics_fmu_base_class.hpp"

class HullSeakeepingFMU : public FMUBaseClass
{
   public:
    explicit HullSeakeepingFMU(const std::string& instance_name);
    HullSeakeepingFMU() = default;

    void ExitInitializationMode() override;

    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean new_step,
                cppfmu::FMIReal& end_of_step) override;

   private:
    std::unique_ptr<HullSeakeeping> m_model;

    // --------------- parameters -----------------
    const cppfmu::FMIValueReference m_density_i = 1;
    const cppfmu::FMIValueReference m_gravity_i = 2;

    const cppfmu::FMIValueReference m_d_heave_i = 3;
    const cppfmu::FMIValueReference m_d_roll_i = 4;
    const cppfmu::FMIValueReference m_d_pitch_i = 5;

    const cppfmu::FMIValueReference m_gm_roll_i = 6;
    const cppfmu::FMIValueReference m_gm_pitch_i = 7;
    const cppfmu::FMIValueReference m_mass_i = 8;
    const cppfmu::FMIValueReference m_waterplane_area_i = 9;

    // --------------- input -----------------
    const cppfmu::FMIValueReference m_heave_i = 101;
    const cppfmu::FMIValueReference m_roll_i = 102;
    const cppfmu::FMIValueReference m_pitch_i = 103;

    const cppfmu::FMIValueReference m_velocity_heave_i = 104;
    const cppfmu::FMIValueReference m_velocity_roll_i = 105;
    const cppfmu::FMIValueReference m_velocity_pitch_i = 106;

    // --------------- output -----------------
    const cppfmu::FMIValueReference m_force_heave_i = 201;
    const cppfmu::FMIValueReference m_moment_roll_i = 202;
    const cppfmu::FMIValueReference m_moment_pitch_i = 203;

    const cppfmu::FMIValueReference m_force_surge_i = 204;
    const cppfmu::FMIValueReference m_force_sway_i = 205;
    const cppfmu::FMIValueReference m_moment_yaw_i = 206;
};

#endif  // end header guard