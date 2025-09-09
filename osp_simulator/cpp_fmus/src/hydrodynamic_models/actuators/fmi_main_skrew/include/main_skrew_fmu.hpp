/*----------------------------------------------------------------------------*\

FMU interface class for main skrew propeller model

\*----------------------------------------------------------------------------*/

#ifndef PROPELLER_FMU_H
#define PROPELLER_FMU_H

#include "hydrodynamic_models/actuators/fmi_main_skrew/include/main_skrew.hpp"
#include "physics_fmu_base_class/physics_fmu_base_class.hpp"

class PropellerFMU : public FMUBaseClass
{
   public:
    explicit PropellerFMU(const std::string& instance_name);
    PropellerFMU() = default;

    void ExitInitializationMode() override;

    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean new_step,
                cppfmu::FMIReal& end_of_step) override;

   private:
    std::unique_ptr<MainSkrew> m_model;

    // --------------- parameters -----------------
    const cppfmu::FMIValueReference m_density_i = 1;
    const cppfmu::FMIValueReference m_diameter_i = 2;
    const cppfmu::FMIValueReference m_longitudinal_position_i = 3;

    const cppfmu::FMIValueReference m_wake_factor_i = 4;

    const cppfmu::FMIValueReference m_jet_stream_model_kappa_i = 7;
    const cppfmu::FMIValueReference m_jet_stream_model_eta_i = 8;
    const cppfmu::FMIValueReference m_jet_stream_model_epsilon_i = 9;

    const cppfmu::FMIValueReference m_thrust_deduction_factor_i = 10;

    const cppfmu::FMIValueReference m_thrust_coefficient_a0_i = 11;
    const cppfmu::FMIValueReference m_thrust_coefficient_a1_i = 12;
    const cppfmu::FMIValueReference m_thrust_coefficient_a2_i = 13;

    const cppfmu::FMIValueReference m_torque_coefficient_a0_i = 14;
    const cppfmu::FMIValueReference m_torque_coefficient_a1_i = 15;
    const cppfmu::FMIValueReference m_torque_coefficient_a2_i = 16;

    // --------------- input -----------------
    const cppfmu::FMIValueReference m_ship_velocity_surge_i = 101;
    const cppfmu::FMIValueReference m_ship_velocity_sway_i = 102;
    const cppfmu::FMIValueReference m_ship_velocity_yaw_i = 103;
    const cppfmu::FMIValueReference m_rotations_per_second_i = 104;

    // --------------- output -----------------
    const cppfmu::FMIValueReference m_thrust_i = 201;
    const cppfmu::FMIValueReference m_torque_i = 202;
    const cppfmu::FMIValueReference m_power_i = 203;

    const cppfmu::FMIValueReference m_surge_velocity_increase_factor_i = 204;
};

#endif  // end header guard