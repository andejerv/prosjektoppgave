
#ifndef RIGID_BODY_MANEUVERING_FMU_H
#define RIGID_BODY_MANEUVERING_FMU_H

#include "cppfmu_cs.hpp"
#include "physics_fmu_base_class/physics_fmu_base_class.hpp"
#include "rigid_body_maneuvering.hpp"

class RigidBodyManeuveringFMU : public FMUBaseClass
{
   public:
    explicit RigidBodyManeuveringFMU(const std::string& instance_name);
    RigidBodyManeuveringFMU() = default;

    void ExitInitializationMode() override;

    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean new_step,
                cppfmu::FMIReal& end_of_step) override;

   private:
    std::unique_ptr<RigidBodyManeuvering> m_model;

    // --------------- parameters -----------------
    const cppfmu::FMIValueReference m_mass_i = 1;
    const cppfmu::FMIValueReference m_moment_of_inertia_i = 2;

    const cppfmu::FMIValueReference m_added_mass_surge_i = 3;
    const cppfmu::FMIValueReference m_added_mass_sway_i = 4;
    const cppfmu::FMIValueReference m_added_mass_yaw_i = 5;

    const cppfmu::FMIValueReference m_initial_position_x_i = 6;
    const cppfmu::FMIValueReference m_initial_position_y_i = 7;
    const cppfmu::FMIValueReference m_initial_heading_i = 8;

    const cppfmu::FMIValueReference m_initial_velocity_surge_i = 9;
    const cppfmu::FMIValueReference m_initial_velocity_sway_i = 10;
    const cppfmu::FMIValueReference m_initial_velocity_yaw_i = 11;

    const cppfmu::FMIValueReference m_longitudinal_center_of_gravity_i = 12;

    // --------------- input -----------------
    const cppfmu::FMIValueReference m_external_force_surge_i = 101;
    const cppfmu::FMIValueReference m_external_force_sway_i = 102;
    const cppfmu::FMIValueReference m_external_force_yaw_i = 103;

    // --------------- output -----------------
    const cppfmu::FMIValueReference m_x_i = 201;
    const cppfmu::FMIValueReference m_y_i = 202;
    const cppfmu::FMIValueReference m_heading_i = 203;

    const cppfmu::FMIValueReference m_velocity_surge_i = 204;
    const cppfmu::FMIValueReference m_velocity_sway_i = 205;
    const cppfmu::FMIValueReference m_velocity_yaw_i = 206;
};

#endif