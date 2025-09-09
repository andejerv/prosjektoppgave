#ifndef SIMPLE_THRUSTER_FMU_H
#define SIMPLE_THRUSTER_FMU_H

#include "physics_fmu_base_class/physics_fmu_base_class.hpp"
#include "simple_thruster.hpp"

class SimpleThrusterFMU : public FMUBaseClass
{
   public:
    explicit SimpleThrusterFMU(const std::string& instance_name);
    SimpleThrusterFMU() = default;

    void ExitInitializationMode() override;

    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean new_step,
                cppfmu::FMIReal& end_of_step) override;

   private:
    std::unique_ptr<SimpleThruster> m_model;

    // --------------- parameters -----------------
    const cppfmu::FMIValueReference m_p0_x_i = 1;
    const cppfmu::FMIValueReference m_p0_y_i = 2;
    const cppfmu::FMIValueReference m_orientation0_x_i = 3;
    const cppfmu::FMIValueReference m_orientation0_y_i = 4;
    const cppfmu::FMIValueReference m_max_thrust_i = 5;

    // --------------- input -----------------
    const cppfmu::FMIValueReference m_angle_i = 101;
    const cppfmu::FMIValueReference m_loading_i = 102;

    // --------------- output -----------------
    const cppfmu::FMIValueReference m_force_surge_i = 201;
    const cppfmu::FMIValueReference m_force_sway_i = 202;
    const cppfmu::FMIValueReference m_moment_yaw_i = 203;
};

#endif  // end header guard