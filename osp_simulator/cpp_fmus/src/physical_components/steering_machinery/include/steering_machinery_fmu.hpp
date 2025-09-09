#ifndef STEERINGMACHINERYFMU_H
#define STEERINGMACHINERYFMU_H

#include <cppfmu_cs.hpp>
#include <unordered_map>

#include "steering_machinery.hpp"

class SteeringMachineryFMU : public cppfmu::SlaveInstance
{
   public:
    explicit SteeringMachineryFMU(const std::string& instance_name);
    SteeringMachineryFMU() = default;

    void ExitInitializationMode() override;

    void SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[]) override;
    void GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const override;
    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean new_step,
                cppfmu::FMIReal& end_of_step) override;

   private:
    std::unique_ptr<SteeringMachinery> m_model;

    const std::string model_name;

    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIReal> m_real_signals;

    // --------------- parameters -----------------
    const cppfmu::FMIValueReference max_speed_vr = 1;
    const cppfmu::FMIValueReference initial_angle_vr = 2;
    const cppfmu::FMIValueReference angle_stop_1_vr = 3;
    const cppfmu::FMIValueReference angle_stop_2_vr = 4;

    // --------------- input -----------------
    const cppfmu::FMIValueReference target_angle_vr = 11;

    // --------------- output -----------------
    const cppfmu::FMIValueReference angle_vr = 21;
};

#endif  // end header guard