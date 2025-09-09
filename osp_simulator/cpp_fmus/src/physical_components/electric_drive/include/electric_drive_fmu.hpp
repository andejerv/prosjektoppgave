#ifndef PHYSICAL_COMPONENTS_ELECTRIC_DRIVE_INCLUDE_ELECTRIC_DRIVE_FMU_HPP
#define PHYSICAL_COMPONENTS_ELECTRIC_DRIVE_INCLUDE_ELECTRIC_DRIVE_FMU_HPP

#include <cppfmu_cs.hpp>
#include <unordered_map>

class ElectricDriveFMU : public cppfmu::SlaveInstance
{
   public:
    explicit ElectricDriveFMU(const std::string& instance_name);
    ElectricDriveFMU() = default;

    void ExitInitializationMode() override;

    void SetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, const cppfmu::FMIReal value[]) override;
    void GetReal(const cppfmu::FMIValueReference vr[], std::size_t nvr, cppfmu::FMIReal value[]) const override;
    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean new_step,
                cppfmu::FMIReal& end_of_step) override;

   private:
    const std::string model_name;

    std::unordered_map<cppfmu::FMIValueReference, cppfmu::FMIReal> m_real_signals;

    // --------------- parameters -----------------
    const cppfmu::FMIValueReference max_rpm_vr = 1;
    const cppfmu::FMIValueReference time_constant_vr = 2;
    const cppfmu::FMIValueReference max_rpm_rate_vr = 3;

    // --------------- input -----------------
    const cppfmu::FMIValueReference rpm_command_vr = 11;

    // --------------- output -----------------
    const cppfmu::FMIValueReference rpm_vr = 21;
};

#endif /* PHYSICAL_COMPONENTS_ELECTRIC_DRIVE_INCLUDE_ELECTRIC_DRIVE_FMU_HPP */
