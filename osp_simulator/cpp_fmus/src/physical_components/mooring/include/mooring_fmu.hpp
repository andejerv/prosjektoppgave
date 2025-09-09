/*----------------------------------------------------------------------------*\

FMU interface class for mooring to quay when hatch is lowered

\*----------------------------------------------------------------------------*/

#ifndef MOORINGFMU_H
#define MOORINGFMU_H

#include "cppfmu_cs.hpp"
#include "physics_fmu_base_class.hpp"

class MooringFMU : public FMUBaseClass
{
   public:
    explicit MooringFMU(const std::string& instance_name);
    MooringFMU() = default;

    void ExitInitializationMode() override;

    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean new_step,
                cppfmu::FMIReal& end_of_step) override;

   private:
    // --------------- input -----------------
    const cppfmu::FMIValueReference m_hatch_position_fore_i = 101;
    const cppfmu::FMIValueReference m_hatch_position_aft_i = 102;

    // --------------- output -----------------
    const cppfmu::FMIValueReference m_moored_i = 201;
};

#endif  // end header guard