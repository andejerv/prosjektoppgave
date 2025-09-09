#ifndef MANEUVERINGTESTCONTROLLER_H
#define MANEUVERINGTESTCONTROLLER_H

#include <unordered_map>

#include "cppfmu_cs.hpp"
#include "physics_fmu_base_class.hpp"

class ManeuveringTestControllerFMU : public FMUBaseClass
{
   public:
    ManeuveringTestControllerFMU(const std::string& instanceName);
    ManeuveringTestControllerFMU() = default;

    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean newStep, cppfmu::FMIReal& endOfStep) override;

   private:
    // --------------- parameters -----------------
    const cppfmu::FMIValueReference m_INIT_TIME_I = 1;
    const cppfmu::FMIValueReference m_MAX_RUDDER_ANGLE_I = 2;
    const cppfmu::FMIValueReference m_HEADING_EXECUTE_I = 3;

    // --------------- input -----------------
    const cppfmu::FMIValueReference m_HEADING_I = 11;

    // --------------- output -----------------
    const cppfmu::FMIValueReference m_RUDDER_ANGLE_I = 21;

    double m_direction;
};

#endif  // end header guard