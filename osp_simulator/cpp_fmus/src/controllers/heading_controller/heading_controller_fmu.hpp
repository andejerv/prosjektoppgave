/*----------------------------------------------------------------------------*\

Autopilot that adjust the angle of a rudder / thruster to keep a certain heading

\*----------------------------------------------------------------------------*/

#ifndef HEADING_CONTROLLER_H
#define HEADING_CONTROLLER_H

#include <unordered_map>

#include "cppfmu_cs.hpp"
#include "physics_fmu_base_class.hpp"
#include "pid_controller.hpp"

class HeadingControllerFMU : public FMUBaseClass
{
   public:
    HeadingControllerFMU(const std::string& instanceName);
    HeadingControllerFMU() = default;

    void ExitInitializationMode();

    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean newStep, cppfmu::FMIReal& endOfStep) override;

   private:
    std::unique_ptr<PIDController> m_controller;

    // ------------------------- parameters ------------------------------------
    const cppfmu::FMIValueReference m_K_P = 1;
    const cppfmu::FMIValueReference m_K_I = 2;
    const cppfmu::FMIValueReference m_K_D = 3;

    const cppfmu::FMIValueReference m_MIN_ANGLE = 4;
    const cppfmu::FMIValueReference m_MAX_ANGLE = 5;

    // --------------------------- input ---------------------------------------
    const cppfmu::FMIValueReference m_TARGET_HEADING = 11;
    const cppfmu::FMIValueReference m_ESTIMATED_HEADING = 12;
    const cppfmu::FMIValueReference m_PID_ON = 13;
    const cppfmu::FMIValueReference m_MANUEL_ANGLE = 14;
    const cppfmu::FMIValueReference m_GAIN_ADJUST = 15;

    // --------------------------- output --------------------------------------
    const cppfmu::FMIValueReference m_ANGLE = 21;
    const cppfmu::FMIValueReference m_ERROR = 22;
};

#endif  // end header guard