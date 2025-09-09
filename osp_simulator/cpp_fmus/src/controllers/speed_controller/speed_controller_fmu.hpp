/*----------------------------------------------------------------------------*\

Autopilot that adjust the "loading" of a propeller to keep a certain target
speed. The controller does not assumes anything about what the loading
represents. Could be a non-dimensional parameter or rotational speed. Dependent
on the propeller model that gets the signal from this controller

\*----------------------------------------------------------------------------*/

#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <unordered_map>

#include "cppfmu_cs.hpp"
#include "physics_fmu_base_class.hpp"
#include "pid_controller.hpp"

class SpeedControllerFMU : public FMUBaseClass
{
   public:
    SpeedControllerFMU(const std::string& instanceName);
    SpeedControllerFMU() = default;

    void ExitInitializationMode();

    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean newStep, cppfmu::FMIReal& endOfStep) override;

   private:
    void SetControllerParameters();

    std::unique_ptr<PIDController> m_controller;

    // ------------------------- parameters ------------------------------------
    const cppfmu::FMIValueReference m_K_P = 1;
    const cppfmu::FMIValueReference m_K_I = 2;
    const cppfmu::FMIValueReference m_K_D = 3;

    const cppfmu::FMIValueReference m_MIN_LOADING = 4;
    const cppfmu::FMIValueReference m_MAX_LOADING = 5;

    // --------------------------- input ---------------------------------------
    const cppfmu::FMIValueReference m_TARGET_SPEED = 11;
    const cppfmu::FMIValueReference m_ESTIMATED_U = 12;
    const cppfmu::FMIValueReference m_ESTIMATED_V = 13;

    // --------------------------- output --------------------------------------
    const cppfmu::FMIValueReference m_LOADING = 21;
    const cppfmu::FMIValueReference m_ERROR = 22;
};

#endif  // end header guard