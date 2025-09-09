#ifndef SET_POINTS_CLIENT_FMU_H
#define SET_POINTS_CLIENT_FMU_H

#include <unordered_map>

#include "physics_fmu_base_class.hpp"
#include "set_points_client.hpp"

class SetPointsClientFMU : public FMUBaseClass
{
   public:
    SetPointsClientFMU(const std::string& instance_name);
    SetPointsClientFMU() = default;

    // OSP methods
    void ExitInitializationMode();

    bool DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean newStep, cppfmu::FMIReal& endOfStep) override;

   private:
    std::unique_ptr<SetPointsClient> m_client;

    const std::string m_instance_name;

    // --------------- parameters -----------------
    const cppfmu::FMIValueReference m_CONNECTION_STRING = 1;

    // --------------- input -----------------
    const cppfmu::FMIValueReference m_X = 11;
    const cppfmu::FMIValueReference m_Y = 12;
    const cppfmu::FMIValueReference m_U = 13;
    const cppfmu::FMIValueReference m_V = 14;
    const cppfmu::FMIValueReference m_HEADING = 15;
    const cppfmu::FMIValueReference m_LOADING = 16;

    // --------------- output -----------------
    const cppfmu::FMIValueReference m_SPEED_SET_POINT = 21;
    const cppfmu::FMIValueReference m_HEADING_SET_POINT = 22;

    const cppfmu::FMIValueReference m_SPEED_PID_ON = 23;
    const cppfmu::FMIValueReference m_HEADING_PID_ON = 24;

    const cppfmu::FMIValueReference m_SPEED_GAIN_ADJUST = 25;
    const cppfmu::FMIValueReference m_HEADING_GAIN_ADJUST = 26;
};

#endif  // end header guard