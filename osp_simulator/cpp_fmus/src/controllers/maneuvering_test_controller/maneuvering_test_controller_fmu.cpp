#include "maneuvering_test_controller_fmu.hpp"

#include <math.h>

#include <cstring>

ManeuveringTestControllerFMU::ManeuveringTestControllerFMU(const std::string& instance_name)
: FMUBaseClass(instance_name)
{
    // parameters
    m_real_signals.insert(std::make_pair(m_INIT_TIME_I, 0.0));
    m_real_signals.insert(std::make_pair(m_MAX_RUDDER_ANGLE_I, 0.0));
    m_real_signals.insert(std::make_pair(m_HEADING_EXECUTE_I, 0.0));

    // input
    m_real_signals.insert(std::make_pair(m_HEADING_I, 0.0));

    // ouput
    m_real_signals.insert(std::make_pair(m_RUDDER_ANGLE_I, 0.0));

    // ---------------- Initialize directions ------------------------
    m_direction = 1.0;
}

bool ManeuveringTestControllerFMU::DoStep(cppfmu::FMIReal t, cppfmu::FMIReal dt, cppfmu::FMIBoolean newStep,
                                          cppfmu::FMIReal& endOfStep)
{
    double t_cor = t - m_real_signals.at(m_INIT_TIME_I);

    if (t_cor < 0.0) {
        m_real_signals.at(m_RUDDER_ANGLE_I) = 0.0;
    }
    else {
        double heading = m_real_signals.at(m_HEADING_I);

        double heading_mag = sqrt(pow(heading, 2.0));

        double heading_sign;

        if (heading_mag > 0.0) {
            heading_sign = heading / heading_mag;
        }
        else {
            heading_sign = 1.0;
        }

        double heading_execute = m_real_signals.at(m_HEADING_EXECUTE_I);

        if (heading_mag >= heading_execute && heading_sign == m_direction) {
            m_direction *= -1;
        }

        m_real_signals.at(m_RUDDER_ANGLE_I) = m_real_signals.at(m_MAX_RUDDER_ANGLE_I) * m_direction;
    }

    return true;
}

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString instanceName, cppfmu::FMIString fmuGUID, cppfmu::FMIString fmuResourceLocation,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger logger)
{
    if (strcmp(fmuGUID, FMU_UUID) != 0) {
        throw std::runtime_error("FMU GUID mismatch");
    }

    return cppfmu::AllocateUnique<ManeuveringTestControllerFMU>(memory, instanceName);
}
