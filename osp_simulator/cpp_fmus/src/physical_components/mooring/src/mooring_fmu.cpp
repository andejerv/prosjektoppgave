#include "mooring_fmu.hpp"

#define _USE_MATH_DEFINES
#include <cstring>

MooringFMU::MooringFMU(const std::string& instance_name) : FMUBaseClass(instance_name)
{
    // input
    m_real_signals.insert(std::make_pair(m_hatch_position_fore_i, 100.0));
    m_real_signals.insert(std::make_pair(m_hatch_position_aft_i, 100.0));

    // ouput
    m_boolean_signals.insert(std::make_pair(m_moored_i, false));
}

void MooringFMU::ExitInitializationMode()
{
}

bool MooringFMU::DoStep(cppfmu::FMIReal /*t*/, cppfmu::FMIReal /*dt*/, cppfmu::FMIBoolean /*new_step*/,
                        cppfmu::FMIReal& /*end_of_step*/)
{
    double fore_hatch_pos = m_real_signals.at(m_hatch_position_fore_i);
    double aft_hatch_pos = m_real_signals.at(m_hatch_position_aft_i);

    if (fore_hatch_pos < 1.0 || aft_hatch_pos < 1.0) {
        m_boolean_signals.at(m_moored_i) = true;
    }
    else {
        m_boolean_signals.at(m_moored_i) = false;
    }

    return true;
}

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString instance_name, cppfmu::FMIString fmu_guid, cppfmu::FMIString /*fmu_resource_location*/,
    cppfmu::FMIString /*mime_type*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger /*logger*/)
{
    if (strcmp(fmu_guid, FMU_UUID) != 0) {
        throw std::runtime_error("FMU GUID mismatch");
    }

    return cppfmu::AllocateUnique<MooringFMU>(memory, instance_name);
}