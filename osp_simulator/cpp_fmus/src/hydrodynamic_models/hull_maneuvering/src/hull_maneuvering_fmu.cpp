#include "hull_maneuvering_fmu.hpp"

#include <cstring>

#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

HullManeuveringFMU::HullManeuveringFMU(const std::string& instance_name) : FMUBaseClass(instance_name)
{
    // parameters
    m_real_signals.insert(std::make_pair(m_density_i, 1025.9));
    m_real_signals.insert(std::make_pair(m_kinematic_viscosity_i, 1.19e-6));

    m_real_signals.insert(std::make_pair(m_length_i, 0.0));
    m_real_signals.insert(std::make_pair(m_depth_i, 0.0));
    m_real_signals.insert(std::make_pair(m_wetted_surface_i, 0.0));

    m_real_signals.insert(std::make_pair(m_shape_factor_i, 0.0));
    m_real_signals.insert(std::make_pair(m_cr_m_i, 0.0));
    m_real_signals.insert(std::make_pair(m_cr_p_i, 0.0));
    m_real_signals.insert(std::make_pair(m_cd_lateral_i, 0.0));

    m_real_signals.insert(std::make_pair(m_xvv_i, 0.0));
    m_real_signals.insert(std::make_pair(m_xvr_i, 0.0));
    m_real_signals.insert(std::make_pair(m_xrr_i, 0.0));
    m_real_signals.insert(std::make_pair(m_xvvvv_i, 0.0));

    m_real_signals.insert(std::make_pair(m_yv_i, 0.0));
    m_real_signals.insert(std::make_pair(m_yr_i, 0.0));
    m_real_signals.insert(std::make_pair(m_yvvv_i, 0.0));
    m_real_signals.insert(std::make_pair(m_yvvr_i, 0.0));
    m_real_signals.insert(std::make_pair(m_yvrr_i, 0.0));
    m_real_signals.insert(std::make_pair(m_yrrr_i, 0.0));

    m_real_signals.insert(std::make_pair(m_nv_i, 0.0));
    m_real_signals.insert(std::make_pair(m_nr_i, 0.0));
    m_real_signals.insert(std::make_pair(m_nvvv_i, 0.0));
    m_real_signals.insert(std::make_pair(m_nvvr_i, 0.0));
    m_real_signals.insert(std::make_pair(m_nvrr_i, 0.0));
    m_real_signals.insert(std::make_pair(m_nrrr_i, 0.0));

    m_real_signals.insert(std::make_pair(m_cx_low_i, 0.04));
    m_real_signals.insert(std::make_pair(m_cy_low_i, 0.70));
    m_real_signals.insert(std::make_pair(m_cn_low_i, 0.15));
    m_real_signals.insert(std::make_pair(m_nr_low_i, -2200));

    m_real_signals.insert(std::make_pair(m_speed_limit_low_i, 0.5 * 0.514444444));
    m_real_signals.insert(std::make_pair(m_speed_transfer_range_i, 0.5 * 0.514444444));

    m_boolean_signals.insert(std::make_pair(m_low_speed_model_i, true));
    m_boolean_signals.insert(std::make_pair(m_high_speed_model_i, true));

    // input
    m_real_signals.insert(std::make_pair(m_velocity_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_velocity_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_velocity_yaw_i, 0.0));

    // ouput
    m_real_signals.insert(std::make_pair(m_force_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_force_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_yaw_i, 0.0));

    m_real_signals.insert(std::make_pair(m_force_heave_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_roll_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_pitch_i, 0.0));

    m_model = std::make_unique<HullManeuvering>();
}

void HullManeuveringFMU::ExitInitializationMode()
{
    m_model->density = m_real_signals.at(m_density_i);
    m_model->kinematic_viscosity = m_real_signals.at(m_kinematic_viscosity_i);

    m_model->length = m_real_signals.at(m_length_i);
    m_model->depth = m_real_signals.at(m_depth_i);
    m_model->wetted_surface = m_real_signals.at(m_wetted_surface_i);

    m_model->shape_factor = m_real_signals.at(m_shape_factor_i);
    m_model->CR_m = m_real_signals.at(m_cr_m_i);
    m_model->CR_p = m_real_signals.at(m_cr_p_i);
    m_model->CD_lateral = m_real_signals.at(m_cd_lateral_i);

    m_model->Xvv = m_real_signals.at(m_xvv_i);
    m_model->Xvr = m_real_signals.at(m_xvr_i);
    m_model->Xrr = m_real_signals.at(m_xrr_i);
    m_model->Xvvvv = m_real_signals.at(m_xvvvv_i);

    m_model->Yv = m_real_signals.at(m_yv_i);
    m_model->Yr = m_real_signals.at(m_yr_i);
    m_model->Yvvv = m_real_signals.at(m_yvvv_i);
    m_model->Yvvr = m_real_signals.at(m_yvvr_i);
    m_model->Yvrr = m_real_signals.at(m_yvrr_i);
    m_model->Yrrr = m_real_signals.at(m_yrrr_i);

    m_model->Nv = m_real_signals.at(m_nv_i);
    m_model->Nr = m_real_signals.at(m_nr_i);
    m_model->Nvvv = m_real_signals.at(m_nvvv_i);
    m_model->Nvvr = m_real_signals.at(m_nvvr_i);
    m_model->Nvrr = m_real_signals.at(m_nvrr_i);
    m_model->Nrrr = m_real_signals.at(m_nrrr_i);

    m_model->CX_low = m_real_signals.at(m_cx_low_i);
    m_model->CY_low = m_real_signals.at(m_cy_low_i);
    m_model->CN_low = m_real_signals.at(m_cn_low_i);
    m_model->Nr_low = m_real_signals.at(m_nr_low_i);

    m_model->speed_limit_low = m_real_signals.at(m_speed_limit_low_i);
    m_model->speed_transfer_range = m_real_signals.at(m_speed_transfer_range_i);

    if (m_boolean_signals.at(m_high_speed_model_i) && m_boolean_signals.at(m_low_speed_model_i)) {
        m_model->model_mode = ModelMode::COMBINED;
    }
    else if (m_boolean_signals.at(m_high_speed_model_i)) {
        m_model->model_mode = ModelMode::HIGH_SPEED;
    }
    else if (m_boolean_signals.at(m_low_speed_model_i)) {
        m_model->model_mode = ModelMode::LOW_SPEED;
    }
    else {
        m_model->model_mode = ModelMode::COMBINED;
    }
}

bool HullManeuveringFMU::DoStep(cppfmu::FMIReal /*t*/, cppfmu::FMIReal /*dt*/, cppfmu::FMIBoolean /*new_step*/,
                                cppfmu::FMIReal& /*end_of_step*/)
{
    double u = m_real_signals.at(m_velocity_surge_i);
    double v = m_real_signals.at(m_velocity_sway_i);
    double r = m_real_signals.at(m_velocity_yaw_i);

    m_model->set_velocity(u, v, r);

    ManeuveringBodyVector force = m_model->calculate_force();

    m_real_signals.at(m_force_surge_i) = force.surge;
    m_real_signals.at(m_force_sway_i) = force.sway;
    m_real_signals.at(m_moment_yaw_i) = force.yaw;

    return true;
}

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString instance_name, cppfmu::FMIString fmu_guid, cppfmu::FMIString /*fmu_resource_location*/,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger /*logger*/)
{
    if (strcmp(fmu_guid, FMU_UUID) != 0) {
        throw std::runtime_error("FMU GUID mismatch");
    }

    return cppfmu::AllocateUnique<HullManeuveringFMU>(memory, instance_name);
}