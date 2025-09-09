#include "rigid_body_maneuvering_fmu.hpp"

RigidBodyManeuveringFMU::RigidBodyManeuveringFMU(const std::string& instance_name) : FMUBaseClass(instance_name)
{
    // parameters
    m_real_signals.insert(std::make_pair(m_mass_i, 0.0));
    m_real_signals.insert(std::make_pair(m_moment_of_inertia_i, 0.0));

    m_real_signals.insert(std::make_pair(m_added_mass_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_added_mass_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_added_mass_yaw_i, 0.0));

    m_real_signals.insert(std::make_pair(m_initial_position_x_i, 0.0));
    m_real_signals.insert(std::make_pair(m_initial_position_y_i, 0.0));
    m_real_signals.insert(std::make_pair(m_initial_heading_i, 0.0));

    m_real_signals.insert(std::make_pair(m_initial_velocity_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_initial_velocity_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_initial_velocity_yaw_i, 0.0));

    m_real_signals.insert(std::make_pair(m_longitudinal_center_of_gravity_i, 0.0));

    // input
    m_real_signals.insert(std::make_pair(m_external_force_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_external_force_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_external_force_yaw_i, 0.0));

    // ouput
    m_real_signals.insert(std::make_pair(m_x_i, 0.0));
    m_real_signals.insert(std::make_pair(m_y_i, 0.0));
    m_real_signals.insert(std::make_pair(m_heading_i, 0.0));

    m_real_signals.insert(std::make_pair(m_velocity_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_velocity_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_velocity_yaw_i, 0.0));

    m_model = std::make_unique<RigidBodyManeuvering>();
}

void RigidBodyManeuveringFMU::ExitInitializationMode()
{
    m_model->mass = m_real_signals.at(m_mass_i);
    m_model->moment_of_inertia = m_real_signals.at(m_moment_of_inertia_i);

    m_model->long_center_of_gravity = m_real_signals.at(m_longitudinal_center_of_gravity_i);

    m_model->added_mass.surge = m_real_signals.at(m_added_mass_surge_i);
    m_model->added_mass.sway = m_real_signals.at(m_added_mass_sway_i);
    m_model->added_mass.yaw = m_real_signals.at(m_added_mass_yaw_i);

    ManeuveringBodyVector external_force;
    external_force.surge = m_real_signals.at(m_external_force_surge_i);
    external_force.sway = m_real_signals.at(m_external_force_sway_i);
    external_force.yaw = m_real_signals.at(m_external_force_yaw_i);
    m_model->set_external_force(external_force);

    ManeuveringPositionVector position;
    position.x = m_real_signals.at(m_initial_position_x_i);
    position.y = m_real_signals.at(m_initial_position_y_i);
    position.heading = m_real_signals.at(m_initial_heading_i);
    m_model->set_position(position);

    ManeuveringBodyVector velocity;
    velocity.surge = m_real_signals.at(m_initial_velocity_surge_i);
    velocity.sway = m_real_signals.at(m_initial_velocity_sway_i);
    velocity.yaw = m_real_signals.at(m_initial_velocity_yaw_i);
    m_model->set_veloctity(velocity);
}

bool RigidBodyManeuveringFMU::DoStep(cppfmu::FMIReal /*t*/, cppfmu::FMIReal dt, cppfmu::FMIBoolean /*newStep*/,
                                     cppfmu::FMIReal& /*endOfStep*/)
{
    ManeuveringBodyVector external_force;
    external_force.surge = m_real_signals.at(m_external_force_surge_i);
    external_force.sway = m_real_signals.at(m_external_force_sway_i);
    external_force.yaw = m_real_signals.at(m_external_force_yaw_i);
    m_model->set_external_force(external_force);

    m_model->do_step(dt);

    auto position = m_model->get_position();
    m_real_signals.at(m_x_i) = position.x;
    m_real_signals.at(m_y_i) = position.y;
    m_real_signals.at(m_heading_i) = position.heading;

    auto velocity = m_model->get_velocity();
    m_real_signals.at(m_velocity_surge_i) = velocity.surge;
    m_real_signals.at(m_velocity_sway_i) = velocity.sway;
    m_real_signals.at(m_velocity_yaw_i) = velocity.yaw;

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

    return cppfmu::AllocateUnique<RigidBodyManeuveringFMU>(memory, instance_name);
}