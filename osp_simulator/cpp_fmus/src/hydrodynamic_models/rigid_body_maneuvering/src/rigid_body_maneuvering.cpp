#include "rigid_body_maneuvering.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

RigidBodyManeuvering::RigidBodyManeuvering()
{
    m_coriolis_matrix = Eigen::Matrix3d::Zero(3, 3);
    m_inertia_matrix = Eigen::Matrix3d::Zero(3, 3);
}

void RigidBodyManeuvering::compute_inertia_matrix()
{
    m_inertia_matrix(0, 0) = mass + added_mass.surge;
    m_inertia_matrix(1, 1) = mass + added_mass.sway;
    m_inertia_matrix(2, 2) = moment_of_inertia + added_mass.yaw + mass * pow(long_center_of_gravity, 2.0);

    m_inertia_matrix(1, 2) = mass * long_center_of_gravity;
    m_inertia_matrix(2, 1) = mass * long_center_of_gravity;

    m_inverse_inertia_matrix = m_inertia_matrix.inverse();
}

void RigidBodyManeuvering::compute_coriolis_matrix(const double yaw_velocity)
{
    m_coriolis_matrix(0, 1) = -(mass + added_mass.sway) * yaw_velocity;
    m_coriolis_matrix(0, 2) = -long_center_of_gravity * mass * yaw_velocity;

    m_coriolis_matrix(1, 0) = (mass + added_mass.surge) * yaw_velocity;

    m_coriolis_matrix(2, 0) = long_center_of_gravity * mass * yaw_velocity;
    ;
}

void RigidBodyManeuvering::compute_acceleration()
{
    this->compute_coriolis_matrix(m_velocity.yaw);

    Eigen::Vector3d force_vector(m_external_force.surge, m_external_force.sway, m_external_force.yaw);

    Eigen::Vector3d velocity_vector(m_velocity.surge, m_velocity.sway, m_velocity.yaw);

    Eigen::Vector3d effective_force = force_vector - m_coriolis_matrix * velocity_vector;

    Eigen::Vector3d acceleration_vector = m_inverse_inertia_matrix * effective_force;

    m_acceleration.surge = acceleration_vector(0);
    m_acceleration.sway = acceleration_vector(1);
    m_acceleration.yaw = acceleration_vector(2);
}

void RigidBodyManeuvering::do_step(const double dt)
{
    compute_inertia_matrix();
    compute_coriolis_matrix(m_velocity.yaw);
    compute_acceleration();

    // ------ do time integration -----------
    m_velocity.surge += m_acceleration.surge * dt;
    m_velocity.sway += m_acceleration.sway * dt;
    m_velocity.yaw += m_acceleration.yaw * dt;

    double c = cos(m_position.heading);
    double s = sin(m_position.heading);

    m_position.x += (m_velocity.surge * c - m_velocity.sway * s) * dt;
    m_position.y += (m_velocity.sway * c + m_velocity.surge * s) * dt;

    m_position.heading += m_velocity.yaw * dt;
}

ManeuveringPositionVector RigidBodyManeuvering::get_position() const
{
    return m_position;
}
ManeuveringBodyVector RigidBodyManeuvering::get_velocity() const
{
    return m_velocity;
}
ManeuveringBodyVector RigidBodyManeuvering::get_acceleration() const
{
    return m_acceleration;
}

void RigidBodyManeuvering::set_external_force(const ManeuveringBodyVector& force)
{
    m_external_force = force;
}

void RigidBodyManeuvering::set_position(const ManeuveringPositionVector& position)
{
    m_position = position;
}

void RigidBodyManeuvering::set_veloctity(const ManeuveringBodyVector& velocity)
{
    m_velocity = velocity;
}