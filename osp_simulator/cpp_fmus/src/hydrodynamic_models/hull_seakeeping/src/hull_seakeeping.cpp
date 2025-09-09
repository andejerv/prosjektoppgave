#include "hull_seakeeping.hpp"

HullSeakeeping::HullSeakeeping()
{
    mass = waterplane_area = 0.0;
    heave_damping = roll_damping = pitch_damping = 0.0;
    GM_roll = GM_pitch = 0.0;

    this->set_velocity(0.0, 0.0, 0.0);
    this->set_displacement(0.0, 0.0, 0.0);

    m_g_heave = m_g_roll = m_g_pitch = 0.0;
}

void HullSeakeeping::set_velocity(const double heave, const double roll, const double pitch)
{
    m_velocity.heave = heave;
    m_velocity.roll = roll;
    m_velocity.pitch = pitch;
}

void HullSeakeeping::set_displacement(const double heave, const double roll, const double pitch)
{
    m_displacement.heave = heave;
    m_displacement.roll = roll;
    m_displacement.pitch = pitch;
}

void HullSeakeeping::calc_restoring_force_coeffients()
{
    m_g_heave = density * gravity * waterplane_area;
    m_g_roll = mass * gravity * GM_roll;
    m_g_pitch = mass * gravity * GM_pitch;
}
void HullSeakeeping::calc_force()
{
    m_force.heave = -(heave_damping * m_velocity.heave + m_g_heave * m_displacement.heave);
    m_force.roll = -(roll_damping * m_velocity.roll + m_g_roll * m_displacement.roll);
    m_force.pitch = -(pitch_damping * m_velocity.pitch + m_g_pitch * m_displacement.pitch);
}

SeakeepingBodyVector HullSeakeeping::get_force() const
{
    return m_force;
}