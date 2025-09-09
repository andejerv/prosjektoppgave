#include "simple_thruster.hpp"

#include <cmath>

void SimpleThruster::set_parameters(const Vector& p0, const Vector& orientation0, const double max_thrust)
{
    m_p0 = p0;
    m_orientation0 = orientation0;
    m_max_thrust = max_thrust;

    m_orientation = orientation0;
}

void SimpleThruster::set_angle(const double angle)
{
    m_angle = angle;

    m_orientation.x = m_orientation0.x * std::cos(m_angle) + m_orientation0.y * std::sin(m_angle);
    m_orientation.y = -m_orientation0.x * std::sin(m_angle) + m_orientation0.y * std::cos(m_angle);
}

void SimpleThruster::set_loading(const double loading)
{
    m_loading = loading;
}

ManeuveringBodyVector SimpleThruster::do_step()
{
    ManeuveringBodyVector result;

    result.surge = m_orientation.x * m_max_thrust * m_loading;
    result.sway = m_orientation.y * m_max_thrust * m_loading;
    result.yaw = result.sway * m_p0.x - result.surge * m_p0.y;

    return result;
}