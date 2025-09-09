#include "steering_machinery.hpp"

#include <algorithm>
#include <cmath>

void SteeringMachinery::set_initial_angle(const double angle)
{
    m_angle = zeabuz::common::utilities::math::inf2pipi(angle);
}

void SteeringMachinery::set_target_angle(const double angle)
{
    m_target_angle = zeabuz::common::utilities::math::inf2pipi(angle);
};
void SteeringMachinery::set_max_speed(const double max_speed)
{
    m_max_speed = std::abs(max_speed);
};

double SteeringMachinery::get_angle() const
{
    return m_angle;
};

void SteeringMachinery::do_step(const double dt)
{
    if (m_angle != m_target_angle && dt > 0.0) {
        double move = zeabuz::common::utilities::math::inf2pipi(m_target_angle - m_angle);

        double required_speed = move / dt;
        double clipped_speed = std::clamp(required_speed, -m_max_speed, m_max_speed);

        m_angle += clipped_speed * dt;
        m_angle = zeabuz::common::utilities::math::inf2pipi(m_angle);
    }
}