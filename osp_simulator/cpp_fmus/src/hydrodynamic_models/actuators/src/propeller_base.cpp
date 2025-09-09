#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"
#define _USE_MATH_DEFINES
#include <math.h>

#include "hydrodynamic_models/actuators/include/propeller_base.hpp"

PropellerBase::PropellerBase(std::unique_ptr<IOpenWaterPropellerCharacteristics> open_water_chars)
: m_open_water_chars(std::move(open_water_chars)){};

void PropellerBase::update_state()
{
    calculate_inflow_velocity();
    calculate_advance_ratio();

    auto thrust_coefficient = m_open_water_chars->get_thrust_coefficient(m_advance_ratio, m_rotations_per_second);
    auto raw_thrust = thrust_coefficient * m_density * pow(m_rotations_per_second, 2.0) * pow(m_diameter, 4.0);
    m_thrust = raw_thrust * (1 - m_thrust_deduction_factor);

    auto torque_coefficient = m_open_water_chars->get_torque_coefficient(m_advance_ratio, m_rotations_per_second);
    m_torque = torque_coefficient * m_density * pow(m_rotations_per_second, 2.0) * pow(m_diameter, 5.0);

    m_power = 2 * M_PI * m_rotations_per_second * m_torque;

    calculate_body_force();
}

void PropellerBase::calculate_inflow_velocity()
{
    ManeuveringBodyVector inflow_velocity{0.0, 0.0, 0.0};

    inflow_velocity.surge = m_ship_velocity.surge * (1 - m_wake_factor) - m_lateral_pos * m_ship_velocity.yaw;
    inflow_velocity.sway = m_ship_velocity.sway + m_longitudinal_pos * m_ship_velocity.yaw;
    inflow_velocity.yaw = 0.0;

    m_inflow_velocity = inflow_velocity.surge * cos(m_angle) + inflow_velocity.sway * sin(m_angle);
}

void PropellerBase::calculate_advance_ratio()
{
    if (abs(m_rotations_per_second) > divide_by_zero_tol && m_diameter > divide_by_zero_tol) {
        m_advance_ratio = m_inflow_velocity / (m_diameter * m_rotations_per_second);
    }
    else {
        m_advance_ratio = 0.0;
    }
}

void PropellerBase::calculate_body_force()
{
    m_body_force.surge = m_thrust * cos(m_angle);
    m_body_force.sway = m_thrust * sin(m_angle);
    m_body_force.yaw = m_thrust * (-m_lateral_pos * cos(m_angle) + m_longitudinal_pos * sin(m_angle));
}
