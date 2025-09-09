#include "hydrodynamic_models/actuators/include/rudder_base.hpp"

#include <math.h>

#include "hydrodynamic_models/hydro_utils/include/hydro_functions.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

void RudderBase::set_ship_velocity(const ManeuveringBodyVector& ship_velocity)
{
    m_ship_velocity = ship_velocity;
}

void RudderBase::set_rudder_angle(const double angle)
{
    m_rudder_angle = angle;
}

void RudderBase::set_rudder_geometry(const double area, const double chord)
{
    m_area = area;
    m_chord = chord;
}
void RudderBase::set_rudder_placement(const double longitudinal_pos, const double lateral_pos)
{
    m_longitudinal_position = longitudinal_pos;
    m_lateral_position = lateral_pos;
}

void RudderBase::set_fluid_properies(const double density, const double kinematic_viscosity)
{
    m_density = density;
    m_kinematic_viscosity = kinematic_viscosity;
}

ManeuveringBodyVector RudderBase::get_body_force_from_lift_and_drag(const double lift, const double drag)
{
    double velocity_magnitude = get_velocity_magnitude(m_inflow_velocity);

    Vector drag_dir;

    if (velocity_magnitude > divide_by_zero_tol) {
        drag_dir.x = -m_inflow_velocity.surge / velocity_magnitude;
        drag_dir.y = -m_inflow_velocity.sway / velocity_magnitude;
    }
    else {
        drag_dir.x = -1.0;
    }

    Vector lift_dir = cross_product(m_ROTATION_DIR, drag_dir);

    ManeuveringBodyVector body_forces;

    body_forces.surge = lift * lift_dir.x + drag * drag_dir.x;
    body_forces.sway = lift * lift_dir.y + drag * drag_dir.y;
    body_forces.yaw = body_forces.sway * m_longitudinal_position - body_forces.surge * m_lateral_position;

    return body_forces;
}

double RudderBase::get_inflow_angle() const
{
    return m_rudder_angle - atan2(m_inflow_velocity.sway, m_inflow_velocity.surge);
}