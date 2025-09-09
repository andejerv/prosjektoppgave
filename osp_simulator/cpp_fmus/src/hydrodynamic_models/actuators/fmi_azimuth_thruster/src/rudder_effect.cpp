#include "hydrodynamic_models/actuators/fmi_azimuth_thruster/include/rudder_effect.hpp"

#include <math.h>

#include "hydrodynamic_models/hydro_utils/include/hydro_functions.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

RudderEffect::RudderEffect(std::unique_ptr<LiftingSurface> lifting_surface)
: m_lifting_surface(std::move(lifting_surface))
{
}

ManeuveringBodyVector RudderEffect::get_body_force()
{
    m_inflow_velocity = get_inflow_velocity();
    auto velocity_magnitude = get_velocity_magnitude(m_inflow_velocity);
    auto reynolds_number = velocity_magnitude * m_chord / m_kinematic_viscosity;
    auto inflow_angle = get_inflow_angle();

    m_lifting_surface->set_alpha(inflow_angle);
    m_lifting_surface->set_reynolds_number(reynolds_number);
    double cl = m_lifting_surface->get_cl();
    double cd = m_lifting_surface->get_cd(cl);
    double force_factor = 0.5 * m_density * m_area * pow(velocity_magnitude, 2.0);
    double lift = cl * force_factor;
    double drag = cd * force_factor;

    auto direct_rudder_force = get_body_force_from_lift_and_drag(lift, drag);
    ManeuveringBodyVector induced_rudder_force = calc_induced_rudder_force(direct_rudder_force);

    ManeuveringBodyVector total_rudder_force;
    total_rudder_force.surge = direct_rudder_force.surge + induced_rudder_force.surge;
    total_rudder_force.sway = direct_rudder_force.sway + induced_rudder_force.sway;
    total_rudder_force.yaw = direct_rudder_force.yaw + induced_rudder_force.yaw;

    return total_rudder_force;
}

void RudderEffect::set_hull_interaction_params(const HullInteractionParameters& params)
{
    m_hull_interaction_params = params;
}

ManeuveringBodyVector RudderEffect::calc_induced_rudder_force(const ManeuveringBodyVector& direct_rudder_force) const
{
    ManeuveringBodyVector induced_force;

    induced_force.surge = -direct_rudder_force.surge * m_hull_interaction_params.tr;
    induced_force.sway = direct_rudder_force.sway * m_hull_interaction_params.ah;
    induced_force.yaw = induced_force.sway * m_hull_interaction_params.xh * m_hull_interaction_params.ship_length;

    return induced_force;
}

ManeuveringBodyVector RudderEffect::get_inflow_velocity() const
{
    ManeuveringBodyVector inflow_velocity;

    inflow_velocity.surge =
        m_ship_velocity.surge * (1 - m_hull_interaction_params.wake_factor) - m_lateral_position * m_ship_velocity.yaw;
    inflow_velocity.sway = m_ship_velocity.sway + m_longitudinal_position * m_ship_velocity.yaw;
    inflow_velocity.yaw = 0.0;

    return inflow_velocity;
}