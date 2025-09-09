#include "hydrodynamic_models/actuators/fmi_rudder/include/mmg_rudder.hpp"

#include "hydrodynamic_models/hydro_utils/include/hydro_functions.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

MMGRudder::MMGRudder(std::unique_ptr<LiftingSurface> lifting_surface, std::unique_ptr<HullInteraction> hull_interaction)
: m_lifting_surface(std::move(lifting_surface)), m_hull_interaction(std::move(hull_interaction))
{
}

ManeuveringBodyVector MMGRudder::get_body_force()
{
    m_inflow_velocity = m_hull_interaction->get_effective_velocity(m_ship_velocity);
    m_inflow_velocity.surge *= m_surge_velocity_increase_factor;

    double velocity_magnitude = get_velocity_magnitude(m_inflow_velocity);
    double force_factor = 0.5 * m_density * m_area * pow(velocity_magnitude, 2.0);

    double effective_angle = m_hull_interaction->get_effective_angle(m_rudder_angle, m_inflow_velocity);
    double reynolds_number = velocity_magnitude * m_chord / m_kinematic_viscosity;

    m_lifting_surface->set_alpha(effective_angle);
    m_lifting_surface->set_reynolds_number(reynolds_number);

    ManeuveringBodyVector direct_force;

    if (m_model_mode == ModelMode::LIFTING_LINE) {
        double cl = m_lifting_surface->get_cl();
        double cd = m_lifting_surface->get_cd(cl);

        double lift = cl * force_factor;
        double drag = cd * force_factor;

        direct_force = get_body_force_from_lift_and_drag(lift, drag);
    }
    else {
        double fa = m_lifting_surface->get_normal_force_coefficient();
        double fn = fa * force_factor;

        double surge_factor = fn * sin(m_rudder_angle);
        double sway_factor = fn * cos(m_rudder_angle);

        direct_force.surge = -surge_factor;
        direct_force.sway = -sway_factor;
        direct_force.yaw = -m_longitudinal_position * sway_factor;
    }

    ManeuveringBodyVector induced_force = m_hull_interaction->get_induced_hull_force(direct_force);

    ManeuveringBodyVector combined_force;

    combined_force.surge = direct_force.surge + induced_force.surge;
    combined_force.sway = direct_force.sway + induced_force.sway;
    combined_force.yaw = direct_force.yaw + induced_force.yaw;

    return combined_force;
}