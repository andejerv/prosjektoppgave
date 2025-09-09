#include "hydrodynamic_models/actuators/include/hull_interaction.hpp"

#include "hydrodynamic_models/hydro_utils/include/hydro_functions.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

ManeuveringBodyVector HullInteraction::get_effective_velocity(const ManeuveringBodyVector& hull_velocity) const
{
    double U = get_velocity_magnitude(hull_velocity);
    double beta = get_drift_angle(hull_velocity);

    double r_non_dim;

    if (U > 0.0) {
        r_non_dim = hull_velocity.yaw * ship_length / U;
    }
    else {
        r_non_dim = 0.0;
    }

    double beta_r_surge = beta - lr_surge * r_non_dim;
    double beta_r_sway = beta - lr_sway * r_non_dim;

    double w_effective = w0 * exp(-4 * pow(beta_r_surge, 2));

    ManeuveringBodyVector effective_velocity;

    effective_velocity.surge = hull_velocity.surge * (1 - w_effective);

    double gamma;

    if (beta >= 0.0) {
        gamma = gamma_positive;
    }
    else {
        gamma = gamma_negative;
    }

    effective_velocity.sway = gamma * beta_r_sway * U;
    effective_velocity.yaw = 0.0;

    return effective_velocity;
}

double HullInteraction::get_effective_angle(const double geometric_angle,
                                            const ManeuveringBodyVector& effective_velocity) const
{
    double effective_angle = geometric_angle - atan2(effective_velocity.sway, effective_velocity.surge);

    return effective_angle;
}

ManeuveringBodyVector HullInteraction::get_induced_hull_force(const ManeuveringBodyVector& direct_force) const
{
    ManeuveringBodyVector induced_force;

    induced_force.surge = -direct_force.surge * tr;
    induced_force.sway = direct_force.sway * ah;
    induced_force.yaw = induced_force.sway * xh * ship_length;

    return induced_force;
}
