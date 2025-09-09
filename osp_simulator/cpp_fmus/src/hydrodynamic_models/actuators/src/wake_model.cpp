#include "hydrodynamic_models/actuators/include/wake_model.hpp"

#include "hydrodynamic_models/hydro_utils/include/hydro_functions.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

double WakeModel::get_effective_velocity(const ManeuveringBodyVector& ship_velocity)
{
    double U = get_velocity_magnitude(ship_velocity);
    double beta = get_drift_angle(ship_velocity);

    double beta_p = beta;

    if (U > divide_by_zero_tol) {
        beta_p -= longitudinal_position * ship_velocity.yaw / U;
    }

    double wp = wake_factor * exp(-4 * pow(beta_p, 2));

    return ship_velocity.surge * (1 - wp);
}