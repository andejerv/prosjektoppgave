#include "hydrodynamic_models/actuators/include/jet_stream_model.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

double JetStreamModel::get_surge_increase_factor(const double thrust_coefficient) const
{
    double f1 = (1 + kappa * (sqrt(1 + thrust_coefficient) - 1));

    double surge_increase_factor = epsilon * sqrt(eta * pow(f1, 2.0) + (1 - eta));

    return surge_increase_factor;
}