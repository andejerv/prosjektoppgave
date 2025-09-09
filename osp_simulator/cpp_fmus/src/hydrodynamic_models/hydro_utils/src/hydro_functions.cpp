#include "hydrodynamic_models/hydro_utils/include/hydro_functions.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

double cf_ittc(const double Re)
{
    double Re_effective;
    double Re_min = 1000;  // Formualtion only valid for relativly large Reynolds numbers

    if (Re > Re_min) {
        Re_effective = Re;
    }
    else {
        Re_effective = Re_min;
    }

    double denominator = pow(log10(Re_effective) - 2.0, 2.0);

    return 0.075 / denominator;
}

double sigmoid(const double x, const double x0, const double transfer_range)
{
    double slope = 4.6 / transfer_range;  // x = 4.6, corresponds to y = 0.99 for an unscaled sigmoid function

    double x_prime = slope * (x - x0);

    return 1 / (1 + exp(-x_prime));
}

double get_velocity_magnitude(const ManeuveringBodyVector& velocity)
{
    return sqrt(pow(velocity.surge, 2.0) + pow(velocity.sway, 2.0));
}

double get_drift_angle(const ManeuveringBodyVector& velocity)
{
    return atan2(-velocity.sway, velocity.surge);
}

double get_sign(const double x)
{
    double sign{};

    if (x != 0) {
        sign = x / fabs(x);
    }
    else {
        sign = 1.0;
    }

    return sign;
}

Vector cross_product(const Vector& a, const Vector& b)
{
    Vector result;

    result.x = (a.y * b.z - a.z * b.y);
    result.y = -(a.x * b.z - a.z * b.x);
    result.z = (a.x * b.y - a.y * b.x);

    return result;
}
