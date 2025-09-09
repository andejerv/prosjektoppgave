/*----------------------------------------------------------------------------*\

Collection of various functions that are used in multiple hydrodynamic models

\*----------------------------------------------------------------------------*/

#ifndef HYDRO_FUNCTIONS_H
#define HYDRO_FUNCTIONS_H

#include "hydro_structures.hpp"

double cf_ittc(const double Re);

double sigmoid(const double x, const double x0, const double transfer_range);

double get_velocity_magnitude(const ManeuveringBodyVector& velocity);
double get_drift_angle(const ManeuveringBodyVector& velocity);

double get_sign(const double x);

Vector cross_product(const Vector& a, const Vector& b);

#endif