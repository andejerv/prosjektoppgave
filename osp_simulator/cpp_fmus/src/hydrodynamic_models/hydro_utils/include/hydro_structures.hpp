/*----------------------------------------------------------------------------*\

Collection of various structures that are used in multiple hydrodynamic models

\*----------------------------------------------------------------------------*/

#ifndef HYDRO_STRUCTURES_H
#define HYDRO_STRUCTURES_H

struct Vector
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct ManeuveringBodyVector
{
    double surge = 0.0;
    double sway = 0.0;
    double yaw = 0.0;
};

struct ManeuveringPositionVector
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
};

struct SeakeepingBodyVector
{
    double heave = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
};

static constexpr double divide_by_zero_tol = 1e-6;

#endif