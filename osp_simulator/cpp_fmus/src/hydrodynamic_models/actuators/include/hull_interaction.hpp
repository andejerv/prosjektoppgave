/*----------------------------------------------------------------------------*\

Model that computes the interaction between a rudder and a ship hull using
simplfied expressions. Based on the standard MMG maneuvering model.

Includes the following effects:
 - Decrease in surge velocity due the boundary layer of the hull
 - Effect of drift angle on the wake - also known as flow-straightening
 - Induced forces on the hull due to the rudder

\*----------------------------------------------------------------------------*/

#ifndef HULL_INTERACTION_H
#define HULL_INTERACTION_H

#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

class HullInteraction
{
   public:
    ManeuveringBodyVector get_effective_velocity(const ManeuveringBodyVector& hull_velocity) const;
    double get_effective_angle(const double geometric_angle, const ManeuveringBodyVector& effective_velocity) const;
    ManeuveringBodyVector get_induced_hull_force(const ManeuveringBodyVector& direct_force) const;

    double ship_length{0.0};
    double w0{0.0};
    double gamma_positive{1.0};
    double gamma_negative{1.0};
    double lr_surge{-0.5};
    double lr_sway{-0.5};
    double ah{0.0};
    double xh{0.0};
    double tr{0.0};
};

#endif