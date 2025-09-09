/*----------------------------------------------------------------------------*\

Propeller wake model from the MMG model

Reference:
- H. Yasukawa, et al: Introduction of MMG standard method for ship maneuvering
  predictions

\*----------------------------------------------------------------------------*/

#ifndef WAKE_MODEL_H
#define WAKE_MODEL_H

#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

class WakeModel
{
   public:
    double longitudinal_position{0.0};
    double wake_factor{0.0};

    double get_effective_velocity(const ManeuveringBodyVector& ship_velocity);
};

#endif