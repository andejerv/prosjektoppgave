/*----------------------------------------------------------------------------*\

A rotatable thruster in the simplest form possible.

It provide a constant force in a direction that is dependent on the angle of the
thruster. The magnitude of the force can be controller through a "max_thrust"
parameter and by specifying the loading (the loading will typcially be between
-1 and 1)

This model neglects all complicated effects related to the propeller,
such as variation in max thrust with forward speed and interaction with the hull.
The purpose of this model is to be a placeholder until a more physical correct
model is made.

In situations where the propeller dynamics is not of interest, this model will
probably be a good subsitute.

Another possible use case is to use this model to apply a given thrust, and
calculate the required torque and power to the propeller in an external
post-processing step. This is only possible in cases where the propeller and
thruster is guranteed to achive the necessary thrust (i.e., no limitations on
rotational speed or torque from the engine)

\*----------------------------------------------------------------------------*/

#ifndef SIMPLE_THRUSTER_H
#define SIMPLE_THRUSTER_H

#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

class SimpleThruster
{
   public:
    void set_parameters(const Vector& p0, const Vector& orientation0, const double max_thrust);

    void set_angle(const double angle);
    void set_loading(const double loading);

    ManeuveringBodyVector do_step();

   private:
    // --------------- Model parameters -------------------------------------
    Vector m_p0{};            // location in body fixed coordinate system
    Vector m_orientation0{};  // initial orientation in body fixed coordinate system

    double m_max_thrust{0.0};

    // --------------- Control variables ------------------------------------
    double m_angle{0.0};
    double m_loading{0.0};  // control variables

    // -------------- Internal variables ------------------------------------
    Vector m_orientation{};
};

#endif  // end header guard
