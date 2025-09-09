/*----------------------------------------------------------------------------*\

3DOF Model (surge, sway, yaw) for hydrodynamic forces acting on hull shapes.
The implementaiton is a composite of three different models:

- A calm water model for the straight ahead resistance based on "classical"
  ship hydrodynamics
- The MMG maneuvering model, intended to be used for "high speed" manuvering.
- A cross-flow model for "low-speed" manuvering

The straight ahead resistance is caled as follows:
- Frictional resistance is scaled based on empirical friction coefficient
  with the Reynolds number (Re) as input and a constant shape factor
- Wave resistance is calculated from a polynomial model with the Froude number
  as input
- Optional additonal drag calculated from a constant drag coefficient and the
  underwater lateral area (length multipled with depth)

Forces from the manuvering model is scaled based on dynamic pressure and
underwater lateral area

The transfer between "high-speed" and "low-speed" is done with a sigmoid
function that depends on forward speed, to capture Re-effects, AND drift angle
(whatever transfer criteria that occurs first)

The model can be set to only use one model, i.e. high-speed or low-speed, or a
combination.

References:
- H. Yasukawa, et al: Introduction of MMG standard method for ship maneuvering
  predictions
- Thor I. Fossen: Handbook of Marine Craft Hydrodynamics and Motion Control
- Sverre Steen: Lecture notes for TMR4220 Ship hydrodynamics

Notation:
Follows the sname nottaion and coordinate system for the forces
x = positive forward
y = positive starboard
z = positive downwards

Input:
velocity in surge, sway and yaw

Output:
Force and moment in surge sway and yaw

\*----------------------------------------------------------------------------*/

#ifndef HULL_MANEUVERING_H
#define HULL_MANEUVERING_H

#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

enum class ModelMode { LOW_SPEED, HIGH_SPEED, COMBINED };

class HullManeuvering
{
   public:
    HullManeuvering();
    void set_velocity(const double surge, const double sway, const double yaw);
    ManeuveringBodyVector calculate_force() const;

    // ------------------------ Parameters -------------------------------------
    double density, kinematic_viscosity;
    double length, depth, wetted_surface;

    double shape_factor;
    double CR_m, CR_p;  // Wave resistance polynomial coefficients
    double CD_lateral;  // Optional way of representing resistance

    double Xvv, Xvr, Xrr, Xvvvv;            // Surge coefficients
    double Yv, Yr, Yvvv, Yvvr, Yvrr, Yrrr;  // Sway coeffcients
    double Nv, Nr, Nvvv, Nvvr, Nvrr, Nrrr;  // Yaw moment coefficients

    double CX_low, CY_low, CN_low, Nr_low;  // Low speed coefficients

    double speed_limit_low;       // When to swich between high and low speed models
    double speed_transfer_range;  // How fast to switch between the models

    ModelMode model_mode;

   private:
    double get_straight_ahead_resistance() const;
    ManeuveringBodyVector get_mmg_force() const;
    ManeuveringBodyVector get_low_speed_force() const;

    double low_speed_transfer_function() const;
    ManeuveringBodyVector get_combined_force(const ManeuveringBodyVector& low_speed,
                                             const ManeuveringBodyVector& high_speed) const;

    // --------------------------- Input ---------------------------------------
    ManeuveringBodyVector m_velocity{};

    // -------------------- Internal variables ---------------------------------
    ManeuveringBodyVector m_velocity_non_dimensional{};

    double m_velocity_magnitude{0.0};
    double m_drift_angle{0.0};
    double m_dynamic_pressure{0.0};
    const double m_acceleration_of_gravity{9.81};
    const double m_max_yaw_rate_non_dimensional{10};
};

#endif  // end header guard
