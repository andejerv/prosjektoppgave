/*----------------------------------------------------------------------------*\

3DOF seakeeping model (heave, roll, pitch) based on a simple harmonic oscillator.
Restoring forces from hydrostatics and linear damping forces.


Input:
dispancement in heave, roll, pitch
velocity in heave, roll, pitch

Output:
Force and moment in heave, roll, pitch

\*----------------------------------------------------------------------------*/

#ifndef HULL_SEAKEEPING_H
#define HULL_SEAKEEPING_H

#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

class HullSeakeeping
{
   public:
    HullSeakeeping();

    SeakeepingBodyVector get_force() const;
    void set_velocity(const double heave, const double roll, const double pitch);
    void set_displacement(const double heave, const double roll, const double pitch);
    void calc_restoring_force_coeffients();
    void calc_force();

    // ------------------------ Parameters -------------------------------------
    double density{1025.9};
    double gravity{9.81};
    double mass, waterplane_area;
    double heave_damping, roll_damping, pitch_damping;  // Linear damping coefficients
    double GM_roll, GM_pitch;                           // Metacentric heights

   private:
    // ------------------------ Output -----------------------------------------
    SeakeepingBodyVector m_force{};

    // --------------------------- Input ---------------------------------------
    SeakeepingBodyVector m_velocity{};
    SeakeepingBodyVector m_displacement{};

    double m_g_heave{0.0}, m_g_roll{0.0}, m_g_pitch{0.0};  // Restoring force coefficients
};

#endif  // end header guard
