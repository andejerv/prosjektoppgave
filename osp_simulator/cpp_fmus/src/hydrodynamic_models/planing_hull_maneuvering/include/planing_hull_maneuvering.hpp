/*----------------------------------------------------------------------------*\
Maneuvering model of a vessel with resistance models for both displacement and planing speeds, and simple linear
dynamics for sway and yaw maneuvering forces.

The resistance model implements model mixing based on the speed. If the speed is lower than `model_mixing_start`, the
displacement model is used, if the speed is greater than `model_mixing_end`, the planing model is used, and between
`model_mixing_start` and `model_mixing_end` the models are linearly interpolated.

The displacement model resistance model is based on the Holtrop method with a curve fitted wave resistance. This model
is stateless: Given speed as input, the resistance is explicitly calculated.

The planing resistance model is based on Savitsky 1964/1976. The states of this model are the trim angle and
heave position of the hull. For a given speed, trim and heave, the surge force (resistance), trim moment, and heave
force can be calculated explicitly. For each time step, a multidimensional rootfinding solver finds the equilibrium
state by adapting the trim angle and heave position until the hydrodynamic trim moment and heave force are in
equilibrium with the forces/moments from gravity and thrust.

The maneuvering force/moment in sway and yaw are based on a decoupled linear model, where the damping force/moment is
proportional to the velocity.


Notation:
Follows the sname notation and coordinate system for the forces
x = positive forward
y = positive starboard
z = positive downwards

Input:
velocity in surge, sway and yaw

Output:
Force and moment in surge sway and yaw

\*----------------------------------------------------------------------------*/

#ifndef PLANING_HULL_MANEUVERING_H
#define PLANING_HULL_MANEUVERING_H

#include <gsl/gsl_multiroots.h>
#include <gsl/gsl_vector.h>

#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"
struct PlaningBodyForce
{
    double surge{};
    double heave{};
    double pitch{};
};

class PlaningHullManeuvering
{
   public:
    PlaningHullManeuvering();
    ~PlaningHullManeuvering();
    void do_step(const double surge_vel, const double sway_vel, const double yaw_vel);
    void calc_planing_geo_lengths();
    ManeuveringBodyVector get_maneuvering_force() const;
    PlaningBodyForce get_planing_model_forces() const;

    void set_trim_angle(const double trim_angle)
    {
        m_trim_angle = trim_angle;
    }
    void set_heave(const double heave)
    {
        m_heave = heave;
    }

    // ------------------------ Parameters -------------------------------------
    double density{1025.87}, kinematic_viscosity{1.19e-6}, acceleration_of_gravity{9.8066};
    double length{}, beam{}, depth{}, mass{}, wetted_surface{};

    // Displacement model
    double shape_factor{};
    double CR_m{}, CR_p{};  // Low-speed wave resistance polynomial coefficients
    double CD_lateral{};    // Additional resistance due to viscous effects such as submerged transom stern

    // Planing model
    double lcg{}, vcg{};  // Longitudonal and vertical center of gravity measured from the aft perpendicular / baseline.
    double deadrise_deg{};                 // Deadrise angle
    double vertical_thruster_placement{};  // Vertical position of the thrust relative to the baseline.

    // Model mixing
    double model_mixing_start{3.0};  // Lower speed limit for model mixing (only displacement model below)
    double model_mixing_end{4.0};    // Upper speed limit for model mixing (only planing model above)

    // Lateral model
    double sway_linear_damping{};
    double yaw_linear_damping{};

   private:
    double get_displacement_resistance() const;
    double get_planing_resistance() const;
    void step_planing_model();

    // --------------------------- Input ---------------------------------------
    ManeuveringBodyVector m_velocity{};

    // -------------------- Internal variables ---------------------------------
    ManeuveringBodyVector m_velocity_non_dimensional{};
    double m_velocity_magnitude{};
    double m_dynamic_pressure{};
    double m_trim_angle{3.0 * M_PI / 180.0};
    double m_heave{0.0};

    // Wetted lengths
    double m_L_K{}, m_L_C{}, m_L_C2{}, m_lambda_W{}, m_x_s{}, m_alpha{};

    // Rootfinding solver variables
    gsl_multiroot_fsolver* m_rootfinding_solver_ptr{};
    gsl_vector* m_rootfinding_vars_ptr{};
    static constexpr double ROOTFINDING_TOL{1e-6};
    static constexpr size_t ROOTFINDING_MAX_ITER{100};
};

int gsl_root_finding_function(const gsl_vector* x, void* params, gsl_vector* f);

#endif  // end header guard
