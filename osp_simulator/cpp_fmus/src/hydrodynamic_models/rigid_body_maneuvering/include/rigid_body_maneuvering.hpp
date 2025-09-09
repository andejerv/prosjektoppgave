/*----------------------------------------------------------------------------*\

3DOF rigid body model (surge, sway, yaw) with the same simplifications and
assumptions as the standard MMG model.

Reference:
- H. Yasukawa, et al: Introduction of MMG standard method for ship maneuvering
  predictions

Notation: see reference

\*----------------------------------------------------------------------------*/

#ifndef RIGID_BODY_MANEUVERING_H
#define RIGID_BODY_MANEUVERING_H

#include <Eigen/Dense>

#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

class RigidBodyManeuvering
{
   public:
    RigidBodyManeuvering();
    ManeuveringPositionVector get_position() const;
    ManeuveringBodyVector get_velocity() const;
    ManeuveringBodyVector get_acceleration() const;
    void set_external_force(const ManeuveringBodyVector& force);
    void set_position(const ManeuveringPositionVector& position);
    void set_veloctity(const ManeuveringBodyVector& velocity);
    void do_step(const double dt);

    double mass{0.0};
    double moment_of_inertia{0.0};
    double long_center_of_gravity{0.0};
    ManeuveringBodyVector added_mass{};

   private:
    ManeuveringPositionVector m_position{};
    ManeuveringBodyVector m_acceleration{};
    ManeuveringBodyVector m_velocity{};

    ManeuveringBodyVector m_external_force{};

    Eigen::Matrix3d m_coriolis_matrix{};
    Eigen::Matrix3d m_inertia_matrix{};
    Eigen::Matrix3d m_inverse_inertia_matrix{};

    void compute_coriolis_matrix(const double r);
    void compute_inertia_matrix();
    void compute_acceleration();
};

#endif