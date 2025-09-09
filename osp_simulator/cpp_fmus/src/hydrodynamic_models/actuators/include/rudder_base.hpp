#ifndef RUDDER_BASE_H
#define RUDDER_BASE_H

#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

class RudderBase
{
   public:
    RudderBase() = default;
    virtual ~RudderBase() = default;
    RudderBase(const RudderBase& from) = delete;
    RudderBase& operator=(const RudderBase& from) = delete;
    RudderBase(RudderBase&&) = default;
    RudderBase& operator=(RudderBase&&) = default;

    void set_ship_velocity(const ManeuveringBodyVector& ship_velocity);
    void set_rudder_angle(const double angle);
    void set_rudder_geometry(const double area, const double chord);
    void set_rudder_placement(const double longitudinal_pos, const double lateral_pos);
    void set_fluid_properies(const double density, const double kinematic_viscosity);

    virtual ManeuveringBodyVector get_body_force() = 0;

   protected:
    ManeuveringBodyVector get_body_force_from_lift_and_drag(const double lift, const double drag);
    double get_inflow_angle() const;

    ManeuveringBodyVector m_ship_velocity{};
    ManeuveringBodyVector m_inflow_velocity{};
    double m_rudder_angle{0.0};
    double m_area{0.0};
    double m_chord{0.0};
    double m_longitudinal_position{0.0};
    double m_lateral_position{0.0};
    double m_density{1025.9};
    double m_kinematic_viscosity{1.19e-6};

    static constexpr Vector m_ROTATION_DIR{0.0, 0.0, 1.0};
};

#endif