#ifndef PROPELLER_BASE_H
#define PROPELLER_BASE_H

#include <memory>

#include "hydrodynamic_models/actuators/include/open_water_propeller_characteristics.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

class PropellerBase
{
   public:
    explicit PropellerBase(std::unique_ptr<IOpenWaterPropellerCharacteristics> open_water_chars);
    virtual ~PropellerBase() = default;
    PropellerBase(const PropellerBase& from) = delete;
    PropellerBase& operator=(const PropellerBase& from) = delete;
    PropellerBase(PropellerBase&&) = default;
    PropellerBase& operator=(PropellerBase&&) = default;

    virtual void update_state();

    void set_water_density(const double density)
    {
        m_density = density;
    }

    void set_propeller_diameter(const double diameter)
    {
        m_diameter = diameter;
    }

    void set_thrust_deduction_factor(const double factor)
    {
        m_thrust_deduction_factor = factor;
    }

    void set_wake_factor(const double factor)
    {
        m_wake_factor = factor;
    }

    void set_ship_velocity(const ManeuveringBodyVector& ship_velocity)
    {
        m_ship_velocity = ship_velocity;
    }

    void set_propeller_speed(const double rotations_per_second)
    {
        m_rotations_per_second = rotations_per_second;
    }

    void set_propeller_angle(const double angle)
    {
        m_angle = angle;
    }

    void set_placement(const double long_pos, const double lat_pos)
    {
        m_longitudinal_pos = long_pos;
        m_lateral_pos = lat_pos;
    }

    double get_thrust() const
    {
        return m_thrust;
    }
    double get_torque() const
    {
        return m_torque;
    }
    double get_power() const
    {
        return m_power;
    }
    ManeuveringBodyVector get_body_force() const
    {
        return m_body_force;
    }

   protected:
    virtual void calculate_inflow_velocity();
    void calculate_advance_ratio();
    void calculate_body_force();

    std::unique_ptr<IOpenWaterPropellerCharacteristics> m_open_water_chars;
    ManeuveringBodyVector m_ship_velocity{0.0, 0.0, 0.0};
    ManeuveringBodyVector m_body_force{0.0, 0.0, 0.0};
    double m_rotations_per_second{0.0};
    double m_inflow_velocity{0.0};
    double m_advance_ratio{0.0};
    double m_angle{0.0};
    double m_density{1025.9};
    double m_diameter{0.0};
    double m_longitudinal_pos, m_lateral_pos = 0.0;
    double m_thrust_deduction_factor, m_wake_factor = 0.0;
    double m_thrust, m_torque, m_power = 0.0;
};

#endif