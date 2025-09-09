/*----------------------------------------------------------------------------*\
Model of an azimuth_thruster with rudder effect
\*----------------------------------------------------------------------------*/

#ifndef azimuth_thruster_H
#define azimuth_thruster_H

#include <string>
#include <vector>

#include "hydrodynamic_models/actuators/include/propeller_base.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"
#include "rudder_effect.hpp"

class AzimuthThruster
{
   public:
    AzimuthThruster(std::unique_ptr<RudderEffect> rudder_effect, std::unique_ptr<PropellerBase> propeller);
    void set_ship_velocity(const ManeuveringBodyVector& ship_velocity_ms);
    void set_thruster_angle(const double angle_rad);
    void set_propeller_speed(const double speed_rps);
    void set_thruster_placement(const double long_pos, double lat_pos);
    double get_propeller_thrust() const;
    double get_propeller_shaft_torque() const;
    double get_propeller_shaft_power() const;
    ManeuveringBodyVector get_body_force() const;
    void update_state();

   private:
    std::unique_ptr<RudderBase> m_rudder_effect;
    std::unique_ptr<PropellerBase> m_propeller;
    ManeuveringBodyVector m_body_force{0.0, 0.0, 0.0};
};

std::vector<double> four_quadrant_coeff_from_string(const std::string& string_coeffs);
std::unique_ptr<FourQuadrantPropellerCharacteristics> load_four_quadrant_model_from_file(const std::string& file);

#endif