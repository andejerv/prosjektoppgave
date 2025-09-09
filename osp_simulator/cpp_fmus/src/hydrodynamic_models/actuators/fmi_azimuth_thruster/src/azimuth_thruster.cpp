#include "hydrodynamic_models/actuators/fmi_azimuth_thruster/include/azimuth_thruster.hpp"

#include <fstream>
#include <ios>
#include <sstream>
#include <string>
#include <vector>

#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

AzimuthThruster::AzimuthThruster(std::unique_ptr<RudderEffect> rudder_effect, std::unique_ptr<PropellerBase> propeller)
: m_rudder_effect(std::move(rudder_effect)), m_propeller(std::move(propeller))
{
}

void AzimuthThruster::set_ship_velocity(const ManeuveringBodyVector& ship_velocity_ms)
{
    m_rudder_effect->set_ship_velocity(ship_velocity_ms);
    m_propeller->set_ship_velocity(ship_velocity_ms);
}

void AzimuthThruster::set_thruster_angle(const double angle_rad)
{
    m_rudder_effect->set_rudder_angle(-angle_rad);  // Sign convention for rudder angles is opposite of thruster angles
    m_propeller->set_propeller_angle(angle_rad);
}

void AzimuthThruster::set_propeller_speed(const double speed_rps)
{
    m_propeller->set_propeller_speed(speed_rps);
}

void AzimuthThruster::set_thruster_placement(const double long_pos, const double lat_pos)
{
    m_rudder_effect->set_rudder_placement(long_pos, lat_pos);
    m_propeller->set_placement(long_pos, lat_pos);
}

double AzimuthThruster::get_propeller_thrust() const
{
    return m_propeller->get_thrust();
}

double AzimuthThruster::get_propeller_shaft_torque() const
{
    return m_propeller->get_torque();
}

double AzimuthThruster::get_propeller_shaft_power() const
{
    return m_propeller->get_power();
}

ManeuveringBodyVector AzimuthThruster::get_body_force() const
{
    return m_body_force;
}

void AzimuthThruster::update_state()
{
    auto rudder_force = m_rudder_effect->get_body_force();

    m_propeller->update_state();
    auto propeller_force = m_propeller->get_body_force();

    m_body_force.surge = rudder_force.surge + propeller_force.surge;
    m_body_force.sway = rudder_force.sway + propeller_force.sway;
    m_body_force.yaw = rudder_force.yaw + propeller_force.yaw;
}

std::vector<double> four_quadrant_coeff_from_string(const std::string& string_coeffs)
{
    std::stringstream ss(string_coeffs);
    std::vector<double> result;

    while (ss.good()) {
        std::string substr;
        getline(ss, substr, ',');
        result.emplace_back(stod(substr));
    }

    return result;
}

std::unique_ptr<FourQuadrantPropellerCharacteristics> load_four_quadrant_model_from_file(const std::string& filename)
{
    // Read four quadrant model from file. Expects each line of the file to be formatted as AT,BT,AQ,BQ.
    std::vector<double> AT, BT, AQ, BQ;
    std::string line;
    std::ifstream file(filename);

    if (file.is_open()) {
        while (getline(file, line)) {
            std::stringstream ss(line);
            std::vector<double> result;

            while (ss.good()) {
                std::string substr;
                getline(ss, substr, ',');
                result.emplace_back(stod(substr));
            }

            AT.emplace_back(result[0]);
            BT.emplace_back(result[1]);
            AQ.emplace_back(result[2]);
            BQ.emplace_back(result[3]);
        }

        file.close();
    }
    else {
        throw std::ios_base::failure("Error opening thruster coefficient file!");
    }

    auto thrust_fourier_series = FourierSeries(AT, BT);
    auto torque_fourier_series = FourierSeries(AQ, BQ);

    return std::make_unique<FourQuadrantPropellerCharacteristics>(thrust_fourier_series, torque_fourier_series);
}