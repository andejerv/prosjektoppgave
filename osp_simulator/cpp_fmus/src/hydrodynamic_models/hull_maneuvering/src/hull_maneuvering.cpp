#include "hydrodynamic_models/hull_maneuvering/include/hull_maneuvering.hpp"

#include "hydrodynamic_models/hydro_utils/include/hydro_functions.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

#include <stdexcept>

HullManeuvering::HullManeuvering()
{
    density = 1025.9;
    kinematic_viscosity = 1.19e-6;

    length = depth = 0.0;

    shape_factor = CR_m = CR_p = 0.0;

    Xvv = Xvr = Xrr = Xvvvv = 0.0;
    Yv = Yr = Yvvv = Yvvr = Yvrr = Yrrr = 0.0;
    Nv = Nr = Nvvv = Nvvr = Nvrr = Nrrr = 0.0;

    CX_low = 0.45;
    CY_low = 0.70;
    CN_low = 0.15;
    Nr_low = -2200;

    speed_limit_low = 0.5 * 0.5144444444;
    speed_transfer_range = 0.5 * 0.514444444;

    this->set_velocity(0.0, 0.0, 0.0);

    model_mode = ModelMode::COMBINED;
}

void HullManeuvering::set_velocity(const double surge, const double sway, const double yaw)
{
    m_velocity.surge = surge;
    m_velocity.sway = sway;
    m_velocity.yaw = yaw;

    m_velocity_magnitude = get_velocity_magnitude(m_velocity);
    m_drift_angle = get_drift_angle(m_velocity);

    if (m_velocity_magnitude <= 1e-6) {
        m_velocity_non_dimensional.surge = 0.0;
        m_velocity_non_dimensional.sway = 0.0;
        m_velocity_non_dimensional.yaw = 0.0;

        m_dynamic_pressure = 0.0;
    }
    else {
        m_velocity_non_dimensional.surge = m_velocity.surge / m_velocity_magnitude;
        m_velocity_non_dimensional.sway = m_velocity.sway / m_velocity_magnitude;
        m_velocity_non_dimensional.yaw = m_velocity.yaw * length / m_velocity_magnitude;

        m_dynamic_pressure = 0.5 * density * pow(m_velocity_magnitude, 2.0);

        // Avoid large non-dimensional yaw rate at when in low-speed domain
        if (m_velocity_non_dimensional.yaw > m_max_yaw_rate_non_dimensional) {
            m_velocity_non_dimensional.yaw = m_max_yaw_rate_non_dimensional;
        }
        else if (m_velocity_non_dimensional.yaw < -m_max_yaw_rate_non_dimensional) {
            m_velocity_non_dimensional.yaw = -m_max_yaw_rate_non_dimensional;
        }
    }
}

double HullManeuvering::get_straight_ahead_resistance() const
{
    double Re = fabs(m_velocity.surge) * length / kinematic_viscosity;
    double Fr = fabs(m_velocity.surge) / sqrt(length * m_acceleration_of_gravity);

    double CF = cf_ittc(Re);

    double CR = CR_m * pow(Fr, CR_p);

    double CD_lateral_corrected = CD_lateral * length * depth / wetted_surface;

    double R_prime = (1 + shape_factor) * CF + CR + CD_lateral_corrected;

    return R_prime * wetted_surface * m_dynamic_pressure;
}

ManeuveringBodyVector HullManeuvering::get_mmg_force() const
{
    double sign = get_sign(m_velocity.surge);

    double v = m_velocity_non_dimensional.sway * sign;
    double r = m_velocity_non_dimensional.yaw;

    double vvr = pow(v, 2.0) * r;
    double vrr = v * pow(r, 2.0);

    ManeuveringBodyVector result;

    double area = length * depth;

    // --------------- Surge force -----------------------
    double X_sway = Xvv * pow(v, 2.0) + Xvvvv * pow(v, 4.0);
    double X_yaw = Xrr * pow(r, 2.0);
    double X_cc = Xvr * v * r;

    double X_prime = X_sway + X_yaw + X_cc;

    double R0 = this->get_straight_ahead_resistance();

    result.surge = (-R0 + X_prime * area * m_dynamic_pressure) * sign;

    // --------------- Sway force -----------------------
    double Y_sway = Yv * v + Yvvv * pow(v, 3.0);
    double Y_yaw = Yr * r + Yrrr * pow(r, 3.0);

    double Y_cc = Yvvr * vvr + Yvrr * vrr;

    double Y_prime = Y_sway + Y_yaw + Y_cc;

    result.sway = (Y_prime * area * m_dynamic_pressure) * sign;

    // --------------- Yaw moment -----------------------
    double N_sway = Nv * v + Nvvv * pow(v, 3);
    double N_yaw = Nr * r + Nrrr * pow(r, 3);

    double N_cc = Nvvr * vvr + Nvrr * vrr;

    double N_prime = N_sway + N_yaw + N_cc;

    result.yaw = N_prime * area * m_dynamic_pressure * length;

    return result;
}

ManeuveringBodyVector HullManeuvering::get_low_speed_force() const
{
    double area = length * depth;

    double c = cos(m_drift_angle);
    double s = sin(m_drift_angle);
    double s2 = sin(2 * m_drift_angle);

    ManeuveringBodyVector result;

    // --------------- surge force -----------------------
    double X_prime = -CX_low * c * fabs(c);

    result.surge = X_prime * area * m_dynamic_pressure;

    // --------------- sway force -----------------------
    double Y_prime = CY_low * s * fabs(s);

    result.sway = Y_prime * area * m_dynamic_pressure;

    // --------------- yaw moment -----------------------
    double N_drift = CN_low * s2;

    double N_yaw = Nr_low * m_velocity.yaw;

    result.yaw = N_drift * area * m_dynamic_pressure * length + N_yaw;

    return result;
}

double HullManeuvering::low_speed_transfer_function() const
{
    return 1 - sigmoid(m_velocity_magnitude, speed_limit_low, speed_transfer_range);
}

ManeuveringBodyVector HullManeuvering::get_combined_force(const ManeuveringBodyVector& low_speed,
                                                          const ManeuveringBodyVector& high_speed) const
{
    double s_low_speed = this->low_speed_transfer_function();

    ManeuveringBodyVector result;

    result.surge = high_speed.surge * (1 - s_low_speed) + s_low_speed * low_speed.surge;
    result.sway = high_speed.sway * (1 - s_low_speed) + s_low_speed * low_speed.sway;
    result.yaw = high_speed.yaw * (1 - s_low_speed) + s_low_speed * low_speed.yaw;

    return result;
}

ManeuveringBodyVector HullManeuvering::calculate_force() const
{
    ManeuveringBodyVector mmg_force = this->get_mmg_force();
    ManeuveringBodyVector low_speed_force = this->get_low_speed_force();

    switch (model_mode) {
        case ModelMode::LOW_SPEED:
            return low_speed_force;
            break;
        case ModelMode::HIGH_SPEED:
            return mmg_force;
            break;
        case ModelMode::COMBINED:
            return this->get_combined_force(low_speed_force, mmg_force);
            break;
        default:
            throw std::runtime_error("Unhandled ModelMode");
    }
}