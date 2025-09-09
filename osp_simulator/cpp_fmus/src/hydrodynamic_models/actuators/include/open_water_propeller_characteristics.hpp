#ifndef OPEN_WATER_PROPELLER_CHARACTERISTICS_H
#define OPEN_WATER_PROPELLER_CHARACTERISTICS_H

#define _USE_MATH_DEFINES
#include <cmath>

#include "hydrodynamic_models/hydro_utils/include/fourier_series.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_functions.hpp"
#include "hydrodynamic_models/hydro_utils/include/second_order_polynomial_model.hpp"

class IOpenWaterPropellerCharacteristics
{
   public:
    IOpenWaterPropellerCharacteristics() = default;
    virtual ~IOpenWaterPropellerCharacteristics() = default;
    IOpenWaterPropellerCharacteristics(const IOpenWaterPropellerCharacteristics&) = delete;
    IOpenWaterPropellerCharacteristics(IOpenWaterPropellerCharacteristics&&) = delete;
    IOpenWaterPropellerCharacteristics& operator=(const IOpenWaterPropellerCharacteristics&) = delete;
    IOpenWaterPropellerCharacteristics& operator=(IOpenWaterPropellerCharacteristics&&) = delete;

    virtual double get_thrust_coefficient(double advance_ratio, double propeller_speed_rps) const = 0;
    virtual double get_torque_coefficient(double advance_ratio, double propeller_speed_rps) const = 0;
};

class FourQuadrantPropellerCharacteristics : public IOpenWaterPropellerCharacteristics
{
   public:
    FourQuadrantPropellerCharacteristics(FourierSeries thrust_coeffs, FourierSeries torque_coeffs)
    : m_thrust_coeffs(thrust_coeffs), m_torque_coeffs(torque_coeffs)
    {
    }
    double get_thrust_coefficient(double advance_ratio, double propeller_speed_rps) const override
    {
        double beta = atan2(advance_ratio, 0.7 * M_PI * get_sign(propeller_speed_rps));
        double c_t = m_thrust_coeffs.calculate(beta);
        return c_t * (M_PI / 8.0) * (pow(advance_ratio, 2) + pow(0.7 * M_PI, 2));
    }

    double get_torque_coefficient(double advance_ratio, double propeller_speed_rps) const override
    {
        double beta = atan2(advance_ratio, 0.7 * M_PI * get_sign(propeller_speed_rps));
        double C_Q = m_torque_coeffs.calculate(beta);
        return C_Q * (M_PI / 8.0) * (pow(advance_ratio, 2) + pow(0.7 * M_PI, 2));
    }

   private:
    FourierSeries m_thrust_coeffs;
    FourierSeries m_torque_coeffs;
};

class SecondOrderPolynomialPropellerCharacteristics : public IOpenWaterPropellerCharacteristics
{
   public:
    SecondOrderPolynomialPropellerCharacteristics(SecondOrderPolynomialModel thrust_coeffs,
                                                  SecondOrderPolynomialModel torque_coeffs)
    : m_thrust_coeffs(thrust_coeffs), m_torque_coeffs(torque_coeffs){};
    double get_thrust_coefficient(double advance_ratio, double propeller_speed_rps) const override
    {
        return m_thrust_coeffs.calculate(advance_ratio) * get_sign(propeller_speed_rps);
    }

    double get_torque_coefficient(double advance_ratio, double propeller_speed_rps) const override
    {
        return m_torque_coeffs.calculate(advance_ratio) * get_sign(propeller_speed_rps);
    }

   private:
    SecondOrderPolynomialModel m_thrust_coeffs;
    SecondOrderPolynomialModel m_torque_coeffs;
};

#endif