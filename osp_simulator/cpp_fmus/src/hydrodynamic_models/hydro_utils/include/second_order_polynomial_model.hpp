#ifndef SECOND_ORDER_POLYNOMIAL_MODEL_H
#define SECOND_ORDER_POLYNOMIAL_MODEL_H

#include <math.h>
class SecondOrderPolynomialModel
{
   public:
    SecondOrderPolynomialModel(const double a0, const double a1, const double a2) : m_a0(a0), m_a1(a1), m_a2(a2){};
    double calculate(const double x) const
    {
        return m_a0 + m_a1 * x + m_a2 * pow(x, 2.0);
    }

   private:
    double m_a0, m_a1, m_a2 = 0;
};

#endif