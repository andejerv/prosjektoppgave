#ifndef FOURIER_SERIES_H
#define FOURIER_SERIES_H

#include <math.h>

#include <stdexcept>
#include <vector>

class FourierSeries
{
   public:
    FourierSeries(const std::vector<double>& A, const std::vector<double>& B) : m_A(A), m_B(B)
    {
        if (m_A.size() != m_B.size()) {
            throw std::length_error("Length of coefficient vectors must be equal!");
        }
    };

    double calculate(const double beta) const
    {
        double out = 0.0;

        for (unsigned int k = 0; k < m_A.size(); k++) {
            out += m_A[k] * cos(beta * k) + m_B[k] * sin(beta * k);
        }

        return out;
    }

   private:
    std::vector<double> m_A{0}, m_B{0};
};

#endif