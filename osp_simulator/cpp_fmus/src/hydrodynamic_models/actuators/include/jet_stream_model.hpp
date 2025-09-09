/*----------------------------------------------------------------------------*\

Model for the increase in surge velocity due to the propeller from the MMG model

Reference:
- H. Yasukawa, et al: Introduction of MMG standard method for ship maneuvering
  predictions

\*----------------------------------------------------------------------------*/

#ifndef JET_STREAM_MODEL_H
#define JET_STREAM_MODEL_H

class JetStreamModel
{
   public:
    double kappa{0.5};
    double eta{1.0};
    double epsilon{1.0};

    double get_surge_increase_factor(const double thrust_coefficient) const;
};

#endif