/*----------------------------------------------------------------------------*\

Steering machinery model with saturation of rotational speed

\*----------------------------------------------------------------------------*/

#ifndef STEERINGMACHINERY_H
#define STEERINGMACHINERY_H

#include <cmath>
#include <zeabuz/common/utilities/math.hpp>

class SteeringMachinery
{
   public:
    void set_initial_angle(const double angle);
    void set_target_angle(const double angle);
    void set_max_speed(const double max_speed);
    double get_angle() const;
    void do_step(double dt);

   private:
    double m_angle{0};
    double m_max_speed{0};
    double m_target_angle{0};
};

#endif  // end header guard
