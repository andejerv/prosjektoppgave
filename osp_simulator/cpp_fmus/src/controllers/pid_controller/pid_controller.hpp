/*----------------------------------------------------------------------------*\

General PID-loop, with anti-windup.

\*----------------------------------------------------------------------------*/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController
{
   public:
    PIDController(double, double, double, double, double);

    void set_gains(double, double, double);
    void set_set_point(double);
    void set_measurment(double);
    void set_control_limits(double, double);

    void set_pid_on(bool);
    void set_fixed_output(double);
    void set_gain_adjust_factor(double);

    void reset_errors();

    double do_step(double dt, double y_measurment);
    double get_error();

   private:
    double m_k_p;
    double m_k_i;
    double m_k_d;

    double m_u_p;
    double m_u_i;
    double m_u_d;

    double m_u;
    double m_u_fixed;

    double m_y_current;
    double m_y_target;

    double m_u_min;
    double m_u_max;

    double m_e;
    double m_de_dt;
    double m_e_i;
    double m_e_last;

    bool m_pid_on;

    double m_gain_adjust_factor;
};

#endif