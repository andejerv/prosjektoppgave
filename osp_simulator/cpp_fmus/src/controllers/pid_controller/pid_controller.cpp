#include "pid_controller.hpp"

PIDController::PIDController(double k_p, double k_i, double k_d, double u_min, double u_max)
{
    this->reset_errors();

    this->set_gains(k_p, k_i, k_d);
    this->set_set_point(0.0);
    this->set_measurment(0.0);

    this->set_control_limits(u_min, u_max);

    this->set_pid_on(true);
    this->set_fixed_output(0.0);
    this->set_gain_adjust_factor(1.0);
}

void PIDController::reset_errors()
{
    m_e = 0.0;
    m_de_dt = 0.0;
    m_e_i = 0.0;
    m_e_last = 0.0;
}

void PIDController::set_gains(double k_p, double k_i, double k_d)
{
    m_k_p = k_p;
    m_k_i = k_i;
    m_k_d = k_d;
}

void PIDController::set_set_point(double y_target)
{
    m_y_target = y_target;
}

void PIDController::set_measurment(double y_current)
{
    m_y_current = y_current;

    m_e = m_y_target - m_y_current;
}

void PIDController::set_control_limits(double u_min, double u_max)
{
    m_u_min = u_min;
    m_u_max = u_max;
}

void PIDController::set_fixed_output(double u_fixed)
{
    m_u_fixed = m_u;
}

void PIDController::set_pid_on(bool on)
{
    m_pid_on = on;
}

void PIDController::set_gain_adjust_factor(double gain_adjust_factor)
{
    m_gain_adjust_factor = gain_adjust_factor;
}

double PIDController::do_step(double dt, double y_measurment)
{
    this->set_measurment(y_measurment);

    if (m_pid_on) {
        m_de_dt = (m_e - m_e_last) / dt;

        m_u_p = m_gain_adjust_factor * m_k_p * m_e;
        m_u_i = m_gain_adjust_factor * m_k_i * m_e_i;
        m_u_d = m_gain_adjust_factor * m_k_d * m_de_dt;

        m_u = m_u_p + m_u_i + m_u_d;

        if (m_u > m_u_max) {
            m_u = m_u_max;
        }
        else if (m_u < m_u_min) {
            m_u = m_u_min;
        }
        else {
            m_e_i += m_e * dt;
        }
    }
    else {
        m_u = m_u_fixed;
    }
    m_e_last = m_e;
    return m_u;
}

double PIDController::get_error()
{
    return m_e;
}