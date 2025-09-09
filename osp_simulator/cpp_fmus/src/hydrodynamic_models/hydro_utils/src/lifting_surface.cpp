#include "hydrodynamic_models/hydro_utils/include/lifting_surface.hpp"

#include "hydrodynamic_models/hydro_utils/include/hydro_functions.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

LiftingSurface::LiftingSurface()
{
    this->set_lift_stall_model(2.0, 1.0);
    this->set_alpha(0.0);
}

void LiftingSurface::set_alpha(const double alpha)
{
    double alpha_abs = fabs(alpha);

    while (alpha_abs > M_PI) {
        alpha_abs -= M_PI;
    }

    m_alpha = alpha_abs * get_sign(alpha);
}

void LiftingSurface::set_reynolds_number(const double reynolds_number)
{
    m_reynolds_number = reynolds_number;
}

void LiftingSurface::set_aspect_ratio(const double effective_aspect_ratio_lift,
                                      const double effective_aspect_ratio_drag)
{
    m_effective_aspect_ratio_lift = effective_aspect_ratio_lift;
    m_effective_aspect_ratio_drag = effective_aspect_ratio_drag;

    this->set_lift_stall_model(m_cl_stall_mean, m_cl_post_stall_max);
}

void LiftingSurface::set_lift_stall_model(const double cl_stall_mean, const double cl_post_stall_max)
{
    m_cl_stall_mean = cl_stall_mean;
    m_cl_post_stall_max = cl_post_stall_max;

    if (m_effective_aspect_ratio_lift > divide_by_zero_tol) {
        this->m_alpha_stall_mean = m_cl_stall_mean * (1 + 2 / m_effective_aspect_ratio_lift) / (2 * M_PI);
    }
    else {
        this->m_alpha_stall_mean = m_cl_stall_mean / (2 * M_PI);
    }
}

void LiftingSurface::set_drag_post_stall_model(const double cd_post_stall_max, const double cd_post_stall_power)
{
    m_cd_post_stall_max = cd_post_stall_max;
    m_cd_post_stall_power = cd_post_stall_power;
}

void LiftingSurface::set_drag_pre_stall_model(const double cd_pre_stall_k, const double cd_pre_stall_a2)
{
    m_cd_pre_stall_k = cd_pre_stall_k;
    m_cd_pre_stall_a2 = cd_pre_stall_a2;
}

double LiftingSurface::get_alpha_linear() const
{
    double alpha_abs = fabs(m_alpha);
    double alpha_linear{};

    if (alpha_abs > M_PI / 2.0) {
        alpha_linear = alpha_abs - M_PI;
    }
    else {
        alpha_linear = alpha_abs;
    }

    return alpha_linear;
}

double LiftingSurface::get_cl_pre_stall() const
{
    double alpha_linear = this->get_alpha_linear();

    double cl_2d = 2 * M_PI * alpha_linear;

    double cl;

    if (m_effective_aspect_ratio_lift > divide_by_zero_tol) {
        cl = cl_2d / (1.0 + 2.0 / m_effective_aspect_ratio_lift);
    }
    else {
        cl = cl_2d;
    }

    cl *= get_sign(m_alpha);

    return cl;
}

double LiftingSurface::get_cl_post_stall() const
{
    return m_cl_post_stall_max * sin(2 * m_alpha);
}

double LiftingSurface::get_cl() const
{
    double cl_pre_stall = this->get_cl_pre_stall();
    double cl_post_stall = this->get_cl_post_stall();

    double cl = this->merge_pre_and_post_stall(cl_pre_stall, cl_post_stall);

    return cl;
}

double LiftingSurface::get_cd_pre_stall(const double cl) const
{
    double cf = cf_ittc(m_reynolds_number);

    double alpha_linear = this->get_alpha_linear();

    double cd_v = 2 * (1 + m_cd_pre_stall_k) * cf + m_cd_pre_stall_a2 * pow(alpha_linear, 2.0);

    double cd_i{};

    if (m_effective_aspect_ratio_drag > divide_by_zero_tol) {
        cd_i = pow(cl, 2.0) / (M_PI * m_effective_aspect_ratio_drag);
    }
    else {
        cd_i = 0.0;
    }

    return cd_v + cd_i;
}

double LiftingSurface::get_cd_post_stall() const
{
    return m_cd_post_stall_max * pow(fabs(sin(m_alpha)), m_cd_post_stall_power);
}

double LiftingSurface::get_cd(double cl) const
{
    double cd_pre_stall = this->get_cd_pre_stall(cl);
    double cd_post_stall = this->get_cd_post_stall();

    double cd = this->merge_pre_and_post_stall(cd_pre_stall, cd_post_stall);

    return cd;
}

double LiftingSurface::get_normal_force_coefficient() const
{
    double asp = m_effective_aspect_ratio_lift;

    double fa = 6.13 * asp / (2.25 + asp);

    return fa * sin(m_alpha);
}

double LiftingSurface::merge_pre_and_post_stall(const double pre_stall, const double post_stall) const
{
    double alpha_abs = fabs(m_alpha);

    double alpha_right = M_PI - alpha_abs;

    double alpha_transfer;

    if (alpha_right < alpha_abs) {
        alpha_transfer = alpha_right;
    }
    else {
        alpha_transfer = alpha_abs;
    }

    double s = sigmoid(alpha_transfer, m_alpha_stall_mean, m_alpha_stall_range);

    return pre_stall * (1 - s) + post_stall * s;
}
