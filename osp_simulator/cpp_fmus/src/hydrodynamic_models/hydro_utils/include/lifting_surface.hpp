/*----------------------------------------------------------------------------*\

Model of a lifting surface using simplified expressions.

Two general models are implemented; one pre-stall model based on lifting line
theory and one post-stall model based on empirical data.

The transfer between pre and post stall is done based on a max lift coefficient
in pre-stall mode and a sigmoid function.

The model uses only the angle of attack (alpha) and the Reynolds number (Re) as
input variables. The output is lift and drag coefficients.

\*----------------------------------------------------------------------------*/

#ifndef LIFTING_SURFACE_H
#define LIFTING_SURFACE_H

#define _USE_MATH_DEFINES
#include <math.h>

class LiftingSurface
{
   public:
    LiftingSurface();

    double get_cl() const;
    double get_cd(const double cl) const;
    double get_normal_force_coefficient() const;

    void set_alpha(const double alpha);
    void set_reynolds_number(const double re);
    void set_aspect_ratio(const double effective_aspect_ratio_lift, const double effective_aspect_ratio_drag);
    void set_lift_stall_model(const double cl_stall_mean, const double cl_post_stall_max);
    void set_drag_post_stall_model(const double cd_post_stall_max, const double cd_post_stall_power);
    void set_drag_pre_stall_model(const double cd_pre_stall_k, const double cd_pre_stall_a2);

   private:
    double m_effective_aspect_ratio_lift{0.0};
    double m_effective_aspect_ratio_drag{0.0};
    double m_alpha{0.0};
    double m_reynolds_number{1e6};
    double m_cl_stall_mean{0.0};
    double m_alpha_stall_mean{0.0};
    double m_alpha_stall_range{4 * M_PI / 180};
    double m_cl_post_stall_max{1.0};
    double m_cd_pre_stall_k{0.0};
    double m_cd_pre_stall_a2{0.1};
    double m_cd_post_stall_max{1.0};
    double m_cd_post_stall_power{1.6};

    double get_alpha_linear() const;

    double get_cl_pre_stall() const;
    double get_cl_post_stall() const;

    double get_cd_pre_stall(double cl) const;
    double get_cd_post_stall() const;

    double merge_pre_and_post_stall(const double pre_stall, const double post_stall) const;
};

#endif