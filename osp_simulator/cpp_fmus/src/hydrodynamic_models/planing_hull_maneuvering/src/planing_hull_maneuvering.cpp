#include "hydrodynamic_models/planing_hull_maneuvering/include/planing_hull_maneuvering.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "hydrodynamic_models/hydro_utils/include/hydro_functions.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

PlaningHullManeuvering::PlaningHullManeuvering()
: m_rootfinding_solver_ptr(gsl_multiroot_fsolver_alloc(gsl_multiroot_fsolver_hybrids, 2))
, m_rootfinding_vars_ptr(gsl_vector_alloc(2))
{
    if (m_rootfinding_solver_ptr == NULL || m_rootfinding_vars_ptr == NULL) {
        throw std::bad_alloc{};
    }
}

PlaningHullManeuvering::~PlaningHullManeuvering()
{
    gsl_multiroot_fsolver_free(m_rootfinding_solver_ptr);
    gsl_vector_free(m_rootfinding_vars_ptr);
}

void PlaningHullManeuvering::do_step(const double surge_vel, const double sway_vel, const double yaw_vel)
{
    m_velocity.surge = surge_vel;
    m_velocity.sway = sway_vel;
    m_velocity.yaw = yaw_vel;

    m_velocity_magnitude = get_velocity_magnitude(m_velocity);
    m_dynamic_pressure = 0.5 * density * pow(m_velocity_magnitude, 2.0);

    if (m_velocity_magnitude > model_mixing_start) {
        step_planing_model();
    }
}

void PlaningHullManeuvering::step_planing_model()
{
    int status{};
    size_t iter{0};
    const auto prev_trim = m_trim_angle;
    const auto prev_heave = m_heave;

    gsl_multiroot_function f = {&gsl_root_finding_function, 2, this};

    gsl_vector_set(m_rootfinding_vars_ptr, 0, m_trim_angle);
    gsl_vector_set(m_rootfinding_vars_ptr, 1, m_heave);

    gsl_multiroot_fsolver_set(m_rootfinding_solver_ptr, &f, m_rootfinding_vars_ptr);

    do {
        iter++;
        status = gsl_multiroot_fsolver_iterate(m_rootfinding_solver_ptr);

        if (status != 0) { /* check if solver is stuck */
            break;
        }

        status = gsl_multiroot_test_residual(m_rootfinding_solver_ptr->f, ROOTFINDING_TOL);
    } while (status == GSL_CONTINUE && iter < ROOTFINDING_MAX_ITER);

    if (status != GSL_SUCCESS) {  // Restore previous state if solver failed
        m_trim_angle = prev_trim;
        m_heave = prev_heave;
        calc_planing_geo_lengths();

        std::cerr << "Planing model rootfinding solver did not succeed at finding the equilibrium trim and heave. "
                     "Using previous trim and heave for this time step.\n"
                  << "Solver status: " << status << ".\n"
                  << "Iterations / Max iterations: " << iter << " / " << ROOTFINDING_MAX_ITER << ".\n";
    }
}

void PlaningHullManeuvering::calc_planing_geo_lengths()
{
    // Keel wetted length, Eq. 9.50 of Faltinsen 2005, page 367
    m_L_K = lcg + vcg / std::tan(m_trim_angle) - (m_heave) / std::sin(m_trim_angle);
    if (m_L_K < 0.0) {
        m_L_K = 0.0;
    }

    // Eq. 12 of Savitsky '76
    const double w = (0.57 + deadrise_deg / 1000.0) *
                     (std::tan(M_PI / 180.0 * deadrise_deg) / (2 * std::tan(m_trim_angle)) - deadrise_deg / 167.0);

    const double lambda_K = m_L_K / beam;

    // Eq. 14 of Savitsky '76
    double lambda_C = (lambda_K - w) - 0.2 * std::exp(-(lambda_K - w) / 0.3);
    m_L_C = lambda_C * beam;

    m_x_s = m_L_K - m_L_C;
    m_alpha =
        std::atan(beam / (2 * m_x_s)) * 180.0 / M_PI;  // Angle between spray line and keel(projected to plan view)

    if (lambda_C < 0) {
        lambda_C = 0.0;
        m_L_C = 0.0;
        m_x_s = 0.0;
    }

    // Mean wetted length-to-beam ratio, Eq. 15 of Savitsky '76
    m_lambda_W = (lambda_K + lambda_C) / 2.0 + 0.03;

    // Chines-dry planing condition (Eq. 3 of Savitsky '76)
    const double Fn_B = m_velocity_magnitude / std::sqrt(acceleration_of_gravity * beam);  // Beam Froude number
    const double chines_dry =
        std::pow(Fn_B, 2) - (m_lambda_W - 0.16 * std::tan(deadrise_deg * M_PI / 180.0) / std::tan(m_trim_angle)) /
                                (3 * std::sin(m_trim_angle));
    if (chines_dry >= 0) {
        m_L_C2 = 0.0;
    }
    else {
        m_L_C2 = m_L_C - 3 * std::pow(m_velocity_magnitude, 2) * std::sin(m_trim_angle) /
                             acceleration_of_gravity;  //#Side wetting length (Eq. 1 of Savitsky '76)
    }
}

PlaningBodyForce PlaningHullManeuvering::get_planing_model_forces() const
{
    //-------Hydrodynamic force---------//

    // Beam Froude number
    const double Fn_B = m_velocity_magnitude / std::sqrt(acceleration_of_gravity * beam);

    // 0-Deadrise lift coefficient
    const auto C_L0 = std::pow(m_trim_angle * 180 / M_PI, 1.1) *
                      (0.012 * std::sqrt(m_lambda_W) + 0.0055 * std::pow(m_lambda_W, 2.5) / std::pow(Fn_B, 2));

    // Lift coefficient with deadrise, C_Lbeta
    const auto C_Lbeta = C_L0 - 0.0065 * deadrise_deg * std::pow(C_L0, 0.6);

    // Vertical force(lift)
    const auto F_z_hydro = C_Lbeta * 0.5 * density * std::pow(m_velocity_magnitude * beam, 2);

    // Horizontal force
    const auto F_x_hydro = F_z_hydro * std::tan(m_trim_angle);

    // Lift's Normal force w.r.t. keel
    const auto F_N_hydro = F_z_hydro / std::cos(m_trim_angle);

    // Longitudinal position of the center of pressure, l_p(Eq.4.41, Doctors 1985)
    const auto l_p = m_lambda_W * beam * (0.75 - 1 / (5.21 * std::pow(Fn_B / m_lambda_W, 2) + 2.39));

    // Moment about CG(Axis consistent with Fig.9.24 of Faltinsen(P.366)
    const auto M_cg_hydro = -F_N_hydro * (lcg - l_p);

    //------------Skin friction------------//

    double F_x_skin{}, F_z_skin{}, M_cg_skin{};
    // Surface area of the non-wetted-chine region
    const auto S1 = std::pow(m_x_s, 2) * std::tan(m_alpha * M_PI / 180) / std::cos(deadrise_deg * M_PI / 180);

    // Surface area of the wetted-chine region
    const auto S2 = beam * m_L_C / std::cos(deadrise_deg * M_PI / 180);

    // Total surface area
    const auto S = S1 + S2;
    if (S <= 0) {
        F_x_skin = 0.0;
        F_z_skin = 0.0;
        M_cg_skin = 0.0;
    }
    else {
        // Mean bottom fluid velocity, Savitsky 1964 - derived to include deadrise effect
        const auto V_m =
            m_velocity_magnitude *
            std::sqrt(
                1 - (0.012 * std::pow((m_trim_angle * 180.0 / M_PI), 1.1) * std::sqrt(m_lambda_W) -
                     0.0065 * deadrise_deg *
                         std::pow((0.012 * std::sqrt(m_lambda_W) * std::pow(m_trim_angle * 180.0 / M_PI, 1.1)), 0.6)) /
                        (m_lambda_W * std::cos(m_trim_angle)));

        // Reynolds number(with bottom fluid velocity)
        const auto Re = V_m * m_lambda_W * beam / kinematic_viscosity;

        // 'Friction coefficient' ITTC 1957
        const auto C_f = cf_ittc(Re);

        // Additional 'friction coefficient' due to skin friction, Townsin(1985) roughness allowance
        constexpr double AHR = 150e-6;
        const auto deltaC_f =
            (44 * (std::pow(AHR / (m_lambda_W * beam), 1.0 / 3) - 10 * std::pow(Re, (-1.0 / 3))) + 0.125) / 1000;

        // Frictional force
        const auto R_f = 0.5 * density * (C_f + deltaC_f) * S * std::pow(m_velocity_magnitude, 2);

        // Geometric vertical distance from keel
        const auto l_f = (beam / 4 * std::tan(deadrise_deg * M_PI / 180.0) * S2 +
                          beam / 6 * std::tan(deadrise_deg * M_PI / 180.0) * S1) /
                         (S1 + S2);

        // Horizontal force
        F_x_skin = R_f * std::cos(m_trim_angle);

        // Vertical force
        F_z_skin = -R_f * std::sin(m_trim_angle);

        // Moment about CG
        M_cg_skin = R_f * (l_f - vcg);
    }

    //----------- Hull force sum --------------//
    const auto F_x = F_x_hydro + F_x_skin;
    const auto F_z = F_z_hydro + F_z_skin - acceleration_of_gravity * mass;
    const auto M_cg = M_cg_hydro + M_cg_skin;

    // Required thrust and resultant forces
    const auto thrust = F_x / std::cos(m_trim_angle);                     // Magnitude
    const auto thrust_z = thrust * std::sin(m_trim_angle);                // Vertical
    const auto thrust_cg = thrust * (vcg - vertical_thruster_placement);  // Moment about cg

    // Update resultant thurst values
    return PlaningBodyForce{F_x, F_z + thrust_z, M_cg + thrust_cg};
}

double PlaningHullManeuvering::get_planing_resistance() const
{
    const auto forces = get_planing_model_forces();
    return forces.surge;
}

double PlaningHullManeuvering::get_displacement_resistance() const
{
    double Re = fabs(m_velocity.surge) * length / kinematic_viscosity;
    double Fr = fabs(m_velocity.surge) / sqrt(length * acceleration_of_gravity);

    double CF = cf_ittc(Re);

    double CR = CR_m * pow(Fr, CR_p);

    double CD_lateral_corrected = CD_lateral * length * depth / wetted_surface;

    double R_prime = (1 + shape_factor) * CF + CR + CD_lateral_corrected;

    return R_prime * wetted_surface * m_dynamic_pressure;
}

ManeuveringBodyVector PlaningHullManeuvering::get_maneuvering_force() const
{
    ManeuveringBodyVector result{};

    // ---------------- Resistance model mixing -------------
    double resistance{};
    if (m_velocity_magnitude <= model_mixing_start) {
        resistance = get_displacement_resistance();
    }
    else if (m_velocity_magnitude >= model_mixing_end) {
        resistance = get_planing_resistance();
    }
    else {
        const auto planing_fraction =
            (m_velocity_magnitude - model_mixing_start) / (model_mixing_end - model_mixing_start);
        resistance =
            get_planing_resistance() * planing_fraction + get_displacement_resistance() * (1 - planing_fraction);
    }

    // --------------- Surge force -----------------------
    const auto sign = get_sign(m_velocity.surge);
    result.surge = -sign * resistance;

    // --------------- Sway force -----------------------
    result.sway = -sway_linear_damping * m_velocity.sway;

    // --------------- Yaw moment -----------------------

    result.yaw = -yaw_linear_damping * m_velocity.yaw;

    return result;
}

int gsl_root_finding_function(const gsl_vector* x, void* params, gsl_vector* f)
{
    auto planing_model_ptr = (PlaningHullManeuvering*)params;

    const double trim = gsl_vector_get(x, 0);
    const double heave = gsl_vector_get(x, 1);

    planing_model_ptr->set_trim_angle(trim);
    planing_model_ptr->set_heave(heave);

    planing_model_ptr->calc_planing_geo_lengths();
    const auto forces = planing_model_ptr->get_planing_model_forces();

    gsl_vector_set(f, 0, forces.heave);
    gsl_vector_set(f, 1, forces.pitch);

    return GSL_SUCCESS;
}