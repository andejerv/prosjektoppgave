#include "hydrodynamic_models/actuators/fmi_main_skrew/include/main_skrew.hpp"

MainSkrew::MainSkrew(std::unique_ptr<IOpenWaterPropellerCharacteristics> open_water_chars,
                     std::unique_ptr<WakeModel> wake_model, std::unique_ptr<JetStreamModel> jet_stream_model)
: PropellerBase(std::move(open_water_chars))
, m_wake_model(std::move(wake_model))
, m_jet_stream_model(std::move(jet_stream_model)){};

double MainSkrew::get_surge_velocity_increase_factor() const
{
    double raw_thrust = m_thrust / (1 - m_thrust_deduction_factor);

    double thrust_loading_coeffcient = 0.0;

    if (m_inflow_velocity > 0.0) {
        thrust_loading_coeffcient =
            raw_thrust / (0.5 * m_density * M_PI * pow(m_diameter / 2, 2.0) * pow(m_inflow_velocity, 2.0));
    }

    return m_jet_stream_model->get_surge_increase_factor(thrust_loading_coeffcient);
}

void MainSkrew::calculate_inflow_velocity()
{
    m_inflow_velocity = m_wake_model->get_effective_velocity(m_ship_velocity);
}
