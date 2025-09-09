#include "main_skrew_fmu.hpp"

#include <cstring>
#include <memory>
#include <utility>

#include "hydrodynamic_models/actuators/fmi_main_skrew/include/main_skrew.hpp"
#include "hydrodynamic_models/actuators/include/jet_stream_model.hpp"
#include "hydrodynamic_models/actuators/include/open_water_propeller_characteristics.hpp"
#include "hydrodynamic_models/actuators/include/wake_model.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"
#include "hydrodynamic_models/hydro_utils/include/second_order_polynomial_model.hpp"

PropellerFMU::PropellerFMU(const std::string& instance_name) : FMUBaseClass(instance_name)
{
    // parameters
    m_real_signals.insert(std::make_pair(m_density_i, 1025.9));
    m_real_signals.insert(std::make_pair(m_diameter_i, 0.0));
    m_real_signals.insert(std::make_pair(m_longitudinal_position_i, 0.0));

    m_real_signals.insert(std::make_pair(m_wake_factor_i, 0.0));

    m_real_signals.insert(std::make_pair(m_jet_stream_model_kappa_i, 0.0));
    m_real_signals.insert(std::make_pair(m_jet_stream_model_eta_i, 0.0));
    m_real_signals.insert(std::make_pair(m_jet_stream_model_epsilon_i, 0.0));

    m_real_signals.insert(std::make_pair(m_thrust_deduction_factor_i, 0.0));

    m_real_signals.insert(std::make_pair(m_thrust_coefficient_a0_i, 0.0));
    m_real_signals.insert(std::make_pair(m_thrust_coefficient_a1_i, 0.0));
    m_real_signals.insert(std::make_pair(m_thrust_coefficient_a2_i, 0.0));

    m_real_signals.insert(std::make_pair(m_torque_coefficient_a0_i, 0.0));
    m_real_signals.insert(std::make_pair(m_torque_coefficient_a1_i, 0.0));
    m_real_signals.insert(std::make_pair(m_torque_coefficient_a2_i, 0.0));

    // input
    m_real_signals.insert(std::make_pair(m_ship_velocity_surge_i, 0.0));
    m_real_signals.insert(std::make_pair(m_ship_velocity_sway_i, 0.0));
    m_real_signals.insert(std::make_pair(m_ship_velocity_yaw_i, 0.0));

    m_real_signals.insert(std::make_pair(m_rotations_per_second_i, 0.0));

    // ouput
    m_real_signals.insert(std::make_pair(m_thrust_i, 0.0));
    m_real_signals.insert(std::make_pair(m_torque_i, 0.0));
    m_real_signals.insert(std::make_pair(m_power_i, 0.0));

    m_real_signals.insert(std::make_pair(m_surge_velocity_increase_factor_i, 1.0));
}

void PropellerFMU::ExitInitializationMode()
{
    // Create wake model
    auto wake_model = std::make_unique<WakeModel>();
    wake_model->longitudinal_position = m_real_signals.at(m_longitudinal_position_i);
    wake_model->wake_factor = m_real_signals.at(m_wake_factor_i);

    // Create jet stream model
    auto jet_stream_model = std::make_unique<JetStreamModel>();
    jet_stream_model->kappa = m_real_signals.at(m_jet_stream_model_kappa_i);
    jet_stream_model->epsilon = m_real_signals.at(m_jet_stream_model_epsilon_i);
    jet_stream_model->eta = m_real_signals.at(m_jet_stream_model_eta_i);

    // Create propeller characteristics
    auto thrust_coeff = SecondOrderPolynomialModel(m_real_signals.at(m_thrust_coefficient_a0_i),
                                                   m_real_signals.at(m_thrust_coefficient_a1_i),
                                                   m_real_signals.at(m_thrust_coefficient_a2_i));
    auto torque_coeff = SecondOrderPolynomialModel(m_real_signals.at(m_torque_coefficient_a0_i),
                                                   m_real_signals.at(m_torque_coefficient_a1_i),
                                                   m_real_signals.at(m_torque_coefficient_a2_i));

    auto second_order_model =
        std::make_unique<SecondOrderPolynomialPropellerCharacteristics>(thrust_coeff, torque_coeff);

    // Create propeller model
    m_model =
        std::make_unique<MainSkrew>(std::move(second_order_model), std::move(wake_model), std::move(jet_stream_model));
    m_model->set_propeller_diameter(m_real_signals.at(m_diameter_i));
    m_model->set_placement(m_real_signals.at(m_longitudinal_position_i), 0.0);
    m_model->set_thrust_deduction_factor(m_real_signals.at(m_thrust_deduction_factor_i));
    m_model->set_wake_factor(m_real_signals.at(m_wake_factor_i));
    m_model->set_water_density(m_real_signals.at(m_density_i));
}

bool PropellerFMU::DoStep(cppfmu::FMIReal /*t*/, cppfmu::FMIReal /*dt*/, cppfmu::FMIBoolean /*new_step*/,
                          cppfmu::FMIReal& /*end_of_step*/)
{
    // Read inputs
    ManeuveringBodyVector ship_velocity;
    ship_velocity.surge = m_real_signals.at(m_ship_velocity_surge_i);
    ship_velocity.sway = m_real_signals.at(m_ship_velocity_sway_i);
    ship_velocity.yaw = m_real_signals.at(m_ship_velocity_yaw_i);

    double rotations_per_second = m_real_signals.at(m_rotations_per_second_i);

    // Update model
    m_model->set_ship_velocity(ship_velocity);
    m_model->set_propeller_speed(rotations_per_second);
    m_model->update_state();

    // Set outputs
    m_real_signals.at(m_thrust_i) = m_model->get_thrust();
    m_real_signals.at(m_torque_i) = m_model->get_torque();
    m_real_signals.at(m_power_i) = m_model->get_power();
    m_real_signals.at(m_surge_velocity_increase_factor_i) = m_model->get_surge_velocity_increase_factor();

    return true;
}

cppfmu::UniquePtr<cppfmu::SlaveInstance> CppfmuInstantiateSlave(
    cppfmu::FMIString instance_name, cppfmu::FMIString fmu_guid, cppfmu::FMIString /*fmuResourceLocation*/,
    cppfmu::FMIString /*mimeType*/, cppfmu::FMIReal /*timeout*/, cppfmu::FMIBoolean /*visible*/,
    cppfmu::FMIBoolean /*interactive*/, cppfmu::Memory memory, cppfmu::Logger /*logger*/)
{
    if (strcmp(fmu_guid, FMU_UUID) != 0) {
        throw std::runtime_error("FMU GUID mismatch");
    }

    return cppfmu::AllocateUnique<PropellerFMU>(memory, instance_name);
}