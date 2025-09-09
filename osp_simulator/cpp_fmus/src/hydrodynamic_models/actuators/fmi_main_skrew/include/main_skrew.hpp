/*----------------------------------------------------------------------------*\

Propeller model based on the formulations in the standard MMG maneuvering model.

Reference:
- H. Yasukawa, et al: Introduction of MMG standard method for ship maneuvering
  predictions

Includes:
- variation in propeller thrust and torque based on a polynomial model of
force / moment coefficients as a function of advance ratio
- Wake factor and thrust deduction to account for propeller hull interaction
- Calculates the increase in surge veloicty due to the propeller that can be fed
to the rudder model

Simplifications:
- Only one quadrant, corresponding to forward speed and thrust. That is, this
  model is only really applicable for forward speed situations
- Not possible to rotate this propeller (can be added in the future).

\*----------------------------------------------------------------------------*/

#ifndef MAIN_SKREW_H
#define MAIN_SKREW_H

#include <memory>

#include "hydrodynamic_models/actuators/include/jet_stream_model.hpp"
#include "hydrodynamic_models/actuators/include/propeller_base.hpp"
#include "hydrodynamic_models/actuators/include/wake_model.hpp"

class MainSkrew : public PropellerBase
{
   public:
    MainSkrew(std::unique_ptr<IOpenWaterPropellerCharacteristics> open_water_chars,
              std::unique_ptr<WakeModel> wake_model, std::unique_ptr<JetStreamModel> jet_stream_model);

    double get_surge_velocity_increase_factor() const;

   private:
    void calculate_inflow_velocity() override;
    double get_raw_thrust();

    std::unique_ptr<WakeModel> m_wake_model;
    std::unique_ptr<JetStreamModel> m_jet_stream_model;
};

#endif