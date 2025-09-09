/*----------------------------------------------------------------------------*\
3DOF model of a rudder. Can also be used to model the "body" of a thuster.

The model includes:
- Lift and drag on the rudder as a function of an effective velocity at the
  rudder and the angle of attack.
- Interaction from the hull on the rudder through a wake model
- Interaction from the rudder on the hull through a model for induced forces

The interaction with the hull is based on the standard MMG model (see reference).

The model for the lift and drag can be calculated with two different methods:

The first method is the same as used in most maneuvering models, called the
"normal force" model. This is useful for comparing results from this model
against other maneuvering models, and can also achive accurate results is the
hull-interaction model is tuned appropriatly.

However, based on the second reference in the list below, there are situations
where the maneuvering model approximation is too simple or too dependend on
proper hull-interaction model. An alternative liftting line model is therefore
also avialble. This model calculates the lift and drag with the classical lifting
line equations based on an effective aspect ratio.

The lift and drag on the rudder with the lifting line model is calculated with
a simplified model that is intended to also give reasonable values after the
rudder has stalled. The post-stall behaviour will be highly geometry dependent,
but a simplfied model is often approximate as the rudder should not operate
above stall during normal conditions. In short, after stall, the rudder will
loose most of the lift and get a large increase in drag.

The angle of attack where the rudder will stall with the lifting line model is
dependent on the effective aspect ratio and a user controlled parameter for the
lift-coefficient at stall. The transfer between a pre-stall and post-stall model
is done through a sigmoid function.

References:
- Original MMG rudder model: H. Yasukawa, et al: Introduction of MMG standard
  method for ship maneuvering predictions
- Modifications to the rudder drag: J. V. Kramer, et al: Simplified test
  program for hydrodynamic CFD simulations of wind-powered cargo ships
\*----------------------------------------------------------------------------*/

#ifndef MMG_RUDDER_H
#define MMG_RUDDER_H

#include <memory>

#include "hydrodynamic_models/actuators/include/hull_interaction.hpp"
#include "hydrodynamic_models/actuators/include/rudder_base.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"
#include "hydrodynamic_models/hydro_utils/include/lifting_surface.hpp"

enum class ModelMode { NORMAL_FORCE, LIFTING_LINE };

class MMGRudder : public RudderBase
{
   public:
    MMGRudder(std::unique_ptr<LiftingSurface> lifting_surface, std::unique_ptr<HullInteraction> hull_interaction);

    void set_surge_velocity_increase_factor(const double surge_velocity_increse_factor)
    {
        m_surge_velocity_increase_factor = surge_velocity_increse_factor;
    };

    void set_model_mode(const ModelMode& model_mode)
    {
        m_model_mode = model_mode;
    };

    ManeuveringBodyVector get_body_force() override;

   private:
    std::unique_ptr<LiftingSurface> m_lifting_surface;
    std::unique_ptr<HullInteraction> m_hull_interaction;
    double m_surge_velocity_increase_factor{1.0};
    ModelMode m_model_mode{ModelMode::NORMAL_FORCE};
};

#endif