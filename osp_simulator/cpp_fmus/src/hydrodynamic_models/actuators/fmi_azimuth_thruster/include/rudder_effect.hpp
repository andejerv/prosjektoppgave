#ifndef RUDDER_EFFECT_H
#define RUDDER_EFFECT_H

#include <memory>

#include "hydrodynamic_models/actuators/include/rudder_base.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"
#include "hydrodynamic_models/hydro_utils/include/lifting_surface.hpp"

struct HullInteractionParameters
{
    double ah{};
    double xh{};
    double tr{};
    double wake_factor{};
    double ship_length{};
};

class RudderEffect : public RudderBase
{
   public:
    explicit RudderEffect(std::unique_ptr<LiftingSurface> lifting_surface);

    ManeuveringBodyVector get_body_force() override;
    void set_hull_interaction_params(const HullInteractionParameters& params);

   private:
    ManeuveringBodyVector calc_induced_rudder_force(const ManeuveringBodyVector& direct_rudder_force) const;
    ManeuveringBodyVector get_inflow_velocity() const;
    std::unique_ptr<LiftingSurface> m_lifting_surface;
    HullInteractionParameters m_hull_interaction_params{};
};

#endif