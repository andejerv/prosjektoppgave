/*----------------------------------------------------------------------------*\
Data for four quadrant propeller models represented as fourier series of the
torque and thrust coefficients.

The propeller data have been gathered from the publication "Neural Network
Predictions of the 4-Quadrant Wageningen Propeller Series", which contains data
for many other propellers.
\*----------------------------------------------------------------------------*/

#ifndef four_quadrant_model_data_H
#define four_quadrant_model_data_H

#include <memory>

#include "hydrodynamic_models/actuators/include/open_water_propeller_characteristics.hpp"

// Naming convention:
// "<propeller_series>_B<number_of_blades>_BAR<%blade_area_ratio>_PD<pitch_diameter_ratio>"
enum class PropellerGeometry {
    WAGENINGEN_B3_BAR65_PD1,
    WAGENINGEN_B4_BAR55_PD1,
    WAGENINGEN_B4_BAR70_PD08,
    WAGENINGEN_B4_BAR70_PD1
};

std::unique_ptr<FourQuadrantPropellerCharacteristics> generate_four_quadrant_model(
    const PropellerGeometry& propeller_geometry, const double pitch_scaling_factor = 1.0);

#endif