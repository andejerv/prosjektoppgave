#include <gsl/gsl_multiroots.h>
#include <gsl/gsl_vector.h>
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"
#include "hydrodynamic_models/planing_hull_maneuvering/include/planing_hull_maneuvering.hpp"

class PlaningHullManeuveringUnitTests : public ::testing::Test
{
   public:
    std::unique_ptr<PlaningHullManeuvering> model;
    PlaningHullManeuveringUnitTests()
    {
        model = std::make_unique<PlaningHullManeuvering>();

        // Elly model
        model->length = 8.0;
        model->beam = 3.169;
        model->depth = 0.7;
        model->mass = 7200;
        model->wetted_surface = 22.0;

        model->shape_factor = 0.11530219489019286;
        model->CR_m = 0.3052385248245691;
        model->CR_p = 4.123079171388858;
        model->CD_lateral = 0.1;

        model->model_mixing_start = 7 * 0.5144444444;
        model->model_mixing_end = 9 * 0.514444444;
        model->deadrise_deg = 23.7;
        model->lcg = 2.725;
        model->vcg = 1.054;
        model->vertical_thruster_placement = 1.054;

        model->sway_linear_damping = 1487.52;
        model->yaw_linear_damping = 16512.0;
    }
};

/*
Test zero speed gives zero force
*/
TEST_F(PlaningHullManeuveringUnitTests, ZeroSpeed)
{
    model->do_step(0.0, 0.0, 0.0);
    const auto force = model->get_maneuvering_force();
    EXPECT_NEAR(force.surge, 0.0, 1e-9);
    EXPECT_NEAR(force.sway, 0.0, 1e-9);
    EXPECT_NEAR(force.yaw, 0.0, 1e-9);
}

/*
Compare planing model resistance to results from vessel_model_tuning_lib
*/
TEST_F(PlaningHullManeuveringUnitTests, PlaningModelResistance)
{
    model->do_step(5.0, 0.0, 0.0);
    auto force = model->get_maneuvering_force();
    EXPECT_NEAR(force.surge, -13918.99, 1.0);
    EXPECT_NEAR(force.sway, 0.0, 0.0);
    EXPECT_NEAR(force.yaw, 0.0, 0.0);

    model->do_step(10.0, 0.0, 0.0);
    force = model->get_maneuvering_force();
    EXPECT_NEAR(force.surge, -12845.23, 1.0);
    EXPECT_NEAR(force.sway, 0.0, 0.0);
    EXPECT_NEAR(force.yaw, 0.0, 0.0);

    model->do_step(15.0, 0.0, 0.0);
    force = model->get_maneuvering_force();
    EXPECT_NEAR(force.surge, -11083.81, 1.0);
    EXPECT_NEAR(force.sway, 0.0, 0.0);
    EXPECT_NEAR(force.yaw, 0.0, 0.0);
}

/*
Compare displacement model resistance to results from vessel_model_tuning_lib
*/
TEST_F(PlaningHullManeuveringUnitTests, DisplacementModelResistance)
{
    model->do_step(2.0, 0.0, 0.0);
    auto force = model->get_maneuvering_force();
    EXPECT_NEAR(force.surge, -1322.3753, 1.0);
    EXPECT_NEAR(force.sway, 0.0, 0.0);
    EXPECT_NEAR(force.yaw, 0.0, 0.0);

    model->do_step(3.0, 0.0, 0.0);
    force = model->get_maneuvering_force();
    EXPECT_NEAR(force.surge, -3244.2787, 1.0);
    EXPECT_NEAR(force.sway, 0.0, 0.0);
    EXPECT_NEAR(force.yaw, 0.0, 0.0);
}

/*
Test model mixing at the boundaries and inside the mixing region
*/
TEST_F(PlaningHullManeuveringUnitTests, MixedModelResistance)
{
    model->do_step(model->model_mixing_start, 0.0, 0.0);
    auto force = model->get_maneuvering_force();
    EXPECT_NEAR(force.surge, -5239.994582171676, 1.0);
    EXPECT_NEAR(force.sway, 0.0, 0.0);
    EXPECT_NEAR(force.yaw, 0.0, 0.0);

    model->do_step(0.5 * (model->model_mixing_end + model->model_mixing_start), 0.0, 0.0);
    force = model->get_maneuvering_force();
    EXPECT_NEAR(force.surge, -10088.004257668283, 1.0);
    EXPECT_NEAR(force.sway, 0.0, 0.0);
    EXPECT_NEAR(force.yaw, 0.0, 0.0);

    model->do_step(model->model_mixing_end, 0.0, 0.0);
    force = model->get_maneuvering_force();
    EXPECT_NEAR(force.surge, -13285.60776235408, 1.0);
    EXPECT_NEAR(force.sway, 0.0, 0.0);
    EXPECT_NEAR(force.yaw, 0.0, 0.0);
}

/*
Test correct sign of the resistance for both displacement and planing regimes with forwards ant aftwards speed
*/
TEST_F(PlaningHullManeuveringUnitTests, ResistanceSign)
{
    model->do_step(2.0, 0.0, 0.0);
    auto force = model->get_maneuvering_force();
    EXPECT_LT(force.surge, 0);

    model->do_step(10.0, 0.0, 0.0);
    force = model->get_maneuvering_force();
    EXPECT_LT(force.surge, 0);

    model->do_step(-2.0, 0.0, 0.0);
    force = model->get_maneuvering_force();
    EXPECT_GT(force.surge, 0);

    model->do_step(-10.0, 0.0, 0.0);
    force = model->get_maneuvering_force();
    EXPECT_GT(force.surge, 0);
}

/*
Test correct sign of maneuvering forces in sway and yaw
*/
TEST_F(PlaningHullManeuveringUnitTests, ManeuveringModelSign)
{
    model->do_step(10, 1.0, 0.0);
    auto force = model->get_maneuvering_force();
    EXPECT_LT(force.sway, 0);

    model->do_step(10, -1.0, 0.0);
    force = model->get_maneuvering_force();
    EXPECT_GT(force.sway, 0);

    model->do_step(10, 0.0, 1.0);
    force = model->get_maneuvering_force();
    EXPECT_LT(force.yaw, 0);

    model->do_step(10, 0.0, -1.0);
    force = model->get_maneuvering_force();
    EXPECT_GT(force.yaw, 0);
}