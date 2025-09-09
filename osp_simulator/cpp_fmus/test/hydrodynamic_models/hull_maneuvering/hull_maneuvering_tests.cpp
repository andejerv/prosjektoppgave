#include <gtest/gtest.h>
#include <math.h>

#include <memory>

#include "hydrodynamic_models/hull_maneuvering/include/hull_maneuvering.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

class HullManeuveringUnitTests : public ::testing::Test
{
   public:
    std::unique_ptr<HullManeuvering> model;
    HullManeuveringUnitTests()
    {
        model = std::make_unique<HullManeuvering>();
        model->density = 1025.0;
        model->kinematic_viscosity = 1.19e-6;

        // Load standard KVLCC2 model for use in tests
        model->length = 320.0;
        model->depth = 20.8;
        model->wetted_surface = 27194.0;

        model->shape_factor = -1.0;
        model->CR_m = 0.0;
        model->CR_p = 4.0;
        model->CD_lateral = 0.022;

        model->Xvv = -0.040;
        model->Xvr = 0.002;
        model->Xrr = 0.011;
        model->Xvvvv = 0.771;
        model->Yv = -0.315;
        model->Yr = 0.083;
        model->Yvvv = -1.607;
        model->Yvrr = -0.391;
        model->Yvvr = 0.379;
        model->Yrrr = 0.008;
        model->Nv = -0.137;
        model->Nr = -0.049;
        model->Nvvv = -0.030;
        model->Nvrr = 0.055;
        model->Nvvr = -0.294;
        model->Nrrr = -0.013;

        model->CX_low = 0.45;
        model->CY_low = 0.70;
        model->CN_low = 0.15;
        model->Nr_low = -1e8;

        model->speed_limit_low = 2 * 0.5144444444;
        model->speed_transfer_range = 1 * 0.514444444;

        model->model_mode = ModelMode::COMBINED;
    }
    void SetUp() override
    {
    }
};

TEST_F(HullManeuveringUnitTests, T01_ZeroSpeed)
{
    for (int i = 0; i <= 2; i++) {
        model->model_mode = static_cast<ModelMode>(i);
        model->set_velocity(0.0, 0.0, 0.0);
        auto force = model->calculate_force();
        EXPECT_NEAR(force.surge, 0.0, 1e-9);
        EXPECT_NEAR(force.sway, 0.0, 1e-9);
        EXPECT_NEAR(force.yaw, 0.0, 1e-9);
    }
}

TEST_F(HullManeuveringUnitTests, T02_NoSingularityInHeadingMove)
{
    double vel = 1e-5;
    for (int mode = 0; mode <= 2; mode++) {
        model->model_mode = static_cast<ModelMode>(mode);
        model->set_velocity(vel, vel, 0.1);
        auto force = model->calculate_force();
        EXPECT_LT(abs(force.surge), 1e6);
        EXPECT_LT(abs(force.sway), 1e6);
        EXPECT_GT(force.yaw, -1e8);
        EXPECT_LT(force.yaw, 0.0);

        model->set_velocity(vel, vel, -0.1);
        force = model->calculate_force();
        EXPECT_LT(abs(force.surge), 1e6);
        EXPECT_LT(abs(force.sway), 1e6);
        EXPECT_LT(force.yaw, 1e8);
        EXPECT_GT(force.yaw, 0.0);
    }
}

TEST_F(HullManeuveringUnitTests, T03_LowSpeedModelSanity)
{
    model->model_mode = ModelMode::LOW_SPEED;
    ManeuveringBodyVector force;

    // Positive velocity, decoupled
    model->set_velocity(0.1, 0.0, 0.0);
    force = model->calculate_force();
    EXPECT_LT(force.surge, 0.0);

    model->set_velocity(0.0, 0.1, 0.0);
    force = model->calculate_force();
    EXPECT_LT(force.sway, 0.0);

    model->set_velocity(0.0, 0.0, 0.01);
    force = model->calculate_force();
    EXPECT_LT(force.yaw, 0.0);

    // Negative velocity, decoupled
    model->set_velocity(-0.1, 0.0, 0.0);
    force = model->calculate_force();
    EXPECT_GT(force.surge, 0.0);

    model->set_velocity(0.0, -0.1, 0.0);
    force = model->calculate_force();
    EXPECT_GT(force.sway, 0.0);

    model->set_velocity(0.0, 0.0, -0.01);
    force = model->calculate_force();
    EXPECT_GT(force.yaw, 0.0);
}

TEST_F(HullManeuveringUnitTests, T04_HighSpeedModelSanity)
{
    model->model_mode = ModelMode::HIGH_SPEED;
    ManeuveringBodyVector force;

    double speed = 8.0;  // About 16 knots
    double tol = 1e-6;

    // Straight ahead
    model->set_velocity(speed, 0.0, 0.0);
    force = model->calculate_force();
    EXPECT_LT(force.surge, 0.0);
    EXPECT_NEAR(force.sway, 0.0, tol);
    EXPECT_NEAR(force.yaw, 0.0, tol);

    // Lateral symmetry
    model->set_velocity(speed, 0.0, 0.1);
    auto force_stb_turn = model->calculate_force();
    model->set_velocity(speed, 0.0, -0.1);
    auto force_port_turn = model->calculate_force();
    EXPECT_DOUBLE_EQ(force_stb_turn.surge, force_port_turn.surge);
    EXPECT_DOUBLE_EQ(force_stb_turn.sway, -force_port_turn.sway);
    EXPECT_DOUBLE_EQ(force_stb_turn.yaw, -force_port_turn.yaw);

    // Longitudonal symmetry
    model->set_velocity(speed, 0.0, 0.1);
    auto force_forward = model->calculate_force();
    model->set_velocity(-speed, 0.0, 0.1);
    auto force_aftwards = model->calculate_force();
    EXPECT_DOUBLE_EQ(force_forward.surge, -force_aftwards.surge);
    EXPECT_DOUBLE_EQ(force_forward.sway, -force_aftwards.sway);
    EXPECT_DOUBLE_EQ(force_forward.yaw, force_aftwards.yaw);
}

TEST_F(HullManeuveringUnitTests, T05_TestCombinedModelSanity)
{
    model->model_mode = ModelMode::COMBINED;
    ManeuveringBodyVector force;

    // Positive velocity, decoupled
    model->set_velocity(0.1, 0.0, 0.0);
    force = model->calculate_force();
    EXPECT_LT(force.surge, 0.0);

    model->set_velocity(0.0, 0.1, 0.0);
    force = model->calculate_force();
    EXPECT_LT(force.sway, 0.0);

    model->set_velocity(0.0, 0.0, 0.01);
    force = model->calculate_force();
    EXPECT_LT(force.yaw, 0.0);

    // Negative velocity, decoupled
    model->set_velocity(-0.1, 0.0, 0.0);
    force = model->calculate_force();
    EXPECT_GT(force.surge, 0.0);

    model->set_velocity(0.0, -0.1, 0.0);
    force = model->calculate_force();
    EXPECT_GT(force.sway, 0.0);

    model->set_velocity(0.0, 0.0, -0.01);
    force = model->calculate_force();
    EXPECT_GT(force.yaw, 0.0);

    double speed = 8.0;
    double tol = 1e-6;

    // Straight ahead
    model->set_velocity(speed, 0.0, 0.0);
    force = model->calculate_force();
    EXPECT_LT(force.surge, 0.0);
    EXPECT_NEAR(force.sway, 0.0, tol);
    EXPECT_NEAR(force.yaw, 0.0, tol);

    // Lateral symmetry
    model->set_velocity(speed, 0.0, 0.1);
    auto force_stb_turn = model->calculate_force();
    model->set_velocity(speed, 0.0, -0.1);
    auto force_port_turn = model->calculate_force();
    EXPECT_DOUBLE_EQ(force_stb_turn.surge, force_port_turn.surge);
    EXPECT_DOUBLE_EQ(force_stb_turn.sway, -force_port_turn.sway);
    EXPECT_DOUBLE_EQ(force_stb_turn.yaw, -force_port_turn.yaw);

    // Longitudonal symmetry (TODO : Add flag to disable this feature for
    // non-double ended vessels)
    model->set_velocity(speed, 0.0, 0.1);
    auto force_forward = model->calculate_force();
    model->set_velocity(-speed, 0.0, 0.1);
    auto force_aftwards = model->calculate_force();
    EXPECT_DOUBLE_EQ(force_forward.surge, -force_aftwards.surge);
    EXPECT_DOUBLE_EQ(force_forward.sway, -force_aftwards.sway);
    EXPECT_DOUBLE_EQ(force_forward.yaw, force_aftwards.yaw);
}

TEST_F(HullManeuveringUnitTests, T06_TestCombinedModelCoherence)
{
    // Test cohenrence with low-speed model
    model->set_velocity(0.1, 0.1, 0.01);
    model->model_mode = ModelMode::COMBINED;
    auto force_combined = model->calculate_force();
    model->model_mode = ModelMode::LOW_SPEED;
    auto force_low_speed = model->calculate_force();

    double percent_difference_surge = 100 * abs((force_combined.surge - force_low_speed.surge) / force_low_speed.surge);
    double percent_difference_sway = 100 * abs((force_combined.sway - force_low_speed.sway) / force_low_speed.sway);
    double percent_difference_yaw = 100 * abs((force_combined.yaw - force_low_speed.yaw) / force_low_speed.yaw);

    EXPECT_LT(percent_difference_surge, 5.0);
    EXPECT_LT(percent_difference_sway, 5.0);
    EXPECT_LT(percent_difference_yaw, 5.0);

    // Test cohenrence with high-speed model
    model->set_velocity(8.0, 0.1, 0.01);
    model->model_mode = ModelMode::COMBINED;
    force_combined = model->calculate_force();
    model->model_mode = ModelMode::HIGH_SPEED;
    auto force_high_speed = model->calculate_force();

    percent_difference_surge = 100 * abs((force_combined.surge - force_high_speed.surge) / force_high_speed.surge);
    percent_difference_sway = 100 * abs((force_combined.sway - force_high_speed.sway) / force_high_speed.sway);
    percent_difference_yaw = 100 * abs((force_combined.yaw - force_high_speed.yaw) / force_high_speed.yaw);

    EXPECT_LT(percent_difference_surge, 5.0);
    EXPECT_LT(percent_difference_sway, 5.0);
    EXPECT_LT(percent_difference_yaw, 5.0);
}