#include <gtest/gtest.h>
#include <math.h>

#include "hydrodynamic_models/actuators/fmi_azimuth_thruster/include/azimuth_thruster.hpp"
#include "hydrodynamic_models/actuators/fmi_azimuth_thruster/include/four_quadrant_model_data.hpp"
#include "hydrodynamic_models/hydro_utils/include/hydro_structures.hpp"

class AzimuthThrusterTests : public ::testing::Test
{
   public:
    std::unique_ptr<AzimuthThruster> model;
    const double atol{1.0};

    AzimuthThrusterTests()
    {
        // Create lifting surface model for rudder effect
        auto lifting_surface = std::make_unique<LiftingSurface>();
        lifting_surface->set_aspect_ratio(1.313207547, 0.902830189);
        lifting_surface->set_lift_stall_model(0.6, 1.0);
        lifting_surface->set_drag_pre_stall_model(19.92625501, 0.0);
        lifting_surface->set_drag_post_stall_model(1.0, 1.6);

        // Create rudder effect model
        auto rudder = std::make_unique<RudderEffect>(std::move(lifting_surface));
        HullInteractionParameters hull_interaction_params{};
        hull_interaction_params.ah = 0.22;
        hull_interaction_params.xh = -0.055;
        hull_interaction_params.tr = 0.0;
        hull_interaction_params.wake_factor = 0.0;
        hull_interaction_params.ship_length = 12.0;
        rudder->set_rudder_geometry(0.1153, 0.265);
        rudder->set_rudder_placement(0.0, 0.0);
        rudder->set_hull_interaction_params(hull_interaction_params);

        // Create propeller model
        auto four_quadrant_model = generate_four_quadrant_model(PropellerGeometry::WAGENINGEN_B3_BAR65_PD1);
        auto propeller = std::make_unique<PropellerBase>(std::move(four_quadrant_model));
        propeller->set_propeller_diameter(0.355);
        propeller->set_placement(0.0, 0.0);
        propeller->set_thrust_deduction_factor(0.029);
        propeller->set_wake_factor(0.0);
        propeller->set_water_density(1025.9);

        // Create azimuth thruster model
        model = std::make_unique<AzimuthThruster>(std::move(rudder), std::move(propeller));
    }
};

TEST_F(AzimuthThrusterTests, T01_TestZeroSpeedAndThrust)
{
    model->update_state();
    auto force = model->get_body_force();

    EXPECT_DOUBLE_EQ(model->get_propeller_thrust(), 0.0);
    EXPECT_DOUBLE_EQ(model->get_propeller_shaft_power(), 0.0);
    EXPECT_DOUBLE_EQ(model->get_propeller_shaft_torque(), 0.0);
    EXPECT_DOUBLE_EQ(force.surge, 0.0);
    EXPECT_DOUBLE_EQ(force.sway, 0.0);
    EXPECT_DOUBLE_EQ(force.yaw, 0.0);
}

TEST_F(AzimuthThrusterTests, T02_TestLiftAndDrag)
{
    model->set_ship_velocity({3.0, 0.0, 0.0});
    model->update_state();
    auto force = model->get_body_force();

    EXPECT_LT(force.surge, 0.0);
    EXPECT_DOUBLE_EQ(force.sway, 0.0);
    EXPECT_DOUBLE_EQ(force.yaw, 0.0);

    double angle_deg = 10.0;
    model->set_thruster_angle(angle_deg * M_PI / 180.0);
    model->update_state();
    auto force_10 = model->get_body_force();
    EXPECT_LT(force_10.surge, 0.0);
    EXPECT_GT(force_10.sway, 0.0);

    angle_deg = -10.0;
    model->set_thruster_angle(angle_deg * M_PI / 180.0);
    model->update_state();
    auto force_neg_10 = model->get_body_force();
    EXPECT_DOUBLE_EQ(force_neg_10.surge, force_10.surge);
    EXPECT_DOUBLE_EQ(force_neg_10.sway, -force_10.sway);

    angle_deg = 90;
    model->set_thruster_angle(angle_deg * M_PI / 180.0);
    model->update_state();
    auto force_90 = model->get_body_force();
    EXPECT_LT(force_90.surge, 0.0);
    EXPECT_LT(abs(force_90.sway), atol);

    angle_deg = 180;
    model->set_thruster_angle(angle_deg * M_PI / 180.0);
    model->update_state();
    auto force_180 = model->get_body_force();
    EXPECT_LT(force_180.surge, 0.0);
    EXPECT_LT(abs(force_180.sway), atol);
    EXPECT_LT(abs(force_180.yaw), atol);

    model->set_ship_velocity({-3.0, 0.0, 0.0});
    angle_deg = 180.0;
    model->set_thruster_angle(angle_deg * M_PI / 180.0);
    model->update_state();
    auto force_reverse = model->get_body_force();
    EXPECT_DOUBLE_EQ(force_reverse.surge, -force.surge);
    EXPECT_LT(abs(force_reverse.sway), atol);
    EXPECT_LT(abs(force_reverse.yaw), atol);

    angle_deg = 10;
    model->set_thruster_angle(angle_deg * M_PI / 180.0);
    model->update_state();
    auto force_reverse_10 = model->get_body_force();
    EXPECT_GT(force_reverse_10.surge, 0.0);
    EXPECT_LT(force_reverse_10.sway, 0.0);

    model->set_ship_velocity({0.0, 0.5, 0.0});
    angle_deg = 0.0;
    model->set_thruster_angle(angle_deg * M_PI / 180.0);
    model->update_state();
    auto force_crab = model->get_body_force();
    EXPECT_LT(force_crab.sway, 0.0);

    model->set_ship_velocity({0.0, 0.0, 0.1});
    angle_deg = 0.0;
    model->set_thruster_angle(angle_deg * M_PI / 180.0);
    model->update_state();
    auto force_turn = model->get_body_force();
    EXPECT_LT(abs(force_turn.surge), atol);
    EXPECT_LT(abs(force_turn.sway), atol);
    EXPECT_LT(abs(force_turn.yaw), atol);
}

TEST_F(AzimuthThrusterTests, T03_TestZeroSpeed)
{
    model->set_ship_velocity({0.0, 0.0, 0.0});
    model->set_propeller_speed(20.0);
    double angle_deg = 0.0;
    model->set_thruster_angle(angle_deg * M_PI / 180.0);
    model->update_state();
    auto force = model->get_body_force();
    EXPECT_GT(force.surge, 0.0);
    EXPECT_DOUBLE_EQ(force.sway, 0.0);
    EXPECT_DOUBLE_EQ(force.yaw, 0.0);
    EXPECT_DOUBLE_EQ(model->get_propeller_thrust(), force.surge);
    EXPECT_GT(model->get_propeller_shaft_power(), 0.0);
    EXPECT_GT(model->get_propeller_shaft_torque(), 0.0);

    model->set_propeller_speed(-20.0);
    model->update_state();
    auto force_reverse = model->get_body_force();
    EXPECT_LT(force_reverse.surge, 0.0);
    EXPECT_DOUBLE_EQ(force_reverse.sway, 0.0);
    EXPECT_DOUBLE_EQ(force_reverse.yaw, 0.0);
    EXPECT_DOUBLE_EQ(model->get_propeller_thrust(), force_reverse.surge);
    EXPECT_GT(model->get_propeller_shaft_power(), 0.0);
    EXPECT_LT(model->get_propeller_shaft_torque(), 0.0);

    model->set_propeller_speed(20.0);
    angle_deg = 90.0;
    model->set_thruster_angle(angle_deg * M_PI / 180.0);
    model->update_state();
    auto force_90 = model->get_body_force();
    EXPECT_NEAR(force_90.surge, 0.0, atol);
    EXPECT_DOUBLE_EQ(force_90.sway, model->get_propeller_thrust());

    angle_deg = -90.0;
    model->set_thruster_angle(angle_deg * M_PI / 180.0);
    model->update_state();
    auto force_neg_90 = model->get_body_force();
    EXPECT_NEAR(force_neg_90.surge, 0.0, atol);
    EXPECT_DOUBLE_EQ(force_neg_90.sway, -model->get_propeller_thrust());

    angle_deg = 180;
    model->set_thruster_angle(angle_deg * M_PI / 180.0);
    model->update_state();
    auto force_180 = model->get_body_force();
    EXPECT_DOUBLE_EQ(force_180.surge, -model->get_propeller_thrust());
    EXPECT_NEAR(force_180.sway, 0.0, atol);
}

TEST_F(AzimuthThrusterTests, T04_TestPlacementVariations)
{
    model->set_ship_velocity({0.0, 0.0, 0.0});
    model->set_propeller_speed(20.0);
    double angle_deg = 0.0;
    model->set_thruster_angle(angle_deg * M_PI / 180.0);
    double long_disp = 6.0;
    double lat_disp = 3.0;

    model->set_thruster_placement(long_disp, lat_disp);
    model->update_state();
    auto force_fore_stb = model->get_body_force();

    model->set_thruster_placement(-long_disp, lat_disp);
    model->update_state();
    auto force_aft_stb = model->get_body_force();

    model->set_thruster_placement(long_disp, -lat_disp);
    model->update_state();
    auto force_fore_port = model->get_body_force();

    model->set_thruster_placement(-long_disp, -lat_disp);
    model->update_state();
    auto force_aft_port = model->get_body_force();

    EXPECT_DOUBLE_EQ(force_fore_stb.surge, force_aft_stb.surge);
    EXPECT_DOUBLE_EQ(force_fore_stb.surge, force_aft_port.surge);
    EXPECT_DOUBLE_EQ(force_fore_stb.surge, force_fore_port.surge);

    EXPECT_DOUBLE_EQ(force_fore_stb.sway, 0.0);
    EXPECT_DOUBLE_EQ(force_fore_port.sway, 0.0);
    EXPECT_DOUBLE_EQ(force_aft_stb.sway, 0.0);
    EXPECT_DOUBLE_EQ(force_aft_port.sway, 0.0);

    EXPECT_DOUBLE_EQ(force_fore_stb.yaw, force_aft_stb.yaw);
    EXPECT_DOUBLE_EQ(force_fore_stb.yaw, -force_aft_port.yaw);
    EXPECT_DOUBLE_EQ(force_fore_stb.yaw, -force_fore_port.yaw);

    angle_deg = 90.0;
    model->set_thruster_angle(angle_deg * M_PI / 180.0);
    model->set_thruster_placement(long_disp, lat_disp);
    model->update_state();
    force_fore_stb = model->get_body_force();

    model->set_thruster_placement(-long_disp, lat_disp);
    model->update_state();
    force_aft_stb = model->get_body_force();

    model->set_thruster_placement(long_disp, -lat_disp);
    model->update_state();
    force_fore_port = model->get_body_force();

    model->set_thruster_placement(-long_disp, -lat_disp);
    model->update_state();
    force_aft_port = model->get_body_force();

    EXPECT_NEAR(force_fore_stb.surge, 0.0, atol);
    EXPECT_NEAR(force_fore_port.surge, 0.0, atol);
    EXPECT_NEAR(force_aft_stb.surge, 0.0, atol);
    EXPECT_NEAR(force_aft_port.surge, 0.0, atol);

    EXPECT_DOUBLE_EQ(force_fore_stb.sway, force_aft_stb.sway);
    EXPECT_DOUBLE_EQ(force_fore_stb.sway, force_aft_port.sway);
    EXPECT_DOUBLE_EQ(force_fore_stb.sway, force_fore_port.sway);

    EXPECT_DOUBLE_EQ(force_fore_stb.yaw, force_fore_port.yaw);
    EXPECT_DOUBLE_EQ(force_fore_stb.yaw, -force_aft_port.yaw);
    EXPECT_DOUBLE_EQ(force_fore_stb.yaw, -force_aft_port.yaw);
}
