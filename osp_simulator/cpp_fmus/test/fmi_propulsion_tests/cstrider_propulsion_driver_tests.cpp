#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>

#include "fmi_propulsion_drivers/cstrider/cstrider_spec_utilities.hpp"
#include "zeabuz/common/utilities/math.hpp"

TEST(CStriderPropulsionTests, T01_angle_conversion)
{
    auto angle_deg = 900;
    auto angle_rad = cstrider_deg_to_wrapped_rad(angle_deg);
    auto angle_deg_fb = wrapped_rad_to_cstrider_deg(angle_rad);
    EXPECT_EQ(angle_rad, M_PI / 2.0);
    EXPECT_EQ(angle_deg_fb, angle_deg);

    angle_deg = 1800;
    angle_rad = cstrider_deg_to_wrapped_rad(angle_deg);
    angle_deg_fb = wrapped_rad_to_cstrider_deg(angle_rad);
    EXPECT_EQ(angle_rad, M_PI);
    EXPECT_EQ(angle_deg_fb, angle_deg);

    angle_deg = -1800;
    angle_rad = cstrider_deg_to_wrapped_rad(angle_deg);
    angle_deg_fb = wrapped_rad_to_cstrider_deg(angle_rad);
    EXPECT_EQ(angle_rad, M_PI);
    EXPECT_EQ(angle_deg_fb, 1800);

    angle_deg = 3600;
    angle_rad = cstrider_deg_to_wrapped_rad(angle_deg);
    angle_deg_fb = wrapped_rad_to_cstrider_deg(angle_rad);
    EXPECT_EQ(angle_rad, 0.0);
    EXPECT_EQ(angle_deg_fb, 0.0);

    angle_deg = 3590;
    angle_rad = cstrider_deg_to_wrapped_rad(angle_deg);
    angle_deg_fb = wrapped_rad_to_cstrider_deg(angle_rad);
    EXPECT_NEAR(angle_rad, zeabuz::common::utilities::math::to_rad(-1.0), 1e-6);
    EXPECT_EQ(angle_deg_fb, angle_deg);
}

TEST(CStriderPropulsionTests, T02_zeabuz_system_heartbeat_sequence)
{
    // The autonomy system stops sending data to propulsion_system-P and/or propulsion_system-S
    // 1. A Watchdog in propulsion_system-P and/or propulsion_system-S detect that the autonomy system stops sending
    // data (Heartbeat)
    // 2. “Ready for autonomy” flag is set to False (if it was True)
    // 3. The VMS HMI should alarm the operator
    // 4. RPM/angle commands is read from the levers (which are in the neutral position

    auto currentCommunicationPoint = 1.0;
    auto heartbeat_timeout = std::chrono::milliseconds(1000);
    auto inputs = ModelInputs{};
    // Autonomy heartbeat is now 5, and was 3 0.5 seconds ago meaning heartbeat counter was updated within heartbeat
    // timeout window -> expect heartbeat sequence to allow ready_for_autonmy and button_available, and counter to be 5
    inputs.modbus_tcp_server.zeabuz_system_heartbeat = 5;
    auto result =
        zeabuz_system_heartbeat_sequence(inputs, ZeabuzSystemHeartbeatState{currentCommunicationPoint - 0.5, 3},
                                         currentCommunicationPoint, heartbeat_timeout);
    EXPECT_TRUE(result.ready_for_zeabuz_system_allowed);
    EXPECT_EQ(result.heartbeat_state.heartbeat_counter, 5);

    currentCommunicationPoint = 123.5;
    inputs.modbus_tcp_server.zeabuz_system_heartbeat = 56;
    heartbeat_timeout = std::chrono::milliseconds(1000);
    // Autonomy heartbeat is now 56, and was 55 0.5 seconds ago meaning heartbeat counter was updated within heartbeat
    // timeout window -> expect heartbeat sequence to allow ready_for_autonmy and button_available, and counter to be 56
    result = zeabuz_system_heartbeat_sequence(inputs, ZeabuzSystemHeartbeatState{currentCommunicationPoint - 0.5, 55},
                                              currentCommunicationPoint, heartbeat_timeout);
    EXPECT_TRUE(result.ready_for_zeabuz_system_allowed);
    EXPECT_EQ(result.heartbeat_state.heartbeat_counter, 56);

    currentCommunicationPoint = 75.0;
    heartbeat_timeout = std::chrono::milliseconds(1000);
    inputs.modbus_tcp_server.zeabuz_system_heartbeat = 6;
    // Autonomy heartbeat is now 6, and was 6 2.5 seconds ago meaning heartbeat counter was not updated within
    // heartbeat timeout window -> expect heartbeat sequence to not allow ready_for_autonmy and button_available, and
    // counter to be 6
    result = zeabuz_system_heartbeat_sequence(inputs, ZeabuzSystemHeartbeatState{currentCommunicationPoint - 2.5, 6},
                                              currentCommunicationPoint, heartbeat_timeout);
    EXPECT_FALSE(result.ready_for_zeabuz_system_allowed);
    EXPECT_EQ(result.heartbeat_state.heartbeat_counter, 6);
}

TEST(CStriderPropulsionTests, T03_test_fmu_outputs_zeabuz_system_available_false_neutral_cmds_sent)
{
    auto inputs = ModelInputs{};
    // Autonomy system is not available, expect that no commands are sent to the propulsion and steering
    inputs.modbus_tcp_server.zeabuz_system_available = false;
    inputs.modbus_tcp_server.stb_percent_thrust_cmd = 900;
    inputs.modbus_tcp_server.port_percent_thrust_cmd = 900;
    inputs.modbus_tcp_server.stb_angle_cmd = 1800;

    const auto currentCommunicationPoint = 1.0;
    auto zeabuz_system_heartbeat_sequence_result = zeabuz_system_heartbeat_sequence(
        inputs, ZeabuzSystemHeartbeatState{0.5, 3}, currentCommunicationPoint, std::chrono::milliseconds(1000));
    ASSERT_TRUE(zeabuz_system_heartbeat_sequence_result.ready_for_zeabuz_system_allowed);

    auto outputs =
        get_model_output_signals(inputs, zeabuz_system_heartbeat_sequence_result, {currentCommunicationPoint, 0});
    EXPECT_FALSE(outputs.fmu.ready_for_zeabuz_system);
    EXPECT_EQ(outputs.fmu.stb_rpm_cmd, 0.0);
    EXPECT_EQ(outputs.fmu.port_rpm_cmd, 0.0);
    EXPECT_EQ(outputs.fmu.stb_angle_cmd, 0.0);
    EXPECT_EQ(outputs.fmu.port_angle_cmd, 0.0);
}

TEST(CStriderPropulsionTests, T07_test_fmu_outputs_zeabuz_system_heartbeat_missing_neutral_cmds_sent)
{
    auto inputs = ModelInputs{};
    // Autonomy system heartbeat is not received, expect that no commands are sent to the propulsion and steering
    // 3.9 seconds has passed and heartbeat counter is still the same as input, expect that no commands are sent
    inputs.modbus_tcp_server.zeabuz_system_available = true;
    inputs.modbus_tcp_server.stb_percent_thrust_cmd = 900;
    inputs.modbus_tcp_server.port_percent_thrust_cmd = 900;
    inputs.modbus_tcp_server.stb_angle_cmd = 1800;
    inputs.modbus_tcp_server.zeabuz_system_heartbeat = 3;
    inputs.fmu.ready_for_zeabuz_system = true;

    // Autonomy heartbeat is now 3, and was 3 3.9 seconds ago meaning heartbeat counter was not updated within
    // heartbeat timeout window -> expect heartbeat sequence to not allow ready_for_autonmy and button_available, and
    // counter to be 3
    const auto currentCommunicationPoint = 5.0;
    auto zeabuz_system_heartbeat_sequence_result = zeabuz_system_heartbeat_sequence(
        inputs, ZeabuzSystemHeartbeatState{1.1, 3}, currentCommunicationPoint, std::chrono::milliseconds(1000));
    ASSERT_FALSE(zeabuz_system_heartbeat_sequence_result.ready_for_zeabuz_system_allowed);

    auto outputs =
        get_model_output_signals(inputs, zeabuz_system_heartbeat_sequence_result, {currentCommunicationPoint, 0});
    EXPECT_FALSE(outputs.fmu.ready_for_zeabuz_system);
    EXPECT_EQ(outputs.fmu.stb_rpm_cmd, 0.0);
    EXPECT_EQ(outputs.fmu.port_rpm_cmd, 0.0);
    EXPECT_EQ(outputs.fmu.stb_angle_cmd, 0.0);
    EXPECT_EQ(outputs.fmu.port_angle_cmd, 0.0);
    EXPECT_EQ(outputs.fmu.zeabuz_system_heartbeat, 3);
}

TEST(CStriderPropulsionTests,
     T08_test_fmu_outputs_propulsion_system_ready_for_zeabuz_system_condition_false_neutral_cmds_sent)
{
    auto inputs = ModelInputs{};
    // Autonomy system heartbeat is not received, expect that no commands are sent to the propulsion and steering
    // 3.9 seconds has passed and heartbeat counter is still the same as input, expect that no commands are sent
    inputs.modbus_tcp_server.zeabuz_system_available = true;
    inputs.modbus_tcp_server.stb_percent_thrust_cmd = 900;
    inputs.modbus_tcp_server.port_percent_thrust_cmd = 900;
    inputs.modbus_tcp_server.stb_angle_cmd = 1800;
    inputs.modbus_tcp_server.zeabuz_system_heartbeat = 3;

    const auto currentCommunicationPoint = 4.0;
    auto zeabuz_system_heartbeat_sequence_result = zeabuz_system_heartbeat_sequence(
        inputs, ZeabuzSystemHeartbeatState{3.1, 3}, currentCommunicationPoint, std::chrono::milliseconds(1000));
    ASSERT_TRUE(zeabuz_system_heartbeat_sequence_result.ready_for_zeabuz_system_allowed);

    auto outputs =
        get_model_output_signals(inputs, zeabuz_system_heartbeat_sequence_result, {currentCommunicationPoint, 0});
    EXPECT_FALSE(outputs.fmu.ready_for_zeabuz_system);
    EXPECT_EQ(outputs.fmu.stb_rpm_cmd, 0.0);
    EXPECT_EQ(outputs.fmu.port_rpm_cmd, 0.0);
    EXPECT_EQ(outputs.fmu.stb_angle_cmd, 0.0);
    EXPECT_EQ(outputs.fmu.port_angle_cmd, 0.0);
}

TEST(CStriderPropulsionTests, T09_test_that_all_fmu_output_signals_struct_elements_are_set_correctly)
{
    auto inputs = ModelInputs{};
    // Autonomy available = true, ready for autonomy button available = true, ready for autonomy button = true,
    // not enough time has passed for heartbeat to be missing, expect that all elements in output_signals are set
    inputs.modbus_tcp_server.zeabuz_system_heartbeat = 3;
    inputs.modbus_tcp_server.zeabuz_system_available = true;
    inputs.modbus_tcp_server.stb_percent_thrust_cmd = 1000;
    inputs.modbus_tcp_server.port_percent_thrust_cmd = 1000;
    inputs.modbus_tcp_server.stb_angle_cmd = 1800;
    inputs.fmu.stb_angle_rad = M_PI / 2.0;
    inputs.fmu.port_angle_rad = M_PI / 2.0;
    inputs.fmu.stb_propeller_rpm = 1700.0;
    inputs.fmu.port_propeller_rpm = 1700.0;
    inputs.fmu.stb_propulsion_ready = true;
    inputs.fmu.port_propulsion_ready = true;
    inputs.fmu.stb_steering_ready = true;
    inputs.fmu.ready_for_zeabuz_system = true;

    const auto currentCommunicationPoint = 2.0;
    auto zeabuz_system_heartbeat_sequence_result = zeabuz_system_heartbeat_sequence(
        inputs, ZeabuzSystemHeartbeatState{1.1, 3}, currentCommunicationPoint, std::chrono::milliseconds(1000));
    ASSERT_TRUE(zeabuz_system_heartbeat_sequence_result.ready_for_zeabuz_system_allowed);

    auto output_signals =
        get_model_output_signals(inputs, zeabuz_system_heartbeat_sequence_result, {currentCommunicationPoint, 9});
    // Assert that all elements in output_signals are set
    EXPECT_TRUE(output_signals.fmu.zeabuz_system_available);
    EXPECT_TRUE(output_signals.fmu.ready_for_zeabuz_system);
    EXPECT_DOUBLE_EQ(output_signals.fmu.stb_rpm_cmd, 1700.0);
    EXPECT_DOUBLE_EQ(output_signals.fmu.port_rpm_cmd, 1700.0);
    EXPECT_DOUBLE_EQ(output_signals.fmu.stb_angle_cmd, M_PI);
    EXPECT_DOUBLE_EQ(output_signals.fmu.port_angle_cmd, 0.0);
    EXPECT_EQ(output_signals.fmu.zeabuz_system_heartbeat, 3);
    EXPECT_EQ(output_signals.fmu.propulsion_system_heartbeat, 9);

    EXPECT_EQ(output_signals.modbus_tcp_server.ready_for_zeabuz_system, true);
    EXPECT_EQ(output_signals.modbus_tcp_server.stb_thrust_percent_fb, 1000);
    EXPECT_EQ(output_signals.modbus_tcp_server.port_thrust_percent_fb, 1000);
    EXPECT_EQ(output_signals.modbus_tcp_server.stb_angle_fb, 900);
    EXPECT_EQ(output_signals.modbus_tcp_server.stb_steering_ready, 1);
    EXPECT_EQ(output_signals.modbus_tcp_server.stb_propulsion_ready, 1);
    EXPECT_EQ(output_signals.modbus_tcp_server.port_propulsion_ready, 1);
}

TEST(CStriderPropulsionTests, T09_zeabuz_system_available_false_results_ready_for_zeabuz_system_false_and_neutral_cmds)
{
    auto inputs = ModelInputs{};
    // Autonomy available = true, ready for autonomy button available = true, ready for autonomy button = true,
    // not enough time has passed for heartbeat to be missing, expect that all elements in output_signals are set
    inputs.modbus_tcp_server.zeabuz_system_heartbeat = 3;
    inputs.modbus_tcp_server.zeabuz_system_available = false;
    inputs.modbus_tcp_server.stb_percent_thrust_cmd = 1000;
    inputs.modbus_tcp_server.port_percent_thrust_cmd = 1000;
    inputs.modbus_tcp_server.stb_angle_cmd = 1800;
    inputs.fmu.stb_angle_rad = M_PI / 2.0;
    inputs.fmu.port_angle_rad = M_PI / 2.0;
    inputs.fmu.stb_propeller_rpm = 1700.0;
    inputs.fmu.port_propeller_rpm = 1700.0;
    inputs.fmu.stb_propulsion_ready = true;
    inputs.fmu.port_propulsion_ready = true;
    inputs.fmu.stb_steering_ready = true;
    inputs.fmu.ready_for_zeabuz_system = true;

    const auto currentCommunicationPoint = 2.0;
    auto zeabuz_system_heartbeat_sequence_result = zeabuz_system_heartbeat_sequence(
        inputs, ZeabuzSystemHeartbeatState{1.1, 3}, currentCommunicationPoint, std::chrono::milliseconds(1000));
    ASSERT_TRUE(zeabuz_system_heartbeat_sequence_result.ready_for_zeabuz_system_allowed);

    auto output_signals =
        get_model_output_signals(inputs, zeabuz_system_heartbeat_sequence_result, {currentCommunicationPoint, 9});
    // Assert that all elements in output_signals are set
    EXPECT_FALSE(output_signals.fmu.zeabuz_system_available);
    EXPECT_FALSE(output_signals.fmu.ready_for_zeabuz_system);
    EXPECT_DOUBLE_EQ(output_signals.fmu.stb_rpm_cmd, 0.0);
    EXPECT_DOUBLE_EQ(output_signals.fmu.port_rpm_cmd, 0.0);
    EXPECT_DOUBLE_EQ(output_signals.fmu.stb_angle_cmd, 0.0);
    EXPECT_DOUBLE_EQ(output_signals.fmu.port_angle_cmd, 0.0);
    EXPECT_EQ(output_signals.fmu.zeabuz_system_heartbeat, 3);
    EXPECT_EQ(output_signals.fmu.propulsion_system_heartbeat, 9);

    EXPECT_EQ(output_signals.modbus_tcp_server.ready_for_zeabuz_system, false);
    EXPECT_EQ(output_signals.modbus_tcp_server.stb_thrust_percent_fb, 1000);
    EXPECT_EQ(output_signals.modbus_tcp_server.port_thrust_percent_fb, 1000);
    EXPECT_EQ(output_signals.modbus_tcp_server.stb_angle_fb, 900);
    EXPECT_EQ(output_signals.modbus_tcp_server.stb_steering_ready, 1);
    EXPECT_EQ(output_signals.modbus_tcp_server.stb_propulsion_ready, 1);
    EXPECT_EQ(output_signals.modbus_tcp_server.port_propulsion_ready, 1);
}

TEST(CStriderPropulsionTests, T10_test_propulsion_system_heartbeat_wrap_around)
{
    auto propulsion_system_heartbeat = PropulsionSystemHeartbeatState{0, 0};

    for (int t = 0; t <= 100; t++) {
        propulsion_system_heartbeat =
            update_propulsion_system_heartbeat(propulsion_system_heartbeat, t, std::chrono::seconds(1), 60, false);
    }
    // Expect that heartbeat has wrapped around from 60 -> 1, and is at 40 after 100 seconds
    EXPECT_EQ(propulsion_system_heartbeat.heartbeat_counter, 40);
}

TEST(CStriderPropulsionTests, T11_test_propulsion_system_heartbeat_stopped_by_user)
{
    auto propulsion_system_heartbeat = PropulsionSystemHeartbeatState{0, 0};

    auto stop_propulsion_system_heartbeat = false;
    for (int t = 0; t <= 100; t++) {
        propulsion_system_heartbeat = update_propulsion_system_heartbeat(
            propulsion_system_heartbeat, t, std::chrono::seconds(1), 60, stop_propulsion_system_heartbeat);
        if (t == 50) {
            stop_propulsion_system_heartbeat = true;
        }
    }
    // Verify that the stop propulsion_system heartbeat override state is stopping the heartbeat from being incremented
    EXPECT_EQ(propulsion_system_heartbeat.heartbeat_counter, 50);
}