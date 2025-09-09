#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <zeabuz/common/io/messages/custom/wago_hatch.hpp>

#include "fmi_propulsion_drivers/zpa/zpa_spec_utilities.hpp"
#include "zeabuz/common/utilities/math.hpp"

TEST(ZpaPropulsionTests, T01_angle_conversion)
{
    auto angle_deg = 900;
    auto angle_rad = zpa_deg_to_wrapped_rad(angle_deg);
    auto angle_deg_fb = wrapped_rad_to_zpa_deg(angle_rad);
    EXPECT_EQ(angle_rad, M_PI / 2.0);
    EXPECT_EQ(angle_deg_fb, angle_deg);

    angle_deg = 1800;
    angle_rad = zpa_deg_to_wrapped_rad(angle_deg);
    angle_deg_fb = wrapped_rad_to_zpa_deg(angle_rad);
    EXPECT_EQ(angle_rad, M_PI);
    EXPECT_EQ(angle_deg_fb, angle_deg);

    angle_deg = -1800;
    angle_rad = zpa_deg_to_wrapped_rad(angle_deg);
    angle_deg_fb = wrapped_rad_to_zpa_deg(angle_rad);
    EXPECT_EQ(angle_rad, M_PI);
    EXPECT_EQ(angle_deg_fb, 1800);

    angle_deg = 3600;
    angle_rad = zpa_deg_to_wrapped_rad(angle_deg);
    angle_deg_fb = wrapped_rad_to_zpa_deg(angle_rad);
    EXPECT_EQ(angle_rad, 0.0);
    EXPECT_EQ(angle_deg_fb, 0.0);

    angle_deg = 3590;
    angle_rad = zpa_deg_to_wrapped_rad(angle_deg);
    angle_deg_fb = wrapped_rad_to_zpa_deg(angle_rad);
    EXPECT_NEAR(angle_rad, zeabuz::common::utilities::math::to_rad(-1.0), 1e-6);
    EXPECT_EQ(angle_deg_fb, angle_deg);
}
TEST(ZpaPropulsionTests, T02a_ready_for_autonomy_sequence_autonomy_is_not_in_control)
{
    auto previous_ready_for_autonomy_state = PreviousReadyForAutonomyState{};
    previous_ready_for_autonomy_state.button_signal = false;
    previous_ready_for_autonomy_state.ready_for_autonomy = false;

    auto inputs = ModelInputs{};
    inputs.modbus_tcp_server.autonomy_available = false;
    inputs.fmu.lcp_ready_for_autonomy_condition = false;
    inputs.modbus_tcp_server.ready_for_autonomy_button = false;
    auto result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_FALSE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_FALSE(result.ready_for_autonomy_allowed);

    inputs.modbus_tcp_server.autonomy_available = true;
    inputs.fmu.lcp_ready_for_autonomy_condition = true;
    inputs.modbus_tcp_server.ready_for_autonomy_button = true;
    result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_TRUE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_TRUE(result.ready_for_autonomy_allowed);

    inputs.modbus_tcp_server.autonomy_available = true;
    inputs.fmu.lcp_ready_for_autonomy_condition = false;
    inputs.modbus_tcp_server.ready_for_autonomy_button = false;
    result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_FALSE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_FALSE(result.ready_for_autonomy_allowed);

    inputs.modbus_tcp_server.autonomy_available = false;
    inputs.fmu.lcp_ready_for_autonomy_condition = true;
    inputs.modbus_tcp_server.ready_for_autonomy_button = false;
    result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_FALSE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_FALSE(result.ready_for_autonomy_allowed);

    inputs.modbus_tcp_server.autonomy_available = false;
    inputs.fmu.lcp_ready_for_autonomy_condition = false;
    inputs.modbus_tcp_server.ready_for_autonomy_button = true;
    result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_FALSE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_FALSE(result.ready_for_autonomy_allowed);

    inputs.modbus_tcp_server.autonomy_available = true;
    inputs.fmu.lcp_ready_for_autonomy_condition = true;
    inputs.modbus_tcp_server.ready_for_autonomy_button = false;
    result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_TRUE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_FALSE(result.ready_for_autonomy_allowed);

    inputs.modbus_tcp_server.autonomy_available = true;
    inputs.fmu.lcp_ready_for_autonomy_condition = false;
    inputs.modbus_tcp_server.ready_for_autonomy_button = true;
    result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_FALSE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_FALSE(result.ready_for_autonomy_allowed);

    inputs.modbus_tcp_server.autonomy_available = false;
    inputs.fmu.lcp_ready_for_autonomy_condition = true;
    inputs.modbus_tcp_server.ready_for_autonomy_button = true;
    result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_FALSE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_FALSE(result.ready_for_autonomy_allowed);
}

TEST(ZpaPropulsionTests, T02b_ready_for_autonomy_sequence_autonomy_already_in_control)
{
    // Sequence for setting ready for autonomy
    // 1.  Autonomy system available (“autonomy_available” = True)
    // 2. Set the levers (AL-P and AL-S) in neutral position (angle and RPM) (Is controllable by the user through input,
    // defaults to true)
    // 3. The LCP-P and LCP-S makes the “Ready for autonomy button” available by setting the
    //    “Ready for autonomy button available” signal to True.
    // 4. Press the “Ready for autonomy button” button on the Autonomy HMI
    // 5. The “Ready for autonomy button” signal is sent to LCP-P/LCP-S.
    // 6. LCP-P and LCP-S sets the “Ready for autonomy” flag to be read by the autonomy system.

    auto previous_ready_for_autonomy_state = PreviousReadyForAutonomyState{};
    previous_ready_for_autonomy_state.button_signal = false;
    previous_ready_for_autonomy_state.ready_for_autonomy = true;

    auto inputs = ModelInputs{};
    inputs.modbus_tcp_server.autonomy_available = false;
    inputs.fmu.lcp_ready_for_autonomy_condition = false;
    inputs.modbus_tcp_server.ready_for_autonomy_button = false;
    auto result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_FALSE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_FALSE(result.ready_for_autonomy_allowed);

    inputs.modbus_tcp_server.autonomy_available = true;
    inputs.fmu.lcp_ready_for_autonomy_condition = true;
    inputs.modbus_tcp_server.ready_for_autonomy_button = true;
    result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_TRUE(result.ready_for_autonomy_button_available_allowed);
    // Button press shall not be able to set ready_for_autonomy to false when it is already true
    EXPECT_TRUE(result.ready_for_autonomy_allowed);

    inputs.modbus_tcp_server.autonomy_available = true;
    inputs.fmu.lcp_ready_for_autonomy_condition = false;
    inputs.modbus_tcp_server.ready_for_autonomy_button = false;
    result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_FALSE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_FALSE(result.ready_for_autonomy_allowed);

    inputs.modbus_tcp_server.autonomy_available = false;
    inputs.fmu.lcp_ready_for_autonomy_condition = true;
    inputs.modbus_tcp_server.ready_for_autonomy_button = false;
    result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_FALSE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_FALSE(result.ready_for_autonomy_allowed);

    inputs.modbus_tcp_server.autonomy_available = false;
    inputs.fmu.lcp_ready_for_autonomy_condition = false;
    inputs.modbus_tcp_server.ready_for_autonomy_button = true;
    result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_FALSE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_FALSE(result.ready_for_autonomy_allowed);

    inputs.modbus_tcp_server.autonomy_available = true;
    inputs.fmu.lcp_ready_for_autonomy_condition = true;
    inputs.modbus_tcp_server.ready_for_autonomy_button = false;
    result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_TRUE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_TRUE(result.ready_for_autonomy_allowed);

    inputs.modbus_tcp_server.autonomy_available = true;
    inputs.fmu.lcp_ready_for_autonomy_condition = false;
    inputs.modbus_tcp_server.ready_for_autonomy_button = true;
    result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_FALSE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_FALSE(result.ready_for_autonomy_allowed);

    inputs.modbus_tcp_server.autonomy_available = false;
    inputs.fmu.lcp_ready_for_autonomy_condition = true;
    inputs.modbus_tcp_server.ready_for_autonomy_button = true;
    result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    EXPECT_FALSE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_FALSE(result.ready_for_autonomy_allowed);
}

TEST(ZpaPropulsionTests, T04_autonomy_heartbeat_sequence)
{
    // The autonomy system stops sending data to LCP-P and/or LCP-S
    // 1. A Watchdog in LCP-P and/or LCP-S detect that the autonomy system stops sending data (Heartbeat)
    // 2. “Ready for autonomy” flag is set to False (if it was True)
    // 3. The VMS HMI should alarm the operator
    // 4. RPM/angle commands is read from the levers (which are in the neutral position

    auto currentCommunicationPoint = 1.0;
    auto heartbeat_timeout = std::chrono::milliseconds(1000);
    auto inputs = ModelInputs{};
    // Autonomy heartbeat is now 5, and was 3 0.5 seconds ago meaning heartbeat counter was updated within heartbeat
    // timeout window -> expect heartbeat sequence to allow ready_for_autonmy and button_available, and counter to be 5
    inputs.modbus_tcp_server.autonomy_system_heartbeat = 5;
    auto result = autonomy_heartbeat_sequence(inputs, AutonomySystemHeartbeatState{currentCommunicationPoint - 0.5, 3},
                                              currentCommunicationPoint, heartbeat_timeout);
    EXPECT_TRUE(result.ready_for_autonomy_allowed);
    EXPECT_TRUE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_EQ(result.heartbeat_state.heartbeat_counter, 5);

    currentCommunicationPoint = 123.5;
    inputs.modbus_tcp_server.autonomy_system_heartbeat = 56;
    heartbeat_timeout = std::chrono::milliseconds(1000);
    // Autonomy heartbeat is now 56, and was 55 0.5 seconds ago meaning heartbeat counter was updated within heartbeat
    // timeout window -> expect heartbeat sequence to allow ready_for_autonmy and button_available, and counter to be 56
    result = autonomy_heartbeat_sequence(inputs, AutonomySystemHeartbeatState{currentCommunicationPoint - 0.5, 55},
                                         currentCommunicationPoint, heartbeat_timeout);
    EXPECT_TRUE(result.ready_for_autonomy_allowed);
    EXPECT_TRUE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_EQ(result.heartbeat_state.heartbeat_counter, 56);

    currentCommunicationPoint = 75.0;
    heartbeat_timeout = std::chrono::milliseconds(1000);
    inputs.modbus_tcp_server.autonomy_system_heartbeat = 6;
    // Autonomy heartbeat is now 6, and was 6 2.5 seconds ago meaning heartbeat counter was not updated within
    // heartbeat timeout window -> expect heartbeat sequence to not allow ready_for_autonmy and button_available, and
    // counter to be 6
    result = autonomy_heartbeat_sequence(inputs, AutonomySystemHeartbeatState{currentCommunicationPoint - 2.5, 6},
                                         currentCommunicationPoint, heartbeat_timeout);
    EXPECT_FALSE(result.ready_for_autonomy_allowed);
    EXPECT_FALSE(result.ready_for_autonomy_button_available_allowed);
    EXPECT_EQ(result.heartbeat_state.heartbeat_counter, 6);
}

TEST(ZpaPropulsionTests,
     T05a_test_on_rising_edge_of_button_when_ready_for_autonomy_false_causes_autonomy_in_control_and_cmds_sent)
{
    auto inputs = ModelInputs{};
    inputs.modbus_tcp_server.autonomy_available = true;
    inputs.fmu.lcp_ready_for_autonomy_condition = true;
    inputs.modbus_tcp_server.ready_for_autonomy_button = true;
    inputs.modbus_tcp_server.rpm_cmd = 900;
    inputs.modbus_tcp_server.angle_cmd = 1800;
    auto previous_ready_for_autonomy_state = PreviousReadyForAutonomyState{};
    previous_ready_for_autonomy_state.ready_for_autonomy = false;
    previous_ready_for_autonomy_state.button_signal = false;

    // Autonomy should be in control with the given inputs, autonomy available  = true, ready for autonomy button
    // available user input = true, and rising edge on ready_for_autonomy_button. Expect that the
    // commands are propagated to the output signals
    const auto currentCommunicationPoint = 1.0;

    auto ready_for_autonomy_sequence_result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    ASSERT_TRUE(ready_for_autonomy_sequence_result.ready_for_autonomy_allowed);
    ASSERT_TRUE(ready_for_autonomy_sequence_result.ready_for_autonomy_button_available_allowed);

    auto autonomy_heartbeat_sequence_result = autonomy_heartbeat_sequence(
        inputs, AutonomySystemHeartbeatState{0.5, 3}, currentCommunicationPoint, std::chrono::milliseconds(1000));
    ASSERT_TRUE(autonomy_heartbeat_sequence_result.ready_for_autonomy_allowed);
    ASSERT_TRUE(autonomy_heartbeat_sequence_result.ready_for_autonomy_button_available_allowed);

    auto outputs = get_model_output_signals(inputs, ready_for_autonomy_sequence_result,
                                            autonomy_heartbeat_sequence_result, {currentCommunicationPoint, 0});
    EXPECT_TRUE(outputs.fmu.ready_for_autonomy);
    EXPECT_TRUE(outputs.fmu.ready_for_autonomy_button_available);
    EXPECT_EQ(outputs.fmu.rpm_cmd, 900.0);
    EXPECT_EQ(outputs.fmu.angle_cmd, M_PI);
}

TEST(ZpaPropulsionTests,
     T05b_test_on_rising_edge_on_button_when_ready_for_autonomy_true_keeps_autonomy_in_control_and_cmds_sent)
{
    auto inputs = ModelInputs{};
    inputs.modbus_tcp_server.autonomy_available = true;
    inputs.fmu.lcp_ready_for_autonomy_condition = true;
    inputs.modbus_tcp_server.ready_for_autonomy_button = true;
    inputs.modbus_tcp_server.rpm_cmd = 900;
    inputs.modbus_tcp_server.angle_cmd = 1800;
    auto previous_ready_for_autonomy_state = PreviousReadyForAutonomyState{};
    previous_ready_for_autonomy_state.ready_for_autonomy = true;
    previous_ready_for_autonomy_state.button_signal = false;

    // Autonomy should be in control with the given inputs, autonomy available  = true, ready for autonomy button
    // available user input = true, and rising edge on ready_for_autonomy_button. Expect that the
    // commands are propagated to the output signals
    const auto currentCommunicationPoint = 1.0;

    auto ready_for_autonomy_sequence_result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    ASSERT_TRUE(ready_for_autonomy_sequence_result.ready_for_autonomy_allowed);
    ASSERT_TRUE(ready_for_autonomy_sequence_result.ready_for_autonomy_button_available_allowed);
    auto autonomy_heartbeat_sequence_result = autonomy_heartbeat_sequence(
        inputs, AutonomySystemHeartbeatState{0.5, 3}, currentCommunicationPoint, std::chrono::milliseconds(1000));
    ASSERT_TRUE(autonomy_heartbeat_sequence_result.ready_for_autonomy_allowed);
    ASSERT_TRUE(autonomy_heartbeat_sequence_result.ready_for_autonomy_button_available_allowed);

    auto outputs = get_model_output_signals(inputs, ready_for_autonomy_sequence_result,
                                            autonomy_heartbeat_sequence_result, {currentCommunicationPoint, 0});
    EXPECT_TRUE(outputs.fmu.ready_for_autonomy);
    EXPECT_TRUE(outputs.fmu.ready_for_autonomy_button_available);
    EXPECT_EQ(outputs.fmu.rpm_cmd, 900.0);
    EXPECT_EQ(outputs.fmu.angle_cmd, M_PI);
}

TEST(ZpaPropulsionTests, T06_test_fmu_outputs_autonomy_available_false_neutral_cmds_sent)
{
    auto inputs = ModelInputs{};
    // Autonomy system is not available, expect that no commands are sent to the propulsion and steering
    inputs.modbus_tcp_server.autonomy_available = false;
    inputs.fmu.lcp_ready_for_autonomy_condition = true;
    inputs.modbus_tcp_server.ready_for_autonomy_button = true;
    inputs.modbus_tcp_server.rpm_cmd = 900;
    inputs.modbus_tcp_server.angle_cmd = 1800;
    auto previous_ready_for_autonomy_state = PreviousReadyForAutonomyState{};
    previous_ready_for_autonomy_state.ready_for_autonomy = false;
    previous_ready_for_autonomy_state.button_signal = false;

    const auto currentCommunicationPoint = 1.0;
    auto ready_for_autonomy_sequence_result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    ASSERT_FALSE(ready_for_autonomy_sequence_result.ready_for_autonomy_allowed);
    ASSERT_FALSE(ready_for_autonomy_sequence_result.ready_for_autonomy_button_available_allowed);
    auto autonomy_heartbeat_sequence_result = autonomy_heartbeat_sequence(
        inputs, AutonomySystemHeartbeatState{0.5, 3}, currentCommunicationPoint, std::chrono::milliseconds(1000));
    ASSERT_TRUE(autonomy_heartbeat_sequence_result.ready_for_autonomy_allowed);
    ASSERT_TRUE(autonomy_heartbeat_sequence_result.ready_for_autonomy_button_available_allowed);

    auto outputs = get_model_output_signals(inputs, ready_for_autonomy_sequence_result,
                                            autonomy_heartbeat_sequence_result, {currentCommunicationPoint, 0});
    EXPECT_FALSE(outputs.fmu.ready_for_autonomy);
    EXPECT_FALSE(outputs.fmu.ready_for_autonomy_button_available);
    EXPECT_EQ(outputs.fmu.rpm_cmd, 0.0);
    EXPECT_EQ(outputs.fmu.angle_cmd, 0.0);
}

TEST(ZpaPropulsionTests, T07_test_fmu_outputs_autonomy_heartbeat_missing_neutral_cmds_sent)
{
    auto inputs = ModelInputs{};
    // Autonomy system heartbeat is not received, expect that no commands are sent to the propulsion and steering
    // 3.9 seconds has passed and heartbeat counter is still the same as input, expect that no commands are sent
    inputs.modbus_tcp_server.autonomy_available = true;
    inputs.modbus_tcp_server.ready_for_autonomy_button = true;
    inputs.modbus_tcp_server.rpm_cmd = 900;
    inputs.modbus_tcp_server.angle_cmd = 1800;
    inputs.modbus_tcp_server.autonomy_system_heartbeat = 3;
    inputs.fmu.lcp_ready_for_autonomy_condition = true;
    auto previous_ready_for_autonomy_state = PreviousReadyForAutonomyState{};
    previous_ready_for_autonomy_state.ready_for_autonomy = false;
    previous_ready_for_autonomy_state.button_signal = false;

    // Autonomy heartbeat is now 3, and was 3 3.9 seconds ago meaning heartbeat counter was not updated within
    // heartbeat timeout window -> expect heartbeat sequence to not allow ready_for_autonmy and button_available, and
    // counter to be 3
    const auto currentCommunicationPoint = 5.0;
    auto ready_for_autonomy_sequence_result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    ASSERT_TRUE(ready_for_autonomy_sequence_result.ready_for_autonomy_allowed);
    ASSERT_TRUE(ready_for_autonomy_sequence_result.ready_for_autonomy_button_available_allowed);
    auto autonomy_heartbeat_sequence_result = autonomy_heartbeat_sequence(
        inputs, AutonomySystemHeartbeatState{1.1, 3}, currentCommunicationPoint, std::chrono::milliseconds(1000));
    ASSERT_FALSE(autonomy_heartbeat_sequence_result.ready_for_autonomy_allowed);
    ASSERT_FALSE(autonomy_heartbeat_sequence_result.ready_for_autonomy_button_available_allowed);

    auto outputs = get_model_output_signals(inputs, ready_for_autonomy_sequence_result,
                                            autonomy_heartbeat_sequence_result, {currentCommunicationPoint, 0});
    EXPECT_FALSE(outputs.fmu.ready_for_autonomy);
    EXPECT_FALSE(outputs.fmu.ready_for_autonomy_button_available);
    EXPECT_EQ(outputs.fmu.rpm_cmd, 0.0);
    EXPECT_EQ(outputs.fmu.angle_cmd, 0.0);
    EXPECT_EQ(outputs.fmu.autonomy_system_heartbeat, 3);
}

TEST(ZpaPropulsionTests, T08_test_fmu_outputs_lcp_ready_for_autonomy_condition_false_neutral_cmds_sent)
{
    auto inputs = ModelInputs{};
    // Autonomy system heartbeat is not received, expect that no commands are sent to the propulsion and steering
    // 3.9 seconds has passed and heartbeat counter is still the same as input, expect that no commands are sent
    inputs.modbus_tcp_server.autonomy_available = true;
    inputs.modbus_tcp_server.ready_for_autonomy_button = true;
    inputs.modbus_tcp_server.rpm_cmd = 900;
    inputs.modbus_tcp_server.angle_cmd = 1800;
    inputs.modbus_tcp_server.autonomy_system_heartbeat = 3;
    inputs.fmu.lcp_ready_for_autonomy_condition = false;
    auto previous_ready_for_autonomy_state = PreviousReadyForAutonomyState{};
    previous_ready_for_autonomy_state.ready_for_autonomy = true;
    previous_ready_for_autonomy_state.button_signal = false;

    const auto currentCommunicationPoint = 4.0;
    auto ready_for_autonomy_sequence_result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    ASSERT_FALSE(ready_for_autonomy_sequence_result.ready_for_autonomy_allowed);
    ASSERT_FALSE(ready_for_autonomy_sequence_result.ready_for_autonomy_button_available_allowed);
    auto autonomy_heartbeat_sequence_result = autonomy_heartbeat_sequence(
        inputs, AutonomySystemHeartbeatState{3.1, 3}, currentCommunicationPoint, std::chrono::milliseconds(1000));
    ASSERT_TRUE(autonomy_heartbeat_sequence_result.ready_for_autonomy_allowed);
    ASSERT_TRUE(autonomy_heartbeat_sequence_result.ready_for_autonomy_button_available_allowed);

    auto outputs = get_model_output_signals(inputs, ready_for_autonomy_sequence_result,
                                            autonomy_heartbeat_sequence_result, {currentCommunicationPoint, 0});
    EXPECT_FALSE(outputs.fmu.ready_for_autonomy);
    EXPECT_FALSE(outputs.fmu.ready_for_autonomy_button_available);
    EXPECT_EQ(outputs.fmu.rpm_cmd, 0.0);
    EXPECT_EQ(outputs.fmu.angle_cmd, 0.0);
}

TEST(ZpaPropulsionTests, T09_test_that_all_fmu_output_signals_struct_elements_are_set_correctly)
{
    auto inputs = ModelInputs{};
    // Autonomy available = true, ready for autonomy button available = true, ready for autonomy button = true,
    // not enough time has passed for heartbeat to be missing, expect that all elements in output_signals are set
    inputs.modbus_tcp_server.autonomy_system_heartbeat = 3;
    inputs.modbus_tcp_server.autonomy_available = true;
    inputs.modbus_tcp_server.ready_for_autonomy_button = true;
    inputs.modbus_tcp_server.rpm_cmd = 900;
    inputs.modbus_tcp_server.angle_cmd = 1800;
    inputs.fmu.angle_rad = M_PI / 2.0;
    inputs.fmu.propeller_rpm = 69.0;
    inputs.fmu.propulsion_drive_ready = true;
    inputs.fmu.propulsion_drive_running = true;
    inputs.fmu.propulsion_drive_fault = true;
    inputs.fmu.steering_drive_ready = true;
    inputs.fmu.steering_drive_running = true;
    inputs.fmu.steering_drive_fault = true;
    inputs.fmu.lcp_ready_for_autonomy_condition = true;
    auto previous_ready_for_autonomy_state = PreviousReadyForAutonomyState{};
    previous_ready_for_autonomy_state.ready_for_autonomy = false;
    previous_ready_for_autonomy_state.button_signal = false;

    const auto currentCommunicationPoint = 2.0;
    auto ready_for_autonomy_sequence_result = ready_for_autonomy_sequence(inputs, previous_ready_for_autonomy_state);
    ASSERT_TRUE(ready_for_autonomy_sequence_result.ready_for_autonomy_allowed);
    ASSERT_TRUE(ready_for_autonomy_sequence_result.ready_for_autonomy_button_available_allowed);
    auto autonomy_heartbeat_sequence_result = autonomy_heartbeat_sequence(
        inputs, AutonomySystemHeartbeatState{1.1, 3}, currentCommunicationPoint, std::chrono::milliseconds(1000));
    ASSERT_TRUE(autonomy_heartbeat_sequence_result.ready_for_autonomy_allowed);
    ASSERT_TRUE(autonomy_heartbeat_sequence_result.ready_for_autonomy_button_available_allowed);

    auto output_signals = get_model_output_signals(inputs, ready_for_autonomy_sequence_result,
                                                   autonomy_heartbeat_sequence_result, {currentCommunicationPoint, 9});
    // Assert that all elements in output_signals are set
    EXPECT_TRUE(output_signals.fmu.autonomy_available);
    EXPECT_TRUE(output_signals.fmu.ready_for_autonomy);
    EXPECT_TRUE(output_signals.fmu.ready_for_autonomy_button);
    EXPECT_TRUE(output_signals.fmu.ready_for_autonomy_button_available);
    EXPECT_EQ(output_signals.fmu.rpm_cmd, 900.0);
    EXPECT_EQ(output_signals.fmu.angle_cmd, M_PI);
    EXPECT_EQ(output_signals.fmu.autonomy_system_heartbeat, 3);
    EXPECT_EQ(output_signals.fmu.lcp_heartbeat, 9);

    EXPECT_EQ(output_signals.modbus_tcp_server.ready_for_autonomy, true);
    EXPECT_EQ(output_signals.modbus_tcp_server.ready_for_autonomy_button_available, true);
    EXPECT_EQ(output_signals.modbus_tcp_server.rpm_fb, 69);
    EXPECT_EQ(output_signals.modbus_tcp_server.angle_fb, 900);
    EXPECT_EQ(output_signals.modbus_tcp_server.steering_status, 7);
    EXPECT_EQ(output_signals.modbus_tcp_server.propulsion_status, 7);
}

TEST(ZpaPropulsionTests, T10_test_lcp_heartbeat_wrap_around)
{
    auto lcp_heartbeat = LCPHeartbeatState{0, 0};

    for (int t = 0; t <= 100; t++) {
        lcp_heartbeat = update_lcp_heartbeat(lcp_heartbeat, t, std::chrono::seconds(1), 60, false);
    }
    // Expect that heartbeat has wrapped around from 60 -> 1, and is at 40 after 100 seconds
    EXPECT_EQ(lcp_heartbeat.heartbeat_counter, 40);
}

TEST(ZpaPropulsionTests, T11_test_lcp_heartbeat_stopped_by_user)
{
    auto lcp_heartbeat = LCPHeartbeatState{0, 0};

    auto stop_lcp_heartbeat = false;
    for (int t = 0; t <= 100; t++) {
        lcp_heartbeat = update_lcp_heartbeat(lcp_heartbeat, t, std::chrono::seconds(1), 60, stop_lcp_heartbeat);
        if (t == 50) {
            stop_lcp_heartbeat = true;
        }
    }
    // Verify that the stop lcp heartbeat override state is stopping the heartbeat from being incremented
    EXPECT_EQ(lcp_heartbeat.heartbeat_counter, 50);
}