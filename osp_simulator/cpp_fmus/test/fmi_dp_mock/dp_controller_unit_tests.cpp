#include <gtest/gtest.h>


#include "fmi_dp_mock/data_types.hpp"
#include "fmi_dp_mock/dynamic_positioning.hpp"
#include "fmi_dp_mock/models/models.hpp"
#include <zeabuz/common/utilities/math.hpp>

using namespace std::chrono_literals;
using namespace DPMock;
using zeabuz::common::utilities::math::heading_to_rotation_matrix;

class SecondOrderFilter
{
   public:
    SecondOrderFilter(double tau, Eigen::Vector2d x0, double dt) : m_x(x0), m_dt(dt)
    {
        double tausqr = tau * tau;
        m_A << 0, 1, -1 / tausqr, -2 / tau;
        m_B = {0, 1 / tausqr};
    }

    void step(double u)
    {
        Eigen::Vector2d x_dot = m_A * m_x + m_B * u;
        m_x = m_x + m_dt * x_dot;
    }

    Eigen::Vector2d get_state()
    {
        return m_x;
    }

   private:
    Eigen::Matrix2d m_A;
    Eigen::Vector2d m_B;
    Eigen::Vector2d m_x;
    double m_dt;
};

class ThirdOrderFilter
{
   public:
    ThirdOrderFilter(double tau, Eigen::Vector3d x0, double dt) : m_x(x0), m_dt(dt)
    {
        double tausqr = tau * tau;
        double taucub = tausqr * tau;
        m_A << 0, 1, 0, 0, 0, 1, -1 / taucub, -3 / tausqr, -3 / tau;
        m_B = {0, 0, 1 / taucub};
    }

    void step(double u)
    {
        Eigen::Vector3d x_dot = m_A * m_x + m_B * u;
        m_x = m_x + m_dt * x_dot;
    }

    double get_pos()
    {
        return m_x(0);
    }

    double get_velocity()
    {
        return m_x(1);
    }

    double get_accel()
    {
        return m_x(2);
    }

   private:
    Eigen::Matrix3d m_A;
    Eigen::Vector3d m_B;
    Eigen::Vector3d m_x;
    double m_dt;
};

class ReferenceFilter
{
   public:
    ReferenceFilter(double tau, Eigen::Vector3d eta, double dt)
    {
        Eigen::Vector3d nf_0 = {eta(0), 0, 0};
        Eigen::Vector3d ef_0 = {eta(1), 0, 0};
        Eigen::Vector3d hf_0 = {eta(2), 0, 0};
        m_FilterNorth = std::make_unique<ThirdOrderFilter>(tau, nf_0, dt);
        m_FilterEast = std::make_unique<ThirdOrderFilter>(tau, ef_0, dt);
        m_FilterHeading = std::make_unique<ThirdOrderFilter>(tau, hf_0, dt);
    }

    void step(Eigen::Vector3d u)
    {
        m_FilterNorth->step(u(0));
        m_FilterEast->step(u(1));
        m_FilterHeading->step(u(2));
    }

    Eigen::Vector3d get_positions()
    {
        return {m_FilterNorth->get_pos(), m_FilterEast->get_pos(), m_FilterHeading->get_pos()};
    }

    Eigen::Vector3d get_velocities()
    {
        return {m_FilterNorth->get_velocity(), m_FilterEast->get_velocity(), m_FilterHeading->get_velocity()};
    }

    Eigen::Vector3d get_acceleration()
    {
        return {m_FilterNorth->get_accel(), m_FilterEast->get_accel(), m_FilterHeading->get_accel()};
    }

   private:
    std::unique_ptr<ThirdOrderFilter> m_FilterNorth;
    std::unique_ptr<ThirdOrderFilter> m_FilterEast;
    std::unique_ptr<ThirdOrderFilter> m_FilterHeading;
};

class DpControllerUnitTests : public ::testing::Test
{
   public:
    DpControllerUnitTests()
    {
        m_defaultInputs.enable_ready_for_autonomy = false;
        m_defaultInputs.enable_autonomy_mode = false;
        m_defaultInputs.step_size = 0.05ms;
        m_defaultInputs.desired_control_mode = ControlMode::JOYSTICK;
    }

    void SetUp() override
    {
    }

    DPInputs m_defaultInputs;
};

TEST_F(DpControllerUnitTests, T01_OpenLoop_StepController)
{
    auto ctrl = std::make_unique<DPController>();
}

TEST_F(DpControllerUnitTests, T02_SetReadyForAutonomyStatus_getReadyAutonomyStatus)
{
    auto ctrl = std::make_unique<DPController>();
    DPOutputs outputs = ctrl->get_outputs();
    EXPECT_FALSE(outputs.ready_for_autonomy_enabled);

    m_defaultInputs.enable_ready_for_autonomy = true;

    ctrl->step(m_defaultInputs);
    outputs = ctrl->get_outputs();
    EXPECT_TRUE(outputs.ready_for_autonomy_enabled);

    m_defaultInputs.enable_ready_for_autonomy = false;
    ctrl->step(m_defaultInputs);
    outputs = ctrl->get_outputs();
    EXPECT_FALSE(outputs.ready_for_autonomy_enabled);
}

TEST_F(DpControllerUnitTests, T03_SetEnableAutonomyStatus_getEnableAutonomyStatus)
{
    auto ctrl = std::make_unique<DPController>();

    DPOutputs outputs = ctrl->get_outputs();
    EXPECT_FALSE(outputs.autonomy_mode_enabled);

    m_defaultInputs.enable_autonomy_mode = true;
    m_defaultInputs.enable_ready_for_autonomy = false;
    ctrl->step(m_defaultInputs);

    outputs = ctrl->get_outputs();
    EXPECT_FALSE(outputs.autonomy_mode_enabled);

    m_defaultInputs.enable_autonomy_mode = true;
    m_defaultInputs.enable_ready_for_autonomy = true;
    ctrl->step(m_defaultInputs);
    outputs = ctrl->get_outputs();
    EXPECT_TRUE(outputs.autonomy_mode_enabled);

    m_defaultInputs.enable_autonomy_mode = false;
    m_defaultInputs.enable_ready_for_autonomy = true;
    ctrl->step(m_defaultInputs);
    outputs = ctrl->get_outputs();
    EXPECT_FALSE(outputs.autonomy_mode_enabled);
}

TEST_F(DpControllerUnitTests, T04_EnableAutonomy_ChangeReadyForAutonomyToFalse_ExpectAutonomyModeFalse)
{
    auto ctrl = std::make_unique<DPController>();

    m_defaultInputs.enable_autonomy_mode = true;
    m_defaultInputs.enable_ready_for_autonomy = true;

    ctrl->step(m_defaultInputs);
    DPOutputs outputs = ctrl->get_outputs();
    EXPECT_TRUE(outputs.autonomy_mode_enabled);

    m_defaultInputs.enable_autonomy_mode = true;
    m_defaultInputs.enable_ready_for_autonomy = false;
    ctrl->step(m_defaultInputs);
    outputs = ctrl->get_outputs();
    EXPECT_FALSE(outputs.autonomy_mode_enabled);
}

TEST_F(DpControllerUnitTests, T05_SetControlMode_CheckControlMode)
{
    auto ctrl = std::make_unique<DPController>();
    DPOutputs outputs = ctrl->get_outputs();
    EXPECT_EQ(outputs.control_mode, DPMock::ControlMode::STANDBY);

    m_defaultInputs.enable_autonomy_mode = true;
    m_defaultInputs.enable_ready_for_autonomy = true;
    m_defaultInputs.desired_control_mode = ControlMode::DP;
    ctrl->step(m_defaultInputs);
    outputs = ctrl->get_outputs();
    EXPECT_EQ(outputs.control_mode, DPMock::ControlMode::DP);
}

TEST_F(DpControllerUnitTests, T06_SetJoystickControlMode_CheckCommand)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.forces_ref_autonomy = {10, 20, 30};
    inputs.forces_ref_manual = {40, 50, 60};
    inputs.desired_control_mode = ControlMode::JOYSTICK;
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;

    DPParameters param;
    param.max_effort = {1000, 1000, 1000};
    ctrl->configure(param);

    ctrl->step(inputs);
    DPOutputs outputs = ctrl->get_outputs();
    EXPECT_EQ(outputs.generalized_forces, inputs.forces_ref_manual * 10);

    inputs.enable_autonomy_mode = true;
    inputs.enable_ready_for_autonomy = false;
    ctrl->step(inputs);
    outputs = ctrl->get_outputs();
    EXPECT_EQ(outputs.generalized_forces,
              inputs.forces_ref_manual * 10);  // autonomy mode false because enableReadyForAutonomy=false

    inputs.enable_autonomy_mode = true;
    inputs.enable_ready_for_autonomy = true;
    ctrl->step(inputs);
    outputs = ctrl->get_outputs();
    EXPECT_EQ(outputs.generalized_forces, inputs.forces_ref_autonomy * 10);
}

TEST_F(DpControllerUnitTests, T07_SetJoystickControlMode_CheckDPOutputStruct)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.forces_ref_autonomy = {10, 20, 30};
    inputs.forces_ref_manual = {40, 50, 60};
    inputs.desired_control_mode = ControlMode::JOYSTICK;
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;

    DPParameters param;
    param.max_effort = {1000, 1000, 1000};
    ctrl->configure(param);

    ctrl->step(inputs);
    DPOutputs outputs = ctrl->get_outputs();
    EXPECT_EQ(outputs.generalized_forces, inputs.forces_ref_manual * 10);
}

TEST_F(DpControllerUnitTests, T08_SwitchControlMode_BumplessTransfer)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.dp_ref_manual.north_east_heading = {10.0, 20.0, 0.5};
    inputs.measurements.north_east_heading = {15.0, 20.0, 0.5};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};
    inputs.dp_ref_manual.surge_integrator_mode = IntegrationMode::ENABLE;
    inputs.dp_ref_manual.sway_integrator_mode = IntegrationMode::ENABLE;
    inputs.dp_ref_manual.heading_integrator_mode = IntegrationMode::ENABLE;
    inputs.measurements.nu = {0.0, 0.0, 0.0};

    inputs.forces_ref_manual = {30, 40, 50};
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;

    DPParameters param;
    param.Kp = {0.1, 0.2, 0.3};
    param.Ki = {0.11, 0.22, 0.33};
    param.Kd = {0.111, 0.222, 0.333};
    param.generalized_forces_saturation = {5000.0, 5000.0, 5000.0};
    param.integral_action_saturation = {3000.0, 3000.0, 3000.0};
    param.max_effort = {1000, 1000, 1000};
    ctrl->configure(param);

    inputs.desired_control_mode = ControlMode::JOYSTICK;
    ctrl->step(inputs);
    DPOutputs outputs = ctrl->get_outputs();
    EXPECT_FALSE(outputs.ready_for_autonomy_enabled);
    EXPECT_FALSE(outputs.autonomy_mode_enabled);

    EXPECT_EQ(outputs.control_mode, ControlMode::JOYSTICK);
    EXPECT_EQ(outputs.generalized_forces, inputs.forces_ref_manual * 10);

    inputs.desired_control_mode = ControlMode::DP;
    ctrl->step(inputs);
    Eigen::Vector3d integral = ctrl->get_integral_action();
    Eigen::Vector3d expected(0, 0, 0);
    Eigen::Vector3d ctrlError = inputs.dp_ref_manual.north_east_heading - inputs.measurements.north_east_heading;
    expected = expected.array() + (param.Ki.array() * ctrlError.array()) *
                                      inputs.step_size.count();  // expects integrator to start from zero
    EXPECT_EQ(integral, expected);
}

TEST_F(DpControllerUnitTests, T09_SetInvalidTimeStep_ExpectException)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.dp_ref_manual.north_east_heading = {10.0, 20.0, 0.5};
    inputs.forces_ref_manual = {0.33, 0.44, 0.1};
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = -0.05s;
    EXPECT_THROW(ctrl->step(inputs), std::invalid_argument);
}

TEST_F(DpControllerUnitTests, T10_SetDPMode_GetActiveReference)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.dp_ref_manual.north_east_heading = {10.0, 20.0, 0.5};
    inputs.dp_ref_autonomy.north_east_heading = {30.0, 40.0, 0.3};

    inputs.forces_ref_manual = {0.33, 0.44, 0.1};
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;

    inputs.desired_control_mode = ControlMode::DP;
    ctrl->step(inputs);
    DPReference activeRef = ctrl->get_active_dp_ref();
    EXPECT_EQ(activeRef, inputs.dp_ref_manual);

    inputs.enable_autonomy_mode = true;
    inputs.enable_ready_for_autonomy = false;
    ctrl->step(inputs);
    activeRef = ctrl->get_active_dp_ref();
    EXPECT_EQ(activeRef, inputs.dp_ref_manual);

    inputs.enable_autonomy_mode = true;
    inputs.enable_ready_for_autonomy = true;
    ctrl->step(inputs);
    activeRef = ctrl->get_active_dp_ref();
    EXPECT_EQ(activeRef, inputs.dp_ref_autonomy);
}

TEST_F(DpControllerUnitTests, T11_ConfigureController)
{
    auto ctrl = std::make_unique<DPController>();
    DPParameters param;
    param.Kp = {0.1, 0.2, 0.3};
    param.Ki = {0.11, 0.22, 0.33};
    param.Kd = {0.111, 0.222, 0.333};
    param.generalized_forces_saturation = {2.0, 3.0, 4.0};
    param.integral_action_saturation = {5.0, 6.0, 7.0};
    param.autopilot.pid_heading = {3, 4, 5};
    param.autopilot.pid_surge = {3, 4, 5};
    ctrl->configure(param);
    DPParameters activeConfig = ctrl->get_active_config();
    EXPECT_EQ(param, activeConfig);
}

TEST_F(DpControllerUnitTests, T12_StepDPMode_VerifyIntegrator)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.dp_ref_manual.north_east_heading = {10.0, 20.0, 0.5};
    inputs.measurements.north_east_heading = {15.0, 25.0, 0.4};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};
    inputs.measurements.nu = {0.0, 0.0, 0.0};
    inputs.dp_ref_manual.north_east_heading_accel = {0.0, 0.0, 0.0};
    inputs.dp_ref_manual.surge_integrator_mode = IntegrationMode::ENABLE;
    inputs.dp_ref_manual.sway_integrator_mode = IntegrationMode::ENABLE;
    inputs.dp_ref_manual.heading_integrator_mode = IntegrationMode::ENABLE;

    inputs.forces_ref_manual = {0.33, 0.44, 0.1};
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;
    inputs.desired_control_mode = ControlMode::DP;

    DPParameters param;
    param.Kp = {0.1, 0.2, 0.3};
    param.Ki = {0.11, 0.22, 0.33};
    param.Kd = {0.111, 0.222, 0.333};
    param.generalized_forces_saturation = {5000.0, 5000.0, 5000.0};
    param.integral_action_saturation = {3000.0, 3000.0, 3000.0};
    ctrl->configure(param);

    ctrl->step(inputs);
    Eigen::Vector3d intAction0 = ctrl->get_integral_action();
    int nSteps = 100;
    for (int i = 0; i < nSteps; ++i) {
        ctrl->step(inputs);
    }
    Eigen::Vector3d intActionFinal = ctrl->get_integral_action();
    Eigen::Vector3d etaError = inputs.dp_ref_manual.north_east_heading - inputs.measurements.north_east_heading;
    Eigen::Vector3d expectedIntAction =
        intAction0.array() + (param.Ki.array() * etaError.array()) * nSteps * inputs.step_size.count();

    Eigen::Vector3d error = expectedIntAction - intActionFinal;
    EXPECT_NEAR(error.norm(), 0.0, 1e-12);

    Eigen::Matrix3d rotMat =
       heading_to_rotation_matrix(inputs.measurements.north_east_heading(2));  // Rotation body to NED

    Eigen::Vector3d PAction = ctrl->get_proportional_action();
    Eigen::Vector3d errorBody = rotMat.transpose() * etaError;
    Eigen::Vector3d expectedPAction = param.Kp.array() * errorBody.array();

    Eigen::Vector3d PError = expectedPAction - PAction;
    EXPECT_NEAR(PError.norm(), 0.0, 1e-12);

    Eigen::Vector3d expectedDAction = {0, 0, 0};
    Eigen::Vector3d DAction = ctrl->get_derivative_action();
    Eigen::Vector3d DError = expectedDAction - DAction;
    EXPECT_NEAR(DError.norm(), 0.0, 1e-12);

    Eigen::Vector3d expectedFFAction = {0, 0, 0};
    Eigen::Vector3d FFAction = ctrl->get_feedforward_action();
    Eigen::Vector3d FFError = expectedFFAction - FFAction;
    EXPECT_NEAR(FFError.norm(), 0.0, 1e-12);

    Eigen::Vector3d expectedGenForces = rotMat.transpose() * ctrl->get_integral_action() +
                                        ctrl->get_proportional_action() + ctrl->get_derivative_action() +
                                        ctrl->get_feedforward_action();

    DPOutputs outputs = ctrl->get_outputs();
    Eigen::Vector3d errorForce = expectedGenForces - outputs.generalized_forces;
    EXPECT_NEAR(errorForce.norm(), 0.0, 1e-12);
}

TEST_F(DpControllerUnitTests, T13_ClosedLoop_AutonomyOff)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.dp_ref_manual.north_east_heading = {12.0, 20.0, 0.5};
    inputs.measurements.north_east_heading = {12.0, 20.0, 0.4};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};
    inputs.dp_ref_manual.surge_integrator_mode = IntegrationMode::ENABLE;
    inputs.dp_ref_manual.sway_integrator_mode = IntegrationMode::ENABLE;
    inputs.dp_ref_manual.heading_integrator_mode = IntegrationMode::ENABLE;

    inputs.forces_ref_manual = {0.0, 0.0, 0.0};
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;
    inputs.desired_control_mode = ControlMode::JOYSTICK;

    DPParameters param;
    param.Kp = {200.0, 200.0, 800.0};
    param.Ki = {10.0, 10.0, 15.0};
    param.Kd = {700.0, 700.0, 1600.0};
    ctrl->configure(param);

    auto dt = 0.05s;
    Eigen::Vector3d tau0 = {0, 0, 0};
    auto model = std::make_unique<ThreeDOFModel>(inputs.measurements.north_east_heading,
                                                 inputs.dp_ref_manual.north_east_heading_vel);

    int nSteps = 3000;
    int indexChangeMode = 10;

    for (int i = 0; i < nSteps; i++) {
        if (i == indexChangeMode) {
            inputs.desired_control_mode = ControlMode::DP;
        }

        model->step(tau0, dt);
        Eigen::Vector3d eta_m = model->getNorthEastHeading();
        Eigen::Vector3d nu_m = model->getSurgeSwayYawRate();

        inputs.measurements.north_east_heading = eta_m;
        inputs.measurements.nu = nu_m;
        inputs.step_size = dt;

        ctrl->step(inputs);
        DPOutputs outputs = ctrl->get_outputs();
        tau0 = outputs.generalized_forces;
    }
}

TEST_F(DpControllerUnitTests, T14_InitialState_Standby_OutputZeroForces)
{
    auto ctrl = std::make_unique<DPController>();
    DPOutputs outputs = ctrl->get_outputs();
    EXPECT_EQ(outputs.control_mode, DPMock::ControlMode::STANDBY);
    Eigen::Vector3d expectedForce = {0, 0, 0};
    EXPECT_EQ(outputs.generalized_forces, expectedForce);

    m_defaultInputs.desired_control_mode = DPMock::ControlMode::STANDBY;
    ctrl->step(m_defaultInputs);

    outputs = ctrl->get_outputs();
    EXPECT_EQ(outputs.control_mode, DPMock::ControlMode::STANDBY);
    EXPECT_EQ(outputs.generalized_forces, expectedForce);
}

TEST_F(DpControllerUnitTests, T15_StepDPMode_VerifyIntegratorSaturation)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.dp_ref_manual.north_east_heading = {1.0, 2.0, 0.0};
    inputs.measurements.north_east_heading = {15.0, 25.0, M_PI / 2};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};
    inputs.dp_ref_manual.surge_integrator_mode = IntegrationMode::ENABLE;
    inputs.dp_ref_manual.sway_integrator_mode = IntegrationMode::ENABLE;
    inputs.dp_ref_manual.heading_integrator_mode = IntegrationMode::ENABLE;

    inputs.measurements.nu = {0.0, 0.0, 0.0};

    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;
    inputs.desired_control_mode = ControlMode::DP;

    DPParameters param;
    param.Kp = {500, 300, 200};
    param.Ki = {30, 30, 15};
    param.Kd = {8000, 5000, 3000};
    param.generalized_forces_saturation = {5000.0, 5000.0, 5000.0};
    param.integral_action_saturation = {3000.0, 3000.0, 3000.0};
    ctrl->configure(param);

    ctrl->step(inputs);
    int nSteps = 10000;
    for (int i = 0; i < nSteps; ++i) {
        ctrl->step(inputs);
    }

    Eigen::Vector3d intActionFinal = ctrl->get_integral_action();

    EXPECT_NEAR(intActionFinal(0), -param.integral_action_saturation(0), 1e-12);
    EXPECT_NEAR(intActionFinal(1), -param.integral_action_saturation(1), 1e-12);
    EXPECT_NEAR(intActionFinal(2), -param.integral_action_saturation(2), 1e-12);

    // change inputs to test negative saturation
    inputs.dp_ref_manual.north_east_heading = {1.0, 2.0, 0.0};
    inputs.measurements.north_east_heading = {-15.0, -25.0, -M_PI / 2};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};
    inputs.measurements.nu = {0.0, 0.0, 0.0};

    ctrl->step(inputs);
    for (int i = 0; i < nSteps; ++i) {
        ctrl->step(inputs);
    }

    intActionFinal = ctrl->get_integral_action();

    EXPECT_NEAR(intActionFinal(0), param.integral_action_saturation(0), 1e-12);
    EXPECT_NEAR(intActionFinal(1), param.integral_action_saturation(1), 1e-12);
    EXPECT_NEAR(intActionFinal(2), param.integral_action_saturation(2), 1e-12);
}

TEST_F(DpControllerUnitTests, T15_StepDPMode_VerifyGeneralizedForcesSaturation)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.dp_ref_manual.north_east_heading = {1.0, 1.0, 0.0};
    inputs.measurements.north_east_heading = {15.0, 15.0, M_PI / 3};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};
    inputs.measurements.nu = {0.0, 0.0, 0.0};
    inputs.dp_ref_manual.north_east_heading_accel = {0.0, 0.0, 0.0};
    inputs.dp_ref_manual.surge_integrator_mode = IntegrationMode::ENABLE;
    inputs.dp_ref_manual.sway_integrator_mode = IntegrationMode::ENABLE;
    inputs.dp_ref_manual.heading_integrator_mode = IntegrationMode::ENABLE;

    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;
    inputs.desired_control_mode = ControlMode::DP;

    DPParameters param;
    param.Kp = {500, 300, 200};
    param.Ki = {30, 30, 15};
    param.Kd = {8000, 5000, 3000};
    param.generalized_forces_saturation = {1000.0, 1000.0, 1000.0};
    param.integral_action_saturation = {1000, 1000, 1000};
    ctrl->configure(param);

    ctrl->step(inputs);
    int nSteps = 4000;
    for (int i = 0; i < nSteps; ++i) {
        ctrl->step(inputs);
    }

    Eigen::Vector3d generalizedForcesFinal = ctrl->get_outputs().generalized_forces;

    EXPECT_NEAR(generalizedForcesFinal(0), -param.generalized_forces_saturation(0), 1e-12);
    EXPECT_NEAR(generalizedForcesFinal(1), param.generalized_forces_saturation(1), 1e-12);
    EXPECT_NEAR(generalizedForcesFinal(2), -param.generalized_forces_saturation(2), 1e-12);

    // change inputs to test negative saturation
    inputs.dp_ref_manual.north_east_heading = {1.0, 1.0, 0.0};
    inputs.measurements.north_east_heading = {15.0, -15.0, -M_PI / 3};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};
    inputs.measurements.nu = {0.0, 0.0, 0.0};
    inputs.dp_ref_manual.north_east_heading_accel = {0.0, 0.0, 0.0};

    ctrl->step(inputs);
    for (int i = 0; i < nSteps; ++i) {
        ctrl->step(inputs);
    }

    generalizedForcesFinal = ctrl->get_outputs().generalized_forces;

    EXPECT_NEAR(generalizedForcesFinal(0), -param.generalized_forces_saturation(0), 1e-12);
    EXPECT_NEAR(generalizedForcesFinal(1), -param.generalized_forces_saturation(1), 1e-12);
    EXPECT_NEAR(generalizedForcesFinal(2), param.generalized_forces_saturation(2), 1e-12);
}

TEST_F(DpControllerUnitTests, T16_ClosedLoop_TestFeedFoward)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;

    inputs.measurements.north_east_heading = {12.0, 20.0, 0.5};
    inputs.dp_ref_manual.north_east_heading = {12.0, 20.0, 0.5};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};
    inputs.dp_ref_manual.north_east_heading_accel = {0.0, 0.0, 0.0};

    inputs.forces_ref_manual = {0.0, 0.0, 0.0};
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;
    inputs.desired_control_mode = ControlMode::JOYSTICK;

    DPParameters param;
    param.Kp = {0, 0, 0};
    param.Ki = {0, 0, 0};
    param.Kd = {0, 0, 0};

    param.mass_matrix << 6891.9, 0.0, 0.0, 0.0, 7059.1, 0.0, 0.0, 0.0, 3863.4;

    param.damping_matrix << 238.6, 0.0, 0.0, 0.0, 65.9, 0.0, 0.0, 0.0, 880.1;

    ctrl->configure(param);

    auto dt = 0.05s;
    Eigen::Vector3d tau0 = {0, 0, 0};
    auto model = std::make_unique<ThreeDOFModel>(inputs.measurements.north_east_heading,
                                                 inputs.dp_ref_manual.north_east_heading_vel);

    model->setM(param.mass_matrix);
    model->setD(param.damping_matrix);

    int nSteps = 3000;
    int indexChangeMode = 10;

    double timeConst = 2.0;
    std::unique_ptr<ReferenceFilter> refFilter =
        std::make_unique<ReferenceFilter>(timeConst, inputs.dp_ref_manual.north_east_heading, dt.count());
    Eigen::Vector3d eta_m = model->getNorthEastHeading();
    for (int i = 0; i < nSteps; i++) {
        if (i == indexChangeMode) {
            inputs.desired_control_mode = ControlMode::DP;
            inputs.dp_ref_manual.north_east_heading = {30.0, 30.0, 0.5};  // step change in setpoint
        }
        refFilter->step(inputs.dp_ref_manual.north_east_heading);
        inputs.dp_ref_manual.north_east_heading_vel = refFilter->get_velocities();
        inputs.dp_ref_manual.north_east_heading_accel = refFilter->get_acceleration();

        model->step(tau0, dt);
        eta_m = model->getNorthEastHeading();
        Eigen::Vector3d nu_m = model->getSurgeSwayYawRate();

        inputs.measurements.north_east_heading = eta_m;
        inputs.measurements.nu = nu_m;
        inputs.step_size = dt;

        ctrl->step(inputs);
        DPOutputs outputs = ctrl->get_outputs();
        tau0 = outputs.generalized_forces;

        //  auto ff = ctrl->get_feedforward_action();
    }

    EXPECT_NEAR(eta_m(0), 30.0, 1e-3);
    EXPECT_NEAR(eta_m(1), 30.0, 1e-3);
    EXPECT_NEAR(eta_m(2), 0.5, 1e-3);
}

TEST_F(DpControllerUnitTests, T17a_BumplessModeChange_AutonomyON2OFF)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;

    inputs.measurements.north_east_heading = {12.0, 20.0, 0.5};
    inputs.dp_ref_manual.north_east_heading = {12.0, 20.0, 0.5};
    inputs.dp_ref_manual.north_east_heading_vel = {1.0, 2.0, 3.0};
    inputs.dp_ref_manual.north_east_heading_accel = {4.0, 5.0, 6.0};

    inputs.dp_ref_autonomy.north_east_heading = {13.0, 21.0, 0.6};
    inputs.dp_ref_autonomy.north_east_heading_vel = {2.0, 3.0, 4.0};
    inputs.dp_ref_autonomy.north_east_heading_accel = {5.0, 6.0, 7.0};

    inputs.forces_ref_manual = {0.0, 0.0, 0.0};
    inputs.enable_autonomy_mode = true;
    inputs.enable_ready_for_autonomy = true;
    inputs.step_size = 0.05s;
    inputs.desired_control_mode = ControlMode::DP;

    ctrl->step(inputs);
    DPOutputs outputs = ctrl->get_outputs();
    EXPECT_EQ(outputs.dp_ref_active, inputs.dp_ref_autonomy);

    inputs.enable_autonomy_mode = false;
    ctrl->step(inputs);
    DPOutputs outputs2 = ctrl->get_outputs();
    EXPECT_EQ(outputs2.dp_ref_active,
              outputs.dp_ref_active);  // expect no change in active ref until refDpManual has changed

    inputs.dp_ref_manual.north_east_heading = {12.2, 20.2, 0.55};
    ctrl->step(inputs);
    DPOutputs outputs3 = ctrl->get_outputs();
    EXPECT_EQ(outputs3.dp_ref_active, inputs.dp_ref_manual);

    inputs.enable_autonomy_mode = true;
    ctrl->step(inputs);
    DPOutputs outputs4 = ctrl->get_outputs();
    EXPECT_EQ(outputs4.dp_ref_active,
              inputs.dp_ref_autonomy);  // expect active refence to output autonomy ref immediately

    inputs.enable_autonomy_mode = false;
    inputs.dp_ref_manual.north_east_heading = {22.2, 30.2, 1.55};
    ctrl->step(inputs);
    DPOutputs outputs5 = ctrl->get_outputs();
    EXPECT_EQ(outputs5.dp_ref_active, inputs.dp_ref_manual);  // it's possible to change mode and setpoint at same time

    inputs.desired_control_mode = ControlMode::AUTOPILOT;
    inputs.autopilot_inputs_manual.surge_accel_ref = 13;
    inputs.autopilot_inputs_autonomy.surge_accel_ref = 17;
    inputs.enable_autonomy_mode = false;
    ctrl->step(inputs);
    DPOutputs outputs6 = ctrl->get_outputs();
    EXPECT_EQ(outputs6.active_autopilot_inputs, inputs.autopilot_inputs_manual);

    inputs.enable_autonomy_mode = true;
    ctrl->step(inputs);
    DPOutputs outputs7 = ctrl->get_outputs();
    EXPECT_EQ(outputs7.active_autopilot_inputs, inputs.autopilot_inputs_autonomy);
}

TEST_F(DpControllerUnitTests, T17b_BumplessModeChange_AutonomyON2OFF_AutopilotToDP)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;

    inputs.measurements.north_east_heading = {15.0, 22.0, 0.54};

    inputs.dp_ref_manual.north_east_heading = {12.0, 20.0, 0.5};
    inputs.dp_ref_manual.north_east_heading_vel = {1.0, 2.0, 3.0};
    inputs.dp_ref_manual.north_east_heading_accel = {4.0, 5.0, 6.0};

    inputs.dp_ref_autonomy.north_east_heading = {13.0, 21.0, 0.6};
    inputs.dp_ref_autonomy.north_east_heading_vel = {2.0, 3.0, 4.0};
    inputs.dp_ref_autonomy.north_east_heading_accel = {5.0, 6.0, 7.0};

    
    inputs.enable_autonomy_mode = true;
    inputs.enable_ready_for_autonomy = true;
    inputs.step_size = 0.05s;
    inputs.desired_control_mode = ControlMode::AUTOPILOT;

    ctrl->step(inputs);        

    inputs.enable_autonomy_mode = false;
    inputs.desired_control_mode = ControlMode::DP;
    ctrl->step(inputs);
    DPOutputs outputs2 = ctrl->get_outputs();

    // when disabling autonomy and changing to DP mode at the same time, set DP setpoints to current measurement
    EXPECT_EQ(outputs2.dp_ref_active.north_east_heading, inputs.measurements.north_east_heading);    
}

TEST_F(DpControllerUnitTests, T18_SetAutopilotControlMode_NoException)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.forces_ref_autonomy = {0.5, 0.6, 0};
    inputs.forces_ref_manual = {0.33, 0.44, 0.1};
    inputs.desired_control_mode = ControlMode::AUTOPILOT;
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;

    inputs.autopilot_inputs_manual.heading_ref = 0.0;
    inputs.autopilot_inputs_manual.surge_speed_ref = 1.0;
    inputs.autopilot_inputs_manual.surge_accel_ref = 0.0;

    inputs.autopilot_inputs_manual.yaw_rate_ref = 0.0;
    inputs.autopilot_inputs_manual.yaw_accel_ref = 0.0;

    ctrl->step(inputs);
    // auto outputs = ctrl->get_active_config();
}

TEST_F(DpControllerUnitTests, T19_autopilot_autonomyOff_feedforward_only_expect_convergence_head_control)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.dp_ref_manual.north_east_heading = {0, 0, 0.0};
    inputs.measurements.north_east_heading = {0, 0, 0.0};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};

    inputs.forces_ref_manual = {0.0, 0.0, 0.0};
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;
    inputs.desired_control_mode = ControlMode::JOYSTICK;

    DPParameters param;

    param.autopilot.pid_heading.Kp = 0;
    param.autopilot.pid_heading.Ki = 0;
    param.autopilot.pid_heading.Kd = 0;
    param.autopilot.pid_surge.Kp = 0;
    param.autopilot.pid_surge.Ki = 0;
    param.autopilot.pid_surge.Kd = 0;

    param.mass_matrix << 6891.9, 0.0, 0.0, 0.0, 7059.1, 0.0, 0.0, 0.0, 3863.4;

    param.damping_matrix << 238.6, 0.0, 0.0, 0.0, 65.9, 0.0, 0.0, 0.0, 880.1;

    ctrl->configure(param);

    auto dt = 0.05s;
    Eigen::Vector3d tau0 = {0, 0, 0};
    auto model = std::make_unique<ThreeDOFModel>(inputs.measurements.north_east_heading,
                                                 inputs.dp_ref_manual.north_east_heading_vel);

    int nSteps = 3000;
    int indexChangeMode = 10;

    double timeConst = 10.0;
    std::unique_ptr<ReferenceFilter> ref_filter =
        std::make_unique<ReferenceFilter>(timeConst, inputs.dp_ref_manual.north_east_heading, dt.count());
    Eigen::Vector3d eta_m = model->getNorthEastHeading();
    double heading_sp = 0.5;
    uint8_t head_index = 2;
    Eigen::Vector3d u_filter = {0, 0, 0};
    for (int i = 0; i < nSteps; i++) {
        if (i == indexChangeMode) {
            inputs.desired_control_mode = ControlMode::AUTOPILOT;
            u_filter = {0, 0, heading_sp};  // step change in setpoint
        }
        ref_filter->step(u_filter);

        auto pos = ref_filter->get_positions();
        auto vel = ref_filter->get_velocities();
        auto accel = ref_filter->get_acceleration();

        inputs.autopilot_inputs_manual.heading_ref = pos(head_index);
        inputs.autopilot_inputs_manual.yaw_rate_ref = vel(head_index);
        inputs.autopilot_inputs_manual.yaw_accel_ref = accel(head_index);

        model->step(tau0, dt);
        eta_m = model->getNorthEastHeading();
        Eigen::Vector3d nu_m = model->getSurgeSwayYawRate();

        inputs.measurements.north_east_heading = eta_m;
        inputs.measurements.nu = nu_m;
        inputs.step_size = dt;

        ctrl->step(inputs);
        DPOutputs outputs = ctrl->get_outputs();
        tau0 = outputs.generalized_forces;
    }
    double final_heading = eta_m(head_index);
    EXPECT_NEAR(heading_sp, final_heading, 0.1);
}

TEST_F(DpControllerUnitTests, T20a_autopilot_autonomyOff_enviroment_disturbance_expect_convergence_head_control)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.dp_ref_manual.north_east_heading = {0, 0, 0.0};
    inputs.measurements.north_east_heading = {0, 0, 0.0};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};

    inputs.forces_ref_manual = {0.0, 0.0, 0.0};
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;
    inputs.desired_control_mode = ControlMode::JOYSTICK;

    DPParameters param;

    double z = 0.8;
    double wb = 0.5;  // rad/s
    double wn = 1 / sqrt(1 - 2 * z * z + sqrt(4 * z * z * z * z - 4 * z * z + 2)) * wb;

    double m = 3863.4;
    double T = 50;

    double Kp = m * wn * wn;
    double Ki = wn * Kp / 10;
    double Kd = m * (2 * z * wn - 1 / T);

    param.autopilot.pid_heading.Kp = Kp;
    param.autopilot.pid_heading.Ki = Ki;
    param.autopilot.pid_heading.Kd = Kd;

    param.mass_matrix << 6891.9, 0.0, 0.0, 0.0, 7059.1, 0.0, 0.0, 0.0, 3863.4;

    param.damping_matrix << 238.6, 0.0, 0.0, 0.0, 65.9, 0.0, 0.0, 0.0, 880.1;

    ctrl->configure(param);

    auto dt = 0.05s;
    Eigen::Vector3d tau0 = {0, 0, 0};
    auto model = std::make_unique<ThreeDOFModel>(inputs.measurements.north_east_heading,
                                                 inputs.dp_ref_manual.north_east_heading_vel);

    int nSteps = 3000;
    int indexChangeMode = 10;

    double timeConst = 10.0;
    std::unique_ptr<ReferenceFilter> ref_filter =
        std::make_unique<ReferenceFilter>(timeConst, inputs.dp_ref_manual.north_east_heading, dt.count());
    Eigen::Vector3d eta_m = model->getNorthEastHeading();
    double heading_sp = 0.5;
    uint8_t head_index = 2;
    Eigen::Vector3d u_filter = {0, 0, 0};
    Eigen::Vector3d tau_disturbance = {0, 0, 0};
    for (int i = 0; i < nSteps; i++) {
        if (i == indexChangeMode) {
            inputs.desired_control_mode = ControlMode::AUTOPILOT;
            u_filter = {0, 0, heading_sp};  // step change in setpoint
            tau_disturbance = {0, 0, 20};
        }
        ref_filter->step(u_filter);

        auto pos = ref_filter->get_positions();
        auto vel = ref_filter->get_velocities();
        auto accel = ref_filter->get_acceleration();

        inputs.autopilot_inputs_manual.heading_ref = pos(head_index);
        inputs.autopilot_inputs_manual.yaw_rate_ref = vel(head_index);
        inputs.autopilot_inputs_manual.yaw_accel_ref = accel(head_index);
        inputs.autopilot_inputs_autonomy.surge_integrator_mode = DPMock::IntegrationMode::FREEZE;
        inputs.autopilot_inputs_autonomy.heading_integrator_mode = DPMock::IntegrationMode::ENABLE;

        model->step(tau0 + tau_disturbance, dt);
        eta_m = model->getNorthEastHeading();
        Eigen::Vector3d nu_m = model->getSurgeSwayYawRate();

        inputs.measurements.north_east_heading = eta_m;
        inputs.measurements.nu = nu_m;
        inputs.step_size = dt;

        ctrl->step(inputs);
        DPOutputs outputs = ctrl->get_outputs();
        tau0 = outputs.generalized_forces;
    }
    double final_heading = eta_m(head_index);
    EXPECT_NEAR(heading_sp, final_heading, 0.01);
}

TEST_F(DpControllerUnitTests, T20b_autopilot_autonomyOff_enviroment_disturbance_expect_nonconvergence_head_control)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.dp_ref_manual.north_east_heading = {0, 0, 0.0};
    inputs.measurements.north_east_heading = {0, 0, 0.0};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};

    inputs.forces_ref_manual = {0.0, 0.0, 0.0};
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;
    inputs.desired_control_mode = ControlMode::JOYSTICK;

    DPParameters param;

    double z = 0.8;
    double wb = 0.5;  // rad/s
    double wn = 1 / sqrt(1 - 2 * z * z + sqrt(4 * z * z * z * z - 4 * z * z + 2)) * wb;

    double m = 3863.4;
    double T = 50;

    double Kp = m * wn * wn;
    double Ki = wn * Kp / 10;
    double Kd = m * (2 * z * wn - 1 / T);

    param.autopilot.pid_heading.Kp = Kp;
    param.autopilot.pid_heading.Ki = Ki;
    param.autopilot.pid_heading.Kd = Kd;

    param.mass_matrix << 6891.9, 0.0, 0.0, 0.0, 7059.1, 0.0, 0.0, 0.0, 3863.4;

    param.damping_matrix << 238.6, 0.0, 0.0, 0.0, 65.9, 0.0, 0.0, 0.0, 880.1;

    ctrl->configure(param);

    auto dt = 0.05s;
    Eigen::Vector3d tau0 = {0, 0, 0};
    auto model = std::make_unique<ThreeDOFModel>(inputs.measurements.north_east_heading,
                                                 inputs.dp_ref_manual.north_east_heading_vel);

    int nSteps = 3000;
    int indexChangeMode = 10;

    double timeConst = 10.0;
    std::unique_ptr<ReferenceFilter> ref_filter =
        std::make_unique<ReferenceFilter>(timeConst, inputs.dp_ref_manual.north_east_heading, dt.count());
    Eigen::Vector3d eta_m = model->getNorthEastHeading();
    double heading_sp = 0.5;
    uint8_t head_index = 2;
    Eigen::Vector3d u_filter = {0, 0, 0};
    Eigen::Vector3d tau_disturbance = {0, 0, 0};
    for (int i = 0; i < nSteps; i++) {
        if (i == indexChangeMode) {
            inputs.desired_control_mode = ControlMode::AUTOPILOT;
            u_filter = {0, 0, heading_sp};  // step change in setpoint
            tau_disturbance = {0, 0, 20};
        }
        ref_filter->step(u_filter);

        auto pos = ref_filter->get_positions();
        auto vel = ref_filter->get_velocities();
        auto accel = ref_filter->get_acceleration();

        inputs.autopilot_inputs_manual.heading_ref = pos(head_index);
        inputs.autopilot_inputs_manual.yaw_rate_ref = vel(head_index);
        inputs.autopilot_inputs_manual.yaw_accel_ref = accel(head_index);
        inputs.autopilot_inputs_manual.surge_integrator_mode = DPMock::IntegrationMode::FREEZE;
        inputs.autopilot_inputs_manual.heading_integrator_mode = DPMock::IntegrationMode::FREEZE;

        model->step(tau0 + tau_disturbance, dt);
        eta_m = model->getNorthEastHeading();
        Eigen::Vector3d nu_m = model->getSurgeSwayYawRate();

        inputs.measurements.north_east_heading = eta_m;
        inputs.measurements.nu = nu_m;
        inputs.step_size = dt;

        ctrl->step(inputs);
        DPOutputs outputs = ctrl->get_outputs();
        tau0 = outputs.generalized_forces;
    }
    double final_heading = eta_m(head_index);
    EXPECT_GT(final_heading, heading_sp);
}
TEST_F(DpControllerUnitTests, T21_autopilot_autonomyOff_feedforward_only_expect_convergence_surge_speed)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.dp_ref_manual.north_east_heading = {0, 0, 0.0};
    inputs.measurements.north_east_heading = {0, 0, 0.0};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};

    inputs.forces_ref_manual = {0.0, 0.0, 0.0};
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;
    inputs.desired_control_mode = ControlMode::JOYSTICK;

    DPParameters param;

    param.autopilot.pid_heading.Kp = 0;
    param.autopilot.pid_heading.Ki = 0;
    param.autopilot.pid_heading.Kd = 0;
    param.autopilot.pid_surge.Kp = 0;
    param.autopilot.pid_surge.Ki = 0;
    param.autopilot.pid_surge.Kd = 0;

    param.mass_matrix << 6891.9, 0.0, 0.0, 0.0, 7059.1, 0.0, 0.0, 0.0, 3863.4;

    param.damping_matrix << 238.6, 0.0, 0.0, 0.0, 65.9, 0.0, 0.0, 0.0, 880.1;

    ctrl->configure(param);

    auto dt = 0.05s;
    Eigen::Vector3d tau0 = {0, 0, 0};
    auto model = std::make_unique<ThreeDOFModel>(inputs.measurements.north_east_heading,
                                                 inputs.dp_ref_manual.north_east_heading_vel);

    int nSteps = 3000;
    int indexChangeMode = 10;

    Eigen::Vector3d nf_0 = {0, 0, 0};
    double time_constant = 5;
    auto surge_ref_filter = std::make_unique<ThirdOrderFilter>(time_constant, nf_0, dt.count());

    Eigen::Vector3d eta_m = model->getNorthEastHeading();
    Eigen::Vector3d nu_m = model->getSurgeSwayYawRate();
    double surge_speed_sp = 3;
    uint8_t surge_idx = 0;
    double u_filter = 0;
    for (int i = 0; i < nSteps; i++) {
        if (i == indexChangeMode) {
            inputs.desired_control_mode = ControlMode::AUTOPILOT;
            u_filter = surge_speed_sp;
        }
        surge_ref_filter->step(u_filter);

        inputs.autopilot_inputs_manual.surge_speed_ref = surge_ref_filter->get_pos();
        inputs.autopilot_inputs_manual.surge_accel_ref = surge_ref_filter->get_velocity();

        model->step(tau0, dt);
        eta_m = model->getNorthEastHeading();
        nu_m = model->getSurgeSwayYawRate();

        inputs.measurements.north_east_heading = eta_m;
        inputs.measurements.nu = nu_m;
        inputs.step_size = dt;

        ctrl->step(inputs);
        DPOutputs outputs = ctrl->get_outputs();
        tau0 = outputs.generalized_forces;
    }
    double final_surge_speed = nu_m(surge_idx);
    EXPECT_NEAR(surge_speed_sp, final_surge_speed, 0.1);
}

TEST_F(DpControllerUnitTests, T22a_autopilot_autonomyOff_enviromental_disturbance_expect_convergence_surge_speed)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.dp_ref_manual.north_east_heading = {0, 0, 0.0};
    inputs.measurements.north_east_heading = {0, 0, 0.0};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};

    inputs.forces_ref_manual = {0.0, 0.0, 0.0};
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;
    inputs.desired_control_mode = ControlMode::JOYSTICK;

    DPParameters param;

    param.mass_matrix << 6891.9, 0.0, 0.0, 0.0, 7059.1, 0.0, 0.0, 0.0, 3863.4;

    param.damping_matrix << 238.6, 0.0, 0.0, 0.0, 65.9, 0.0, 0.0, 0.0, 880.1;

    double tau_I = param.mass_matrix(0, 0) / param.damping_matrix(0, 0);
    param.autopilot.pid_surge.Kp = param.mass_matrix(0, 0);
    param.autopilot.pid_surge.Ki = param.autopilot.pid_surge.Kp / tau_I;
    param.autopilot.pid_surge.Kd = 0;

    ctrl->configure(param);

    auto dt = 0.05s;
    Eigen::Vector3d tau_0 = {0, 0, 0};
    auto model = std::make_unique<ThreeDOFModel>(inputs.measurements.north_east_heading,
                                                 inputs.dp_ref_manual.north_east_heading_vel);

    int nSteps = 3000;
    int indexChangeMode = 10;

    Eigen::Vector3d nf_0 = {0, 0, 0};
    double time_constant = 5;
    auto surge_ref_filter = std::make_unique<ThirdOrderFilter>(time_constant, nf_0, dt.count());

    Eigen::Vector3d eta_m = model->getNorthEastHeading();
    Eigen::Vector3d nu_m = model->getSurgeSwayYawRate();
    double surge_speed_sp = 3;
    uint8_t surge_idx = 0;
    double u_filter = 0;

    Eigen::Vector3d tau_disturbance = {0, 0, 0};
    Eigen::Vector3d tau = {0, 0, 0};

    for (int i = 0; i < nSteps; i++) {
        if (i == indexChangeMode) {
            inputs.desired_control_mode = ControlMode::AUTOPILOT;
            u_filter = surge_speed_sp;

            tau_disturbance = {-800, 0, 0};
        }
        surge_ref_filter->step(u_filter);

        inputs.autopilot_inputs_manual.surge_speed_ref = surge_ref_filter->get_pos();
        inputs.autopilot_inputs_manual.surge_accel_ref = surge_ref_filter->get_velocity();
        inputs.autopilot_inputs_manual.surge_integrator_mode = DPMock::IntegrationMode::ENABLE;
        inputs.autopilot_inputs_manual.heading_integrator_mode = DPMock::IntegrationMode::FREEZE;

        tau = tau_0 + tau_disturbance;
        model->step(tau, dt);
        eta_m = model->getNorthEastHeading();
        nu_m = model->getSurgeSwayYawRate();

        inputs.measurements.north_east_heading = eta_m;
        inputs.measurements.nu = nu_m;
        inputs.step_size = dt;

        ctrl->step(inputs);
        DPOutputs outputs = ctrl->get_outputs();
        tau_0 = outputs.generalized_forces;
    }
    double final_surge_speed = nu_m(surge_idx);
    EXPECT_NEAR(surge_speed_sp, final_surge_speed, 0.01);
}
TEST_F(DpControllerUnitTests, T22b_autopilot_autonomyOff_enviromental_disturbance_expect_nonconvergence_surge_speed)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.dp_ref_manual.north_east_heading = {0, 0, 0.0};
    inputs.measurements.north_east_heading = {0, 0, 0.0};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};

    inputs.forces_ref_manual = {0.0, 0.0, 0.0};
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;
    inputs.desired_control_mode = ControlMode::JOYSTICK;

    DPParameters param;

    param.mass_matrix << 6891.9, 0.0, 0.0, 0.0, 7059.1, 0.0, 0.0, 0.0, 3863.4;

    param.damping_matrix << 238.6, 0.0, 0.0, 0.0, 65.9, 0.0, 0.0, 0.0, 880.1;

    double tau_I = param.mass_matrix(0, 0) / param.damping_matrix(0, 0);
    param.autopilot.pid_surge.Kp = param.mass_matrix(0, 0);
    param.autopilot.pid_surge.Ki = param.autopilot.pid_surge.Kp / tau_I;
    param.autopilot.pid_surge.Kd = 0;

    ctrl->configure(param);

    auto dt = 0.05s;
    Eigen::Vector3d tau_0 = {0, 0, 0};
    auto model = std::make_unique<ThreeDOFModel>(inputs.measurements.north_east_heading,
                                                 inputs.dp_ref_manual.north_east_heading_vel);

    int nSteps = 3000;
    int indexChangeMode = 10;

    Eigen::Vector3d nf_0 = {0, 0, 0};
    double time_constant = 5;
    auto surge_ref_filter = std::make_unique<ThirdOrderFilter>(time_constant, nf_0, dt.count());

    Eigen::Vector3d eta_m = model->getNorthEastHeading();
    Eigen::Vector3d nu_m = model->getSurgeSwayYawRate();
    double surge_speed_sp = 3;
    uint8_t surge_idx = 0;
    double u_filter = 0;

    Eigen::Vector3d tau_disturbance = {0, 0, 0};
    Eigen::Vector3d tau = {0, 0, 0};

    for (int i = 0; i < nSteps; i++) {
        if (i == indexChangeMode) {
            inputs.desired_control_mode = ControlMode::AUTOPILOT;
            u_filter = surge_speed_sp;

            tau_disturbance = {-800, 0, 0};
        }
        surge_ref_filter->step(u_filter);

        inputs.autopilot_inputs_manual.surge_speed_ref = surge_ref_filter->get_pos();
        inputs.autopilot_inputs_manual.surge_accel_ref = surge_ref_filter->get_velocity();
        inputs.autopilot_inputs_manual.surge_integrator_mode = DPMock::IntegrationMode::FREEZE;
        inputs.autopilot_inputs_manual.heading_integrator_mode = DPMock::IntegrationMode::FREEZE;

        tau = tau_0 + tau_disturbance;
        model->step(tau, dt);
        eta_m = model->getNorthEastHeading();
        nu_m = model->getSurgeSwayYawRate();

        inputs.measurements.north_east_heading = eta_m;
        inputs.measurements.nu = nu_m;
        inputs.step_size = dt;

        ctrl->step(inputs);
        DPOutputs outputs = ctrl->get_outputs();
        tau_0 = outputs.generalized_forces;
    }
    double final_surge_speed = nu_m(surge_idx);
    EXPECT_LT(final_surge_speed, surge_speed_sp);
}

TEST_F(DpControllerUnitTests, T23_verify_surge_speed_ctrl_saturation)
{
    auto ctrl = std::make_unique<DPController>();
    DPInputs inputs;
    inputs.dp_ref_manual.north_east_heading = {0, 0, 0.0};
    inputs.measurements.north_east_heading = {0, 0, 0.0};
    inputs.dp_ref_manual.north_east_heading_vel = {0.0, 0.0, 0.0};

    inputs.forces_ref_manual = {0.0, 0.0, 0.0};
    inputs.enable_autonomy_mode = false;
    inputs.enable_ready_for_autonomy = false;
    inputs.step_size = 0.05s;
    inputs.desired_control_mode = ControlMode::JOYSTICK;

    DPParameters param;

    param.mass_matrix << 6891.9, 0.0, 0.0, 0.0, 7059.1, 0.0, 0.0, 0.0, 3863.4;

    param.damping_matrix << 238.6, 0.0, 0.0, 0.0, 65.9, 0.0, 0.0, 0.0, 880.1;

    param.autopilot.pid_surge.Kp = 1000;
    param.autopilot.pid_surge.Ki = 1000;
    param.autopilot.pid_heading.Kp = 1000;
    param.autopilot.pid_heading.Ki = 1000;
    param.autopilot.pid_heading.Kd = 1000;

    param.generalized_forces_saturation = {10, 0, 20};
    param.integral_action_saturation = {20, 0, 30};

    ctrl->configure(param);

    int nSteps = 3000;
    int indexChangeMode = 10;

    for (int i = 0; i < nSteps; i++) {
        if (i == indexChangeMode) {
            inputs.desired_control_mode = ControlMode::AUTOPILOT;
        }

        inputs.autopilot_inputs_manual.surge_speed_ref = -10;
        inputs.autopilot_inputs_manual.surge_accel_ref = -5;

        inputs.autopilot_inputs_manual.heading_ref = -2;
        inputs.autopilot_inputs_manual.yaw_rate_ref = -1;
        inputs.autopilot_inputs_manual.yaw_accel_ref = -1;

        inputs.measurements.nu = {0, 0, 0};
        inputs.measurements.north_east_heading = {0, 0, 0};
        inputs.step_size = 0.05s;

        ctrl->step(inputs);
        // DPOutputs outputs = ctrl->get_outputs();
        // auto tau_0 = outputs.generalized_forces;
    }
    DPOutputs outputs = ctrl->get_outputs();
    auto tau = outputs.generalized_forces;
    EXPECT_EQ(tau(0), -param.generalized_forces_saturation(0));
    EXPECT_EQ(tau(2), -param.generalized_forces_saturation(2));

    auto head_integral = ctrl->get_heading_ctrl_integral_action();
    auto surge_integral = ctrl->get_surge_speed_ctrl_integral_action();

    EXPECT_EQ(surge_integral, -param.integral_action_saturation(0));
    EXPECT_EQ(head_integral, -param.integral_action_saturation(2));

    inputs.desired_control_mode = ControlMode::JOYSTICK;
    ctrl->step(inputs);

    for (int i = 0; i < nSteps; i++) {
        if (i == indexChangeMode) {
            inputs.desired_control_mode = ControlMode::AUTOPILOT;
        }

        inputs.autopilot_inputs_manual.surge_speed_ref = 10;
        inputs.autopilot_inputs_manual.surge_accel_ref = 5;

        inputs.autopilot_inputs_manual.heading_ref = 2;
        inputs.autopilot_inputs_manual.yaw_rate_ref = 1;
        inputs.autopilot_inputs_manual.yaw_accel_ref = 1;
        inputs.autopilot_inputs_manual.surge_integrator_mode = DPMock::IntegrationMode::ENABLE;
        inputs.autopilot_inputs_manual.heading_integrator_mode = DPMock::IntegrationMode::ENABLE;

        inputs.measurements.nu = {0, 0, 0};
        inputs.step_size = 0.05s;

        ctrl->step(inputs);
        // DPOutputs outputs = ctrl->get_outputs();
        // auto tau_0 = outputs.generalized_forces;
    }
    outputs = ctrl->get_outputs();
    tau = outputs.generalized_forces;
    EXPECT_EQ(tau(0), param.generalized_forces_saturation(0));
    EXPECT_EQ(tau(2), param.generalized_forces_saturation(2));

    head_integral = ctrl->get_heading_ctrl_integral_action();
    surge_integral = ctrl->get_surge_speed_ctrl_integral_action();

    EXPECT_EQ(surge_integral, param.integral_action_saturation(0));
    EXPECT_EQ(head_integral, param.integral_action_saturation(2));
}
