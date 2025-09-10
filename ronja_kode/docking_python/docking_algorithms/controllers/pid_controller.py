import numpy as np
from docking_algorithms.utils.types import PIDGains
from docking_algorithms.utils.inf2pipi import inf2pipi

class PIDController:    # MIMO

    def __init__(self,  dt: float, pid_gains: PIDGains=None) -> None:
        self.kp = np.array([0, 0, 0]) if pid_gains is None else pid_gains.kp
        self.ki = np.array([0, 0, 0]) if pid_gains is None else pid_gains.ki
        self.kd = np.array([0, 0, 0]) if pid_gains is None else pid_gains.kd

        # Used for integral anti-windup TODO: tune and move to config file
        # self.control_saturation = WindupParams.control_saturation #np.array([150, 150, 200]) # saturation for surge, sway, yaw
        #self.tau_windup_increased = np.array([250, 250, 200])

        self.error_integral_state = np.zeros(3)

        self.dt = dt    # step size


    def reset_errors(self) -> None: 
        """
        Call when switching phases to reset integrator
        """
        self.error_integral_state = np.zeros(3)
        return
    

    def set_pid_gains(self, pid_gains: PIDGains) -> None:
        """
        Update the diagonal values of the kp, ki, kd matrices.
        Each of kp_vals, ki_vals, kd_vals should be a 1D array of length 3.
        """
        self.kp = pid_gains.kp
        self.ki = pid_gains.ki
        self.kd = pid_gains.kd
        return
    

    def update_control(self, state_d: np.ndarray, state_d_dot: np.ndarray, state: np.ndarray, state_dot: np.ndarray) -> np.ndarray:
        """
        Compute control forces based on PID control law
        """
        error_state = state - state_d
        psi_error = state[2] - state_d[2]
        error_state[2] = inf2pipi(ang=psi_error)  # smallest signed angle
        error_state_dot = state_dot - state_d_dot

        # error_integral_state_update = self.error_integral_state + error_state*self.dt
        self.error_integral_state_update = self.error_integral_state + error_state*self.dt

        P = np.multiply(self.kp, error_state)
        # I = np.multiply(self.ki, error_integral_state_update)
        I = np.multiply(self.ki, self.error_integral_state_update)
        D = np.multiply(self.kd, error_state_dot)
        control = - (P + I + D) # control forces unclamped

        # TODO: REMOVE IF NOT GOING TO USE
        # Apply control saturation (clamping)
        # control_clamped = np.clip(control, -self.control_saturation, self.control_saturation)

        # is_saturated = not np.array_equal(control, control_clamped)  # should be equal if control not saturated
        # is_error_aligned = np.sign(error_state) == np.sign(control)

        # if is_saturated and is_error_aligned.all():
        #    # clamping is active, do not update integrator state
        #     pass
        # else:
        #     self.error_integral_state = error_integral_state_update  # only update integrator if clamping not needed

        # return control_clamped
        return control