import pytest
import numpy as np
from docking_algorithms.controllers.pid_controller import PIDController
from docking_algorithms.utils.types import PIDGains

def test_pid_controller_initialization():
    # Test initialization and setting of gains
    dt = 0.1
    kp = np.array([200, 200, 800])
    ki = np.array([10, 10, 15])
    kd = np.array([700, 700, 1600])
    pid_gains = PIDGains(kp, ki, kd)

    pid = PIDController(dt, pid_gains)
    
    assert np.array_equal(pid.kp, kp)
    assert np.array_equal(pid.ki, ki)
    assert np.array_equal(pid.kd, kd)


def test_pid_controller_set_gains():
    dt = 0.1    
    zero_n = np.zeros(3)
    pid_gains = PIDGains(zero_n, zero_n, zero_n)
    pid = PIDController(dt, pid_gains)

    kp = np.array([200, 200, 800])
    ki = np.array([10, 10, 15])
    kd = np.array([700, 700, 1600])

    updated_gains = PIDGains(kp, ki, kd)
    pid.set_pid_gains(updated_gains)
    
    assert np.array_equal(pid.kp, kp)
    assert np.array_equal(pid.ki, ki)
    assert np.array_equal(pid.kd, kd)



def test_pid_controller_update_control():
    dt = 0.1
    kp = np.array([200, 200, 800])
    ki = np.array([10, 10, 15])
    kd = np.array([700, 700, 1600])
    pid_gains = PIDGains(kp, ki, kd)

    pid = PIDController(dt, pid_gains)
    
    state_d = np.array([10, 0, 0])
    state_d_dot = np.array([1, 0, 0])
    state = np.array([0, 0, 0])
    state_dot = np.array([0.5, 0, 0])
    
    tau = pid.update_control(state_d, state_d_dot, state, state_dot)
    
    check_array = np.array([0, 0, 0])
    assert tau.shape == check_array.shape
    assert not np.any(np.isnan(tau))  # Ensure no NaN values
