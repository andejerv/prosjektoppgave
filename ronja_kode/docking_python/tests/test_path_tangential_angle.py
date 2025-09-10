import numpy as np
import pytest
from docking_algorithms.utils.transformations import path_tangential_to_NED

# Function to compute the path tangential angle
def compute_path_tangential_angle(prev_wp, curr_wp):
    delta_x = curr_wp[0] - prev_wp[0]
    delta_y = curr_wp[1] - prev_wp[1]
    pi_p = np.arctan2(delta_y, delta_x)
    return pi_p

# Test cases: corners and side midpoints
test_positions = {
    "top-right corner": np.array([1.0, 1.0, 0.0]),
    "top-left corner": np.array([1.0, -1.0, 0.0]),
    "bottom-right corner": np.array([-1.0, 1.0, 0.0]),
    "bottom-left corner": np.array([-1.0, -1.0, 0.0]),
    "middle-top": np.array([1.0, 0.0, 0.0]),
    "middle-bottom": np.array([-1.0, 0.0, 0.0]),
    "middle-right": np.array([0.0, 1.0, 0.0]),
    "middle-left": np.array([0.0, -1.0, 0.0])
}

@pytest.mark.parametrize("description, curr_wp, expected_angle", [
    ("top-right corner", np.array([1.0, 1.0, 0.0]), np.pi/4),       # 45 degrees
    ("top-left corner", np.array([1.0, -1.0, 0.0]), -np.pi/4),     # -45 degrees
    ("bottom-right corner", np.array([-1.0, 1.0, 0.0]), 3*np.pi/4), # 135 degrees
    ("bottom-left corner", np.array([-1.0, -1.0, 0.0]), -3*np.pi/4),# -135 degrees
    ("middle-top", np.array([1.0, 0.0, 0.0]), 0),                  # 0 degrees
    ("middle-bottom", np.array([-1.0, 0.0, 0.0]), np.pi),          # 180 degrees
    ("middle-right", np.array([0.0, 1.0, 0.0]), np.pi/2),          # 90 degrees
    ("middle-left", np.array([0.0, -1.0, 0.0]), -np.pi/2),         # -90 degrees
])
def test_compute_path_tangential_angle(description, curr_wp, expected_angle):
    prev_wp = np.array([0.0, 0.0, 0.0])  # Origin as prev_wp
    computed_angle = compute_path_tangential_angle(prev_wp, curr_wp)
    assert np.isclose(computed_angle, expected_angle, atol=1e-6), f"Failed on {description}: expected {expected_angle}, got {computed_angle}"
