import numpy as np
from docking_algorithms.guidance import Guidance
from docking_algorithms.utils.types import Waypoint
from docking_algorithms.utils.transformations import path_tangential_to_NED
from docking_algorithms.utils.computations import compute_velocity_components, compute_distance_between, compute_direction_toward_point

# Initialize a Guidance instance for testing
def initialize_guidance(dt: float, init_pose: np.ndarray, init_velocity: np.ndarray):
    guidance = Guidance(dt=dt, init_pose=init_pose, init_velocity=init_velocity)
    print(f'waypoints: {guidance.waypoints[0].pose}, {guidance.waypoints[1].pose}, {guidance.waypoints[2].pose}')
    return guidance

# Test for _update_path_tangential_angle
def test_update_path_tangential_angle(guidance):
    guidance._update_path_tangential_angle()
    print("Updated path tangential angle (pi_p):", guidance.pi_p)

# Test for _update_cross_and_along_track_error
def test_update_cross_and_along_track_error(guidance, eta):
    eta = np.array([92.245,491.315])
    guidance._update_cross_and_along_track_error(eta)
    print("Along-track error (x_ep):", guidance.x_ep)
    print("Cross-track error (y_ep):", guidance.y_ep)

# Test for _update_crab_angle
def test_update_crab_angle(guidance, eta_dot):
    guidance._update_crab_angle(eta_dot)
    print("Crab angle:", guidance.crab_angle)

# Test for _compute_desired_LOS_course
def test_compute_desired_LOS_course(guidance):
    try:
        course = guidance._compute_desired_LOS_course()
        print("Desired LOS course (chi_d):", course)
    except ValueError as e:
        print("Error:", e)

# Test for _compute_desired_LOS_heading
def test_compute_desired_LOS_heading(guidance):
    try:
        heading = guidance._compute_desired_LOS_heading()
        print("Desired LOS heading (psi_d):", heading)
    except ValueError as e:
        print("Error:", e)

# Test for _compute_desired_pos
def test_compute_desired_pos(guidance, eta, velocity_ref, dt):
    try:
        desired_pos = guidance._compute_desired_pos(eta, velocity_ref, dt)
        print("Desired position in NED frame:", desired_pos)
    except ValueError as e:
        print("Error:", e)

# Test for is_within_circle_of_acceptance
def test_is_within_circle_of_acceptance(guidance, eta):
    in_acceptance = guidance.is_within_circle_of_acceptance(eta)
    print("Within circle of acceptance:", in_acceptance)

# Test for end_of_wp_list
def test_end_of_wp_list(guidance):
    end = guidance.end_of_wp_list()
    print("End of waypoint list:", end)

# Test for update_waypoints
def test_update_waypoints(guidance):
    print("Current waypoint index before:", guidance.current_wp_index)
    guidance.update_waypoints()
    print("Current waypoint index after:", guidance.current_wp_index)

# Test for compute_reference_states
def test_compute_reference_states(guidance, eta, eta_dot, dt):
    try:
        eta_ref, eta_dot_ref = guidance.compute_reference_states(eta, eta_dot, dt)
        print("Reference states (eta_ref):", eta_ref)
        print("Reference velocity components (eta_dot_ref):", eta_dot_ref)
    except Exception as e:
        print("Error:", e)


# Sample test inputs
eta = np.array([94.0, 488.0, 2.058088872974556])  # NED frame position and heading
eta_dot = np.array([0.7, 0.01, 0])      # Velocity in NED frame
dt = 0.1
velocity_ref = np.array([0.5, 0.2])
# Initialize a test guidance instance
guidance = initialize_guidance(dt=dt, init_pose=eta, init_velocity=eta_dot)


# Run tests
print("Testing _update_path_tangential_angle:")
test_update_path_tangential_angle(guidance)
# direction = compute_direction_toward_point(from_point=np.array([94., 488., 2.05808887]), to_point=np.array([90.48765513, 494.62812353, 3.05084265]))
# print(f'direction between wp3 and wp2: {direction}')

print("\nTesting _update_cross_and_along_track_error:")
test_update_cross_and_along_track_error(guidance, eta)

print("\nTesting _update_crab_angle:")
test_update_crab_angle(guidance, eta_dot)

print("\nTesting _compute_desired_LOS_course:")
test_compute_desired_LOS_course(guidance)

print("\nTesting _compute_desired_LOS_heading:")
test_compute_desired_LOS_heading(guidance)

print("\nTesting _compute_desired_pos:")
test_compute_desired_pos(guidance, eta, velocity_ref, dt)

print("\nTesting is_within_circle_of_acceptance:")
test_is_within_circle_of_acceptance(guidance, eta)

print("\nTesting end_of_wp_list:")
test_end_of_wp_list(guidance)

print("\nTesting update_waypoints:")
test_update_waypoints(guidance)

print("\nTesting compute_reference_states:")
test_compute_reference_states(guidance, eta, eta_dot, dt)
