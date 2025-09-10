from docking_algorithms.utils.computations import compute_velocity_components, compute_direction_toward_point
from docking_algorithms.utils.transformations import body_to_ned_transform
import numpy as np


def compute_new_wp1_position(old_wp1_pos: np.ndarray, distance: float,
                             quay_start: np.ndarray, quay_end: np.ndarray) -> np.ndarray:
    """
    Purpose:
        - The new location of wp1 is computed such that a straight line going from old wp1 to new wp1
        is parallel to the quay.
        - New wp1 is located "distance" meters from old wp1, along the computed line
        - old_wp1_pos is the location at which the vessel is aligned completely towards the quay
    """
    # Compute the direction vector of the quay
    quay_vector = quay_end - quay_start
    
    # Compute the length of the quay vector
    quay_length = np.linalg.norm(quay_vector)
    if quay_length == 0:
        raise ValueError("Quay start and end points must be different.")
    
    # Normalize the quay vector to get a unit direction
    unit_vector = quay_vector / quay_length
    
    # Compute the displacement along the parallel direction
    displacement = unit_vector * distance
    
    # Compute the new wp1 position
    new_wp1_pos = old_wp1_pos + displacement
    
    return new_wp1_pos


def move_points_along_line(point1, point2, d):
    """
    Move both points (N1, E1) and (N2, E2) by distance d along the line they span.
    """
    # Compute the direction vector
    N1 = point1[0]
    E1 = point1[1]

    N2 = point2[0]
    E2 = point2[1]

    direction = np.array([N2 - N1, E2 - E1])
    distance = np.linalg.norm(direction)
    
    if distance == 0:
        raise ValueError("Points are identical; cannot determine a direction.")
    
    unit_direction = direction / distance
    
    # Move both points by distance d in the same direction
    N1_new, E1_new = np.array([N1, E1]) + d * unit_direction
    N2_new, E2_new = np.array([N2, E2]) + d * unit_direction

    point1_new = np.array([N1_new, E1_new])
    point2_new = np.array([N2_new, E2_new])
    
    return point1_new, point2_new


def compute_initial_velocity_components(
        start_point: np.ndarray, to_point: np.ndarray,
        speed: float, direction: float=None) -> tuple[float, np.ndarray]:
    if direction == None:
        theta = compute_direction_toward_point(from_point=start_point, to_point=to_point)
        velocity = compute_velocity_components(speed=speed, direction_rad=theta)
        print(f'direction of total speed is {theta}')
        print(f'NED velocity: {velocity}')
        velocity_body = body_to_ned_transform(theta).T @ velocity
        print(f'set initial velocity (BODY) in simulink to be {velocity_body}')
    else: # direction != None
        velocity = compute_velocity_components(speed=speed, direction_rad=direction)
        print(f'direction of total speed is {direction}')
        print(f'NED velocity: {velocity}')
        velocity_body = body_to_ned_transform(direction).T @ velocity
        print(f'set initial velocity (BODY) in simulink to be {velocity_body}')
        theta = 0.0 # placeholder
    return theta, velocity_body


def compute_wp2_pose(wp3: np.ndarray, distance_between=3):
    """
    Compute pose of wp, which has same heading as wp3 and is located
    at distance_between "in front of" wp3    """

    psi = wp3[2]
    x_wp2 = wp3[0] - distance_between*np.cos(psi)   # North position
    y_wp2 = wp3[1] -distance_between*np.sin(psi)    # East position
    wp2_pose = np.array([x_wp2, y_wp2, psi])

    return wp2_pose