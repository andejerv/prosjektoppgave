import numpy as np

def rotated_box_vertices(center_north, center_east, yaw_rad, length, width) -> np.ndarray:
    # Define box vertices in local coordinates (centered at origin)
    vertices = np.array([
        [-length/2, -width/2],  # Bottom-left
         [length/2, -width/2],  # Bottom-right
         [length/2, width/2],   # Top-right
         [-length/2, width/2]   # Top-left
    ])

    # Rotation matrix for yaw angle (90 degrees about Z-axis)
    R_yaw = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad)],
        [np.sin(yaw_rad), np.cos(yaw_rad)]
    ])

    R_z = np.array([
        [0, -1],
        [1, 0]
    ])

    R_x = np.array([
        [1, 0],
        [0, -1]
    ])

    # Apply yaw rotation
    rotated_vertices = (R_yaw @ vertices.T).T

    # Rotate 90 degrees about the z-axis
    rotated_vertices = (R_z @ rotated_vertices.T).T

    # Rotate 180 degrees about the new x-axis
    rotated_vertices = (R_x @ rotated_vertices.T).T

    # Translate the vertices to the given center position
    translated_vertices = rotated_vertices + np.array([center_east, center_north])

    # Close the polygon by repeating the first vertex
    translated_vertices = np.vstack([translated_vertices, translated_vertices[0, :]])
    return translated_vertices


# def compute_distance_to_segment(point, segment_start, segment_end) -> float:
#     segment_vec = segment_end - segment_start
#     point_vec= point - segment_start
    
#     # Project point_vec onto segment_vec
#     segment_length_sq = np.dot(segment_vec, segment_vec)
#     if segment_length_sq == 0:
#         return np.linalg.norm(point_vec)  # The segment is just a point
    
#     t = np.clip(np.dot(point_vec, segment_vec) / segment_length_sq, 0, 1)
#     closest_point = segment_start + t * segment_vec
    
#     return np.linalg.norm(point - closest_point)

def point_extension_of_line(start_point: np.ndarray, end_point:np.ndarray, d: float):
    """
    Find the point that is a distance d from end_point, along the line spanned by
    start_point and end_point
    """
    # Compute the direction vector
    N1 = start_point[0]
    E1 = start_point[1]

    N2 = end_point[0]
    E2 = end_point[1]

    direction = np.array([N2 - N1, E2 - E1])
    distance = np.linalg.norm(direction)
    
    if distance == 0:
        raise ValueError("Points are identical; cannot determine a direction.")
    
    unit_direction = direction / distance
    
    # Move both points by distance d in the same direction
    N2_new, E2_new = np.array([N2, E2]) + d * unit_direction

    extended_point = np.array([N2_new, E2_new])
    
    return extended_point

def compute_distance_between(pose1: np.ndarray, pose2: np.ndarray) -> float:
    distance_between = np.linalg.norm(pose1[:2]-pose2[:2])
    return distance_between

def compute_direction_toward_point(from_point: np.ndarray, to_point: np.ndarray) -> float:
    """
    Compute the angle between North and a vector pointing from from_point to to_point.
    The angle is in radians
    """
    delta_N = to_point[0] - from_point[0]
    delta_E = to_point[1] - from_point[1]

    theta = np.arctan2(delta_E, delta_N)
    return theta


def compute_velocity_components(speed: float, direction_rad: float) -> np.ndarray:
    """
    Decompose speed in a given direction into x and y components.
    The direction (angle) is in radians
    """
    speed_N = speed*np.cos(direction_rad)
    speed_E = speed*np.sin(direction_rad)
    yaw_rate = 0

    velocity = np.array([speed_N, speed_E, yaw_rate])
    return velocity