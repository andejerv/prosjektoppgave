from docking_algorithms.utils.development_utils import move_points_along_line, compute_new_wp1_position
from docking_algorithms.utils.development_utils import compute_initial_velocity_components, compute_wp2_pose
from docking_algorithms.utils.computations import compute_distance_between, rotated_box_vertices
from docking_algorithms.utils.transformations import body_to_ned_transform
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass

# old_wp1 = np.array([87.5, 494.9])
# distance = 0.998
# quay_start = np.array([93.6113, 496.9546])
# quay_end = np.array([81.6607, 498.0421])
# new_wp1_pos = compute_new_wp1_position(old_wp1_pos=old_wp1, distance=distance,
#                                        quay_start=quay_start, quay_end=quay_end)
# print(f'new position of wp1 computed to be: {new_wp1_pos}')



# wp2 = np.array([96, 492])
# old_wp1 = np.array([87.5, 494.9])
# _, new_wp1 = move_points_along_line(point1=wp2, point2=old_wp1, d=0.98)
# print("Extended Point:", new_wp1)

def test_init_velocity_components():
    waypoint2 = np.array([100, 490])
    eta_0 = np.array([120, 480])
    theta, velocity_body = compute_initial_velocity_components(start_point=eta_0, to_point=waypoint2, speed=1.5, direction=None)
    return


def test_lines_parallel():
    # Example Usage
    # p1, p2 = (0, 0), (2, 2)  # Line 1

    # # q1, q2 = (1, 1), (3, 3)  # Line 2 (parallel)
    # q1, q2 = (1, 1), (3, 2)  # Line 2 (not parallel)

    p1, p2 = (81.6607, 498.0421), (93.6113, 496.9546) # quay line

    q1, q2 = p1, (96.59895522, 496.68272452) # extended line

    # Compute direction vectors
    v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
    v2 = np.array([q2[0] - q1[0], q2[1] - q1[1]])

    # Compute dot product and magnitudes
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)

    # Compute cosine of the angle
    cos_theta = dot_product / (norm_v1 * norm_v2)

    # Check if cos(theta) is close to ±1 (parallel)
    is_parallel =  np.isclose(abs(cos_theta), 1.0)
    return




# For testing compute_wp2_pose
# wp3 = np.array([87.5, 494.9, 3.05084265359])
# wp2 = compute_wp2_pose(wp3, 3)
# print(wp2)


# For testing compute_velocity_components_NED
# wp2 = np.array([0.0, 0.0, 0.0]) # NED
# wp3 = np.array([5.0, 0.0, 0.0]) # NED
# wp3_desired_speed = 0.1
# wp2_desired_speed = 0.25
# velocity = compute_velocity_components_NED(wp2, wp3, wp2_desired_speed)
# print(f'velocity components: {velocity}')

# wp2 = np.array([96, 492])
# wp1 = np.array([86.57,  495.1])
# d = compute_distance_between(pose1=wp2, pose2=wp1)
# print(f'distance between waypoints:{d}')
      
def test_distance_between():
    dock_pos = np.array([87.5, 494.9])
    wp1 = np.array([86.6, 495.2])
    distance = compute_distance_between(pose1=dock_pos, pose2=wp1)
    return


def test_angle_between_lines():
    """
    Compute the angle between two lines defined by points (p1, p2) and (p3, p4).
    
    Args:
    - p1, p2: Points defining the first line.
    - p3, p4: Points defining the second line.
    
    Returns:
    - angle in degrees between the two lines.
    """
    # Direction vectors of the lines
    p1 = np.array([87.5, 494.9]) 
    p2 = np.array([81.42510916718687, 495.45279269923486])
    p3 = np.array([80.9217, 489.9206])
    p4 = np.array([81.7373, 498.8836])
    vector1 = p2 - p1
    vector2 = p4 - p3
    
    # Dot product
    dot_product = np.dot(vector1, vector2)
    
    # Magnitudes (lengths) of the vectors
    mag_vector1 = np.linalg.norm(vector1)
    mag_vector2 = np.linalg.norm(vector2)
    
    # Compute the cosine of the angle
    cos_theta = dot_product / (mag_vector1 * mag_vector2)
    
    # Clip the cosine value to avoid domain errors due to floating point precision
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    
    # Compute the angle in radians, and convert it to degrees
    angle_rad = np.arccos(cos_theta)
    angle_deg = np.degrees(angle_rad)
    
    return angle_deg



def test_find_cone_center_line() -> tuple[np.ndarray, np.ndarray]:
    
    docking_point = np.array([87.5, 494.9]) 
    # head-on part of the quay
    head_on_quay_start = np.array([80.9217, 489.9206]) 
    head_on_quay_end = np.array([81.7373, 498.8836])    # "corner" of the whole quay structure
    
    quay_line = head_on_quay_end - head_on_quay_start
    start_to_docking_point = docking_point - head_on_quay_start

    # project
    projection = np.dot(start_to_docking_point, quay_line) / np.dot(quay_line, quay_line)

    docking_point_on_quay_line = head_on_quay_start + projection*quay_line

    center_line = docking_point - docking_point_on_quay_line
    angle = np.arctan2(quay_line[1]-center_line[1], quay_line[0]-center_line[0])

    # Plotting
    plt.figure(figsize=(8, 6))
    
    # Plot the quay line
    quay_x = [head_on_quay_start[0], head_on_quay_end[0]]
    quay_y = [head_on_quay_start[1], head_on_quay_end[1]]
    plt.plot(quay_y, quay_x, color='orange', label="Quay Line")

    # Plot the center line
    center_line_x = [docking_point[0], docking_point_on_quay_line[0]]
    center_line_y = [docking_point[1], docking_point_on_quay_line[1]]
    plt.plot(center_line_y, center_line_x, color='green', label="Center Line")
    
    # Plot the docking point and its projection on the quay line
    plt.scatter(docking_point[1], docking_point[0], color='red', label="Docking Point")
    plt.scatter(docking_point_on_quay_line[1], docking_point_on_quay_line[0], color='green', label="Projected Point")
    
    # Mark the start of the quay
    plt.scatter(head_on_quay_start[1], head_on_quay_start[0], color='orange', label="Quay Start")
    plt.scatter(head_on_quay_end[1], head_on_quay_end[0], color='orange', label="Quay End")
    
    # Labels and title
    plt.title("Docking Point and Projection on Quay Line")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.legend(loc="best")
    plt.grid(True)
    
    plt.show()

    return docking_point, docking_point_on_quay_line
    

def test_compute_unit_normal_vector() -> np.ndarray:
    # TODO: maybe use the heading reference and choose the normal vector that points to the opposite side of the line
    line_start = np.array([81.6607, 498.0421])
    line_end = np.array([93.6113, 496.9546])

    line_vector = line_end - line_start
    normal_vec = np.array([line_vector[1], -line_vector[0]])    # points away from the quay
    unit_normal_vec = normal_vec/np.linalg.norm(normal_vec)

    ref_vector = np.array([87.5, 494.9]) - line_start
    dot_product = np.dot(ref_vector, unit_normal_vec)

    # Check the dot product to determine the correct normal direction
    unit_normal_vec = unit_normal_vec if dot_product > 0 else -unit_normal_vec

    # Plot the line
    plt.figure(figsize=(6, 6))
    plt.plot([line_start[1], line_end[1]], [line_start[0], line_end[0]], 'bo-', label="Line Segment")  # Line with endpoints

    # Plot the normal vector at the midpoint of the line
    midpoint = (line_start + line_end) / 2
    plt.quiver(midpoint[1], midpoint[0], unit_normal_vec[1], unit_normal_vec[0], 
               angles='xy', scale_units='xy', scale=1, color='r', label="Unit Normal Vector")

    # Labeling
    plt.scatter(line_start[1], line_start[0], color='blue', label="Start Point")
    plt.scatter(line_end[1], line_end[0], color='blue', label="End Point")
    plt.text(line_start[1], line_start[0], " Start", fontsize=12, verticalalignment='bottom')
    plt.text(line_end[1], line_end[0], " End", fontsize=12, verticalalignment='bottom')

    # plot computed line to see if offset if correct
    a1, a2 = unit_normal_vec  # Normal vector components (example)
    x0, y0 = 81.7373, 498.8836  # Point on the line (example)
    b = a1 * x0 + a2 * y0  # Calculate b using the point
    x_vals = np.linspace(80, 83, 400)
    y_vals = (b - a1 * x_vals) / a2
    plt.plot(y_vals, x_vals, color='green', label=f'Line from normal vector')


    plt.axis("equal")
    plt.axhline(0, color='black', linewidth=0.5)
    plt.axvline(0, color='black', linewidth=0.5)
    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.legend()
    plt.title("Line and Unit Normal Vector")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.show()

    return unit_normal_vec


def test_parallel_move_line():
    line_start = np.array([81.7373, 498.8836])
    line_end = np.array([80.9217, 489.9206])
    ref_point = np.array([87.5, 494.9])

    line_vector = line_end - line_start
    normal_vec = np.array([line_vector[1], -line_vector[0]])    # points away from the quay
    unit_normal_vec = normal_vec/np.linalg.norm(normal_vec)

    ref_vector = np.array([87.5, 494.9]) - line_start
    dot_product = np.dot(ref_vector, unit_normal_vec)

    # Check the dot product to determine the correct normal direction
    unit_normal_vec = unit_normal_vec if dot_product > 0 else -unit_normal_vec
    displacement = (np.dot(ref_point - line_end, unit_normal_vec) / np.dot(unit_normal_vec, unit_normal_vec)) * unit_normal_vec

    # Move line by the displacement vector
    new_start = line_start + displacement
    new_end = line_end + displacement
    

    plt.figure(figsize=(6,6))
    
    # Plot original line
    plt.plot([line_start[1], line_end[1]], [line_start[0], line_end[0]], 'r--', label="Original Line")
    
    # Plot moved line
    plt.plot([new_start[1], new_end[1]], [new_start[0], new_end[0]], 'b-', label="Moved Line")
    
    # Plot points
    plt.scatter(line_start[1], line_start[0], color='red', label="Original Start", zorder=3)
    plt.scatter(line_end[1], line_end[0], color='red', label="Original End", zorder=3)
    plt.scatter(new_start[1], new_start[0], color='blue', label="New Start", zorder=3)
    plt.scatter(new_end[1], new_end[0], color='blue', label="New End", zorder=3)
    plt.scatter(ref_point[1], ref_point[0], color='green', label="Reference Point", zorder=3, marker='x', s=100)

    plt.axis("equal")
    plt.axhline(0, color='black', linewidth=0.5)
    plt.axvline(0, color='black', linewidth=0.5)
    plt.legend()
    plt.grid()
    plt.title("Parallel Line Translation")
    plt.show()

    # Print results
    print("Original Start:", line_start)
    print("Original End:", line_end)
    print("New Start:", new_start)
    print("New End:", new_end)
    print("Unit Normal Vector:", unit_normal_vec)



def vessel_corners_ned(vessel_pose: np.ndarray) -> float:
    """
    Compute cone angle, such that all four corners of the vessel is within the lines
    spanned by \pm cone_angle, when the lines span from docking_point to the corner
    or hte vessel that is furhtest away from the docking_point
    """
    l = 12  # vessel length
    w = 5   # vessel width

    dx = l/2
    dy = w/2

    vessel_corners_body = np.array([
        [dx, dy], [-dx, dy], [-dx, -dy], [dx, -dy]  # (front-starboard, front-port, back-port, back-starboard)
    ])

    R_B2N = body_to_ned_transform(psi=vessel_pose[2])[:2, :2]
    vessel_corners_ned = np.array([R_B2N @ corner + vessel_pose[:2] for corner in vessel_corners_body])
    return vessel_corners_ned


def test_point_extension_of_line():
    """
    Find the point that is a distance d from end_point, along the line spanned by
    start_point and end_point.
    Also plots the original and extended points with connecting lines.
    """ 
    start_point = np.array([100, 488])
    end_point = np.array([87.5, 494.9])
    d = 1
    # Extract coordinates
    N1, E1 = start_point
    N2, E2 = end_point

    # Compute the direction vector
    direction = np.array([N2 - N1, E2 - E1])
    distance = np.linalg.norm(direction)

    if distance == 0:
        raise ValueError("Points are identical; cannot determine a direction.")

    # Normalize the direction vector
    unit_direction = direction / distance

    # Compute the extended point
    extended_point = end_point + d * unit_direction

    # Plot the original points and extended point
    plt.figure(figsize=(6, 6))
    plt.plot([E1, E2], [N1, N2], 'bo-', label="Original Line")  # Line between start and end
    plt.plot([E2, extended_point[1]], [N2, extended_point[0]], 'ro--', label="Extended Line")  # Extension

    # Scatter plot for points
    plt.scatter(start_point[1], start_point[0], color='blue', label="Start Point")
    plt.scatter(end_point[1], end_point[0], color='green', label="End Point")
    plt.scatter(extended_point[1], extended_point[0], color='red', label="Extended Point")

    # Annotate points
    plt.text(E1, N1, " Start", fontsize=12, verticalalignment='bottom', color='blue')
    plt.text(E2, N2, " End", fontsize=12, verticalalignment='bottom', color='green')
    plt.text(extended_point[1], extended_point[0], " Extended", fontsize=12, verticalalignment='bottom', color='red')

    # Formatting
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.axhline(0, color='black', linewidth=0.5)
    plt.axvline(0, color='black', linewidth=0.5)
    plt.legend()
    plt.grid(True)
    plt.title("Point Extension Along a Line")
    
    # Show plot
    plt.show()

    return extended_point


def test_point_on_line():
    point = np.array([81.6607, 498.0421])

    # Normal vector
    # n = np.array([-0.09062516, -0.99588507])
    n = np.array([-0.08947649767167255, -0.9959889338564014])
    
    # Reference point on the line
    x0 = np.array([93.6113, 496.9546])

    # Compute line equation n^T (x - x0) = 0 → Solve for x along the line
    d = np.dot(n, x0)

    v = np.array([-n[1], n[0]])  # Perpendicular to n

    # Generate points along the line by using a parameter t
    t = np.linspace(-10, 10, 100)  # Range for extending the line
    line_points = x0.reshape(2,1) + t.reshape(1,-1) * v.reshape(2,1)

    # Plot the line
    plt.plot(line_points[1, :], line_points[0, :], 'b-', label="Line n^T x = d")

    # Plot normal vector from x0
    plt.quiver(x0[1], x0[0], n[1], n[0], angles='xy', scale_units='xy', scale=0.5, color='r', label="Normal vector n")

    # Plot x0 and test point
    plt.scatter(x0[1], x0[0], color='black', marker='o', label="x0 (Reference Point)")
    plt.scatter(point[1], point[0], color='green', marker='x', label="Test Point")

    # Compute the signed distance from the point to the line
    val = np.dot(n, point - x0)
    print(f"Signed distance from test point to line: {val}")

    # Plot settings
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.axis('equal')
    plt.grid()
    plt.show()

def test_find_docking_point():
    vessel_pose = np.array([87.5, 494.9, 3.05084265359])
    vessel_corners = rotated_box_vertices(center_north=vessel_pose[0],
                                          center_east=vessel_pose[1],
                                          yaw_rad=vessel_pose[2],
                                          length=12.25, width=5.25)
    
    # Remove duplicate last corner
    vessel_corners = vessel_corners[:-1, :]
    
    sidequay_start = np.array([81.6607, 498.0421])  # "corner"
    sidequay_end = np.array([93.6113, 496.9546])
    sidequay_n = np.array([-0.09062516, -0.99588507])   # quay normal vector

    distance = []
    for _, c in enumerate(vessel_corners):
        dist_to_line = np.dot(sidequay_n, c - sidequay_start)
        distance.append(dist_to_line)
    return

def test_perpendicular_distance():
    x0, y0 = 87.5, 494.9
    x1, y1 = 81.6607, 498.0421
    x2, y2 = 93.6113, 496.9546
    # Calculate the perpendicular distance
    numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    denominator = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    distance = numerator / denominator
    return distance


def test_orthogonality():
    sidequay_start = np.array([81.6607, 498.0421])  # "corner"
    sidequay_end = np.array([93.6113, 496.9546])
    sidequay_line = sidequay_end - sidequay_start

    frontquay_start = np.array([81.7373, 498.8836])  # "corner"
    frontquay_end = np.array([80.9217, 489.9206])
    frontquay_line = frontquay_end - frontquay_start

    val = np.dot(sidequay_line, frontquay_line)
    return



def test_find_point():
    """
    Finds a point that is at distance d1 from the first line and d2 from the second line.

    Parameters:
    d1, d2  : Distances from the respective lines
    p1, p2  : Two points defining the first line
    q1, q2  : Two points defining the second line
    n1, n2  : Unit normal vectors for the two lines

    Returns:
    (x, y)  : The coordinates of the required point
    """
    d1 = 5.25/2
    d2 = 12.25/2

    sidequay_start = np.array([81.6607, 498.0421])  # "corner"
    sidequay_end = np.array([93.6113, 496.9546])
    sidequay_dir = sidequay_end - sidequay_start

    frontquay_start = np.array([81.7373, 498.8836])  # "corner"
    frontquay_end = np.array([80.9217, 489.9206])
    frontquay_dir = frontquay_end - frontquay_start

    # Compute unit normal vectors (perpendicular to direction vectors)
    # n1 = np.array([-sidequay_dir[1], sidequay_dir[0]])  # Rotate 90 degrees
    # n1 /= np.linalg.norm(n1)  # Normalize

    # n2 = np.array([-frontquay_dir[1], frontquay_dir[0]])  # Rotate 90 degrees
    # n2 /= np.linalg.norm(n2)  # Normalize

    n1 = np.array([-0.09062516, -0.99588507])
    n2 = np.array([0.9958853703460269, -0.09062190204777651])

    # Find the intersection point (P0) using a linear system
    A = np.array([n1, n2])  # Normal vectors as coefficients
    C = np.array([np.dot(n1, sidequay_start), np.dot(n2, frontquay_start)])

    P0 = np.linalg.solve(A, C)  # Intersection point

    # Compute the required point
    P = P0 + d1 * n1 + d2 * n2
    return


def test_find_aligned_center():
    """
    Finds the center position (north, east) of a rectangle such that one of its sides aligns with a given line.
    
    Parameters:
        line_start (tuple): (north, east) coordinates of the line start point.
        line_end (tuple): (north, east) coordinates of the line end point.
        heading_rad (float): The heading angle (yaw) the vessel must have.
        length (float): Length of the vessel (rectangle).
        width (float): Width of the vessel (rectangle).
    
    Returns:
        tuple: (center_north, center_east) coordinates of the vessel's required center position.
    """
    line_start = np.array([81.6607, 498.0421])  # "corner"
    line_end = np.array([93.6113, 496.9546])

    yaw_rad = np.arctan2(line_end[1] - line_start[1], line_end[0] - line_start[0])
    length = 12.25
    width = 5.25
    # Compute the midpoint of the line segment
    line_midpoint = ((line_start[0] + line_end[0]) / 2, (line_start[1] + line_end[1]) / 2)
    
    # Compute unit direction vector of the line segment
    direction_vector = np.array([line_end[0] - line_start[0], line_end[1] - line_start[1]])
    direction_vector /= np.linalg.norm(direction_vector)  # Normalize

    # Compute the two possible normal unit vectors
    normal_vector_1 = np.array([-direction_vector[1], direction_vector[0]])  # Left-hand normal
    normal_vector_2 = np.array([direction_vector[1], -direction_vector[0]])  # Right-hand normal
    
    # Compute both possible center positions
    center_position_1 = np.array(line_midpoint) + (normal_vector_1 * (width / 2))
    center_position_2 = np.array(line_midpoint) + (normal_vector_2 * (width / 2))
    return



def test_plot_normal_line_vector():
    """
    Plots a 2D line using vector notation given a unit normal vector and a point on the line.

    Parameters:
    - n: A 2D NumPy array representing the unit normal vector.
    - P: A 2D NumPy array representing the point on the line.
    - x_range: Tuple (x_min, x_max) defining the x-axis range for plotting.
    """
    # Example usage
    n = np.array([1/np.sqrt(2), 1/np.sqrt(2)])  # Unit normal vector
    P = np.array([2, 3])  # Given point
    x_range=(-10, 10)

    # Compute d from the equation n . (r - P) = 0
    d = np.dot(n, P)  # This gives the constant term in the equation

    # Generate x values
    x_values = np.linspace(x_range[0], x_range[1], 400)

    # Solve for y using vector form: n . r = d → y = (d - ax) / b
    if n[1] != 0:
        y_values = (d - n[0] * x_values) / n[1]
    else:  # Vertical line case
        x_values = np.full_like(x_values, P[0])
        y_values = np.linspace(x_range[0], x_range[1], 400)

    # Plot the line
    plt.plot(x_values, y_values, label="Line: n . (r - P) = 0")

    # Plot the given point
    plt.scatter(P[0], P[1], color='red', s=100, label="Given Point")

    # Plot the normal vector
    plt.quiver(P[0], P[1], n[0], n[1], angles='xy', scale_units='xy', scale=1, color='blue', label="Normal Vector")

    # Labels and legend
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axhline(0, color='black', linewidth=0.5)
    plt.axvline(0, color='black', linewidth=0.5)
    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.axis("equal")
    plt.legend()
    plt.show()

def test_rotate_unit_normal():
    angle = np.deg2rad(-2) # rad
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                [np.sin(angle), np.cos(angle)]])
    old_normal = np.array([0.9958853703460269, -0.09062190204777651])
    new_normal = rotation_matrix@old_normal
    new_normal /=np.linalg.norm(new_normal)
    return

def test_plotting_color():
    x = np.linspace(0, 10, 100)
    y = x  # simple straight line (y = x)
    color = [0.85, 0.35, 0.0]
    plt.plot(x, y, linewidth=1.5, linestyle='-',
             color=color, label=f'color{color}')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid()
    plt.legend()
    plt.title('Straight Line Plot with Color-Blind-Friendly Orange')
    plt.show()