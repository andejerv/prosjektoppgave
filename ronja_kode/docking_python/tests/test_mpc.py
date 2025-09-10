import pytest
import numpy as np
from docking_algorithms.controllers.mpc_guidance import MPCGuidance
import matplotlib.pyplot as plt
from docking_algorithms.utils.transformations import body_to_ned_transform
from docking_algorithms.utils.computations import rotated_box_vertices
from docking_algorithms.utils.inf2pipi import inf2pipi
from generate_results.utils import unwrap_angle, plot_thick_quay

from casadi import *

class MockMPCConfig():
    def __init__(self):
        self.horizon = 1000
        self.sidequay_extension_length = 6.5
        self.frontquay_unit_normal = np.array([0.9958853703460269, -0.09062190204777651])
        self.sidequay_unit_normal = np.array([-0.09062516, -0.99588507])

        # self.ref_line_unit_normal = np.array([-0.09062516, -0.99588507])  # center line
        # self.ref_line_unit_normal = np.array([-0.17707418749501908, -0.984197506663667]) # diagonal line 5 deg
        # self.ref_line_unit_normal = np.array([-0.2617883, -0.96501913]) # diagonal line 10 deg
        self.ref_line_unit_normal = np.array([-0.42576959, -0.90483162]) # diagonal line 20 deg

class VesselConfig:
    def __init__(self):
        self.length = 12.25
        self.width = 5.25

class Segment:
    def __init__(self, corner, end):
        self.corner = np.array(corner)
        self.end = np.array(end)

class QuaysConfig:
    def __init__(self):
        self.side_segment = Segment([81.6607, 498.0421], [93.6113, 496.9546])
        self.front_segment = Segment([81.7373, 498.8836], [80.9217, 489.9206])

class QuaysVesselConfig:
    def __init__(self):
        self.vessel = VesselConfig()
        self.quays = QuaysConfig()


@pytest.fixture
def setup_mpc():
    """Fixture to initialize the MPC guidance with default parameters."""
    dt = 0.1  # Time step (s)
    mpc_cfg = MockMPCConfig()
    quays_vessel_cfg = QuaysVesselConfig()

    wp_ref = np.array([87.5, 494.9, 3.05084265359]) # Target waypoint (x, y, psi) 
    mpc = MPCGuidance(dt=dt, mpc_cfg=mpc_cfg,
                        quays_vessel_cfg=quays_vessel_cfg, waypoint_ref=wp_ref,
                        max_speed=1)

    return mpc

def test_simple_mpc_visualization(setup_mpc):
    """Visual test: Plot trajectory and velocity components to see MPC behavior."""
    state = np.array([0.0, 0.0, 0.34906585])  # Initial position and orientation
    state_dot = np.array([0.5, 0.01, 0.001])  # Initial velocity components
    accel_ned = np.array([0.0, 0.0, 0.0])  # No initial acceleration

    trajectory = [state]
    velocities = [state_dot]
    speeds = [np.linalg.norm(state_dot[:2])]
    
    setup_mpc.opti.set_value(setup_mpc.wp_ref, np.array([1, 6, 0.34906585]))
    target_wp = setup_mpc.opti.value(setup_mpc.wp_ref)

    # Run MPC until the vessel is within 0.2m of the target waypoint
    num_iter = 0
    while np.linalg.norm(state[:2] - target_wp[:2]) > 0.05:
    # for i in range(0,50):
        num_iter += 1
        state, state_dot = setup_mpc.solve_mpc(state, state_dot, accel_ned)
        trajectory.append(state)
        velocities.append(state_dot)
        speeds.append(np.linalg.norm(state_dot[:2]))

    trajectory = np.array(trajectory)
    velocities = np.array(velocities)
    speeds = np.array(speeds)    

    time_steps = np.arange(len(velocities))  # Time steps

    # Plot the trajectory with smaller markers
    plt.figure(figsize=(10, 5))
    plt.subplot(2, 2, 1)
    plt.plot(trajectory[:, 1], trajectory[:, 0], marker='o', markersize=2, label='MPC Path')
    plt.scatter(target_wp[1], target_wp[0], color='red', s=20, label='Target')  # Smaller scatter point
    plt.xlabel('East Position')
    plt.ylabel('North Position')
    plt.title(f'MPC Trajectory Visualization (#iterations: {num_iter})')
    plt.legend()
    plt.grid()
    plt.axis('equal')

    plt.subplot(2, 2, 2)
    plt.plot(time_steps, speeds, label='Total Linear Velocity', color='b')
    plt.xlabel('Time Step')
    plt.ylabel('Speed (m/s)')
    plt.title('Total Linear Velocity Over Time')
    plt.legend()
    plt.grid()

    plt.figure(figsize=(10, 5))
    plt.subplot(3, 1, 1)
    plt.plot(time_steps, velocities[:, 0], label='North Velocity (m/s)')
    plt.xlabel('Time Step')
    plt.ylabel('Velocity')
    plt.legend()
    plt.grid()

    plt.subplot(3, 1, 2)
    plt.plot(time_steps, velocities[:, 1], label='East Velocity (m/s)', color='orange')
    plt.xlabel('Time Step')
    plt.ylabel('Velocity')
    plt.legend()
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(time_steps, velocities[:, 2], label='Yaw Rate (rad/s)', color='green')
    plt.xlabel('Time Step')
    plt.ylabel('Yaw Rate')
    plt.legend()
    plt.grid()

    plt.suptitle('Velocity Components Over Time')
    plt.tight_layout()
    plt.show()


def test_full_mpc_visualization(setup_mpc):
    """Visual test: Plot trajectory and velocity components, including line constraints."""
    state = np.array([120, 500, 3.84])  # Initial position and orientation [105, 480, 2.4] [90, 510, 5.23]  [115.0, 510.0, 4.36332313]  [105, 480, 8.68318530718] [130.0, 500.0, 3.84]
    state[2] = inf2pipi(ang=state[2])
    body_vel = np.array([1, 0, 0])
    R = body_to_ned_transform(psi=state[2])
    state_dot = R@body_vel  # Initial velocity components

    speed = np.linalg.norm(state_dot[:2])
    max_speed = min(speed, 2)   # 2 m/s is absolute max speed
    setup_mpc.max_speed = max_speed # not really necessary, but nice for consistency
    setup_mpc.speed_max_squared = max_speed**2

    wp_ref = np.array([87.5, 494.9, 3.05084265359])
    wp_ref[2] = inf2pipi(ang=wp_ref[2])
    relative_heading = inf2pipi(ang=(state[2] - wp_ref[2]))  # Adjust heading to be relative to wp_ref[2]
    state = np.array([state[0], state[1], relative_heading])

    nan_vec = np.array([np.nan, np.nan])

    trajectory = [state]
    velocities = [state_dot]
    speeds = [np.linalg.norm(state_dot[:2])]
    frontquay_unit_normal_vectors = [nan_vec]
    frontquay_points_on_line = [nan_vec]
    sidequay_unit_normal_vectors = [nan_vec]
    sidequay_points_on_line = [nan_vec]    

    mpc_wp_ref = np.array([wp_ref[0], wp_ref[1], 0])
    setup_mpc.opti.set_value(setup_mpc.wp_ref, mpc_wp_ref) # [87.5, 494.9, 0]
    target_wp = setup_mpc.opti.value(setup_mpc.wp_ref)

    a = 2

    # Run MPC until the vessel is within 0.2m of the target waypoint
    i = 0
    while np.linalg.norm(state[:2] - target_wp[:2]) > 0.05:
    # for i in range(0, 500):

        # ETA = np.linalg.norm(state[:2]-wp_ref[:2])/np.linalg.norm(state_dot[:2])
        # N = int(np.ceil(ETA/0.1))   # dt = 0.1
        # setup_mpc.opti.set_value(setup_mpc.N, N)

        optimal_trajectories = setup_mpc.solve_mpc(t=i, state=trajectory[i], state_dot=velocities[i])
        # state_array, state_dot_array = setup_mpc.solve_mpc(state=trajectory[i], state_dot=velocities[i])
        state = optimal_trajectories.state[1]
        state_dot = optimal_trajectories.state_dot[0]
        trajectory.append(state)
        velocities.append(state_dot)
        frontquay_unit_normal_vectors.append(optimal_trajectories.frontquay_line_normals[0])    # static
        frontquay_points_on_line.append(optimal_trajectories.frontquay_line_points[0])  # static
        sidequay_unit_normal_vectors.append(optimal_trajectories.sidequay_line_normals[0])  # dynamic for every optimization iteration
        sidequay_points_on_line.append(optimal_trajectories.sidequay_line_points[0])    # static
        
        speeds.append(np.linalg.norm(state_dot[:2]))
        i += 1

    trajectory = np.array(trajectory)
    velocities = np.array(velocities)
    speeds = np.array(speeds)
    frontquay_normal_vectors = np.array(frontquay_unit_normal_vectors)
    frontquay_points = np.array(frontquay_points_on_line)
    sidequay_normal_vectors = np.array(sidequay_unit_normal_vectors)
    sidequay_points = np.array(sidequay_points_on_line)
    time_steps = np.arange(len(velocities))  # Time steps

    ### Extract quay line constraints ###
    quay_side_start = setup_mpc.opti.value(setup_mpc.sidequay_start)
    quay_side_end = setup_mpc.opti.value(setup_mpc.sidequay_end)
    quay_front_start = setup_mpc.opti.value(setup_mpc.frontquay_start)
    quay_front_end = setup_mpc.opti.value(setup_mpc.frontquay_end)
    a = 2

    # Plot trajectory and quay constraints
    plt.figure(figsize=(10, 5))
    
    # Plot MPC Path
    plt.plot(trajectory[:, 1], trajectory[:, 0], marker='o', markersize=2, label='MPC Path')
    plt.scatter(target_wp[1], target_wp[0], color='red', s=20, label='Target')  # Smaller scatter point

    # Plot quay
    plt.plot([quay_side_start[1], quay_side_end[1]], 
             [quay_side_start[0], quay_side_end[0]], 'k', label="Quay")
    plt.plot([quay_front_start[1], quay_front_end[1]], 
             [quay_front_start[0], quay_front_end[0]], 'k')
    
    line_length = 30
    # Plot frontquay line constraint
    frontquay_point = frontquay_points[1]
    frontquay_normal = frontquay_normal_vectors[1]
    frontquay_line_tangent = np.array([-frontquay_normal[1], frontquay_normal[0]])
    
    frontquay_start_point = frontquay_point[:2]  # Extract the (x, y) coordinates of the point on the line
    frontquay_end_point_1 = frontquay_start_point + line_length * frontquay_line_tangent
    frontquay_end_point_2 = frontquay_start_point - line_length * frontquay_line_tangent 
    
    plt.scatter(frontquay_points[1, 1], frontquay_points[1, 0], color='blue', s=30)  # Plot line points
    plt.plot([frontquay_end_point_1[1], frontquay_end_point_2[1]], [frontquay_end_point_1[0], frontquay_end_point_2[0]], color='green', alpha=0.5)
    
    # Plot sidequay point on line
    sidequay_point = sidequay_points[1]
    plt.scatter(sidequay_point[1], sidequay_point[0], color='blue', s=30)
    sidequay_normal = sidequay_normal_vectors[1]
    # sidequay_normal_unit = sidequay_normal / np.linalg.norm(sidequay_normal) 
    sidequay_start_point = sidequay_point[:2]  # Extract the (x, y) coordinates of the point on the line
    sidequay_line_tangent = np.array([-sidequay_normal[1], sidequay_normal[0]])
    sidequay_end_point_1 = sidequay_start_point + line_length * sidequay_line_tangent  # Point in the direction of the normal vector
    sidequay_end_point_2 = sidequay_start_point - line_length * sidequay_line_tangent
    # Plot sidequay line constraints
    for i in range(len(sidequay_normal_vectors)):
        sidequay_normal = sidequay_normal_vectors[i]
        # sidequay_normal_unit = sidequay_normal / np.linalg.norm(sidequay_normal) 
        sidequay_start_point = sidequay_point[:2]  # Extract the (x, y) coordinates of the point on the line
        sidequay_line_tangent = np.array([-sidequay_normal[1], sidequay_normal[0]])
        sidequay_end_point_1 = sidequay_start_point + line_length * sidequay_line_tangent  # Point in the direction of the normal vector
        sidequay_end_point_2 = sidequay_start_point - line_length * sidequay_line_tangent
        # Plot every line constraint
        # plt.plot([sidequay_end_point_1[1], sidequay_end_point_2[1]], [sidequay_end_point_1[0], sidequay_end_point_2[0]], color='green', alpha=0.5)
    # Plot end constraint
    plt.plot([sidequay_end_point_1[1], sidequay_end_point_2[1]], [sidequay_end_point_1[0], sidequay_end_point_2[0]], color='green', alpha=0.5)

    heading_angles = []
    unwrapped_headings = []
    for i in range(0, len(trajectory)):
        yaw = inf2pipi(trajectory[i, 2] + wp_ref[2])  # transforming back to heading relative north
        unwrapped_headings.append(np.rad2deg(unwrap_angle(angle_rad=yaw)))
        heading_angles.append(np.rad2deg(yaw))
        if i % 10 == 0: # plot vessel outline for every nth timestep
            vessel_vertices = rotated_box_vertices(center_north=trajectory[i, 0], center_east=trajectory[i, 1],
                                                yaw_rad=yaw, length=12.25, width=5.25)
            plt.plot(vessel_vertices[:, 0], vessel_vertices[:, 1], ':')   # plot (x,y) coordinates of vertices
    # plotting the last vessel outline
    vessel_vertices = rotated_box_vertices(center_north=trajectory[-1, 0], center_east=trajectory[-1, 1],
                                               yaw_rad=yaw, length=12.25, width=5.25)
    plt.plot(vessel_vertices[:, 0], vessel_vertices[:, 1], ':', color='red')   # plot (x,y) coordinates of vertices
    
    heading_angles = np.array(heading_angles)
    unwrapped_headings = np.array(unwrapped_headings)

    plt.xlabel('East Position')
    plt.ylabel('North Position')
    plt.title(f'MPC Trajectory Visualization (#iterations: {i})')
    plt.legend()
    plt.grid()
    plt.axis('equal')

    plt.figure(figsize=(10, 5))
    plt.subplot(3, 1, 1)
    plt.plot(time_steps, trajectory[:, 0], label='North Position (m)', color='green')
    plt.xlabel('Time Step')
    plt.ylabel('Position')
    plt.legend()
    plt.grid()    

    plt.subplot(3, 1, 2)
    plt.plot(time_steps, trajectory[:, 1], label='East Position (m)', color='orange')
    plt.xlabel('Time Step')
    plt.ylabel('Position')
    plt.legend()
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(time_steps, unwrapped_headings, label="Heading angle (deg)", color='b')
    plt.xlabel('Time Step')
    plt.ylabel('Angle (degree)')
    plt.legend()
    plt.grid()
    plt.suptitle('Pose Components Over Time')

    # Plot total speed over time
    plt.figure(figsize=(10, 5))
    plt.plot(time_steps, speeds, label='Total Linear Velocity', color='b')
    plt.xlabel('Time Step')
    plt.ylabel('Speed (m/s)')
    plt.title('Total Linear Velocity Over Time')
    plt.legend()
    plt.grid()

    # Velocity components
    plt.figure(figsize=(10, 5))
    plt.subplot(3, 1, 1)
    plt.plot(time_steps, velocities[:, 0], label='North Velocity (m/s)', color='green')
    plt.xlabel('Time Step')
    plt.ylabel('Velocity')
    plt.legend()
    plt.grid()

    plt.subplot(3, 1, 2)
    plt.plot(time_steps, velocities[:, 1], label='East Velocity (m/s)', color='orange')
    plt.xlabel('Time Step')
    plt.ylabel('Velocity')
    plt.legend()
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(time_steps, velocities[:, 2], label='Yaw Rate (rad/s)', color='blue')
    plt.xlabel('Time Step')
    plt.ylabel('Yaw Rate')
    plt.legend()
    plt.grid()

    plt.suptitle('Velocity Components Over Time')
    plt.tight_layout()
    plt.show()


def test_run_and_plot_one_mpc_iteration(setup_mpc):
    SAVE_DIR = r'C:\Users\rkhan\masteroppgave-rapport\results\mpc_output\docking_scenario1_penalize_accel'
    os.makedirs(SAVE_DIR, exist_ok=True)

    # Initial state and body velocity
    state = np.array([130, 505, 3.84])
    state[2] = inf2pipi(ang=state[2])
    body_vel = np.array([1, 0, 0])
    
    R = body_to_ned_transform(psi=state[2])
    state_dot = R @ body_vel  # Initial velocity components

    wp_ref = np.array([87.5, 494.9, 3.05084265359])
    wp_ref[2] = inf2pipi(ang=wp_ref[2])
    relative_heading = inf2pipi(ang=(state[2] - wp_ref[2]))
    state = np.array([state[0], state[1], relative_heading])

    nan_vec = np.array([np.nan, np.nan])

    # Set reference for MPC
    mpc_wp_ref = np.array([wp_ref[0], wp_ref[1], 0])
    setup_mpc.opti.set_value(setup_mpc.wp_ref, mpc_wp_ref)
    
    # Run MPC once
    opt_sol = setup_mpc.solve_mpc(t=0, state=state, state_dot=state_dot)
    trajectory = opt_sol.state
    velocity = opt_sol.state_dot
    frontquay_normals = opt_sol.frontquay_line_normals
    frontquay_points = opt_sol.frontquay_line_points
    sidequay_normals = opt_sol.sidequay_line_normals
    sidequay_points = opt_sol.sidequay_line_points

    velocity_body = np.array([body_to_ned_transform(psi=trajectory[i, 2]).T @ velocity[i, :] for i in range(len(velocity))])
    lin_speed = np.linalg.norm(velocity_body[:, :2], axis=1)
    accelerations = np.diff(velocity_body, axis=0) / setup_mpc.dt
    lin_accel = np.linalg.norm(accelerations[:, :2], axis=1)
    
    time_state_dot = np.arange(len(velocity_body))
    time_state = np.arange(len(trajectory))
    time_accel = time_state_dot[:-1]  # acceleration has one less sample than state_dots

    # Extract quay line constraints
    sidequay_start = setup_mpc.opti.value(setup_mpc.sidequay_start)
    sidequay_end = setup_mpc.opti.value(setup_mpc.sidequay_end)
    frontquay_start = setup_mpc.opti.value(setup_mpc.frontquay_start)
    frontquay_end = setup_mpc.opti.value(setup_mpc.frontquay_end)
    quay_thickness = 0.87

    # Plot trajectory with vessel outline, quay and wp
    fig, ax = plt.subplots(figsize=(10, 5))
    plot_thick_quay(ax=ax, quay_start=frontquay_start, quay_end=frontquay_end, quay_thickness=quay_thickness,
                    color=[0.7216, 0.6078, 0.1647], label="Front quay")
    # plot_thick_quay(ax=ax, quay_start=sidequay_start, quay_end=sidequay_end, quay_thickness=quay_thickness,
    #                 color=[0.7216, 0.6078, 0.1647], label="Side quay")

    plt.plot(trajectory[:, 1], trajectory[:, 0], linewidth=1.5, label='Planned trajectory', color=[0.0, 0.3647, 0.9686])

    # Plot vessel outline and compute heading relative north
    heading_angles = []
    unwrapped_headings = []
    for i in range(0, len(trajectory)):
        yaw = inf2pipi(trajectory[i, 2] + wp_ref[2])  # transforming back to heading relative north
        unwrapped_headings.append(np.rad2deg(unwrap_angle(angle_rad=yaw)))
        heading_angles.append(np.rad2deg(yaw))
        if i % 30 == 0: # plot vessel outline for every nth timestep
            vessel_vertices = rotated_box_vertices(center_north=trajectory[i, 0], center_east=trajectory[i, 1],
                                                yaw_rad=yaw, length=12.25, width=5.25)
            plt.plot(vessel_vertices[:, 0], vessel_vertices[:, 1], ':', linewidth=1, color=[0.0, 0.3647, 0.9686], alpha=0.5)   # plot (x,y) coordinates of vertices
    # plotting the last vessel outline
    vessel_vertices = rotated_box_vertices(center_north=trajectory[-1, 0], center_east=trajectory[-1, 1],
                                               yaw_rad=yaw, length=12.25, width=5.25)
    plt.plot(vessel_vertices[:, 0], vessel_vertices[:, 1], ':', linewidth=1, color='blue', alpha=0.5, label='Vessel outline')   # plot (x,y) coordinates of vertices
    
    heading_angles = np.array(heading_angles)
    unwrapped_headings = np.array(unwrapped_headings)
    distances_to_goal = np.linalg.norm(trajectory[:, 0:2] - wp_ref[0:2], axis=1)

    plt.scatter(wp_ref[1], wp_ref[0], color='k', s=20, label='Docking point')
    
    # Plot reference/guide line
    guideline_normal = setup_mpc.opti.value(setup_mpc.ref_line_unit_normal)
    guideline_dir = np.array([-guideline_normal[1], guideline_normal[0]])
    line_length= 40
    guideline_start = wp_ref[:2] - guideline_dir * line_length*0.4
    guideline_end = wp_ref[:2] + guideline_dir * line_length

    plt.plot([guideline_start[1], guideline_end[1]],
            [guideline_start[0], guideline_end[0]],
            linestyle='--', color='k', linewidth=1.5, label='Guide line')

    plt.xlabel('East position (m)')
    plt.ylabel('North position (m)')
    # plt.title('MPC Planned Trajectory')
    plt.legend()
    plt.grid()
    plt.axis('equal')
    plt.savefig(os.path.join(SAVE_DIR,'mpc_traj.pdf'), bbox_inches='tight')
    # -----------------------------------------

    # POSE COMPONENTS (NED)
    plt.figure(figsize=(10, 5))
    plt.subplot(3, 1, 1)
    plt.plot(distances_to_goal, trajectory[:, 0], linewidth=1.5, label='x', color=[0.0, 0.3647, 0.9686])
    plt.ylabel('m')
    plt.legend()
    plt.gca().invert_xaxis()
    plt.grid()

    plt.subplot(3, 1, 2)
    plt.plot(distances_to_goal, trajectory[:, 1], linewidth=1.5, label='y', color=[0.0, 0.3647, 0.9686])
    plt.ylabel('m')
    plt.legend()
    plt.gca().invert_xaxis()
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(distances_to_goal, unwrapped_headings, linewidth=1.5, label=r'$\psi$', color=[0.0, 0.3647, 0.9686])
    plt.xlabel('Distance to docking point')
    plt.ylabel('deg')
    plt.legend()
    plt.grid()
    plt.gca().invert_xaxis()
    plt.savefig(os.path.join(SAVE_DIR,'mpc_pose_components.pdf'), bbox_inches='tight')
    # plt.suptitle('Pose Components Over Time')
    # ----------------------------------------

    # VELOCITY COMPONENTS (BODY)
    plt.figure(figsize=(10, 5))
    plt.subplot(3, 1, 1)
    plt.plot(distances_to_goal[:-1], velocity_body[:, 0], linewidth=1.5, label='u', color=[0.0, 0.3647, 0.9686])
    plt.ylabel('m/s')
    plt.legend()
    plt.gca().invert_xaxis()
    plt.grid()

    plt.subplot(3, 1, 2)
    plt.plot(distances_to_goal[:-1], velocity_body[:, 1], linewidth=1.5, label='v', color=[0.0, 0.3647, 0.9686])

    plt.ylabel('m/s')
    plt.legend()
    plt.gca().invert_xaxis()
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(distances_to_goal[:-1], velocity_body[:, 2], linewidth=1.5, label='r', color=[0.0, 0.3647, 0.9686])
    plt.xlabel('Distance to docking point (m)')
    plt.ylabel('deg/s')
    plt.legend()
    plt.grid()
    plt.gca().invert_xaxis()

    # plt.suptitle('Velocity Components Over Time')
    plt.tight_layout()
    plt.savefig(os.path.join(SAVE_DIR,'mpc_nu_components.pdf'), bbox_inches='tight')

    # -------------------------------

    # LINEAR AND ANGULAR SPEED
    plt.figure(figsize=(8, 6))
    plt.subplot(2, 1, 1)
    plt.plot(distances_to_goal[:-1], lin_speed, linewidth=1.5, color=[0.0, 0.3647, 0.9686], label='Linear speed')
    plt.ylabel('m/s')
    plt.legend()
    # plt.title('Total Linear Speed vs. Distance to Docking Pose')
    plt.gca().invert_xaxis()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(distances_to_goal[:-1], velocity_body[:, 2], linewidth=1.5, color=[0.0, 0.3647, 0.9686], label='r')
    plt.xlabel('Distance to docking point (m)')
    plt.ylabel('deg/s')
    plt.legend()
    # plt.title('Total Linear Speed vs. Distance to Docking Pose')
    plt.gca().invert_xaxis()
    plt.grid(True)

    plt.savefig(os.path.join(SAVE_DIR,'mpc_speed.pdf'), bbox_inches='tight')

    # --------------------------------

    # ACCELERATION COMPONENTS (BODY)
    plt.figure(figsize=(8, 8))
    plt.subplot(3, 1, 1)
    plt.plot(distances_to_goal[:-2], accelerations[:, 1], linewidth=1.5, color=[0.0, 0.3647, 0.9686], label=r'$\overset{\mathbf{\bullet}}{u}$')
    # plt.xlabel('Distance to docking point (m)')
    plt.ylabel('m/s²')
    # plt.title('Acceleration Components vs. Time')
    plt.gca().invert_xaxis()
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(distances_to_goal[:-2], accelerations[:, 0], linewidth=1.5, color=[0.0, 0.3647, 0.9686], label=r'$\overset{\mathbf{\bullet}}{v}$')
    # plt.xlabel('Distance to docking point (m)')
    plt.ylabel('m/s²')
    plt.gca().invert_xaxis()
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(distances_to_goal[:-2], accelerations[:, 2], linewidth=1.5, color=[0.0, 0.3647, 0.9686], label=r'$\overset{\mathbf{\bullet}}{r}$')
    plt.xlabel('Distance to docking point (m)')
    plt.ylabel('deg/s²')
    plt.gca().invert_xaxis()
    plt.grid(True)
    plt.legend()
    plt.savefig(os.path.join(SAVE_DIR,'mpc_accel_components.pdf'), bbox_inches='tight')
    # -------------------------------------
    
    # LINEAR AND ANGULAR ACCELERATION
    plt.figure(figsize=(8, 6))
    plt.subplot(2, 1, 1)
    plt.plot(distances_to_goal[:-2], lin_accel, linewidth=1.5, color=[0.0, 0.3647, 0.9686], label='Linear acceleration')
    plt.ylabel('m/s²')
    plt.legend()
    plt.gca().invert_xaxis()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(distances_to_goal[:-2], accelerations[:, 2], linewidth=1.5, color=[0.0, 0.3647, 0.9686], label=r'$\overset{\mathbf{\bullet}}{r}$')
    plt.xlabel('Distance to docking point (m)')
    plt.ylabel('deg/s²')
    plt.legend()
    plt.gca().invert_xaxis()
    plt.grid(True)

    plt.savefig(os.path.join(SAVE_DIR,'mpc_accel.pdf'), bbox_inches='tight')

    # plt.show()