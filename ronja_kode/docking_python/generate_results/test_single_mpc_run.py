import os
import numpy as np
from generate_results.utils import unwrap_angle
import pickle
import matplotlib.pyplot as plt

from docking_algorithms.utils.data_logger import load_config, load_mpc_trajectories
from docking_algorithms.utils.types import OptimalTrajectories
from docking_algorithms.utils.transformations import body_to_ned_transform
from docking_algorithms.utils.inf2pipi import inf2pipi

from docking_algorithms.utils.computations import rotated_box_vertices
from generate_results.utils import plot_thick_quay

from docking_algorithms.controllers.interpolation import InterpolateMPCOutput
from docking_algorithms.controllers.mpc_guidance import MPCGuidance
from generate_results.types import Metric
from generate_results.read_write_files import save_metrics_single_run


BASE_DIR = '../masteroppgave-rapport/results/'
PLOTTING_COLORS = load_config(cfg_name="plotting", cfg_path="../../generate_results")["plotting_colors"]
QUAYS_VESSEL_CFG = load_config(cfg_name="quays_vessel", cfg_path="../config")
MPC_CFG = load_config(cfg_name="mpc", cfg_path="../config").no_disturbances
ALL_CONTROLLERS = ["pid", "mpc"]
ALL_BATCHES = ["no_disturbances", "with_disturbances"]
# ALL_BATCHES = ["no_disturbances", "small_disturbances", "moderate_disturbances", "large_disturbances"]


# TODO: ensure that these are correct before plotting !!
CONTROLLER_TYPE = "mpc_pid" # pid or mpc_pid
QUAY_TYPE = "l_quay" # l_quay or h_quay. TODO: Maybe this will be part of the results hierarchy
RUN_ID = 1 # 1 to n (n is to be decided)

# ----------
BATCH = "no_disturbances" # no_disturbances, small_disturbances, moderate_disturbances, large_disturbances
RUN_INFO = 'guideline_frontCenter_sideCenter_right'
# ----------

CONTROLLER_DIR = os.path.join(BASE_DIR, CONTROLLER_TYPE)
BATCH_DIR = os.path.join(CONTROLLER_DIR, BATCH)
RUN_DIR = os.path.join(BATCH_DIR, f"run{str(RUN_ID)}", RUN_INFO)
MPC_OUTPUT_DIR = os.path.join(RUN_DIR, "mpc_output")


WP_REF = np.array(MPC_CFG.docking_pose)
WP_REF[2] = inf2pipi(ang=WP_REF[2])

FONTSIZE_LEGEND = 12
FONTSIZE_LEGEND_LARGE = 13

plt.rcParams.update({
    'axes.labelsize': 13,   # Axis labels
    'xtick.labelsize': 13,  # Tick labels x
    'ytick.labelsize': 13,  # Tick labels y
    # 'legend.fontsize': 10,  # Legend
    # 'font.size': 12         # Default font size
})


def test_single_mpc_run() -> None:
# ----------
    init_pose = np.array([130, 470, 2.4])   # from left: [130, 470, 2.4]    from right: [130, 505, 3.84] (possible 510)
    init_pose[2] = inf2pipi(ang=init_pose[2])
    init_vel_body = np.array([1, 0, 0])
# ----------
    
    # ---------------------------------------
    # LOGGING 
    filename = "mpc_opt_trajectories.pkl"
    filepath = os.path.join(MPC_OUTPUT_DIR, filename)

    # Ensure the directory exists
    os.makedirs(MPC_OUTPUT_DIR, exist_ok=True)

    # Check and raise error if file already exists
    if os.path.exists(filepath):
        raise FileExistsError(f"The file '{filepath}' already exists. Terminating to avoid overwriting.")
    # --------------------------------------------------
    
    R = body_to_ned_transform(psi=init_pose[2])
    init_vel_ned = R @ init_vel_body  # Initial velocity components
    
    dt = 0.1
    mpc_wp_ref = np.array([WP_REF[0], WP_REF[1], 0])

    heading_rel_wp = inf2pipi(ang=(init_pose[2] - WP_REF[2]))
    init_pose = np.array([init_pose[0], init_pose[1], heading_rel_wp])

    mpc = MPCGuidance(dt=dt, mpc_cfg=MPC_CFG, quays_vessel_cfg=QUAYS_VESSEL_CFG, waypoint_ref=mpc_wp_ref,
                      max_speed=np.linalg.norm(init_vel_ned[:2]))
    
    opt_sol = mpc.solve_mpc(t=0, state=init_pose, state_dot=init_vel_ned)
    opt_sol.state[:, 2] = np.array([inf2pipi(psi + WP_REF[2]) for psi in opt_sol.state[:, 2]])
    opt_sol.state_dot = np.vstack([init_vel_ned, opt_sol.state_dot])

    # --- STORE MPC OUTPUT --- #
    try:
        # Try loading existing data
        with open(filepath, "rb") as f:
            data = pickle.load(f)
    except (FileNotFoundError, EOFError):  
        # File not found or empty → Create new list
        # print("Pickle file not found or empty, creating a new one.")
        data = []

    # Append new trajectory and save
    data.append(opt_sol)

    with open(filepath, "wb") as f:
        pickle.dump(data, f)
    return


def test_plot_mpc_output():
    dt = 0.1
    wp_ref = np.array(MPC_CFG.docking_pose)

    mpc_output = load_mpc_trajectories(directory=MPC_OUTPUT_DIR)[0]
    trajectory = mpc_output.state
    velocity = mpc_output.state_dot

    velocity_body = np.array([body_to_ned_transform(psi=trajectory[i, 2]).T @ velocity[i, :] for i in range(len(velocity))])
    lin_speed = np.linalg.norm(velocity_body[:, :2], axis=1)
    accelerations = np.diff(velocity_body, axis=0) / dt
    lin_accel = np.linalg.norm(accelerations[:, :2], axis=1)

    time_state = np.arange(len(trajectory)) * dt
    time_accel = time_state[:-1]

    sidequay_start = np.array(QUAYS_VESSEL_CFG.quays.side_segment.corner)
    sidequay_end = np.array(QUAYS_VESSEL_CFG.quays.side_segment.end)
    frontquay_start = np.array(QUAYS_VESSEL_CFG.quays.front_segment.corner)
    frontquay_end = np.array(QUAYS_VESSEL_CFG.quays.front_segment.end)
    quay_thickness = QUAYS_VESSEL_CFG.quays.thickness

    fig, ax = plt.subplots(figsize=(10, 5))
    plot_thick_quay(ax=ax, quay_start=frontquay_start, quay_end=frontquay_end, quay_thickness=quay_thickness,
                    quay_normal_vector=MPC_CFG.frontquay_unit_normal, color=[0.7216, 0.6078, 0.1647])
    plot_thick_quay(ax=ax, quay_start=sidequay_start, quay_end=sidequay_end, quay_thickness=quay_thickness,
                    quay_normal_vector=MPC_CFG.sidequay_unit_normal, color=[0.7216, 0.6078, 0.1647], label="Quay")

    ## ----- PLOT TRAJECTORY ----- ##
    ax.plot(trajectory[:, 1], trajectory[:, 0], linewidth=1.5, label='Planned trajectory', color=[0.0, 0.3647, 0.9686])

    unwrapped_headings = []
    for i in range(len(trajectory)):
        yaw = trajectory[i, 2]
        unwrapped_headings.append(np.rad2deg(unwrap_angle(angle_rad=yaw)))
        if i % 30 == 0:
            vessel_vertices = rotated_box_vertices(center_north=trajectory[i, 0], center_east=trajectory[i, 1],
                                                    yaw_rad=yaw, length=12.25, width=5.25)
            ax.plot(vessel_vertices[:, 0], vessel_vertices[:, 1], ':', linewidth=1, color=[0.0, 0.3647, 0.9686], alpha=0.5)
    vessel_vertices = rotated_box_vertices(center_north=trajectory[-1, 0], center_east=trajectory[-1, 1],
                                           yaw_rad=yaw, length=12.25, width=5.25)
    ax.plot(vessel_vertices[:, 0], vessel_vertices[:, 1], ':', linewidth=1, color='blue', alpha=0.5, label='Vessel outline')

    unwrapped_headings = np.array(unwrapped_headings)
    distances_to_goal = np.linalg.norm(trajectory[:, 0:2] - wp_ref[0:2], axis=1)

    # ax.scatter(wp_ref[1], wp_ref[0], color='k', s=20, label='Docking point')
    ax.plot(wp_ref[1], wp_ref[0], 'o', markerfacecolor='black', markeredgecolor='black', markersize=6, alpha=1)
    ax.text(wp_ref[1] - 4, wp_ref[0] - 2, r'$\boldsymbol{\eta}_{\text{dock}}$', fontsize=FONTSIZE_LEGEND_LARGE, color='black')  # not cropper
    # ax.text(wp_ref[1] - 1, wp_ref[0] - 1.5, r'$\boldsymbol{\eta}_{\text{dock}}$', fontsize=FONTSIZE_LEGEND_LARGE, color='black')  # cropped

    if "guideline" in RUN_INFO:
        guideline_normal = np.array(MPC_CFG.ref_line_unit_normal)
        guideline_dir = np.array([-guideline_normal[1], guideline_normal[0]])
        line_length = 40
        guideline_start = wp_ref[:2] - guideline_dir * line_length * 0.4
        guideline_end = wp_ref[:2] + guideline_dir * line_length

        ax.plot([guideline_start[1], guideline_end[1]],
                [guideline_start[0], guideline_end[0]],
                linestyle='--', color='k', linewidth=1.5, alpha=0.6, label='Guide line')

    ax.set_xlabel(r'East [$m$]', fontsize=10)
    ax.set_ylabel(r'North [$m$]', fontsize=10)
    ax.legend(loc='best', fontsize=10)
    ax.tick_params(axis='both', labelsize=10)
    ax.grid(True)
    # ax.axis('equal')
    ax.set_aspect('equal')
    ax.set_xlim(470, 540)   # from right
    # ax.set_xlim(460, 520)   # from left
    ax.set_ylim(80, 140)
    # ax.set_xlim(485, 515)   # zoom in
    # ax.set_ylim(80, 110)   # zoom in
    fig.tight_layout()
    fig.savefig(os.path.join(MPC_OUTPUT_DIR, 'mpc_traj.pdf'), bbox_inches='tight')    


    ## ----- PLOT HEADING, YAW RATE AND LINEAR SPEED ----- ##
    fig, axs = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    axs[0].plot(distances_to_goal, unwrapped_headings, linewidth=1.5, label=r'$\psi$', color=[0.0, 0.3647, 0.9686])
    axs[0].axhline(y=np.rad2deg(unwrap_angle(wp_ref[2])), linestyle='--', color='black', alpha=0.6, label=r'$\boldsymbol{\eta}_{\psi, \; \text{dock}}$')
    axs[0].set_ylabel(r'Heading [$deg$]')
    axs[0].legend(loc='upper right', fontsize=FONTSIZE_LEGEND_LARGE)
    axs[0].grid(True)

    axs[1].plot(distances_to_goal, velocity_body[:, 2], linewidth=1.5, label='r', color=[0.0, 0.3647, 0.9686])
    axs[1].set_ylabel(r'Yaw rate [$deg/s$]')
    axs[1].legend(loc='lower right', fontsize=FONTSIZE_LEGEND_LARGE)
    axs[1].grid(True)

    axs[2].plot(distances_to_goal, lin_speed, linewidth=1.5, label='U', color=[0.0, 0.3647, 0.9686])
    axs[2].set_ylabel(r'Speed [$m/s$]')
    axs[2].set_xlabel(r'Distance to $\boldsymbol{\eta}_{\text{dock}}$ [m]')
    axs[2].legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGE)
    axs[2].grid(True)

    for ax in axs:
        ax.invert_xaxis()

    fig.tight_layout()
    fig.savefig(os.path.join(MPC_OUTPUT_DIR, 'mpc_head_yawRate_speed.pdf'), bbox_inches='tight')


    ## ----- PLOT ACCELERATION COMPONENTS ----- ##
    fig, axs = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    axs[0].plot(time_accel, accelerations[:, 0], linewidth=1.5, label=r'$\overset{\mathbf{\bullet}}{u}$', color=[0.0, 0.3647, 0.9686])
    axs[0].set_ylabel('Acceleration' + '\n' + r'[$m/s^2$]')
    axs[0].legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGE)
    axs[0].grid(True)

    axs[1].plot(time_accel, accelerations[:, 1], linewidth=1.5, label=r'$\overset{\mathbf{\bullet}}{v}$', color=[0.0, 0.3647, 0.9686])
    axs[1].set_ylabel('Acceleration' + '\n' + r'[$m/s^2$]')
    axs[1].legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGE)
    axs[1].grid(True)

    axs[2].plot(time_accel, accelerations[:, 2], linewidth=1.5, label=r'$\overset{\mathbf{\bullet}}{r}$', color=[0.0, 0.3647, 0.9686])
    axs[2].set_ylabel('Acceleration' + '\n' + r' [$deg/s^2$]')
    axs[2].set_xlabel(r'Time [$s$]')
    axs[2].legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGE)
    axs[2].grid(True)

    fig.tight_layout()
    fig.savefig(os.path.join(MPC_OUTPUT_DIR, 'mpc_accel_components.pdf'), bbox_inches='tight')

    ## ----- PLOT LINEAR AND ANGULAR ACCELERATION ----- ##
    # fig, axs = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    # axs[0].plot(time_accel, lin_accel, linewidth=1.5, label=r'$\overset{\mathbf{\bullet}}{U}$', color=[0.0, 0.3647, 0.9686])
    # axs[0].set_ylabel(r'Acceleration [$m/s^2$]')
    # axs[0].legend(loc='best', fontsize=FONTSIZE_LEGEND)
    # axs[0].grid(True)

    # axs[1].plot(time_accel, accelerations[:, 2], linewidth=1.5, label=r'$\overset{\mathbf{\bullet}}{r}$', color=[0.0, 0.3647, 0.9686])
    # axs[1].set_ylabel(r'Acceleration [$deg/s^2$]')
    # axs[1].set_xlabel(r'Time [$s$]')
    # axs[1].legend(loc='best', fontsize=FONTSIZE_LEGEND)
    # axs[1].grid(True)

    # fig.tight_layout()
    # fig.savefig(os.path.join(MPC_OUTPUT_DIR, 'mpc_accel.pdf'), bbox_inches='tight')

    # ## ----- PLOT POSE COMPONENTS ----- ##
    # fig, axs = plt.subplots(3, 1, figsize=(10, 5), sharex=True)
    # axs[0].plot(time_state, trajectory[:, 0], linewidth=1.5, label='x', color=[0.0, 0.3647, 0.9686])
    # axs[0].axhline(y=wp_ref[0], linestyle='--', color='black', alpha=0.6, label=r'$\boldsymbol{\eta}_{\text{dock}, \; x}$')
    # axs[0].set_ylabel(r'North [$m$]')
    # axs[0].legend(loc='best', fontsize=FONTSIZE_LEGEND)
    # axs[0].grid(True)

    # axs[1].plot(time_state, trajectory[:, 1], linewidth=1.5, label='y', color=[0.0, 0.3647, 0.9686])
    # axs[1].axhline(y=wp_ref[1], linestyle='--', color='black', alpha=0.6, label=r'$\boldsymbol{\eta}_{\text{dock}, \; y}$')
    # axs[1].set_ylabel(r'East [$m$]')
    # axs[1].legend(loc='best', fontsize=FONTSIZE_LEGEND)
    # axs[1].grid(True)

    # axs[2].plot(time_state, unwrapped_headings, linewidth=1.5, label=r'$\psi$', color=[0.0, 0.3647, 0.9686])
    # axs[2].axhline(y=np.rad2deg(unwrap_angle(wp_ref[2])), linestyle='--', color='black', alpha=0.6, label=r'$\boldsymbol{\eta}_{\text{dock}, \; \psi}$')
    # axs[2].set_ylabel(r'Heading [$deg$]')
    # axs[2].set_xlabel(r'Time [$s$]')
    # axs[2].legend(loc='best', fontsize=FONTSIZE_LEGEND)
    # axs[2].grid(True)

    # fig.tight_layout()
    # fig.savefig(os.path.join(MPC_OUTPUT_DIR, 'mpc_pose_components.pdf'), bbox_inches='tight')

    # ## ----- PLOT NU COMPONENTS ----- ##
    # fig, axs = plt.subplots(3, 1, figsize=(10, 5), sharex=True)
    # axs[0].plot(distances_to_goal, velocity_body[:, 0], linewidth=1.5, label='u', color=[0.0, 0.3647, 0.9686])
    # axs[0].set_ylabel(r'Velocity [$m/s$]')
    # axs[0].legend(loc='best', fontsize=FONTSIZE_LEGEND)
    # axs[0].grid(True)

    # axs[1].plot(distances_to_goal, velocity_body[:, 1], linewidth=1.5, label='v', color=[0.0, 0.3647, 0.9686])
    # axs[1].set_ylabel(r'Velocity [$m/s$]')
    # axs[1].legend(loc='best', fontsize=FONTSIZE_LEGEND)
    # axs[1].grid(True)

    # axs[2].plot(distances_to_goal, velocity_body[:, 2], linewidth=1.5, label='r', color=[0.0, 0.3647, 0.9686])
    # axs[2].set_ylabel(r'Yaw rate [$deg/s$]')
    # axs[2].set_xlabel(r'Distance to $\boldsymbol{\eta}_{\text{dock}}$ [m]')
    # axs[2].legend(loc='best', fontsize=FONTSIZE_LEGEND)
    # axs[2].grid(True)

    # for ax in axs:
    #     ax.invert_xaxis()

    # fig.tight_layout()
    # fig.savefig(os.path.join(MPC_OUTPUT_DIR, 'mpc_nu_components.pdf'), bbox_inches='tight')

    # ## ----- PLOT LINEAR AND ANGULAR SPEED ----- ##
    # fig, axs = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    # axs[0].plot(distances_to_goal, lin_speed, linewidth=1.5, color=[0.0, 0.3647, 0.9686], label='U')
    # axs[0].set_ylabel(r'Speed [$m/s$]')
    # axs[0].legend(loc='best', fontsize=FONTSIZE_LEGEND)
    # axs[0].grid(True)

    # axs[1].plot(distances_to_goal, velocity_body[:, 2], linewidth=1.5, color=[0.0, 0.3647, 0.9686], label='r')
    # axs[1].set_ylabel(r'Yaw rate [$deg/s$]')
    # axs[1].set_xlabel(r'Distance to $\boldsymbol{\eta}_{\text{dock}}$ [m]')
    # axs[1].legend(loc='best', fontsize=FONTSIZE_LEGEND)
    # axs[1].grid(True)

    # for ax in axs:
    #     ax.invert_xaxis()

    # fig.tight_layout()
    # fig.savefig(os.path.join(MPC_OUTPUT_DIR, 'mpc_speed.pdf'), bbox_inches='tight')

    return




def test_replot_all_mpc_runs() -> None:
    all_mpc_run_descriptions = [
    "baseline_mpc_left",
    "frontCenter_left",
    "frontCenter_sideCenter_left",
    "frontCenter_sideCenter_right",
    "frontCenter_sideCorner_left",
    "frontCenter_sideCorner_right",
    "frontCorner_left",
    "guideline_frontCenter_left",
    "guideline_frontCenter_right",
    "guideline_frontCenter_sideCenter_left",
    "guideline_frontCenter_sideCenter_right",
    "guideline_left",
    "guideline_right",
    "sideCenter_left",
    "sideCenter_right",
    "sideCorner_left",
    "sideCorner_right"
]

    original_globals = globals().copy()
    globals()['CONTROLLER_TYPE'] = "mpc_pid"
    globals()['CONTROLLER_DIR'] = os.path.join(BASE_DIR, CONTROLLER_TYPE)
    for batch in ALL_BATCHES:
        globals()['BATCH'] = batch
        globals()['BATCH_DIR'] = os.path.join(CONTROLLER_DIR, batch)
        for run_info in all_mpc_run_descriptions:
            globals()['RUN_INFO'] = run_info
            run_dir = os.path.join(BATCH_DIR, f"run{str(RUN_ID)}", run_info)
            globals()['RUN_DIR'] = run_dir
            globals()['MPC_OUTPUT_DIR'] = os.path.join(RUN_DIR, "mpc_output")

            if not os.path.isdir(run_dir):
                    print(f"Skipping missing folder: {run_dir}")
                    continue

            test_plot_mpc_output()
    globals().update(original_globals)
    return



# def test_plot_mpc_output():
#     dt = 0.1
#     wp_ref = np.array(MPC_CFG.docking_pose)

#     mpc_output = load_mpc_trajectories(directory=MPC_OUTPUT_DIR)[0]
#     trajectory = mpc_output.state
#     velocity = mpc_output.state_dot
#     frontquay_normals = mpc_output.frontquay_line_normals
#     frontquay_points = mpc_output.frontquay_line_points
#     sidequay_normals = mpc_output.sidequay_line_normals
#     sidequay_points = mpc_output.sidequay_line_points
        
#     velocity_body = np.array([body_to_ned_transform(psi=trajectory[i, 2]).T @ velocity[i, :] for i in range(len(velocity))])
#     lin_speed = np.linalg.norm(velocity_body[:, :2], axis=1)
#     accelerations = np.diff(velocity_body, axis=0) / dt
#     lin_accel = np.linalg.norm(accelerations[:, :2], axis=1)

#     time_state = np.arange(len(trajectory)) * dt
#     time_state_dot = time_state
#     time_accel = time_state_dot[:-1]  # acceleration has one less sample than state and state_dot

#     # Extract quay line constraints
#     sidequay_start = np.array(QUAYS_VESSEL_CFG.quays.side_segment.corner)
#     sidequay_end = np.array(QUAYS_VESSEL_CFG.quays.side_segment.end)
#     frontquay_start = np.array(QUAYS_VESSEL_CFG.quays.front_segment.corner)
#     frontquay_end = np.array(QUAYS_VESSEL_CFG.quays.front_segment.end)
#     quay_thickness = QUAYS_VESSEL_CFG.quays.thickness

#     # Plot trajectory with vessel outline, quay and wp
#     fig, ax = plt.subplots(figsize=(10, 5))
#     plot_thick_quay(ax=ax, quay_start=frontquay_start, quay_end=frontquay_end, quay_thickness=quay_thickness,
#                     quay_normal_vector=MPC_CFG.frontquay_unit_normal, color=[0.7216, 0.6078, 0.1647])
#     plot_thick_quay(ax=ax, quay_start=sidequay_start, quay_end=sidequay_end, quay_thickness=quay_thickness,
#                     quay_normal_vector=MPC_CFG.sidequay_unit_normal, color=[0.7216, 0.6078, 0.1647], label="Quay")

#     plt.plot(trajectory[:, 1], trajectory[:, 0], linewidth=1.5, label='Planned trajectory', color=[0.0, 0.3647, 0.9686])

#     # Plot vessel outline and compute heading relative north
#     unwrapped_headings = []
#     for i in range(0, len(trajectory)):
#         yaw = trajectory[i, 2]
#         unwrapped_headings.append(np.rad2deg(unwrap_angle(angle_rad=yaw)))
#         if i % 30 == 0: # plot vessel outline for every nth timestep
#             vessel_vertices = rotated_box_vertices(center_north=trajectory[i, 0], center_east=trajectory[i, 1],
#                                                 yaw_rad=yaw, length=12.25, width=5.25)
#             plt.plot(vessel_vertices[:, 0], vessel_vertices[:, 1], ':', linewidth=1, color=[0.0, 0.3647, 0.9686], alpha=0.5)   # plot (x,y) coordinates of vertices
#     # plotting the last vessel outline
#     vessel_vertices = rotated_box_vertices(center_north=trajectory[-1, 0], center_east=trajectory[-1, 1],
#                                                yaw_rad=yaw, length=12.25, width=5.25)
#     plt.plot(vessel_vertices[:, 0], vessel_vertices[:, 1], ':', linewidth=1, color='blue', alpha=0.5, label='Vessel outline')   # plot (x,y) coordinates of vertices
    
#     unwrapped_headings = np.array(unwrapped_headings)
#     distances_to_goal = np.linalg.norm(trajectory[:, 0:2] - wp_ref[0:2], axis=1)

#     plt.scatter(wp_ref[1], wp_ref[0], color='k', s=20, label='Docking point')
    
#     if "guideline" in RUN_INFO:
#     # Plot reference/guide line
#         guideline_normal = np.array(MPC_CFG.ref_line_unit_normal)
#         guideline_dir = np.array([-guideline_normal[1], guideline_normal[0]])
#         line_length= 40
#         guideline_start = wp_ref[:2] - guideline_dir * line_length*0.4
#         guideline_end = wp_ref[:2] + guideline_dir * line_length

#         plt.plot([guideline_start[1], guideline_end[1]],
#                 [guideline_start[0], guideline_end[0]],
#                 linestyle='--', color='k', linewidth=1.5, alpha=0.6, label='Guide line')

#     plt.xlabel(r'East [$m$]')
#     plt.ylabel(r'North [$m$]')
#     # plt.title('MPC Planned Trajectory')
#     plt.legend()
#     plt.grid()
#     plt.axis('equal')
#     plt.savefig(os.path.join(MPC_OUTPUT_DIR,'mpc_traj.pdf'), bbox_inches='tight')
#     # -----------------------------------------

#     # POSE COMPONENTS (NED)
#     plt.figure(figsize=(10, 5))
#     plt.subplot(3, 1, 1)
#     plt.plot(time_state, trajectory[:, 0], linewidth=1.5, label='x', color=[0.0, 0.3647, 0.9686])
#     plt.axhline(y=wp_ref[0], linestyle='--', color='black', alpha=0.6, label=r'$\boldsymbol{\eta}_{\text{dock}, \; x}$')
#     plt.ylabel(r'North [$m$]')
#     plt.legend()
#     # plt.gca().invert_xaxis()
#     plt.grid()

#     plt.subplot(3, 1, 2)
#     plt.plot(time_state, trajectory[:, 1], linewidth=1.5, label='y', color=[0.0, 0.3647, 0.9686])
#     plt.axhline(y=wp_ref[1], linestyle='--', color='black', alpha=0.6, label=r'$\boldsymbol{\eta}_{\text{dock}, \; y}$')
#     plt.ylabel(r'East [$m$]')
#     plt.legend()
#     # plt.gca().invert_xaxis()
#     plt.grid()

#     plt.subplot(3, 1, 3)
#     plt.plot(time_state, unwrapped_headings, linewidth=1.5, label=r'$\psi$', color=[0.0, 0.3647, 0.9686])
#     plt.axhline(y=np.rad2deg(unwrap_angle(wp_ref[2])), linestyle='--', color='black', alpha=0.6, label=r'$\boldsymbol{\eta}_{\text{dock}, \; \psi}$')
#     plt.ylabel(r'Heading [$deg$]')
#     plt.xlabel(r'Time [$s$]')
#     plt.legend()
#     plt.grid()
#     # plt.gca().invert_xaxis()
#     plt.savefig(os.path.join(MPC_OUTPUT_DIR,'mpc_pose_components.pdf'), bbox_inches='tight')
#     # plt.suptitle('Pose Components Over Time')
#     # ----------------------------------------

#     # VELOCITY COMPONENTS (BODY)
#     plt.figure(figsize=(10, 5))
#     plt.subplot(3, 1, 1)
#     plt.plot(distances_to_goal, velocity_body[:, 0], linewidth=1.5, label='u', color=[0.0, 0.3647, 0.9686])
#     plt.ylabel(r'Velocity [$m/s$]')
#     plt.legend()
#     plt.gca().invert_xaxis()
#     plt.grid()

#     plt.subplot(3, 1, 2)
#     plt.plot(distances_to_goal, velocity_body[:, 1], linewidth=1.5, label='v', color=[0.0, 0.3647, 0.9686])

#     plt.ylabel(r'Velocity [$m/s$]')
#     plt.legend()
#     plt.gca().invert_xaxis()
#     plt.grid()

#     plt.subplot(3, 1, 3)
#     plt.plot(distances_to_goal, velocity_body[:, 2], linewidth=1.5, label='r', color=[0.0, 0.3647, 0.9686])
#     plt.ylabel(r'Yaw rate [$deg/s$]')
#     plt.xlabel(r'Distance to $\boldsymbol{\eta}_{\text{dock}}$ [m]')
#     plt.legend()
#     plt.grid()
#     plt.gca().invert_xaxis()

#     # plt.suptitle('Velocity Components Over Time')
#     plt.tight_layout()
#     plt.savefig(os.path.join(MPC_OUTPUT_DIR,'mpc_nu_components.pdf'), bbox_inches='tight')

#     # -------------------------------

#     # LINEAR AND ANGULAR SPEED
#     plt.figure(figsize=(8, 6))
#     plt.subplot(2, 1, 1)
#     plt.plot(distances_to_goal, lin_speed, linewidth=1.5, color=[0.0, 0.3647, 0.9686], label='U')
#     plt.ylabel(r'Speed [$m/s$]')
#     plt.legend()
#     # plt.title('Total Linear Speed vs. Distance to Docking Pose')
#     plt.gca().invert_xaxis()
#     plt.grid(True)

#     plt.subplot(2, 1, 2)
#     plt.plot(distances_to_goal, velocity_body[:, 2], linewidth=1.5, color=[0.0, 0.3647, 0.9686], label='r')
#     plt.ylabel(r'Velocity [$deg/s$]')
#     plt.xlabel(r'Distance to $\boldsymbol{\eta}_{\text{dock}}$ [m]')
#     plt.legend()
#     # plt.title('Total Linear Speed vs. Distance to Docking Pose')
#     plt.gca().invert_xaxis()
#     plt.grid(True)

#     plt.savefig(os.path.join(MPC_OUTPUT_DIR,'mpc_speed.pdf'), bbox_inches='tight')

#     # --------------------------------

#     # ACCELERATION COMPONENTS (BODY)
#     plt.figure(figsize=(8, 8))
#     plt.subplot(3, 1, 1)
#     plt.plot(time_accel, accelerations[:, 1], linewidth=1.5, color=[0.0, 0.3647, 0.9686], label=r'$\overset{\mathbf{\bullet}}{u}$')
#     # plt.xlabel('Distance to docking point [m]')
#     plt.ylabel(r'Acceleration [$m/s²$]')
#     # plt.title('Acceleration Components vs. Time')
#     # plt.gca().invert_xaxis()
#     plt.legend()
#     plt.grid(True)

#     plt.subplot(3, 1, 2)
#     plt.plot(time_accel, accelerations[:, 0], linewidth=1.5, color=[0.0, 0.3647, 0.9686], label=r'$\overset{\mathbf{\bullet}}{v}$')
#     # plt.xlabel('Distance to docking point [m]')
#     plt.ylabel(r'Acceleration [$m/s²$]')
#     # plt.gca().invert_xaxis()
#     plt.legend()
#     plt.grid(True)

#     plt.subplot(3, 1, 3)
#     plt.plot(time_accel, accelerations[:, 2], linewidth=1.5, color=[0.0, 0.3647, 0.9686], label=r'$\overset{\mathbf{\bullet}}{r}$')
#     plt.ylabel(r'Acceleration [$deg/s²$]')
#     plt.xlabel(r'Time [$s$]')
#     # plt.gca().invert_xaxis()
#     plt.grid(True)
#     plt.legend()
#     plt.savefig(os.path.join(MPC_OUTPUT_DIR,'mpc_accel_components.pdf'), bbox_inches='tight')
#     # -------------------------------------
    
#     # LINEAR AND ANGULAR ACCELERATION
#     plt.figure(figsize=(8, 6))
#     plt.subplot(2, 1, 1)
#     plt.plot(time_accel, lin_accel, linewidth=1.5, color=[0.0, 0.3647, 0.9686], label=r'$\overset{\mathbf{\bullet}}{U}$')
#     plt.ylabel(r'Acceleration [$m/s²$]')
#     plt.legend()
#     # plt.gca().invert_xaxis()
#     plt.grid(True)

#     plt.subplot(2, 1, 2)
#     plt.plot(time_accel, accelerations[:, 2], linewidth=1.5, color=[0.0, 0.3647, 0.9686], label=r'$\overset{\mathbf{\bullet}}{r}$')
#     plt.ylabel(r'Acceleration [$deg/s²$]')
#     plt.xlabel(r'Time [$s$]')
#     plt.legend()
#     # plt.gca().invert_xaxis()
#     plt.grid(True)

#     plt.savefig(os.path.join(MPC_OUTPUT_DIR,'mpc_accel.pdf'), bbox_inches='tight')


#     # -------------------------------------

#     # Heading, Heading rate, speed
#     plt.figure(figsize=(8,6))
#     plt.subplot(3,1,1)
#     plt.plot(distances_to_goal, unwrapped_headings, linewidth=1.5, label=r'$\psi$', color=[0.0, 0.3647, 0.9686])
#     plt.axhline(y=np.rad2deg(unwrap_angle(wp_ref[2])), linestyle='--', color='black', alpha=0.6, label=r'$\boldsymbol{\eta}_{\text{dock}, \; \psi}$')
#     plt.ylabel(r'Heading [$deg$]')
#     plt.legend()
#     plt.grid()
#     plt.gca().invert_xaxis()

#     plt.subplot(3,1,2)
#     plt.plot(distances_to_goal, velocity_body[:, 2], linewidth=1.5, color=[0.0, 0.3647, 0.9686], label='r')
#     plt.ylabel(r'Yaw rate [$deg/s$]')
#     plt.legend()
#     plt.gca().invert_xaxis()
#     plt.grid(True)

#     plt.subplot(3,1,3)
#     plt.plot(distances_to_goal, lin_speed, linewidth=1.5, color=[0.0, 0.3647, 0.9686], label='U')
#     plt.ylabel(r'Speed [$m/s$]')
#     plt.legend()
#     plt.xlabel(r'Distance to $\boldsymbol{\eta}_{\text{dock}}$ [m]')
#     # plt.title('Total Linear Speed vs. Distance to Docking Pose')
#     plt.gca().invert_xaxis()
#     plt.grid(True)

#     plt.savefig(os.path.join(MPC_OUTPUT_DIR,'mpc_head_yawRate_speed.pdf'), bbox_inches='tight')

#     # plt.show()
#     return

def test_interpolation():
    # ----------
    init_vel_body = np.array([1, 0, 0])
    # ----------
    
    dt = 0.1 # interpolation timestep
    dt_sim_min = 0.10
    dt_sim_max = 0.11
    t_end = 100
    
    # Load interpolated trajectory
    sim_times = [0]
    while sim_times[-1] < t_end:
        jittered_dt = np.random.uniform(dt_sim_min, dt_sim_max)
        next_t = sim_times[-1] + jittered_dt
        if next_t > t_end:
            break
        sim_times.append(next_t)
    np.array(sim_times)

    mpc_data = load_mpc_trajectories(MPC_OUTPUT_DIR)
    opt_traj = mpc_data[0]  # take the first stored run
    
    # Heading relative ref -> relative north (-pi, pi]. TODO: remove when data is transformed before storing data in run function
    # opt_traj.state[:, 2] = np.array([inf2pipi(psi + WP_REF[2]) for psi in opt_traj.state[:, 2]])

    init_yaw = opt_traj.state[0, 2] # used for computing initial vessel velocity (NED)
    opt_traj.state[:, 2] = np.array([unwrap_angle(psi) for psi in opt_traj.state[:, 2]])
    
    init_vel_ned = body_to_ned_transform(psi=init_yaw)@init_vel_body
    opt_traj.state_dot = np.vstack([init_vel_ned, opt_traj.state_dot])  # include vessel velocity
    

    # Initialize interpolator
    interpolator = InterpolateMPCOutput(dt=dt, sequence=opt_traj)

    # Interpolated references
    interpolated_states = []
    interpolated_vels = []

    for t in sim_times:
        state, vel = interpolator.get_references(at_t=t)
        state[2] = unwrap_angle(state[2])   # unwrap angle
        interpolated_states.append(state)
        interpolated_vels.append(vel)

    interpolated_states = np.array(interpolated_states)
    interpolated_vels = np.array(interpolated_vels)


    # Plot interpolated vs MPC states
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    axs[0].plot(sim_times, interpolated_states[:, 0], '-', color='blue', linewidth=1.5, label='x_interp')
    # axs[0].plot(interpolator.t_vec, opt_traj.state[:, 0], '--', color='orange', linewidth=1, label='x_mpc')
    axs[0].set_ylabel('North (x)')
    axs[0].legend()
    axs[0].grid()

    axs[1].plot(sim_times, interpolated_states[:, 1], '-', color='blue', linewidth=1.5, label='y_interp')
    # axs[1].plot(interpolator.t_vec, opt_traj.state[:, 1], '--', color='orange', linewidth=1, label='y_mpc')
    axs[1].set_ylabel('East (y)')
    axs[1].legend()
    axs[1].grid()

    axs[2].plot(sim_times, np.rad2deg(interpolated_states[:, 2]), '-', color='blue', linewidth=1.5, label=r'$\psi$ interp')
    # axs[2].plot(interpolator.t_vec, np.rad2deg(opt_traj.state[:, 2]), '--', color='orange', linewidth=1, label=r'$\psi$ mpc')
    axs[2].set_ylabel('Heading [deg]')
    axs[2].set_xlabel('Time [s]')
    axs[2].legend()
    axs[2].grid()

    plt.suptitle('Interpolated vs. MPC States')
    plt.tight_layout()

    # Plot interpolated vs MPC velocities
    fig, axs = plt.subplots(3, 1, figsize=(10, 6), sharex=True)

    axs[0].plot(sim_times, interpolated_vels[:, 0], '-', color='blue', linewidth=1.5, label='xdot_interp')
    # axs[0].plot(interpolator.t_vec, opt_traj.state_dot[:, 0], '--', color='orange', linewidth=1.5, label='xdot_mpc')
    axs[0].set_ylabel('xdot (m/s)')
    axs[0].legend()
    axs[0].grid()

    axs[1].plot(sim_times, interpolated_vels[:, 1], '-', color='blue', linewidth=1.5, label='ydot_interp')
    # axs[1].plot(interpolator.t_vec, opt_traj.state_dot[:, 1], '--', color='orange', linewidth=1.5, label='ydot_mpc')
    axs[1].set_ylabel('ydot (m/s)')
    axs[1].legend()
    axs[1].grid()

    axs[2].plot(sim_times, np.rad2deg(interpolated_vels[:, 2]), '-', color='blue', linewidth=1.5, label='psidot_interp')
    # axs[2].plot(interpolator.t_vec, np.rad2deg(opt_traj.state_dot[:, 2]), '--', color='orange', linewidth=1.5, label='psidot_mpc')
    axs[2].set_ylabel('psidot (deg/s)')
    axs[2].set_xlabel('Time (s)')
    axs[2].legend()
    axs[2].grid()

    plt.suptitle('Interpolated vs. MPC Velocities')
    plt.tight_layout()
    # plt.savefig(os.path.join(MPC_OUTPUT_DIR, 'some_plot.pdf'), bbox_inches='tight')
    plt.show()




def test_compute_and_save_mpc_smoothness_metrics() -> None:
    mpc_output = load_mpc_trajectories(directory=MPC_OUTPUT_DIR)[0]
    poses = mpc_output.state
    velocities_ned = mpc_output.state_dot        
    velocities_body = np.array([body_to_ned_transform(psi=poses[i, 2]).T @ velocities_ned[i, :]
                                for i in range(len(velocities_ned))])

    metrics = []
    metrics_pose = compute_smoothness_metrics(signal=poses,
                                              dt=0.1,signal_type="pose",
                                              component_names=["x", "y", "psi"])
    metrics_velocity = compute_smoothness_metrics(signal=velocities_body,
                                                  dt=0.1, signal_type="nu",
                                                  component_names=["u", "v", "r"])
    metrics.extend(metrics_pose)
    metrics.extend(metrics_velocity)

    save_metrics_single_run(output_dir=MPC_OUTPUT_DIR,
                            metrics_list=metrics,
                            controller_type=CONTROLLER_TYPE,
                            batch=BATCH,
                            run_id=RUN_ID,
                            quay_type=QUAY_TYPE)
    return
    

def compute_smoothness_metrics(signal: np.ndarray, dt: float, signal_type: str,
                               component_names: list[str] = None) -> list[Metric]:
    """
    Computes and formats smoothness metrics into a list of Metric objects.
    
    Parameters:
        signal: np.ndarray
            Trajectory array of shape (T, D) where T = time, D = components
        dt: float
            Time step between samples
        signal_type: str
            Either "pose" or "velocity"
        component_names: list of str (optional)
            Custom names for components, e.g., ["x", "y", "psi"] or ["u", "v", "r"]

    Returns:
        List[Metric]
    """
    metrics_list = []
    D = signal.shape[1]

    if component_names is None:
        component_names = [f"component_{i}" for i in range(D)]

    for i in range(D):
        name = component_names[i]
        data = signal[:, i]

        d1 = np.diff(data) / dt
        d2 = np.diff(d1) / dt

        # Compute smoothness metrics
        rms_d1 = np.sqrt(np.mean(d1**2))
        rms_d2 = np.sqrt(np.mean(d2**2))
        total_variation = np.sum(np.abs(np.diff(data)))
        max_d1 = np.max(np.abs(d1))
        max_d2 = np.max(np.abs(d2))
        energy = np.sum(data**2) * dt

        # Add to list with consistent naming
        metrics_list.extend([
            Metric(f"{signal_type}_{name}_rms_derivative", rms_d1), # first derivative
            Metric(f"{signal_type}_{name}_rms_second_derivative", rms_d2),  #second derivative
            Metric(f"{signal_type}_{name}_total_variation", total_variation),
            Metric(f"{signal_type}_{name}_max_derivative", max_d1),
            Metric(f"{signal_type}_{name}_max_second_derivative", max_d2),
            Metric(f"{signal_type}_{name}_signal_energy", energy),
        ])

    return metrics_list