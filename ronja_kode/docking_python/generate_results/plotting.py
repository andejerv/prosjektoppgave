import matplotlib.pyplot as plt
import numpy as np
from typing import Dict, Any
from generate_results.utils import plot_box
from generate_results.types import ProcessedData, StatisticsCollection, WindData
from omegaconf import DictConfig, OmegaConf
from generate_results.utils import unwrap_angle
from scipy.interpolate import interp1d
import os
import pandas as pd

from generate_results.read_write_files import save_fig
# import seaborn as sns

FONTSIZE_LEGEND = 12
FONTSIZE_LEGEND_LARGER = 13

plt.rcParams.update({
    'axes.labelsize': 13,   # Axis labels
    'xtick.labelsize': 13,  # Tick labels x
    'ytick.labelsize': 13,  # Tick labels y
    # 'legend.fontsize': 10,  # Legend
    # 'font.size': 12         # Default font size
})

def plot_vessel_trajectory(data: ProcessedData, quay_type: str,
                           colors: DictConfig, quays_vessel_cfg: DictConfig,
                           controller_type: str, plot_contour=None) -> plt.Figure:
    """
    Plots the vessel trajectory with waypoints, reference trajectory, quay lines, and vessel pose at different time points.

    :param data: Dictionary containing trajectory and phase switch information.
    :return: matplotlib figure object containing the plot.
    """

    # Dimensions
    vessel_length = quays_vessel_cfg.vessel.length  # Length of the vessel in meters
    vessel_width = quays_vessel_cfg.vessel.width    # Width of the vessel in meters
    quay_thickness = quays_vessel_cfg.quays.thickness  # meters

    # Quay positions
    quays = np.array([
        quays_vessel_cfg.quays.front_segment.corner,  # Quay 1_0 - head-on
        quays_vessel_cfg.quays.front_segment.end,  # Quay 1_1 - head-on
        quays_vessel_cfg.quays.side_segment.end,  # Quay 2_0 - side
        quays_vessel_cfg.quays.side_segment.corner  # Quay 2_1 - side
    ])

    if quay_type == 'h_quay':
        # Only plot head-on quays (indices 0 and 1 in the `quays` array)
        quays_to_plot = quays[:2]
    elif quay_type == 'l_quay':
        # Plot all quays
        quays_to_plot = quays
    else:
        raise ValueError("Invalid value for quay_type. Expected 'h_quay' or 'l_quay'.")

    # Unpack data
    t_vec = data.t  # Time vector
    x_ned = data.eta[:, 0] # x positions
    y_ned = data.eta[:, 1]  # y positions
    psi_rad = data.eta[:, 2]  # Heading (yaw) [rad]
    x_d = data.eta_d[:, 0]  # Desired x positions
    y_d = data.eta_d[:, 1]  # Desired y positions
    psi_d = data.eta_d[:, 2]  # Desired headings [rad]
    t_switch = data.t_switch

    t_switch_indices = [np.argmin(np.abs(t_vec - switch_time)) for switch_time in t_switch]
    target_WPs = [data.target_wp[i-1] for i in t_switch_indices]
    
    waypoint_labels = [rf"$\mathbf{{WP_{{{i + 1}}}}}$" for i in range(len(t_switch_indices))][::-1]

    # Create the plot
    fig, ax = plt.subplots(figsize=(10, 5))    

    # Plot thick quay lines as filled polygons
    for i in range(0, len(quays_to_plot), 2):
        point1 = quays_to_plot[i]  # Quay i_0
        point2 = quays_to_plot[i+1]  # Quay i_1

        # Compute a perpendicular offset for the quay thickness
        dx = point2[1] - point1[1]  # Change in y (East)
        dy = point2[0] - point1[0]  # Change in x (North)
        length = (dx**2 + dy**2)**0.5

        # Normalize the offset vector
        offset_x = -dy / length * quay_thickness
        offset_y = dx / length * quay_thickness

        # Define the polygon for the thick quay (rectangle)
        quay_polygon = [
            [point1[1], point1[0]],  # Original start point
            [point2[1], point2[0]],  # Original end point
            [point2[1] + offset_x, point2[0] + offset_y],  # Offset end point
            [point1[1] + offset_x, point1[0] + offset_y],  # Offset start point
        ]

        # Fill the quay polygon
        ax.fill(
            [p[0] for p in quay_polygon],  # East (x) coordinates
            [p[1] for p in quay_polygon],  # North (y) coordinates
            color=colors.quay,
            alpha=0.8,  # Slight transparency for quay
            label='Quay' if i == 0 else None  # Add label only for the first quay
        )

    eta0_x = x_ned[0]
    eta0_y = y_ned[1]

    # Plot vessel box at the start and end pose with legend entry
    plot_box(ax, eta0_x, eta0_y, psi_rad[0], vessel_length, vessel_width, 'Vessel outline')
    plot_box(ax, x_ned[-1], y_ned[-1], psi_rad[-1], vessel_length, vessel_width)

    for i, t in enumerate(t_switch_indices):
       plot_box(ax, x_ned[t], y_ned[t], psi_rad[t], vessel_length, vessel_width)

    if plot_contour is not None:
        # Plot vessel every n timesteps
        for i in range(0, len(t_vec), plot_contour):
            plot_box(ax, x_ned[i], y_ned[i], psi_rad[i], vessel_length, vessel_width)

    # Plot vessel trajectory in the NED frame
    ax.plot(y_ned, x_ned, '-', color=colors.actual_traj, linewidth=1.5, label='Vessel trajectory')

    # Plot desired trajectory
    line, = ax.plot(y_d, x_d, '--', color=colors.desired_traj, linewidth=1.5, label='Reference trajectory')
    line.set_dashes([6, 4])  # [dash_length, space_length]

    # Plot waypoints/eta_0/eta_end/eta_ref (marked with black circles)
    if controller_type == 'pid':
        for i, (wp, label) in enumerate(zip(target_WPs, waypoint_labels)):
            x, y, _ = wp  # Extract x (north) and y (east) values
            if i == 0:  # plot WP2
                ax.plot(y, x, 'o', markerfacecolor='black', markeredgecolor='black', markersize=6, alpha=1)  # Plot the waypoint as a black circle ('ko')
                ax.text(y + 0.3, x + 0.2, label, fontsize=FONTSIZE_LEGEND, color=colors.wp)  # Label it WP_1, WP_2, ...
            else:   # plot WP1
                ax.plot(y, x, 'o', markerfacecolor='black', markeredgecolor='black', markersize=6, alpha=1)  # Plot the waypoint as a black circle ('ko')
                ax.text(y - 0.8, x - 2.7, label, fontsize=FONTSIZE_LEGEND, color=colors.wp)  # Label it WP_1, WP_2, ...
            # Plot docking point
            x = 87.53
            y = 494.87
            ax.plot(y, x, 'o', markerfacecolor='black', markeredgecolor='black', markersize=6, alpha=1)  # Plot the waypoint as a black circle ('ko')
            ax.text(y + 0.2, x + 0.8, r'$\boldsymbol{\eta}_{\text{dock}}$', fontsize=FONTSIZE_LEGEND, color=colors.wp)
    else: # mpc docking method
    # Plot docking point
        x, y, _ = target_WPs[0]
        # x = 87.53
        # y = 494.87
        ax.plot(y, x, 'o', markerfacecolor='black', markeredgecolor='black', markersize=6, alpha=1)  # Plot the waypoint as a black circle ('ko')
        ax.text(y+0.5, x + 1, r'$\boldsymbol{\eta}_{\text{dock}}$', fontsize=FONTSIZE_LEGEND, color=colors.wp)
        # if controller_type == 'mpc_pid':
    #     ax.plot(eta0_y, eta0_x, 'o', markerfacecolor='black', markeredgecolor='black', markersize=6, alpha=1)
    #     ax.text(eta0_y + 0.2, eta0_x + 0.1, r'$\mathbf{\eta_0}$', fontsize=10, color=colors.wp)
    #     eta_end_x = x_ned[-1]
    #     eta_end_y = y_ned[-1]
        # ax.plot(eta_end_y, eta_end_x, 'o', markerfacecolor='black', markeredgecolor='black', markersize=6, alpha=1)
        # ax.text(eta_end_y + 0.2, eta_end_x + 0.1, r'$\mathbf{\eta_{ref}}$', fontsize=10, color=colors.wp)

    # Set plot labels and legend
    ax.set_xlabel(r'East [$m$]', fontsize=10)
    ax.set_ylabel(r'North [$m$]', fontsize=10)
    ax.legend(loc='best', fontsize=10)
    ax.grid(True)
    # ax.axis('equal')  # Ensure equal scaling for both axes
    ax.set_aspect('equal')
    ax.set_xlim(480, 540)   # from right
    # ax.set_xlim(460, 520)   # from left
    ax.set_ylim(80, 140)
    ax.tick_params(axis='both', labelsize=10)

    return fig  # Return the figure object 


def plot_accel_components(data: ProcessedData, colors, controller_type: str) -> None:
    # Unpack data
    t_vec = data.t  # Time vector
    u_dot = data.accel_body[:, 0]
    v_dot = data.accel_body[:, 1]
    r_dot = data.accel_body[:, 2]

    t_switch = data.t_switch

    phase_switch_indices = np.where(~np.isnan(t_switch))[0]  # Indices where phase switches occur
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"$WP_{i + 1}$" for i in range(len(phase_switch_times))][::-1]

    # Create the plot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6), sharex=True)

    # Plot surge velocity
    ax1.plot(t_vec, u_dot, label=r'$\overset{\mathbf{\bullet}}{u}$', color=colors.actual_traj, linestyle='-')
    # ax1.set_title('North Position vs Time')
    # ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Acceleration'+ '\n' +r'[$m/s^2$]')
    ax1.legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGER)
    ax1.grid(True)
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax1.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        if controller_type == "mpc_pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, r'$\boldsymbol{\eta}_{\text{dock}}$', ha='center', va='bottom', fontsize=FONTSIZE_LEGEND_LARGER, color=colors.wp)
        elif controller_type == "pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, label, ha='center', va='bottom', fontsize=FONTSIZE_LEGEND_LARGER, color=colors.wp)

    # Plot sway velocity
    ax2.plot(t_vec, v_dot, label=r'$\overset{\mathbf{\bullet}}{v}$', color=colors.actual_traj, linestyle='-')
    # ax2.set_title('East Position vs Time')
    # ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Acceleration'+ '\n' + r'[$m/s^2$]')
    ax2.legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGER)
    ax2.grid(True)
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax2.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax2.text(t, ax2.get_ylim()[1] * 0.3, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Plot yaw rate (body)
    ax3.plot(t_vec, np.rad2deg(r_dot), label=r'$\overset{\mathbf{\bullet}}{r}$', color=colors.actual_traj, linestyle='-')
    # ax3.set_title('Heading (Yaw) vs Time')
    ax3.set_ylabel('Acceleration'+ '\n' + r' [$deg/s^2$]')
    ax3.set_xlabel(r'Time [$s$]')
    ax3.legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGER)
    ax3.grid(True)
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax3.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax3.text(t, ax3.get_ylim()[1] * 0.989, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Adjust layout to prevent overlap
    plt.tight_layout()
    # plt.subplots_adjust(hspace=0.15)  # adjust spacing between subplots
    return fig  # Return the figure object


def plot_control_components(data: ProcessedData, colors: DictConfig, controller_type: str) -> plt.Figure:
    # Unpack data
    t_vec = data.t  # Time vector
    Fx = data.control_forces_body[:, 0]
    Fy = data.control_forces_body[:, 1]
    Mz = data.control_forces_body[:, 2]

    t_switch = data.t_switch

    phase_switch_indices = np.where(~np.isnan(t_switch))[0]  # Indices where phase switches occur
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"$WP_{i + 1}$" for i in range(len(phase_switch_times))][::-1]

    # Create the plot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6), sharex=True)

    # Plot surge velocity
    ax1.plot(t_vec, Fx, label='X', color=colors.actual_traj, linestyle='-')
    # ax1.set_title('North Position vs Time')
    # ax1.set_xlabel('Time (s)')
    ax1.set_ylabel(r'Force [$N$]')
    ax1.legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGER)
    ax1.grid(True)
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax1.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        if controller_type == "mpc_pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, r'$\boldsymbol{\eta}_{\text{dock}}$', ha='center', va='bottom', fontsize=FONTSIZE_LEGEND_LARGER, color=colors.wp)
        elif controller_type == "pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, label, ha='center', va='bottom', fontsize=FONTSIZE_LEGEND_LARGER, color=colors.wp)

    # Plot sway force
    ax2.plot(t_vec, Fy, label='Y', color=colors.actual_traj, linestyle='-')
    # ax2.set_title('East Position vs Time')
    # ax2.set_xlabel('Time (s)')
    ax2.set_ylabel(r'Force [$N$]')
    ax2.legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGER)
    ax2.grid(True)
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax2.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax2.text(t, ax2.get_ylim()[1] * 0.3, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Plot yaw moment
    ax3.plot(t_vec, Mz, label='N', color=colors.actual_traj, linestyle='-')
    # ax3.set_title('Heading (Yaw) vs Time')
    ax3.set_ylabel(r'Moment [$Nm$]')
    ax3.set_xlabel(r'Time [$s$]')
    ax3.legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGER)
    ax3.grid(True)
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax3.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax3.text(t, ax3.get_ylim()[1] * 0.989, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Adjust layout to prevent overlap
    plt.tight_layout()
    # plt.subplots_adjust(hspace=0.15)  # adjust spacing between subplots
    return fig  # Return the figure object



def plot_wind_forces_body(data: WindData, colors: DictConfig, controller_type: str) -> plt.Figure:
    # Unpack data
    t_vec = data.t  # Time vector
    Fx = data.X
    Fy = data.Y
    Mz = data.N

    # Create the plot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6), sharex=True)

    # Plot wind forces acting in surge direction
    ax1.plot(t_vec, Fx, label=r'$X_\text{wind}$', color=colors.actual_traj, linestyle='-')
    ax1.set_ylabel(r'Force [$N$]')
    ax1.legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGER)
    ax1.grid(True)


    # Plot wind forces acting in sway direction
    ax2.plot(t_vec, Fy, label=r'$Y_\text{wind}$', color=colors.actual_traj, linestyle='-')
    ax2.set_ylabel(r'Force [$N$]')
    ax2.legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGER)
    ax2.grid(True)

    # Plot wind moments acting on yaw
    ax3.plot(t_vec, Mz, label=r'$N_\text{wind}$', color=colors.actual_traj, linestyle='-')
    ax3.set_ylabel(r'Moment [$Nm$]')
    ax3.set_xlabel(r'Time [$s$]')
    ax3.legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGER)
    ax3.grid(True)

    # Adjust layout to prevent overlap
    plt.tight_layout()
    # plt.subplots_adjust(hspace=0.15)  # adjust spacing between subplots
    return fig  # Return the figure object

def plot_heading_yawRate_speed(data: ProcessedData, colors: DictConfig, controller_type: str) -> plt.Figure:
    # Unpack data
    psi_rad = data.eta[:, 2]  # heading
    psi_d_rad = data.eta_d[:, 2]
    speed = np.linalg.norm(data.nu[:, :2], axis=1)
    speed_d = np.linalg.norm(data.nu_d[:, :2], axis=1)
    psi_dot = data.nu[:, 2]  # yaw rate
    psi_d_dot = data.nu_d[:, 2]

    yaw_deg = np.rad2deg(unwrap_angle(angle_rad=psi_rad))
    yaw_d_deg = np.rad2deg(unwrap_angle(angle_rad=psi_d_rad))

    # Compute x-axis: Euclidean distance to docking point
    # eta_dock = data.target_wp
    eta_dock = np.array([87.53, 494.87, 3.05084265359])
    eta = data.eta
    distances_to_goal = np.linalg.norm(eta[:, :2] - eta_dock[:2], axis=1)

    ref_heading_deg = np.rad2deg(unwrap_angle(angle_rad=eta_dock[2]))

    # Find phase switch times
    t_switch = data.t_switch
    phase_switch_indices = np.where(~np.isnan(t_switch))[0]
    t_switch_clean = t_switch[phase_switch_indices]

    # Interpolate distance values at each t_switch
    distance_interp = interp1d(data.t, distances_to_goal, kind='linear', fill_value='extrapolate')
    x_switch = distance_interp(t_switch_clean)

    # Waypoint labels
    waypoint_labels = [f"$WP_{i + 1}$" for i in range(len(x_switch))][::-1]

    # Create the plot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6), sharex=True)

    # Plot heading
    ax1.plot(distances_to_goal, yaw_deg, label=r'$\psi$', color=colors.actual_traj, linestyle='-')
    ax1.plot(distances_to_goal, yaw_d_deg, label=r'$\psi_d$', color=colors.desired_traj, linestyle='--')
    ax1.axhline(y=ref_heading_deg, linestyle='--', color='black', alpha=0.6, label=r'$\mathbf{\eta}_{\psi, \; \text{dock}}$')
    ax1.set_ylabel(r'Heading [$deg$]')
    ax1.legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGER)
    ax1.grid(True)
    # Mark the phase switch distances with vertical lines and labels
    for i, (x, label) in enumerate(zip(x_switch, waypoint_labels)):
        ax1.axvline(x=x, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        if controller_type == "mpc_pid":
            ax1.text(x, ax1.get_ylim()[1], r'$\boldsymbol{\eta}_{\text{dock}}$',
                    ha='center', va='bottom', fontsize=FONTSIZE_LEGEND_LARGER, color=colors.wp)
        elif controller_type == "pid":
            ax1.text(x, ax1.get_ylim()[1], label,
                    ha='center', va='bottom', fontsize=FONTSIZE_LEGEND_LARGER, color=colors.wp)
    plt.gca().invert_xaxis()

    # Plot yaw rate
    ax2.plot(distances_to_goal, psi_dot, label=r'$r$', color=colors.actual_traj, linestyle='-')
    ax2.plot(distances_to_goal, psi_d_dot, label=r'$r_d$', color=colors.desired_traj, linestyle='--')
    ax2.set_ylabel(r'Yaw rate [$deg/s$]')
    ax2.legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGER)
    ax2.grid(True)
    for x in x_switch:
        ax2.axvline(x=x, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
    plt.gca().invert_xaxis()

    # Plot speed
    ax3.plot(distances_to_goal, speed, label=r'$U$', color=colors.actual_traj, linestyle='-')
    ax3.plot(distances_to_goal, speed_d, label=r'$U_d$', color=colors.desired_traj, linestyle='--')
    ax3.set_ylabel(r'Speed [$m/s$]')
    ax3.set_xlabel(r'Distance to $\boldsymbol{\eta}_{\text{dock}}$ [$m$]')
    ax3.legend(loc='best', fontsize=FONTSIZE_LEGEND_LARGER)
    ax3.grid(True)
    for x in x_switch:
        ax3.axvline(x=x, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
    plt.gca().invert_xaxis()

    # Adjust layout to prevent overlap
    plt.tight_layout()
    return fig  # Return the figure object


def get_label(var_symbol: str, folder_name: str, controller_type:str) -> str:
    # Parse speed, direction, side
    parts = folder_name.split("_")

    if controller_type.lower() == "pid":
        speed = parts[0].replace("speed", "")
        direction = parts[1].replace("dir", "")
    else:   # mpc_pid
        speed = parts[1].replace("speed", "")
        direction = parts[2].replace("dir", "")
    
    # Build LaTeX label
    return fr"${{{var_symbol}}}_{{{speed}\,\mathrm{{m/s}},\,{direction}^\circ}}$"


def plot_accels_and_controls(output_dir: str, show_fig: bool, suffix: str, controller_type: str):
    folders = [f for f in os.listdir(output_dir)
               if os.path.isdir(os.path.join(output_dir, f)) and f.endswith(suffix)]

    # --- ACCELERATIONS ---
    fig_accel, ax_accel = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    for folder in folders:
        data_path = os.path.join(output_dir, folder, "logged_data.csv")
        if not os.path.isfile(data_path):
            continue
        df = pd.read_csv(data_path, skiprows=1)
        if "accel_body" in df.columns:
            acc_data = df["accel_body"].dropna().apply(lambda x: list(map(float, x.split(","))))
            acc_matrix = pd.DataFrame(acc_data.tolist(), columns=["u_dot", "v_dot", "r_dot"])
            t_acc = df["t"].iloc[:len(acc_matrix)]

            label0 = get_label(r"\dot{u}", folder, controller_type=controller_type)
            label1 = get_label(r"\dot{v}", folder, controller_type=controller_type)
            label2 = get_label(r"\dot{r}", folder, controller_type=controller_type)

            ax_accel[0].plot(t_acc, acc_matrix["u_dot"], label=label0, linewidth=1.0)
            ax_accel[1].plot(t_acc, acc_matrix["v_dot"], label=label1, linewidth=1.0)
            ax_accel[2].plot(t_acc, np.rad2deg(acc_matrix["r_dot"]), label=label2, linewidth=1.0)

    ax_accel[0].set_ylabel('Acceleration' + '\n' + r'[$m/s^2$]')
    ax_accel[1].set_ylabel('Acceleration' + '\n' + r'[$m/s^2$]')
    ax_accel[2].set_ylabel('Acceleration' + '\n' + r'[$deg/s^2$]')
    ax_accel[2].set_xlabel("Time [s]")
    for ax in ax_accel:
        ax.legend(fontsize=FONTSIZE_LEGEND_LARGER,loc='center left', bbox_to_anchor=(1.02, 0.5))
        ax.grid(True)
        if controller_type == "pid":
            ax.set_xlim(30, 80)
    # fig_accel.tight_layout(rect=[0, 0, 0.85, 1])
    fig_accel.tight_layout()

    if not show_fig:
        save_fig(output_dir=output_dir, fig_name=f"accelerations_body{suffix}", file_format="pdf")

    # --- CONTROL ACTIONS ---
    fig_ctrl, ax_ctrl = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    for folder in folders:
        data_path = os.path.join(output_dir, folder, "logged_data.csv")
        if not os.path.isfile(data_path):
            continue
        df = pd.read_csv(data_path, skiprows=1)
        if "control_forces_body" in df.columns:
            ctrl_data = df["control_forces_body"].dropna().apply(lambda x: list(map(float, x.split(","))))
            ctrl_matrix = pd.DataFrame(ctrl_data.tolist(), columns=["X", "Y", "N"])
            t_ctrl = df["t"].iloc[:len(ctrl_matrix)]

            label0 = get_label(r"X", folder, controller_type=controller_type)
            label1 = get_label(r"Y", folder, controller_type=controller_type)
            label2 = get_label(r"N", folder, controller_type=controller_type)

            ax_ctrl[0].plot(t_ctrl, ctrl_matrix["X"], label=label0, linewidth=1.0)
            ax_ctrl[1].plot(t_ctrl, ctrl_matrix["Y"], label=label1, linewidth=1.0)
            ax_ctrl[2].plot(t_ctrl, ctrl_matrix["N"], label=label2, linewidth=1.0)

    ax_ctrl[0].set_ylabel(r"Force [$N$]")
    ax_ctrl[1].set_ylabel(r"Force [$N$]")
    ax_ctrl[2].set_ylabel(r"Moment [$Nm$]")
    ax_ctrl[2].set_xlabel(r"Time [$s$]")
    for ax in ax_ctrl:
        # ax.legend(loc='best', fontsize=LEGEND_FONTSIZE)
        ax.legend(fontsize=FONTSIZE_LEGEND_LARGER, loc='center left', bbox_to_anchor=(1.02, 0.5))
        ax.grid(True)
        if controller_type == "pid":
            ax.set_xlim(30, 80)
    # fig_ctrl.tight_layout(rect=[0, 0, 0.85, 1])
    fig_ctrl.tight_layout()

    if not show_fig:
        save_fig(output_dir=output_dir, fig_name=f"control_body{suffix}", file_format="pdf")

    if show_fig:
        plt.show()
    else:
        plt.close("all")







def plot_traj_jerk_heatmat(data: ProcessedData, quay_type: str,
                           colors: DictConfig, quays_vessel_cfg: DictConfig) -> plt.Figure:
    """
    Plots the vessel trajectory with a heatmap based on the total linear jerk.
    """

    # Dimensions
    vessel_length = quays_vessel_cfg.vessel.length  # meters
    vessel_width = quays_vessel_cfg.vessel.width  # meters
    quay_thickness = quays_vessel_cfg.quays.thickness  # meters

    # Quay positions
    quays = np.array([
        quays_vessel_cfg.quays.front_segment.corner,  # Quay 1_0 - head-on
        quays_vessel_cfg.quays.front_segment.end,  # Quay 1_1 - head-on
        quays_vessel_cfg.quays.side_segment.end,  # Quay 2_0 - side
        quays_vessel_cfg.quays.side_segment.corner   # Quay 2_1 - side
    ])

    # Select quay type
    if quay_type == 'h_quay':
        quays_to_plot = quays[:2]  # Head-on quays
    elif quay_type == 'l_quay':
        quays_to_plot = quays  # All quays
    else:
        raise ValueError("Invalid quay_type. Expected 'h_quay' or 'l_quay'.")

    # Unpack data
    t_vec = data.t
    x_ned, y_ned, psi_rad = data.eta[:, 0], data.eta[:, 1], data.eta[:, 2]
    u_dot, v_dot = data.accel_body[:, 0], data.accel_body[:, 1]

    # Compute jerk
    u_ddot = np.gradient(u_dot, t_vec)
    v_ddot = np.gradient(v_dot, t_vec)
    total_linear_jerk = np.sqrt(u_ddot**2 + v_ddot**2)

    # Identify phase switch points & waypoints
    t_switch = data.t_switch
    t_switch_indices = [np.argmin(np.abs(t_vec - ts)) for ts in t_switch]
    waypoints = [data.target_wp[i-1] for i in t_switch_indices]
    waypoint_labels = [rf"$\mathbf{{WP_{{{i + 1}}}}}$" for i in range(len(t_switch_indices))][::-1]

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 8))

    # Plot quay structure
    for i in range(0, len(quays_to_plot), 2):
        p1, p2 = quays_to_plot[i], quays_to_plot[i+1]
        dx, dy = p2[1] - p1[1], p2[0] - p1[0]
        length = np.hypot(dx, dy)
        offset_x, offset_y = -dy / length * quay_thickness, dx / length * quay_thickness
        quay_polygon = [[p1[1], p1[0]], [p2[1], p2[0]], 
                        [p2[1] + offset_x, p2[0] + offset_y], 
                        [p1[1] + offset_x, p1[0] + offset_y]]
        ax.fill(*zip(*quay_polygon), color=colors.quay, alpha=0.8, label='Quay' if i == 0 else None)

    # Plot waypoints
    for i, (wp, label) in enumerate(zip(waypoints, waypoint_labels)):
        x, y, _ = wp  # Extract x (north) and y (east) values
        ax.plot(y, x, 'o', markerfacecolor='none', markeredgecolor='black', markersize=6, alpha=1)  # Plot the waypoint as a black circle ('ko')
        ax.text(y + 0.2, x + 0.1, label, fontsize=10, color=colors.wp)  # Label it WP_1, WP_2, ...
    nu0_x = x_ned[0]
    nu0_y = y_ned[1]
    ax.plot(nu0_y, nu0_x, 'o', markerfacecolor='none', markeredgecolor='black', markersize=6, alpha=1)
    ax.text(nu0_y + 0.2, nu0_x + 0.1, r'$\mathbf{\eta_0}$', fontsize=10, color=colors.wp)

    # Plot vessel trajectory with jerk heatmap
    sc = ax.scatter(y_ned, x_ned, c=total_linear_jerk, cmap='cividis_r', s=10, alpha=0.9)

    # Color bar for jerk magnitude
    cbar = plt.colorbar(sc, ax=ax)
    cbar.set_label(r'Total Linear Jerk (m/$s^3$)')

    # Plot vessel outline at key points
    plot_box(ax, x_ned[0], y_ned[0], psi_rad[0], vessel_length, vessel_width, 'Vessel')
    plot_box(ax, x_ned[-1], y_ned[-1], psi_rad[-1], vessel_length, vessel_width)
    for t in t_switch_indices:
        plot_box(ax, x_ned[t], y_ned[t], psi_rad[t], vessel_length, vessel_width)

    # Set labels & layout
    ax.set_xlabel('East position (m)')
    ax.set_ylabel('North position (m)')
    ax.grid(True, color='gray', alpha=0.3)
    ax.axis('equal')  # Ensure equal axis scaling
    ax.legend(loc='best', fontsize=8)

    return fig  # Return figure object


def plot_pose_components(data: ProcessedData, colors: DictConfig, controller_type: str) -> plt.Figure:
    # Unpack data
    t_vec = data.t  # Time vector
    x_ned = data.eta[:, 0] # x positions
    y_ned = data.eta[:, 1]  # y positions
    psi_rad = data.eta[:, 2]  # Heading (yaw) [rad]
    x_d = data.eta_d[:, 0]  # Desired x positions
    y_d = data.eta_d[:, 1]  # Desired y positions
    psi_d = data.eta_d[:, 2]  # Desired headings [rad]
    t_switch = data.t_switch

    phase_switch_indices = np.where(~np.isnan(t_switch))[0]  # Indices where phase switches occur
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"$WP_{i + 1}$" for i in range(len(phase_switch_times))][::-1]

    # Unwrap and convert to degrees
    psi_deg = np.rad2deg(unwrap_angle(angle_rad=psi_rad))
    psi_d_deg = np.rad2deg(unwrap_angle(angle_rad=psi_d))

    # Create the plot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # Plot x positions
    ax1.plot(t_vec, x_ned, label=f'$x^n$', color=colors.actual_traj, linestyle='-')
    ax1.plot(t_vec, x_d, label=f'$x^n_d$', color=colors.desired_traj, linestyle='--')
    # ax1.set_title('North Position vs Time')
    # ax1.set_xlabel('Time (s)')
    ax1.set_ylabel(r'North [$m$]')
    ax1.legend(loc='best')
    ax1.grid(True)
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax1.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        if controller_type == "mpc_pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, r'$\boldsymbol{\eta}_{\text{dock}}$', ha='center', va='bottom', fontsize=10, color=colors.wp)
        elif controller_type == "pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Plot y positions
    ax2.plot(t_vec, y_ned, label=f'$y^n$', color=colors.actual_traj, linestyle='-')
    ax2.plot(t_vec, y_d, label=f'$y^n_d$', color=colors.desired_traj, linestyle='--')
    # ax2.set_title('East Position vs Time')
    # ax2.set_xlabel('Time (s)')
    ax2.set_ylabel(r'East [$m$]')
    ax2.legend(loc='best')
    ax2.grid(True)
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax2.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax2.text(t, ax2.get_ylim()[1] * 0.3, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Plot heading (psi)
    ax3.plot(t_vec, psi_deg, label=f'$\psi$', color=colors.actual_traj, linestyle='-')
    ax3.plot(t_vec, psi_d_deg, label=f'$\psi_d$', color=colors.desired_traj, linestyle='--')
    # ax3.set_title('Heading (Yaw) vs Time')
    ax3.set_ylabel(r'Heading [$deg$]')
    ax3.set_xlabel(r'Time [$s$]')
    ax3.legend(loc='best')
    ax3.grid(True)
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax3.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax3.text(t, ax3.get_ylim()[1] * 0.989, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Adjust layout to prevent overlap
    plt.tight_layout()
    # plt.subplots_adjust(hspace=0.15)  # adjust spacing between subplots
    return fig  # Return the figure object

def plot_pose_component_errors(data: ProcessedData, colors: DictConfig, controller_type: str) -> plt.Figure:
    # Unpack data
    t_vec = data.t  # Time vector
    x_ned = data.eta[:, 0] # x positions
    y_ned = data.eta[:, 1]  # y positions
    psi_rad = data.eta[:, 2]  # Heading (yaw) [rad]
    x_d = data.eta_d[:, 0]  # Desired x positions
    y_d = data.eta_d[:, 1]  # Desired y positions
    psi_d = data.eta_d[:, 2]  # Desired headings [rad]
    t_switch = data.t_switch

    x_error = np.abs(x_d - x_ned)
    y_error = np.abs(y_d - y_ned)
    # psi_error = np.abs(psi_d - psi_rad)

    # Unwrap and convert to degrees
    psi_deg = np.rad2deg(unwrap_angle(psi_rad))
    psi_d_deg = np.rad2deg(unwrap_angle(psi_d))
    psi_error = np.abs(psi_d_deg - psi_deg)

    phase_switch_indices = np.where(~np.isnan(t_switch))[0]  # Indices where phase switches occur
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"$WP_{i + 1}$" for i in range(len(phase_switch_times))][::-1]

    # Create the plot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # Plot north error
    ax1.plot(t_vec, x_error, label=f'$|x^n_d - x^n|$', color=colors.actual_traj, linestyle='-')
    # ax1.set_title('North Position vs Time')
    # ax1.set_xlabel('Time (s)')
    ax1.set_ylabel(r'North [$m$]')
    ax1.legend()
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax1.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        if controller_type == "mpc_pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, r'$\boldsymbol{\eta}_{\text{dock}}$', ha='center', va='bottom', fontsize=10, color=colors.wp)
        elif controller_type == "pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Plot east error
    ax2.plot(t_vec, y_error, label=f'$|y^n_d - y^n|$', color=colors.actual_traj, linestyle='-')
    # ax2.set_title('East Position vs Time')
    # ax2.set_xlabel('Time (s)')
    ax2.set_ylabel(r'East [$m$]')
    ax2.legend()
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax2.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax2.text(t, ax2.get_ylim()[1] * 0.3, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Plot heading error
    ax3.plot(t_vec, psi_error, label=r'$|\psi_d - \psi|$', color=colors.actual_traj, linestyle='-')
    # ax3.set_title('Heading (Yaw) vs Time')
    ax3.set_ylabel(r'Heading [$deg$]')
    ax3.set_xlabel(r'Time [$s$]')
    ax3.legend()
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax3.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax3.text(t, ax3.get_ylim()[1] * 0.989, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Adjust layout to prevent overlap
    plt.tight_layout()
    # plt.subplots_adjust(hspace=0.15)  # adjust spacing between subplots
    return fig  # Return the figure object


def plot_pos_heading_errors(data: ProcessedData, colors: DictConfig, controller_type: str) -> plt.Figure:
    # Unpack data
    t_vec = data.t  # Time vector
    x_ned = data.eta[:, 0] # x positions
    y_ned = data.eta[:, 1]  # y positions
    psi_rad = data.eta[:, 2]  # Heading (yaw) [rad]
    x_d = data.eta_d[:, 0]  # Desired x positions
    y_d = data.eta_d[:, 1]  # Desired y positions
    psi_d = data.eta_d[:, 2]  # Desired headings [rad]
    t_switch = data.t_switch

    pos_error = np.sqrt((x_d - x_ned)**2 + (y_d - y_ned)**2)

    # Unwrap and convert to degrees
    psi_deg = np.rad2deg(unwrap_angle(psi_rad))
    psi_d_deg = np.rad2deg(unwrap_angle(psi_d))
    psi_error = np.abs(psi_d_deg - psi_deg)

    phase_switch_indices = np.where(~np.isnan(t_switch))[0]  # Indices where phase switches occur
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"$WP_{i + 1}$" for i in range(len(phase_switch_times))][::-1]

    # Create the plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Plot position error
    ax1.plot(t_vec, pos_error, label=r'||$\mathbf{\tilde{\eta}}_{xy}$||', color=colors.actual_traj, linestyle='-')
    # ax1.set_title('North Position vs Time')
    # ax1.set_xlabel('Time (s)')
    ax1.set_ylabel(r'Error [$m$]')
    ax1.legend()
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax1.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        if controller_type == "mpc_pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, r'$\boldsymbol{\eta}_{\text{dock}}$', ha='center', va='bottom', fontsize=10, color=colors.wp)
        elif controller_type == "pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Plot heading error
    ax2.plot(t_vec, psi_error, label=r'$|\tilde{\psi}|$', color=colors.actual_traj, linestyle='-')
    # ax2.set_title('East Position vs Time')
    # ax2.set_xlabel('Time (s)')
    ax2.set_ylabel(r'Error [$deg$]')
    ax2.legend()
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax2.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax2.text(t, ax2.get_ylim()[1] * 0.3, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Adjust layout to prevent overlap
    plt.tight_layout()
    # plt.subplots_adjust(hspace=0.15)  # adjust spacing between subplots
    return fig  # Return the figure object


def plot_nu(data: ProcessedData, colors, controller_type: str) -> plt.Figure:
    # Unpack data
    t_vec = data.t  # Time vector
    u = data.nu[:, 0]
    v = data.nu[:, 1]
    r = data.nu[:, 2]
    u_d = data.nu_d[:, 0]
    v_d = data.nu_d[:, 1]
    r_d = data.nu_d[:, 2]

    t_switch = data.t_switch

    phase_switch_indices = np.where(~np.isnan(t_switch))[0]  # Indices where phase switches occur
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"$WP_{i + 1}$" for i in range(len(phase_switch_times))][::-1]

    # Create the plot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # Plot surge velocity
    ax1.plot(t_vec, u, label=f'$u$', color=colors.actual_traj, linestyle='-')
    ax1.plot(t_vec, u_d, label=f'$u_d$', color=colors.desired_traj, linestyle='--')
    # ax1.set_title('North Position vs Time')
    # ax1.set_xlabel('Time (s)')
    ax1.set_ylabel(r'Velocity [$m/s$]')
    ax1.legend()
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax1.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        if controller_type == "mpc_pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, r'$\boldsymbol{\eta}_{\text{dock}}$', ha='center', va='bottom', fontsize=10, color=colors.wp)
        elif controller_type == "pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Plot sway velocity
    ax2.plot(t_vec, v, label=f'$v$', color=colors.actual_traj, linestyle='-')
    ax2.plot(t_vec, v_d, label=f'$v_d$', color=colors.desired_traj, linestyle='--')
    # ax2.set_title('East Position vs Time')
    # ax2.set_xlabel('Time (s)')
    ax2.set_ylabel(r'Velocity [$m/s$]')
    ax2.legend()
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax2.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax2.text(t, ax2.get_ylim()[1] * 0.3, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Plot yaw rate (body)
    ax3.plot(t_vec, np.rad2deg(r), label=f'$r$', color=colors.actual_traj, linestyle='-')
    ax3.plot(t_vec, np.rad2deg(r_d), label=f'$r_d$', color=colors.desired_traj, linestyle='--')
    # ax3.set_title('Heading (Yaw) vs Time')
    ax3.set_ylabel(r'Yaw rate [$deg/s$]')
    ax3.set_xlabel('Time [$s$]')
    ax3.legend()
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax3.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax3.text(t, ax3.get_ylim()[1] * 0.989, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Adjust layout to prevent overlap
    plt.tight_layout()
    # plt.subplots_adjust(hspace=0.15)  # adjust spacing between subplots
    return fig  # Return the figure object


def plot_speed_with_error(data: ProcessedData, colors, controller_type: str) -> plt.Figure:
    """
    """

    t_vec = data.t  # Time vector
    u = data.nu[:, 0]   # surge velocity
    v = data.nu[:, 1]   # sway velocity
    u_d = data.nu_d[:, 0]  # Surge velocity (reference)
    v_d = data.nu_d[:, 1]  # Sway velocity (reference)

    t_switch = data.t_switch
    phase_switch_indices = np.where(~np.isnan(t_switch))[0]  # Indices where phase switches occur
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"$WP_{i + 1}$" for i in range(len(phase_switch_times))][::-1]

    speed = np.sqrt(u**2 + v**2)  # Vessel total speed
    speed_d = np.sqrt(u_d**2 + v_d**2)  # Desired total speed
    error_speed = np.abs(speed - speed_d)

    # Create a figure with 2 subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Plot Surge Velocity (u) on the first subplot (ax1)
    ax1.plot(t_vec, speed, '-', color=colors.actual_traj, linewidth=1.5, label='Vessel speed')
    ax1.plot(t_vec, speed_d, '--', color=colors.desired_traj, linewidth=1.5, label='Reference speed')

    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax1.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        if controller_type == "mpc_pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, r'$\boldsymbol{\eta}_{\text{dock}}$', ha='center', va='bottom', fontsize=10, color=colors.wp)
        elif controller_type == "pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    ax1.set_ylabel('Speed (m/s)')
    ax1.legend(loc='best')
    ax1.grid(True, color='gray', alpha=0.3)

    # Plot the speed error on the second subplot (ax2)
    ax2.plot(t_vec, error_speed, '-', color=colors.control_error, linewidth=1.5, label='Speed error')

    # Mark the phase switch times with vertical lines and waypoint labels on ax2
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax2.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax2.text(t, ax2.get_ylim()[1] * 0.935, label, ha='center', va='bottom', fontsize=10, color=colors["wp"])

    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (m/s)')
    ax2.legend(loc='best')
    ax2.grid(True, color='gray', alpha=0.3)

    plt.tight_layout()

    return fig  # Return the figure object

def plot_eta_dot(data: ProcessedData, colors, controller_type: str) -> plt.Figure:
    # Unpack data
    t_vec = data.t  # Time vector
    x_dot = data.eta_dot[:, 0]
    y_dot = data.eta_dot[:, 1]
    psi_dot = data.eta_dot[:, 2]
    x_d_dot = data.eta_d_dot[:, 0]
    y_d_dot = data.eta_d_dot[:, 1]
    psi_d_dot = data.eta_d_dot[:, 2]

    t_switch = data.t_switch

    phase_switch_indices = np.where(~np.isnan(t_switch))[0]  # Indices where phase switches occur
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"$WP_{i + 1}$" for i in range(len(phase_switch_times))][::-1]

    # Create the plot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # Plot surge velocity
    ax1.plot(t_vec, x_dot, label=r'$\overset{\mathbf{\bullet}}{x}$', color=colors.actual_traj, linestyle='-')
    ax1.plot(t_vec, x_d_dot, label=r'$\overset{\mathbf{\bullet}}{x}_d$', color=colors.desired_traj, linestyle='--')
    # ax1.set_title('North Position vs Time')
    # ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('North velocity (m/s)')
    ax1.legend()

    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax1.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        if controller_type == "mpc_pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, r'$\boldsymbol{\eta}_{\text{dock}}$', ha='center', va='bottom', fontsize=10, color=colors.wp)
        elif controller_type == "pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Plot sway velocity
    ax2.plot(t_vec, y_dot, label=r'$\overset{\mathbf{\bullet}}{y}$', color=colors.actual_traj, linestyle='-')
    ax2.plot(t_vec, y_d_dot, label=r'$\overset{\mathbf{\bullet}}{y}_d$', color=colors.desired_traj, linestyle='--')
    # ax2.set_title('East Position vs Time')
    # ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('East velocity (m/s)')
    ax2.legend()
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax2.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax2.text(t, ax2.get_ylim()[1] * 0.3, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Plot yaw rate (body)
    ax3.plot(t_vec, np.rad2deg(psi_dot), label=r'$\overset{\mathbf{\bullet}}{\psi}$', color=colors.actual_traj, linestyle='-')
    ax3.plot(t_vec, np.rad2deg(psi_d_dot), label=r'$\overset{\mathbf{\bullet}}{\psi}_d$', color=colors.desired_traj, linestyle='--')
    # ax3.set_title('Heading (Yaw) vs Time')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Yaw rate (°/s)')
    ax3.legend()
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax3.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax3.text(t, ax3.get_ylim()[1] * 0.989, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Adjust layout to prevent overlap
    plt.tight_layout()
    # plt.subplots_adjust(hspace=0.15)  # adjust spacing between subplots
    return fig  # Return the figure object


def plot_lin_angular_accel(data: ProcessedData, colors, controller_type: str) -> None:
    # Unpack data
    t_vec = data.t  # Time vector
    u_dot = data.accel_body[:, 0]
    v_dot = data.accel_body[:, 1]
    r_dot = data.accel_body[:, 2]

    t_switch = data.t_switch
    phase_switch_indices = np.where(~np.isnan(t_switch))[0]  # Indices where phase switches occur
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"$WP_{i + 1}$" for i in range(len(phase_switch_times))][::-1]

    lin_accel = np.sqrt(u_dot**2 + v_dot**2)

    # Create a figure with 2 subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    # Plot Surge Velocity (u) on the first subplot (ax1)
    ax1.plot(t_vec, lin_accel, '-', color=colors.actual_traj, linewidth=1.5, label=r'$\overset{\mathbf{\bullet}}{U}$')

    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax1.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        if controller_type == "mpc_pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, r'$\boldsymbol{\eta}_{\text{dock}}$', ha='center', va='bottom', fontsize=12, color=colors.wp)
        elif controller_type == "pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    ax1.set_ylabel(r'Acceleration [$m/s^2$]')
    ax1.legend(loc='best', fontsize=FONTSIZE_LEGEND)
    ax1.grid(True)

    # Plot the speed error on the second subplot (ax2)
    ax2.plot(t_vec, np.rad2deg(r_dot), '-', color=colors.actual_traj, linewidth=1.5, label=r'$\overset{\mathbf{\bullet}}{r}$')

    # Mark the phase switch times with vertical lines and waypoint labels on ax2
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax2.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax2.text(t, ax2.get_ylim()[1] * 0.935, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    ax2.set_ylabel(r'Acceleration [$deg/s^2$]')
    ax2.set_xlabel(r'Time [$s$]')
    ax2.legend(loc='best', fontsize=FONTSIZE_LEGEND)
    ax2.grid(True)

    plt.tight_layout()

    return fig  # Return the figure object
    


def plot_lin_angular_control(data: ProcessedData, colors: DictConfig, controller_type: str) -> plt.Figure:
    # Unpack data
    t_vec = data.t  # Time vector
    Fx = data.control_forces_body[:, 0]
    Fy = data.control_forces_body[:, 1]
    Mz = data.control_forces_body[:, 2]

    t_switch = data.t_switch
    phase_switch_indices = np.where(~np.isnan(t_switch))[0]  # Indices where phase switches occur
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"$WP_{i + 1}$" for i in range(len(phase_switch_times))][::-1]

    lin_control = np.sqrt(Fx**2 + Fy**2)

    # Create a figure with 2 subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    # Plot Surge Velocity (u) on the first subplot (ax1)
    ax1.plot(t_vec, lin_control, '-', color=colors.actual_traj, linewidth=1.5, label='Linear control force')

    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax1.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        if controller_type == "mpc_pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, r'$\boldsymbol{\eta}_{\text{dock}}$', ha='center', va='bottom', fontsize=12, color=colors.wp)
        elif controller_type == "pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, label, ha='center', va='bottom', fontsize=12, color=colors.wp)

    ax1.set_ylabel(r'Force [$N$]')
    ax1.legend(loc='best', fontsize=FONTSIZE_LEGEND)
    ax1.grid(True)

    # Plot the speed error on the second subplot (ax2)
    ax2.plot(t_vec, Mz, '-', color=colors.actual_traj, linewidth=1.5, label='Angular control moment')

    # Mark the phase switch times with vertical lines and waypoint labels on ax2
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax2.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        # ax2.text(t, ax2.get_ylim()[1] * 0.935, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    ax2.set_ylabel(r'Moment [$Nm$]')
    ax2.set_xlabel(r'Time [$s$]')
    ax2.legend(loc='best', fontsize=FONTSIZE_LEGEND)
    ax2.grid(True)

    plt.tight_layout()

    return fig  # Return the figure object


def plot_jerk_components(data: ProcessedData, colors: DictConfig, controller_type: str) -> plt.Figure:
    # Unpack data
    t_vec = data.t  # Time vector
    u_dot = data.accel_body[:, 0]  # Surge acceleration
    v_dot = data.accel_body[:, 1]  # Sway acceleration
    r_dot = data.accel_body[:, 2]  # Yaw acceleration

    t_switch = data.t_switch

    # Compute the jerk components (time derivative of acceleration)
    u_ddot = np.gradient(u_dot, t_vec)  # Surge jerk
    v_ddot = np.gradient(v_dot, t_vec)  # Sway jerk
    r_ddot = np.gradient(r_dot, t_vec)  # Yaw jerk

    # Find phase switch times and create waypoint labels
    phase_switch_indices = np.where(~np.isnan(t_switch))[0]  
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"$WP_{i + 1}$" for i in range(len(phase_switch_times))][::-1]

    # Create the plot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # Plot Surge Jerk
    ax1.plot(t_vec, u_ddot, label=r'$\overset{\mathbf{\cdot\cdot}}{u}$', color=colors.actual_traj, linestyle='-')
    ax1.set_ylabel(r'Surge jerk (m/$s^3$)')
    ax1.legend()
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax1.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        if controller_type == "mpc_pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, r'$\boldsymbol{\eta}_{\text{dock}}$', ha='center', va='bottom', fontsize=10, color=colors.wp)
        elif controller_type == "pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Plot Sway Jerk
    ax2.plot(t_vec, v_ddot, label=r'$\overset{\mathbf{\cdot\cdot}}{v}$', color=colors.actual_traj, linestyle='-')
    ax2.set_ylabel(r'Sway jerk (m/$s^3$)')
    ax2.legend()
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax2.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)

    # Plot Yaw Jerk
    ax3.plot(t_vec, np.rad2deg(r_ddot), label=r'$\overset{\mathbf{\cdot\cdot}}{r}$', color=colors.actual_traj, linestyle='-')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel(r'Yaw jerk (°/$s^3$)')
    ax3.legend()
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax3.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)

    # Adjust layout to prevent overlap
    plt.tight_layout()
    return fig  # Return the figure object


def plot_lin_angular_jerk(data: ProcessedData, colors: DictConfig, controller_type: str) -> plt.Figure:
    # Unpack data
    t_vec = data.t  # Time vector
    u_dot = data.accel_body[:, 0]  # Surge acceleration
    v_dot = data.accel_body[:, 1]  # Sway acceleration
    r_dot = data.accel_body[:, 2]  # Yaw acceleration

    t_switch = data.t_switch

    # Compute the jerk components (time derivative of acceleration)
    u_ddot = np.gradient(u_dot, t_vec)  # Surge jerk
    v_ddot = np.gradient(v_dot, t_vec)  # Sway jerk
    r_ddot = np.gradient(r_dot, t_vec)  # Yaw jerk

    # Compute the total linear jerk: sqrt(u_ddot² + v_ddot²)
    linear_jerk = np.sqrt(u_ddot**2 + v_ddot**2)

    # Find phase switch times and create waypoint labels
    phase_switch_indices = np.where(~np.isnan(t_switch))[0]  
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"$WP_{i + 1}$" for i in range(len(phase_switch_times))][::-1]

    # Create the plot with two subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    # Plot Total Linear Jerk
    ax1.plot(t_vec, linear_jerk, label='Linear jerk', color=colors.actual_traj, linestyle='-')
    ax1.set_ylabel(r'Jerk (m/$s^3$)')
    ax1.legend()
    # Mark the phase switch times with vertical lines and waypoint labels
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax1.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)
        if controller_type == "mpc_pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, r'$\boldsymbol{\eta}_{\text{dock}}$', ha='center', va='bottom', fontsize=10, color=colors.wp)
        elif controller_type == "pid":
            ax1.text(t, ax1.get_ylim()[1] * 1, label, ha='center', va='bottom', fontsize=10, color=colors.wp)

    # Plot Angular Jerk
    ax2.plot(t_vec, np.rad2deg(r_ddot), label='Angular jerk', color=colors.actual_traj, linestyle='-')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel(r'Jerk (°/$s^3$)')
    ax2.legend()
    for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
        ax2.axvline(x=t, color=colors.wp, linestyle='--', linewidth=1.5, ymax=1)

    # Adjust layout to prevent overlap
    plt.tight_layout()
    return fig  # Return the figure object















def plot_error_bars():
    """
    POSSIBLY implement this
    IF implemented, it should plot the mean and standard deviation for each batch
    One plot pr metric (possibly)
    """
    pass



def plot_statistics_boxplots(statistics_collection: StatisticsCollection) -> Dict[str, plt.Figure]:
    # TODO: fix this plotting - each box should represent the metric data from all runs in one batch !!! 
    # One plot pr metric (possibly)
    """
    Generate one box plot per metric type, marking each box with its respective controller and batch.
    
    Args:
        statistics_collection (StatisticsCollection): The structured statistics data.

    Returns:
        Dict[str, plt.Figure]: A dictionary of figures in the format {metric_type: figure}.
    """
    # A dictionary to store figures, one for each metric type
    figures = {}

    # Collect data for all metrics
    metric_data = {}

    for controller, batches in statistics_collection.statistic_collection.items():
        for batch, metrics in batches.items():
            for metric_type, stats in metrics.items():
                if metric_type not in metric_data:
                    metric_data[metric_type] = []
                # Append a tuple (controller, batch, mean, std dev) for each metric
                metric_data[metric_type].append((controller, batch, stats.mu, stats.sigma))

    # Create one figure for each metric type
    for metric_type, data in metric_data.items():
        # Data preparation for the box plot
        boxplot_data = []
        labels = []

        for controller, batch, mean, std_dev in data:
            boxplot_data.append([mean, std_dev])  # Use both mean and std as box plot data points
            labels.append(f"{controller}-{batch}")  # Label as "controller-batch"

        # Create the box plot
        fig, ax = plt.subplots(figsize=(10, 7))
        ax.boxplot(boxplot_data, vert=True, patch_artist=True, labels=labels)

        # Add titles and labels
        ax.set_title(f"Box Plot for Metric: {metric_type}", fontsize=16)
        ax.set_xlabel("Controller-Batch", fontsize=12)
        ax.set_ylabel("Values (Mean & Std Dev)", fontsize=12)
        ax.grid(axis='y', linestyle='--', alpha=0.7)

        # Rotate x-axis labels for better readability
        plt.setp(ax.xaxis.get_majorticklabels(), rotation=45, ha='right')

        # Store the figure in the dictionary
        figures[metric_type] = fig

    return figures

