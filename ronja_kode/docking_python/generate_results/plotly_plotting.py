import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np
from typing import Dict
from omegaconf import DictConfig
from generate_results.types import ProcessedData
from generate_results.types import StatisticsCollection
from docking_algorithms.utils.computations import rotated_box_vertices


def plot_traj_with_vessel_outline(data: ProcessedData, quay_type: str,
                                  colors: DictConfig, run_name: str,
                                  quays_vessel_cfg: DictConfig):
    
    # Dimensions
    quay_thickness = quays_vessel_cfg.quays.thickness  # meters
    vessel_length = quays_vessel_cfg.vessel.length
    vessel_width = quays_vessel_cfg.vessel.width

    # Quay positions
    quays = np.array([
        [81.73730603916813, 498.8836394557981],  # Quay 1_0 - head-on
        [80.92167664357352, 489.9206740598456],  # Quay 1_1 - head-on
        [93.61134802940731, 496.95461073282974],  # Quay 2_0 - side
        [81.66072750147063, 498.04211659362255]   # Quay 2_1 - side
    ])

    head_on_quay = np.array([quays_vessel_cfg.quays.front_segment.corner,
                             quays_vessel_cfg.quays.front_segment.end])
    side_quay = np.array([quays_vessel_cfg.quays.side_segment.end,
                          quays_vessel_cfg.quays.side_segment.corner])

    # Unpack data
    t_vec = data.t  # Time vector
    x_ned, y_ned, psi_rad = data.eta[:, 0], data.eta[:, 1], data.eta[:, 2]
    x_d, y_d = data.eta_d[:, 0], data.eta_d[:, 1]
    t_switch = data.t_switch

    t_switch_indices = [np.argmin(np.abs(t_vec - switch_time)) for switch_time in t_switch]
    target_WPs = np.array([np.array([data.target_wp[i-1][0], data.target_wp[i-1][1]]) for i in t_switch_indices])

    fig = go.Figure()
    
    n_timesteps = len(x_ned)
    vessel_traj = np.array([x_ned, y_ned])
    ref_traj = np.array([x_d, y_d])
    vessel_outline_traj = np.array([
        rotated_box_vertices(center_north=x_ned[k], center_east=y_ned[k],
                             yaw_rad=psi_rad[k], length=vessel_length, width=vessel_width)
                             for k in range(n_timesteps)
    ])
    
    # Add traces
    fig.add_trace(go.Scatter(
        x=[],
        y=[],
        mode="lines",
        name="vessel trajectory"
    ))

    fig.add_trace(go.Scatter(
        x=[],
        y=[],
        mode="lines",
        name="reference trajectory"
    ))
    
    fig.add_trace(go.Scatter(
        x=[],
        y=[],
        mode="lines",
        name="ship shape",
    ))

    # Waypoints trace
    fig.add_trace(go.Scatter(x=[], y=[], mode='markers', name='waypoints'))
        
    # Quay lines trace     
    fig.add_trace(go.Scatter(x=[], y=[],
                             mode='lines', name=f'frontquay'))
    
    if quay_type == "l_quay":
        fig.add_trace(go.Scatter(x=[], y=[],
                                mode='lines', name=f'sidequay'))
    
    # Set up frames
    frame_data: list[go.Frame] = [[] for _ in range(n_timesteps)]
    
    for i in range(n_timesteps):
        frame_data[i].append(go.Scatter(
            x=vessel_traj[1, :i+1], y=vessel_traj[0, :i+1],
            mode="lines", line=dict(color='red'), name="vessel trajectory"))
        
        frame_data[i].append(go.Scatter(
            x=ref_traj[1, :i+1], y=ref_traj[0, :i+1],
            mode="lines", line=dict(color='blue', dash='dash'), name="reference trajectory"))
    
        frame_data[i].append(go.Scatter(
            x=vessel_outline_traj[i, :, 0], y=vessel_outline_traj[i, :, 1],
            mode="lines", line=dict(color='black', width=1), name="ship shape"))
        
        frame_data[i].append(go.Scatter(
            x=target_WPs[:, 1], y=target_WPs[:, 0],
            mode='markers', marker=dict(color='black'), name="waypoints"))
        
        frame_data[i].append(go.Scatter(
            x=head_on_quay[:, 1], y=head_on_quay[:, 0],
            mode='lines', line=dict(color='orange', width=1), name="frontquay"))
        
        if quay_type == "l_quay":
            frame_data[i].append(go.Scatter(
                x=side_quay[:, 1], y=side_quay[:, 0],
                mode='lines', line=dict(color='orange', width=1), name="sidequay"))
    
    fig.frames = [go.Frame(data=frame, name=str(i)) for i, frame in enumerate(frame_data)]
    
    # fig.update_layout(title=f'{run_name}',
    #                   xaxis_title='East Position (m)',
    #                   yaxis_title='North Position (m)',
    #                   xaxis=dict(title="X-axis", scaleanchor="y"),  # Ensures equal scaling
    #                   yaxis=dict(title="Y-axis", range=[80, 130]),
    #                   showlegend=True, autosize=True)
    
    fig.update_layout(
        title=f'{run_name}',
        xaxis_title='East Position (m)',
        yaxis_title='North Position (m)',
        xaxis=dict(title="X-axis", scaleanchor="y"),  # Ensures equal scaling
        yaxis=dict(title="Y-axis", range=[80, 130]),
        showlegend=True, autosize=True,
        updatemenus=[{
            'buttons': [
                {
                    'args': [
                        None,
                        {
                            'frame': {'duration': 0, 'redraw': True},
                            'fromcurrent': True
                        }
                    ],
                    'label': 'Play',
                    'method': 'animate'
                },
                {
                    'args': [
                        [None],
                        {
                            'frame': {'duration': 0, 'redraw': True},
                            'mode': 'immediate',
                            'transition': {'duration': 0}
                        }
                    ],
                    'label': 'Pause',
                    'method': 'animate'
                }
            ],
            'direction': 'left',
            'pad': {'r': 10, 't': 87},
            'showactive': False,
            'type': 'buttons',
            'x': 0.1,
            'xanchor': 'right',
            'y': 0,
            'yanchor': 'top'
        }],
        sliders=[{
            'active': 0,
            'yanchor': 'top',
            'xanchor': 'left',
            'currentvalue': {
                'font': {'size': 10},
                'prefix': 'Time step: ',
                'visible': True,
                'xanchor': 'right'
            },
            'transition': {'duration': 0},
            'pad': {'b': 10, 't': 50},
            'len': 0.9,
            'x': 0.1,
            'y': 0,
            'steps': [{
                'args': [
                    [str(k)],
                    {
                        'frame': {'duration': 0, 'redraw': True},
                        'mode': 'immediate'
                    }
                ],
                'label': str(k),
                'method': 'animate',
            } for k in range(0, n_timesteps)]
        }]
    )
    return fig
 

def plot_pose_components(data, colors):
    """
    Converts matplotlib's plot_pose_components to an interactive Plotly figure.
    """
    t_vec = data.t
    x_ned, y_ned, psi_rad = data.eta[:, 0], data.eta[:, 1], data.eta[:, 2]
    x_d, y_d, psi_d = data.eta_d[:, 0], data.eta_d[:, 1], data.eta_d[:, 2]
    t_switch = data.t_switch

    psi_deg = np.rad2deg(psi_rad)
    psi_d_deg = np.rad2deg(psi_d)

    phase_switch_times = t_switch[~np.isnan(t_switch)]

    fig = make_subplots(rows=3, cols=1, shared_xaxes=True, subplot_titles=[
        "North Position vs Time", "East Position vs Time", "Heading (Yaw) vs Time"
    ])

    # X component (North position)
    fig.add_trace(go.Scatter(x=t_vec, y=x_ned, mode='lines', name="x^n", line=dict(color=colors.actual_traj)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=x_d, mode='lines', name="x^n_d", line=dict(color=colors.desired_traj, dash='dash')), row=1, col=1)

    # Y component (East position)
    fig.add_trace(go.Scatter(x=t_vec, y=y_ned, mode='lines', name="y^n", line=dict(color=colors.actual_traj)), row=2, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=y_d, mode='lines', name="y^n_d", line=dict(color=colors.desired_traj, dash='dash')), row=2, col=1)

    # Heading (Yaw)
    fig.add_trace(go.Scatter(x=t_vec, y=psi_deg, mode='lines', name="ψ", line=dict(color=colors.actual_traj)), row=3, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=psi_d_deg, mode='lines', name="ψ_d", line=dict(color=colors.desired_traj, dash='dash')), row=3, col=1)

    # Vertical lines for phase switches
    for t in phase_switch_times:
        for i in range(1, 4):
            fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'), row=i, col=1)

    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
    fig.update_yaxes(title_text="Position (m)", row=1, col=1)
    fig.update_yaxes(title_text="Position (m)", row=2, col=1)
    fig.update_yaxes(title_text="Heading (degrees)", row=3, col=1)
    fig.update_layout(height=800, width=900, showlegend=True)

    return fig

def plot_pose_component_errors(data, colors):
    """
    Converts matplotlib's plot_pose_component_errors to an interactive Plotly figure.
    """
    t_vec = data.t
    x_ned, y_ned, psi_rad = data.eta[:, 0], data.eta[:, 1], data.eta[:, 2]
    x_d, y_d, psi_d = data.eta_d[:, 0], data.eta_d[:, 1], data.eta_d[:, 2]
    t_switch = data.t_switch

    x_error, y_error, psi_error = np.abs(x_d - x_ned), np.abs(y_d - y_ned), np.abs(psi_d - psi_rad)
    phase_switch_times = t_switch[~np.isnan(t_switch)]

    fig = make_subplots(rows=3, cols=1, shared_xaxes=True, subplot_titles=[
        "North Position Error", "East Position Error", "Heading Error"
    ])

    fig.add_trace(go.Scatter(x=t_vec, y=x_error, mode='lines', name="|x^n_d - x^n|", line=dict(color=colors.actual_traj)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=y_error, mode='lines', name="|y^n_d - y^n|", line=dict(color=colors.actual_traj)), row=2, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=psi_error, mode='lines', name="|ψ_d - ψ|", line=dict(color=colors.actual_traj)), row=3, col=1)

    for t in phase_switch_times:
        for i in range(1, 4):
            fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'), row=i, col=1)

    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
    fig.update_yaxes(title_text="Error (m)", row=1, col=1)
    fig.update_yaxes(title_text="Error (m)", row=2, col=1)
    fig.update_yaxes(title_text="Error (rad)", row=3, col=1)
    fig.update_layout(height=800, width=900, showlegend=True)

    return fig

def plot_pos_heading_errors(data, colors):
    """
    Converts matplotlib's plot_pos_heading_errors to an interactive Plotly figure.
    """
    t_vec = data.t
    x_ned, y_ned, psi_rad = data.eta[:, 0], data.eta[:, 1], data.eta[:, 2]
    x_d, y_d, psi_d = data.eta_d[:, 0], data.eta_d[:, 1], data.eta_d[:, 2]
    t_switch = data.t_switch

    pos_error = np.sqrt((x_d - x_ned) ** 2 + (y_d - y_ned) ** 2)
    heading_error = np.abs(psi_d - psi_rad)
    phase_switch_times = t_switch[~np.isnan(t_switch)]

    fig = make_subplots(rows=2, cols=1, shared_xaxes=True, subplot_titles=[
        "Position Error", "Heading Error"
    ])

    fig.add_trace(go.Scatter(x=t_vec, y=pos_error, mode='lines', name="Position Error", line=dict(color=colors.actual_traj)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=heading_error, mode='lines', name="|ψ_d - ψ|", line=dict(color=colors.actual_traj)), row=2, col=1)

    for t in phase_switch_times:
        for i in range(1, 3):
            fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'), row=i, col=1)

    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_yaxes(title_text="Error (m)", row=1, col=1)
    fig.update_yaxes(title_text="Error (rad)", row=2, col=1)
    fig.update_layout(height=600, width=900, showlegend=True)

    return fig

def plot_nu(data: ProcessedData, colors: DictConfig):
    t_vec = data.t
    u, v, r = data.nu[:, 0], data.nu[:, 1], data.nu[:, 2]
    u_d, v_d, r_d = data.nu_d[:, 0], data.nu_d[:, 1], data.nu_d[:, 2]
    
    t_switch = data.t_switch
    phase_switch_times = t_switch[np.where(~np.isnan(t_switch))[0]]
    
    fig = make_subplots(rows=3, cols=1, shared_xaxes=True, subplot_titles=[
        "Surge", "Sway", "Yaw rate"
    ])
    
    # Plot surge velocity
    fig.add_trace(go.Scatter(x=t_vec, y=u, mode='lines', name='u', line=dict(color=colors.actual_traj)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=u_d, mode='lines', name='u_d', line=dict(color=colors.desired_traj, dash='dash')), row=1, col=1)
    
    # Plot sway velocity
    fig.add_trace(go.Scatter(x=t_vec, y=v, mode='lines', name='v', line=dict(color=colors.actual_traj)), row=2, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=v_d, mode='lines', name='v_d', line=dict(color=colors.desired_traj, dash='dash')), row=2, col=1)
    
    # Plot yaw rate
    fig.add_trace(go.Scatter(x=t_vec, y=r, mode='lines', name='r', line=dict(color=colors.actual_traj)), row=3, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=r_d, mode='lines', name='r_d', line=dict(color=colors.desired_traj, dash='dash')), row=3, col=1)
    
    # Phase switch markers
    for t in phase_switch_times:
        fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'))

    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
    fig.update_yaxes(title_text="Velocity (m/s)", row=1, col=1)
    fig.update_yaxes(title_text="Velocity (m/s)", row=2, col=1)  
    fig.update_yaxes(title_text="Velocity (rad/s)", row=3, col=1)    
    fig.update_layout(title='Velocity Components')
    return fig

def plot_speed_with_error(data: ProcessedData, colors: DictConfig):
    t_vec = data.t
    u, v = data.nu[:, 0], data.nu[:, 1]
    u_d, v_d = data.nu_d[:, 0], data.nu_d[:, 1]

    t_switch = data.t_switch
    phase_switch_times = t_switch[np.where(~np.isnan(t_switch))[0]]
    
    speed = np.sqrt(u**2 + v**2)
    speed_d = np.sqrt(u_d**2 + v_d**2)
    error_speed = np.abs(speed - speed_d)
    
    fig = make_subplots(rows=2, cols=1, shared_xaxes=True, subplot_titles=[
        "Linear speed", "Linear Speed Error"
    ])
    
    fig.add_trace(go.Scatter(x=t_vec, y=speed, mode='lines', name='Vessel Speed', line=dict(color=colors.actual_traj)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=speed_d, mode='lines', name='Reference Speed', line=dict(color=colors.desired_traj, dash='dash')), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=error_speed, mode='lines', name='Speed Error', line=dict(color=colors.control_error)), row=2, col=1)

    # Phase switch markers
    for t in phase_switch_times:
        fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'))
    
    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_yaxes(title_text="Speed (m/s)", row=1, col=1)
    fig.update_yaxes(title_text="Error (m/s)", row=2, col=1)    
    fig.update_layout(title='Speed and speed error')
    return fig

def plot_accel_components(data: ProcessedData, colors: DictConfig):
    t_vec = data.t
    u_dot, v_dot, r_dot = data.accel_body[:, 0], data.accel_body[:, 1], data.accel_body[:, 2]

    t_switch = data.t_switch
    phase_switch_times = t_switch[np.where(~np.isnan(t_switch))[0]]
    
    fig = make_subplots(rows=3, cols=1, shared_xaxes=True, subplot_titles=[
        "Surge acceleration", "Sway acceleration", "Yaw acceleration"
    ])

    
    fig.add_trace(go.Scatter(x=t_vec, y=u_dot, mode='lines', name='Surge Acceleration', line=dict(color=colors.actual_traj)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=v_dot, mode='lines', name='Sway Acceleration', line=dict(color=colors.actual_traj)), row=2, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=r_dot, mode='lines', name='Yaw Acceleration', line=dict(color=colors.actual_traj)), row=3, col=1)

    # Phase switch markers
    for t in phase_switch_times:
        fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'))
    
    
    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
    fig.update_yaxes(title_text="Acceleration (m/s^2)", row=1, col=1)
    fig.update_yaxes(title_text="Acceleration (m/s^2)", row=2, col=1)
    fig.update_yaxes(title_text="Acceleration (rad/s^2)", row=3, col=1)
    fig.update_layout(title='Acceleration Components')
    return fig

def plot_lin_angular_accel(data: ProcessedData, colors: DictConfig):
    t_vec = data.t
    u_dot, v_dot, r_dot = data.accel_body[:, 0], data.accel_body[:, 1], data.accel_body[:, 2]
    lin_accel = np.sqrt(u_dot**2 + v_dot**2)

    t_switch = data.t_switch
    phase_switch_times = t_switch[np.where(~np.isnan(t_switch))[0]]
    
    fig = make_subplots(rows=2, cols=1, shared_xaxes=True, subplot_titles=[
        "Linear acceleration", "Angular acceleration"
    ])
    
    fig.add_trace(go.Scatter(x=t_vec, y=lin_accel, mode='lines', name='Linear Acceleration', line=dict(color=colors.actual_traj)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=r_dot, mode='lines', name='Angular Acceleration', line=dict(color=colors.actual_traj)), row=2, col=1)

    # Phase switch markers
    for t in phase_switch_times:
        fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'))

    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
    fig.update_yaxes(title_text="Acceleration (m/s^2)", row=1, col=1)
    fig.update_yaxes(title_text="Acceleration (rad/s^2)", row=2, col=1)  
    fig.update_layout(title='Linear and Angular Acceleration')
    return fig

def plot_control_components(data: ProcessedData, colors: DictConfig):
    t_vec = data.t
    Fx, Fy, Mz = data.control_forces_body.T
    t_switch = data.t_switch

    phase_switch_indices = np.where(~np.isnan(t_switch))[0]
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"WP_{i + 1}" for i in range(len(phase_switch_times))][::-1]

    fig = make_subplots(rows=3, cols=1, shared_xaxes=True, subplot_titles=[
        "F_x", "F_y", "M_z"
    ])
    
    fig.add_trace(go.Scatter(x=t_vec, y=Fx, mode='lines', name='Surge Force (N)', line=dict(color=colors.actual_traj)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=Fy, mode='lines', name='Sway Force (N)', line=dict(color=colors.actual_traj)), row=2, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=Mz, mode='lines', name='Yaw Moment (Nm)', line=dict(color=colors.actual_traj)), row=3, col=1)
    
    # for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
    #     fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'))
    #     fig.add_annotation(x=t, y=max(Fx.max(), Fy.max(), Mz.max()), text=label, showarrow=False, font=dict(color=colors.wp))

    # Phase switch markers
    for t in phase_switch_times:
        fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'))

    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
    fig.update_yaxes(title_text="Force (N)", row=1, col=1)
    fig.update_yaxes(title_text="Force (N)", row=2, col=1)
    fig.update_yaxes(title_text="Moment (Nm)", row=3, col=1)
    fig.update_layout(title='Control Components')
    return fig

def plot_lin_angular_control(data: ProcessedData, colors: DictConfig):
    t_vec = data.t
    Fx, Fy, Mz = data.control_forces_body.T
    lin_control = np.sqrt(Fx**2 + Fy**2)
    t_switch = data.t_switch

    phase_switch_indices = np.where(~np.isnan(t_switch))[0]
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"WP_{i + 1}" for i in range(len(phase_switch_times))][::-1]

    fig = make_subplots(rows=2, cols=1, shared_xaxes=True, subplot_titles=[
        "Linear control force", "Angular control moment"
    ])
    fig.add_trace(go.Scatter(x=t_vec, y=lin_control, mode='lines', name='Linear Control Force', line=dict(color=colors.actual_traj)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=Mz, mode='lines', name='Angular Control Moment', line=dict(color=colors.actual_traj)), row=2, col=1)
    
    # for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
    #     fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'))
    #     fig.add_annotation(x=t, y=max(lin_control.max(), Mz.max()), text=label, showarrow=False, font=dict(color=colors.wp))

    # Phase switch markers
    for t in phase_switch_times:
        fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'))

    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_yaxes(title_text="Force (N)", row=1, col=1)
    fig.update_yaxes(title_text="Moment (Nm)", row=2, col=1)
    
    fig.update_layout(title='Linear & Angular Control')
    return fig

def plot_jerk_components(data: ProcessedData, colors: DictConfig):
    t_vec = data.t
    u_dot, v_dot, r_dot = data.accel_body.T
    t_switch = data.t_switch
    
    u_ddot = np.gradient(u_dot, t_vec)
    v_ddot = np.gradient(v_dot, t_vec)
    r_ddot = np.gradient(r_dot, t_vec)
    
    phase_switch_indices = np.where(~np.isnan(t_switch))[0]
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"WP_{i + 1}" for i in range(len(phase_switch_times))][::-1]
    
    fig = make_subplots(rows=3, cols=1, shared_xaxes=True, subplot_titles=[
        "Surge jerk", "Sway jerk", "Yaw jerk"
    ])
    fig.add_trace(go.Scatter(x=t_vec, y=u_ddot, mode='lines', name='Surge Jerk (m/s³)', line=dict(color=colors.actual_traj)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=v_ddot, mode='lines', name='Sway Jerk (m/s³)', line=dict(color=colors.actual_traj)), row=2, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=r_ddot, mode='lines', name='Yaw Jerk (rad/s³)', line=dict(color=colors.actual_traj)), row=3, col=1)
    
    # for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
    #     fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'))
    #     fig.add_annotation(x=t, y=max(u_ddot.max(), v_ddot.max(), r_ddot.max()), text=label, showarrow=False, font=dict(color=colors.wp))
    
    # Phase switch markers
    for t in phase_switch_times:
        fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'))

    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
    fig.update_yaxes(title_text="Jerk (m/s³)", row=1, col=1)
    fig.update_yaxes(title_text="Jerk (m/s³)", row=2, col=1)
    fig.update_yaxes(title_text="Jerk (rad/s³)", row=3, col=1)
    fig.update_layout(title='Jerk Components')
    return fig

def plot_lin_angular_jerk(data: ProcessedData, colors: DictConfig):
    t_vec = data.t
    u_dot, v_dot, r_dot = data.accel_body.T
    t_switch = data.t_switch

    u_ddot = np.gradient(u_dot, t_vec)
    v_ddot = np.gradient(v_dot, t_vec)
    r_ddot = np.gradient(r_dot, t_vec)
    linear_jerk = np.sqrt(u_ddot**2 + v_ddot**2)

    phase_switch_indices = np.where(~np.isnan(t_switch))[0]
    phase_switch_times = t_switch[phase_switch_indices]
    waypoint_labels = [f"WP_{i + 1}" for i in range(len(phase_switch_times))][::-1]

    fig = make_subplots(rows=2, cols=1, shared_xaxes=True, subplot_titles=[
        "Linear jerk", "Angular jerk"
    ])
    fig.add_trace(go.Scatter(x=t_vec, y=linear_jerk, mode='lines', name='Linear Jerk (m/s³)', line=dict(color=colors.actual_traj)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t_vec, y=r_ddot, mode='lines', name='Angular Jerk (rad/s³)', line=dict(color=colors.actual_traj)), row=2, col=1)
    
    # for i, (t, label) in enumerate(zip(phase_switch_times, waypoint_labels)):
    #     fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'))
    #     fig.add_annotation(x=t, y=max(linear_jerk.max(), r_ddot.max()), text=label, showarrow=False, font=dict(color=colors.wp))

    # Phase switch markers
    for t in phase_switch_times:
        fig.add_vline(x=t, line=dict(color=colors.wp, dash='dash'))

    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_yaxes(title_text="Jerk (m/s³)", row=1, col=1)
    fig.update_yaxes(title_text="Jerk (rad/s³)", row=2, col=1)
    fig.update_layout(title='Linear & Angular Jerk')
    return fig




# TODO: not tested
def plot_statistics_boxplots(statistics_collection: StatisticsCollection) -> Dict[str, go.Figure]:
    """
    Generate one box plot per metric type, marking each box with its respective controller and batch.
    
    Args:
        statistics_collection (StatisticsCollection): The structured statistics data.

    Returns:
        Dict[str, go.Figure]: A dictionary of Plotly figures in the format {metric_type: figure}.
    """
    figures = {}

    # Collect data for all metrics
    metric_data = {}

    for controller, batches in statistics_collection.statistic_collection.items():
        for batch, metrics in batches.items():
            for metric_type, stats in metrics.items():
                if metric_type not in metric_data:
                    metric_data[metric_type] = []
                metric_data[metric_type].append((controller, batch, stats.mu, stats.sigma))

    # Create one figure per metric type
    for metric_type, data in metric_data.items():
        fig = go.Figure()

        for controller, batch, mean, std_dev in data:
            label = f"{controller}-{batch}"
            fig.add_trace(go.Box(
                y=[mean, std_dev],  # Box plot based on mean & std dev
                name=label,
                boxpoints="all",  # Show all points
                jitter=0.3,  # Add slight jitter for better visualization
                pointpos=-1.8  # Offset for points
            ))

        # Update layout for readability
        fig.update_layout(
            title=f"Box Plot for Metric: {metric_type}",
            xaxis_title="Controller-Batch",
            yaxis_title="Values (Mean & Std Dev)",
            xaxis=dict(tickangle=-45),  # Rotate x-axis labels
            template="plotly_white",
            showlegend=False
        )

        figures[metric_type] = fig  # Store figure

    return figures