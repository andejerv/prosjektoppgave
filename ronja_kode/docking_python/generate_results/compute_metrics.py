import numpy as np
from generate_results.types import ProcessedData, Metric


def compute_all_metric_types(data: ProcessedData, controller_type: str) -> list[Metric]:
    metric_list = []
    metric_list.append(time_to_dock(data=data))
    metric_list.append(unsigned_pose_error_at_dock(data=data))
    metric_list.append(unsigned_pos_error_at_dock(data=data))
    metric_list.extend(velocity_and_speed_at_dock(data=data))
    metric_list.append(spacer())
    metric_list.append(IAE_pose_control_error(data=data, controller_type=controller_type))
    metric_list.append(IAE_pos_control_error(data=data, controller_type=controller_type))
    metric_list.append(IAE_velocity_control_error(data=data, controller_type=controller_type))
    metric_list.append(IAE_speed_control_error(data=data, controller_type=controller_type))
    metric_list.append(spacer())
    # metric_list.append(jerk_components_body(data=data))
    # metric_list.append(jerk_translational_body(data=data))
    metric_list.append(compute_energy_consumption(data=data))
    metric_list.append(control_effort(data=data))
    metric_list.append(spacer())
    metric_list.extend(acceleration_metrics(data=data))
    metric_list.append(spacer())
    metric_list.extend(jerk_metrics(data=data))
    return metric_list


def compute_energy_consumption(data: ProcessedData) -> Metric:
    """
    Compute the total energy consumption from start until arrival at wp1

    Parameters:
        Fx (array): Force in the x-direction.
        Fy (array): Force in the y-direction.
        Mz (array): Moment about the z-axis.
        u (array): Velocity in the x-direction.
        v (array): Velocity in the y-direction.
        r_rad (array): Angular velocity about the z-axis.
        t_vec (array): Time vector.

    Returns:
        float: Total energy consumption.

    Instantaneous power: P(t) = F(t)*v(t) (vectors)
    Energy: E = integral P(t) dt
    """
    control = data.control_forces_body
    Fx = control[:, 0]
    Fy = control[:, 1]
    Mz = control[:, 2]

    nu = data.nu
    u = nu[:, 0]
    v = nu[:, 1]
    r_rad = nu[:, 2]
 
    t_vec = data.t
    t_wp1 = data.t_switch[-1]   # time of arrival at wp1
    t_start_to_wp1 = t_vec[t_vec <= t_wp1] # time vector from start to arrival at wp1
    # Compute instantaneous energy
    instantaneous_energy = np.zeros_like(t_start_to_wp1)
    for i in range(len(t_start_to_wp1)):
        tau_i = np.array([Fx[i], Fy[i], Mz[i]])
        nu_i = np.array([u[i], v[i], r_rad[i]])
        instantaneous_energy[i] = np.abs(np.dot(tau_i, nu_i))   # power

    # Integrate energy over time using the np.trapzal rule
    total_energy = np.trapz(instantaneous_energy, t_start_to_wp1)
    energy_metric = Metric(metric_type="energy consumption until arrival at wp1", value=total_energy)

    return energy_metric


def time_to_dock(data: ProcessedData) -> Metric:
    t_vec = data.t_switch
    time_arrival_wp1 = t_vec[-1]
    time_metric = Metric(metric_type="time arrival", value=time_arrival_wp1)
    return time_metric

def control_effort(data: ProcessedData) -> Metric:
    """ "Smoothness" of control inputs """
    Fx = data.control_forces_body[:, 0]
    Fy = data.control_forces_body[:, 1]
    Mz = data.control_forces_body[:, 2]

    effort_integrand = Fx**2 + Fy**2 + Mz**2
    control_effort = np.trapz(effort_integrand, data.t)
    return Metric(metric_type="control effort (smoothness)", value=control_effort)

def jerk_metrics(data: ProcessedData) -> list[Metric]:
    dt = np.gradient(data.t)
    T = data.t[-1] - data.t[0]
    uv = data.nu[:, :2]
    accel = np.gradient(uv, axis=0) / dt[:, np.newaxis]
    jerk = np.gradient(accel, axis=0) / dt[:, np.newaxis]
    
    jerk_mag = np.linalg.norm(jerk, axis=1) # Magnitude of jerk vector at each time step
    total_jerk_energy = np.trapz(jerk_mag**2, data.t) # Total jerk energy: ∫ ||jerk||² dt
    normalized_jerk_energy = total_jerk_energy / T # Normalized jerk energy: ∫ ||jerk||² dt / T
    peak_jerk = np.max(jerk_mag) # Peak jerk: max ||jerk||
    mean_jerk = np.trapz(jerk_mag, data.t) / T # Mean jerk: ∫ ||jerk|| dt / T

    metrics = [
        Metric(metric_type="jerk: total energy", value=total_jerk_energy),
        Metric(metric_type="jerk: normalized energy", value=normalized_jerk_energy),
        Metric(metric_type="jerk: peak", value=peak_jerk),
        Metric(metric_type="jerk: mean", value=mean_jerk)
    ]

    return metrics

def acceleration_metrics(data: ProcessedData) -> list[Metric]:
    dt = np.gradient(data.t)
    T = data.t[-1] - data.t[0]

    u_dot = data.accel_body[:, 0]
    v_dot = data.accel_body[:, 1]
    r_dot = np.rad2deg(data.accel_body[:, 2])

    lin_accel_mag  = np.linalg.norm(data.accel_body[:, :2], axis=1)


    u_dot_peak = np.max(np.abs(u_dot))
    v_dot_peak = np.max(np.abs(v_dot))
    r_dot_peak = np.max(np.abs(r_dot))
    peak_accel_components = np.array([u_dot_peak, v_dot_peak, r_dot_peak])
    
    total_accel_energy_lin = np.trapz(lin_accel_mag**2, data.t) # Total accel energy: ∫ ||accel||² dt
    normalized_accel_energy_lin = total_accel_energy_lin / T # Normalized accel energy: ∫ ||accel||² dt / T
    peak_accel_lin = np.max(lin_accel_mag) # Peak accel: max ||accel||
    mean_accel_lin = np.trapz(lin_accel_mag, data.t) / T # Mean accel: ∫ ||accel|| dt / T

    metrics = [
        Metric(metric_type="acceleration: total energy", value=total_accel_energy_lin),
        Metric(metric_type="acceleration: normalized energy", value=normalized_accel_energy_lin),
        Metric(metric_type="acceleration: peak", value=peak_accel_lin),
        Metric(metric_type="acceleration: mean", value=mean_accel_lin),
        Metric(metric_type="acceleration: peak components", value=peak_accel_components)
    ]

    return metrics
    

def jerk_components_body(data: ProcessedData) -> Metric:
    dt = np.gradient(data.t)
    nu = data.nu
    nu[:, 2] = np.rad2deg(nu[:, 2])

    accel = np.gradient(nu, axis=0) / dt[:, np.newaxis]
    jerk = np.gradient(accel, axis=0) / dt[:, np.newaxis]

    return Metric(metric_type="jerk components (body)", value=jerk)


def jerk_translational_body(data: ProcessedData) -> Metric:
    dt = np.gradient(data.t)
    uv = data.nu[:, :2]

    accel = np.gradient(uv, axis=0) / dt[:, np.newaxis]
    jerk = np.gradient(accel, axis=0) / dt[:, np.newaxis]
    jerk_translational = np.linalg.norm(jerk, axis=1)

    return Metric(metric_type="jerk translational (body)", value=jerk_translational)


def unsigned_pose_error_at_dock(data: ProcessedData) -> Metric:
    pose_end = data.eta[-1, :]
    docking_pose = data.target_wp[-1]
    unsigned_error = np.abs(pose_end - docking_pose)
    unsigned_error[2] = np.rad2deg(unsigned_error[2])  # Convert yaw to degrees
    unsigned_pose_error_metric = Metric(metric_type="unsigned end pose error", value=unsigned_error)
    return unsigned_pose_error_metric

def unsigned_pos_error_at_dock(data: ProcessedData) -> Metric:
    pos_end = data.eta[-1, :2]
    docking_pose = data.target_wp[-1]
    docking_pos = docking_pose[:2]
    unsigned_error = np.linalg.norm(np.abs(pos_end - docking_pos))
    unsigned_pos_error_metric = Metric(metric_type="unsigned end pos error", value=unsigned_error)
    return unsigned_pos_error_metric

def velocity_and_speed_at_dock(data:ProcessedData) -> list[Metric]:
    end_velocity = data.nu[-1]
    end_velocity[2] = np.rad2deg(end_velocity[2])
    end_speed = np.linalg.norm(end_velocity[:2])
    metrics = [
        Metric(metric_type="velocity at dock (body)", value=end_velocity),
        Metric(metric_type="speed at dock", value=end_speed)
    ]
    return metrics

def IAE_pose_control_error(data: ProcessedData, controller_type: str) -> Metric:
    if controller_type == "mpc":
        abs_error = np.abs(data.eta_d - data.eta)
        iae_x = np.trapz(abs_error[:, 0], data.t)
        iae_y = np.trapz(abs_error[:, 1], data.t)
        iae_psi = np.trapz(abs_error[:, 2], data.t)
        iae_psi_deg = np.rad2deg(iae_psi)  # Convert yaw error integral to degrees
        iae_pose = np.array([iae_x, iae_y, iae_psi_deg])
    else:  # controller_type == "pid"
        eta_d = data.eta_d
        eta = data.eta
        t = data.t

        # Create valid masks per component
        mask_x = ~np.isnan(eta_d[:, 0])
        mask_y = ~np.isnan(eta_d[:, 1])
        mask_psi = ~np.isnan(eta_d[:, 2])
        
        # Compute absolute errors
        abs_error_x = np.abs(eta_d[mask_x, 0] - eta[mask_x, 0])
        abs_error_y = np.abs(eta_d[mask_y, 1] - eta[mask_y, 1])
        abs_error_psi = np.abs(eta_d[mask_psi, 2] - eta[mask_psi, 2])

        # Corresponding time vectors
        t_x = t[mask_x]
        t_y = t[mask_y]
        t_psi = t[mask_psi]

        # Integrate using trapezoidal rule
        iae_x = np.trapz(abs_error_x, t_x)
        iae_y = np.trapz(abs_error_y, t_y)
        iae_psi = np.trapz(abs_error_psi, t_psi)
        iae_psi_deg = np.rad2deg(iae_psi)

        # Final IAE vector
        iae_pose = np.array([iae_x, iae_y, iae_psi_deg])

    return Metric(metric_type="iae pose control-error", value=iae_pose)

def IAE_pos_control_error(data: ProcessedData, controller_type: str) -> Metric:
    if controller_type == "mpc":
        pos_error = np.linalg.norm(data.eta_d[:, :2] - data.eta[:, :2], axis=1)
        iae_pos = np.trapz(pos_error, data.t)
    else:  # pid
        mask_x = ~np.isnan(data.eta_d[:, 0])
        mask_y = ~np.isnan(data.eta_d[:, 1])
        mask = mask_x & mask_y

        pos_error = np.linalg.norm(data.eta_d[mask, :2] - data.eta[mask, :2], axis=1)
        iae_pos = np.trapz(pos_error, data.t[mask])

    return Metric(metric_type="iae pos control-error", value=iae_pos)

def IAE_velocity_control_error(data: ProcessedData, controller_type: str) -> Metric:
    if controller_type == "mpc":
        abs_error = np.abs(data.eta_d_dot - data.eta_dot)
        iae_x_dot = np.trapz(abs_error[:, 0], data.t)
        iae_y_dot = np.trapz(abs_error[:, 1], data.t)
        iae_psi_dot = np.trapz(abs_error[:, 2], data.t)
        iae_psi_dot_deg = np.rad2deg(iae_psi_dot)  # Convert yaw rate integral to deg/s
        iae_velocity = np.array([iae_x_dot, iae_y_dot, iae_psi_dot_deg])
    else:  # pid
        mask_x = ~np.isnan(data.eta_d_dot[:, 0])
        mask_y = ~np.isnan(data.eta_d_dot[:, 1])
        mask_psi = ~np.isnan(data.eta_d_dot[:, 2])

        abs_error_x = np.abs(data.eta_d_dot[mask_x, 0] - data.eta_dot[mask_x, 0])
        abs_error_y = np.abs(data.eta_d_dot[mask_y, 1] - data.eta_dot[mask_y, 1])
        abs_error_psi = np.abs(data.eta_d_dot[mask_psi, 2] - data.eta_dot[mask_psi, 2])

        iae_x_dot = np.trapz(abs_error_x, data.t[mask_x])
        iae_y_dot = np.trapz(abs_error_y, data.t[mask_y])
        iae_psi_dot = np.trapz(abs_error_psi, data.t[mask_psi])

    iae_psi_dot_deg = np.rad2deg(iae_psi_dot)
    iae_velocity = np.array([iae_x_dot, iae_y_dot, iae_psi_dot_deg])
    return Metric(metric_type="iae velocity control-error", value=iae_velocity)

def IAE_speed_control_error(data: ProcessedData, controller_type: str) -> Metric:
    if controller_type == "mpc":
        speed_error = np.linalg.norm(data.eta_d_dot[:, :2] - data.eta_dot[:, :2], axis=1)
        iae_speed = np.trapz(speed_error, data.t)
    else:  # pid
        mask_x = ~np.isnan(data.eta_d_dot[:, 0])
        mask_y = ~np.isnan(data.eta_d_dot[:, 1])
        mask = mask_x & mask_y

        speed_error = np.linalg.norm(data.eta_d_dot[mask, :2] - data.eta_dot[mask, :2], axis=1)
        iae_speed = np.trapz(speed_error, data.t[mask])
    return Metric(metric_type="iae speed control-error", value=iae_speed)

def spacer() -> Metric:
    return Metric(metric_type="", value="")

