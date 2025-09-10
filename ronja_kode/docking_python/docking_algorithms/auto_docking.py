import numpy as np
import time

from docking_algorithms.controllers.pid_controller import PIDController
from docking_algorithms.controllers.mpc_guidance import MPCGuidance
from docking_algorithms.utils.transformations import body_to_ned_transform
from docking_algorithms.utils.data_logger import DataLogger, load_config
from docking_algorithms.utils.computations import compute_velocity_components, compute_direction_toward_point
from docking_algorithms.guidance.guidance import Guidance
from docking_algorithms.utils.types import Waypoint, PIDGains, OptimalTrajectories
from docking_algorithms.utils.reference_filters import ThirdOrderReferenceFilter
from docking_algorithms.utils.inf2pipi import inf2pipi
from docking_algorithms.controllers.interpolation import InterpolateMPCOutput


class AutoDocking:
    """
    Purpose: 
        - Manage phase transitions: switch set of gains based on phase and handle wp progression
        - Higher-level logic controller for docking using extended DP
    """
    def __init__(self, t0: float, dt: float, init_pose: np.ndarray, init_velocity: np.ndarray,
                 batch: str, run_id: int, controller_type: str, quay_type: str,
                 run_description: str) -> None:
        run_id = int(run_id)
        self.dt = dt    # eg 0.1
        self.t = t0

        self.nan_vec = np.array([np.nan, np.nan, np.nan])
        self.quay_type = quay_type

        self.init_pose = np.array(init_pose)
        self.init_velocity = np.array(init_velocity)
        print(f'controller type: {controller_type}, batch: {batch}, run id: {run_id}, quay: {quay_type}')

        """ Quay and vessel config"""
        self.quays_vessel_cfg = load_config(cfg_name="quays_vessel", cfg_path="../config")
        
        # print(f"{controller_type} config for {scenario}: ")
        # print(OmegaConf.to_yaml(self.controller_cfg))
        if controller_type == "pid":
            self.pid_cfg = load_config(cfg_name="pid", cfg_path="../config")[batch]
            # Create PID gains sets
            pid_gains = self.pid_cfg["gains"]
            self.pid_gains_default = PIDGains(kp=np.array(pid_gains["default"]["kp"]),
                                              ki=np.array(pid_gains["default"]["ki"]),
                                              kd=np.array(pid_gains["default"]["kd"]))
            self.pid_gains_phase2 = PIDGains(kp=np.array(pid_gains["phase2"]["kp"]),
                                              ki=np.array(pid_gains["phase2"]["ki"]),
                                              kd=np.array(pid_gains["phase2"]["kd"]))
            # Initialize PID controller
            self.pid = PIDController(dt=dt, pid_gains=self.pid_gains_default)
            self.ref_filter_course = ThirdOrderReferenceFilter(x_is_angle=True)
            course_0 = np.arctan2(init_velocity[1], init_velocity[0])
            self.ref_filter_course.set_states(t=t0, x_d=course_0)

            """ Load end stage forces and waypoints setup """
            wp_cfg = load_config(cfg_name="runs", cfg_path="../config")[f"run{run_id}"]["waypoints"]
            self.end_stage_forces = load_config(cfg_name="runs", cfg_path="../config")["end_stage_forces"][quay_type]

            """ Guidance """
            guidance_cfg = load_config(cfg_name="guidance", cfg_path="../config")
            # Create waypoint list for guidance module
            self.waypoint_list = []
            for key, value in wp_cfg.items():
                pose = np.array(value['pose'])
                speed = value['speed']
                x_ep_threshold = value['x_ep_threshold']
                is_reached = value['is_reached']
                self.waypoint_list.append(Waypoint(pose=pose, speed=speed,x_ep_threshold=x_ep_threshold,
                                                is_reached=is_reached))

            # Insert the initial pose as the first waypoint
            eta_0 = Waypoint(pose=init_pose, speed=np.linalg.norm(init_velocity[:2]),
                            is_reached=True)
            self.waypoint_list.insert(0, eta_0)

            # Initialize guidance module
            self.guidance = Guidance(dt=dt, waypoint_list=self.waypoint_list, cfg=guidance_cfg)

            cfg_list = [self.pid_cfg, wp_cfg, self.end_stage_forces, self.quays_vessel_cfg]

        elif controller_type == "mpc_pid":
            self.mpc_cfg = load_config(cfg_name="mpc", cfg_path="../config")[batch]
            self.pid_cfg = load_config(cfg_name="pid", cfg_path="../config")[batch]
            cfg_list = [self.mpc_cfg, self.pid_cfg, self.quays_vessel_cfg]
            self.wp_ref = np.array(self.mpc_cfg.docking_pose)

            pid_gains = self.pid_cfg["gains"]
            self.pid_gains = PIDGains(kp=np.array(pid_gains["default"]["kp"]),
                                              ki=np.array(pid_gains["default"]["ki"]),
                                              kd=np.array(pid_gains["default"]["kd"]))
            # Initialize PID controller
            self.pid = PIDController(dt=dt, pid_gains=self.pid_gains)
            rel_wp_ref = np.array([self.wp_ref[0], self.wp_ref[1], 0.0])  # heading error is "state" in mpc
            self.mpc = MPCGuidance(dt=dt, mpc_cfg=self.mpc_cfg,
                                     quays_vessel_cfg=self.quays_vessel_cfg,
                                     waypoint_ref=rel_wp_ref,
                                     max_speed=np.linalg.norm(init_velocity[:2]))
            self.mpc_counter = 0

        
        """ Initialize logger module """
        log_cfg = load_config(cfg_name="logging", cfg_path="../config")
        self.logger = DataLogger(
            filename=log_cfg["filename"],
            base_directory=log_cfg["base_directory"],
            controller_type = controller_type,
            batch = batch,
            run_id = run_id,
            quay_type=quay_type,
            run_description=run_description
        )

        # Save configs
        self.logger.save_configs_to_file(configs=cfg_list)
        print(f"Initialized AutoDocking")


    def extended_DP_with_LOS(self, t: float, dt: float, eta: np.ndarray, eta_dot: np.ndarray,
                                     accel: np.ndarray) -> np.ndarray:
        
        """
        Used for testing LOS method with another heading law than the original one for LOS.
        Can decide whether to include reference filters on speed, course, heading
        """
        t_switch = np.nan
        accel_3DOF = np.array([accel[0], accel[1], accel[5]])
        eta = np.array(eta)
        eta_dot = np.array(eta_dot)

        vessel_speed = np.linalg.norm(eta_dot[:2])
        # print(f'vessel velocity: {eta_dot}')
        # print(f'vessel speed: {vessel_speed}')

        # Check if reached a wp
        if self.guidance.has_reached_wp(eta=eta) and not self.guidance.get_current_wp().is_reached:
            print(f'flag wp as reached: {self.guidance.get_current_wp().pose}')
            if np.array_equal(self.guidance.get_current_wp().pose, self.guidance.get_wp2().pose):
                self.pid.set_pid_gains(pid_gains=self.pid_gains_phase2)
                self.pid.reset_errors()
            self.guidance.update_waypoints(eta=eta)
            t_switch = t

        if self.guidance.get_wp1().is_reached:
            # Is in end stage - wp following is ended
            print(f'wp1 is reached: {self.guidance.get_wp1().is_reached}')
            control_ned = self.pid.update_control(state_d=self.guidance.get_current_wp().pose, state=eta,
                                                  state_d_dot=np.array([0,0,0]), state_dot=eta_dot)
            control_body = body_to_ned_transform(eta[2]).T @ control_ned

            # control_body[0] = self.end_stage_forces.surge
            # if self.quay_type == "l_quay":  # apply const force in sway only if L-shaped quay
            #     control_body[1] = self.end_stage_forces.sway

            # output = control_body

            # zero_vector = np.array([0, 0, 0])
            # output = zero_vector    # will cause simulation to terminate
            nu = body_to_ned_transform(eta[2]).T @ eta_dot  # only for logging purposes
            self.logger.log_data(t=t, eta=eta, eta_dot=eta_dot, eta_d=self.nan_vec,
                                 eta_d_dot=self.nan_vec, chi_d=np.nan,
                                 chi=np.arctan2(eta_dot[1], eta_dot[0]), nu=nu, nu_d=self.nan_vec,
                                 accel_body=accel_3DOF, control_forces_body=control_body,
                                 target_wp=self.guidance.get_current_wp().pose, t_switch=t_switch)
            print(f'Terminating - reaching docking point')
            return 0    # gives an error -> simulation terminates (docking goal is reached)
        
        if self.guidance.get_wp2().is_reached:
            # zero_vector = np.array([0, 0, 0])
            # output = zero_vector    # will cause simulation to terminate
            # print(f'wp2 is reached: {self.guidance.get_wp2().is_reached}')
            wp1_pose = self.guidance.get_wp1().pose

            speed_d = self.guidance.sigmoid_speed_law()
            # print(f'desired speed phase 2: {speed_d}')

            course_d = self.guidance.compute_LOS_course()
            self.ref_filter_course.set_reference(x_r=course_d)
            self.ref_filter_course.step(t1=t+dt)
            _, course_d, _, _ = self.ref_filter_course.get_desired_values()
            
            #course_d = compute_direction_toward_point(from_point=self.guidance.get_wp2().pose, to_point=wp1_pose)
            heading_d = self.guidance.heading_law()

            eta_d = np.array([wp1_pose[0], wp1_pose[1], heading_d])
            eta_d_dot = compute_velocity_components(speed=speed_d, direction_rad=course_d)

            control_ned = self.pid.update_control(state_d=eta_d, state_d_dot=eta_d_dot, state=eta, state_dot=eta_dot)
            control_body = body_to_ned_transform(psi=eta[2]).T @ control_ned

            output = control_body

            nu = body_to_ned_transform(psi=eta[2]).T @ eta_dot
            nu_d = body_to_ned_transform(psi=eta[2]).T @ eta_d_dot

            # print(f't: {t}, eta: {eta}, eta_dot: {eta_dot}, eta_d_dot: {eta_d_dot}, chi: {np.arctan2(eta_dot[1], eta_dot[0])}')
            # print(f'nu: {nu}, nu_d: {nu_d}, accel: {accel}, control body: {control_body}, control ned: {control_ned}')
            # print(f'target wp: {self.guidance.get_current_wp().pose}, t switch: {t_switch}')
            eta_d_log = np.array([np.nan, np.nan, eta_d[2]])
            self.logger.log_data(t=t, eta=eta, eta_dot=eta_dot, eta_d=eta_d_log,
                                 eta_d_dot=eta_d_dot, chi_d=course_d,
                                 chi=np.arctan2(eta_dot[1], eta_dot[0]), nu=nu, nu_d=nu_d,
                                 accel_body=accel_3DOF, control_forces_body=control_body,
                                 target_wp=self.guidance.get_current_wp().pose, t_switch=t_switch)
            return output

               
        # Has not yet reached wp2 or wp1 -> (modified) LOS guidance
        speed_ref = self.guidance.sigmoid_speed_law()
        # print(f'desired speed phase 1: {speed_ref}')
        course_ref = self.guidance.compute_LOS_course()
        psi_ref = self.guidance.sigmoid_heading_law()

        psi_d = psi_ref
        speed_d = speed_ref
        course_d = course_ref

        self.ref_filter_course.set_reference(x_r=course_d)
        self.ref_filter_course.step(t1=t+dt)
        _, course_d, _, _ = self.ref_filter_course.get_desired_values()

        eta_d_dot = compute_velocity_components(speed=speed_d, direction_rad=course_d)
        # print(f'desired velocity phase 1: {eta_d_dot}')
        speed_from_velocity = np.linalg.norm(eta_d_dot[:2])
        # print(f'speed from desired velocity: {speed_from_velocity}')
        pos_d = self.guidance.compute_position_reference(velocity_ref=None, speed=speed_d, dt=dt)
        eta_d = np.array([pos_d[0], pos_d[1], psi_d])  
        # print(f'desired velocity components: {eta_d}')
        # print(f'pose: {eta}, velocity: {eta_dot}')
        # print(f'desires pose: {eta_d}, desired velocity: {eta_d_dot}')  

        control_forces_ned = self.pid.update_control(state_d=eta_d, state_d_dot=eta_d_dot, state=eta, state_dot=eta_dot) 
        control_forces_body = body_to_ned_transform(eta[2]).T @ control_forces_ned
        # print(f'control forces body: {control_forces_body}')
        output = control_forces_body
        
        nu = body_to_ned_transform(eta[2]).T @ eta_dot
        nu_d = body_to_ned_transform(eta[2]).T @ eta_d_dot
        self.logger.log_data(t=t, eta=eta, eta_dot=eta_dot, eta_d=eta_d, eta_d_dot=eta_d_dot,
                             chi_d=course_d, chi=np.arctan2(eta_dot[1], eta_dot[0]), nu=nu, nu_d=nu_d,
                             accel_body=accel_3DOF, control_forces_body=control_forces_body,
                             target_wp=self.guidance.get_current_wp().pose, t_switch=t_switch)
        return output
    
    
    def MPC_PID_docking(self, t: float, dt: float, eta: np.ndarray, eta_dot: np.ndarray,
                            accel: np.ndarray) -> np.ndarray:
            """
            Output from MPC / input to PID: pose and velocity
            Output from PID controller: tau

            Alternative 1: run MPC for every timestep and give first optimal as input to PID.
                - Both MPC and PID run at 10 Hz (which likely is neither realistic nor necessary)
            Alternative 2: run MPC once every few second, then use output as look up for input to PID.
                - MPC runs at eg. 1 Hz, while PID runs at 10 Hz
            """
            eta = np.array(eta)
            eta[2] = inf2pipi(ang=eta[2])
            eta_dot = np.array(eta_dot)
            accel_3dof = np.array([accel[0], accel[1], accel[5]])


            # print(f'current pose: {eta}')
            if self.mpc_counter == 0:   # run MPC guidance (path-planner) in the very beginning
                relative_heading = inf2pipi(ang=(eta[2]-self.wp_ref[2]))    # error, "state" in mpc
                eta_relative_heading = np.array([eta[0], eta[1], relative_heading])
                
                start_solve_time = time.perf_counter()
                self.opt_sol = self.mpc.solve_mpc(t=t, state=eta_relative_heading,
                                                               state_dot=eta_dot)
                self.mpc_solver_time = time.perf_counter() - start_solve_time
                print(f'mpc solver time: {self.mpc_solver_time:.6f} seconds')
                self.opt_sol.state_dot = np.vstack([eta_dot, self.opt_sol.state_dot])  # include vessel velocity
                
                # Heading relative ref -> relative north (-pi, pi] 
                self.opt_sol.state[:, 2] = np.array([inf2pipi(psi + self.wp_ref[2]) for psi in self.opt_sol.state[:, 2]])        
                
                # Store mpc output
                self.logger.save_mpc_trajectories(trajectory=self.opt_sol)
                self.interpolator = InterpolateMPCOutput(dt=self.dt, sequence=self.opt_sol)

            # Retrieve references (interpolate)
            eta_d, eta_d_dot = self.interpolator.get_references(at_t=t)

            tau_ned = self.pid.update_control(state=eta, state_dot=eta_dot,
                                              state_d=eta_d,
                                              state_d_dot=eta_d_dot)
            tau_body = body_to_ned_transform(psi=eta[2]).T @ tau_ned
            # print(f'next pose ref: {eta_d}')

            # Only for logging purposes
            nu = body_to_ned_transform(psi=eta[2]).T@eta_dot
            nu_d = body_to_ned_transform(psi=eta[2]).T@eta_d_dot

            if np.linalg.norm(eta[:2] - self.wp_ref[:2]) <= 0.05:   # docking point threshold
                self.logger.log_data(t=t, eta=eta, eta_dot=eta_dot, eta_d=eta_d, eta_d_dot=eta_d_dot,
                                    nu=nu, nu_d=nu_d, control_forces_body=tau_body,
                                    target_wp=self.wp_ref, accel_body=accel_3dof,
                                    side_normal=self.opt_sol.sidequay_line_normals[self.mpc_counter],
                                    side_line_point=self.opt_sol.sidequay_line_points[self.mpc_counter],
                                    front_normal=self.opt_sol.frontquay_line_normals[0],
                                    front_line_point=self.opt_sol.frontquay_line_points[0],
                                    t_switch=t)
                
                print(f'Terminating - reaching WP')
                return 0 # gives error -> terminates simulation

            self.logger.log_data(t=t, eta=eta, eta_dot=eta_dot, eta_d=eta_d, eta_d_dot=eta_d_dot,
                                    nu=nu, nu_d=nu_d, control_forces_body=tau_body,
                                    target_wp=self.wp_ref, accel_body=accel_3dof,
                                    side_normal=self.opt_sol.sidequay_line_normals[self.mpc_counter],
                                    side_line_point=self.opt_sol.sidequay_line_points[self.mpc_counter],
                                    front_normal=self.opt_sol.frontquay_line_normals[0],
                                    front_line_point=self.opt_sol.frontquay_line_points[0])
            
            # if self.mpc_counter < 9:
            #     self.mpc_counter += 1
            # else:
            #     self.mpc_counter = 0
            self.mpc_counter += 1
            return tau_body


    def run(self, t: float, dt: float, eta: np.ndarray, eta_dot: np.ndarray,
            accel: np.ndarray, controller_type: str) -> np.ndarray:
        if controller_type == "pid":
            output = self.extended_DP_with_LOS(t=t, dt=dt, eta=eta, eta_dot=eta_dot, accel=accel)
            # print(f'run pid')
        elif controller_type == "mpc_pid":
            # print(f'run mpc pid')
            output = self.MPC_PID_docking(t=t, dt=dt, eta=eta, eta_dot=eta_dot, accel=accel)
            # output = asyncio.run(self.MPC_PID(t=t, dt=dt, eta=eta, eta_dot=eta_dot, accel=accel))
        elif controller_type == "mpc":
            # print(f'run mpc')
            output = self.full_MPC_docking(t=t, dt=dt, eta=eta, eta_dot=eta_dot, accel=accel)
        return output 