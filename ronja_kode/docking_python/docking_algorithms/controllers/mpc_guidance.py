import casadi as ca
import numpy as np
from docking_algorithms.utils.computations import point_extension_of_line
from docking_algorithms.utils.types import OptimalTrajectories
from omegaconf import DictConfig, OmegaConf

class MPCGuidance():
    def __init__(self, dt: float, mpc_cfg: DictConfig,
                 quays_vessel_cfg: DictConfig, waypoint_ref: np.ndarray,
                 max_speed: float) -> None:
        self.mpc_cfg = mpc_cfg
        self.quays_vessel_cfg = quays_vessel_cfg
        self.dt = dt
        self.N = mpc_cfg.horizon    # number of FUTURE control intervals to evaluate

        self.vessel_length = quays_vessel_cfg.vessel.length
        self.vessel_width = quays_vessel_cfg.vessel.width

        # reshape to a row vector (this is the standard in this MPC formulation)
        self.sidequay_start = ca.reshape(ca.MX(quays_vessel_cfg.quays.side_segment.corner), 1, 2)
        self.sidequay_end = ca.reshape(ca.MX(quays_vessel_cfg.quays.side_segment.end), 1, 2)
        self.frontquay_start = ca.reshape(ca.MX(quays_vessel_cfg.quays.front_segment.corner), 1, 2)
        self.frontquay_end = ca.reshape(ca.MX(quays_vessel_cfg.quays.front_segment.end), 1, 2)

        self.sidequay_unit_normal = ca.reshape(ca.MX(mpc_cfg.sidequay_unit_normal), 1, 2)
        self.ref_line_unit_normal = ca.reshape(ca.MX(mpc_cfg.ref_line_unit_normal), 1, 2)
        self.sidequay_point_on_line = self._compute_quayside_extension_point(extension_length=mpc_cfg.sidequay_extension_length)
        self.frontquay_unit_normal = ca.reshape(ca.MX(mpc_cfg.frontquay_unit_normal), 1, 2)
        self.frontquay_point_on_line = ca.reshape(ca.MX(waypoint_ref[0:2]), 1, 2)  # only used in vessel center constraint function

        self.opti = ca.Opti()   # Create Opti instance
        opts = {'ipopt.print_level':0, 'print_time':0}
        s_opts = {
            "tol": 1e-8,  
            "constr_viol_tol": 1e-4,  
            "dual_inf_tol": 1e-8,  
            "compl_inf_tol": 1e-8,  
            "acceptable_tol": 1e-6,  
            "acceptable_constr_viol_tol": 1e-4,  
            "nlp_scaling_method": "gradient-based"  
        }

        self.opti.solver("ipopt", opts)  # set type of solver 
        # self.opti.solver("ipopt")

        # Parameters/variables for side-quay constraint
        self.sidequay_all_unit_normals = self.opti.variable(self.N, 2)
        self.sidequay_line_offsets = self.opti.variable(self.N, 1)

        # TODO: find reasonable values and possibly add to config
        self.psi_dot_max = 0.0872664626  # [rad] (5 deg) - this is just for safety, but minimizing velocity keeps it way lower
        self.max_speed = min(max_speed, 2)  # 2 m/s is ABSOLUTE max !!
        self.speed_max_squared = self.max_speed*self.max_speed
    
        # Decision variables (symbolic variables)
        self.eta = self.opti.variable(self.N+1, 3)    # [x, y, psi] for all timesteps - includes given x0
        self.eta_dot = self.opti.variable(self.N, 3) # [x_dot, y_dot, psi_dot] for all timesteps

        # Parameters for initial states (constants during optimization)
        self.eta_0 = self.opti.parameter(1, 3)
        self.eta_dot_0 = self.opti.parameter(1, 3)
        self.wp_ref = self.opti.parameter(1,3)  # [x_ref, y_ref, psi_ref]
        self.opti.set_value(self.wp_ref, waypoint_ref)

        self._define_constraints()
        self._define_cost()
        return
    
    def solve_mpc(self, t: float,
                  state: np.ndarray,
                  state_dot: np.ndarray)-> OptimalTrajectories:
        
        self.opti.set_value(self.eta_0, state)
        self.opti.set_value(self.eta_dot_0, state_dot)
        
        try:
            b = 1
            solution = self.opti.solve()
            eta_opt = solution.value(self.eta)
            eta_dot_opt = solution.value(self.eta_dot)
            # TL = solution.value(self.TL)
            # TR = solution.value(self.TR)
            # BL = solution.value(self.BL)
            # BR = solution.value(self.BR)
            b = 1
            # frontquay_line_normal_opt = np.tile(solution.value(self.front_normal_vec).flatten(), (self.N, 1)) # front quay
            # frontquay_point_on_line_opt = np.tile(solution.value(self.front_point_on_line).flatten(), (self.N, 1))    # front quay
            frontquay_line_normal_opt = np.array([solution.value(self.frontquay_unit_normal)]) #solution.value(self.sidequay_all_unit_normals)
            frontquay_point_on_line_opt = np.array([solution.value(self.frontquay_point_on_line)]) # np.tile(solution.value(self.sidequay_point_on_line).flatten(), (self.N, 1))
            # sidequay_line_normals_opt = solution.value(self.sidequay_all_unit_normals)
            sidequay_line_normals_opt = np.tile(solution.value(self.sidequay_unit_normal).flatten(), (self.N, 1))
            sidequay_line_points_opt = np.tile(solution.value(self.sidequay_point_on_line).flatten(), (self.N, 1))
            # sidequay_line_points_opt = solution.value(self.sidequay_all_points)
            b = 1
            optimal_trajectories = OptimalTrajectories(
                state=eta_opt,
                state_dot=eta_dot_opt,
                frontquay_line_normals=frontquay_line_normal_opt,
                frontquay_line_points=frontquay_point_on_line_opt,
                sidequay_line_normals=sidequay_line_normals_opt,
                sidequay_line_points=sidequay_line_points_opt,
                t=t
            )
            # print(f'return solution: {optimal_trajectories}')
            return optimal_trajectories
        
        except RuntimeError as e:
            print("MPC solver failed:", e, "\nSolver stats:", self.opti.debug)
            b = 1
            return OptimalTrajectories(
                state=np.array([[0,0,0]]),
                state_dot=np.array([[0,0,0]]),
                frontquay_line_points=np.array([[0,0]]),
                frontquay_line_normals=np.array([[0,0]]),
                sidequay_line_points=np.array([[0,0]]),
                sidequay_line_normals=np.array([[0,0]]),
                t=0
            )
        
    def _define_cost(self) -> None:
        # Large value -> higher importance -> penalize deviation more (matrices are defined for one timestep)
        Q = np.diag([1, 1, 0.3]) #ca.DM([[10, 0, 0], [0, 10, 0], [0, 0, 1]]) #  weighting matrix for pose errors
        # Q_terminal = np.diag([2000, 2000, 2000])
        R = np.diag([10, 10, 10])
        # R_terminal = np.diag([2000, 2000, 2000])
        S = np.diag([10000, 10000, 700])     # np.diag([10000, 10000, 700])
        w_cross_track = 5
        
        cost = 0
        for k in range(0, self.N): # iterating through [0, N-1] -> N iterations

            # penalize pose deviations
            state_error = self.eta[k+1, :] - self.wp_ref
            cost += self.dt*ca.mtimes([state_error, Q, state_error.T])

            # penalize cross track from "ref line"
            cross_track = ca.dot(self.ref_line_unit_normal, self.eta[k+1, 0:2] - self.wp_ref[0:2])
            cost += self.dt*w_cross_track*cross_track*cross_track

            # penalize high velocities
            control_effort = self.eta_dot[k, :]
            cost += self.dt*ca.mtimes([control_effort, R, control_effort.T])

        # Penalize acceleration
        accel_0 = (self.eta_dot[0, :] - self.eta_dot_0)/self.dt
        cost += self.dt*ca.mtimes([accel_0, S, accel_0.T])
        for k in range(1, self.N):  # Iterating from 1 to N-1 to compute acceleration
            accel = (self.eta_dot[k, :] - self.eta_dot[k-1, :])/self.dt
            cost += self.dt * ca.mtimes([accel, S, accel.T])  # Penalize acceleration
        
        # Terminal cost
        # state_error_terminal = self.eta[self.N, :] - self.wp_ref
        # cost += self.dt*ca.mtimes([state_error_terminal, Q_terminal, state_error_terminal.T])

        # vel_error_terminal = self.eta_dot[self.N-1, :]  # desired terminal velocity is zero
        # cost += self.dt * ca.mtimes([vel_error_terminal, R_terminal, vel_error_terminal.T])
        
        #  Optional: normalize cost by total time/duration
        # cost /= (self.N*self.dt)
        
        self.opti.minimize(cost)
        return
    
    def _define_constraints(self) -> None:
        # Initial value constraints
        self.opti.subject_to(self.eta[0, :] == self.eta_0)

        # Upper/lower bounds constraints
        self.opti.subject_to(self.opti.bounded(
            -self.psi_dot_max, self.eta_dot[:, 2], self.psi_dot_max))
        self.opti.subject_to(
            self.eta_dot[:, 0]*self.eta_dot[:, 0] + self.eta_dot[:, 1]*self.eta_dot[:, 1] <= self.speed_max_squared) # speed mangitude constraint
        
        # Multiple shooting for state continuity
        for k in range(0, self.N): # iterates from (and including) 0, to (not including) self.N
            self.opti.subject_to(self.eta[k+1, :] == (self.eta[k, :] + self.eta_dot[k, :]*self.dt))  # shooting constraint
            # Ensure normal vector has approximately unit length for ALL timesteps (N + 1) !!
            self.opti.subject_to(self.opti.bounded(-1, self.sidequay_all_unit_normals[k,0], 1))
            self.opti.subject_to(self.opti.bounded(-1, self.sidequay_all_unit_normals[k,1], 1))


        # The following 4 constraints are to be tested
        # self._frontquay_corner_constraints()
        self._frontquay_center_constraint()

        # self._sidequay_static_point_constraint()
        self._sidequay_center_constraint()


        # self._sidequay_const_constraint()
        # self._sidequay_dynamic_point_constraint()
        # self._sidequay_implicit_constraint()
        return
    
    def _frontquay_corner_constraints(self) -> None:
        c1, c2, c3, c4 = self._compute_vessel_corners_NED()  # vessel corners (x,y) in NED

        for k in range(0, self.N):    # k = 0, ..., k = N-1
            # Ensure all vessel points are on the "positive/correct" side of the line/front-quay
            self.opti.subject_to(ca.dot(self.frontquay_unit_normal, c1[k+1, :] - self.frontquay_end) > 0)
            self.opti.subject_to(ca.dot(self.frontquay_unit_normal, c2[k+1, :] - self.frontquay_end) > 0)
            self.opti.subject_to(ca.dot(self.frontquay_unit_normal, c3[k+1, :] - self.frontquay_end) > 0)
            self.opti.subject_to(ca.dot(self.frontquay_unit_normal, c4[k+1, :] - self.frontquay_end) > 0)
        return

    def _frontquay_center_constraint(self) -> None: 
        displacement = (ca.dot(self.wp_ref[0:2] - self.frontquay_end,
                               self.frontquay_unit_normal) / ca.dot(self.frontquay_unit_normal, self.frontquay_unit_normal)) * self.frontquay_unit_normal
        
        point_on_displaced_line = self.frontquay_end + displacement

        for k in range(0, self.N):
            self.opti.subject_to(ca.dot(self.frontquay_unit_normal,
                                        self.eta[k+1, :2] - point_on_displaced_line) >= 0)
        return
    
    def _sidequay_static_point_constraint(self) -> None:
        c1, c2, c3, c4 = self._compute_vessel_corners_NED()
        for k in range(0, self.N):
            # Ensure all vessel points are on the "positive/correct" side of the line/front-quay
            self.opti.subject_to(ca.dot(self.sidequay_all_unit_normals[k, :], c1[k+1, :] - self.sidequay_point_on_line) > 0)
            self.opti.subject_to(ca.dot(self.sidequay_all_unit_normals[k, :], c2[k+1, :] - self.sidequay_point_on_line) > 0)
            self.opti.subject_to(ca.dot(self.sidequay_all_unit_normals[k, :], c3[k+1, :] - self.sidequay_point_on_line) > 0)
            self.opti.subject_to(ca.dot(self.sidequay_all_unit_normals[k, :], c4[k+1, :] - self.sidequay_point_on_line) > 0)

            self.opti.subject_to(ca.dot(self.sidequay_all_unit_normals[k, :], self.sidequay_start - self.sidequay_point_on_line) <= 0)
            self.opti.subject_to(ca.dot(self.sidequay_all_unit_normals[k, :], self.sidequay_end - self.sidequay_point_on_line) <= 0)
        return
    
    def _sidequay_const_constraint(self) -> None:
        for k in range(0, self.N):
            x_k, y_k, theta_k = self.eta[k+1, 0], self.eta[k+1, 1], self.eta[k+1, 2]

            cos_theta = ca.cos(theta_k)
            sin_theta = ca.sin(theta_k)

            # Compute the vessel's rectangle corners
            TL = ca.horzcat(x_k - (self.vessel_length/2)*cos_theta + (self.vessel_width/2)*sin_theta,
                            y_k - (self.vessel_length/2)*sin_theta - (self.vessel_width/2)*cos_theta)  # Top Left
            TR = ca.horzcat(x_k + (self.vessel_length/2)*cos_theta + (self.vessel_width/2)*sin_theta,
                            y_k + (self.vessel_length/2)*sin_theta - (self.vessel_width/2)*cos_theta)  # Top Right
            BL = ca.horzcat(x_k - (self.vessel_length/2)*cos_theta - (self.vessel_width/2)*sin_theta,
                            y_k - (self.vessel_length/2)*sin_theta + (self.vessel_width/2)*cos_theta)  # Bottom Left
            BR = ca.horzcat(x_k + (self.vessel_length/2)*cos_theta - (self.vessel_width/2)*sin_theta,
                            y_k + (self.vessel_length/2)*sin_theta + (self.vessel_width/2)*cos_theta)   # Bottom Right

            self.opti.subject_to(ca.dot(self.sidequay_unit_normal, TL - self.sidequay_point_on_line) >= 0)
            self.opti.subject_to(ca.dot(self.sidequay_unit_normal, TR - self.sidequay_point_on_line) >= 0)
            self.opti.subject_to(ca.dot(self.sidequay_unit_normal, BL - self.sidequay_point_on_line) >= 0)
            self.opti.subject_to(ca.dot(self.sidequay_unit_normal, BR - self.sidequay_point_on_line) >= 0)
        return

    
    def _sidequay_center_constraint(self) -> None:
        """
        Line defined by static point on the quay (or along an extension)
        and a 2unit" normal vector (optimization variable)
        """
        for k in range(0, self.N):
            # keep a distance equal to 1/2 boat width from the line
            self.opti.subject_to(ca.dot(self.sidequay_all_unit_normals[k, :], self.eta[k+1, :2] - self.sidequay_point_on_line) >= 5.25/2)
            self.opti.subject_to(ca.dot(self.sidequay_all_unit_normals[k, :], self.sidequay_start - self.sidequay_point_on_line) <= 0)
            self.opti.subject_to(ca.dot(self.sidequay_all_unit_normals[k, :], self.sidequay_end - self.sidequay_point_on_line) <= 0)
        return

    def _sidequay_implicit_constraint(self) -> None:
        for k in range(0, self.N):
            x_k, y_k, theta_k = self.eta[k+1, 0], self.eta[k+1, 1], self.eta[k+1, 2]
            n_k = self.sidequay_all_unit_normals[k, :]  # Normal vector at time k
            c_k = self.sidequay_line_offsets[k]    # Offset at time k

            cos_theta = ca.cos(theta_k)
            sin_theta = ca.sin(theta_k)

            # Compute the vessel's rectangle corners
            TL = ca.horzcat(x_k - (self.vessel_length/2)*cos_theta + (self.vessel_width/2)*sin_theta,
                            y_k - (self.vessel_length/2)*sin_theta - (self.vessel_width/2)*cos_theta)  # Top Left
            TR = ca.horzcat(x_k + (self.vessel_length/2)*cos_theta + (self.vessel_width/2)*sin_theta,
                            y_k + (self.vessel_length/2)*sin_theta - (self.vessel_width/2)*cos_theta)  # Top Right
            BL = ca.horzcat(x_k - (self.vessel_length/2)*cos_theta - (self.vessel_width/2)*sin_theta,
                            y_k - (self.vessel_length/2)*sin_theta + (self.vessel_width/2)*cos_theta)  # Bottom Left
            BR = ca.horzcat(x_k + (self.vessel_length/2)*cos_theta - (self.vessel_width/2)*sin_theta,
                            y_k + (self.vessel_length/2)*sin_theta + (self.vessel_width/2)*cos_theta)   # Bottom Right

            # Enforce implicit line constraint for all rectangle corners
            self.opti.subject_to(ca.dot(n_k, TL) + c_k >= 0)  # Element-wise constraint
            self.opti.subject_to(ca.dot(n_k, TR) + c_k >= 0)  # Element-wise constraint
            self.opti.subject_to(ca.dot(n_k, BL) + c_k >= 0)  # Element-wise constraint
            self.opti.subject_to(ca.dot(n_k, BR) + c_k >= 0)  # Element-wise constraint

            self.TL = ca.dot(n_k, TL) + c_k
            self.TR = ca.dot(n_k, TR) + c_k
            self.BL = ca.dot(n_k, BL) + c_k
            self.BR = ca.dot(n_k, BR) + c_k
            
            # Ensure obstacle endpoints remain on the opposite side
            self.opti.subject_to(ca.dot(n_k, self.sidequay_end) + c_k <= 0)
            self.opti.subject_to(ca.dot(n_k, self.sidequay_start) + c_k <= 0)

        # c1, c2, c3, c4 = self._compute_vessel_corners_NED()

        # for k in range(0, self.N):
        #     # Ensure all vessel corners are on the "correct" side of the line
        #     for corner in [c1[k+1, :], c2[k+1, :], c3[k+1, :], c4[k+1, :]]:
        #         self.opti.subject_to(ca.dot(self.sidequay_all_unit_normals[k, :], corner) - self.sidequay_all_d[k] >= 0)

        #     # Ensure quay start and end points are on the "opposite" side
        #     self.opti.subject_to(ca.dot(self.sidequay_all_unit_normals[k, :], self.sidequay_start) - self.sidequay_all_d[k] <= 0)
        #     self.opti.subject_to(ca.dot(self.sidequay_all_unit_normals[k, :], self.sidequay_end) - self.sidequay_all_d[k] <= 0)
        return


    
    def _compute_vessel_corners_NED(self) -> tuple[ca.MX, ca.MX, ca.MX, ca.MX]:
        """
        Compute vessel corner points for all time steps.
        
        Parameters:
            eta (ca.MX): (N, 3) CasADi matrix, where each row is [north, east, yaw].
            length (float): Length of the vessel.
            width (float): Width of the vessel.

        Returns:
            ca.MX: (4, n, 2) CasADi matrix where each slice `[:, t, :]` contains 
                the four corners [north, east] for time step `t`.
        """
        n_steps = self.eta.shape[0]  # Number of time steps (n)

        # Extract components
        north = self.eta[:, 0]  # Shape: (n,)
        east = self.eta[:, 1]   # Shape: (n,)
        yaw = self.eta[:, 2]    # Shape: (n,)

        # Define box vertices in the local frame (centered at origin)
        half_length = self.vessel_length / 2
        half_width = self.vessel_width / 2

        vertices_body = ca.DM([
            [-half_length, -half_width],  # Bottom-left
            [half_length, -half_width],   # Bottom-right
            [half_length, half_width],    # Top-right
            [-half_length, half_width]    # Top-left
        ])  # Shape (4,2)

        # Rotation matrices for all time steps
        cos_yaw = ca.cos(yaw)  # Shape (n, 1)
        sin_yaw = ca.sin(yaw)  # Shape (n, 1)

        x_1_n = ca.mtimes(vertices_body[0, 0], cos_yaw) + ca.mtimes(vertices_body[0, 1], -sin_yaw) # shape: (n_steps, 4) x-coordinates of all corners for all timesteps
        y_1_n = ca.mtimes(vertices_body[0, 0], sin_yaw) + ca.mtimes(vertices_body[0, 1], cos_yaw) # shape: (n_steps, 4) y-coordinates of all corners for all timesteps
        
        x_2_n = ca.mtimes(vertices_body[1, 0], cos_yaw) + ca.mtimes(vertices_body[1, 1], -sin_yaw)
        y_2_n = ca.mtimes(vertices_body[1, 0], sin_yaw) + ca.mtimes(vertices_body[1, 1], cos_yaw)

        x_3_n = ca.mtimes(vertices_body[2, 0], cos_yaw) + ca.mtimes(vertices_body[2, 1], -sin_yaw)
        y_3_n = ca.mtimes(vertices_body[2, 0], sin_yaw) + ca.mtimes(vertices_body[2, 1], cos_yaw)

        x_4_n = ca.mtimes(vertices_body[3, 0], cos_yaw) + ca.mtimes(vertices_body[3, 1], -sin_yaw)
        y_4_n = ca.mtimes(vertices_body[3, 0], sin_yaw) + ca.mtimes(vertices_body[3, 1], cos_yaw)

        # reconstruct to achieve 4 instances (corners) of shape (n_steps, 2)
        corner_1_ned = ca.MX(ca.horzcat(x_1_n + north, y_1_n + east))
        corner_2_ned = ca.MX(ca.horzcat(x_2_n + north, y_2_n + east))
        corner_3_ned = ca.MX(ca.horzcat(x_3_n + north, y_3_n + east))
        corner_4_ned = ca.MX(ca.horzcat(x_4_n + north, y_4_n + east))

        return corner_1_ned, corner_2_ned, corner_3_ned, corner_4_ned   # row_i = (x_i, y_i)
    
    def _compute_quayside_extension_point(self, extension_length) -> ca.MX:
        """
        Compute point that is extension to the side-piece of the quay.
        At this location, the vessel will not be able to touch the quay
        """
        start_pt_nparray = np.array(ca.evalf(self.sidequay_start)).flatten()
        end_pt_nparray = np.array(ca.evalf(self.sidequay_end)).flatten()
        pt_ext_mx = ca.reshape(ca.MX(point_extension_of_line(start_point=start_pt_nparray,
                                                  end_point=end_pt_nparray, d=extension_length)), 1, 2)
        return pt_ext_mx
    



