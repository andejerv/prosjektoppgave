import numpy as np
from docking_algorithms.utils.transformations import path_tangential_to_NED
from docking_algorithms.utils.computations import compute_distance_between
from docking_algorithms.utils.inf2pipi import inf2pipi
from docking_algorithms.utils.types import Waypoint


class Guidance:
    def __init__(self, dt: float, waypoint_list: list[Waypoint], cfg) -> None:
        self.wp_list = waypoint_list
        self.current_wp_index = 1    # wp P_{i+1} in LOS algorithm
        self.cfg = cfg
        self.look_ahead = cfg.look_ahead
        
        self.dt = dt    # timestep

        # LOS states
        self.pi_p: float = None     # path-tangential angle
        self.x_ep: float = None     # along-track error/distance
        self.y_ep: float = None     # cross-track error
        self.crab_angle: float = None

        self.chi_d: float = None    # desired course
        self.psi_d: float = None    # desired course


    def _compute_path_tangential_angle(self) -> None:
        prev_wp = self.wp_list[self.current_wp_index-1]
        curr_wp = self.wp_list[self.current_wp_index]
        delta_x = curr_wp.pose[0] - prev_wp.pose[0]
        delta_y = curr_wp.pose[1] - prev_wp.pose[1]
        pi_p = np.arctan2(delta_y, delta_x)
        pi_p = inf2pipi(ang=pi_p)
        # print(f'path tangential angle: {pi_p}')
        return pi_p

    def _compute_cross_and_along_track_error(self, eta: np.ndarray,
                                             start_point: np.ndarray, pi_p: float) -> None:
        """
        compute vessel distance in x and y direction from P_i, in tangential frame
        """
        R_pn = path_tangential_to_NED(pi_p=pi_p)
        error = R_pn.T @ (eta[:2] - start_point[:2])
        x_ep = error[0]
        y_ep = error[1]
        # print(f'along track:{x_ep}, cross track: {y_ep}')
        return x_ep, y_ep


    def linear_speed_law(self) -> float:
        """
        Linearly decreasing velocity from P_i to P_{i+1}
        """
        p_A = self.wp_list[self.current_wp_index-1].pose
        p_B = self.wp_list[self.current_wp_index].pose

        v_A = self.wp_list[self.current_wp_index-1].speed
        v_B = self.wp_list[self.current_wp_index].speed

        # look_ahead = self.wp_list[self.current_wp_index].x_ep_threshold
        # Take into account look-ahead: the wp is "reached" a distance look-ahead in front of actual wp
        distance_AB = compute_distance_between(pose1=p_A, pose2=p_B) - self.look_ahead  # distance between target waypoints

        
        effective_x_ep = max(self.x_ep, 0)  # avoids increase in speed when x_ep < 0
        desired_speed = effective_x_ep * (v_B - v_A)/distance_AB + v_A
        # print(f'desired speed: {desired_speed}')
        return desired_speed 
    
    def sigmoid_speed_law(self) -> float:
        """
        This is the function to use 
        """
        curr_wp = self.get_current_wp()
        curr_wp_pose = curr_wp.pose
        curr_wp_speed = curr_wp.speed
        if self.current_wp_index == len(self.wp_list) - 1:
            curr_wp_pose = np.array([87.5, 494.9, curr_wp_pose[2]])
        prev_wp = self.get_prev_wp()
        prev_wp_pose = prev_wp.pose
        prev_wp_speed = prev_wp.speed
        dist_prev2curr = np.linalg.norm(curr_wp_pose[:2]-prev_wp_pose[:2])
        dist_left_to_curr_wp = dist_prev2curr - self.x_ep
        # deceleration_threshold = (prev_wp_speed**2-curr_wp_speed**2)/(2*self.cfg.decel_accel)
        if self.current_wp_index == len(self.wp_list) - 1:
            deceleration_threshold = 0.8
            lambda_wp = 8
        if self.current_wp_index == len(self.wp_list) - 2:
            deceleration_threshold = 14
            lambda_wp = 8
        # print(f'deceleration threshold: {deceleration_threshold}')
        # print(f'distance segment: {dist_prev2curr}')

        # if dist_left_to_curr_wp <= deceleration_threshold:
        speed_factor = 1 / (1 + np.exp(-lambda_wp * (dist_left_to_curr_wp / deceleration_threshold - 0.5)))  # sigmoid factor
            # speed_factor = (dist_left_to_curr_wp / deceleration_threshold) ** 3 # cubic factor
        speed = curr_wp_speed + (prev_wp_speed - curr_wp_speed) * speed_factor
        # else:
        #     speed = prev_wp_speed

        # print(f'speed: {speed}')
        return speed
    
    def sigmoid_heading_law(self) -> float:
        """
        This is the function to use
        """
        curr_wp = self.get_current_wp()
        curr_wp_pose = curr_wp.pose
        prev_wp = self.get_prev_wp()
        prev_wp_pose = prev_wp.pose

        # Compute segment length and progress toward current waypoint
        dist_prev2curr = np.linalg.norm(curr_wp_pose[:2] - prev_wp_pose[:2])
        dist_left_to_curr_wp = dist_prev2curr - self.x_ep

        # Sigmoid parameters (consider making these waypoint- or config-dependent if needed)
        deceleration_threshold = 10
        lambda_wp = 6

        # Always apply sigmoid
        heading_factor = 1 / (1 + np.exp(-lambda_wp * (dist_left_to_curr_wp / deceleration_threshold - 0.5)))
        heading = curr_wp_pose[2] + (prev_wp_pose[2] - curr_wp_pose[2]) * heading_factor

        return inf2pipi(heading)


    def compute_LOS_course(self) -> float:
        if (self.pi_p is None or self.y_ep is None):
            raise ValueError("pi_p and/or y_ep must be set!")
        
        # look_ahead = self.wp_list[self.current_wp_index].x_ep_threshold
        chi_d = self.pi_p - np.arctan(self.y_ep/self.look_ahead)
        self.chi_d = inf2pipi(ang=chi_d)  # wrapping to (-pi, pi)
        # print(f'desired LOS course: {self.chi_d}')
        return self.chi_d

    def compute_position_reference(self, velocity_ref: np.array, dt: float,
                                   speed: float=None) -> np.ndarray:
        prev_wp_pose = self.wp_list[self.current_wp_index-1].pose
        
        if not speed is None:
            # print(f'using speed ref to compute pos ref')
            effective_x_ep = max(self.x_ep, 0)
            x_ep_d = effective_x_ep + dt*speed 
            y_ep_d = 0 
        else:
            # print(f'using velocity ref to compute pos ref')
            velocity_ref_tangential = path_tangential_to_NED(self.pi_p).T @ velocity_ref[:2]
            x_ep_d = dt*velocity_ref_tangential[0]
            y_ep_d = self.y_ep + dt*velocity_ref_tangential[1]

        pos_d_tangential = np.array([x_ep_d, y_ep_d])
        pos_d_ned = path_tangential_to_NED(self.pi_p) @ pos_d_tangential + prev_wp_pose[:2]
        # print(f'desired pos: {pos_d_ned}')
        return pos_d_ned
    
    def heading_law(self) -> float:
        p_A = self.wp_list[self.current_wp_index-1].pose
        p_B = self.wp_list[self.current_wp_index].pose

        psi_A = p_A[2]
        psi_B = p_B[2]
        psi_diff = inf2pipi(ang=(psi_B - psi_A))
        # look_ahead = self.wp_list[self.current_wp_index].x_ep_threshold
        distance_AB = compute_distance_between(pose1=p_A, pose2=p_B) - self.look_ahead  # distance between target waypoints

        effective_x_ep = max(self.x_ep, 0)
        desired_heading = effective_x_ep * (psi_diff)/distance_AB + psi_A
        # if self.x_ep >= 0:
        #     desired_heading = self.x_ep * (psi_diff)/distance_AB + psi_A
        # else:
        #     desired_heading = self._compute_desired_LOS_heading()

        desired_heading = inf2pipi(desired_heading)
        # print(f'desired heading: {desired_heading}')

        return desired_heading
    

    def has_reached_wp(self, eta: np.ndarray) -> bool:
        """
        This one is more robust than radius of acceptance
        TODO: consider taking y_ep into account?
        """
        curr_wp = self.wp_list[self.current_wp_index]
        prev_wp = self.wp_list[self.current_wp_index - 1]
        # print(f'pose1: {prev_wp.pose}, pose2: {curr_wp.pose}')
        self.pi_p = self._compute_path_tangential_angle()  # really only necessary in the first iteration
        prev_wp_pos = prev_wp.pose[:2]
        self.x_ep, self.y_ep = self._compute_cross_and_along_track_error(eta=eta, start_point=prev_wp_pos, pi_p=self.pi_p)
        # print(f'computing cross track: {self.y_ep}')

        segment_length = compute_distance_between(pose1=prev_wp.pose, pose2=curr_wp.pose)
        distance_left = segment_length - self.x_ep
        wp_switch_threshold = self.wp_list[self.current_wp_index].x_ep_threshold
        return distance_left <= wp_switch_threshold
    

    def end_of_wp_list(self):
        if self.current_wp_index == (len(self.wp_list)-1):
            return True    # reached the last target wp
        return False


    def update_waypoints(self, eta: np.ndarray) -> None:
        """
        Only run this if within threshold of flagging WP as reached
        """
        self.wp_list[self.current_wp_index].is_reached = True
        # Only increment target if not end of wp list
        if self.current_wp_index < len(self.wp_list)-1:
            self.current_wp_index += 1
            self.pi_p = self._compute_path_tangential_angle()
            # update along- and cross-track for the new segment
            current_wp = self.wp_list[self.current_wp_index].pose
            self.x_ep, self.y_ep = self._compute_cross_and_along_track_error(
                eta=eta, start_point=current_wp, pi_p=self.pi_p)
        return
    
    # def compute_end_stage_forces(
    #         self, vessel_pos: np.ndarray,
    #         current_force_body: np.ndarray=np.array([0, 0])):
    #     """
    #     Compute the surge and sway forces to move the vessel toward the docking point.
        
    #     Parameters:
    #     vessel_pos: np.array([x, y]) - Current vessel position.
    #     quay_corners: tuple of np.array - (corner, segment1_end, segment2_end)
    #     max_force: float - Maximum force to be applied.
    #     force_ramp: float - Rate of force increase per step.
    #     current_force: np.array([Fx, Fy]) - Current applied force.
        
    #     Returns:
    #     np.array([Fx, Fy]) - Updated control forces.
    #     """

    #     lat_quay_start = self.cfg.side_quay_segment.corner
    #     lat_quay_end = self.cfg.side_quay_segment.end
    #     lon_quay_start = self.front_quay_segment.corner
    #     lon_quay_end = self.front_quay_segment.end

    #     # Compute distances to both segments
    #     lat_dist = compute_distance_to_segment(vessel_pos, lat_quay_start, lat_quay_end)
    #     lon_dist = compute_distance_to_segment(vessel_pos, lon_quay_start, lon_quay_end)
        
    #     # Normalize distances to get force direction
    #     force_lat = -lat_dist if vessel_pos[0] > lat_quay_start[0] else lat_dist
    #     force_lon = -lon_dist if vessel_pos[1] > lon_quay_start[1] else lon_dist
        
    #     # Normalize total force magnitude
    #     force_magnitude = np.linalg.norm([force_lat, force_lon])
    #     if force_magnitude > 0:
    #         force_direction = np.array([force_lat, force_lon]) / force_magnitude
    #     else:
    #         force_direction = np.array([0, 0])
        
    #     # Compute target force magnitude (proportional to distance, capped at max_force)
    #     target_force = min(force_magnitude, self.cfg.max_force) * force_direction
        
    #     # Apply force ramping
    #     updated_force = current_force_body + np.clip(
    #         target_force - current_force_body,
    #         -self.cfg.force_ramp,
    #         self.cfg.force_ramp)
        
    #     return updated_force

    def get_inital_waypoint(self) -> np.ndarray:
        return self.wp_list[0]

    def get_current_wp(self) -> np.ndarray:
        return self.wp_list[self.current_wp_index]
    
    def get_prev_wp(self) -> np.ndarray:    # currently unused
        return self.wp_list[self.current_wp_index-1]

    def get_wp1(self) -> np.ndarray:
        return self.wp_list[-1]

    def get_wp2(self) -> np.ndarray:
        return self.wp_list[-2]

