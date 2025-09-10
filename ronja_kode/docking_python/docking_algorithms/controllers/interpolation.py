import numpy as np
from scipy.interpolate import interp1d

from docking_algorithms.utils.types import OptimalTrajectories
from docking_algorithms.utils.inf2pipi import inf2pipi
# from generate_results.utils import unwrap_angle


class InterpolateMPCOutput:
    def __init__(self, dt: float, sequence: OptimalTrajectories) -> None:
        self.dt = dt    # elements in sequence are sampled at fixed intervals dt
        self.N = sequence.state.shape[0]
        self.t_vec = sequence.t + np.arange(self.N)*dt

        self.x_vals = sequence.state[:, 0]
        self.y_vals = sequence.state[:, 1]
        self.psi_vals = np.unwrap(sequence.state[:, 2]) # avoids jumps across plus/minus pi

        self.xdot_vals = sequence.state_dot[:, 0]
        self.ydot_vals = sequence.state_dot[:, 1]
        self.psidot_vals = sequence.state_dot[:, 2]

        # Interpolators
        self. x_interp = interp1d(self.t_vec, self.x_vals, kind='cubic', fill_value='extrapolate')
        self.y_interp = interp1d(self.t_vec, self.y_vals, kind='cubic', fill_value='extrapolate')
        self.psi_interp = interp1d(self.t_vec, self.psi_vals, kind='cubic', fill_value='extrapolate')

        self.xdot_interp = interp1d(self.t_vec, self.xdot_vals, kind='cubic', fill_value='extrapolate')
        self.ydot_interp = interp1d(self.t_vec, self.ydot_vals, kind='cubic', fill_value='extrapolate')
        self.psidot_interp = interp1d(self.t_vec, self.psidot_vals, kind='cubic', fill_value='extrapolate')

        # Quay lines
        # self.front_points = sequence.frontquay_line_points
        # self.front_normals = sequence.frontquay_line_normals
        # self.side_points = sequence.sidequay_line_points
        # self.side_normals = sequence.sidequay_line_normals



    def get_references(self, at_t: float) -> tuple[np.ndarray, np.ndarray]: # tuple[np.ndarray, np.ndarray, dict]
        t = np.clip(at_t, self.t_vec[0], self.t_vec[-1])
        pose = np.array([
            float(self.x_interp(t)),
            float(self.y_interp(t)),
            inf2pipi(float(self.psi_interp(t)))    # wrap back to (-pi, pi] (same interval as the input was)
        ])
        
        vel = np.array([
            float(self.xdot_interp(t)),
            float(self.ydot_interp(t)),
            float(self.psidot_interp(t))
            ])
        

        # # Step-wise lookup for quay-related data
        # # This is not really necessary, but logging in auto docking becomes cleaner
        # i = np.searchsorted(self.t_vec, t, side='right') - 1
        # i = np.clip(i, 0, self.N - 1)  # ensure index stays in valid range

        # quay_constraints_data = {
        #     "front_point": self.front_points[i],
        #     "front_normal": self.front_normals[i],
        #     "side_point": self.side_points[i],
        #     "side_normal": self.side_normals[i],
        # }
        return pose, vel#, quay_constraints_data