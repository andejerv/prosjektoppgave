from dataclasses import dataclass
import numpy as np

# Parameters for reference filter
@dataclass
class VesselParams:
    omega_n = np.array([1.0, 1.0, 1.0])
    zeta = np.array([1.0, 1.0, 1.0])

    eta_d_dot_min = np.array([-3., -3., -0.4])
    eta_d_dot_max = np.array([3., 3., 0.4])
    eta_d_ddot_min = np.array([-0.25, -0.25, -0.05])
    eta_d_ddot_max = np.array([0.25, 0.25, 0.05])
    eta_d_dddot_min = np.array([-10., -10., -10.])
    eta_d_dddot_max = np.array([10., 10., 10.])


## Original values

# @dataclass
# class MA2Params:
#     omega_n = np.array([0.7, 0.7, 0.3])
#     zeta = np.array([1., 1., 1.])

#     eta_d_dot_min = np.array([-3., -3., -0.4])
#     eta_d_dot_max = np.array([3., 3., 0.4])
#     eta_d_ddot_min = np.array([-0.25, -0.25, -0.05])
#     eta_d_ddot_max = np.array([0.25, 0.25, 0.05])
#     eta_d_dddot_min = np.array([-10., -10., -10.])
#     eta_d_dddot_max = np.array([10., 10., 10.])


# @dataclass
# class EstelleParams:
#     omega_n = np.array([0.7, 0.7, 0.3])
#     zeta = np.array([1., 1., 1.])

#     eta_d_dot_min = np.array([-4.5, -4.5, -0.4])
#     eta_d_dot_max = np.array([4.5, 4.5, 0.4])
#     eta_d_ddot_min = np.array([-0.25, -0.25, -0.05])
#     eta_d_ddot_max = np.array([0.25, 0.25, 0.05])
#     eta_d_dddot_min = np.array([-10., -10., -10.])
#     eta_d_dddot_max = np.array([10., 10., 10.])