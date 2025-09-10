import numpy as np


# Eg. eta_dot = R @ nu
body_to_ned_transform = lambda psi: np.array([
    [np.cos(psi), -np.sin(psi), 0],
    [np.sin(psi), np.cos(psi), 0],
    [0, 0, 1]
])


path_tangential_to_NED = lambda pi_p: np.array([
    [np.cos(pi_p), -np.sin(pi_p)],
    [np.sin(pi_p), np.cos(pi_p)],
])