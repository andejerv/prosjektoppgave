from dataclasses import dataclass, field
import numpy as np
from typing import List


@dataclass
class Waypoint:
    pose: np.ndarray = field(default_factory=lambda: np.array([np.nan, np.nan, np.nan]))
    speed: float = None
    x_ep_threshold: float = None
    is_reached: bool = False
    # def __init__(self, pose=None, speed=None, x_ep_threshold=None, is_reached=False):
    #     self.pose = pose if pose is not None else np.array([np.nan, np.nan, np.nan])
    #     self.x_ep_threshold = x_ep_threshold
    #     self.speed = speed
    #     self.is_reached = is_reached

@dataclass
class PIDGains:
    kp: np.ndarray
    ki: np.ndarray
    kd: np.ndarray

@dataclass
class OptimalTrajectories:
    state: np.ndarray           # yaw is relative 0, 2D numpy array
    state_dot: np.ndarray       # 2D numpy array
    frontquay_line_points: np.ndarray     # 2D numpy array
    frontquay_line_normals: np.ndarray # 2D numpy array, unit length
    sidequay_line_points: np.ndarray     # 2D numpy array
    sidequay_line_normals: np.ndarray # 2D numpy array, unit length
    t: np.ndarray   # this is really a single float value, indicating the sim time whe nthe mpc is run


