import numpy as np
from typing import List, Dict
from dataclasses import dataclass


@dataclass
class ProcessedData:
    t: np.ndarray
    eta: np.ndarray
    eta_dot: np.ndarray
    eta_d: np.ndarray
    eta_d_dot: np.ndarray
    chi_d: np.ndarray
    chi: np.ndarray
    nu: np.ndarray
    nu_d: np.ndarray
    accel_body: np.ndarray
    control_forces_body: np.ndarray
    target_wp: List[np.ndarray]
    t_switch: np.ndarray

@dataclass  # used for storing computed metrics to file
class Metric:
    metric_type: str
    value: float

@dataclass
class MetricCollection:
    metric_collection: Dict[str, np.ndarray]

@dataclass
class Statistics:
    metric_type: str # eg. energy consumption, IAE ...
    mu: float   # mean
    sigma: float  # standard deviation

@dataclass
class StatisticsCollection:
    statistic_collection: Dict[str, Dict[str, Statistics]]    # Dict[controller_type, Dict[batch, statistics]]

@dataclass
class WindData:
    t: np.ndarray
    X: np.ndarray
    Y: np.ndarray
    N: np.ndarray
    
