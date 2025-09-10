import numpy as np
from typing import TypeVar, Any
from nptyping import NDArray, Float
import casadi as ca

T = TypeVar("T", float, NDArray[Any, Float], int, np.ndarray, np.float64, np.float32, ca.MX, ca.SX)


def inf2pipi(ang: T) -> T:
    """
    Wrap angle to (-pi,pi].

    https://stackoverflow.com/questions/15927755/opposite-of-numpy-unwrap.

    :param ang: Angle(s)
    :return: Wrapped angle(s)
    """
    if isinstance(ang, (float, int, np.ndarray, np.float64, np.float32)):
        return -((-ang + np.pi) % (2 * np.pi) - np.pi)
    elif isinstance(ang, (ca.MX, ca.SX)):
        mod_result = ca.fmod(-ang + ca.pi, 2*ca.pi)
        # Ensure the result is in the range [0, 2*pi)
        mod_result += (mod_result < 0) * 2*ca.pi  # Add 2*pi if mod_result is negative
        final_result = -(mod_result - ca.pi) # Adjust to the range [-pi, pi)
        return final_result
    else:
        raise TypeError("Unsupported input type. Expected float, NumPy array, or CasADi MX/SX.")

