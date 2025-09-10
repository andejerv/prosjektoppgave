import numpy as np
import logging as pylog
from nptyping import NDArray, Float, Shape
from typing import Callable, Optional, Tuple
from types import ModuleType
# flake8: F722 - ignore flake8 false positive,
# https://github.com/PyCQA/pyflakes/issues/687 TODO: Remove when fixed


def euler(odefun: Callable[[float, NDArray[Shape["*"], Float]],  # type: ignore # noqa: F722,E501 - Ignore flake8 false positive https://github.com/PyCQA/pyflakes/issues/687, remove when fixed
                           NDArray[Shape["*"], Float]],  # type: ignore # noqa: F722,E501
          t: NDArray[Shape["*"], Float], y0: NDArray[Shape["*"], Float]  # type: ignore # noqa: F722,E501
          ) -> NDArray[Shape["*, *"], Float]:  # type: ignore # noqa: F722,E501
    """
    Euler integrator function.

    :param odefun: Differential  function
    :param t: List of timesteps with initial timestep as first element
    :param y0: Initial state
    :return: Array with states. Shape is (num_t, num_y)
    """
    N = np.size(t)
    n = np.size(y0)
    y = np.full((N, n), np.NaN)
    y[0, :] = y0
    for i in range(N - 1):
        h = t[i + 1] - t[i]
        k = odefun(t[i], y[i, :])
        y[i + 1, :] = y[i, :] + h * k

    return np.squeeze(y)


def rk4(odefun: Callable[[float, NDArray[Shape["*"], Float]],  # type: ignore # noqa: F722,E501
                         NDArray[Shape["*"], Float]],  # type: ignore # noqa: F722,E501
        t: NDArray[Shape["*"], Float], y0: NDArray[Shape["*"], Float]  # type: ignore # noqa: F722,E501
        ) -> NDArray[Shape["*, *"], Float]:  # type: ignore # noqa: F722,E501
    """
    One-step RK4 integrator function.

    :param odefun: Differential  function
    :param t: List of timesteps with initial timestep as first element
    :param y0: Initial state
    :return: Array with states. Shape is (num_t, num_y), y0 is included as the
        first element.
    """
    N = np.size(t)
    n = np.size(y0)
    y = np.full((N, n), np.NaN)
    y[0, :] = y0
    for i in range(N - 1):
        h = t[i + 1] - t[i]
        k1 = odefun(t[i], y[i, :])
        k2 = odefun(t[i] + 0.5 * h, y[i, :] + 0.5 * h * k1)
        k3 = odefun(t[i] + 0.5 * h, y[i, :] + 0.5 * h * k2)
        k4 = odefun(t[i] + 1.0 * h, y[i, :] + 1.0 * h * k3)
        y[i + 1, :] = y[i, :] + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

    return y


class Integrator:
    """
    Explicit-step integrator.

    :param y0: initial state
    :param t0: initial time
    :param max_dt: maximum time step
    :param odefun: ode function odefun(t,y)
    :param integrator: integrator type, defaults to 'rk4'
    :param logger: Logger module, defaults to pylog
    :param sat_min: Optional lower state limit. Must have same dimension as
        y0
    :param sat_max: Optional upper state limit. Must have same dimension as
        y0
    """

    def __init__(self, y0: NDArray[Shape["*"], Float], t0: float,  # type: ignore # noqa: F722,E501
                 max_dt: float,
                 odefun: Callable[[float, NDArray[Shape["*"], Float]],  # type: ignore # noqa: F722,E501
                                  NDArray[Shape["*"], Float]],  # type: ignore # noqa: F722,E501
                 integrator: str = 'rk4', logger: ModuleType = pylog,
                 sat_min: Optional[NDArray[Shape["*"], Float]] = None,  # type: ignore # noqa: F722,E501
                 sat_max: Optional[NDArray[Shape["*"], Float]] = None  # type: ignore # noqa: F722,E501
                 ) -> None:
        if not isinstance(y0, np.ndarray):
            raise ValueError(
                f'y0 has to be of type numpy.ndarray, not {type(y0)}')
        self.y = y0
        self.t = t0
        self.max_dt = max_dt
        self.sat_min = sat_min
        self.sat_max = sat_max

        if integrator == "rk4":
            self.int_fun = rk4
        else:
            raise NotImplementedError(
                'Integrator {} not implemented'.format(integrator))

        self.logger = logger
        self.odefun = odefun

    def step(self, t1: float) -> Tuple[float, NDArray[Shape["*"], Float]]:  # type: ignore # noqa: F722,E501
        """
        Perform integrator step.

        :param t1: Step end time
        :return: (t1, y1)
        """
        t0 = self.t
        dt = t1 - t0
        if dt < 0:
            err = 'Step backwards in time. Got t1: %.3f, internal time: ' \
                  '%.3f' % (t1, t0)
            self.logger.critical(err)
            raise RuntimeError(err[0] % err[1:])
        elif dt == 0:
            err = 'Step with zero time. Got t1: %.3f, internal time: ' \
                  '%.3f' % (t1, t0)
            self.logger.warning(err)
            return self.t, self.y
        elif dt > self.max_dt:
            err = 'Too large step. Got t1: %.3f, internal time: %.3f. dt:' \
                  ' %.3f, maximum allowed: %.3f' % (t1, t0, dt, self.max_dt)
            self.logger.error(err)

        y1 = self.int_fun(self.odefun, np.array([t0, t1]), self.y)[1]

        # Saturate
        if self.sat_min is not None:
            y1 = np.maximum(y1, self.sat_min)
        if self.sat_max is not None:
            y1 = np.minimum(y1, self.sat_max)

        self.t = t1
        self.y = y1

        return t1, y1

    def set_sat(self, sat_min: Optional[NDArray[Shape["*"], Float]] = None,  # type: ignore # noqa: F722,E501
                sat_max: Optional[NDArray[Shape["*"], Float]] = None) -> None:  # type: ignore # noqa: F722,E501
        """
        Set saturation limits.

        :param sat_min: Optional lower state limit. Must have same dimension as
            y0
        :param sat_max: Optional upper state limit. Must have same dimension as
            y0
        """
        self.sat_min = sat_min
        self.sat_max = sat_max