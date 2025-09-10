import numpy as np
import logging as pylog
from typing import Optional, Tuple, cast
from nptyping import NDArray, Float, Shape
from docking_algorithms.utils.integrator import Integrator
from docking_algorithms.utils.inf2pipi import inf2pipi


class SecondOrderReferenceFilter:
    """
    Second-order single DOF reference filter.

    The implementation is **NOT** thread safe.

    :param t0: Initial time.
    :param omega_n: Natural frequency
    :param zeta: Relative damping. Defaults to 1
    :param x_r0: Initial reference, defaults to 0
    :param x_r_dot0: Initial derivative reference, defaults to 0
    :param x_d_min: Optional saturation limits
    :param x_d_max: Optional saturation limits
    :param x_d_dot_min: Optional saturation limits
    :param x_d_dot_max: Optional saturation limits
    :param x_d_ddot_min: Optional saturation limits
    :param x_d_ddot_max: Optional saturation limits
    :param max_dt: Maximum allowable time step.
    :param x_is_angle: Set to True if x is an angle. Even if True, the state is
        not guaranteed to be in (-pi, pi] - but the internal state error will
        be wrapped to (-pi, pi]
    """

    def __init__(self,
                 t0=0.,
                 omega_n=1.,
                 zeta=1.,
                 x_r0=0.,
                 x_r_dot0=0.,
                 x_d_min: Optional[float] = None,
                 x_d_max: Optional[float] = None,
                 x_d_dot_min: Optional[float] = None,
                 x_d_dot_max: Optional[float] = None,
                 x_d_ddot_min: Optional[float] = None,
                 x_d_ddot_max: Optional[float] = None,
                 max_dt=1.,
                 x_is_angle=False,
                 logger=pylog) -> None:
        self._x_r = x_r0
        self._x_r_dot = x_r_dot0
        self._x_is_angle = x_is_angle

        def odefun(t: float, y: NDArray[Shape["2"], Float]) -> NDArray[
                Shape["2"], Float]:
            x_d, x_d_dot = self._y_to_elem(y)
            x_e: NDArray[Shape["3"], Float] = self._x_r - x_d
            if self._x_is_angle:
                x_e = inf2pipi(x_e)  # Wrap angle error
            x_e_dot = self._x_r_dot - x_d_dot
            x_d_ddot = -np.clip(-2*self._zeta*self._omega_n * x_e_dot - self._omega_n**2 * x_e,
                                self._x_d_ddot_min, self._x_d_ddot_max)

            return self._elem_to_y(x_d_dot, x_d_ddot)

        y0 = self._elem_to_y(x_r0, x_r_dot0)

        self.integrator = Integrator(y0, t0, max_dt, odefun=odefun,
                                     logger=logger)
        self._logger = logger
        self.update_params(omega_n, zeta,
                           x_d_min, x_d_max,
                           x_d_dot_min, x_d_dot_max,
                           x_d_ddot_min, x_d_ddot_max)

    def set_reference(self, x_r: Optional[float] = None, x_r_dot: Optional[float] = None) \
            -> None:
        """
        Update internal references.

        :param x_r: State referece, optional
        :param x_r_dot: Derivative reference, optional
        """
        if x_r is not None:
            self._x_r = x_r
        if x_r_dot is not None:
            self._x_r_dot = x_r_dot

    def step(self, t1: float, x_r: Optional[float] = None, x_r_dot: Optional[float] = None) \
            -> None:
        """
        Step function.

        :param t1: Time step end
        :param x_r: State reference. Defaults to/updates self.x_r
        :param x_r_dot: Derivative reference. Defaults to/updates self.x_r_dot
        """
        if x_r is not None:
            self._x_r = x_r
        if x_r_dot is not None:
            self._x_r_dot = x_r_dot
        self._logger.debug(  # type: ignore
            f'Ref filter step. x_r: {self._x_r}, x_r_dot: {self._x_r_dot}')
        self.integrator.step(t1)

    def update_params(
            self,
            omega_n=1.,
            zeta=1.,
            x_d_min: Optional[float] = None,
            x_d_max: Optional[float] = None,
            x_d_dot_min: Optional[float] = None,
            x_d_dot_max: Optional[float] = None,
            x_d_ddot_min: Optional[float] = None,
            x_d_ddot_max: Optional[float] = None) \
            -> None:
        """
        Update reference filter parameters.

        :param omega_n: Natural frequency. Defaults to 1.
        :param zeta: Relative damping. Defaults to 1.
        :param x_d_min: State min. Defaults to -inf
        :param x_d_max: State max. Defaults to inf
        :param x_d_dot_min: Derivative min. Defaults to -inf
        :param x_d_dot_max: Derivative max. Defaults to inf
        :param x_d_ddot_min: Second derivative min. Defaults to -inf
        :param x_d_ddot_max: Second derivative max. Defaults to inf
        """
        self._logger.debug(
            "Updating parameters with: "
            f"omega_n: {omega_n}, "
            f"zeta: {zeta}, "
            f"x_d_min: {x_d_min}, "
            f"x_d_max: {x_d_max}, "
            f"x_d_dot_min: {x_d_dot_min}, "
            f"x_d_dot_max: {x_d_dot_max}, "
            f"x_d_ddot_min: {x_d_ddot_min}, "
            f"x_d_ddot_max: {x_d_ddot_max}."
        )
        if x_d_min is None:
            x_d_min = -np.inf
        if x_d_max is None:
            x_d_max = np.inf
        if x_d_dot_min is None:
            x_d_dot_min = -np.inf
        if x_d_dot_max is None:
            x_d_dot_max = np.inf
        if x_d_ddot_min is None:
            x_d_ddot_min = -np.inf
        if x_d_ddot_max is None:
            x_d_ddot_max = np.inf

        # Store parameters
        self._omega_n = omega_n
        self._zeta = zeta
        self._x_d_min = x_d_min
        self._x_d_max = x_d_max
        self._x_d_dot_min = x_d_dot_min
        self._x_d_dot_max = x_d_dot_max
        self._x_d_ddot_min = x_d_ddot_min
        self._x_d_ddot_max = x_d_ddot_max

        sat_min = self._elem_to_y(x_d_min, x_d_dot_min)
        sat_max = self._elem_to_y(x_d_max, x_d_dot_max)

        self.integrator.set_sat(sat_min, sat_max)

    def get_desired_values(self) -> tuple[float, float, float]:
        r"""
        Get desired values.

        :return: (t, x_d, x_d_dot)
        """
        x_d, x_d_dot = self._y_to_elem(self.integrator.y)
        return self.integrator.t, x_d, x_d_dot

    def set_states(self, t: Optional[float] = None, x_d: Optional[float] = None,
                   x_d_dot: Optional[float] = None) -> None:
        """
        Set reference filter states.

        All arguments are optional, only those specified are set.

        :param t: Time
        :param x_d: State
        :param x_d_dot: Derivative
        """
        _x_d, _x_d_dot = self._y_to_elem(self.integrator.y)
        if x_d is not None:
            _x_d = x_d
        if x_d_dot is not None:
            _x_d_dot = x_d_dot

        self.integrator.y = self._elem_to_y(_x_d, _x_d_dot)
        if t is not None:
            self.integrator.t = t

    @staticmethod
    def _y_to_elem(y: NDArray[Shape["2"], Float]) -> tuple[float, float]:
        x_d = y[0]
        x_d_dot = y[1]
        return x_d, x_d_dot

    @staticmethod
    def _elem_to_y(x_d: float, x_d_dot: float) -> NDArray[Shape["2"], Float]:
        return np.array([x_d, x_d_dot])

    @property
    def x_r(self) -> float:
        return self._x_r

    @property
    def x_r_dot(self) -> float:
        return self._x_r_dot

    @property
    def t(self) -> float:
        return self.integrator.t


class ThirdOrderReferenceFilter:
    """
    Third-order single DOF reference filter.

    The implementation is **NOT** thread safe.

    :param t0: Initial time.
    :param omega_n: Natural frequency
    :param zeta: Relative damping. Defaults to 1
    :param x_r0: Initial reference, defaults to 0
    :param x_r_dot0: Initial derivative reference, defaults to 0
    :param x_d_min: Optional saturation limits
    :param x_d_max: Optional saturation limits
    :param x_d_dot_min: Optional saturation limits
    :param x_d_dot_max: Optional saturation limits
    :param x_d_ddot_min: Optional saturation limits
    :param x_d_ddot_max: Optional saturation limits
    :param x_d_dddot_min: Optional saturation limits
    :param x_d_dddot_max: Optional saturation limits
    :param max_dt: Maximum allowable time step.
    :param x_is_angle: Set to True if x is an angle. Even if True, the state is
        not guaranteed to be in (-pi, pi] - but the internal state error will
        be wrapped to (-pi, pi]
    """

    def __init__(self,
                 t0=0.,
                 omega_n=1.,
                 zeta=1.,
                 x_r0=0.,
                 x_r_dot0=0.,
                 x_r_ddot0=0.,
                 x_d_min: Optional[float] = None,
                 x_d_max: Optional[float] = None,
                 x_d_dot_min: Optional[float] = None,
                 x_d_dot_max: Optional[float] = None,
                 x_d_ddot_min: Optional[float] = None,
                 x_d_ddot_max: Optional[float] = None,
                 x_d_dddot_min: Optional[float] = None,
                 x_d_dddot_max: Optional[float] = None,
                 max_dt=1.,
                 x_is_angle=False,
                 logger=pylog) -> None:
        self._x_r = x_r0
        self._x_r_dot = x_r_dot0
        self._x_r_ddot = x_r_ddot0
        self._x_is_angle = x_is_angle

        def odefun(t: float, y: NDArray[Shape["3"], Float]) -> NDArray[
                Shape["3"], Float]:
            x_d, x_d_dot, x_d_ddot = self._y_to_elem(y)
            x_e = self._x_r - x_d
            if self._x_is_angle:
                x_e = inf2pipi(x_e)  # Wrap angle error
            x_e_dot = self._x_r_dot - x_d_dot
            x_e_ddot = self._x_r_ddot - x_d_ddot
            x_d_dddot = -np.clip(
                - (2*self._zeta + 1)*self._omega_n * x_e_ddot
                - (2*self._zeta + 1)*self._omega_n**2 * x_e_dot
                - self._omega_n**3 * x_e,
                self._x_d_dddot_min, self._x_d_dddot_max)

            return self._elem_to_y(x_d_dot, x_d_ddot, x_d_dddot)

        y0 = self._elem_to_y(x_r0, x_r_dot0, x_r_ddot0)

        self.integrator = Integrator(y0, t0, max_dt, odefun=odefun,
                                     logger=logger)
        self._logger = logger
        self.update_params(omega_n, zeta,
                           x_d_min, x_d_max,
                           x_d_dot_min, x_d_dot_max,
                           x_d_ddot_min, x_d_ddot_max,
                           x_d_dddot_min, x_d_dddot_max)

    def set_reference(self, x_r: Optional[float] = None, x_r_dot: Optional[float] = None,
                      x_r_ddot: Optional[float] = None) -> None:
        """
        Update internal references.

        :param x_r: State referece, optional
        :param x_r_dot: Derivative reference, optional
        :param x_r_dot: Second order derivative reference, optional
        """
        if x_r is not None:
            self._x_r = x_r
        if x_r_dot is not None:
            self._x_r_dot = x_r_dot
        if x_r_ddot is not None:
            self._x_r_ddot = x_r_ddot

    def step(self, t1: float, x_r: Optional[float] = None, x_r_dot: Optional[float] = None,
             x_r_ddot: Optional[float] = None) -> None:
        """
        Step function.

        :param t1: Time step end
        :param x_r: State reference. Defaults to/updates self.x_r
        :param x_r_dot: Derivative reference. Defaults to/updates self.x_r_dot
        """
        if x_r is not None:
            self._x_r = x_r
        if x_r_dot is not None:
            self._x_r_dot = x_r_dot
        if x_r_ddot is not None:
            self._x_r_ddot = x_r_ddot
        self._logger.debug(  # type: ignore
            f'Ref filter step. x_r: {self._x_r}, x_r_dot: {self._x_r_dot}, '
            f'x_r_ddot: {self._x_r_ddot}')
        self.integrator.step(t1)

    def update_params(
            self,
            omega_n=1.,
            zeta=1.,
            x_d_min: Optional[float] = None,
            x_d_max: Optional[float] = None,
            x_d_dot_min: Optional[float] = None,
            x_d_dot_max: Optional[float] = None,
            x_d_ddot_min: Optional[float] = None,
            x_d_ddot_max: Optional[float] = None,
            x_d_dddot_min: Optional[float] = None,
            x_d_dddot_max: Optional[float] = None) \
            -> None:
        """
        Update reference filter parameters.

        :param omega_n: Natural frequency. Defaults to 1.
        :param zeta: Relative damping. Defaults to 1.
        :param x_d_min: State min. Defaults to -inf
        :param x_d_max: State max. Defaults to inf
        :param x_d_dot_min: Derivative min. Defaults to -inf
        :param x_d_dot_max: Derivative max. Defaults to inf
        :param x_d_ddot_min: Second derivative min. Defaults to -inf
        :param x_d_ddot_max: Second derivative max. Defaults to inf
        :param x_d_dddot_min: Third derivative min. Defaults to -inf
        :param x_d_dddot_max: Third derivative max. Defaults to inf
        """
        self._logger.debug(
            "Updating parameters with: "
            f"omega_n: {omega_n}, "
            f"zeta: {zeta}, "
            f"x_d_min: {x_d_min}, "
            f"x_d_max: {x_d_max}, "
            f"x_d_dot_min: {x_d_dot_min}, "
            f"x_d_dot_max: {x_d_dot_max}, "
            f"x_d_ddot_min: {x_d_ddot_min}, "
            f"x_d_ddot_max: {x_d_ddot_max}, "
            f"x_d_dddot_min: {x_d_dddot_min}, "
            f"x_d_dddot_max: {x_d_dddot_max}."
        )
        if x_d_min is None:
            x_d_min = -np.inf
        if x_d_max is None:
            x_d_max = np.inf
        if x_d_dot_min is None:
            x_d_dot_min = -np.inf
        if x_d_dot_max is None:
            x_d_dot_max = np.inf
        if x_d_ddot_min is None:
            x_d_ddot_min = -np.inf
        if x_d_ddot_max is None:
            x_d_ddot_max = np.inf
        if x_d_dddot_min is None:
            x_d_dddot_min = -np.inf
        if x_d_dddot_max is None:
            x_d_dddot_max = np.inf

        # Store parameters
        self._omega_n = omega_n
        self._zeta = zeta
        self._x_d_min = x_d_min
        self._x_d_max = x_d_max
        self._x_d_dot_min = x_d_dot_min
        self._x_d_dot_max = x_d_dot_max
        self._x_d_ddot_min = x_d_ddot_min
        self._x_d_ddot_max = x_d_ddot_max
        self._x_d_dddot_min = x_d_dddot_min
        self._x_d_dddot_max = x_d_dddot_max

        sat_min = self._elem_to_y(x_d_min, x_d_dot_min, x_d_ddot_min)
        sat_max = self._elem_to_y(x_d_max, x_d_dot_max, x_d_ddot_max)

        self.integrator.set_sat(sat_min, sat_max)

    def get_desired_values(self) -> tuple[float, float, float, float]:
        r"""
        Get desired values.

        :return: (t, x_d, x_d_dot, x_d_ddot)
        """
        x_d, x_d_dot, x_d_ddot = self._y_to_elem(self.integrator.y)
        return self.integrator.t, x_d, x_d_dot, x_d_ddot

    def set_states(self, t: Optional[float] = None, x_d: Optional[float] = None,
                   x_d_dot: Optional[float] = None, x_d_ddot: Optional[float] = None) -> None:
        """
        Set reference filter states.

        All arguments are optional, only those specified are set.

        :param t: Time
        :param x_d: State
        :param x_d_dot: State derivative
        :param x_d_ddot: State second derivative
        """
        _x_d, _x_d_dot, _x_d_ddot = self._y_to_elem(self.integrator.y)
        if x_d is not None:
            _x_d = x_d
        if x_d_dot is not None:
            _x_d_dot = x_d_dot
        if x_d_ddot is not None:
            _x_d_ddot = x_d_ddot

        self.integrator.y = self._elem_to_y(_x_d, _x_d_dot, _x_d_ddot)
        if t is not None:
            self.integrator.t = t

    @staticmethod
    def _y_to_elem(y: NDArray[Shape["3"], Float]) -> tuple[float, float, float]:
        x_d = y[0]
        x_d_dot = y[1]
        x_d_ddot = y[2]
        return x_d, x_d_dot, x_d_ddot

    @staticmethod
    def _elem_to_y(x_d: float, x_d_dot: float, x_d_ddot: float) -> NDArray[Shape["3"], Float]:
        return np.array([x_d, x_d_dot, x_d_ddot])

    @property
    def x_r(self) -> float:
        return self._x_r

    @property
    def x_r_dot(self) -> float:
        return self._x_r_dot

    @property
    def x_r_ddot(self) -> float:
        return self._x_r_ddot

    @property
    def t(self) -> float:
        return self.integrator.t


class ThreeDOFReferenceFilter:
    """
    Third-order reference filter with saturation and velocity feed-forward.

    The implementation is **NOT** thread safe.

    :param t0: Initial time.
    :param omega_n: Natural frequencies, shape (3,)
    :param zeta: Relative damping, shape (3,). Defaults to [1, 1, 1]
    :param eta_r0: Initial pose reference, defaults to [0, 0, 0]
    :param eta_r_dot0: Initial velocity reference, defaults to [0, 0, 0]
    :param eta_r_ddot0: Initial acceleration reference, defaults to [0, 0, 0]
    :param eta_d_dot_min: Optional saturation limits, shape (3,)
    :param eta_d_dot_max: Optional saturation limits, shape (3,)
    :param eta_d_ddot_min: Optional saturation limits, shape (3,)
    :param eta_d_ddot_max: Optional saturation limits, shape (3,)
    :param eta_d_dddot_min: Optional saturation limits, shape (3,)
    :param eta_d_dddot_max: Optional saturation limits, shape (3,)
    :param max_dt: Maximum allowable time step.
    """

    def __init__(self,
                 t0: float = 0.,
                 omega_n: Optional[NDArray[Shape["3"], Float]] = None,
                 zeta: Optional[NDArray[Shape["3"], Float]] = None,
                 eta_r0: Optional[NDArray[Shape["3"], Float]] = None,
                 eta_r_dot0: Optional[NDArray[Shape["3"], Float]] = None,
                 eta_r_ddot0: Optional[NDArray[Shape["3"], Float]] = None,
                 eta_d_dot_min: Optional[NDArray[Shape["3"], Float]] = None,
                 eta_d_dot_max: Optional[NDArray[Shape["3"], Float]] = None,
                 eta_d_ddot_min: Optional[NDArray[Shape["3"], Float]] = None,
                 eta_d_ddot_max: Optional[NDArray[Shape["3"], Float]] = None,
                 eta_d_dddot_min: Optional[NDArray[Shape["3"], Float]] = None,
                 eta_d_dddot_max: Optional[NDArray[Shape["3"], Float]] = None,
                 max_dt: float = 1., logger=pylog) -> None:
        if eta_r0 is None:
            eta_r0 = np.zeros(3, dtype=float)
        if eta_r_dot0 is None:
            eta_r_dot0 = np.zeros(3, dtype=float)
        if eta_r_ddot0 is None:
            eta_r_ddot0 = np.zeros(3, dtype=float)
        self._eta_r: NDArray[Shape["3"], Float] = eta_r0
        self._eta_r_dot: NDArray[Shape["3"], Float] = eta_r_dot0
        self._eta_r_ddot: NDArray[Shape["3"], Float] = eta_r_ddot0

        def odefun(t: float, y: NDArray[Shape["9"], Float]) -> NDArray[
                Shape["9"], Float]:
            eta_d, eta_d_dot, eta_d_ddot = self._y_to_elem(y)
            eta_e: NDArray[Shape["3"], Float] = self._eta_r - eta_d
            eta_e[2] = inf2pipi(eta_e[2])  # Wrap angle error
            eta_e_dot: NDArray[Shape["3"], Float] = self._eta_r_dot - eta_d_dot
            eta_e_ddot: NDArray[Shape["3"], Float] = self._eta_r_ddot - eta_d_ddot

            eta_d_dddot = np.clip(self._A_33.dot(-eta_e_ddot)
                                  + self._A_32.dot(-eta_e_dot)
                                  + self._A_31.dot(-eta_e),
                                  self._eta_d_dddot_min,
                                  self._eta_d_dddot_max)

            return self._elem_to_y(eta_d_dot, eta_d_ddot, eta_d_dddot)

        y0 = self._elem_to_y(eta_r0, eta_r_dot0, eta_r_ddot0)

        self.integrator = Integrator(y0, t0, max_dt, odefun=odefun,
                                     logger=logger)
        self._logger = logger
        self.update_params(omega_n, zeta,
                           eta_d_dot_min, eta_d_dot_max,
                           eta_d_ddot_min, eta_d_ddot_max,
                           eta_d_dddot_min, eta_d_dddot_max)

    def step(self, t1: float,
             eta_r: Optional[NDArray[Shape["3"], Float]] = None,
             eta_r_dot: Optional[NDArray[Shape["3"], Float]] = None,
             eta_r_ddot: Optional[NDArray[Shape["3"], Float]] = None) -> None:
        """
        Step function.

        Accepts discontinuous angle references, will keep angles continuous
        internally.

        :param t1: Time step end
        :param eta_r: Reference pose. Defaults to/updates self.eta_r
        :param eta_r_dot: Reference velocity. Defaults to/updates
            self.eta_r_dot
        :param eta_r_ddot: Reference acceleration. Defaults to/updates
            self.eta_r_ddot
        """
        if eta_r is not None:
            self._eta_r = np.asarray(eta_r)
        if eta_r_dot is not None:
            self._eta_r_dot = np.asarray(eta_r_dot)
        if eta_r_ddot is not None:
            self._eta_r_ddot = eta_r_ddot
        self._logger.debug(  # type: ignore
            'Ref filter step. eta_r: {}, eta_r_dot: {}, eta_r_ddot: {}'.format(self._eta_r,
                                                                               self._eta_r_dot,
                                                                               self._eta_r_ddot))
        self.integrator.step(t1)

    def set_reference(self, eta_r: Optional[NDArray[Shape["3"], Float]] = None,
                      eta_r_dot: Optional[NDArray[Shape["3"], Float]] = None,
                      eta_r_ddot: Optional[NDArray[Shape["3"], Float]] = None) \
            -> None:
        """
        Update internal references.

        :param eta_r: Pose, optional
        :param eta_r_dot: Velocity, optional
        :param eta_r_ddot: Acceleration, optional
        """
        if eta_r is not None:
            self._eta_r = np.asarray(eta_r)
        if eta_r_dot is not None:
            self._eta_r_dot = np.asarray(eta_r_dot)
        if eta_r_ddot is not None:
            self._eta_r_ddot = np.asarray(eta_r_ddot)

    def get_desired_values(self) -> Tuple[
            float, NDArray[Shape["3"], Float], NDArray[Shape["3"], Float],
            NDArray[Shape["3"], Float]]:
        r"""
        Get desired values.

        Heading output is continuous (not constrained to
        :math:`(-\pi, \pi])`, and may differ from heading reference by
        :math:`n2\pi`.

        :rtype: tuple
        :return: (t, eta_d, eta_d_dot, eta_d_ddot)
        """
        eta_d, eta_d_dot, eta_d_ddot = self._y_to_elem(self.integrator.y)
        return self.integrator.t, eta_d, eta_d_dot, eta_d_ddot

    def set_states(self, t: Optional[float] = None,
                   eta_d: Optional[NDArray[Shape["3"], Float]] = None,
                   eta_d_dot: Optional[NDArray[Shape["3"], Float]] = None,
                   eta_d_ddot: Optional[NDArray[Shape["3"], Float]] = None) \
            -> None:
        """
        Set reference filter states.

        All arguments are optional, only those specified are set.

        :param t: Time
        :param eta_d: Pose
        :param eta_d_dot: Velocity
        :param eta_d_ddot: Acceleration
        """
        _eta_d, _eta_d_dot, _eta_d_ddot = self._y_to_elem(self.integrator.y)
        if eta_d is not None:
            _eta_d = np.asarray(eta_d)
        if eta_d_dot is not None:
            _eta_d_dot = np.asarray(eta_d_dot)
        if eta_d_ddot is not None:
            _eta_d_ddot = np.asarray(eta_d_ddot)

        self.integrator.y = self._elem_to_y(_eta_d, _eta_d_dot, _eta_d_ddot)
        if t is not None:
            self.integrator.t = t

    def update_params(
            self,
            omega_n: Optional[NDArray[Shape["3"], Float]] = None,
            zeta: Optional[NDArray[Shape["3"], Float]] = None,
            eta_d_dot_min: Optional[NDArray[Shape["3"], Float]] = None,
            eta_d_dot_max: Optional[NDArray[Shape["3"], Float]] = None,
            eta_d_ddot_min: Optional[NDArray[Shape["3"], Float]] = None,
            eta_d_ddot_max: Optional[NDArray[Shape["3"], Float]] = None,
            eta_d_dddot_min: Optional[NDArray[Shape["3"], Float]] = None,
            eta_d_dddot_max: Optional[NDArray[Shape["3"], Float]] = None) \
            -> None:
        """
        Update reference filter parameters.

        :param omega_n: Natural frequency. Defaults to identity
        :param zeta: Relative damping. Defaults to identity
        :param eta_d_dot_min: Velocity min. Defaults to -inf
        :param eta_d_dot_max: Velocity max. Defaults to inf
        :param eta_d_ddot_min: Acceleration min. Defaults to -inf
        :param eta_d_ddot_max: Acceleration max. Defaults to inf
        :param eta_d_dddot_min: Jerk min. Defaults to -inf
        :param eta_d_dddot_max: Jerk max. Defaults to inf
        """
        self._logger.debug(
            "Updating parameters with: "
            f"omega_n: {omega_n}, "
            f"zeta: {zeta}, "
            f"eta_d_dot_min: {eta_d_dot_min}, "
            f"eta_d_dot_max: {eta_d_dot_max}, "
            f"eta_d_ddot_min: {eta_d_ddot_min}, "
            f"eta_d_ddot_max: {eta_d_ddot_max}, "
            f"eta_d_dddot_min: {eta_d_dddot_min}, "
            f"eta_d_dddot_max: {eta_d_dddot_max}."
        )
        if omega_n is None:
            omega_n = np.ones(3)
        if zeta is None:
            zeta = np.ones(3)
        if eta_d_dot_min is None:
            eta_d_dot_min = -np.inf*np.ones(3)
        if eta_d_dot_max is None:
            eta_d_dot_max = np.inf*np.ones(3)
        if eta_d_ddot_min is None:
            eta_d_ddot_min = -np.inf*np.ones(3)
        if eta_d_ddot_max is None:
            eta_d_ddot_max = np.inf*np.ones(3)
        if eta_d_dddot_min is None:
            eta_d_dddot_min = -np.inf*np.ones(3)
        if eta_d_dddot_max is None:
            eta_d_dddot_max = np.inf*np.ones(3)

        zeta = np.diag(zeta)
        omega_n = np.diag(omega_n)

        # Store parameters
        self._omega_n = cast(
            NDArray[Shape["3"], Float], omega_n)
        self._zeta = cast(
            NDArray[Shape["3"], Float], zeta)
        self._eta_d_dot_min = cast(
            NDArray[Shape["3"], Float], eta_d_dot_min)
        self._eta_d_dot_max = cast(
            NDArray[Shape["3"], Float], eta_d_dot_max)
        self._eta_d_ddot_min = cast(
            NDArray[Shape["3"], Float], eta_d_ddot_min)
        self._eta_d_ddot_max = cast(
            NDArray[Shape["3"], Float], eta_d_ddot_max)
        self._eta_d_dddot_min = cast(
            NDArray[Shape["3"], Float], eta_d_dddot_min)
        self._eta_d_dddot_max = cast(
            NDArray[Shape["3"], Float], eta_d_dddot_max)

        self._A_33: NDArray[Shape["3, 3"], Float] = -(
            2 * self._zeta + np.eye(3)).dot(self._omega_n)
        self._A_32: NDArray[Shape["3, 3"], Float] = self._A_33.dot(
            self._omega_n)
        self._A_31: NDArray[Shape["3, 3"], Float] = -np.linalg.matrix_power(
            self._omega_n, 3)

        sat_min = self._elem_to_y(-np.inf*np.ones(3),
                                  eta_d_dot_min,
                                  eta_d_ddot_min)
        sat_max = self._elem_to_y(np.inf*np.ones(3),
                                  eta_d_dot_max,
                                  eta_d_ddot_max)

        self.integrator.set_sat(sat_min, sat_max)

    @property
    def eta_r(self) -> NDArray[Shape["3"], Float]:
        return self._eta_r

    @property
    def eta_r_dot(self) -> NDArray[Shape["3"], Float]:
        return self._eta_r_dot

    @property
    def eta_r_ddot(self) -> NDArray[Shape["3"], Float]:
        return self._eta_r_ddot

    @property
    def t(self) -> float:
        return self.integrator.t

    @staticmethod
    def _y_to_elem(y: NDArray[Shape["9"], Float]) -> Tuple[
            NDArray[Shape["3"], Float], NDArray[Shape["3"], Float],
            NDArray[Shape["3"], Float]]:
        eta_d = y[:3]
        eta_d_dot = y[3:6]
        eta_d_ddot = y[6:]
        return eta_d, eta_d_dot, eta_d_ddot

    @staticmethod
    def _elem_to_y(eta_d: NDArray[Shape["3"], Float],
                   eta_d_dot: NDArray[Shape["3"], Float],
                   eta_d_ddot: NDArray[Shape["3"], Float]) -> \
            NDArray[Shape["9"], Float]:
        return np.hstack((eta_d, eta_d_dot, eta_d_ddot))
