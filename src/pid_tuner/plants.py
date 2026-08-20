"""Plant models used by the PID tuner.

The original project already separated plant dynamics from the controller.  This
module keeps that good design and makes the interface explicit.  Every plant:

* exposes a human-readable description for the UI;
* has realistic default actuator limits;
* implements ``output()`` and ``step()``;
* can be reset to its initial condition; and
* declares whether open-loop reaction-curve tuning is meaningful.

The models are intentionally lightweight educational approximations rather than
high-fidelity hardware models.  Fourth-order Runge-Kutta (RK4) integration is
used so the simulator does not need SciPy.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable, Sequence
from math import isfinite

State = list[float]
Dynamics = Callable[[Sequence[float], float, float], Sequence[float]]


def rk4_step(
    dynamics: Dynamics,
    state: Sequence[float],
    control: float,
    disturbance: float,
    dt: float,
) -> State:
    """Advance an ODE by one fourth-order Runge-Kutta step.

    ``dynamics(state, control, disturbance)`` must return derivatives in the
    same order and dimension as ``state``.
    """
    if dt <= 0.0 or not isfinite(dt):
        raise ValueError("dt must be a positive finite number")

    def add_scaled(x: Sequence[float], dx: Sequence[float], scale: float) -> State:
        return [xi + scale * dxi for xi, dxi in zip(x, dx, strict=True)]

    k1 = list(dynamics(state, control, disturbance))
    k2 = list(dynamics(add_scaled(state, k1, dt / 2.0), control, disturbance))
    k3 = list(dynamics(add_scaled(state, k2, dt / 2.0), control, disturbance))
    k4 = list(dynamics(add_scaled(state, k3, dt), control, disturbance))

    if not (len(state) == len(k1) == len(k2) == len(k3) == len(k4)):
        raise ValueError("Dynamics must preserve the state dimension")

    return [
        xi + (dt / 6.0) * (a + 2.0 * b + 2.0 * c + d)
        for xi, a, b, c, d in zip(state, k1, k2, k3, k4, strict=True)
    ]


class BasePlant(ABC):
    """Small common interface shared by all simulated plants."""

    name: str
    short_name: str
    description: str
    equation: str
    control_label: str
    control_unit: str
    output_label: str
    output_unit: str
    default_setpoint: float
    output_range: tuple[float, float]
    control_limits: tuple[float, float]
    default_pid: dict[str, float]

    # Open-loop FOPDT identification is deliberately disabled for unstable
    # plants rather than hoping the numerical fit happens to fail.
    supports_open_loop_tuning: bool = True
    open_loop_tuning_duration: float = 30.0
    open_loop_step_size: float = 1.0

    @abstractmethod
    def reset(self) -> None:
        """Restore the plant's initial condition."""

    @abstractmethod
    def output(self) -> float:
        """Return the true controlled output."""

    @abstractmethod
    def step(self, control: float, dt: float, disturbance: float = 0.0) -> float:
        """Advance the plant and return the new true output."""


class DCMotorSpeed(BasePlant):
    r"""First-order approximation of DC motor speed dynamics.

    The electrical transient is neglected, giving

    ``J * dω/dt = Kt*u - b*ω - T_load - d``.

    Here ``u`` is a voltage-equivalent actuator effort and ``d`` is an
    additional load torque disturbance.  The model is deliberately simple so a
    reader can see how PID terms affect a familiar stable first-order plant.
    """

    name = "DC Motor (Speed Control)"
    short_name = "dc_motor"
    description = (
        "Stable first-order speed plant. A good baseline for seeing rise time, "
        "integral action, actuator saturation, and load-torque rejection."
    )
    equation = "J·dω/dt = Kₜu − bω − T_load − d"
    control_label = "Voltage-equivalent effort"
    control_unit = "arb. V"
    output_label = "Angular velocity"
    output_unit = "rad/s"
    default_setpoint = 100.0
    output_range = (0.0, 160.0)
    control_limits = (-24.0, 24.0)
    default_pid = {"kp": 0.8, "ki": 4.0, "kd": 0.02}
    supports_open_loop_tuning = True
    open_loop_tuning_duration = 4.0
    open_loop_step_size = 2.0

    def __init__(
        self,
        inertia: float = 0.02,
        damping: float = 0.2,
        torque_constant: float = 1.0,
        load_torque: float = 0.0,
    ) -> None:
        if inertia <= 0.0:
            raise ValueError("inertia must be positive")
        if damping < 0.0:
            raise ValueError("damping cannot be negative")
        self.inertia = inertia
        self.damping = damping
        self.torque_constant = torque_constant
        self.load_torque = load_torque
        self.reset()

    def reset(self) -> None:
        self.state: State = [0.0]

    def _dynamics(
        self, state: Sequence[float], control: float, disturbance: float
    ) -> Sequence[float]:
        omega = state[0]
        domega = (
            self.torque_constant * control
            - self.damping * omega
            - self.load_torque
            - disturbance
        ) / self.inertia
        return [domega]

    def output(self) -> float:
        return self.state[0]

    def step(self, control: float, dt: float, disturbance: float = 0.0) -> float:
        self.state = rk4_step(self._dynamics, self.state, control, disturbance, dt)
        return self.output()


class ThermalSystem(BasePlant):
    r"""First-order thermal process around an ambient temperature.

    ``τ*dT/dt = -(T - T_ambient) + K*u + d``

    The disturbance ``d`` represents an additive heat-load term.  Positive
    values add heat; negative values increase effective heat loss.
    """

    name = "Thermal System (Heater)"
    short_name = "thermal"
    description = (
        "Slow stable process that makes integral action and windup easy to see. "
        "Its actuator is heater power, so the default lower limit is zero."
    )
    equation = "τ·dT/dt = −(T − Tₐ) + K·u + d"
    control_label = "Heater power"
    control_unit = "%"
    output_label = "Temperature"
    output_unit = "°C"
    default_setpoint = 60.0
    output_range = (15.0, 100.0)
    control_limits = (0.0, 100.0)
    default_pid = {"kp": 2.5, "ki": 0.15, "kd": 1.0}
    supports_open_loop_tuning = True
    open_loop_tuning_duration = 150.0
    open_loop_step_size = 10.0

    def __init__(
        self,
        time_constant: float = 25.0,
        process_gain: float = 2.0,
        ambient: float = 20.0,
    ) -> None:
        if time_constant <= 0.0:
            raise ValueError("time_constant must be positive")
        self.time_constant = time_constant
        self.process_gain = process_gain
        self.ambient = ambient
        self.reset()

    def reset(self) -> None:
        self.state: State = [self.ambient]

    def _dynamics(
        self, state: Sequence[float], control: float, disturbance: float
    ) -> Sequence[float]:
        temperature = state[0]
        dtemperature = (
            -(temperature - self.ambient)
            + self.process_gain * control
            + disturbance
        ) / self.time_constant
        return [dtemperature]

    def output(self) -> float:
        return self.state[0]

    def step(self, control: float, dt: float, disturbance: float = 0.0) -> float:
        self.state = rk4_step(self._dynamics, self.state, control, disturbance, dt)
        return self.output()


class InvertedPendulum(BasePlant):
    r"""Linearized inverted-pendulum angle model.

    State is ``[θ, θ_dot]`` and the simplified dynamics are

    ``θ_ddot = (g/L)θ + u/(M L) + d/(M L)``.

    The positive gravity term makes the upright equilibrium open-loop unstable.
    With ``error = setpoint - θ``, a positive proportional gain commands a
    negative force for positive ``θ``, opposing the fall in this sign
    convention.
    """

    name = "Inverted Pendulum (Angle Control)"
    short_name = "pendulum"
    description = (
        "Open-loop unstable benchmark used to show both PID stabilization and "
        "the limits of open-loop reaction-curve auto-tuning."
    )
    equation = "θ̈ = (g/L)θ + u/(M·L) + d/(M·L)"
    control_label = "Cart-force equivalent"
    control_unit = "N"
    output_label = "Pendulum angle"
    output_unit = "rad"
    default_setpoint = 0.0
    output_range = (-0.5, 0.5)
    control_limits = (-100.0, 100.0)
    default_pid = {"kp": 45.0, "ki": 5.0, "kd": 12.0}
    supports_open_loop_tuning = False
    open_loop_tuning_duration = 5.0
    open_loop_step_size = 1.0

    def __init__(
        self,
        cart_mass: float = 1.0,
        pendulum_length: float = 0.5,
        gravity: float = 9.81,
        initial_angle: float = 0.15,
    ) -> None:
        if cart_mass <= 0.0 or pendulum_length <= 0.0 or gravity <= 0.0:
            raise ValueError("cart_mass, pendulum_length, and gravity must be positive")
        self.cart_mass = cart_mass
        self.pendulum_length = pendulum_length
        self.gravity = gravity
        self.initial_angle = initial_angle
        self.reset()

    def reset(self) -> None:
        self.state: State = [self.initial_angle, 0.0]

    def _dynamics(
        self, state: Sequence[float], control: float, disturbance: float
    ) -> Sequence[float]:
        theta, theta_dot = state
        theta_ddot = (
            (self.gravity / self.pendulum_length) * theta
            + control / (self.cart_mass * self.pendulum_length)
            + disturbance / (self.cart_mass * self.pendulum_length)
        )
        return [theta_dot, theta_ddot]

    def output(self) -> float:
        return self.state[0]

    def step(self, control: float, dt: float, disturbance: float = 0.0) -> float:
        self.state = rk4_step(self._dynamics, self.state, control, disturbance, dt)
        return self.output()


PLANT_REGISTRY: dict[str, type[BasePlant]] = {
    DCMotorSpeed.short_name: DCMotorSpeed,
    ThermalSystem.short_name: ThermalSystem,
    InvertedPendulum.short_name: InvertedPendulum,
}


def create_plant(key: str) -> BasePlant:
    """Create a fresh plant by registry key with a useful error message."""
    try:
        return PLANT_REGISTRY[key]()
    except KeyError as exc:
        valid = ", ".join(sorted(PLANT_REGISTRY))
        raise KeyError(f"Unknown plant {key!r}. Valid plants: {valid}") from exc
