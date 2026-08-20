"""Closed-loop simulation utilities.

This module connects a plant to :class:`pid_tuner.controller.PIDController`.
It intentionally stays independent from Streamlit so the exact same simulation
engine is used by the UI, CLI, examples, and unit tests.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import exp, isfinite
from random import Random
from typing import Literal

from .controller import PIDConfig, PIDController
from .plants import BasePlant

DisturbanceKind = Literal["none", "impulse", "pulse", "step"]


@dataclass(frozen=True, slots=True)
class DisturbanceConfig:
    """Describe an optional plant disturbance.

    ``impulse`` applies the magnitude for one simulation sample, ``pulse`` for
    ``duration`` seconds, and ``step`` from ``start_time`` until the end.
    Disturbance units are plant-specific and are documented by each plant.
    """

    kind: DisturbanceKind = "none"
    start_time: float = 0.0
    magnitude: float = 0.0
    duration: float = 1.0

    def __post_init__(self) -> None:
        if self.kind not in {"none", "impulse", "pulse", "step"}:
            raise ValueError(f"Unsupported disturbance kind: {self.kind!r}")
        if self.start_time < 0.0:
            raise ValueError("disturbance start_time cannot be negative")
        if self.duration < 0.0:
            raise ValueError("disturbance duration cannot be negative")
        if not isfinite(self.magnitude):
            raise ValueError("disturbance magnitude must be finite")

    def value_at(self, time: float, dt: float) -> float:
        """Return the disturbance value active at a given simulation time."""
        if self.kind == "none" or time < self.start_time:
            return 0.0
        if self.kind == "step":
            return self.magnitude
        if self.kind == "impulse":
            return self.magnitude if time < self.start_time + dt else 0.0
        # pulse
        return self.magnitude if time < self.start_time + self.duration else 0.0


@dataclass(frozen=True, slots=True)
class SimulationConfig:
    """Numerical and measurement settings for a closed-loop run."""

    setpoint: float
    duration: float = 10.0
    dt: float = 0.02
    measurement_noise_std: float = 0.0
    measurement_filter_tau: float = 0.0
    random_seed: int = 7
    disturbance: DisturbanceConfig = DisturbanceConfig()

    def __post_init__(self) -> None:
        if self.duration <= 0.0 or not isfinite(self.duration):
            raise ValueError("duration must be a positive finite number")
        if self.dt <= 0.0 or not isfinite(self.dt):
            raise ValueError("dt must be a positive finite number")
        if self.dt > self.duration:
            raise ValueError("dt cannot be larger than duration")
        if self.measurement_noise_std < 0.0:
            raise ValueError("measurement_noise_std cannot be negative")
        if self.measurement_filter_tau < 0.0:
            raise ValueError("measurement_filter_tau cannot be negative")
        if not isfinite(self.setpoint):
            raise ValueError("setpoint must be finite")


def _filtered_measurement(
    previous: float | None,
    raw: float,
    dt: float,
    tau: float,
) -> float:
    """First-order low-pass filter used only for optional sensor filtering."""
    if previous is None or tau <= 0.0:
        return raw

    # Exponential update is numerically well behaved even if dt and tau differ
    # substantially.  For small dt it approaches the familiar Euler low-pass.
    alpha = 1.0 - exp(-dt / tau)
    return previous + alpha * (raw - previous)


def run_closed_loop(
    plant: BasePlant,
    pid_config: PIDConfig,
    sim_config: SimulationConfig,
) -> dict[str, list[float] | list[bool]]:
    """Simulate one PID-controlled plant.

    The recorded ``output`` is the true plant output.  ``measurement`` is the
    value seen by the controller after optional noise and sensor filtering.
    This separation makes noise experiments explicit instead of accidentally
    treating noisy data as the physical state.
    """
    controller = PIDController(pid_config)
    rng = Random(sim_config.random_seed)
    n_steps = int(sim_config.duration / sim_config.dt) + 1

    result: dict[str, list] = {
        "t": [],
        "setpoint": [],
        "output": [],
        "measurement": [],
        "control": [],
        "p": [],
        "i": [],
        "d": [],
        "error": [],
        "saturated": [],
        "disturbance": [],
    }

    filtered_sensor: float | None = None

    for step_idx in range(n_steps):
        time = min(step_idx * sim_config.dt, sim_config.duration)
        true_output = plant.output()

        noise = (
            rng.gauss(0.0, sim_config.measurement_noise_std)
            if sim_config.measurement_noise_std > 0.0
            else 0.0
        )
        noisy_measurement = true_output + noise
        filtered_sensor = _filtered_measurement(
            filtered_sensor,
            noisy_measurement,
            sim_config.dt,
            sim_config.measurement_filter_tau,
        )

        pid_step = controller.step(
            setpoint=sim_config.setpoint,
            measurement=filtered_sensor,
            dt=sim_config.dt,
        )
        disturbance = sim_config.disturbance.value_at(time, sim_config.dt)

        result["t"].append(time)
        result["setpoint"].append(sim_config.setpoint)
        result["output"].append(true_output)
        result["measurement"].append(filtered_sensor)
        result["control"].append(pid_step.output)
        result["p"].append(pid_step.p)
        result["i"].append(pid_step.i)
        result["d"].append(pid_step.d)
        result["error"].append(pid_step.error)
        result["saturated"].append(pid_step.saturated)
        result["disturbance"].append(disturbance)

        # The final sample represents the state at t=duration, so no additional
        # integration is performed after recording it.
        if step_idx < n_steps - 1:
            plant.step(pid_step.output, sim_config.dt, disturbance=disturbance)

    return result


def run_simulation(
    plant: BasePlant,
    kp: float,
    ki: float,
    kd: float,
    setpoint: float,
    duration: float = 10.0,
    dt: float = 0.02,
    disturbance_time: float | None = None,
    disturbance_magnitude: float = 0.0,
    disturbance_kind: DisturbanceKind = "impulse",
    disturbance_duration: float = 1.0,
    output_limits: tuple[float, float] | None = None,
    derivative_filter_tau: float = 0.02,
    anti_windup: bool = True,
    measurement_noise_std: float = 0.0,
    measurement_filter_tau: float = 0.0,
    random_seed: int = 7,
) -> dict[str, list[float] | list[bool]]:
    """Convenience wrapper around :func:`run_closed_loop`.

    The signature remains close to the original project so existing examples
    are easy to migrate.  When ``output_limits`` is omitted, each plant's
    default physical actuator limits are used.
    """
    limits = output_limits if output_limits is not None else plant.control_limits
    disturbance = DisturbanceConfig(
        kind="none" if disturbance_time is None else disturbance_kind,
        start_time=0.0 if disturbance_time is None else disturbance_time,
        magnitude=disturbance_magnitude,
        duration=disturbance_duration,
    )
    pid = PIDConfig(
        kp=kp,
        ki=ki,
        kd=kd,
        output_min=limits[0],
        output_max=limits[1],
        derivative_filter_tau=derivative_filter_tau,
        anti_windup=anti_windup,
    )
    sim = SimulationConfig(
        setpoint=setpoint,
        duration=duration,
        dt=dt,
        measurement_noise_std=measurement_noise_std,
        measurement_filter_tau=measurement_filter_tau,
        random_seed=random_seed,
        disturbance=disturbance,
    )
    return run_closed_loop(plant, pid, sim)
