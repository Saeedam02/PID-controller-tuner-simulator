"""PID controller implementation used by the simulator.

The controller is deliberately small enough to inspect, but it includes the
implementation details that matter in real digital control code:

* proportional, integral, and derivative actions;
* derivative-on-measurement to avoid set-point derivative kick;
* first-order low-pass filtering of the measured derivative;
* actuator saturation; and
* conditional-integration anti-windup.

The module contains no Streamlit code and no plant-specific logic, so it can be
unit-tested independently and reused from the CLI, notebooks, or other UIs.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True, slots=True)
class PIDConfig:
    """Configuration for :class:`PIDController`.

    Parameters
    ----------
    kp, ki, kd:
        Parallel-form PID gains.  The implemented control law is approximately
        ``u = kp*e + ki*integral(e) - kd*d(measurement)/dt``.
    output_min, output_max:
        Hard actuator limits applied after the P/I/D terms are summed.
    derivative_filter_tau:
        Time constant, in seconds, of the first-order low-pass filter used on
        the derivative estimate.  Set to zero to disable derivative filtering.
    anti_windup:
        If ``True``, conditional integration prevents the integral state from
        growing when the actuator is saturated *and* the proposed integral
        update would drive it farther into saturation.  Integration is still
        allowed when it helps the controller recover from saturation.
    """

    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    output_min: float = -1.0e9
    output_max: float = 1.0e9
    derivative_filter_tau: float = 0.02
    anti_windup: bool = True

    def __post_init__(self) -> None:
        """Reject invalid settings early instead of failing during simulation."""
        numeric_values = {
            "kp": self.kp,
            "ki": self.ki,
            "kd": self.kd,
            "output_min": self.output_min,
            "output_max": self.output_max,
            "derivative_filter_tau": self.derivative_filter_tau,
        }
        for name, value in numeric_values.items():
            if not isfinite(value):
                raise ValueError(f"{name} must be finite; got {value!r}")

        if self.output_min >= self.output_max:
            raise ValueError("output_min must be strictly smaller than output_max")
        if self.derivative_filter_tau < 0.0:
            raise ValueError("derivative_filter_tau cannot be negative")


@dataclass(frozen=True, slots=True)
class PIDStep:
    """Result returned by one PID update.

    Keeping the individual terms makes the controller easy to debug and lets
    the UI plot how much each term contributes to the final command.
    """

    output: float
    error: float
    p: float
    i: float
    d: float
    unsaturated_output: float
    saturated: bool
    integral_state: float
    derivative_state: float


class PIDController:
    """Discrete parallel-form PID controller.

    Notes
    -----
    The derivative is taken on the measurement rather than the error.  For a
    constant measurement and a sudden set-point step, this makes the derivative
    term remain near zero instead of producing the large "derivative kick"
    associated with differentiating the error directly.
    """

    def __init__(self, config: PIDConfig):
        self.cfg = config
        self.reset()

    def reset(self) -> None:
        """Reset all controller memory while keeping the configured gains."""
        self._integral = 0.0
        self._filtered_derivative = 0.0
        self._prev_measurement: float | None = None

    @property
    def integral_state(self) -> float:
        """Current integral accumulator, exposed read-only for diagnostics."""
        return self._integral

    @property
    def derivative_state(self) -> float:
        """Current filtered derivative estimate, exposed for diagnostics."""
        return self._filtered_derivative

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)

    def step(self, setpoint: float, measurement: float, dt: float) -> PIDStep:
        """Advance the controller by one sample.

        Parameters
        ----------
        setpoint:
            Desired value of the controlled output.
        measurement:
            Measured output supplied to the controller.  It may be noisy or
            filtered by the simulation layer.
        dt:
            Sample period in seconds.  Must be positive.

        Returns
        -------
        PIDStep
            Applied command and the P/I/D diagnostic terms.
        """
        if not isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be a positive finite number")
        if not isfinite(setpoint) or not isfinite(measurement):
            raise ValueError("setpoint and measurement must be finite")

        error = setpoint - measurement

        # Proportional term: reacts immediately to the current tracking error.
        p_term = self.cfg.kp * error

        # Derivative on measurement: avoids derivative kick when only the
        # set-point changes.  The leading minus sign is required because
        # e = r - y, hence d(e)/dt = -d(y)/dt for constant r.
        if self._prev_measurement is None:
            raw_derivative = 0.0
        else:
            raw_derivative = -(measurement - self._prev_measurement) / dt

        # Exact enough first-order discrete low-pass update for this teaching
        # simulator.  alpha=1 removes filtering when tau is zero.
        if self.cfg.derivative_filter_tau == 0.0:
            alpha = 1.0
        else:
            alpha = dt / (self.cfg.derivative_filter_tau + dt)
        self._filtered_derivative += alpha * (
            raw_derivative - self._filtered_derivative
        )
        d_term = self.cfg.kd * self._filtered_derivative

        # Propose an integral update.  We decide below whether that proposed
        # update is allowed by the conditional anti-windup rule.
        tentative_integral = self._integral + error * dt
        tentative_i = self.cfg.ki * tentative_integral
        tentative_unsat = p_term + tentative_i + d_term
        tentative_output = self._clamp(
            tentative_unsat, self.cfg.output_min, self.cfg.output_max
        )

        if self.cfg.anti_windup and tentative_output != tentative_unsat:
            # Integral contribution added by this sample.  If this increment
            # pushes in the same direction as the active saturation limit,
            # freeze the integral.  If it pushes back toward the linear region,
            # keep integrating so the controller can unwind naturally.
            delta_i = self.cfg.ki * error * dt
            saturating_high = tentative_unsat > self.cfg.output_max
            saturating_low = tentative_unsat < self.cfg.output_min
            pushes_farther = (saturating_high and delta_i > 0.0) or (
                saturating_low and delta_i < 0.0
            )
            if not pushes_farther:
                self._integral = tentative_integral
        else:
            self._integral = tentative_integral

        # Recompute the final output using the accepted integral state.  This is
        # important when the candidate update was rejected by anti-windup.
        i_term = self.cfg.ki * self._integral
        unsaturated_output = p_term + i_term + d_term
        output = self._clamp(
            unsaturated_output, self.cfg.output_min, self.cfg.output_max
        )
        saturated = output != unsaturated_output

        self._prev_measurement = measurement

        return PIDStep(
            output=output,
            error=error,
            p=p_term,
            i=i_term,
            d=d_term,
            unsaturated_output=unsaturated_output,
            saturated=saturated,
            integral_state=self._integral,
            derivative_state=self._filtered_derivative,
        )
