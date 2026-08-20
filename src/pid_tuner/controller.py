"""
controller.py

A discrete PID controller with:
  - Integral anti-windup (clamping)
  - Derivative filtering (to avoid noise amplification)
  - Optional output saturation

This is written to be plant-agnostic: it only needs an error signal at
each timestep and returns a control effort.
"""

from dataclasses import dataclass


@dataclass
class PIDConfig:
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    output_min: float = -1e9
    output_max: float = 1e9
    derivative_filter_tau: float = 0.02  # seconds; low-pass filter time constant for D term
    anti_windup: bool = True


class PIDController:
    def __init__(self, config: PIDConfig):
        self.cfg = config
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._filtered_derivative = 0.0
        self._prev_measurement = None

    def step(self, setpoint: float, measurement: float, dt: float) -> dict:
        """
        Compute one PID update.

        Returns a dict with the control output plus the individual P/I/D
        contributions (useful for plotting and for understanding what each
        term is doing).
        """
        error = setpoint - measurement

        # --- Proportional ---
        p_term = self.cfg.kp * error

        # --- Integral (with conditional anti-windup) ---
        tentative_integral = self._integral + error * dt
        i_term = self.cfg.ki * tentative_integral

        # --- Derivative on measurement (avoids "derivative kick" on setpoint changes) ---
        if self._prev_measurement is None:
            raw_derivative = 0.0
        else:
            raw_derivative = -(measurement - self._prev_measurement) / dt

        # Low-pass filter the derivative term
        alpha = dt / (self.cfg.derivative_filter_tau + dt) if self.cfg.derivative_filter_tau > 0 else 1.0
        self._filtered_derivative = (
            self._filtered_derivative + alpha * (raw_derivative - self._filtered_derivative)
        )
        d_term = self.cfg.kd * self._filtered_derivative

        # --- Combine and saturate ---
        unsaturated_output = p_term + i_term + d_term
        output = min(max(unsaturated_output, self.cfg.output_min), self.cfg.output_max)

        # --- Anti-windup: only integrate if we're not saturated, or if the
        # integration would move us back toward the unsaturated region ---
        saturated = output != unsaturated_output
        if not (self.cfg.anti_windup and saturated):
            self._integral = tentative_integral
            i_term = self.cfg.ki * self._integral

        self._prev_error = error
        self._prev_measurement = measurement

        return {
            "output": output,
            "error": error,
            "p": p_term,
            "i": i_term,
            "d": d_term,
            "saturated": saturated,
        }
