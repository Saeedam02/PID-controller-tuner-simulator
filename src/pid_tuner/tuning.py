"""Open-loop model identification and explainable PID/PI tuning rules.

The original repository used a Ziegler-Nichols reaction-curve tuner for every
plant whose numerical FOPDT fit happened to succeed.  That can be misleading:
open-loop unstable plants are fundamentally incompatible with that procedure,
and Ziegler-Nichols becomes ill-conditioned when the fitted dead time is nearly
zero.

This module therefore separates three concerns:

1. generate an open-loop step response;
2. identify an approximate first-order-plus-dead-time (FOPDT) model; and
3. apply a tuning rule only when its assumptions are reasonable.

Two transparent rules are provided:

* ``lambda_pi`` (default) -- a conservative model-based PI starting point that
  remains well defined for small dead time;
* ``ziegler_nichols`` -- the classic reaction-curve PID rule, enabled only when
  a meaningful positive dead time is identified.

These are starting points, not guarantees of optimal closed-loop performance.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from statistics import fmean
from typing import Literal, Sequence

from .plants import BasePlant

TuningMethod = Literal["lambda_pi", "ziegler_nichols"]


@dataclass(frozen=True, slots=True)
class FOPDTModel:
    """Approximate model ``K * exp(-L s) / (tau s + 1)``."""

    gain: float
    tau: float
    delay: float
    initial_output: float
    final_output: float


@dataclass(frozen=True, slots=True)
class TuningSuggestion:
    """Controller gains plus the identified model that produced them."""

    method: TuningMethod
    kp: float
    ki: float
    kd: float
    model: FOPDTModel
    note: str

    def as_dict(self) -> dict[str, float | str]:
        """Return a flat dictionary convenient for UI tables and JSON export."""
        data: dict[str, float | str] = {
            "method": self.method,
            "kp": self.kp,
            "ki": self.ki,
            "kd": self.kd,
            "note": self.note,
            "fitted_K": self.model.gain,
            "fitted_tau": self.model.tau,
            "fitted_L": self.model.delay,
        }
        return data


@dataclass(frozen=True, slots=True)
class TuningAttempt:
    """Outcome of an auto-tuning request, including a useful failure reason."""

    suggestion: TuningSuggestion | None
    reason: str | None = None

    @property
    def available(self) -> bool:
        return self.suggestion is not None


def open_loop_step_response(
    plant_factory: type[BasePlant],
    *,
    step_size: float | None = None,
    duration: float | None = None,
    dt: float = 0.02,
) -> tuple[list[float], list[float]]:
    """Simulate a constant open-loop input on a fresh plant instance."""
    if dt <= 0.0 or not isfinite(dt):
        raise ValueError("dt must be a positive finite number")

    plant = plant_factory()
    u_step = plant_factory.open_loop_step_size if step_size is None else step_size
    horizon = (
        plant_factory.open_loop_tuning_duration if duration is None else duration
    )
    if u_step == 0.0:
        raise ValueError("step_size must be non-zero")
    if horizon <= dt:
        raise ValueError("duration must be larger than dt")

    times = [0.0]
    outputs = [plant.output()]
    n_steps = int(horizon / dt)
    for index in range(1, n_steps + 1):
        plant.step(u_step, dt)
        times.append(index * dt)
        outputs.append(plant.output())
        if not isfinite(outputs[-1]):
            break

    return times, outputs


def _crossing_time(
    t: Sequence[float], y: Sequence[float], target: float, increasing: bool
) -> float | None:
    """Return a linearly interpolated threshold crossing time."""
    for t0, t1, y0, y1 in zip(t[:-1], t[1:], y[:-1], y[1:], strict=True):
        crossed = (y0 <= target <= y1) if increasing else (y0 >= target >= y1)
        if not crossed:
            continue
        if y1 == y0:
            return float(t1)
        fraction = (target - y0) / (y1 - y0)
        return float(t0 + fraction * (t1 - t0))
    return None


def fit_fopdt(
    t: Sequence[float],
    y: Sequence[float],
    step_size: float,
    *,
    tail_fraction: float = 0.1,
) -> FOPDTModel | None:
    """Fit a rough FOPDT model with the 28.3%/63.2% two-point method.

    The method is intended for a stable, mostly monotonic step response.  A
    finite tail average is used instead of the single last sample to make the
    estimate less sensitive to numerical or measurement noise.
    """
    if len(t) != len(y) or len(t) < 3:
        raise ValueError("t and y must have equal length and at least three samples")
    if step_size == 0.0 or not isfinite(step_size):
        raise ValueError("step_size must be a non-zero finite number")
    if not 0.0 < tail_fraction <= 0.5:
        raise ValueError("tail_fraction must lie in (0, 0.5]")
    if any(not isfinite(value) for value in [*t, *y]):
        return None

    tail_n = max(3, round(len(y) * tail_fraction))
    y0 = float(y[0])
    y_final = float(fmean(y[-tail_n:]))
    delta = y_final - y0
    if abs(delta) < 1.0e-9:
        return None

    # Reject responses whose tail is still changing substantially.  The
    # threshold is intentionally loose: this is a teaching identifier, not a
    # production system-identification package.
    tail_span = max(y[-tail_n:]) - min(y[-tail_n:])
    if tail_span > 0.05 * abs(delta):
        return None

    increasing = delta > 0.0
    target_28 = y0 + 0.283 * delta
    target_63 = y0 + 0.632 * delta
    t28 = _crossing_time(t, y, target_28, increasing)
    t63 = _crossing_time(t, y, target_63, increasing)
    if t28 is None or t63 is None or t63 <= t28:
        return None

    tau = 1.5 * (t63 - t28)
    delay = max(0.0, t63 - tau)
    gain = delta / step_size
    if tau <= 0.0 or abs(gain) < 1.0e-12:
        return None

    return FOPDTModel(
        gain=gain,
        tau=tau,
        delay=delay,
        initial_output=y0,
        final_output=y_final,
    )


def lambda_pi(
    model: FOPDTModel,
    *,
    closed_loop_time_constant: float | None = None,
) -> tuple[float, float, float]:
    """Return a conservative lambda-tuned PI controller.

    For the identified FOPDT model, the parallel-form PI gains are

    ``Kp = tau / (K * (lambda + L))`` and ``Ki = Kp / tau``.

    ``lambda`` controls the desired closed-loop speed.  The default is chosen
    conservatively as ``max(0.5*tau, 2*L)``.  ``Kd`` is zero because this rule is
    explicitly a PI rule; the UI labels it accordingly instead of pretending a
    derivative gain was identified.
    """
    lam = (
        max(0.5 * model.tau, 2.0 * model.delay, 1.0e-6)
        if closed_loop_time_constant is None
        else closed_loop_time_constant
    )
    if lam <= 0.0:
        raise ValueError("closed_loop_time_constant must be positive")

    kp = model.tau / (model.gain * (lam + model.delay))
    ki = kp / model.tau
    return kp, ki, 0.0


def ziegler_nichols_pid(model: FOPDTModel) -> tuple[float, float, float]:
    """Classic Ziegler-Nichols reaction-curve PID rule.

    The formula contains ``1/L`` and therefore becomes meaningless as the
    fitted dead time approaches zero.  This implementation refuses to hide that
    issue behind an arbitrary epsilon.
    """
    minimum_meaningful_delay = max(1.0e-6, 0.01 * model.tau)
    if model.delay < minimum_meaningful_delay:
        raise ValueError(
            "Ziegler-Nichols reaction-curve PID is ill-conditioned because the "
            "identified dead time is too small relative to the process time constant"
        )

    kp = 1.2 * model.tau / (model.gain * model.delay)
    integral_time = 2.0 * model.delay
    derivative_time = 0.5 * model.delay
    ki = kp / integral_time
    kd = kp * derivative_time
    return kp, ki, kd


def tune_plant(
    plant_factory: type[BasePlant],
    *,
    method: TuningMethod = "lambda_pi",
    step_size: float | None = None,
    duration: float | None = None,
    dt: float = 0.02,
) -> TuningAttempt:
    """Identify a plant and return an explainable starting controller."""
    if not plant_factory.supports_open_loop_tuning:
        return TuningAttempt(
            suggestion=None,
            reason=(
                f"{plant_factory.name} is declared open-loop unstable, so a stable "
                "reaction-curve/FOPDT step-test tune is intentionally disabled."
            ),
        )

    u_step = plant_factory.open_loop_step_size if step_size is None else step_size
    t, y = open_loop_step_response(
        plant_factory, step_size=u_step, duration=duration, dt=dt
    )
    model = fit_fopdt(t, y, u_step)
    if model is None:
        return TuningAttempt(
            suggestion=None,
            reason=(
                "The open-loop response did not settle into a usable FOPDT shape. "
                "Try a longer identification horizon or tune manually."
            ),
        )

    try:
        if method == "lambda_pi":
            kp, ki, kd = lambda_pi(model)
            note = (
                "Conservative lambda-tuned PI starting point. Fine-tune in closed loop "
                "and verify actuator limits and disturbance rejection."
            )
        elif method == "ziegler_nichols":
            kp, ki, kd = ziegler_nichols_pid(model)
            note = (
                "Classic Ziegler-Nichols reaction-curve PID. Expect aggressive tuning; "
                "verify overshoot, saturation, and robustness before using it elsewhere."
            )
        else:
            raise ValueError(f"Unknown tuning method: {method!r}")
    except ValueError as exc:
        return TuningAttempt(suggestion=None, reason=str(exc))

    return TuningAttempt(
        suggestion=TuningSuggestion(
            method=method,
            kp=kp,
            ki=ki,
            kd=kd,
            model=model,
            note=note,
        )
    )


def suggest_pid(
    plant_factory: type[BasePlant],
    step_size: float | None = None,
) -> dict[str, float | str] | None:
    """Backward-compatible helper returning the default lambda-PI suggestion.

    New code should prefer :func:`tune_plant` because it preserves the reason an
    attempted tune is unavailable.
    """
    attempt = tune_plant(plant_factory, method="lambda_pi", step_size=step_size)
    return attempt.suggestion.as_dict() if attempt.suggestion else None
