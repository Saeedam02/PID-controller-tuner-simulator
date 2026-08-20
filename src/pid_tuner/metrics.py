"""Performance metrics for closed-loop control simulations.

The original repository computed the standard rise-time/overshoot/settling
metrics.  This version keeps those metrics and makes their definitions more
robust for:

* non-zero initial outputs;
* negative set-point changes;
* stabilization problems with a zero set-point (the inverted pendulum); and
* optional actuator-effort diagnostics.

Only the Python standard library is used here, which keeps the metric code easy
to audit and unit test.
"""

from __future__ import annotations

from math import isfinite, sqrt
from statistics import fmean
from typing import Sequence


def _validate_series(
    t: Sequence[float], setpoint: Sequence[float], output: Sequence[float]
) -> None:
    if not t or not setpoint or not output:
        raise ValueError("t, setpoint, and output must be non-empty")
    if not (len(t) == len(setpoint) == len(output)):
        raise ValueError("t, setpoint, and output must have the same length")
    if any(not isfinite(value) for value in [*t, *setpoint, *output]):
        raise ValueError("metric inputs must contain only finite values")
    if any(t2 <= t1 for t1, t2 in zip(t, t[1:])):
        raise ValueError("time samples must be strictly increasing")


def _trapz(t: Sequence[float], values: Sequence[float]) -> float:
    """Trapezoidal integral without NumPy/SciPy."""
    return sum(
        0.5 * (v0 + v1) * (t1 - t0)
        for t0, t1, v0, v1 in zip(t[:-1], t[1:], values[:-1], values[1:], strict=True)
    )


def compute_metrics(
    t: Sequence[float],
    setpoint: Sequence[float],
    output: Sequence[float],
    *,
    control: Sequence[float] | None = None,
    saturated: Sequence[bool] | None = None,
    settle_band: float = 0.02,
    tail_fraction: float = 0.05,
) -> dict[str, float | None]:
    """Compute standard tracking and control-effort metrics.

    Definitions
    -----------
    Rise time:
        Time from 10% to 90% of the commanded transition from the *initial
        output* to the final set-point.  For stabilization tasks where the
        response starts away from a zero target and moves toward zero, a
        conventional 10-90% rise time is not meaningful, so ``None`` is
        returned.
    Overshoot:
        Peak excursion beyond the final target, normalized by the commanded
        transition magnitude.  Works for both positive and negative steps.
    Settling time:
        First time after which the output never leaves the ±``settle_band``
        region.  The band is scaled by the larger of the target magnitude and
        initial tracking error so zero-target stabilization remains meaningful.
    IAE / ISE / ITAE:
        Standard integral error measures evaluated using trapezoidal
        integration.
    """
    _validate_series(t, setpoint, output)
    if not 0.0 < settle_band < 1.0:
        raise ValueError("settle_band must lie between 0 and 1")
    if not 0.0 < tail_fraction <= 1.0:
        raise ValueError("tail_fraction must lie in (0, 1]")

    target = float(setpoint[-1])
    y0 = float(output[0])
    transition = target - y0
    transition_mag = abs(transition)

    # -------- Rise time --------
    # This metric is defined for a target transition with a clear direction.
    # A pendulum stabilization from nonzero angle to a zero setpoint is better
    # described by settling time and integral error metrics.
    rise_time: float | None = None
    if transition_mag > 1.0e-12 and abs(target) > 1.0e-12:
        y10 = y0 + 0.1 * transition
        y90 = y0 + 0.9 * transition
        positive_transition = transition > 0.0

        def reached(value: float, threshold: float) -> bool:
            return value >= threshold if positive_transition else value <= threshold

        t10 = next(
            (ti for ti, yi in zip(t, output, strict=True) if reached(yi, y10)), None
        )
        t90 = next(
            (ti for ti, yi in zip(t, output, strict=True) if reached(yi, y90)), None
        )
        if t10 is not None and t90 is not None and t90 >= t10:
            rise_time = t90 - t10

    # -------- Percent overshoot --------
    if transition_mag <= 1.0e-12:
        overshoot_pct = 0.0
    elif transition > 0.0:
        overshoot = max(0.0, max(output) - target)
        overshoot_pct = 100.0 * overshoot / transition_mag
    else:
        overshoot = max(0.0, target - min(output))
        overshoot_pct = 100.0 * overshoot / transition_mag

    # -------- Settling time --------
    scale = max(abs(target), abs(target - y0), 1.0e-9)
    band = settle_band * scale
    last_outside_index: int | None = None
    for idx, yi in enumerate(output):
        if abs(yi - target) > band:
            last_outside_index = idx

    if last_outside_index is None:
        settling_time = 0.0
    elif last_outside_index >= len(t) - 1:
        # The response never settled before the simulation ended.
        settling_time = None
    else:
        settling_time = float(t[last_outside_index + 1])

    # -------- Steady-state error --------
    tail_n = max(1, round(len(output) * tail_fraction))
    steady_output = fmean(output[-tail_n:])
    steady_state_error = target - steady_output

    errors = [sp - y for sp, y in zip(setpoint, output, strict=True)]
    abs_errors = [abs(error) for error in errors]
    squared_errors = [error * error for error in errors]
    time_weighted_abs_error = [ti * ae for ti, ae in zip(t, abs_errors, strict=True)]

    iae = _trapz(t, abs_errors)
    ise = _trapz(t, squared_errors)
    itae = _trapz(t, time_weighted_abs_error)
    rms_error = sqrt(fmean(squared_errors))
    peak_abs_error = max(abs_errors)

    metrics: dict[str, float | None] = {
        "rise_time_s": rise_time,
        "overshoot_pct": overshoot_pct,
        "settling_time_s": settling_time,
        "steady_state_error": steady_state_error,
        "IAE": iae,
        "ISE": ise,
        "ITAE": itae,
        "RMS_error": rms_error,
        "peak_abs_error": peak_abs_error,
    }

    if control is not None:
        if len(control) != len(t):
            raise ValueError("control must have the same length as t")
        if any(not isfinite(value) for value in control):
            raise ValueError("control must contain only finite values")
        metrics["control_RMS"] = sqrt(fmean(u * u for u in control))
        metrics["control_total_variation"] = sum(
            abs(u1 - u0) for u0, u1 in zip(control[:-1], control[1:], strict=True)
        )

    if saturated is not None:
        if len(saturated) != len(t):
            raise ValueError("saturated must have the same length as t")
        metrics["saturation_fraction"] = sum(bool(x) for x in saturated) / len(saturated)

    return metrics
