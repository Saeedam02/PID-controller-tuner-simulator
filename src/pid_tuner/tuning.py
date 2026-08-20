"""
tuning.py

A simple Ziegler-Nichols "open-loop reaction curve" auto-tuner.

Method:
  1. Apply an open-loop step input to the plant (no controller).
  2. Fit an approximate first-order-plus-dead-time (FOPDT) model from the
     response: process gain K, time constant tau, and dead time L.
  3. Use the classic Ziegler-Nichols step-response tuning rules to suggest
     Kp, Ki, Kd.

This won't be perfect for every plant (especially the unstable pendulum,
which has no open-loop step response worth fitting), but it gives a solid,
explainable starting point for the DC motor and thermal plants, and it's
the same method taught in every controls course -- good for demonstrating
you understand classical tuning, not just sliders.
"""

from plants import _rk4_step


def open_loop_step_response(plant_factory, step_size=1.0, duration=30.0, dt=0.02):
    plant = plant_factory()
    t_list, y_list = [], []
    n_steps = int(duration / dt)
    for i in range(n_steps):
        t = i * dt
        y = plant.step(step_size, dt)
        t_list.append(t)
        y_list.append(y)
    return t_list, y_list


def fit_fopdt(t, y, step_size):
    """Rough FOPDT fit using the 28.3%/63.2% two-point method."""
    y0 = y[0]
    y_final = y[-1]
    delta = y_final - y0
    if abs(delta) < 1e-9:
        return None  # no response to a step; can't fit (e.g. unstable/integrating plant)

    K = delta / step_size

    target_28 = y0 + 0.283 * delta
    target_63 = y0 + 0.632 * delta

    t_28 = next((ti for ti, yi in zip(t, y) if (yi >= target_28 if delta > 0 else yi <= target_28)), None)
    t_63 = next((ti for ti, yi in zip(t, y) if (yi >= target_63 if delta > 0 else yi <= target_63)), None)

    if t_28 is None or t_63 is None:
        return None

    tau = 1.5 * (t_63 - t_28)
    L = t_63 - tau
    L = max(L, 0.01)  # avoid zero/negative dead time

    return {"K": K, "tau": tau, "L": L}


def ziegler_nichols_pid(K, tau, L):
    """Classic ZN step-response PID tuning rules."""
    if L <= 0:
        L = 0.01
    kp = 1.2 * tau / (K * L)
    ti = 2 * L
    td = 0.5 * L
    ki = kp / ti
    kd = kp * td
    return {"kp": kp, "ki": ki, "kd": kd, "fitted_K": K, "fitted_tau": tau, "fitted_L": L}


def suggest_pid(plant_factory, step_size=1.0):
    """
    Runs an open-loop step test on a fresh plant instance and returns
    suggested PID gains, or None if the plant isn't suited to this method
    (e.g. an unstable open-loop plant like the pendulum).
    """
    t, y = open_loop_step_response(plant_factory, step_size=step_size)
    fit = fit_fopdt(t, y, step_size)
    if fit is None:
        return None
    return ziegler_nichols_pid(fit["K"], fit["tau"], fit["L"])
