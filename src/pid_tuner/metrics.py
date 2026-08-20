"""
metrics.py

Standard step-response performance metrics used to evaluate a PID tune:
  - Rise time (10% -> 90% of final value)
  - Percent overshoot
  - Settling time (within +/- 2% band)
  - Steady-state error
  - ISE (Integral of Squared Error) and ITAE (Integral of Time-weighted Absolute Error)
"""

def compute_metrics(t, setpoint, output, settle_band=0.02):
    if len(t) == 0:
        return {}

    final_setpoint = setpoint[-1]
    final_output = output[-1]

    # Avoid division by zero if setpoint is 0
    ref = final_setpoint if abs(final_setpoint) > 1e-9 else 1.0

    # --- Rise time (10% to 90%) ---
    lo_thresh = 0.1 * final_setpoint
    hi_thresh = 0.9 * final_setpoint
    t_lo, t_hi = None, None
    for ti, yi in zip(t, output):
        if t_lo is None and (yi >= lo_thresh if final_setpoint >= 0 else yi <= lo_thresh):
            t_lo = ti
        if t_hi is None and (yi >= hi_thresh if final_setpoint >= 0 else yi <= hi_thresh):
            t_hi = ti
        if t_lo is not None and t_hi is not None:
            break
    rise_time = (t_hi - t_lo) if (t_lo is not None and t_hi is not None) else None

    # --- Overshoot ---
    if final_setpoint >= 0:
        peak = max(output)
    else:
        peak = min(output)
    overshoot_pct = max(0.0, (peak - final_setpoint) / ref * 100.0) if final_setpoint >= 0 else \
        max(0.0, (final_setpoint - peak) / ref * 100.0)

    # --- Settling time (last time output leaves the +/- settle_band around final value) ---
    band = settle_band * abs(ref)
    settling_time = 0.0
    for ti, yi in zip(t, output):
        if abs(yi - final_setpoint) > band:
            settling_time = ti

    # --- Steady-state error (using last 5% of the simulation) ---
    tail_n = max(1, len(output) // 20)
    steady_state_error = final_setpoint - (sum(output[-tail_n:]) / tail_n)

    # --- ISE and ITAE ---
    dt = t[1] - t[0] if len(t) > 1 else 0.0
    ise = sum((sp - y) ** 2 for sp, y in zip(setpoint, output)) * dt
    itae = sum(ti * abs(sp - y) for ti, sp, y in zip(t, setpoint, output)) * dt

    return {
        "rise_time_s": rise_time,
        "overshoot_pct": overshoot_pct,
        "settling_time_s": settling_time,
        "steady_state_error": steady_state_error,
        "ISE": ise,
        "ITAE": itae,
    }
