# Architecture

The project is organized so that presentation code never owns control logic.

```text
Streamlit UI / CLI
       |
       v
simulation.py  --->  controller.py
       |
       +----------->  plants.py
       |
       +----------->  metrics.py
       |
       +----------->  export.py

tuning.py ---> plants.py (fresh open-loop identification runs)
```

## Module responsibilities

- `controller.py`: PID state, derivative filtering, saturation, anti-windup.
- `plants.py`: dynamic models and RK4 integration.
- `simulation.py`: closed-loop timing, sensor noise/filtering, disturbances.
- `metrics.py`: response-quality and control-effort measurements.
- `tuning.py`: open-loop response generation, FOPDT fitting, tuning rules.
- `export.py`: deterministic CSV/JSON serialization.
- `cli.py`: reproducible non-interactive runs.
- `app.py`: visualization and user interaction only.

This separation is intentional: a unit test can exercise the control equations
without importing Streamlit, and a future notebook or hardware-in-the-loop
client can reuse the same core package.
