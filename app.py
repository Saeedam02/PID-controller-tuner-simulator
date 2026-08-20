"""Streamlit front end for the PID Controller Tuner & Simulator.

Run from a source checkout with:

    streamlit run app.py

The core control/simulation code lives under ``src/pid_tuner``.  The small path
bootstrap below lets Streamlit run directly from an uninstalled checkout while
still keeping the repository in a conventional ``src/`` package layout.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pandas as pd
import streamlit as st

REPO_ROOT = Path(__file__).resolve().parent
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from pid_tuner.export import configuration_to_json, results_to_csv
from pid_tuner.metrics import compute_metrics
from pid_tuner.plants import PLANT_REGISTRY
from pid_tuner.simulation import run_simulation
from pid_tuner.tuning import tune_plant


st.set_page_config(
    page_title="PID Controller Tuner & Simulator",
    page_icon="🎛️",
    layout="wide",
)


def _fmt_metric(value: float | None, unit: str = "", digits: int = 2) -> str:
    """Render missing metrics as N/A instead of formatting ``None``."""
    if value is None:
        return "N/A"
    suffix = f" {unit}" if unit else ""
    return f"{value:.{digits}f}{suffix}"


def _reset_gain_state(plant_key: str) -> None:
    """Load the selected plant's checked-in starting gains into widget state."""
    defaults = PLANT_REGISTRY[plant_key].default_pid
    st.session_state.kp = float(defaults["kp"])
    st.session_state.ki = float(defaults["ki"])
    st.session_state.kd = float(defaults["kd"])


st.title("PID Controller Tuner & Simulator")
st.caption(
    "A transparent classical-control sandbox: tune P/I/D gains, inspect the "
    "individual terms, exercise actuator saturation and anti-windup, inject "
    "disturbances, add sensor noise, and compare tracking metrics."
)

with st.sidebar:
    st.header("1 · Plant")
    plant_key = st.selectbox(
        "System to control",
        options=list(PLANT_REGISTRY),
        format_func=lambda key: PLANT_REGISTRY[key].name,
    )
    plant_cls = PLANT_REGISTRY[plant_key]

    if st.session_state.get("active_plant") != plant_key:
        st.session_state.active_plant = plant_key
        _reset_gain_state(plant_key)
        st.session_state.pop("tune_message", None)

    st.caption(plant_cls.description)
    st.code(plant_cls.equation, language=None)

    st.header("2 · Identification-based start")
    tuning_method = st.selectbox(
        "Tuning rule",
        options=("lambda_pi", "ziegler_nichols"),
        format_func=lambda value: (
            "Lambda-tuned PI (recommended starting point)"
            if value == "lambda_pi"
            else "Ziegler–Nichols reaction-curve PID"
        ),
        help=(
            "Both rules first fit a simple FOPDT model to an open-loop step response. "
            "Open-loop unstable plants are rejected explicitly."
        ),
    )

    if st.button("Identify plant and apply suggested gains", use_container_width=True):
        attempt = tune_plant(plant_cls, method=tuning_method)
        if attempt.suggestion is None:
            st.session_state.tune_message = ("warning", attempt.reason or "Tune unavailable")
        else:
            suggestion = attempt.suggestion
            st.session_state.kp = float(suggestion.kp)
            st.session_state.ki = float(suggestion.ki)
            st.session_state.kd = float(suggestion.kd)
            model = suggestion.model
            st.session_state.tune_message = (
                "success",
                (
                    f"Applied {suggestion.method}: Kp={suggestion.kp:.4g}, "
                    f"Ki={suggestion.ki:.4g}, Kd={suggestion.kd:.4g}. "
                    f"FOPDT fit: K={model.gain:.4g}, τ={model.tau:.3g}s, "
                    f"L={model.delay:.3g}s."
                ),
            )
            st.rerun()

    if "tune_message" in st.session_state:
        level, message = st.session_state.tune_message
        getattr(st, level)(message)

    st.header("3 · PID controller")
    kp = st.number_input("Kp", min_value=0.0, step=0.05, key="kp")
    ki = st.number_input("Ki", min_value=0.0, step=0.05, key="ki")
    kd = st.number_input("Kd", min_value=0.0, step=0.01, key="kd")
    derivative_filter_tau = st.number_input(
        "Derivative filter τ (s)", min_value=0.0, value=0.02, step=0.01
    )
    anti_windup = st.checkbox("Conditional anti-windup", value=True)

    st.header("4 · Simulation")
    setpoint = st.number_input(
        "Setpoint", value=float(plant_cls.default_setpoint), key=f"sp_{plant_key}"
    )
    duration_default = 60.0 if plant_key == "thermal" else 15.0
    duration = st.slider("Duration (s)", 2.0, 120.0, duration_default, step=1.0)
    dt = st.select_slider(
        "Sample period dt (s)", options=[0.005, 0.01, 0.02, 0.05, 0.1], value=0.02
    )

    with st.expander("Sensor experiment"):
        measurement_noise_std = st.number_input(
            "Gaussian noise σ", min_value=0.0, value=0.0, step=0.01
        )
        measurement_filter_tau = st.number_input(
            "Measurement low-pass τ (s)", min_value=0.0, value=0.0, step=0.01
        )
        random_seed = st.number_input("Noise random seed", value=7, step=1)

    with st.expander("Disturbance experiment"):
        enable_disturbance = st.checkbox("Enable disturbance", value=False)
        disturbance_kind = st.selectbox("Type", ("impulse", "pulse", "step"))
        disturbance_time = st.slider(
            "Start time (s)", 0.0, duration, min(duration / 2.0, duration), step=0.1
        )
        disturbance_magnitude = st.number_input("Magnitude", value=5.0, step=0.5)
        disturbance_duration = st.number_input(
            "Pulse duration (s)", min_value=0.0, value=1.0, step=0.1
        )

    if st.button("Reset to plant defaults", use_container_width=True):
        _reset_gain_state(plant_key)
        st.session_state.pop("tune_message", None)
        st.rerun()


plant = plant_cls()
results = run_simulation(
    plant=plant,
    kp=float(kp),
    ki=float(ki),
    kd=float(kd),
    setpoint=float(setpoint),
    duration=float(duration),
    dt=float(dt),
    disturbance_time=float(disturbance_time) if enable_disturbance else None,
    disturbance_magnitude=float(disturbance_magnitude) if enable_disturbance else 0.0,
    disturbance_kind=disturbance_kind,
    disturbance_duration=float(disturbance_duration),
    derivative_filter_tau=float(derivative_filter_tau),
    anti_windup=anti_windup,
    measurement_noise_std=float(measurement_noise_std),
    measurement_filter_tau=float(measurement_filter_tau),
    random_seed=int(random_seed),
)

metrics = compute_metrics(
    results["t"],
    results["setpoint"],
    results["output"],
    control=results["control"],
    saturated=results["saturated"],
)

st.subheader(f"{plant_cls.name} · closed-loop summary")
metric_columns = st.columns(6)
metric_columns[0].metric("Rise time", _fmt_metric(metrics["rise_time_s"], "s"))
metric_columns[1].metric("Overshoot", _fmt_metric(metrics["overshoot_pct"], "%", 1))
metric_columns[2].metric("Settling time", _fmt_metric(metrics["settling_time_s"], "s"))
metric_columns[3].metric("Steady-state error", _fmt_metric(metrics["steady_state_error"], digits=3))
metric_columns[4].metric("ITAE", _fmt_metric(metrics["ITAE"], digits=2))
metric_columns[5].metric(
    "Saturation",
    _fmt_metric(100.0 * float(metrics.get("saturation_fraction") or 0.0), "%", 1),
)

response_df = pd.DataFrame(
    {
        "time (s)": results["t"],
        "setpoint": results["setpoint"],
        "true output": results["output"],
        "controller measurement": results["measurement"],
    }
).set_index("time (s)")
control_df = pd.DataFrame(
    {
        "time (s)": results["t"],
        "control": results["control"],
        "disturbance": results["disturbance"],
    }
).set_index("time (s)")
pid_df = pd.DataFrame(
    {
        "time (s)": results["t"],
        "P": results["p"],
        "I": results["i"],
        "D": results["d"],
    }
).set_index("time (s)")

response_tab, control_tab, pid_tab, details_tab = st.tabs(
    ["Response", "Control & disturbance", "P / I / D terms", "Run details"]
)
with response_tab:
    st.line_chart(response_df)
    st.caption(
        f"True output: {plant_cls.output_label} [{plant_cls.output_unit}]. "
        "The controller-measurement trace separates sensor effects from plant state."
    )
with control_tab:
    st.line_chart(control_df)
    st.caption(
        f"Default actuator limits for this model: {plant_cls.control_limits[0]:g} to "
        f"{plant_cls.control_limits[1]:g} {plant_cls.control_unit}."
    )
with pid_tab:
    st.line_chart(pid_df)
with details_tab:
    st.json(metrics)
    st.markdown(
        "**Interpretation note.** Auto-tuned gains are educational starting points. "
        "They are not validated for physical hardware, unmodelled dynamics, sample-time "
        "jitter, quantization, sensor failure, or safety-critical use."
    )

configuration = {
    "plant": plant_key,
    "plant_name": plant_cls.name,
    "pid": {
        "kp": float(kp),
        "ki": float(ki),
        "kd": float(kd),
        "derivative_filter_tau": float(derivative_filter_tau),
        "anti_windup": bool(anti_windup),
        "output_limits": list(plant_cls.control_limits),
    },
    "simulation": {
        "setpoint": float(setpoint),
        "duration": float(duration),
        "dt": float(dt),
        "measurement_noise_std": float(measurement_noise_std),
        "measurement_filter_tau": float(measurement_filter_tau),
        "random_seed": int(random_seed),
        "disturbance_enabled": bool(enable_disturbance),
        "disturbance_kind": disturbance_kind,
        "disturbance_time": float(disturbance_time),
        "disturbance_magnitude": float(disturbance_magnitude),
        "disturbance_duration": float(disturbance_duration),
    },
    "metrics": metrics,
}

download_col1, download_col2 = st.columns(2)
with download_col1:
    st.download_button(
        "Download simulation CSV",
        data=results_to_csv(results),
        file_name=f"{plant_key}_pid_simulation.csv",
        mime="text/csv",
        use_container_width=True,
    )
with download_col2:
    st.download_button(
        "Download configuration + metrics JSON",
        data=configuration_to_json(configuration),
        file_name=f"{plant_key}_pid_run.json",
        mime="application/json",
        use_container_width=True,
    )

with st.expander("Closed-loop architecture"):
    diagram_path = REPO_ROOT / "assets" / "diagram.svg"
    if diagram_path.exists():
        st.image(str(diagram_path), use_container_width=True)

st.markdown("---")
st.caption(
    "Educational simulator. Use the CLI and automated tests for reproducible, "
    "headless validation; use this UI for interactive intuition and comparison."
)
