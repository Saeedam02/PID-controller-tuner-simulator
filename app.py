"""
app.py

Streamlit UI for the PID Controller Tuner & Simulator.

Run with:
    streamlit run app.py

Lets the user:
  - Pick a plant (DC motor, thermal system, inverted pendulum)
  - Adjust Kp / Ki / Kd live with sliders
  - See the live step response, control effort, and P/I/D contributions
  - Inject a disturbance mid-simulation
  - Get a Ziegler-Nichols auto-tune suggestion for supported plants
  - See standard performance metrics (rise time, overshoot, settling time, etc.)
"""

import streamlit as st
import pandas as pd

from plants import PLANT_REGISTRY
from simulation import run_simulation
from metrics import compute_metrics
from tuning import suggest_pid

st.set_page_config(page_title="PID Controller Tuner & Simulator", layout="wide")

st.title("PID Controller Tuner & Simulator")
st.caption(
    "Tune a PID controller against a simulated plant and watch the closed-loop "
    "response update live. Built to demonstrate classical control concepts: "
    "P/I/D contributions, anti-windup, disturbance rejection, and Ziegler-Nichols tuning."
)

# ----------------------------- Sidebar controls -----------------------------
with st.sidebar:
    st.header("Plant")
    plant_key = st.selectbox(
        "Choose a system to control",
        options=list(PLANT_REGISTRY.keys()),
        format_func=lambda k: PLANT_REGISTRY[k].name,
    )
    plant_cls = PLANT_REGISTRY[plant_key]

    st.header("Setpoint & Simulation")
    setpoint = st.number_input("Setpoint", value=float(plant_cls.default_setpoint))
    duration = st.slider("Simulation duration (s)", 2.0, 60.0, 15.0, step=1.0)

    st.header("PID Gains")
    default_pid = plant_cls.default_pid
    kp = st.slider("Kp", 0.0, max(50.0, default_pid["kp"] * 3), float(default_pid["kp"]), step=0.01)
    ki = st.slider("Ki", 0.0, max(20.0, default_pid["ki"] * 3), float(default_pid["ki"]), step=0.01)
    kd = st.slider("Kd", 0.0, max(20.0, default_pid["kd"] * 3), float(default_pid["kd"]), step=0.01)

    st.header("Disturbance")
    enable_disturbance = st.checkbox("Inject a disturbance mid-run", value=False)
    disturbance_time = None
    disturbance_magnitude = 0.0
    if enable_disturbance:
        disturbance_time = st.slider("Disturbance time (s)", 0.0, duration, duration / 2)
        disturbance_magnitude = st.slider("Disturbance magnitude", -20.0, 20.0, 5.0)

    st.header("Auto-Tune (Ziegler-Nichols)")
    if st.button("Suggest PID gains from open-loop step test"):
        suggestion = suggest_pid(plant_cls)
        if suggestion is None:
            st.warning(
                "This plant doesn't have a stable open-loop step response "
                "(e.g. the inverted pendulum), so this method doesn't apply. "
                "Try the DC motor or thermal system instead."
            )
        else:
            st.success(
                f"Suggested: Kp={suggestion['kp']:.3f}, "
                f"Ki={suggestion['ki']:.3f}, Kd={suggestion['kd']:.3f}"
            )
            st.caption(
                f"(Fitted FOPDT model: K={suggestion['fitted_K']:.3f}, "
                f"tau={suggestion['fitted_tau']:.2f}s, L={suggestion['fitted_L']:.2f}s)"
            )

# ----------------------------- Run simulation -----------------------------
plant = plant_cls()
results = run_simulation(
    plant=plant,
    kp=kp,
    ki=ki,
    kd=kd,
    setpoint=setpoint,
    duration=duration,
    dt=0.02,
    disturbance_time=disturbance_time,
    disturbance_magnitude=disturbance_magnitude,
)

metrics = compute_metrics(results["t"], results["setpoint"], results["output"])

# ----------------------------- Metrics row -----------------------------
col1, col2, col3, col4, col5 = st.columns(5)
col1.metric("Rise time", f"{metrics['rise_time_s']:.2f} s" if metrics["rise_time_s"] else "N/A")
col2.metric("Overshoot", f"{metrics['overshoot_pct']:.1f} %")
col3.metric("Settling time", f"{metrics['settling_time_s']:.2f} s")
col4.metric("Steady-state error", f"{metrics['steady_state_error']:.3f}")
col5.metric("ITAE", f"{metrics['ITAE']:.2f}")

# ----------------------------- Plots -----------------------------
st.subheader(f"{plant_cls.name}: {plant_cls.output_label} ({plant_cls.output_unit})")
response_df = pd.DataFrame({
    "time (s)": results["t"],
    "setpoint": results["setpoint"],
    "output": results["output"],
}).set_index("time (s)")
st.line_chart(response_df)

st.subheader(f"Control Effort ({plant_cls.control_label})")
control_df = pd.DataFrame({
    "time (s)": results["t"],
    "control output": results["control"],
}).set_index("time (s)")
st.line_chart(control_df)

with st.expander("Show P / I / D term breakdown"):
    pid_df = pd.DataFrame({
        "time (s)": results["t"],
        "P term": results["p"],
        "I term": results["i"],
        "D term": results["d"],
    }).set_index("time (s)")
    st.line_chart(pid_df)

st.markdown("---")
st.caption(
    "Tip: start with Kp only (Ki=Kd=0), increase until you see reasonable response speed "
    "with some oscillation, then add Ki to remove steady-state error, then add Kd to damp "
    "overshoot. Or just hit 'Auto-Tune' for a Ziegler-Nichols starting point."
)
