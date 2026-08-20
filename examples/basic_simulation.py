"""Minimal programmatic example using the installed package."""

from pid_tuner.metrics import compute_metrics
from pid_tuner.plants import DCMotorSpeed
from pid_tuner.simulation import run_simulation

plant = DCMotorSpeed()
results = run_simulation(
    plant,
    kp=0.8,
    ki=4.0,
    kd=0.02,
    setpoint=100.0,
    duration=5.0,
    dt=0.02,
)
metrics = compute_metrics(
    results["t"],
    results["setpoint"],
    results["output"],
    control=results["control"],
    saturated=results["saturated"],
)
print(metrics)
