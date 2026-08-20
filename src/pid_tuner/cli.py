"""Command-line interface for reproducible, headless simulations.

Examples
--------
Run the DC motor with its checked-in default PID gains::

    pid-tuner-sim --plant dc_motor

Run a noisy thermal simulation and export the time history::

    pid-tuner-sim --plant thermal --noise-std 0.2 --csv results/thermal.csv

Ask for an identification-based PI starting point before simulating::

    pid-tuner-sim --plant thermal --auto-tune lambda_pi
"""

from __future__ import annotations

import argparse
import json

from .export import results_to_csv, write_text
from .metrics import compute_metrics
from .plants import PLANT_REGISTRY
from .simulation import run_simulation
from .tuning import tune_plant


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Simulate and evaluate a PID-controlled educational plant."
    )
    parser.add_argument("--plant", choices=sorted(PLANT_REGISTRY), default="dc_motor")
    parser.add_argument("--kp", type=float, default=None)
    parser.add_argument("--ki", type=float, default=None)
    parser.add_argument("--kd", type=float, default=None)
    parser.add_argument("--setpoint", type=float, default=None)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument(
        "--auto-tune",
        choices=("lambda_pi", "ziegler_nichols"),
        default=None,
        help="Identify the plant first and use the resulting starting gains.",
    )
    parser.add_argument("--noise-std", type=float, default=0.0)
    parser.add_argument("--measurement-filter-tau", type=float, default=0.0)
    parser.add_argument(
        "--disturbance-kind",
        choices=("impulse", "pulse", "step"),
        default="impulse",
    )
    parser.add_argument("--disturbance-time", type=float, default=None)
    parser.add_argument("--disturbance-magnitude", type=float, default=0.0)
    parser.add_argument("--disturbance-duration", type=float, default=1.0)
    parser.add_argument("--csv", type=str, default=None, help="Optional CSV output path.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    plant_cls = PLANT_REGISTRY[args.plant]
    plant = plant_cls()

    gains = dict(plant_cls.default_pid)
    if args.auto_tune:
        attempt = tune_plant(plant_cls, method=args.auto_tune, dt=args.dt)
        if not attempt.available:
            print(f"Auto-tune unavailable: {attempt.reason}")
            return 2
        assert attempt.suggestion is not None
        gains.update(
            kp=attempt.suggestion.kp,
            ki=attempt.suggestion.ki,
            kd=attempt.suggestion.kd,
        )

    # Explicit CLI gains override defaults or auto-tuned values one-by-one.
    for name in ("kp", "ki", "kd"):
        value = getattr(args, name)
        if value is not None:
            gains[name] = value

    setpoint = plant_cls.default_setpoint if args.setpoint is None else args.setpoint
    results = run_simulation(
        plant,
        kp=gains["kp"],
        ki=gains["ki"],
        kd=gains["kd"],
        setpoint=setpoint,
        duration=args.duration,
        dt=args.dt,
        disturbance_time=args.disturbance_time,
        disturbance_magnitude=args.disturbance_magnitude,
        disturbance_kind=args.disturbance_kind,
        disturbance_duration=args.disturbance_duration,
        measurement_noise_std=args.noise_std,
        measurement_filter_tau=args.measurement_filter_tau,
    )
    metrics = compute_metrics(
        results["t"],
        results["setpoint"],
        results["output"],
        control=results["control"],
        saturated=results["saturated"],
    )

    summary = {
        "plant": plant_cls.name,
        "setpoint": setpoint,
        "gains": gains,
        "metrics": metrics,
    }
    print(json.dumps(summary, indent=2))

    if args.csv:
        path = write_text(args.csv, results_to_csv(results))
        print(f"CSV written to {path}")

    return 0


if __name__ == "__main__":  # pragma: no cover - exercised through console script
    raise SystemExit(main())
