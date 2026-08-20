# Changelog

All notable project changes are recorded here.

## [2.0.0] - 2026-08-20

### Added
- Conventional `src/pid_tuner/` Python package structure.
- Automated unit tests for the controller, plants, simulation, metrics, tuning, export, and CLI.
- GitHub Actions CI across Python 3.10-3.13 with Ruff, Black, pytest, and branch coverage.
- `pyproject.toml` packaging and developer-tool configuration.
- Headless `pid-tuner-sim` command-line interface.
- Measurement-noise experiments with deterministic random seeds.
- Optional first-order sensor low-pass filtering.
- Impulse, pulse, and step disturbance profiles.
- CSV simulation export and JSON configuration/metrics export.
- Plant-specific actuator limits and richer plant metadata.
- Conservative lambda-tuned PI auto-tuning option.
- Explicit diagnostics when open-loop/FOPDT tuning is inappropriate.
- Research metadata (`CITATION.cff`) and architecture/mathematics documentation.

### Changed
- Corrected anti-windup behavior to true conditional integration: the integral is frozen only when its update would push farther into saturation.
- Derivative action remains derivative-on-measurement, now with validation and exposed diagnostic state.
- FOPDT identification uses an averaged tail value, interpolated threshold crossings, and a settling check.
- Ziegler-Nichols reaction-curve PID refuses nearly zero identified dead time instead of hiding the singularity behind an arbitrary epsilon.
- Performance metrics now support negative steps, non-zero initial outputs, and zero-setpoint stabilization more cleanly.
- Simulation records true output separately from the controller's noisy/filtered measurement.
- Disturbance semantics are explicit rather than being implicitly a one-sample event.
- README installation commands and project tree now match the standalone repository.
- Existing root images are organized under `assets/`.

### Removed
- Root-level `controller.py`, `plants.py`, `metrics.py`, `simulation.py`, and `tuning.py`; their maintained implementations now live under `src/pid_tuner/`.

## [1.x]

Original interactive Streamlit prototype with three plants, manual PID tuning,
basic performance metrics, disturbance injection, and reaction-curve tuning.
