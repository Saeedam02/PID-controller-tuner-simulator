"""Educational PID controller tuner and simulation package."""

from .controller import PIDConfig, PIDController, PIDStep
from .metrics import compute_metrics
from .plants import DCMotorSpeed, InvertedPendulum, ThermalSystem
from .simulation import DisturbanceConfig, SimulationConfig, run_closed_loop, run_simulation
from .tuning import FOPDTModel, TuningAttempt, TuningSuggestion, tune_plant

__all__ = [
    "DCMotorSpeed",
    "DisturbanceConfig",
    "FOPDTModel",
    "InvertedPendulum",
    "PIDConfig",
    "PIDController",
    "PIDStep",
    "SimulationConfig",
    "ThermalSystem",
    "TuningAttempt",
    "TuningSuggestion",
    "compute_metrics",
    "run_closed_loop",
    "run_simulation",
    "tune_plant",
]

__version__ = "2.0.0"
