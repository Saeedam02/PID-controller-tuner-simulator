import pytest

from pid_tuner.controller import PIDConfig
from pid_tuner.plants import DCMotorSpeed, ThermalSystem
from pid_tuner.simulation import (
    DisturbanceConfig,
    SimulationConfig,
    run_closed_loop,
    run_simulation,
)


def test_disturbance_modes():
    impulse = DisturbanceConfig(kind="impulse", start_time=1.0, magnitude=3.0)
    assert impulse.value_at(0.99, 0.1) == 0.0
    assert impulse.value_at(1.0, 0.1) == 3.0
    assert impulse.value_at(1.11, 0.1) == 0.0

    pulse = DisturbanceConfig(kind="pulse", start_time=1.0, magnitude=2.0, duration=0.5)
    assert pulse.value_at(1.4, 0.1) == 2.0
    assert pulse.value_at(1.5, 0.1) == 0.0

    step = DisturbanceConfig(kind="step", start_time=1.0, magnitude=-2.0)
    assert step.value_at(20.0, 0.1) == -2.0


def test_run_simulation_returns_aligned_series_and_final_time():
    result = run_simulation(
        DCMotorSpeed(), kp=0.8, ki=4.0, kd=0.02, setpoint=100.0, duration=1.0, dt=0.02
    )
    lengths = {len(values) for values in result.values()}
    assert lengths == {51}
    assert result["t"][-1] == pytest.approx(1.0)


def test_default_plant_limits_are_used():
    result = run_simulation(
        DCMotorSpeed(), kp=100.0, ki=0.0, kd=0.0, setpoint=100.0, duration=0.1, dt=0.01
    )
    assert max(result["control"]) <= DCMotorSpeed.control_limits[1]
    assert any(result["saturated"])


def test_noise_is_reproducible_with_seed():
    kwargs = dict(
        kp=1.0,
        ki=0.0,
        kd=0.0,
        setpoint=10.0,
        duration=0.2,
        dt=0.02,
        measurement_noise_std=0.5,
        random_seed=123,
    )
    a = run_simulation(DCMotorSpeed(), **kwargs)
    b = run_simulation(DCMotorSpeed(), **kwargs)
    assert a["measurement"] == b["measurement"]


def test_measurement_filter_changes_noisy_measurement_trace():
    base = dict(
        kp=1.0,
        ki=0.0,
        kd=0.0,
        setpoint=10.0,
        duration=0.2,
        dt=0.02,
        measurement_noise_std=0.5,
        random_seed=5,
    )
    raw = run_simulation(DCMotorSpeed(), measurement_filter_tau=0.0, **base)
    filtered = run_simulation(DCMotorSpeed(), measurement_filter_tau=0.1, **base)
    assert raw["measurement"][1:] != filtered["measurement"][1:]


def test_disturbance_is_recorded_and_affects_plant():
    nominal = run_simulation(
        ThermalSystem(), kp=2.0, ki=0.1, kd=0.0, setpoint=40.0, duration=3.0, dt=0.05
    )
    disturbed = run_simulation(
        ThermalSystem(),
        kp=2.0,
        ki=0.1,
        kd=0.0,
        setpoint=40.0,
        duration=3.0,
        dt=0.05,
        disturbance_time=1.0,
        disturbance_kind="step",
        disturbance_magnitude=-10.0,
    )
    assert min(disturbed["disturbance"]) == -10.0
    assert disturbed["output"][-1] < nominal["output"][-1]


def test_low_level_run_closed_loop_api():
    plant = DCMotorSpeed()
    pid = PIDConfig(kp=1.0, output_min=-24.0, output_max=24.0)
    sim = SimulationConfig(setpoint=20.0, duration=0.2, dt=0.02)
    result = run_closed_loop(plant, pid, sim)
    assert len(result["t"]) == 11


def test_disturbance_config_validation():
    with pytest.raises(ValueError):
        DisturbanceConfig(kind="bad")  # type: ignore[arg-type]
    with pytest.raises(ValueError):
        DisturbanceConfig(start_time=-1.0)
    with pytest.raises(ValueError):
        DisturbanceConfig(duration=-1.0)
    with pytest.raises(ValueError):
        DisturbanceConfig(magnitude=float("inf"))


def test_simulation_config_validation():
    with pytest.raises(ValueError):
        SimulationConfig(setpoint=1.0, duration=0.0)
    with pytest.raises(ValueError):
        SimulationConfig(setpoint=1.0, dt=0.0)
    with pytest.raises(ValueError):
        SimulationConfig(setpoint=1.0, duration=0.1, dt=0.2)
    with pytest.raises(ValueError):
        SimulationConfig(setpoint=1.0, measurement_noise_std=-1.0)
    with pytest.raises(ValueError):
        SimulationConfig(setpoint=1.0, measurement_filter_tau=-1.0)
    with pytest.raises(ValueError):
        SimulationConfig(setpoint=float("nan"))
