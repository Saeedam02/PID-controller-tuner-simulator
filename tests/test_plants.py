import math

import pytest

from pid_tuner.plants import (
    DCMotorSpeed,
    InvertedPendulum,
    PLANT_REGISTRY,
    ThermalSystem,
    create_plant,
    rk4_step,
)


def test_rk4_matches_simple_exponential_decay():
    def dynamics(state, control, disturbance):
        del control, disturbance
        return [-state[0]]

    x = [1.0]
    dt = 0.1
    for _ in range(10):
        x = rk4_step(dynamics, x, 0.0, 0.0, dt)
    assert x[0] == pytest.approx(math.exp(-1.0), rel=2e-5)


def test_dc_motor_approaches_expected_open_loop_speed():
    motor = DCMotorSpeed()
    # Steady state for u=1 and zero load is Kt*u/b = 5 rad/s.
    for _ in range(300):
        motor.step(1.0, 0.01)
    assert motor.output() == pytest.approx(5.0, rel=1e-4)


def test_dc_motor_load_disturbance_reduces_acceleration():
    nominal = DCMotorSpeed()
    disturbed = DCMotorSpeed()
    nominal.step(2.0, 0.01, disturbance=0.0)
    disturbed.step(2.0, 0.01, disturbance=0.5)
    assert disturbed.output() < nominal.output()


def test_thermal_system_approaches_expected_equilibrium():
    plant = ThermalSystem()
    # At u=10, equilibrium is ambient + K*u = 40 °C.
    for _ in range(2000):
        plant.step(10.0, 0.05)
    assert plant.output() == pytest.approx(40.0, abs=0.4)


def test_negative_thermal_disturbance_cools_relative_to_nominal():
    nominal = ThermalSystem()
    disturbed = ThermalSystem()
    nominal.step(5.0, 1.0, disturbance=0.0)
    disturbed.step(5.0, 1.0, disturbance=-5.0)
    assert disturbed.output() < nominal.output()


def test_pendulum_is_open_loop_unstable():
    pendulum = InvertedPendulum(initial_angle=0.05)
    initial = abs(pendulum.output())
    for _ in range(40):
        pendulum.step(0.0, 0.01)
    assert abs(pendulum.output()) > initial
    assert pendulum.supports_open_loop_tuning is False


def test_pendulum_negative_force_opposes_positive_fall_initially():
    free = InvertedPendulum(initial_angle=0.1)
    controlled = InvertedPendulum(initial_angle=0.1)
    free.step(0.0, 0.02)
    controlled.step(-5.0, 0.02)
    assert controlled.state[1] < free.state[1]


def test_registry_and_factory():
    assert set(PLANT_REGISTRY) == {"dc_motor", "thermal", "pendulum"}
    assert isinstance(create_plant("dc_motor"), DCMotorSpeed)
    with pytest.raises(KeyError):
        create_plant("unknown")
