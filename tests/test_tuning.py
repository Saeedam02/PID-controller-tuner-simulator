import math

import pytest

from pid_tuner.plants import DCMotorSpeed, InvertedPendulum, ThermalSystem
from pid_tuner.tuning import (
    FOPDTModel,
    fit_fopdt,
    lambda_pi,
    suggest_pid,
    tune_plant,
    ziegler_nichols_pid,
)


def synthetic_fopdt(gain=2.0, tau=4.0, delay=1.0, step=3.0, dt=0.02, duration=25.0):
    t = [i * dt for i in range(int(duration / dt) + 1)]
    y = []
    for ti in t:
        if ti <= delay:
            y.append(0.0)
        else:
            y.append(gain * step * (1.0 - math.exp(-(ti - delay) / tau)))
    return t, y


def test_fopdt_two_point_fit_recovers_synthetic_model():
    t, y = synthetic_fopdt()
    fit = fit_fopdt(t, y, step_size=3.0)
    assert fit is not None
    assert fit.gain == pytest.approx(2.0, rel=0.02)
    assert fit.tau == pytest.approx(4.0, rel=0.05)
    assert fit.delay == pytest.approx(1.0, abs=0.15)


def test_lambda_pi_handles_zero_delay():
    model = FOPDTModel(2.0, 5.0, 0.0, 0.0, 2.0)
    kp, ki, kd = lambda_pi(model)
    assert kp > 0
    assert ki > 0
    assert kd == 0.0


def test_ziegler_nichols_rejects_nearly_zero_delay():
    model = FOPDTModel(2.0, 5.0, 0.001, 0.0, 2.0)
    with pytest.raises(ValueError):
        ziegler_nichols_pid(model)


def test_ziegler_nichols_matches_classic_formula():
    model = FOPDTModel(2.0, 10.0, 1.0, 0.0, 2.0)
    kp, ki, kd = ziegler_nichols_pid(model)
    assert kp == pytest.approx(6.0)
    assert ki == pytest.approx(3.0)
    assert kd == pytest.approx(3.0)


def test_pendulum_is_explicitly_rejected_for_open_loop_tuning():
    attempt = tune_plant(InvertedPendulum)
    assert not attempt.available
    assert "open-loop unstable" in (attempt.reason or "")


def test_dc_motor_lambda_tune_is_available():
    attempt = tune_plant(DCMotorSpeed, method="lambda_pi")
    assert attempt.available
    assert attempt.suggestion is not None
    assert attempt.suggestion.kp > 0
    assert attempt.suggestion.ki > 0


def test_thermal_lambda_tune_is_available():
    attempt = tune_plant(ThermalSystem, method="lambda_pi", dt=0.05)
    assert attempt.available


def test_compatibility_suggest_pid_returns_flat_dict():
    suggestion = suggest_pid(DCMotorSpeed)
    assert suggestion is not None
    assert {"kp", "ki", "kd", "fitted_K", "fitted_tau", "fitted_L"} <= suggestion.keys()


def test_open_loop_response_validates_parameters():
    from pid_tuner.tuning import open_loop_step_response

    with pytest.raises(ValueError):
        open_loop_step_response(DCMotorSpeed, dt=0.0)
    with pytest.raises(ValueError):
        open_loop_step_response(DCMotorSpeed, step_size=0.0)
    with pytest.raises(ValueError):
        open_loop_step_response(DCMotorSpeed, duration=0.01, dt=0.02)


def test_fopdt_fit_validation_and_rejection_paths():
    with pytest.raises(ValueError):
        fit_fopdt([0, 1], [0, 1], 1.0)
    with pytest.raises(ValueError):
        fit_fopdt([0, 1, 2], [0, 1, 1], 0.0)
    with pytest.raises(ValueError):
        fit_fopdt([0, 1, 2], [0, 1, 1], 1.0, tail_fraction=0.9)
    assert fit_fopdt([0, 1, 2], [0, float("nan"), 1], 1.0) is None
    assert fit_fopdt([0, 1, 2, 3], [1, 1, 1, 1], 1.0) is None


def test_fopdt_rejects_unsettled_tail():
    t = list(range(20))
    y = [float(i) for i in range(20)]
    assert fit_fopdt(t, y, 1.0, tail_fraction=0.2) is None


def test_lambda_pi_rejects_nonpositive_requested_lambda():
    model = FOPDTModel(2.0, 5.0, 0.2, 0.0, 2.0)
    with pytest.raises(ValueError):
        lambda_pi(model, closed_loop_time_constant=0.0)


def test_tune_plant_reports_ziegler_nichols_small_delay_problem():
    attempt = tune_plant(DCMotorSpeed, method="ziegler_nichols")
    assert not attempt.available
    assert "dead time" in (attempt.reason or "")


def test_tune_plant_reports_unknown_method():
    attempt = tune_plant(DCMotorSpeed, method="not-a-method")  # type: ignore[arg-type]
    assert not attempt.available
    assert "Unknown tuning method" in (attempt.reason or "")
