import pytest

from pid_tuner.controller import PIDConfig, PIDController


def test_config_rejects_bad_limits():
    with pytest.raises(ValueError):
        PIDConfig(output_min=1.0, output_max=1.0)


def test_config_rejects_negative_derivative_tau():
    with pytest.raises(ValueError):
        PIDConfig(derivative_filter_tau=-0.1)


def test_proportional_only_response():
    controller = PIDController(PIDConfig(kp=2.0))
    result = controller.step(setpoint=3.0, measurement=1.0, dt=0.1)
    assert result.error == pytest.approx(2.0)
    assert result.p == pytest.approx(4.0)
    assert result.i == pytest.approx(0.0)
    assert result.d == pytest.approx(0.0)
    assert result.output == pytest.approx(4.0)


def test_derivative_on_measurement_avoids_setpoint_kick():
    controller = PIDController(PIDConfig(kp=0.0, kd=5.0, derivative_filter_tau=0.0))
    controller.step(setpoint=0.0, measurement=2.0, dt=0.1)
    result = controller.step(setpoint=10.0, measurement=2.0, dt=0.1)
    assert result.d == pytest.approx(0.0)


def test_derivative_reacts_to_measurement_change():
    controller = PIDController(PIDConfig(kp=0.0, kd=2.0, derivative_filter_tau=0.0))
    controller.step(setpoint=0.0, measurement=0.0, dt=0.1)
    result = controller.step(setpoint=0.0, measurement=1.0, dt=0.1)
    assert result.derivative_state == pytest.approx(-10.0)
    assert result.d == pytest.approx(-20.0)


def test_conditional_anti_windup_freezes_when_pushing_farther_into_high_limit():
    controller = PIDController(
        PIDConfig(kp=0.0, ki=10.0, output_min=-1.0, output_max=1.0, anti_windup=True)
    )
    result = controller.step(setpoint=10.0, measurement=0.0, dt=1.0)
    assert result.saturated is False  # integral update is rejected before final output
    assert controller.integral_state == pytest.approx(0.0)
    assert result.output == pytest.approx(0.0)


def test_conditional_anti_windup_allows_update_that_opposes_active_saturation():
    controller = PIDController(
        PIDConfig(
            kp=0.0,
            ki=1.0,
            kd=1.0,
            output_min=-2.0,
            output_max=2.0,
            derivative_filter_tau=0.0,
            anti_windup=True,
        )
    )
    # Establish the previous measurement. The large negative error would try to
    # saturate low through I, so anti-windup correctly freezes the integrator.
    controller.step(setpoint=0.0, measurement=10.0, dt=1.0)
    assert controller.integral_state == pytest.approx(0.0)

    # The measurement now drops quickly, producing a large *positive* D term and
    # high saturation. The error is still negative, so the proposed negative
    # integral update opposes that high saturation and must be allowed.
    result = controller.step(setpoint=0.0, measurement=5.0, dt=1.0)
    assert controller.integral_state == pytest.approx(-5.0)
    # The accepted integral update exactly cancels the transient derivative term
    # in this constructed example, pulling the command back inside the limits.
    assert result.saturated is False
    assert result.output == pytest.approx(0.0)


def test_without_anti_windup_integral_accumulates_under_saturation():
    controller = PIDController(
        PIDConfig(kp=0.0, ki=10.0, output_min=-1.0, output_max=1.0, anti_windup=False)
    )
    result = controller.step(setpoint=10.0, measurement=0.0, dt=1.0)
    assert result.saturated is True
    assert controller.integral_state == pytest.approx(10.0)
    assert result.output == pytest.approx(1.0)


def test_reset_clears_controller_memory():
    controller = PIDController(PIDConfig(ki=1.0, kd=1.0))
    controller.step(setpoint=1.0, measurement=0.0, dt=0.1)
    controller.reset()
    assert controller.integral_state == 0.0
    assert controller.derivative_state == 0.0


def test_step_rejects_nonpositive_dt():
    controller = PIDController(PIDConfig())
    with pytest.raises(ValueError):
        controller.step(1.0, 0.0, 0.0)


def test_config_rejects_nonfinite_gain():
    with pytest.raises(ValueError):
        PIDConfig(kp=float("inf"))


def test_step_rejects_nonfinite_signal():
    controller = PIDController(PIDConfig())
    with pytest.raises(ValueError):
        controller.step(float("nan"), 0.0, 0.1)
