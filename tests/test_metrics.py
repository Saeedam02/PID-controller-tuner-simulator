import pytest

from pid_tuner.metrics import compute_metrics


def test_monotonic_step_metrics():
    t = [0, 1, 2, 3, 4, 5]
    sp = [10] * len(t)
    y = [0, 2, 6, 9, 10, 10]
    m = compute_metrics(t, sp, y)
    assert m["rise_time_s"] == pytest.approx(2.0)  # 10% at t=1, 90% at t=3
    assert m["overshoot_pct"] == 0.0
    assert m["settling_time_s"] == pytest.approx(4.0)
    assert m["steady_state_error"] == pytest.approx(0.0)


def test_positive_overshoot_is_normalized_by_transition():
    t = [0, 1, 2, 3]
    sp = [10] * 4
    y = [0, 8, 12, 10]
    m = compute_metrics(t, sp, y)
    assert m["overshoot_pct"] == pytest.approx(20.0)


def test_negative_step_overshoot():
    t = [0, 1, 2, 3]
    sp = [-10] * 4
    y = [0, -8, -12, -10]
    m = compute_metrics(t, sp, y)
    assert m["overshoot_pct"] == pytest.approx(20.0)


def test_zero_target_stabilization_has_no_rise_time():
    t = [0, 1, 2, 3, 4]
    sp = [0] * 5
    y = [0.2, 0.1, 0.04, 0.001, 0.0]
    m = compute_metrics(t, sp, y)
    assert m["rise_time_s"] is None
    assert m["settling_time_s"] == pytest.approx(3.0)


def test_unsettled_response_returns_none_settling_time():
    t = [0, 1, 2]
    sp = [1, 1, 1]
    y = [0, 0.5, 0.7]
    assert compute_metrics(t, sp, y)["settling_time_s"] is None


def test_control_and_saturation_metrics_are_added():
    t = [0, 1, 2]
    sp = [1, 1, 1]
    y = [0, 1, 1]
    m = compute_metrics(t, sp, y, control=[0, 2, 0], saturated=[False, True, False])
    assert m["control_RMS"] is not None
    assert m["control_total_variation"] == pytest.approx(4.0)
    assert m["saturation_fraction"] == pytest.approx(1 / 3)


def test_metric_input_validation():
    with pytest.raises(ValueError):
        compute_metrics([], [], [])
    with pytest.raises(ValueError):
        compute_metrics([0, 1], [1], [0, 1])
    with pytest.raises(ValueError):
        compute_metrics([0, 0], [1, 1], [0, 1])


def test_metric_rejects_nonfinite_values_and_bad_options():
    with pytest.raises(ValueError):
        compute_metrics([0, 1], [1, 1], [0, float("nan")])
    with pytest.raises(ValueError):
        compute_metrics([0, 1], [1, 1], [0, 1], settle_band=0.0)
    with pytest.raises(ValueError):
        compute_metrics([0, 1], [1, 1], [0, 1], tail_fraction=0.0)


def test_metric_rejects_bad_control_and_saturation_lengths():
    t = [0, 1]
    sp = [1, 1]
    y = [0, 1]
    with pytest.raises(ValueError):
        compute_metrics(t, sp, y, control=[0])
    with pytest.raises(ValueError):
        compute_metrics(t, sp, y, control=[0, float("inf")])
    with pytest.raises(ValueError):
        compute_metrics(t, sp, y, saturated=[False])


def test_already_settled_response_reports_zero_settling_time():
    t = [0, 1, 2]
    sp = [1, 1, 1]
    y = [1, 1, 1]
    assert compute_metrics(t, sp, y)["settling_time_s"] == 0.0
