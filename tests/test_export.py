import json

import pytest

from pid_tuner.export import configuration_to_json, results_to_csv


def test_results_to_csv_has_header_and_rows():
    data = {
        "t": [0.0, 1.0],
        "setpoint": [1.0, 1.0],
        "output": [0.0, 0.8],
        "measurement": [0.0, 0.8],
        "control": [1.0, 0.2],
        "p": [1.0, 0.2],
        "i": [0.0, 0.0],
        "d": [0.0, 0.0],
        "error": [1.0, 0.2],
        "saturated": [False, False],
        "disturbance": [0.0, 0.0],
    }
    text = results_to_csv(data)
    assert text.splitlines()[0].startswith("t,setpoint,output")
    assert len(text.splitlines()) == 3


def test_results_to_csv_rejects_misaligned_columns():
    data = {"a": [1, 2], "b": [1]}
    with pytest.raises(ValueError):
        results_to_csv(data, columns=("a", "b"))


def test_configuration_to_json_is_parseable():
    text = configuration_to_json({"plant": "dc_motor", "kp": 1.0})
    assert json.loads(text)["plant"] == "dc_motor"


def test_results_to_csv_rejects_missing_column():
    with pytest.raises(KeyError):
        results_to_csv({"a": [1]}, columns=("a", "missing"))


def test_write_text_creates_parent_directories(tmp_path):
    from pid_tuner.export import write_text

    target = write_text(tmp_path / "nested" / "result.txt", "hello")
    assert target.read_text(encoding="utf-8") == "hello"
