from pid_tuner.cli import main


def test_cli_smoke(capsys):
    code = main(["--plant", "dc_motor", "--duration", "0.1", "--dt", "0.02"])
    captured = capsys.readouterr()
    assert code == 0
    assert '"plant": "DC Motor (Speed Control)"' in captured.out


def test_cli_reports_unavailable_pendulum_tuning(capsys):
    code = main(["--plant", "pendulum", "--auto-tune", "lambda_pi"])
    captured = capsys.readouterr()
    assert code == 2
    assert "Auto-tune unavailable" in captured.out
