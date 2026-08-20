# Contributing

Contributions are welcome, especially when they improve control-theory clarity,
validation, or educational usefulness.

## Development setup

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate
# macOS/Linux: source .venv/bin/activate
python -m pip install -e ".[dev]"
```

## Before opening a pull request

Run all quality checks locally:

```bash
ruff check .
black --check .
pytest --cov=pid_tuner --cov-branch --cov-report=term-missing
```

When adding a controller feature or plant model, include tests that verify the
mathematical behavior rather than only checking that the code executes.

## Design principles

1. Keep plant models, controller logic, metrics, and UI code separate.
2. State modelling assumptions explicitly in docstrings and documentation.
3. Do not label a tuning method as valid when its assumptions are not met.
4. Prefer deterministic tests and seeded noise experiments.
5. Keep the core package lightweight; add large dependencies only when they
   materially improve the engineering value of the project.
