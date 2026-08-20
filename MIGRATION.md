# Migration from the original flat repository

Version 2.0 uses a standard `src/` package layout. If you are upgrading the
existing GitHub repository, the cleanest method is to replace the working tree
with the contents of the provided v2 bundle while preserving `.git/`.

## Delete the old root modules

Remove these old files after copying v2:

```text
controller.py
metrics.py
plants.py
simulation.py
tuning.py
Animation.gif
diagram.svg
```

Their replacements are:

```text
src/pid_tuner/controller.py
src/pid_tuner/metrics.py
src/pid_tuner/plants.py
src/pid_tuner/simulation.py
src/pid_tuner/tuning.py
assets/Animation.gif
assets/diagram.svg
```

`app.py` remains at the repository root because Streamlit uses it as the entry
point. It imports the new package under `src/`.

## Validate before merging

```bash
python -m pip install -e ".[dev]"
ruff check .
black --check .
pytest --cov=pid_tuner --cov-branch --cov-report=term-missing
streamlit run app.py
```

For the safest Git workflow, perform the migration on a branch and merge only
after CI passes.
