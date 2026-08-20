# v2.0.0 release checklist

- [ ] Create a migration branch from the current `main` branch.
- [ ] Replace the working tree with the v2 bundle, preserving `.git/`.
- [ ] Remove the old root modules listed in `MIGRATION.md`.
- [ ] Run `python -m pip install -e ".[dev]"`.
- [ ] Run `ruff check .`.
- [ ] Run `black --check .` (or format once with `black .` before committing).
- [ ] Run `pytest --cov=pid_tuner --cov-branch --cov-report=term-missing`.
- [ ] Run `streamlit run app.py` and manually test all three plants.
- [ ] Test lambda-PI identification for DC motor and thermal plants.
- [ ] Confirm the inverted pendulum gives an explicit auto-tuning rejection.
- [ ] Test noise, sensor filtering, and all three disturbance modes.
- [ ] Test CSV and JSON downloads.
- [ ] Test the CLI: `pid-tuner-sim --plant dc_motor`.
- [ ] Push the migration branch and wait for GitHub Actions to pass.
- [ ] Merge to `main`.
- [ ] Tag `v2.0.0` and create the GitHub release.
- [ ] Optionally archive the release with Zenodo if a DOI is useful for academic applications.
