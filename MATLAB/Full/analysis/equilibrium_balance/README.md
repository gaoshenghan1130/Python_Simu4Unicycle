# Equilibrium-manifold balance controller

Editable MATLAB entry: `MATLAB/Full/studies/run_equilibrium_balance_study.m` (via `run_study('equilibrium_balance')`).

Analysis documents now live in:

- `Derivation/FullModel/analysis/equilibrium_balance_controller.md`
- `Derivation/FullModel/analysis/equilibrium_balance_validation.md`

The entry script shows one figure per speed, and saves CSV and MAT
outputs under `MATLAB/Full/results/equilibrium_balance_interactive/`.
PNG export is opt-in through `save_figures=true` in the entry script.
Each run replaces the same-named outputs there; `study.mat` includes the configuration.
