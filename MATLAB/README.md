# MATLAB longitudinal Segway simulation

This folder contains the MATLAB conversion of the longitudinal Python Segway
rolling-resistance simulation.

The implementation follows:

- `Derivation/Segway/Segway_rollingResistance.ipynb`
- `Derivation/Segway/Models/Segway_model_rollingResistance.py`
- `Derivation/Segway/Models/Segway_model_motorDamp.py`
- `Derivation/Segway/Controllers/Segway_Controller.py`
- `Derivation/Segway/Parameters.py`

Only the longitudinal states are included:

```matlab
z = [x; x_dot; gamma; gamma_dot]
```

The model includes:

- nonlinear pendulum/wheel mass matrix
- motor damping torque
- smoothed rolling resistance
- PD/PID balance, velocity, and position control modes

The default `simulation.m` setup mirrors the position-mode example in
`Segway_rollingResistance.ipynb`:

```matlab
scenario = 'position';
par.control_strategy = 'pd';
par.desired_position = 1.0;
par.desired_velocity = 0.0;
```

Switch `scenario` to `'velocity'` to run the notebook's velocity-mode example.

Run from MATLAB with:

```matlab
cd Matlab
simulation
```
