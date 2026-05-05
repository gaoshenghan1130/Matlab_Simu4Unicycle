# MATLAB longitudinal Segway simulation

This folder contains the MATLAB conversion of the longitudinal Python Segway
rolling-resistance simulation.

The implementation follows:

- `Derivation/Segway/Tuner.ipynb`
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

The default `simulation.m` setup mirrors the latest position-mode Python curve
in `Tuner.ipynb`:

```matlab
scenario = 'position';
par.control_strategy = 'pd';
par.desired_position = 1.0;
par.desired_velocity = 0.0;
```

Switch `scenario` to `'velocity'` to run the latest velocity-mode tuned setup.

Run from MATLAB with:

```matlab
cd Matlab
simulation
```

## Measured data

Use `read_real_data.m` to parse measured BLE logs with the same command-interval
logic used by the Python tuning notebooks. The parser extracts position,
velocity, gamma, and dgamma from command segments such as:

```text
"--- SEND COMMAND: Mode=position, Value=1.0 ---"
```

Run the measured-data plotting example with:

```matlab
plot_real_data
```

Change `mode_name` and `target_value` in `plot_real_data.m` to switch between
position and velocity logs.
