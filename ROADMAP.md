# Roadmap

Version 2.0 focuses on a clean, validated SISO PID teaching and experimentation
platform. The following extensions are valuable but intentionally kept outside
the current release until they can be implemented with equally clear models and
tests.

## High-value next steps

### 1. State-space / LQR comparison for the inverted pendulum
Add a full cart-pendulum state-space model and compare PID angle stabilization
with LQR. This should be a separate controller implementation, not an option
silently mixed into the PID class.

### 2. Plant-specific Kalman filtering
The current simulator supports measurement noise and a simple first-order sensor
low-pass. A Kalman filter should be added only together with explicit state-space
models, process-noise covariance, measurement-noise covariance, and estimator
validation. A generic scalar "Kalman" checkbox would be misleading.

### 3. More realistic DC motor dynamics
Add armature current/electrical dynamics, back-EMF, torque constant, resistance,
and inductance. This creates a second-order electromechanical model and makes
controller bandwidth limits more meaningful.

### 4. Dead-time process model
Add a process with genuine transport delay so the Ziegler-Nichols reaction-curve
method can be demonstrated in the regime where its dead-time formula is well
conditioned.

### 5. MIMO control
A coupled two-tank benchmark should arrive with a multivariable controller or a
clearly documented decentralized-control experiment. It should not be presented
as a normal single-loop PID plant when loop interaction is the central issue.

### 6. Reproducible benchmark reports
Add a script that sweeps controller gains/methods and writes machine-readable
results plus publication-quality plots for comparison across releases.
