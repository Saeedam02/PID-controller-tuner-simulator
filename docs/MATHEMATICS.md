# Mathematical Notes

## 1. PID Law

The controller uses the parallel form

$$
u(t)
====

K_p e(t)
+
K_i\int e(t),dt
---------------

K_d\dot{y}_f(t)
$$

where $e=r-y$. The derivative is taken on the measurement rather than the error. For a set-point step with unchanged measurement, this prevents a direct derivative kick.

The derivative estimate is passed through a first-order low-pass filter. In the discrete implementation,

$$
\alpha
======

\frac{\Delta t}{\tau_f+\Delta t}
$$

and

$$
\dot{y}_f[k]
============

\dot{y}_f[k-1]
+
\alpha
\left(
\dot{y}[k]-\dot{y}_f[k-1]
\right)
$$

where $\tau_f$ is the derivative-filter time constant and $\Delta t$ is the simulation timestep.

---

## 2. Conditional Anti-Windup

The controller first computes a candidate integral update. If the corresponding control command exceeds an actuator limit, the integral update is rejected only when its increment would push the controller farther into the active saturation direction.

If the integral increment instead opposes the saturation direction, the update is accepted so that the controller can unwind.

Conceptually, the unsaturated PID command is

$$
u_{\mathrm{raw}}
================

K_p e
+
K_i I
-----

K_d\dot{y}_f
$$

while the actuator applies

$$
u
=

\mathrm{sat}
\left(
u_{\mathrm{raw}},
u_{\min},
u_{\max}
\right)
$$

where $I$ denotes the controller's integral state.

This conditional-integration strategy is more accurate than unconditional "freeze the integrator whenever saturated" clamping because it still permits the integral state to move in a direction that helps the controller leave saturation.

---

## 3. Plant Models

### 3.1 DC Motor Speed

The simplified motor-speed model is

$$
J\dot{\omega}
=============

## K_tu

## b\omega

## T_{\mathrm{load}}

d
$$

where:

* $J$ is the equivalent rotational inertia,
* $\omega$ is the motor angular speed,
* $K_t$ is the effective torque constant,
* $u$ is the control input,
* $b$ is the viscous damping coefficient,
* $T_{\mathrm{load}}$ is the nominal load torque,
* $d$ represents an additional disturbance torque.

Electrical dynamics are neglected. The model is therefore a teaching-scale first-order approximation of the mechanical motor dynamics.

### 3.2 Thermal Process

The thermal process is represented by

$$
\tau\dot{T}
===========

-(T-T_a)
+
Ku
+
d
$$

where:

* $T$ is the process temperature,
* $T_a$ is the ambient temperature,
* $\tau$ is the thermal time constant,
* $K$ is the process gain,
* $u$ is the heater control input,
* $d$ represents an external thermal disturbance.

This model represents a standard first-order thermal process.

### 3.3 Inverted Pendulum

The simplified linearized pendulum-angle model is

$$
\ddot{\theta}
=============

\frac{g}{L}\theta
+
\frac{u}{ML}
+
\frac{d}{ML}
$$

where:

* $\theta$ is the pendulum angle measured from the upright equilibrium,
* $g$ is gravitational acceleration,
* $L$ is the effective pendulum length,
* $M$ is the effective mass parameter,
* $u$ is the control input,
* $d$ is an external disturbance.

The positive gravity term

$$
\frac{g}{L}\theta
$$

causes deviations from $\theta=0$ to grow in open loop. Therefore, the upright equilibrium is open-loop unstable.

---

## 4. FOPDT Identification

Stable open-loop responses are approximated by a First-Order Plus Dead-Time model,

$$
G(s)
====

\frac{K e^{-Ls}}{\tau s+1}
$$

where:

* $K$ is the steady-state process gain,
* $L$ is the effective dead time,
* $\tau$ is the process time constant.

The implementation uses the 28.3%/63.2% two-point reaction-curve approximation.

For a normalized first-order step response, the characteristic times corresponding to approximately 28.3% and 63.2% of the final response are used to estimate $L$ and $\tau$.

The implementation also checks whether the tail of the simulated response has approximately settled before accepting the identified FOPDT model. This reduces the chance of treating an unstable or insufficiently settled trajectory as a valid first-order process.

---

## 5. Lambda-Tuned PI Starting Point

For an identified FOPDT model, the simulator provides the transparent PI starting rule

$$
K_p
===

\frac{\tau}
{K(\lambda+L)}
$$

$$
K_i
===

\frac{K_p}{\tau}
$$

$$
K_d
===

0
$$

where $\lambda$ is the desired closed-loop response-time parameter.

A larger $\lambda$ generally produces a more conservative response, while a smaller $\lambda$ requests a faster response and may require more aggressive control effort.

The default value used by the simulator is intentionally conservative.

The resulting gains should therefore be interpreted as a practical starting point for closed-loop evaluation rather than as an optimality, stability-margin, or robustness guarantee.

---

## 6. Ziegler-Nichols Reaction-Curve PID

When the identified process contains a meaningful positive dead time, the classical Ziegler-Nichols reaction-curve PID rule is

$$
K_p
===

\frac{1.2\tau}{KL}
$$

$$
T_i
===

2L
$$

$$
T_d
===

0.5L
$$

The parallel-form integral and derivative gains are then obtained from

$$
K_i
===

\frac{K_p}{T_i}
$$

and

$$
K_d
===

K_pT_d
$$

Therefore,

$$
K_i
===

\frac{K_p}{2L}
$$

and

$$
K_d
===

0.5K_pL
$$

Because the proportional-gain formula contains $L$ in the denominator,

$$
K_p
===

\frac{1.2\tau}{KL}
$$

the rule becomes singular as

$$
L\rightarrow0
$$

For that reason, version 2 refuses to apply the Ziegler-Nichols reaction-curve rule when the identified delay is negligible compared with the process time constant.

This prevents unrealistically large controller gains from being generated for processes that are effectively delay-free.

---

## 7. Applicability of the Auto-Tuning Methods

The open-loop FOPDT-based tuning procedures are intended only for plants that produce a stable and sufficiently settled open-loop step response.

They are therefore appropriate for systems such as the simplified DC motor and thermal process when their simulated responses satisfy the identification checks.

The inverted pendulum is different because its upright equilibrium is open-loop unstable. Its response cannot, in general, be meaningfully approximated using the same stable FOPDT reaction-curve procedure.

For this reason, the simulator explicitly prevents the open-loop FOPDT auto-tuning workflow from being applied to the inverted-pendulum model.

The pendulum can still be studied using manually selected feedback gains, but a dedicated unstable-system tuning or state-space control method would be required for a more systematic design.

---

## 8. Performance Metrics

The simulator evaluates closed-loop behavior using several complementary performance measures.

### Integral Absolute Error

$$
\mathrm{IAE}
============

\int_0^{t_f}
|e(t)|,dt
$$

IAE measures the accumulated absolute tracking error.

### Integral Squared Error

$$
\mathrm{ISE}
============

\int_0^{t_f}
e^2(t),dt
$$

ISE penalizes large errors more strongly because the error is squared.

### Integral Time-Weighted Absolute Error

$$
\mathrm{ITAE}
=============

\int_0^{t_f}
t|e(t)|,dt
$$

ITAE gives increasing weight to errors that persist later in the simulation.

### Root-Mean-Square Error

For discrete simulation data,

$$
e_{\mathrm{RMS}}
================

\sqrt{
\frac{1}{N}
\sum_{k=1}^{N}
e_k^2
}
$$

This provides a compact measure of the typical tracking-error magnitude.

### Root-Mean-Square Control Effort

$$
u_{\mathrm{RMS}}
================

\sqrt{
\frac{1}{N}
\sum_{k=1}^{N}
u_k^2
}
$$

This provides a simple indication of how aggressively the actuator is being used.

### Control Total Variation

A discrete measure of control activity is

$$
\mathrm{TV}_u
=============

\sum_{k=1}^{N-1}
|u_{k+1}-u_k|
$$

Large values indicate rapidly changing or highly active control commands.

These metrics should be interpreted together rather than individually. A controller with very small tracking error, for example, may achieve that result only through excessive actuator activity or prolonged saturation.

---

## 9. Sensor Noise and Measurement Filtering

When measurement noise is enabled, the controller does not receive the exact plant output directly.

A noisy measurement can be represented conceptually as

$$
y_m[k]
======

y[k]
+
n[k]
$$

where $n[k]$ is the simulated measurement noise.

If an additional first-order measurement filter is enabled, the filtered measurement can be written as

$$
y_f[k]
======

y_f[k-1]
+
\beta
\left(
y_m[k]-y_f[k-1]
\right)
$$

with

$$
\beta
=====

\frac{\Delta t}
{\tau_m+\Delta t}
$$

where $\tau_m$ is the measurement-filter time constant.

Filtering can reduce high-frequency measurement noise, although excessive filtering introduces additional lag into the feedback loop.

---

## 10. Disturbance Models

The simulator can apply disturbances using different time profiles.

A step disturbance can be represented as

$$
d(t)
====

\begin{cases}
0, & t<t_d \
A_d, & t\ge t_d
\end{cases}
$$

where $A_d$ is the disturbance magnitude and $t_d$ is the disturbance start time.

A finite-duration pulse can be represented as

$$
d(t)
====

\begin{cases}
A_d,
&
t_d\le t<t_d+T_d
\
0,
&
\text{otherwise}
\end{cases}
$$

where $T_d$ is the pulse duration.

An impulse-like disturbance in a numerical simulation is represented over a short finite sampling interval rather than as an ideal mathematical Dirac impulse.

These disturbance options make it possible to evaluate both reference tracking and disturbance rejection.

---

## 11. Scope and Limitations

All plant models, controllers, identification procedures, and tuning rules in this repository are intended for educational simulation and controller-design experiments.

Unless explicitly stated otherwise, the models do not account for effects such as:

* hardware communication latency,
* actuator transport delays,
* sensor quantization,
* encoder resolution,
* nonlinear actuator saturation physics,
* actuator rate limits,
* backlash and mechanical hysteresis,
* unmodelled high-frequency plant dynamics,
* parameter uncertainty,
* time-varying plant parameters,
* sensor bias or sensor faults,
* packet loss,
* stochastic process disturbances,
* formal gain and phase margins,
* formal robust-stability guarantees.

Consequently, successful simulation results should not be interpreted as evidence that the same gains can be transferred directly to physical hardware.

A real implementation should normally include system identification using experimental data, actuator and sensor characterization, uncertainty analysis, stability-margin evaluation, and carefully supervised hardware testing.
