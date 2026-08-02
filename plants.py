"""
plants.py

A small library of plant models to tune a PID controller against.
Each plant exposes:
  - state: list/array of current state variables
  - output(): the measured (controlled) variable
  - step(u, dt, disturbance=0.0): advance the plant by dt given control input u
  - name, unit, control_unit, description: metadata for the UI
  - default_pid: a reasonable starting point for Kp/Ki/Kd
  - default_setpoint, output_min/max for plotting

All plants are simulated with simple RK4 integration of their ODEs, which
is accurate enough for tuning/education purposes without needing scipy.
"""

import math


def _rk4_step(f, x, u, dt):
    """Generic RK4 integrator for state x, input u, dynamics f(x, u) -> dx/dt."""
    k1 = f(x, u)
    k2 = f([xi + dt / 2 * ki for xi, ki in zip(x, k1)], u)
    k3 = f([xi + dt / 2 * ki for xi, ki in zip(x, k2)], u)
    k4 = f([xi + dt * ki for xi, ki in zip(x, k3)], u)
    return [
        xi + (dt / 6) * (k1i + 2 * k2i + 2 * k3i + k4i)
        for xi, k1i, k2i, k3i, k4i in zip(x, k1, k2, k3, k4)
    ]


class DCMotorSpeed:
    """
    First-order DC motor speed model (electrical dynamics neglected):
        J * dw/dt = Kt * u - b * w - T_load
    Control input u is armature voltage-equivalent effort (already scaled by Kt/R internally),
    output is angular velocity w (rad/s).
    """

    name = "DC Motor (Speed Control)"
    control_label = "Voltage effort"
    output_label = "Angular velocity"
    output_unit = "rad/s"
    default_setpoint = 100.0
    output_range = (0, 160)
    default_pid = {"kp": 0.8, "ki": 4.0, "kd": 0.02}

    def __init__(self, J=0.02, b=0.2, Kt=1.0, load_torque=0.0):
        self.J = J
        self.b = b
        self.Kt = Kt
        self.load_torque = load_torque
        self.state = [0.0]  # [angular velocity]

    def _dynamics(self, x, u):
        w = x[0]
        dwdt = (self.Kt * u - self.b * w - self.load_torque) / self.J
        return [dwdt]

    def output(self):
        return self.state[0]

    def step(self, u, dt, disturbance=0.0):
        self.load_torque_effective = self.load_torque + disturbance
        old_load = self.load_torque
        self.load_torque = self.load_torque + disturbance
        self.state = _rk4_step(self._dynamics, self.state, u, dt)
        self.load_torque = old_load
        return self.output()


class ThermalSystem:
    """
    First-order thermal system (e.g. a heater controlling temperature):
        tau * dT/dt = -(T - T_ambient) + K * u
    """

    name = "Thermal System (Heater)"
    control_label = "Heater power"
    output_label = "Temperature"
    output_unit = "°C"
    default_setpoint = 60.0
    output_range = (15, 100)
    default_pid = {"kp": 2.5, "ki": 0.15, "kd": 1.0}

    def __init__(self, tau=25.0, K=2.0, ambient=20.0):
        self.tau = tau
        self.K = K
        self.ambient = ambient
        self.state = [ambient]  # [temperature]

    def _dynamics(self, x, u):
        T = x[0]
        dTdt = (-(T - self.ambient) + self.K * u) / self.tau
        return [dTdt]

    def output(self):
        return self.state[0]

    def step(self, u, dt, disturbance=0.0):
        # disturbance modeled as an ambient temperature shift (e.g. a door opening)
        old_ambient = self.ambient
        self.ambient = self.ambient + disturbance
        self.state = _rk4_step(self._dynamics, self.state, u, dt)
        self.ambient = old_ambient
        return self.output()


class InvertedPendulum:
    """
    Linearized inverted pendulum on a cart, controlling pendulum angle (theta,
    radians from upright) via cart force. This is the classic unstable,
    non-trivial-to-tune benchmark plant.

    State: [theta, theta_dot]
    Linearized about theta = 0 (upright):
        theta_ddot = (g / L) * theta + (u / (M * L))
    (Simplified single-state-pair model; M is cart mass, L pendulum length.
    Sign is chosen so that a standard PID, driven by error = setpoint -
    theta, produces a stabilizing control effort -- i.e. positive Kp
    directly counteracts the fall instead of accelerating it.)
    """

    name = "Inverted Pendulum (Angle Control)"
    control_label = "Cart force"
    output_label = "Pendulum angle"
    output_unit = "rad"
    default_setpoint = 0.0
    output_range = (-0.5, 0.5)
    default_pid = {"kp": 45.0, "ki": 5.0, "kd": 12.0}

    def __init__(self, M=1.0, L=0.5, g=9.81, initial_angle=0.15):
        self.M = M
        self.L = L
        self.g = g
        self.state = [initial_angle, 0.0]  # [theta, theta_dot]

    def _dynamics(self, x, u):
        theta, theta_dot = x
        theta_ddot = (self.g / self.L) * theta + (u / (self.M * self.L))
        return [theta_dot, theta_ddot]

    def output(self):
        return self.state[0]

    def step(self, u, dt, disturbance=0.0):
        # disturbance modeled as an impulsive push (added torque) on theta_ddot
        self._external_disturbance = disturbance
        old_state = self.state
        self.state = _rk4_step(self._dynamics, self.state, u, dt)
        # apply disturbance as a direct angular-velocity kick, scaled small
        self.state[1] += disturbance * dt
        return self.output()


PLANT_REGISTRY = {
    "dc_motor": DCMotorSpeed,
    "thermal": ThermalSystem,
    "pendulum": InvertedPendulum,
}
