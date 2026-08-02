"""
simulation.py

Runs a closed-loop simulation of a PIDController against a plant, over a
fixed time horizon, with an optional step disturbance injected partway
through. Returns time series suitable for plotting and metric computation.
"""

from controller import PIDController, PIDConfig


def run_simulation(
    plant,
    kp: float,
    ki: float,
    kd: float,
    setpoint: float,
    duration: float = 10.0,
    dt: float = 0.02,
    disturbance_time: float = None,
    disturbance_magnitude: float = 0.0,
    output_limits=(-1e9, 1e9),
):
    """
    plant: an instantiated plant object from plants.py
    Returns a dict of parallel lists: t, setpoint, output, control, p, i, d
    """
    cfg = PIDConfig(
        kp=kp,
        ki=ki,
        kd=kd,
        output_min=output_limits[0],
        output_max=output_limits[1],
    )
    pid = PIDController(cfg)

    n_steps = int(duration / dt)
    t_list, sp_list, y_list, u_list = [], [], [], []
    p_list, i_list, d_list = [], [], []

    for step_idx in range(n_steps):
        t = step_idx * dt
        measurement = plant.output()

        result = pid.step(setpoint=setpoint, measurement=measurement, dt=dt)
        u = result["output"]

        disturbance = 0.0
        if disturbance_time is not None and t >= disturbance_time:
            # apply as a one-shot impulse right at/after the disturbance time
            if t < disturbance_time + dt:
                disturbance = disturbance_magnitude

        plant.step(u, dt, disturbance=disturbance)

        t_list.append(t)
        sp_list.append(setpoint)
        y_list.append(measurement)
        u_list.append(u)
        p_list.append(result["p"])
        i_list.append(result["i"])
        d_list.append(result["d"])

    return {
        "t": t_list,
        "setpoint": sp_list,
        "output": y_list,
        "control": u_list,
        "p": p_list,
        "i": i_list,
        "d": d_list,
    }
