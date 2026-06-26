import numpy as np

from auv_sim_bench.dynamics import AUVDynamics, AUVParams


def test_neutral_buoyancy_hovers():
    sim = AUVDynamics(AUVParams())  # default volume = neutral
    for _ in range(2000):
        sim.step(0.005, [0, 0, 0], [0, 0, 0])
    assert abs(sim.s.p[2]) < 0.05  # stays near its starting depth


def test_accelerometer_reads_gravity_at_rest():
    sim = AUVDynamics(AUVParams())
    sim.step(0.005, [0, 0, 0], [0, 0, 0])
    # level, neutrally buoyant: specific force is +g on body z
    assert sim.specific_force[2] > 9.0
    assert abs(sim.specific_force[0]) < 0.1
    assert abs(sim.specific_force[1]) < 0.1


def test_negative_buoyancy_sinks():
    p = AUVParams()
    p.volume = p.mass / p.water_density * 0.9  # under-displaced -> heavy
    sim = AUVDynamics(p)
    for _ in range(2000):
        sim.step(0.005, [0, 0, 0], [0, 0, 0])
    assert sim.s.p[2] < -0.1  # sinks


def test_forward_thrust_moves_and_drag_limits_speed():
    sim = AUVDynamics(AUVParams())
    for _ in range(4000):
        sim.step(0.005, [40, 0, 0], [0, 0, 0])  # steady forward force
    assert sim.s.p[0] > 0.5      # moved forward
    assert sim.s.v[0] < 5.0      # drag bounded the speed


def test_current_pushes_passive_vehicle():
    p = AUVParams()
    p.current_world = np.array([0.5, 0.0, 0.0])  # 0.5 m/s current along +x
    sim = AUVDynamics(p)
    for _ in range(4000):
        sim.step(0.005, [0, 0, 0], [0, 0, 0])    # no thrust
    assert sim.s.p[0] > 0.2      # carried downstream
    assert sim.s.v[0] > 0.1      # drifting toward the water velocity
