"""6-DOF rigid-body AUV dynamics (pure numpy, no ROS).

Body frame is FLU, world frame is ENU (z up). State is integrated in the body
frame with Newton-Euler; orientation as a wxyz quaternion.
"""
import numpy as np


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_normalize(q):
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0.0, 0.0, 0.0])


def quat_to_rot(q):
    """Body -> world rotation matrix for a wxyz quaternion."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class AUVParams:
    def __init__(self):
        self.mass = 15.0                 # kg
        self.water_density = 1025.0      # kg/m^3
        self.gravity = 9.81              # m/s^2
        # displaced volume; rho*V*g should be ~ weight for neutral buoyancy
        self.volume = (self.mass) / self.water_density   # exactly neutral by default
        self.inertia = np.array([0.5, 0.8, 0.8])         # diag Ixx,Iyy,Izz
        # center of buoyancy above the center of mass (body frame) -> righting moment
        self.cob_offset = np.array([0.0, 0.0, 0.02])
        # drag: F = -(lin * v + quad * v*|v|), per body axis
        self.lin_drag = np.array([20.0, 40.0, 40.0])
        self.quad_drag = np.array([30.0, 80.0, 80.0])
        self.ang_drag = np.array([2.0, 4.0, 4.0])
        self.ang_quad_drag = np.array([1.0, 2.0, 2.0])
        # water current in the world frame (m/s); drag acts on velocity
        # relative to the water, so a current pushes a passive vehicle.
        self.current_world = np.zeros(3)


class AUVState:
    def __init__(self):
        self.p = np.zeros(3)                    # world position (ENU)
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) # world orientation (wxyz)
        self.v = np.zeros(3)                    # body-frame linear velocity
        self.w = np.zeros(3)                    # body-frame angular velocity


class AUVDynamics:
    def __init__(self, params=None, state=None):
        self.p = params or AUVParams()
        self.s = state or AUVState()
        self.specific_force = np.zeros(3)  # what the accelerometer would read (body)

    def step(self, dt, force_body, torque_body):
        p, s = self.p, self.s
        R = quat_to_rot(s.q)
        Rt = R.T

        g_body = Rt @ np.array([0.0, 0.0, -p.mass * p.gravity])
        b_body = Rt @ np.array([0.0, 0.0, p.water_density * p.volume * p.gravity])

        v_rel = s.v - Rt @ p.current_world  # velocity relative to the water
        drag_f = -(p.lin_drag * v_rel + p.quad_drag * v_rel * np.abs(v_rel))
        drag_t = -(p.ang_drag * s.w + p.ang_quad_drag * s.w * np.abs(s.w))

        F = np.asarray(force_body, float) + g_body + b_body + drag_f
        T = np.asarray(torque_body, float) + np.cross(p.cob_offset, b_body) + drag_t

        v_dot = F / p.mass - np.cross(s.w, s.v)
        w_dot = (T - np.cross(s.w, p.inertia * s.w)) / p.inertia

        s.v = s.v + v_dot * dt
        s.w = s.w + w_dot * dt
        s.q = quat_normalize(s.q + 0.5 * quat_mul(s.q, np.array([0.0, *s.w])) * dt)
        s.p = s.p + R @ s.v * dt

        # accelerometer specific force = (total - gravity) / m, in body frame
        self.specific_force = (F - g_body) / p.mass
        return s
