"""6DOF rigid-body quadcopter dynamics.

State (12): [n, e, d, vn, ve, vd, roll, pitch, yaw, p, q, r]
  - position (NED) [m]
  - velocity (NED) [m/s]
  - Euler angles ZYX intrinsic (roll about body-x, pitch about body-y,
    yaw about body-z) [rad]
  - body angular rates (p, q, r about body x, y, z) [rad/s]

Input (4): [thrust, tau_x, tau_y, tau_z]
  - collective thrust along body -z (i.e. lifts the vehicle when level) [N]
  - body-frame torques [N*m]

The model assumes a diagonal inertia tensor and ignores aerodynamic drag
beyond simple linear translational drag, which is sufficient for control
demonstration.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

GRAVITY = 9.80665


@dataclass
class QuadParams:
    mass: float = 1.2          # kg
    Ixx: float = 0.012         # kg m^2
    Iyy: float = 0.012
    Izz: float = 0.022
    drag_lin: float = 0.10     # linear drag coefficient (N per m/s)
    max_thrust: float = 4.0 * 9.81  # ~4x weight for a 1.2 kg quad
    max_torque: float = 1.0    # per-axis torque limit [N*m]


@dataclass
class QuadState:
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))   # NED
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))   # NED
    euler: np.ndarray = field(default_factory=lambda: np.zeros(3)) # roll, pitch, yaw
    omega: np.ndarray = field(default_factory=lambda: np.zeros(3)) # body p, q, r

    def to_vector(self) -> np.ndarray:
        return np.concatenate([self.pos, self.vel, self.euler, self.omega])

    @classmethod
    def from_vector(cls, x: np.ndarray) -> "QuadState":
        return cls(pos=x[0:3].copy(), vel=x[3:6].copy(),
                   euler=x[6:9].copy(), omega=x[9:12].copy())


def rotation_body_to_ned(euler: np.ndarray) -> np.ndarray:
    """Rotation from body (FRD) to NED frame, R = Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = euler
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ])


def euler_rate_matrix(euler: np.ndarray) -> np.ndarray:
    """Maps body rates [p, q, r] to Euler angle rates [roll_dot, pitch_dot, yaw_dot]."""
    r, p, _ = euler
    cr, sr = np.cos(r), np.sin(r)
    cp = np.cos(p)
    tp = np.tan(p)
    return np.array([
        [1.0, sr*tp, cr*tp],
        [0.0, cr,    -sr],
        [0.0, sr/cp, cr/cp],
    ])


class QuadDynamics:
    def __init__(self, params: QuadParams | None = None):
        self.p = params or QuadParams()
        self.I = np.diag([self.p.Ixx, self.p.Iyy, self.p.Izz])
        self.I_inv = np.linalg.inv(self.I)

    def derivatives(self, state: QuadState, u: np.ndarray) -> np.ndarray:
        """Compute state derivatives given control input u = [T, tau_x, tau_y, tau_z]."""
        T = float(np.clip(u[0], 0.0, self.p.max_thrust))
        tau = np.clip(u[1:4], -self.p.max_torque, self.p.max_torque)

        R = rotation_body_to_ned(state.euler)
        # Thrust acts along -z_body; in NED that is -T * R[:,2]
        f_thrust_ned = -T * R[:, 2]
        f_gravity_ned = np.array([0.0, 0.0, self.p.mass * GRAVITY])
        f_drag_ned = -self.p.drag_lin * state.vel
        accel = (f_thrust_ned + f_gravity_ned + f_drag_ned) / self.p.mass

        euler_dot = euler_rate_matrix(state.euler) @ state.omega
        omega_dot = self.I_inv @ (tau - np.cross(state.omega, self.I @ state.omega))

        return np.concatenate([state.vel, accel, euler_dot, omega_dot])

    def step(self, state: QuadState, u: np.ndarray, dt: float) -> QuadState:
        """RK4 integration over dt."""
        x = state.to_vector()

        def f(xv):
            return self.derivatives(QuadState.from_vector(xv), u)

        k1 = f(x)
        k2 = f(x + 0.5 * dt * k1)
        k3 = f(x + 0.5 * dt * k2)
        k4 = f(x + dt * k3)
        x_next = x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        # Wrap yaw to [-pi, pi] to keep numbers tidy.
        x_next[8] = (x_next[8] + np.pi) % (2 * np.pi) - np.pi
        return QuadState.from_vector(x_next)
