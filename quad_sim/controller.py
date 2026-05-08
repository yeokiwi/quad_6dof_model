"""Cascaded PID controller for autonomous waypoint flight.

Loops (slowest to fastest):
  1. Position (P, on NED error)        -> velocity setpoint
  2. Velocity (PI)                     -> desired NED acceleration
  3. From desired accel:
       - desired thrust along -z_body
       - desired roll, pitch (small-angle mapping rotated by yaw)
  4. Attitude (P)                      -> body rate setpoint
  5. Body rate (PD)                    -> body torques

The yaw setpoint is held to face the next waypoint when horizontal travel
exceeds a small threshold; otherwise yaw is held at its current value.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dynamics import GRAVITY, QuadParams, QuadState


@dataclass
class ControllerGains:
    # Position -> velocity
    kp_pos_xy: float = 1.0
    kp_pos_z: float = 1.5
    # Velocity -> acceleration
    kp_vel_xy: float = 2.0
    ki_vel_xy: float = 0.2
    kp_vel_z: float = 4.0
    ki_vel_z: float = 0.5
    # Attitude -> body rate
    kp_att_rp: float = 6.0
    kp_att_yaw: float = 2.5
    # Body rate -> torque
    kp_rate_rp: float = 0.10
    kd_rate_rp: float = 0.005
    kp_rate_yaw: float = 0.20
    kd_rate_yaw: float = 0.010

    # Limits
    max_vel_xy: float = 8.0     # m/s
    max_vel_z: float = 3.0      # m/s
    max_accel_xy: float = 6.0   # m/s^2
    max_tilt: float = np.deg2rad(30.0)
    max_rate: float = np.deg2rad(180.0)


class CascadedController:
    def __init__(self, params: QuadParams, gains: ControllerGains | None = None):
        self.p = params
        self.g = gains or ControllerGains()
        self._vel_int = np.zeros(3)
        self._prev_omega_err = np.zeros(3)

    def reset(self):
        self._vel_int[:] = 0.0
        self._prev_omega_err[:] = 0.0

    def compute(self, state: QuadState, target_ned: np.ndarray, dt: float) -> np.ndarray:
        # 1. Position -> velocity
        pos_err = target_ned - state.pos
        vel_set = np.array([
            self.g.kp_pos_xy * pos_err[0],
            self.g.kp_pos_xy * pos_err[1],
            self.g.kp_pos_z  * pos_err[2],
        ])
        vel_set[:2] = _clip_norm(vel_set[:2], self.g.max_vel_xy)
        vel_set[2] = float(np.clip(vel_set[2], -self.g.max_vel_z, self.g.max_vel_z))

        # 2. Velocity -> NED acceleration
        vel_err = vel_set - state.vel
        self._vel_int += vel_err * dt
        # Anti-windup: clamp the integrator
        self._vel_int[:2] = _clip_norm(self._vel_int[:2], 5.0)
        self._vel_int[2] = float(np.clip(self._vel_int[2], -3.0, 3.0))

        accel_des = np.array([
            self.g.kp_vel_xy * vel_err[0] + self.g.ki_vel_xy * self._vel_int[0],
            self.g.kp_vel_xy * vel_err[1] + self.g.ki_vel_xy * self._vel_int[1],
            self.g.kp_vel_z  * vel_err[2] + self.g.ki_vel_z  * self._vel_int[2],
        ])
        accel_des[:2] = _clip_norm(accel_des[:2], self.g.max_accel_xy)

        # 3. Map desired NED accel -> thrust + desired roll/pitch.
        # Required net force in NED (thrust force only): m*(a_des) - m*g_ned.
        # g_ned points +z (down).
        f_ned = self.p.mass * accel_des - np.array([0.0, 0.0, self.p.mass * GRAVITY])
        # Thrust magnitude along -z_body; project onto current -z_body to keep
        # the controller well-behaved while tilted.
        roll, pitch, yaw = state.euler
        # Approx: collective thrust to achieve commanded vertical accel.
        # Using small-angle / current-tilt compensation for vertical.
        cos_tilt = max(np.cos(roll) * np.cos(pitch), 0.5)
        thrust = -f_ned[2] / cos_tilt
        thrust = float(np.clip(thrust, 0.0, self.p.max_thrust))

        # Desired horizontal accel rotated into body-yaw frame
        cy, sy = np.cos(yaw), np.sin(yaw)
        ax_b =  cy * accel_des[0] + sy * accel_des[1]   # body-forward accel
        ay_b = -sy * accel_des[0] + cy * accel_des[1]   # body-right accel
        # Small-angle inversion: accel_x_body = -g*pitch, accel_y_body = g*roll
        pitch_des = float(np.clip(-ax_b / GRAVITY, -self.g.max_tilt, self.g.max_tilt))
        roll_des  = float(np.clip( ay_b / GRAVITY, -self.g.max_tilt, self.g.max_tilt))

        # Yaw setpoint: face the waypoint if horizontal distance is non-trivial.
        horiz = np.hypot(pos_err[0], pos_err[1])
        if horiz > 1.0:
            yaw_des = float(np.arctan2(pos_err[1], pos_err[0]))
        else:
            yaw_des = yaw  # hold

        # 4. Attitude -> body rate
        att_err = np.array([
            _wrap_pi(roll_des - roll),
            _wrap_pi(pitch_des - pitch),
            _wrap_pi(yaw_des - yaw),
        ])
        omega_set = np.array([
            self.g.kp_att_rp  * att_err[0],
            self.g.kp_att_rp  * att_err[1],
            self.g.kp_att_yaw * att_err[2],
        ])
        omega_set = np.clip(omega_set, -self.g.max_rate, self.g.max_rate)

        # 5. Body rate -> torque
        omega_err = omega_set - state.omega
        omega_err_dot = (omega_err - self._prev_omega_err) / max(dt, 1e-3)
        self._prev_omega_err = omega_err
        tau = np.array([
            self.g.kp_rate_rp  * omega_err[0] + self.g.kd_rate_rp  * omega_err_dot[0],
            self.g.kp_rate_rp  * omega_err[1] + self.g.kd_rate_rp  * omega_err_dot[1],
            self.g.kp_rate_yaw * omega_err[2] + self.g.kd_rate_yaw * omega_err_dot[2],
        ])

        return np.array([thrust, tau[0], tau[1], tau[2]])


def _clip_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n > max_norm and n > 0.0:
        return v * (max_norm / n)
    return v


def _wrap_pi(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi
