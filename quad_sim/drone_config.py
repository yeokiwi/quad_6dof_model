"""Load a drone parameter file (drone.json) and apply it to QuadParams /
ControllerGains.

Schema (every field is optional; omitted fields keep their built-in default):

    {
      "max_speed":         8.0,    // max horizontal speed [m/s]
      "max_climb_rate":    3.0,    // max vertical speed   [m/s]
      "max_roll_rate_deg": 180.0,  // body roll rate limit [deg/s]
      "max_pitch_rate_deg":180.0,  // body pitch rate limit [deg/s]
      "max_yaw_rate_deg":  120.0,  // body yaw rate limit  [deg/s]
      "max_tilt_deg":      30.0,   // commanded tilt limit [deg]
      "mass_kg":           1.2,
      "max_thrust_n":      39.24,
      "drag_lin":          0.10,
      "inertia": {"Ixx": 0.012, "Iyy": 0.012, "Izz": 0.022}
    }
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from .controller import ControllerGains
from .dynamics import QuadParams


def load_drone_config(path: str | Path) -> dict:
    return json.loads(Path(path).read_text())


def apply_drone_config(params: QuadParams, gains: ControllerGains, cfg: dict) -> None:
    """Mutate ``params`` and ``gains`` in place using fields from ``cfg``.

    Unknown keys are ignored (with a warning) so config files can carry extra
    metadata without breaking the loader.
    """
    known = set()

    def take(key: str):
        known.add(key)
        return cfg.get(key)

    if (v := take("max_speed")) is not None:
        gains.max_vel_xy = float(v)
    if (v := take("max_climb_rate")) is not None:
        gains.max_vel_z = float(v)
    if (v := take("max_roll_rate_deg")) is not None:
        gains.max_roll_rate = math.radians(float(v))
    if (v := take("max_pitch_rate_deg")) is not None:
        gains.max_pitch_rate = math.radians(float(v))
    if (v := take("max_yaw_rate_deg")) is not None:
        gains.max_yaw_rate = math.radians(float(v))
    if (v := take("max_tilt_deg")) is not None:
        gains.max_tilt = math.radians(float(v))
    if (v := take("mass_kg")) is not None:
        params.mass = float(v)
    if (v := take("max_thrust_n")) is not None:
        params.max_thrust = float(v)
    if (v := take("drag_lin")) is not None:
        params.drag_lin = float(v)
    if (inertia := take("inertia")) is not None:
        if "Ixx" in inertia: params.Ixx = float(inertia["Ixx"])
        if "Iyy" in inertia: params.Iyy = float(inertia["Iyy"])
        if "Izz" in inertia: params.Izz = float(inertia["Izz"])

    extras = set(cfg.keys()) - known
    if extras:
        print(f"[drone_config] ignoring unknown fields: {sorted(extras)}")
