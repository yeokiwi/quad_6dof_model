"""Top-level simulation loop wiring dynamics, controller, waypoints and telemetry."""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .controller import CascadedController, ControllerGains
from .coordinates import GeodeticNEDConverter, GeoPoint
from .dynamics import QuadDynamics, QuadParams, QuadState
from .telemetry import TelemetryPacket, UDPTelemetrySender
from .waypoints import WaypointManager


@dataclass
class SimConfig:
    dt: float = 0.005          # 200 Hz physics
    telem_rate_hz: float = 50.0
    realtime: bool = True
    duration_s: float | None = None
    udp_host: str = "127.0.0.1"
    udp_port: int = 14550
    accept_radius: float = 2.0


class Simulator:
    def __init__(self,
                 home: GeoPoint,
                 waypoints_geo: list[GeoPoint],
                 config: SimConfig | None = None,
                 quad_params: QuadParams | None = None,
                 gains: ControllerGains | None = None):
        self.cfg = config or SimConfig()
        self.params = quad_params or QuadParams()
        self.dynamics = QuadDynamics(self.params)
        self.controller = CascadedController(self.params, gains)
        self.converter = GeodeticNEDConverter(home)
        self.wpm = WaypointManager(waypoints_geo, self.converter,
                                   accept_radius=self.cfg.accept_radius)
        self.state = QuadState()
        self.t = 0.0
        self.sender = UDPTelemetrySender(self.cfg.udp_host, self.cfg.udp_port)

    def _build_packet(self, u: np.ndarray) -> TelemetryPacket:
        n, e, d = self.state.pos
        geo = self.converter.ned_to_geo(n, e, d)
        target = self.wpm.current()
        return TelemetryPacket(
            t=self.t,
            lat=geo.lat, lon=geo.lon, alt=geo.alt,
            north=float(n), east=float(e), down=float(d),
            vn=float(self.state.vel[0]),
            ve=float(self.state.vel[1]),
            vd=float(self.state.vel[2]),
            roll=float(self.state.euler[0]),
            pitch=float(self.state.euler[1]),
            yaw=float(self.state.euler[2]),
            p=float(self.state.omega[0]),
            q=float(self.state.omega[1]),
            r=float(self.state.omega[2]),
            thrust=float(u[0]),
            wp_index=self.wpm.index,
            wp_lat=target.geo.lat,
            wp_lon=target.geo.lon,
            wp_alt=target.geo.alt,
        )

    def run(self):
        dt = self.cfg.dt
        telem_period = 1.0 / self.cfg.telem_rate_hz
        next_telem = 0.0
        wall_start = time.perf_counter()
        last_u = np.zeros(4)
        try:
            while True:
                target = self.wpm.update(self.state.pos)
                u = self.controller.compute(self.state, target.ned, dt)
                self.state = self.dynamics.step(self.state, u, dt)
                self.t += dt
                last_u = u

                if self.t >= next_telem:
                    self.sender.send(self._build_packet(u))
                    next_telem += telem_period

                if self.cfg.duration_s is not None and self.t >= self.cfg.duration_s:
                    break

                if self.cfg.realtime:
                    target_wall = wall_start + self.t
                    sleep = target_wall - time.perf_counter()
                    if sleep > 0:
                        time.sleep(sleep)
        finally:
            # Final telemetry burst so listeners see the terminal state.
            self.sender.send(self._build_packet(last_u))
            self.sender.close()
