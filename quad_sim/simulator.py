"""Top-level simulation loop wiring dynamics, controller, waypoints and telemetry.

A separate UDP command listener thread accepts ``reset`` and ``load_mission``
messages from the GCS at runtime; commands are drained between physics steps
in the main loop, so updates are atomic with respect to integration.
"""
from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass

import numpy as np

from .commands import CommandReceiver
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
    cmd_host: str = "0.0.0.0"
    cmd_port: int = 14551
    cmd_enabled: bool = True


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
        self._wall_start = 0.0
        self._next_telem_t = 0.0
        self.sender = UDPTelemetrySender(self.cfg.udp_host, self.cfg.udp_port)

        self._cmd_queue: queue.Queue = queue.Queue()
        self._cmd_stop = threading.Event()
        self._cmd_thread: threading.Thread | None = None
        self._cmd_rx: CommandReceiver | None = None
        if self.cfg.cmd_enabled:
            self._start_command_listener()

    # ----- Command listener -----
    def _start_command_listener(self) -> None:
        self._cmd_rx = CommandReceiver(self.cfg.cmd_host, self.cfg.cmd_port,
                                       timeout=0.2)

        def loop():
            assert self._cmd_rx is not None
            while not self._cmd_stop.is_set():
                msg = self._cmd_rx.recv()
                if msg is not None:
                    self._cmd_queue.put(msg)

        self._cmd_thread = threading.Thread(target=loop, daemon=True)
        self._cmd_thread.start()

    def _stop_command_listener(self) -> None:
        self._cmd_stop.set()
        if self._cmd_thread is not None:
            self._cmd_thread.join(timeout=1.0)
        if self._cmd_rx is not None:
            self._cmd_rx.close()

    def _drain_commands(self) -> None:
        while True:
            try:
                msg = self._cmd_queue.get_nowait()
            except queue.Empty:
                return
            try:
                self._apply_command(msg)
            except Exception as e:  # bad payloads must not kill the sim
                print(f"[sim] ignoring bad command: {e}")

    def _apply_command(self, msg: dict) -> None:
        typ = msg.get("type")
        if typ == "reset":
            self._reset_vehicle()
            print(f"[sim] reset (replaying {len(self.wpm.waypoints)} waypoints)")
        elif typ == "load_mission":
            wp_dicts = msg.get("waypoints") or []
            wps = [GeoPoint(**w) for w in wp_dicts]
            if not wps:
                raise ValueError("load_mission has no waypoints")
            home_dict = msg.get("home")
            if home_dict is not None:
                self.converter = GeodeticNEDConverter(GeoPoint(**home_dict))
            self.wpm = WaypointManager(wps, self.converter,
                                       accept_radius=self.cfg.accept_radius)
            self._reset_vehicle()
            print(f"[sim] new mission loaded ({len(wps)} waypoints)")
        else:
            raise ValueError(f"unknown command type: {typ!r}")

    def _reset_vehicle(self) -> None:
        # Rebuild waypoint manager so we replay from index 0 with the same plan.
        if self.wpm.waypoints:
            self.wpm = WaypointManager(
                [w.geo for w in self.wpm.waypoints],
                self.converter,
                accept_radius=self.cfg.accept_radius,
            )
        self.state = QuadState()
        self.controller.reset()
        self.t = 0.0
        self._next_telem_t = 0.0
        self._wall_start = time.perf_counter()

    # ----- Telemetry packing -----
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

    # ----- Main loop -----
    def run(self) -> None:
        dt = self.cfg.dt
        telem_period = 1.0 / self.cfg.telem_rate_hz
        self._next_telem_t = 0.0
        self._wall_start = time.perf_counter()
        last_u = np.zeros(4)
        try:
            while True:
                self._drain_commands()

                target = self.wpm.update(self.state.pos)
                u = self.controller.compute(self.state, target.ned, dt)
                self.state = self.dynamics.step(self.state, u, dt)
                self.t += dt
                last_u = u

                if self.t >= self._next_telem_t:
                    self.sender.send(self._build_packet(u))
                    self._next_telem_t += telem_period
                    # If a reset jumped time backwards, snap the cursor forward
                    # rather than blasting a backlog of packets.
                    if self._next_telem_t <= self.t:
                        self._next_telem_t = self.t + telem_period

                if self.cfg.duration_s is not None and self.t >= self.cfg.duration_s:
                    break

                if self.cfg.realtime:
                    target_wall = self._wall_start + self.t
                    sleep = target_wall - time.perf_counter()
                    if sleep > 0:
                        time.sleep(sleep)
        finally:
            try:
                self.sender.send(self._build_packet(last_u))
            finally:
                self.sender.close()
                self._stop_command_listener()
