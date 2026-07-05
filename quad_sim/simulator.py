"""Top-level simulation loop wiring dynamics, controller, waypoints and telemetry.

A separate UDP command listener thread accepts ``start``, ``pause``,
``resume``, ``reset``, ``load_mission`` and ``save_trajectory`` messages from
the GCS at runtime; commands are drained between physics steps in the main
loop, so updates are atomic with respect to integration.

Run-state machine: the sim boots in WAITING (unless ``SimConfig.autostart``)
and only steps physics while RUNNING. ``pause`` freezes the state; ``resume``
or ``start`` continues it; ``reset`` / ``load_mission`` return to WAITING.
While not running, a low-rate telemetry heartbeat (wall-clock scheduled) keeps
the GCS informed of the current state and status.

In addition to telemetry, every 10 ms of sim time the simulator appends a row
to an in-memory trajectory buffer. The ``save_trajectory`` command writes the
buffer to a CSV file (in a worker thread so the physics loop is not stalled).
"""
from __future__ import annotations

import csv
import datetime as _dt
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .commands import CommandReceiver
from .controller import CascadedController, ControllerGains
from .coordinates import GeodeticNEDConverter, GeoPoint
from .dynamics import QuadDynamics, QuadParams, QuadState
from .telemetry import TelemetryPacket, UDPTelemetrySender
from .waypoints import WaypointManager

TRAJECTORY_PERIOD_S = 0.010
_CSV_HEADER = [
    "t", "lat", "lon", "alt",
    "north", "east", "down",
    "vn", "ve", "vd",
    "roll_rad", "pitch_rad", "yaw_rad",
    "p", "q", "r",
    "thrust", "wp_index",
]


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
    autostart: bool = False
    idle_telem_rate_hz: float = 5.0
    autosave_trajectory: bool = True
    mcast_ttl: int = 1


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
        self.run_state = "running" if self.cfg.autostart else "waiting"
        self.sender = UDPTelemetrySender(self.cfg.udp_host, self.cfg.udp_port,
                                         mcast_ttl=self.cfg.mcast_ttl)
        self._mission_complete = False
        self._rows_saved = 0  # rows already written by the latest save

        self._cmd_queue: queue.Queue = queue.Queue()
        self._cmd_stop = threading.Event()
        self._cmd_thread: threading.Thread | None = None
        self._cmd_rx: CommandReceiver | None = None
        if self.cfg.cmd_enabled:
            self._start_command_listener()

        # Trajectory log (10 ms cadence). Guarded by a lock because a worker
        # thread may snapshot it during a save while the main loop appends.
        self._traj_log: list[list] = []
        self._traj_lock = threading.Lock()
        self._next_traj_t = 0.0

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
        if typ == "start":
            if self.run_state in ("waiting", "paused"):
                self._resume_clock()
                self.run_state = "running"
                print("[sim] flight started")
        elif typ == "pause":
            if self.run_state == "running":
                self.run_state = "paused"
                print(f"[sim] paused at t={self.t:.2f}s")
        elif typ == "resume":
            if self.run_state == "paused":
                self._resume_clock()
                self.run_state = "running"
                print(f"[sim] resumed at t={self.t:.2f}s")
        elif typ == "reset":
            self._reset_vehicle()
            self.run_state = "waiting"
            print(f"[sim] reset; waiting for start "
                  f"({len(self.wpm.waypoints)} waypoints)")
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
            self.run_state = "waiting"
            print(f"[sim] new mission loaded ({len(wps)} waypoints); "
                  f"waiting for start")
        elif typ == "save_trajectory":
            self._spawn_save_trajectory(msg.get("path"))
        else:
            raise ValueError(f"unknown command type: {typ!r}")

    def _check_mission_complete(self) -> bool:
        """True when the vehicle has settled at the final waypoint: last
        waypoint active, inside the accept radius, and nearly stationary."""
        if self.wpm.index < len(self.wpm.waypoints) - 1:
            return False
        target = self.wpm.current()
        dist = float(np.linalg.norm(self.state.pos - target.ned))
        speed = float(np.linalg.norm(self.state.vel))
        return dist <= self.cfg.accept_radius and speed < 0.5

    def _resume_clock(self) -> None:
        """Re-anchor the wall clock so real-time pacing continues from the
        current sim time instead of trying to catch up the paused interval."""
        self._wall_start = time.perf_counter() - self.t

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
        self._next_traj_t = 0.0
        self._wall_start = time.perf_counter()
        self._mission_complete = False
        self._rows_saved = 0
        with self._traj_lock:
            self._traj_log.clear()

    # ----- Trajectory log -----
    def _record_trajectory(self, u: np.ndarray) -> None:
        n, e, d = self.state.pos
        geo = self.converter.ned_to_geo(float(n), float(e), float(d))
        row = [
            self.t,
            geo.lat, geo.lon, geo.alt,
            float(n), float(e), float(d),
            float(self.state.vel[0]), float(self.state.vel[1]), float(self.state.vel[2]),
            float(self.state.euler[0]), float(self.state.euler[1]), float(self.state.euler[2]),
            float(self.state.omega[0]), float(self.state.omega[1]), float(self.state.omega[2]),
            float(u[0]),
            self.wpm.index,
        ]
        with self._traj_lock:
            self._traj_log.append(row)

    def _spawn_save_trajectory(self, path: str | None,
                               blocking: bool = False) -> None:
        if not path:
            ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            path = f"trajectory_{ts}.csv"
        with self._traj_lock:
            snapshot = list(self._traj_log)
        if not snapshot:
            print("[sim] trajectory buffer empty; nothing to save")
            return
        n_rows = len(snapshot)
        self._rows_saved = n_rows

        def write():
            try:
                with open(path, "w", newline="") as f:
                    w = csv.writer(f)
                    w.writerow(_CSV_HEADER)
                    w.writerows(snapshot)
                print(f"[sim] saved {n_rows} trajectory rows to {Path(path).resolve()}")
            except OSError as e:
                print(f"[sim] failed to save trajectory: {e}")

        if blocking:
            write()
        else:
            threading.Thread(target=write, daemon=True).start()

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
            status=self.run_state,
        )

    # ----- Main loop -----
    def run(self) -> None:
        dt = self.cfg.dt
        telem_period = 1.0 / self.cfg.telem_rate_hz
        idle_period = 1.0 / self.cfg.idle_telem_rate_hz
        self._next_telem_t = 0.0
        self._next_traj_t = 0.0
        self._wall_start = time.perf_counter()
        next_idle_wall = 0.0
        last_u = np.zeros(4)
        try:
            while True:
                self._drain_commands()

                if self.run_state != "running":
                    # Idle: hold state, heartbeat telemetry at a low rate so
                    # the GCS sees the current status, then sleep briefly.
                    now = time.perf_counter()
                    if now >= next_idle_wall:
                        self.sender.send(self._build_packet(last_u))
                        next_idle_wall = now + idle_period
                    time.sleep(0.02)
                    continue

                target = self.wpm.update(self.state.pos)
                u = self.controller.compute(self.state, target.ned, dt)
                self.state = self.dynamics.step(self.state, u, dt)
                self.t += dt
                last_u = u

                if not self._mission_complete and self._check_mission_complete():
                    self._mission_complete = True
                    print(f"[sim] mission complete at t={self.t:.2f}s")
                    if self.cfg.autosave_trajectory:
                        self._spawn_save_trajectory(None)

                if self.t >= self._next_traj_t:
                    self._record_trajectory(u)
                    self._next_traj_t += TRAJECTORY_PERIOD_S
                    if self._next_traj_t <= self.t:
                        self._next_traj_t = self.t + TRAJECTORY_PERIOD_S

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
                if self.cfg.autosave_trajectory:
                    with self._traj_lock:
                        unsaved = len(self._traj_log) - self._rows_saved
                    if unsaved > 0:
                        # Blocking: must finish before the process exits.
                        self._spawn_save_trajectory(None, blocking=True)
            finally:
                self.sender.close()
                self._stop_command_listener()
