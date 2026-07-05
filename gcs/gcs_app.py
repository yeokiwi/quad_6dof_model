"""Ground control station: UDP telemetry listener + live visualisation.

Layout:
  - Top row: 2D map view | 3D trajectory view | 3D attitude view, plus a
    sidebar with a mission-file picker (radio buttons over *.json files in
    the missions directory) and the current waypoint list (active waypoint
    highlighted).
  - Telemetry text bar (includes the simulator run status).
  - Button row: Start | Stop | Continue | Reset | Save trajectory CSV.

Communication:
  - Telemetry comes from the simulator over UDP (default port 14550).
  - Commands are sent to the simulator over a separate UDP port (default
    14551) using the JSON protocol defined in ``quad_sim.commands``.

The simulator boots in the WAITING state: select a mission (optional) and
press Start to fly. Stop pauses the flight, Continue resumes it, Reset
returns the vehicle to home and back to WAITING.
"""
from __future__ import annotations

import argparse
import json
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, RadioButtons
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from quad_sim.commands import CommandSender
from quad_sim.telemetry import TelemetryPacket, UDPTelemetryReceiver
from quad_sim.waypoints import load_waypoints

MAX_MISSION_FILES = 10


@dataclass
class SharedState:
    latest: TelemetryPacket | None = None
    trail_lat: deque = None
    trail_lon: deque = None
    trail_alt: deque = None
    lock: threading.Lock = None

    def __post_init__(self):
        self.trail_lat = deque(maxlen=4000)
        self.trail_lon = deque(maxlen=4000)
        self.trail_alt = deque(maxlen=4000)
        self.lock = threading.Lock()


def receiver_thread(host: str, port: int, shared: SharedState, stop: threading.Event):
    rx = UDPTelemetryReceiver(host=host, port=port, timeout=0.2)
    try:
        while not stop.is_set():
            pkt = rx.recv()
            if pkt is None:
                continue
            with shared.lock:
                shared.latest = pkt
                # Only extend the trail while flying; idle heartbeats would
                # otherwise pile identical points onto the trail.
                if pkt.status == "running":
                    shared.trail_lat.append(pkt.lat)
                    shared.trail_lon.append(pkt.lon)
                    shared.trail_alt.append(pkt.alt)
    finally:
        rx.close()


def _rotation_body_to_ned(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ])


def scan_mission_files(directory: str | Path) -> list[Path]:
    """Return waypoint-mission JSON files in ``directory`` (sorted by name).

    A file qualifies if it parses as JSON and has a non-empty ``waypoints``
    list; this skips unrelated JSON such as drone.json.
    """
    found = []
    for p in sorted(Path(directory).glob("*.json")):
        try:
            data = json.loads(p.read_text())
        except (OSError, ValueError):
            continue
        if isinstance(data, dict) and isinstance(data.get("waypoints"), list) \
                and data["waypoints"]:
            found.append(p)
    return found[:MAX_MISSION_FILES]


# --- Quadcopter geometry, expressed in the FRD body frame ---------------------
_ARM_LEN = 0.7
_ROTOR_R = 0.18
_BODY_HX, _BODY_HY, _BODY_HZ = 0.14, 0.10, 0.04

_a = _ARM_LEN / np.sqrt(2)
# Order: front-right, front-left, rear-left, rear-right
_ARM_TIPS = np.array([
    [+_a, +_a, 0.0],
    [+_a, -_a, 0.0],
    [-_a, -_a, 0.0],
    [-_a, +_a, 0.0],
])
_ROTOR_COLORS = ["tab:red", "tab:red", "tab:blue", "tab:blue"]  # red = front

_THETA = np.linspace(0.0, 2.0 * np.pi, 28)
_DISK_TEMPLATE = np.stack(
    [_ROTOR_R * np.cos(_THETA), _ROTOR_R * np.sin(_THETA), np.zeros_like(_THETA)],
    axis=1,
)

# Body box vertices (centered at origin, FRD axes)
_BODY_VERTS = np.array([
    [+_BODY_HX, +_BODY_HY, +_BODY_HZ],
    [+_BODY_HX, -_BODY_HY, +_BODY_HZ],
    [-_BODY_HX, -_BODY_HY, +_BODY_HZ],
    [-_BODY_HX, +_BODY_HY, +_BODY_HZ],
    [+_BODY_HX, +_BODY_HY, -_BODY_HZ],
    [+_BODY_HX, -_BODY_HY, -_BODY_HZ],
    [-_BODY_HX, -_BODY_HY, -_BODY_HZ],
    [-_BODY_HX, +_BODY_HY, -_BODY_HZ],
])
_BODY_FACES = [
    [0, 1, 2, 3],
    [4, 5, 6, 7],
    [0, 1, 5, 4],
    [2, 3, 7, 6],
    [0, 3, 7, 4],
    [1, 2, 6, 5],
]


class GCSApp:
    def __init__(self,
                 host: str, port: int,
                 cmd_host: str, cmd_port: int,
                 waypoints_path: str | None,
                 missions_dir: str = "."):
        self.shared = SharedState()
        self.stop = threading.Event()
        self.host = host
        self.port = port
        self.commands = CommandSender(host=cmd_host, port=cmd_port)
        self.cmd_target = (cmd_host, cmd_port)
        self.missions_dir = missions_dir

        self.wp_lat: list[float] = []
        self.wp_lon: list[float] = []
        self.wp_alt: list[float] = []
        self.mission_name = ""
        if waypoints_path and Path(waypoints_path).exists():
            try:
                self._set_waypoints_from_file(waypoints_path)
            except (ValueError, KeyError) as e:
                print(f"[gcs] could not parse {waypoints_path}: {e}")

        self.mission_files = scan_mission_files(missions_dir)

        # ----- Figure layout -----
        self.fig = plt.figure(figsize=(17, 9))
        self.fig.suptitle(
            f"Quadcopter GCS  -  telemetry {host}:{port}  cmd -> {cmd_host}:{cmd_port}"
        )
        gs = self.fig.add_gridspec(
            3, 4, width_ratios=[5, 5, 5, 2.6], height_ratios=[18, 1, 2],
            hspace=0.35, wspace=0.30,
            left=0.05, right=0.98, top=0.92, bottom=0.05,
        )
        self.ax_map = self.fig.add_subplot(gs[0, 0])
        self.ax_traj3d = self.fig.add_subplot(gs[0, 1], projection="3d")
        self.ax_att = self.fig.add_subplot(gs[0, 2], projection="3d")

        side = gs[0, 3].subgridspec(3, 1, height_ratios=[0.22, 1, 1.4],
                                    hspace=0.40)
        self.ax_btn_refresh = self.fig.add_subplot(side[0])
        self.ax_missions = self.fig.add_subplot(side[1])
        self.ax_wplist = self.fig.add_subplot(side[2])

        self.ax_text = self.fig.add_subplot(gs[1, :])
        self.ax_text.axis("off")

        btn_row = gs[2, :].subgridspec(1, 5, wspace=0.25)
        self.ax_btn_start = self.fig.add_subplot(btn_row[0])
        self.ax_btn_stop = self.fig.add_subplot(btn_row[1])
        self.ax_btn_cont = self.fig.add_subplot(btn_row[2])
        self.ax_btn_reset = self.fig.add_subplot(btn_row[3])
        self.ax_btn_save = self.fig.add_subplot(btn_row[4])

        # ----- Panes -----
        self._planned_line = None
        self._planned_labels: list = []
        self._setup_map()
        self._setup_traj3d()
        self._setup_attitude()
        self._setup_missions_panel(waypoints_path)
        self._wplist_artists: list = []
        self._wplist_active = -1
        self._rebuild_wplist()

        self.text_artist = self.ax_text.text(
            0.01, 0.5, "Waiting for telemetry...",
            transform=self.ax_text.transAxes,
            fontsize=10, family="monospace", va="center",
        )
        self._status_msg = ""
        self._status_until_wall: float | None = None

        # ----- Buttons -----
        self.btn_start = Button(self.ax_btn_start, "Start", color="palegreen",
                                hovercolor="lightgreen")
        self.btn_stop = Button(self.ax_btn_stop, "Stop", color="lightsalmon",
                               hovercolor="salmon")
        self.btn_cont = Button(self.ax_btn_cont, "Continue")
        self.btn_reset = Button(self.ax_btn_reset, "Reset")
        self.btn_save = Button(self.ax_btn_save, "Save trajectory CSV")
        self.btn_start.on_clicked(self._on_start)
        self.btn_stop.on_clicked(self._on_stop)
        self.btn_cont.on_clicked(self._on_continue)
        self.btn_reset.on_clicked(self._on_reset)
        self.btn_save.on_clicked(self._on_save_trajectory)

    # ----- Mission helpers -----
    def _set_waypoints_from_file(self, path: str | Path) -> tuple[dict | None, list[dict]]:
        data = json.loads(Path(path).read_text())
        wps = data.get("waypoints") or []
        if not wps:
            raise ValueError(f"{path} has no 'waypoints'")
        self.wp_lat = [w["lat"] for w in wps]
        self.wp_lon = [w["lon"] for w in wps]
        self.wp_alt = [w.get("alt", 0.0) for w in wps]
        self.mission_name = Path(path).name
        return data.get("home"), wps

    # ----- Map (2D) -----
    def _setup_map(self):
        ax = self.ax_map
        ax.set_title("Map view (geodetic)")
        ax.set_xlabel("Longitude [deg]")
        ax.set_ylabel("Latitude [deg]")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.ticklabel_format(useOffset=False, style="plain")
        ax.tick_params(labelsize=8)
        (self.trail_line,) = ax.plot([], [], "-", color="tab:blue",
                                     linewidth=1.5, label="Trail")
        (self.vehicle_dot,) = ax.plot([], [], "o", color="tab:red",
                                      markersize=10, label="Vehicle")
        # _autoscale_map() pins both x/y limits with equalized spans, so the
        # aspect must adapt the box, not the data limits — with "datalim"
        # matplotlib overrides one of our fixed limits and warns every frame.
        ax.set_aspect("equal", adjustable="box")
        self._draw_planned_overlay()
        self._autoscale_map()
        ax.legend(loc="upper right", fontsize=8)

    def _draw_planned_overlay(self):
        ax = self.ax_map
        if self._planned_line is not None:
            try:
                self._planned_line.remove()
            except (ValueError, AttributeError):
                pass
            self._planned_line = None
        for lbl in self._planned_labels:
            try:
                lbl.remove()
            except (ValueError, AttributeError):
                pass
        self._planned_labels = []

        if self.wp_lat:
            (self._planned_line,) = ax.plot(
                self.wp_lon, self.wp_lat, "o--",
                color="tab:orange", label="Waypoints", markersize=8,
            )
            for i, (lo, la) in enumerate(zip(self.wp_lon, self.wp_lat)):
                t = ax.annotate(str(i), (lo, la), textcoords="offset points",
                                xytext=(5, 5), fontsize=8, color="tab:orange")
                self._planned_labels.append(t)

    def _autoscale_map(self):
        """Fit all waypoints + trail + vehicle with 10% margin and equalized
        lat/lon spans (avoids a pencil-thin axis for linear missions)."""
        lats: list[float] = list(self.wp_lat)
        lons: list[float] = list(self.wp_lon)
        with self.shared.lock:
            lats.extend(self.shared.trail_lat)
            lons.extend(self.shared.trail_lon)
            if self.shared.latest is not None:
                lats.append(self.shared.latest.lat)
                lons.append(self.shared.latest.lon)
        if not lats or not lons:
            return
        lat_min, lat_max = min(lats), max(lats)
        lon_min, lon_max = min(lons), max(lons)
        span = max(lat_max - lat_min, lon_max - lon_min, 1e-4)
        half = 0.5 * span + 0.10 * span
        lat_c = 0.5 * (lat_min + lat_max)
        lon_c = 0.5 * (lon_min + lon_max)
        self.ax_map.set_xlim(lon_c - half, lon_c + half)
        self.ax_map.set_ylim(lat_c - half, lat_c + half)

    # ----- Trajectory (3D) -----
    def _setup_traj3d(self):
        ax = self.ax_traj3d
        ax.set_title("3D trajectory")
        ax.set_xlabel("Lon [deg]", fontsize=8)
        ax.set_ylabel("Lat [deg]", fontsize=8)
        ax.set_zlabel("Alt [m]", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.ticklabel_format(useOffset=False, style="plain")
        (self.traj3d_planned,) = ax.plot([], [], [], "o--", color="tab:orange",
                                         markersize=5, label="Waypoints")
        (self.traj3d_trail,) = ax.plot([], [], [], "-", color="tab:blue",
                                       linewidth=1.5, label="Trail")
        (self.traj3d_dot,) = ax.plot([], [], [], "o", color="tab:red",
                                     markersize=8, label="Vehicle")
        ax.legend(loc="upper left", fontsize=7)
        self._update_traj3d_planned()
        self._autoscale_traj3d()

    def _update_traj3d_planned(self):
        self.traj3d_planned.set_data(self.wp_lon, self.wp_lat)
        self.traj3d_planned.set_3d_properties(self.wp_alt)

    def _autoscale_traj3d(self):
        lats: list[float] = list(self.wp_lat)
        lons: list[float] = list(self.wp_lon)
        alts: list[float] = list(self.wp_alt)
        with self.shared.lock:
            lats.extend(self.shared.trail_lat)
            lons.extend(self.shared.trail_lon)
            alts.extend(self.shared.trail_alt)
        if not lats:
            return
        lat_min, lat_max = min(lats), max(lats)
        lon_min, lon_max = min(lons), max(lons)
        span = max(lat_max - lat_min, lon_max - lon_min, 1e-4)
        half = 0.5 * span + 0.10 * span
        lat_c = 0.5 * (lat_min + lat_max)
        lon_c = 0.5 * (lon_min + lon_max)
        self.ax_traj3d.set_xlim(lon_c - half, lon_c + half)
        self.ax_traj3d.set_ylim(lat_c - half, lat_c + half)
        alt_min, alt_max = (min(alts), max(alts)) if alts else (0.0, 1.0)
        alt_pad = max(0.10 * (alt_max - alt_min), 1.0)
        self.ax_traj3d.set_zlim(min(0.0, alt_min - alt_pad), alt_max + alt_pad)

    # ----- Attitude (3D) -----
    def _setup_attitude(self):
        ax = self.ax_att
        ax.set_title("Attitude (body axes in NED)")
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_zlim(-1.2, 1.2)
        ax.set_xlabel("North", fontsize=8)
        ax.set_ylabel("East", fontsize=8)
        ax.set_zlabel("Down", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.invert_zaxis()
        gx, gy = np.meshgrid(np.linspace(-1, 1, 5), np.linspace(-1, 1, 5))
        ax.plot_wireframe(gx, gy, np.zeros_like(gx), color="lightgray",
                          linewidth=0.5, alpha=0.6)
        self._att_artists: list = []

    def _draw_quad(self, roll: float, pitch: float, yaw: float):
        ax = self.ax_att
        for a in self._att_artists:
            try:
                a.remove()
            except (ValueError, AttributeError):
                pass
        self._att_artists = []

        R = _rotation_body_to_ned(roll, pitch, yaw)
        Rt = R.T  # (points @ R.T) maps body -> NED

        arms_n = _ARM_TIPS @ Rt
        for i, j in [(0, 2), (1, 3)]:  # FR<->RL, FL<->RR
            ln, = ax.plot(
                [arms_n[i, 0], arms_n[j, 0]],
                [arms_n[i, 1], arms_n[j, 1]],
                [arms_n[i, 2], arms_n[j, 2]],
                color="black", linewidth=2.5,
            )
            self._att_artists.append(ln)

        for tip, color in zip(_ARM_TIPS, _ROTOR_COLORS):
            disk_n = (_DISK_TEMPLATE + tip) @ Rt
            poly = Poly3DCollection([disk_n], facecolor=color, alpha=0.55,
                                    edgecolor=color, linewidths=1.0)
            ax.add_collection3d(poly)
            self._att_artists.append(poly)
            ln, = ax.plot(disk_n[:, 0], disk_n[:, 1], disk_n[:, 2],
                          color=color, linewidth=1.0)
            self._att_artists.append(ln)

        verts_n = _BODY_VERTS @ Rt
        body = Poly3DCollection([verts_n[f] for f in _BODY_FACES],
                                facecolor="dimgray", alpha=0.85,
                                edgecolor="black", linewidths=0.6)
        ax.add_collection3d(body)
        self._att_artists.append(body)

        fwd = R[:, 0] * 0.95
        q = ax.quiver(0.0, 0.0, 0.0, fwd[0], fwd[1], fwd[2],
                      color="lime", linewidth=2.5, arrow_length_ratio=0.18)
        self._att_artists.append(q)

    # ----- Missions panel -----
    def _setup_missions_panel(self, initial_path: str | None):
        self.mission_radio = None
        self._active_mission_name = Path(initial_path).name if initial_path else ""
        self.btn_refresh = Button(self.ax_btn_refresh, "Refresh list")
        self.btn_refresh.label.set_fontsize(8)
        self.btn_refresh.on_clicked(self._on_refresh_missions)
        self._rebuild_mission_radio()

    def _rebuild_mission_radio(self):
        """(Re)create the mission RadioButtons from the current file scan,
        preserving the active selection when the file still exists. The
        RadioButtons constructor does not fire on_clicked, so rebuilding
        never triggers a spurious mission upload."""
        ax = self.ax_missions
        ax.clear()
        ax.set_title("Missions", fontsize=10)
        self.mission_radio = None
        if not self.mission_files:
            ax.axis("off")
            ax.text(0.5, 0.5, f"No mission files in\n{self.missions_dir}",
                    ha="center", va="center", fontsize=8,
                    transform=ax.transAxes)
            return
        labels = [p.name for p in self.mission_files]
        active = 0
        if self._active_mission_name in labels:
            active = labels.index(self._active_mission_name)
        self.mission_radio = RadioButtons(ax, labels, active=active)
        for lbl in self.mission_radio.labels:
            lbl.set_fontsize(8)
        self.mission_radio.on_clicked(self._on_mission_selected)

    def _on_refresh_missions(self, _event):
        self.mission_files = scan_mission_files(self.missions_dir)
        self._rebuild_mission_radio()
        self.fig.canvas.draw_idle()
        self._flash(f"Mission list refreshed ({len(self.mission_files)} files)")
        print(f"[gcs] mission list refreshed: "
              f"{[p.name for p in self.mission_files]}")

    def _on_mission_selected(self, label: str):
        path = Path(self.missions_dir) / label
        try:
            home, wps = self._set_waypoints_from_file(path)
        except Exception as e:
            self._flash(f"Load failed: {e}")
            print(f"[gcs] failed to load mission '{path}': {e}")
            return
        self._active_mission_name = label
        self.commands.load_mission(home, wps)
        self._clear_trail()
        self._draw_planned_overlay()
        self._autoscale_map()
        self._update_traj3d_planned()
        self._autoscale_traj3d()
        self._rebuild_wplist()
        self._flash(f"Uploaded {label} ({len(wps)} WPs); press Start")
        print(f"[gcs] mission '{label}' uploaded ({len(wps)} waypoints)")

    # ----- Waypoint list panel -----
    def _rebuild_wplist(self):
        ax = self.ax_wplist
        for a in self._wplist_artists:
            try:
                a.remove()
            except (ValueError, AttributeError):
                pass
        self._wplist_artists = []
        ax.clear()
        ax.axis("off")
        title = f"Waypoints ({self.mission_name})" if self.mission_name else "Waypoints"
        ax.set_title(title, fontsize=9)
        if not self.wp_lat:
            t = ax.text(0.5, 0.5, "(no mission)", ha="center", va="center",
                        fontsize=8, transform=ax.transAxes)
            self._wplist_artists.append(t)
            return
        n = len(self.wp_lat)
        header = ax.text(0.02, 0.98, " #      lat        lon      alt",
                         transform=ax.transAxes, fontsize=7.5,
                         family="monospace", va="top", weight="bold")
        self._wplist_artists.append(header)
        for i, (la, lo, al) in enumerate(zip(self.wp_lat, self.wp_lon, self.wp_alt)):
            y = 0.98 - (i + 1) * min(0.9 / (n + 1), 0.085)
            t = ax.text(0.02, y,
                        f"{i:2d} {la:10.5f} {lo:10.5f} {al:6.1f}",
                        transform=ax.transAxes, fontsize=7.5,
                        family="monospace", va="top", color="black")
            self._wplist_artists.append(t)
        self._wplist_active = -1  # force re-highlight on next frame

    def _highlight_wplist(self, active_index: int):
        if active_index == self._wplist_active:
            return
        self._wplist_active = active_index
        # artists[0] is the header; waypoint rows start at 1
        for i, t in enumerate(self._wplist_artists[1:]):
            if i == active_index:
                t.set_color("tab:red")
                t.set_weight("bold")
            else:
                t.set_color("black")
                t.set_weight("normal")

    # ----- Animation -----
    def _on_frame(self, _):
        with self.shared.lock:
            pkt = self.shared.latest
            lats = list(self.shared.trail_lat)
            lons = list(self.shared.trail_lon)
            alts = list(self.shared.trail_alt)

        if pkt is None:
            return []

        self.trail_line.set_data(lons, lats)
        self.vehicle_dot.set_data([pkt.lon], [pkt.lat])
        self._autoscale_map()

        self.traj3d_trail.set_data(lons, lats)
        self.traj3d_trail.set_3d_properties(alts)
        self.traj3d_dot.set_data([pkt.lon], [pkt.lat])
        self.traj3d_dot.set_3d_properties([pkt.alt])
        self._autoscale_traj3d()

        self._draw_quad(pkt.roll, pkt.pitch, pkt.yaw)
        self._highlight_wplist(pkt.wp_index)

        speed = float(np.linalg.norm([pkt.vn, pkt.ve, pkt.vd]))
        status = ""
        now = time.time()
        if self._status_msg and self._status_until_wall is not None \
                and now < self._status_until_wall:
            status = f"   [{self._status_msg}]"
        elif self._status_until_wall is not None and now >= self._status_until_wall:
            self._status_msg = ""
            self._status_until_wall = None
        text = (
            f"[{pkt.status.upper():7s}] "
            f"t={pkt.t:7.2f}s  "
            f"lat={pkt.lat:10.6f}  lon={pkt.lon:10.6f}  alt={pkt.alt:6.1f}m  "
            f"|v|={speed:5.2f}m/s  "
            f"roll={np.degrees(pkt.roll):+6.1f}  "
            f"pitch={np.degrees(pkt.pitch):+6.1f}  "
            f"yaw={np.degrees(pkt.yaw):+6.1f} deg  "
            f"WP {pkt.wp_index}"
            f"{status}"
        )
        self.text_artist.set_text(text)
        return []

    # ----- Button callbacks -----
    def _flash(self, msg: str, duration: float = 4.0):
        self._status_msg = msg
        self._status_until_wall = time.time() + duration

    def _clear_trail(self):
        with self.shared.lock:
            self.shared.trail_lat.clear()
            self.shared.trail_lon.clear()
            self.shared.trail_alt.clear()

    def _on_start(self, _event):
        self.commands.start()
        self._flash("Start sent")
        print("[gcs] start command sent")

    def _on_stop(self, _event):
        self.commands.pause()
        self._flash("Stop (pause) sent")
        print("[gcs] pause command sent")

    def _on_continue(self, _event):
        self.commands.resume()
        self._flash("Continue sent")
        print("[gcs] resume command sent")

    def _on_reset(self, _event):
        self.commands.reset()
        self._clear_trail()
        self._flash("Reset sent; press Start to fly")
        print("[gcs] reset command sent")

    def _on_save_trajectory(self, _event):
        path = self._pick_save_path()
        # Cancelled dialog: empty path means "let the simulator pick a default"
        self.commands.save_trajectory(path)
        if path:
            self._flash(f"Save requested: {Path(path).name}")
            print(f"[gcs] save_trajectory sent (path={path})")
        else:
            self._flash("Save requested (sim will pick default filename)")
            print("[gcs] save_trajectory sent (default path on sim host)")

    def _pick_save_path(self) -> str | None:
        try:
            import tkinter as tk
            from tkinter import filedialog
        except ImportError:
            return None
        root = tk.Tk()
        root.withdraw()
        try:
            path = filedialog.asksaveasfilename(
                title="Save trajectory CSV (path is on the simulator host)",
                defaultextension=".csv",
                initialfile="trajectory.csv",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            )
        finally:
            root.destroy()
        return path or None

    # ----- Run -----
    def run(self):
        rx_t = threading.Thread(
            target=receiver_thread,
            args=(self.host, self.port, self.shared, self.stop),
            daemon=True,
        )
        rx_t.start()
        try:
            self.anim = FuncAnimation(self.fig, self._on_frame,
                                      interval=50, blit=False,
                                      cache_frame_data=False)
            plt.show()
        finally:
            self.stop.set()
            rx_t.join(timeout=1.0)
            self.commands.close()


def main():
    ap = argparse.ArgumentParser(description="Quadcopter GCS (UDP receiver + display)")
    ap.add_argument("--host", default="239.0.0.1",
                    help="Telemetry source: multicast group to join (default) "
                         "or a local bind address for unicast")
    ap.add_argument("--port", type=int, default=14550, help="UDP bind port (telemetry)")
    ap.add_argument("--cmd-host", default="127.0.0.1",
                    help="UDP destination host for commands (sim address)")
    ap.add_argument("--cmd-port", type=int, default=14551,
                    help="UDP destination port for commands")
    ap.add_argument("--waypoints", default="waypoints.json",
                    help="Initially displayed mission (must match the sim's)")
    ap.add_argument("--missions-dir", default=".",
                    help="Directory scanned for selectable mission *.json files")
    args = ap.parse_args()
    app = GCSApp(args.host, args.port, args.cmd_host, args.cmd_port,
                 args.waypoints, missions_dir=args.missions_dir)
    app.run()


if __name__ == "__main__":
    main()
