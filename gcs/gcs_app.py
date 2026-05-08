"""Ground control station: UDP telemetry listener + live visualisation.

Layout:
  - Top row: 2D map view (lat/lon trail + planned waypoints) and 3D attitude
    view (quadcopter body rendered as arms + rotor disks + heading arrow).
  - Telemetry text bar.
  - Button row: Reset (replay current mission), Load Mission (file picker
    for waypoints.json).

Communication:
  - Telemetry comes from the simulator over UDP (default port 14550).
  - Commands are sent to the simulator over a separate UDP port (default 14551)
    using the JSON protocol defined in ``quad_sim.commands``.
"""
from __future__ import annotations

import argparse
import json
import threading
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from quad_sim.commands import CommandSender
from quad_sim.telemetry import TelemetryPacket, UDPTelemetryReceiver
from quad_sim.waypoints import load_waypoints


@dataclass
class SharedState:
    latest: TelemetryPacket | None = None
    trail_lat: deque = None
    trail_lon: deque = None
    lock: threading.Lock = None

    def __post_init__(self):
        self.trail_lat = deque(maxlen=2000)
        self.trail_lon = deque(maxlen=2000)
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
                shared.trail_lat.append(pkt.lat)
                shared.trail_lon.append(pkt.lon)
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
    [0, 1, 2, 3],  # bottom (+z body)
    [4, 5, 6, 7],  # top    (-z body)
    [0, 1, 5, 4],  # right
    [2, 3, 7, 6],  # left
    [0, 3, 7, 4],  # front
    [1, 2, 6, 5],  # rear
]


class GCSApp:
    def __init__(self,
                 host: str, port: int,
                 cmd_host: str, cmd_port: int,
                 waypoints_path: str | None):
        self.shared = SharedState()
        self.stop = threading.Event()
        self.host = host
        self.port = port
        self.commands = CommandSender(host=cmd_host, port=cmd_port)
        self.cmd_target = (cmd_host, cmd_port)

        self.wp_lat: list[float] = []
        self.wp_lon: list[float] = []
        if waypoints_path:
            try:
                _, wps = load_waypoints(waypoints_path)
                self.wp_lat = [w.lat for w in wps]
                self.wp_lon = [w.lon for w in wps]
            except FileNotFoundError:
                print(f"Waypoints file not found: {waypoints_path} (continuing without)")

        self.fig = plt.figure(figsize=(14, 8))
        self.fig.suptitle(
            f"Quadcopter GCS  -  telemetry {host}:{port}  cmd -> {cmd_host}:{cmd_port}"
        )
        gs = self.fig.add_gridspec(3, 4, height_ratios=[18, 1, 2], hspace=0.35)
        self.ax_map = self.fig.add_subplot(gs[0, :2])
        self.ax_att = self.fig.add_subplot(gs[0, 2:], projection="3d")
        self.ax_text = self.fig.add_subplot(gs[1, :])
        self.ax_text.axis("off")
        self.ax_btn_reset = self.fig.add_subplot(gs[2, 0])
        self.ax_btn_load = self.fig.add_subplot(gs[2, 1])
        # Leave gs[2, 2:] empty for spacing.

        self._planned_line = None
        self._planned_labels: list = []
        self._setup_map()
        self._setup_attitude()
        self.text_artist = self.ax_text.text(
            0.01, 0.5, "Waiting for telemetry...",
            transform=self.ax_text.transAxes,
            fontsize=10, family="monospace", va="center",
        )
        self._status_msg = ""
        self._status_until_t: float | None = None

        self.btn_reset = Button(self.ax_btn_reset, "Reset mission")
        self.btn_load = Button(self.ax_btn_load, "Load waypoints.json")
        self.btn_reset.on_clicked(self._on_reset)
        self.btn_load.on_clicked(self._on_load)

    # ----- Map setup -----
    def _setup_map(self):
        ax = self.ax_map
        ax.set_title("Map view (geodetic)")
        ax.set_xlabel("Longitude [deg]")
        ax.set_ylabel("Latitude [deg]")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.ticklabel_format(useOffset=False, style="plain")
        (self.trail_line,) = ax.plot([], [], "-", color="tab:blue",
                                     linewidth=1.5, label="Trail")
        (self.vehicle_dot,) = ax.plot([], [], "o", color="tab:red",
                                      markersize=10, label="Vehicle")
        ax.set_aspect("equal", adjustable="datalim")
        self._draw_planned_overlay()
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
            ax.set_xlim(min(self.wp_lon) - 0.0003, max(self.wp_lon) + 0.0003)
            ax.set_ylim(min(self.wp_lat) - 0.0003, max(self.wp_lat) + 0.0003)

    # ----- Attitude setup -----
    def _setup_attitude(self):
        ax = self.ax_att
        ax.set_title("Attitude (body axes in NED)")
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_zlim(-1.2, 1.2)
        ax.set_xlabel("North")
        ax.set_ylabel("East")
        ax.set_zlabel("Down")
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
        Rt = R.T  # used as (points @ R.T) to map body->NED

        # Arms (X pattern crossing through the origin)
        arms_n = _ARM_TIPS @ Rt
        for i, j in [(0, 2), (1, 3)]:  # FR<->RL, FL<->RR
            ln, = ax.plot(
                [arms_n[i, 0], arms_n[j, 0]],
                [arms_n[i, 1], arms_n[j, 1]],
                [arms_n[i, 2], arms_n[j, 2]],
                color="black", linewidth=2.5,
            )
            self._att_artists.append(ln)

        # Rotor disks (filled), colored to indicate front (red) / rear (blue)
        for tip, color in zip(_ARM_TIPS, _ROTOR_COLORS):
            disk_b = _DISK_TEMPLATE + tip
            disk_n = disk_b @ Rt
            poly = Poly3DCollection(
                [disk_n], facecolor=color, alpha=0.55,
                edgecolor=color, linewidths=1.0,
            )
            ax.add_collection3d(poly)
            self._att_artists.append(poly)
            # Outline for visibility
            ln, = ax.plot(disk_n[:, 0], disk_n[:, 1], disk_n[:, 2],
                          color=color, linewidth=1.0)
            self._att_artists.append(ln)

        # Body box (dark grey, semi-transparent)
        verts_n = _BODY_VERTS @ Rt
        face_polys = [verts_n[face] for face in _BODY_FACES]
        body = Poly3DCollection(face_polys, facecolor="dimgray", alpha=0.85,
                                edgecolor="black", linewidths=0.6)
        ax.add_collection3d(body)
        self._att_artists.append(body)

        # Heading arrow (body +x, drawn ahead of the body)
        fwd = R[:, 0] * 0.95
        q = ax.quiver(0.0, 0.0, 0.0, fwd[0], fwd[1], fwd[2],
                      color="lime", linewidth=2.5, arrow_length_ratio=0.18)
        self._att_artists.append(q)

    # ----- Animation -----
    def _on_frame(self, _):
        with self.shared.lock:
            pkt = self.shared.latest
            lats = list(self.shared.trail_lat)
            lons = list(self.shared.trail_lon)

        if pkt is None:
            return []

        self.trail_line.set_data(lons, lats)
        self.vehicle_dot.set_data([pkt.lon], [pkt.lat])
        xlim = self.ax_map.get_xlim(); ylim = self.ax_map.get_ylim()
        pad_x = max(0.0001, 0.05 * (xlim[1] - xlim[0]))
        pad_y = max(0.0001, 0.05 * (ylim[1] - ylim[0]))
        if pkt.lon < xlim[0] or pkt.lon > xlim[1]:
            self.ax_map.set_xlim(min(xlim[0], pkt.lon - pad_x),
                                 max(xlim[1], pkt.lon + pad_x))
        if pkt.lat < ylim[0] or pkt.lat > ylim[1]:
            self.ax_map.set_ylim(min(ylim[0], pkt.lat - pad_y),
                                 max(ylim[1], pkt.lat + pad_y))

        self._draw_quad(pkt.roll, pkt.pitch, pkt.yaw)

        speed = float(np.linalg.norm([pkt.vn, pkt.ve, pkt.vd]))
        status = ""
        if self._status_msg and self._status_until_t is not None and pkt.t < self._status_until_t:
            status = f"   [{self._status_msg}]"
        elif self._status_until_t is not None and pkt.t >= self._status_until_t:
            self._status_msg = ""
            self._status_until_t = None
        text = (
            f"t={pkt.t:7.2f}s  "
            f"lat={pkt.lat:10.6f}  lon={pkt.lon:10.6f}  alt={pkt.alt:6.1f}m  "
            f"|v|={speed:5.2f}m/s  "
            f"roll={np.degrees(pkt.roll):+6.1f}  "
            f"pitch={np.degrees(pkt.pitch):+6.1f}  "
            f"yaw={np.degrees(pkt.yaw):+6.1f} deg  "
            f"WP {pkt.wp_index}->({pkt.wp_lat:.5f},{pkt.wp_lon:.5f},{pkt.wp_alt:.1f})"
            f"{status}"
        )
        self.text_artist.set_text(text)
        return []

    # ----- Button callbacks -----
    def _flash(self, msg: str, duration: float = 3.0):
        self._status_msg = msg
        with self.shared.lock:
            t_now = self.shared.latest.t if self.shared.latest else 0.0
        self._status_until_t = t_now + duration

    def _clear_trail(self):
        with self.shared.lock:
            self.shared.trail_lat.clear()
            self.shared.trail_lon.clear()

    def _on_reset(self, _event):
        self.commands.reset()
        self._clear_trail()
        self._flash(f"Reset sent to {self.cmd_target[0]}:{self.cmd_target[1]}")
        print("[gcs] reset command sent")

    def _on_load(self, _event):
        path = self._pick_file()
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text())
            wps = data.get("waypoints") or []
            home = data.get("home")
            if not wps:
                raise ValueError("file has no 'waypoints'")
        except Exception as e:
            self._flash(f"Load failed: {e}")
            print(f"[gcs] failed to load mission: {e}")
            return

        self.commands.load_mission(home, wps)
        self.wp_lat = [w["lat"] for w in wps]
        self.wp_lon = [w["lon"] for w in wps]
        self._draw_planned_overlay()
        self._clear_trail()
        self._flash(f"Loaded {len(wps)} waypoints from {Path(path).name}")
        print(f"[gcs] loaded mission '{path}' ({len(wps)} waypoints)")

    def _pick_file(self) -> str | None:
        try:
            import tkinter as tk
            from tkinter import filedialog
        except ImportError:
            print("[gcs] tkinter unavailable; cannot show file picker")
            return None
        root = tk.Tk()
        root.withdraw()
        try:
            path = filedialog.askopenfilename(
                title="Select waypoints JSON",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
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
    ap.add_argument("--host", default="0.0.0.0", help="UDP bind host (telemetry)")
    ap.add_argument("--port", type=int, default=14550, help="UDP bind port (telemetry)")
    ap.add_argument("--cmd-host", default="127.0.0.1",
                    help="UDP destination host for commands (sim address)")
    ap.add_argument("--cmd-port", type=int, default=14551,
                    help="UDP destination port for commands")
    ap.add_argument("--waypoints", default="waypoints.json",
                    help="Path to waypoints JSON (for plan overlay; optional)")
    args = ap.parse_args()
    app = GCSApp(args.host, args.port, args.cmd_host, args.cmd_port, args.waypoints)
    app.run()


if __name__ == "__main__":
    main()
