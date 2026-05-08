"""Ground control station: UDP telemetry listener + live visualisation.

Two panes side-by-side:
  - 2D map view (lat/lon) showing planned waypoints and the vehicle trail
  - 3D attitude view showing the body frame axes and a rotor cross
A status line below prints live telemetry (position, attitude, mode).

The receiver runs on a daemon thread and updates a shared latest-state slot;
the matplotlib animation samples it at the display rate.
"""
from __future__ import annotations

import argparse
import threading
from collections import deque
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

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


class GCSApp:
    def __init__(self, host: str, port: int, waypoints_path: str | None):
        self.shared = SharedState()
        self.stop = threading.Event()
        self.host = host
        self.port = port

        self.wp_lat: list[float] = []
        self.wp_lon: list[float] = []
        if waypoints_path:
            try:
                _, wps = load_waypoints(waypoints_path)
                self.wp_lat = [w.lat for w in wps]
                self.wp_lon = [w.lon for w in wps]
            except FileNotFoundError:
                print(f"Waypoints file not found: {waypoints_path} (continuing without)")

        # Layout: 1 row, 2 cols, with a text bar on the bottom.
        self.fig = plt.figure(figsize=(14, 7))
        self.fig.suptitle("Quadcopter GCS  -  UDP {}:{}".format(host, port))
        gs = self.fig.add_gridspec(2, 2, height_ratios=[20, 1])
        self.ax_map = self.fig.add_subplot(gs[0, 0])
        self.ax_att = self.fig.add_subplot(gs[0, 1], projection="3d")
        self.ax_text = self.fig.add_subplot(gs[1, :])
        self.ax_text.axis("off")

        self._setup_map()
        self._setup_attitude()
        self.text_artist = self.ax_text.text(0.01, 0.5, "Waiting for telemetry...",
                                             transform=self.ax_text.transAxes,
                                             fontsize=10, family="monospace",
                                             va="center")

    # ----- Map -----
    def _setup_map(self):
        ax = self.ax_map
        ax.set_title("Map view (geodetic)")
        ax.set_xlabel("Longitude [deg]")
        ax.set_ylabel("Latitude [deg]")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.ticklabel_format(useOffset=False, style="plain")
        if self.wp_lat:
            ax.plot(self.wp_lon, self.wp_lat, "o--", color="tab:orange",
                    label="Waypoints", markersize=8)
            for i, (lo, la) in enumerate(zip(self.wp_lon, self.wp_lat)):
                ax.annotate(str(i), (lo, la), textcoords="offset points",
                            xytext=(5, 5), fontsize=8, color="tab:orange")
            ax.set_xlim(min(self.wp_lon) - 0.0003, max(self.wp_lon) + 0.0003)
            ax.set_ylim(min(self.wp_lat) - 0.0003, max(self.wp_lat) + 0.0003)
        (self.trail_line,) = ax.plot([], [], "-", color="tab:blue",
                                     linewidth=1.5, label="Trail")
        (self.vehicle_dot,) = ax.plot([], [], "o", color="tab:red",
                                      markersize=10, label="Vehicle")
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(loc="upper right", fontsize=8)

    # ----- Attitude -----
    def _setup_attitude(self):
        ax = self.ax_att
        ax.set_title("Attitude (body axes in NED)")
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_zlim(-1.2, 1.2)
        ax.set_xlabel("North")
        ax.set_ylabel("East")
        ax.set_zlabel("Down")
        ax.invert_zaxis()  # show "up" upward
        # Draw a faint reference ground plane
        gx, gy = np.meshgrid(np.linspace(-1, 1, 5), np.linspace(-1, 1, 5))
        ax.plot_wireframe(gx, gy, np.zeros_like(gx), color="lightgray",
                          linewidth=0.5, alpha=0.6)
        # Initial axis quivers; we'll replace them each frame.
        self._att_artists: list = []

    def _draw_attitude(self, roll: float, pitch: float, yaw: float):
        ax = self.ax_att
        for a in self._att_artists:
            a.remove()
        self._att_artists = []

        R = _rotation_body_to_ned(roll, pitch, yaw)
        # Body axes in NED
        x_b = R[:, 0]
        y_b = R[:, 1]
        z_b = R[:, 2]
        origin = np.zeros(3)
        for vec, color, label in [
            (x_b, "tab:red", "x_b (fwd)"),
            (y_b, "tab:green", "y_b (right)"),
            (z_b, "tab:blue", "z_b (down)"),
        ]:
            q = ax.quiver(origin[0], origin[1], origin[2],
                          vec[0], vec[1], vec[2],
                          color=color, linewidth=2.0, length=1.0, normalize=True)
            self._att_artists.append(q)

        # Rotor cross to suggest a quad shape (arms at +/-x, +/-y body)
        arm = 0.7
        arms = np.array([
            [+arm, 0, 0], [-arm, 0, 0],
            [0, +arm, 0], [0, -arm, 0],
        ])
        arms_n = arms @ R.T
        line1, = ax.plot([arms_n[0, 0], arms_n[1, 0]],
                         [arms_n[0, 1], arms_n[1, 1]],
                         [arms_n[0, 2], arms_n[1, 2]],
                         color="black", linewidth=2)
        line2, = ax.plot([arms_n[2, 0], arms_n[3, 0]],
                         [arms_n[2, 1], arms_n[3, 1]],
                         [arms_n[2, 2], arms_n[3, 2]],
                         color="black", linewidth=2)
        self._att_artists += [line1, line2]

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
        # Auto-expand map limits if the vehicle ranges outside the planned box.
        xlim = self.ax_map.get_xlim(); ylim = self.ax_map.get_ylim()
        pad_x = max(0.0001, 0.05 * (xlim[1] - xlim[0]))
        pad_y = max(0.0001, 0.05 * (ylim[1] - ylim[0]))
        if pkt.lon < xlim[0] or pkt.lon > xlim[1]:
            self.ax_map.set_xlim(min(xlim[0], pkt.lon - pad_x),
                                 max(xlim[1], pkt.lon + pad_x))
        if pkt.lat < ylim[0] or pkt.lat > ylim[1]:
            self.ax_map.set_ylim(min(ylim[0], pkt.lat - pad_y),
                                 max(ylim[1], pkt.lat + pad_y))

        self._draw_attitude(pkt.roll, pkt.pitch, pkt.yaw)

        speed = float(np.linalg.norm([pkt.vn, pkt.ve, pkt.vd]))
        text = (
            f"t={pkt.t:7.2f}s  "
            f"lat={pkt.lat:10.6f}  lon={pkt.lon:10.6f}  alt={pkt.alt:6.1f}m  "
            f"|v|={speed:5.2f}m/s  "
            f"roll={np.degrees(pkt.roll):+6.1f}  "
            f"pitch={np.degrees(pkt.pitch):+6.1f}  "
            f"yaw={np.degrees(pkt.yaw):+6.1f} deg  "
            f"WP {pkt.wp_index}->({pkt.wp_lat:.5f},{pkt.wp_lon:.5f},{pkt.wp_alt:.1f})"
        )
        self.text_artist.set_text(text)
        return []

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
            plt.tight_layout()
            plt.show()
        finally:
            self.stop.set()
            rx_t.join(timeout=1.0)


def main():
    ap = argparse.ArgumentParser(description="Quadcopter GCS (UDP receiver + display)")
    ap.add_argument("--host", default="0.0.0.0", help="UDP bind host")
    ap.add_argument("--port", type=int, default=14550, help="UDP bind port")
    ap.add_argument("--waypoints", default="waypoints.json",
                    help="Path to waypoints JSON (for plan overlay; optional)")
    args = ap.parse_args()
    app = GCSApp(args.host, args.port, args.waypoints)
    app.run()


if __name__ == "__main__":
    main()
