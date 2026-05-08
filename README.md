# Quadcopter 6DOF Simulator

A Python-based quadcopter flight simulator with a 12-state 6DOF rigid-body
dynamics model, a cascaded PID autopilot for autonomous waypoint flight, UDP
telemetry, and a live ground control station (GCS) that displays the vehicle
position on a map and its attitude in 3D.

## Features

- **Geodetic waypoints** — missions are defined in latitude / longitude / altitude.
- **6DOF dynamics** — 12 states (NED position, NED velocity, ZYX Euler
  attitude, body angular rates), RK4 integration at 200 Hz, configurable mass,
  inertia, drag and actuator limits.
- **Autonomous flight** — cascaded position → velocity → attitude → body-rate
  PID controller. Yaw automatically points toward the next waypoint when the
  horizontal travel exceeds 1 m.
- **Real-time UDP telemetry** — JSON packets on port `14550` (default) at 50 Hz
  carrying position, velocity, attitude, body rates, thrust, and current
  waypoint.
- **Live GCS UI** — single matplotlib window with:
  - 2D map view showing planned waypoints and the live lat/lon breadcrumb,
  - 3D attitude indicator showing body axes and a rotor cross,
  - textual telemetry bar (time, lat/lon/alt, speed, roll/pitch/yaw, target WP).

## Repository Layout

```
quad_6dof_model/
├── quad_sim/                # Simulator package
│   ├── coordinates.py       # Geodetic <-> local NED conversion
│   ├── dynamics.py          # 6DOF rigid-body model + RK4 integrator
│   ├── controller.py        # Cascaded PID autopilot
│   ├── waypoints.py         # Waypoint loader + manager
│   ├── telemetry.py         # UDP JSON sender/receiver + TelemetryPacket
│   └── simulator.py         # Main simulation loop
├── gcs/
│   └── gcs_app.py           # UDP receiver + matplotlib map / attitude UI
├── run_sim.py               # Simulator entry point
├── run_gcs.py               # GCS entry point
├── waypoints.json           # Example mission (climb -> square -> land)
├── requirements.txt
└── README.md
```

## Dependencies

- Python ≥ 3.10
- numpy ≥ 1.24
- scipy ≥ 1.10
- matplotlib ≥ 3.7 (with the `mpl_toolkits` 3D backend, included by default)

The GCS uses matplotlib's default interactive backend; on Linux you may need a
GUI backend such as `TkAgg` (install `python3-tk` from the system package
manager) or `Qt5Agg` (`pip install PyQt5`). No internet access is required —
the map view is a plain lat/lon plot, not tile-backed.

## Installation

Clone the repository and install the dependencies, ideally in a virtual
environment:

```bash
git clone <repo-url>
cd quad_6dof_model

python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate

pip install -r requirements.txt
```

## Running

You typically run the GCS first (so it is listening) and then the simulator.

**Terminal 1 — GCS:**

```bash
python run_gcs.py
# Optional flags:
#   --host 0.0.0.0           # UDP bind host (default 0.0.0.0)
#   --port 14550             # UDP bind port (default 14550)
#   --waypoints waypoints.json   # overlay planned waypoints on the map
```

**Terminal 2 — Simulator:**

```bash
python run_sim.py
# Optional flags:
#   --waypoints waypoints.json   # mission file (default waypoints.json)
#   --host 127.0.0.1             # UDP destination host
#   --port 14550                 # UDP destination port
#   --duration 60                # stop after N seconds of sim time
#   --no-realtime                # run as fast as possible (no wall-clock pacing)
#   --telem-hz 50                # telemetry transmit rate
```

The default mission climbs to 30 m, flies a square at 30/50 m altitude, and
returns to a 5 m hover. The full pattern completes in roughly 42 seconds of
simulated time.

## Mission File Format

Waypoints are defined in JSON:

```json
{
  "home": {"lat": 37.4275, "lon": -122.1697, "alt": 0.0},
  "waypoints": [
    {"lat": 37.4275, "lon": -122.1697, "alt": 30.0},
    {"lat": 37.4280, "lon": -122.1697, "alt": 30.0},
    {"lat": 37.4280, "lon": -122.1690, "alt": 50.0}
  ]
}
```

- `home` — geodetic origin used to anchor the local NED frame. Its `alt`
  defines the altitude reference; waypoint altitudes are relative to it.
- `waypoints` — ordered list. The vehicle advances when it is within
  the configured accept-radius (default 2 m) of the current target, and
  hovers indefinitely at the final waypoint.

## UDP Telemetry Format

Each packet is a UTF-8 encoded JSON object:

| Field | Description |
|-------|-------------|
| `t` | Sim time [s] |
| `lat`, `lon`, `alt` | Geodetic position (alt above home) |
| `north`, `east`, `down` | NED position [m] |
| `vn`, `ve`, `vd` | NED velocity [m/s] |
| `roll`, `pitch`, `yaw` | Euler ZYX attitude [rad] |
| `p`, `q`, `r` | Body angular rates [rad/s] |
| `thrust` | Commanded collective thrust [N] |
| `wp_index` | Active waypoint index |
| `wp_lat`, `wp_lon`, `wp_alt` | Active waypoint coordinates |

Any client that can read UDP and parse JSON can consume the stream — see
`quad_sim/telemetry.py` (`TelemetryPacket`, `UDPTelemetryReceiver`) for the
reference Python decoder.

## Coordinate Conventions

- **NED frame** (north, east, down). Altitude maps to `-down`.
- **Body frame** (FRD: forward, right, down). Thrust acts along `-z_body`.
- **Attitude** uses ZYX intrinsic Euler angles (yaw → pitch → roll) consistent
  with standard aerospace usage.
- **Geodetic ↔ NED** uses a flat-earth equirectangular approximation about the
  home point. This is accurate to within a few cm/m for operating areas up to
  a few km; for larger areas swap in a proper ellipsoidal projection
  (e.g. via `pyproj`).

## Tuning and Extension

- `quad_sim.dynamics.QuadParams` — vehicle mass, inertia, drag, thrust and
  torque limits.
- `quad_sim.controller.ControllerGains` — PID gains and per-loop saturation
  limits (max velocity, tilt, body rate).
- `quad_sim.simulator.SimConfig` — physics step (default 5 ms), telemetry rate,
  real-time pacing, and UDP destination.

The controller is intentionally simple (small-angle tilt-from-accel mapping
plus per-axis PID) so it is easy to read; replace it with e.g. an SE(3)
geometric controller or an MPC if you need higher-fidelity tracking.

## Limitations

- Flat-earth coordinate model — not suitable for missions spanning tens of km.
- Euler-angle attitude — gimbal lock at pitch = ±90°. The default 30° tilt
  limit keeps the controller well clear, but aggressive aerobatics would need
  a quaternion formulation.
- Map view is a lat/lon line plot, not a tile-backed basemap. Adding
  `contextily` or `folium` would give a real map background.
- No wind, sensor noise, motor dynamics, or battery model — the simulator
  is built for control / mission-logic demos, not for high-fidelity
  hardware-in-the-loop work.
