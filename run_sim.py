"""Entry point for the quadcopter simulator.

Usage:
    python run_sim.py [--waypoints waypoints.json] [--drone drone.json]
                      [--host 127.0.0.1] [--port 14550]
                      [--cmd-host 0.0.0.0] [--cmd-port 14551]
                      [--duration 60] [--no-realtime]
"""
from __future__ import annotations

import argparse
from pathlib import Path

from quad_sim.controller import ControllerGains
from quad_sim.drone_config import apply_drone_config, load_drone_config
from quad_sim.dynamics import QuadParams
from quad_sim.simulator import SimConfig, Simulator
from quad_sim.waypoints import load_waypoints


def main():
    ap = argparse.ArgumentParser(description="Quadcopter 6DOF simulator")
    ap.add_argument("--waypoints", default="waypoints.json",
                    help="Path to waypoints JSON file")
    ap.add_argument("--drone", default="drone.json",
                    help="Path to drone parameters JSON (optional)")
    ap.add_argument("--host", default="127.0.0.1", help="UDP destination host")
    ap.add_argument("--port", type=int, default=14550, help="UDP destination port")
    ap.add_argument("--duration", type=float, default=None,
                    help="Stop after N seconds of sim time (default: run forever)")
    ap.add_argument("--no-realtime", action="store_true",
                    help="Run as fast as possible (no wall-clock pacing)")
    ap.add_argument("--telem-hz", type=float, default=50.0,
                    help="Telemetry transmit rate")
    ap.add_argument("--cmd-host", default="0.0.0.0",
                    help="UDP bind host for inbound GCS commands")
    ap.add_argument("--cmd-port", type=int, default=14551,
                    help="UDP bind port for inbound GCS commands")
    ap.add_argument("--autostart", action="store_true",
                    help="Start flying immediately instead of waiting for the "
                         "GCS start command")
    args = ap.parse_args()

    home, wps = load_waypoints(args.waypoints)

    params = QuadParams()
    gains = ControllerGains()
    if Path(args.drone).exists():
        drone_cfg = load_drone_config(args.drone)
        apply_drone_config(params, gains, drone_cfg)
        print(f"Loaded drone config: {args.drone}")
    else:
        print(f"Drone config not found ({args.drone}); using built-in defaults")

    cfg = SimConfig(
        udp_host=args.host,
        udp_port=args.port,
        duration_s=args.duration,
        realtime=not args.no_realtime,
        telem_rate_hz=args.telem_hz,
        cmd_host=args.cmd_host,
        cmd_port=args.cmd_port,
        autostart=args.autostart,
    )
    sim = Simulator(home=home, waypoints_geo=wps, config=cfg,
                    quad_params=params, gains=gains)
    print(f"Sending UDP telemetry to {args.host}:{args.port}")
    print(f"Listening for GCS commands on {args.cmd_host}:{args.cmd_port}")
    print(f"Home: {home.lat:.6f}, {home.lon:.6f}  ({len(wps)} waypoints)")
    if not args.autostart:
        print("Waiting for GCS start command...")
    try:
        sim.run()
    except KeyboardInterrupt:
        print("\nSimulator interrupted by user.")


if __name__ == "__main__":
    main()
