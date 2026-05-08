"""Entry point for the quadcopter simulator.

Usage:
    python run_sim.py [--waypoints waypoints.json] [--host 127.0.0.1]
                      [--port 14550] [--duration 60] [--no-realtime]
"""
from __future__ import annotations

import argparse

from quad_sim.simulator import SimConfig, Simulator
from quad_sim.waypoints import load_waypoints


def main():
    ap = argparse.ArgumentParser(description="Quadcopter 6DOF simulator")
    ap.add_argument("--waypoints", default="waypoints.json",
                    help="Path to waypoints JSON file")
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
    args = ap.parse_args()

    home, wps = load_waypoints(args.waypoints)
    cfg = SimConfig(
        udp_host=args.host,
        udp_port=args.port,
        duration_s=args.duration,
        realtime=not args.no_realtime,
        telem_rate_hz=args.telem_hz,
        cmd_host=args.cmd_host,
        cmd_port=args.cmd_port,
    )
    sim = Simulator(home=home, waypoints_geo=wps, config=cfg)
    print(f"Sending UDP telemetry to {args.host}:{args.port}")
    print(f"Listening for GCS commands on {args.cmd_host}:{args.cmd_port}")
    print(f"Home: {home.lat:.6f}, {home.lon:.6f}  ({len(wps)} waypoints)")
    try:
        sim.run()
    except KeyboardInterrupt:
        print("\nSimulator interrupted by user.")


if __name__ == "__main__":
    main()
