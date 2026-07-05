"""GCS -> Simulator command protocol over UDP (JSON).

Messages are simple JSON objects with a ``type`` field. Two types are defined:

  {"type": "start"}
      Begin the flight (from the WAITING state). If the sim is paused this
      behaves like ``resume``.

  {"type": "pause"}
      Freeze the simulation (physics stops; state is held).

  {"type": "resume"}
      Continue a paused simulation from the held state.

  {"type": "reset"}
      Reset the vehicle to (0, 0, 0) at the home point, zero attitude and
      velocity, replay the current mission from the first waypoint. The sim
      returns to the WAITING state (send ``start`` to fly again).

  {"type": "load_mission",
   "home": {"lat": ..., "lon": ..., "alt": ...},   (optional)
   "waypoints": [{"lat": ..., "lon": ..., "alt": ...}, ...]}
      Replace the active mission. If ``home`` is provided, the local NED
      origin is moved to it. The vehicle state is reset as for ``reset``.

  {"type": "save_trajectory",
   "path": "/abs/or/relative/path.csv"}            (optional)
      Dump the 10 ms-cadence trajectory buffer to ``path`` (or a timestamped
      filename in the simulator's working directory if omitted). The buffer
      is preserved; the sim keeps recording afterwards.

Packets are sent on a separate UDP port from telemetry to keep the two
streams independent.
"""
from __future__ import annotations

import json
import socket
from typing import Any


class CommandSender:
    def __init__(self, host: str = "127.0.0.1", port: int = 14551):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, msg: dict[str, Any]) -> None:
        try:
            self.sock.sendto(json.dumps(msg).encode("utf-8"), self.addr)
        except OSError:
            pass

    def start(self) -> None:
        self.send({"type": "start"})

    def pause(self) -> None:
        self.send({"type": "pause"})

    def resume(self) -> None:
        self.send({"type": "resume"})

    def reset(self) -> None:
        self.send({"type": "reset"})

    def load_mission(self, home: dict | None, waypoints: list[dict]) -> None:
        msg: dict[str, Any] = {"type": "load_mission", "waypoints": waypoints}
        if home is not None:
            msg["home"] = home
        self.send(msg)

    def save_trajectory(self, path: str | None = None) -> None:
        """Tell the simulator to dump its 10 ms-cadence trajectory buffer to a
        CSV file. If ``path`` is None the simulator picks a timestamped
        filename in its current working directory.
        """
        msg: dict[str, Any] = {"type": "save_trajectory"}
        if path:
            msg["path"] = path
        self.send(msg)

    def close(self) -> None:
        self.sock.close()


class CommandReceiver:
    def __init__(self, host: str = "0.0.0.0", port: int = 14551, timeout: float = 0.2):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.settimeout(timeout)

    def recv(self) -> dict[str, Any] | None:
        try:
            data, _ = self.sock.recvfrom(65536)
        except socket.timeout:
            return None
        try:
            return json.loads(data.decode("utf-8"))
        except (ValueError, TypeError):
            return None

    def close(self) -> None:
        self.sock.close()
