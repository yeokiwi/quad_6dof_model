"""GCS -> Simulator command protocol over UDP (JSON).

Messages are simple JSON objects with a ``type`` field. Two types are defined:

  {"type": "reset"}
      Reset the vehicle to (0, 0, 0) at the home point, zero attitude and
      velocity, replay the current mission from the first waypoint.

  {"type": "load_mission",
   "home": {"lat": ..., "lon": ..., "alt": ...},   (optional)
   "waypoints": [{"lat": ..., "lon": ..., "alt": ...}, ...]}
      Replace the active mission. If ``home`` is provided, the local NED
      origin is moved to it. The vehicle state is reset as for ``reset``.

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

    def reset(self) -> None:
        self.send({"type": "reset"})

    def load_mission(self, home: dict | None, waypoints: list[dict]) -> None:
        msg: dict[str, Any] = {"type": "load_mission", "waypoints": waypoints}
        if home is not None:
            msg["home"] = home
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
