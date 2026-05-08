"""UDP telemetry sender. Packets are JSON-encoded for easy debugging."""
from __future__ import annotations

import json
import socket
from dataclasses import asdict, dataclass


@dataclass
class TelemetryPacket:
    t: float
    lat: float
    lon: float
    alt: float
    north: float
    east: float
    down: float
    vn: float
    ve: float
    vd: float
    roll: float
    pitch: float
    yaw: float
    p: float
    q: float
    r: float
    thrust: float
    wp_index: int
    wp_lat: float
    wp_lon: float
    wp_alt: float

    def to_bytes(self) -> bytes:
        return json.dumps(asdict(self)).encode("utf-8")

    @classmethod
    def from_bytes(cls, b: bytes) -> "TelemetryPacket":
        return cls(**json.loads(b.decode("utf-8")))


class UDPTelemetrySender:
    def __init__(self, host: str = "127.0.0.1", port: int = 14550):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, packet: TelemetryPacket) -> None:
        try:
            self.sock.sendto(packet.to_bytes(), self.addr)
        except OSError:
            # Receiver may not be up yet; drop silently.
            pass

    def close(self) -> None:
        self.sock.close()


class UDPTelemetryReceiver:
    def __init__(self, host: str = "0.0.0.0", port: int = 14550, timeout: float = 0.5):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.settimeout(timeout)

    def recv(self) -> TelemetryPacket | None:
        try:
            data, _ = self.sock.recvfrom(4096)
        except socket.timeout:
            return None
        try:
            return TelemetryPacket.from_bytes(data)
        except (ValueError, TypeError):
            return None

    def close(self) -> None:
        self.sock.close()
