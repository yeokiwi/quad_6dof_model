"""UDP telemetry sender/receiver. Packets are JSON-encoded for easy debugging.

Both unicast and multicast destinations are supported transparently: pass a
multicast group address (224.0.0.0/4, e.g. 239.0.0.1) as ``host`` and the
sender configures TTL + loopback while the receiver joins the group. Any
number of receivers may join the same group and all see the same stream.
"""
from __future__ import annotations

import ipaddress
import json
import socket
import struct
from dataclasses import asdict, dataclass


def is_multicast(host: str) -> bool:
    try:
        return ipaddress.ip_address(host).is_multicast
    except ValueError:  # hostname, not a literal IP
        return False


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
    status: str = "running"  # waiting | running | paused

    def to_bytes(self) -> bytes:
        return json.dumps(asdict(self)).encode("utf-8")

    @classmethod
    def from_bytes(cls, b: bytes) -> "TelemetryPacket":
        return cls(**json.loads(b.decode("utf-8")))


class UDPTelemetrySender:
    def __init__(self, host: str = "239.0.0.1", port: int = 14550,
                 mcast_ttl: int = 1):
        self.addr = (host, port)
        self.multicast = is_multicast(host)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if self.multicast:
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL,
                                 struct.pack("b", mcast_ttl))
            # Loopback on so receivers on this host get the stream too.
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)

    def send(self, packet: TelemetryPacket) -> None:
        try:
            self.sock.sendto(packet.to_bytes(), self.addr)
        except OSError:
            # Receiver may not be up yet / no route; drop silently.
            pass

    def close(self) -> None:
        self.sock.close()


class UDPTelemetryReceiver:
    def __init__(self, host: str = "239.0.0.1", port: int = 14550,
                 timeout: float = 0.5):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if is_multicast(host):
            # Bind the wildcard address on the shared port, then join the
            # group. Membership is added on the default interface and, best
            # effort, on loopback so same-host delivery works even without a
            # multicast-capable default route.
            self.sock.bind(("", port))
            group = socket.inet_aton(host)
            memberships = [
                struct.pack("4sl", group, socket.INADDR_ANY),
                group + socket.inet_aton("127.0.0.1"),
            ]
            joined = 0
            for mreq in memberships:
                try:
                    self.sock.setsockopt(socket.IPPROTO_IP,
                                         socket.IP_ADD_MEMBERSHIP, mreq)
                    joined += 1
                except OSError:
                    pass
            if not joined:
                raise OSError(f"could not join multicast group {host}")
        else:
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
