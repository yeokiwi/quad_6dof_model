"""Waypoint loading and management.

Waypoints are specified in geodetic coordinates (lat, lon, alt). The manager
converts them to local NED at construction time and yields the active target,
advancing when the vehicle is within ``accept_radius`` of the current target.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .coordinates import GeodeticNEDConverter, GeoPoint


@dataclass
class Waypoint:
    geo: GeoPoint
    ned: np.ndarray  # length-3 array


class WaypointManager:
    def __init__(self, geo_waypoints: list[GeoPoint],
                 converter: GeodeticNEDConverter,
                 accept_radius: float = 2.0,
                 hold_at_end: bool = True):
        if not geo_waypoints:
            raise ValueError("waypoint list is empty")
        self.converter = converter
        self.accept_radius = accept_radius
        self.hold_at_end = hold_at_end
        self.waypoints: list[Waypoint] = []
        for g in geo_waypoints:
            n, e, d = converter.geo_to_ned(g)
            self.waypoints.append(Waypoint(geo=g, ned=np.array([n, e, d])))
        self.index = 0

    @property
    def finished(self) -> bool:
        return self.index >= len(self.waypoints)

    def current(self) -> Waypoint:
        if self.finished:
            return self.waypoints[-1]
        return self.waypoints[self.index]

    def update(self, pos_ned: np.ndarray) -> Waypoint:
        if self.finished:
            return self.waypoints[-1]
        target = self.waypoints[self.index]
        if np.linalg.norm(pos_ned - target.ned) <= self.accept_radius:
            if self.index < len(self.waypoints) - 1:
                self.index += 1
            elif not self.hold_at_end:
                self.index += 1
        return self.current()


def load_waypoints(path: str | Path) -> tuple[GeoPoint, list[GeoPoint]]:
    """Load home + waypoints from a JSON file. Returns (home, waypoints).

    JSON schema::

        {
          "home": {"lat": ..., "lon": ..., "alt": ...},
          "waypoints": [{"lat": ..., "lon": ..., "alt": ...}, ...]
        }
    """
    data = json.loads(Path(path).read_text())
    home = GeoPoint(**data["home"])
    wps = [GeoPoint(**w) for w in data["waypoints"]]
    return home, wps
