"""Geodetic <-> local NED conversions using a flat-earth equirectangular
approximation about a fixed home point. Accurate enough for typical UAV
operating areas (a few km) and avoids a pyproj dependency.

NED axes: x-North, y-East, z-Down. Altitude (positive up) maps to -z (down).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

R_EARTH = 6378137.0  # WGS-84 equatorial radius [m]


@dataclass(frozen=True)
class GeoPoint:
    lat: float  # degrees
    lon: float  # degrees
    alt: float  # metres above home reference altitude


class GeodeticNEDConverter:
    """Converts between geodetic (lat, lon, alt) and a local NED frame
    anchored at a home point."""

    def __init__(self, home: GeoPoint):
        self.home = home
        self._lat0_rad = math.radians(home.lat)
        self._m_per_deg_lat = (math.pi / 180.0) * R_EARTH
        self._m_per_deg_lon = (math.pi / 180.0) * R_EARTH * math.cos(self._lat0_rad)

    def geo_to_ned(self, p: GeoPoint) -> tuple[float, float, float]:
        n = (p.lat - self.home.lat) * self._m_per_deg_lat
        e = (p.lon - self.home.lon) * self._m_per_deg_lon
        d = -(p.alt - self.home.alt)
        return n, e, d

    def ned_to_geo(self, n: float, e: float, d: float) -> GeoPoint:
        lat = self.home.lat + n / self._m_per_deg_lat
        lon = self.home.lon + e / self._m_per_deg_lon
        alt = self.home.alt - d
        return GeoPoint(lat, lon, alt)
