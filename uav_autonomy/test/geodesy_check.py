#!/usr/bin/env python3
"""Exercise Gazebo's actual geodetic conversion against PX4 MapProjection."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from gz.math7 import SphericalCoordinates as S, Vector3d, Angle

ROOT = Path(__file__).resolve().parents[2]
for name in ('warehouse', 'forest_original', 'forest_long', 'forest_blocked'):
    c = ET.parse(ROOT/'assets/gazebo_world'/f'{name}.sdf').getroot().find('world/spherical_coordinates')
    assert c.findtext('surface_model') == 'CUSTOM_SURFACE'
    s = S(S.CUSTOM_SURFACE, float(c.findtext('surface_axis_equatorial')),
          float(c.findtext('surface_axis_polar')))
    lat0, lon0 = [math.radians(float(c.findtext(a))) for a in ('latitude_deg','longitude_deg')]
    s.set_latitude_reference(Angle(lat0)); s.set_longitude_reference(Angle(lon0))
    s.set_elevation_reference(float(c.findtext('elevation')))
    # Same azimuthal projection as PX4 src/lib/geo/geo.cpp, Earth radius
    # declared in geo.h. This test uses no recorded or live flight poses.
    for x, y in ((80., 0.), (0., 80.), (-80., 30.)):
        p = s.position_transform(Vector3d(x,y,0),S.LOCAL2,S.SPHERICAL)
        lat, lon = p.x(), p.y()
        arc = math.acos(max(-1.,min(1.,math.sin(lat0)*math.sin(lat)+math.cos(lat0)*math.cos(lat)*math.cos(lon-lon0))))
        k = arc/math.sin(arc) if arc>1e-12 else 1.
        north = 6371000*k*(math.cos(lat0)*math.sin(lat)-math.sin(lat0)*math.cos(lat)*math.cos(lon-lon0))
        east = 6371000*k*math.cos(lat)*math.sin(lon-lon0)
        assert math.hypot(east-x,north-y)<.01, (name,x,y,east,north)
print('Four simulator geodetic references match PX4 within 1 cm over 85 m: PASS')
