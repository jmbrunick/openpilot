"""Roundabout first slice: funnel detect, speed ease, outer bias, FP guard."""
import math

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import LOOKAHEAD_NORMAL
from openpilot.selfdrive.mapd.osm_db import EARTH_R, OsmSpeedLimitDB
from openpilot.selfdrive.mapd.overpass import ways_from_overpass
from openpilot.selfdrive.mapd.roundabout import (
  RB_FUNNEL_M,
  RB_FUNNEL_MIN_M,
  RB_OUTER_OFFSET_M,
  RB_OUTER_OFFSET_MAX_M,
  RB_OUTER_OFFSET_MIN_M,
  RB_V_DEFAULT_MS,
  RB_V_MAX_MS,
  RB_V_MIN_MS,
  RoundaboutHint,
  junction_is_roundabout,
  live_map_roundabout_hint,
  roundabout_ease_v_ms,
  roundabout_outer_curvature_bias,
  roundabout_outer_path_offset_m,
  roundabout_target_ms,
  way_is_closed_loop,
  way_is_roundabout,
  way_radius_m,
)

# Willmar-ish 30th St SW / 19th Ave SW ring: R ≈ 22.5 m, maxspeed 20.
RB_LAT = 45.1165
RB_LON = -95.0610
RB_R_M = 22.5


def _offset(lat, lon, bearing_deg, dist_m):
  theta = dist_m / EARTH_R
  brng = math.radians(bearing_deg)
  phi1 = math.radians(lat)
  lam1 = math.radians(lon)
  phi2 = math.asin(math.sin(phi1) * math.cos(theta) + math.cos(phi1) * math.sin(theta) * math.cos(brng))
  lam2 = lam1 + math.atan2(
    math.sin(brng) * math.sin(theta) * math.cos(phi1),
    math.cos(theta) - math.sin(phi1) * math.sin(phi2),
  )
  return math.degrees(phi2), (math.degrees(lam2) + 540.0) % 360.0 - 180.0


def _circle(lat0, lon0, r_m=RB_R_M, n=20):
  coords = [_offset(lat0, lon0, 360.0 * i / n, r_m) for i in range(n)]
  coords.append(coords[0])
  return coords


def _eastbound_way(lat, lon_west, lon_east, steps=4):
  return [(lat, lon_west + (lon_east - lon_west) * i / steps) for i in range(steps + 1)]


def test_closed_loop_radius_is_roundabout():
  ring = _circle(RB_LAT, RB_LON)
  assert way_is_closed_loop(ring)
  r = way_radius_m(ring)
  assert r is not None and abs(r - RB_R_M) < 3.0
  assert way_is_roundabout(ring, junction="")
  assert way_is_roundabout(ring, junction="roundabout")
  assert junction_is_roundabout("circular")


def test_tagged_arc_counts_as_roundabout():
  # Multi-way OSM rings are 90° arcs with junction=roundabout, not closed.
  arc = [_offset(RB_LAT, RB_LON, ang, RB_R_M) for ang in (0.0, 30.0, 60.0, 90.0)]
  assert not way_is_closed_loop(arc)
  assert way_is_roundabout(arc, junction="roundabout")
  assert not way_is_roundabout(arc, junction="")


def test_sharp_town_corner_is_not_roundabout():
  # 90° town corner — the false-positive we must not treat as an RB.
  corner = [
    (RB_LAT, RB_LON - 0.002),
    (RB_LAT, RB_LON),
    (RB_LAT + 0.002, RB_LON),
  ]
  assert not way_is_closed_loop(corner)
  assert not way_is_roundabout(corner, junction="")
  assert not way_is_roundabout(corner, highway="residential")


def test_signalized_cross_is_not_roundabout():
  ns = [(RB_LAT - 0.002, RB_LON), (RB_LAT + 0.002, RB_LON)]
  ew = [(RB_LAT, RB_LON - 0.002), (RB_LAT, RB_LON + 0.002)]
  assert not way_is_roundabout(ns, junction="")
  assert not way_is_roundabout(ew, junction="")


def test_service_bulb_without_tag_is_not_roundabout():
  bulb = _circle(RB_LAT, RB_LON, r_m=10.0)
  assert not way_is_roundabout(bulb, junction="", highway="service")
  assert way_is_roundabout(bulb, junction="roundabout", highway="service")


def test_target_clamps_osm_to_15_20():
  assert abs(roundabout_target_ms(0.0) - RB_V_DEFAULT_MS) < 1e-6
  assert abs(roundabout_target_ms(20.0 * CV.MPH_TO_MS) - 20.0 * CV.MPH_TO_MS) < 1e-6
  assert abs(roundabout_target_ms(15.0 * CV.MPH_TO_MS) - 15.0 * CV.MPH_TO_MS) < 1e-6
  assert roundabout_target_ms(25.0 * CV.MPH_TO_MS) == RB_V_MAX_MS
  assert roundabout_target_ms(10.0 * CV.MPH_TO_MS) == RB_V_MIN_MS


def test_funnel_eases_speed():
  hint = RoundaboutHint(
    approaching=True, distance_m=90.0, speed_limit_ms=20.0 * CV.MPH_TO_MS,
  )
  v49 = 49.0 * CV.MPH_TO_MS
  eased = roundabout_ease_v_ms(hint, v49, v49, LOOKAHEAD_NORMAL)
  assert eased is not None
  assert eased < v49 - 2.0
  assert eased >= RB_V_MIN_MS - 1e-6
  on_ring = RoundaboutHint(on_roundabout=True, speed_limit_ms=20.0 * CV.MPH_TO_MS)
  assert abs(roundabout_ease_v_ms(on_ring, v49, v49) - 20.0 * CV.MPH_TO_MS) < 1e-6
  assert 80.0 <= RB_FUNNEL_MIN_M <= RB_FUNNEL_M <= 100.0


def test_no_hint_does_not_ease():
  assert roundabout_ease_v_ms(None, 22.0, 22.0) is None
  assert roundabout_ease_v_ms(RoundaboutHint(), 22.0, 22.0) is None


def test_outer_bias_sign_and_magnitude():
  rht = roundabout_outer_path_offset_m(on_roundabout=True, is_rhd=False)
  lhd = roundabout_outer_path_offset_m(on_roundabout=True, is_rhd=True)
  assert rht < 0.0  # right / outer in US
  assert lhd > 0.0
  assert RB_OUTER_OFFSET_MIN_M <= abs(rht) <= RB_OUTER_OFFSET_MAX_M
  assert abs(abs(rht) - RB_OUTER_OFFSET_M) < 1e-6
  assert roundabout_outer_path_offset_m(on_roundabout=False) == 0.0
  kappa = roundabout_outer_curvature_bias(rht)
  assert kappa < 0.0
  assert abs(kappa) < 0.01  # in-lane nudge, not a lane change


def test_live_map_hint_reads_capnp_fields():
  class _Md:
    onRoundabout = False
    approachingRoundabout = True
    roundaboutDistance = 90.0
    roundaboutSpeedLimit = 20.0 * CV.MPH_TO_MS
    roundaboutWayId = 99

  hint = live_map_roundabout_hint(_Md())
  assert hint is not None and hint.approaching and abs(hint.distance_m - 90.0) < 1e-6
  assert live_map_roundabout_hint(None) is None

  class _Empty:
    onRoundabout = False
    approachingRoundabout = False

  assert live_map_roundabout_hint(_Empty()) is None


def _db_with_ring_and_approach(tmp_path, *, tagged=False, corner=False):
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  west_lat, west_lon = _offset(RB_LAT, RB_LON, 270.0, RB_R_M)
  approach_lat, approach_lon = _offset(west_lat, west_lon, 270.0, 160.0)
  OsmSpeedLimitDB.insert_way(
    con, 1, "30th St SW", "residential", 30.0 * CV.MPH_TO_MS,
    _eastbound_way(west_lat, approach_lon, west_lon),
  )
  if corner:
    OsmSpeedLimitDB.insert_way(
      con, 2, "19th Ave SW", "residential", 30.0 * CV.MPH_TO_MS,
      [
        (west_lat, west_lon - 0.0004),
        (RB_LAT, RB_LON),
        (RB_LAT + 0.002, RB_LON),
      ],
    )
  else:
    OsmSpeedLimitDB.insert_way(
      con, 2, "", "unclassified", 20.0 * CV.MPH_TO_MS,
      _circle(RB_LAT, RB_LON),
      junction="roundabout" if tagged else "",
    )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  return db, west_lat, west_lon


def test_funnel_detects_untagged_closed_ring(tmp_path):
  db, west_lat, west_lon = _db_with_ring_and_approach(tmp_path, tagged=False)
  qlat, qlon = _offset(west_lat, west_lon, 270.0, 90.0)
  hint = db.find_roundabout(qlat, qlon, bearing_deg=90.0)
  assert hint.approaching or hint.on_roundabout
  assert 0.0 < hint.distance_m <= RB_FUNNEL_M or hint.on_roundabout
  assert abs(hint.speed_limit_ms - 20.0 * CV.MPH_TO_MS) < 0.3
  # On the ring.
  on = db.find_roundabout(RB_LAT, RB_LON + 0.0, bearing_deg=0.0)
  # Center is ~22.5 m from the way — still on.
  assert on.on_roundabout
  db.close()


def test_tagged_ring_detects_in_funnel(tmp_path):
  db, west_lat, west_lon = _db_with_ring_and_approach(tmp_path, tagged=True)
  qlat, qlon = _offset(west_lat, west_lon, 270.0, 90.0)
  hint = db.find_roundabout(qlat, qlon, bearing_deg=90.0)
  assert hint.approaching
  assert hint.distance_m <= RB_FUNNEL_M
  db.close()


def test_sharp_corner_does_not_trigger_funnel(tmp_path):
  db, west_lat, west_lon = _db_with_ring_and_approach(tmp_path, corner=True)
  qlat, qlon = _offset(west_lat, west_lon, 270.0, 90.0)
  hint = db.find_roundabout(qlat, qlon, bearing_deg=90.0)
  assert not hint.approaching
  assert not hint.on_roundabout
  db.close()


def test_overpass_keeps_junction_tag():
  payload = {
    "elements": [
      {
        "type": "way",
        "id": 77,
        "tags": {"highway": "unclassified", "maxspeed": "20 mph", "junction": "roundabout"},
        "geometry": [{"lat": p[0], "lon": p[1]} for p in _circle(RB_LAT, RB_LON)],
      }
    ]
  }
  ways = ways_from_overpass(payload)
  assert len(ways) == 1
  assert ways[0]["junction"] == "roundabout"
