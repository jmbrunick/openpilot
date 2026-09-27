"""Current-road match must not jump to a parallel frontage road.

Stickiness keeps the previous way while it is close and heading-aligned.
A lower-class road far below ego speed has to persist ~5 s or ~150 m.
Same-road signed drops and the #53 cross-street cases stay immediate.
"""
import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import (
  DECREASE_START_MARGIN_M,
  LOOKAHEAD_NORMAL,
)
from openpilot.selfdrive.mapd.map_speed_policy import anticipatory_limit_ms
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB, _offset_point

MOTORWAY_ID = 1
FRONTAGE_ID = 2
RAMP_ID = 3
ORIGIN = (45.0, -94.0)
ROAD_BEARING = 90.0
FRONTAGE_M = 28.0


def _build(tmp_path, ways):
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  for way_id, name, highway, limit_mph, coords in ways:
    OsmSpeedLimitDB.insert_way(con, way_id, name, highway, limit_mph * CV.MPH_TO_MS, coords)
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  return db


def _line(start, bearing, length_m, n=5):
  step = float(length_m) / n
  return [_offset_point(start[0], start[1], bearing, step * i) for i in range(n + 1)]


def _north_of(lat, lon, meters):
  return _offset_point(lat, lon, 0.0, meters)


def _frontage_db(tmp_path, separation_m=FRONTAGE_M):
  motorway = _line(ORIGIN, ROAD_BEARING, 2500.0)
  frontage_start = _north_of(ORIGIN[0], ORIGIN[1], separation_m)
  frontage = _line(frontage_start, ROAD_BEARING, 2500.0)
  return _build(tmp_path, [
    (MOTORWAY_ID, "I-94", "motorway", 70.0, motorway),
    (FRONTAGE_ID, "Frontage", "tertiary", 25.0, frontage),
  ])


def _along(along_m, lateral_m=0.0):
  on_road = _offset_point(ORIGIN[0], ORIGIN[1], ROAD_BEARING, along_m)
  if lateral_m == 0.0:
    return on_road
  return _north_of(on_road[0], on_road[1], lateral_m)


def _sample(db, along_m, lateral_m, bearing, v_mph, now_s):
  lat, lon = _along(along_m, lateral_m)
  return db.lookup(
    lat, lon, bearing_deg=bearing, v_ego_ms=v_mph * CV.MPH_TO_MS, now_s=now_s,
  )


@pytest.mark.parametrize("bearing", [90.0, 270.0])
@pytest.mark.parametrize("lateral_m", [0.0, 10.0, 14.0, 20.0])
def test_parallel_frontage_stays_on_motorway(tmp_path, bearing, lateral_m):
  """GPS drift up to 20 m toward a 25 mph frontage 28 m away keeps the 70."""
  db = _frontage_db(tmp_path)
  try:
    along = 500.0
    now = 1000.0
    first = _sample(db, along, 0.0, bearing, 70.0, now)
    assert first is not None and first.way_id == MOTORWAY_ID
    for i in range(1, 5):
      now += 0.5
      along += 12.0
      m = _sample(db, along, lateral_m, bearing, 70.0, now)
      assert m is not None, (bearing, lateral_m, i)
      assert m.way_id == MOTORWAY_ID, (bearing, lateral_m, i, m.way_id, m.highway)
      assert abs(m.speed_limit_ms - 70.0 * CV.MPH_TO_MS) < 0.3
  finally:
    db.close()


@pytest.mark.parametrize("bearing", [90.0, 270.0])
def test_nearer_parallel_road_does_not_steal_without_clear_advantage(tmp_path, bearing):
  """Tertiary is nearer, but not 9 m nearer, and ego is not 20 mph over its limit.

  Nearest-wins would take the frontage. Stickiness must keep the motorway.
  """
  db = _frontage_db(tmp_path)
  try:
    # 16 m toward a road 28 m away: motorway 16 m, tertiary 12 m (4 m closer).
    lateral_m = 16.0
    along = 500.0
    now = 2000.0
    anchored = _sample(db, along, 0.0, bearing, 40.0, now)
    assert anchored is not None and anchored.way_id == MOTORWAY_ID
    for i in range(8):
      now += 0.5
      along += 15.0
      m = _sample(db, along, lateral_m, bearing, 40.0, now)
      assert m is not None
      assert m.way_id == MOTORWAY_ID, (bearing, i, m.way_id, m.distance_m)
  finally:
    db.close()


def test_exit_to_slower_road_switches_within_5s(tmp_path):
  """A real departure onto a lower road is accepted after persistence, within ~5 s."""
  gore = _offset_point(ORIGIN[0], ORIGIN[1], ROAD_BEARING, 400.0)
  ramp_bearing = 30.0
  motorway = _line(ORIGIN, ROAD_BEARING, 800.0)
  ramp = _line(gore, ramp_bearing, 500.0)
  db = _build(tmp_path, [
    (MOTORWAY_ID, "I-94", "motorway", 70.0, motorway),
    (RAMP_ID, "Exit", "tertiary", 25.0, ramp),
  ])
  try:
    now = 3000.0
    on_hwy = _offset_point(ORIGIN[0], ORIGIN[1], ROAD_BEARING, 200.0)
    anchored = db.lookup(on_hwy[0], on_hwy[1], bearing_deg=ROAD_BEARING, v_ego_ms=70.0 * CV.MPH_TO_MS, now_s=now)
    assert anchored is not None and anchored.way_id == MOTORWAY_ID

    def at_ramp(dist_m, t):
      lat, lon = _offset_point(gore[0], gore[1], ramp_bearing, dist_m)
      return db.lookup(lat, lon, bearing_deg=ramp_bearing, v_ego_ms=50.0 * CV.MPH_TO_MS, now_s=t)

    # Far enough down the ramp that the motorway is no longer the road under us.
    early = at_ramp(120.0, now + 0.5)
    assert early is None or early.way_id != RAMP_ID
    still = at_ramp(120.0, now + 0.5 + 4.0)
    assert still is None or still.way_id != RAMP_ID
    switched = at_ramp(120.0, now + 0.5 + 5.0)
    assert switched is not None
    assert switched.way_id == RAMP_ID
    assert abs(switched.speed_limit_ms - 25.0 * CV.MPH_TO_MS) < 0.3
  finally:
    db.close()


def test_exit_persists_by_distance_before_5s(tmp_path):
  """~150 m on the slower road accepts the match even when 5 s have not elapsed."""
  gore = _offset_point(ORIGIN[0], ORIGIN[1], ROAD_BEARING, 400.0)
  ramp_bearing = 30.0
  db = _build(tmp_path, [
    (MOTORWAY_ID, "I-94", "motorway", 70.0, _line(ORIGIN, ROAD_BEARING, 800.0)),
    (RAMP_ID, "Exit", "tertiary", 25.0, _line(gore, ramp_bearing, 800.0)),
  ])
  try:
    now = 4000.0
    on_hwy = _offset_point(ORIGIN[0], ORIGIN[1], ROAD_BEARING, 200.0)
    anchored = db.lookup(on_hwy[0], on_hwy[1], bearing_deg=ROAD_BEARING, v_ego_ms=55.0 * CV.MPH_TO_MS, now_s=now)
    assert anchored is not None and anchored.way_id == MOTORWAY_ID
    accepted = None
    for i in range(0, 20):
      dist = 120.0 + 10.0 * i
      now += 0.1
      lat, lon = _offset_point(gore[0], gore[1], ramp_bearing, dist)
      m = db.lookup(lat, lon, bearing_deg=ramp_bearing, v_ego_ms=55.0 * CV.MPH_TO_MS, now_s=now)
      if m is not None and m.way_id == RAMP_ID:
        accepted = (i, dist, now)
        break
      assert m is None or m.way_id != RAMP_ID
    assert accepted is not None
    _i, dist, t = accepted
    assert dist - 120.0 >= 140.0
    assert t - 4000.0 < 5.0
  finally:
    db.close()


def test_same_road_signed_drop_is_not_delayed(tmp_path):
  """70 → 55 on the same motorway still publishes next, then the new posted limit."""
  assert DECREASE_START_MARGIN_M == 110.0
  split = _offset_point(ORIGIN[0], ORIGIN[1], ROAD_BEARING, 600.0)
  end = _offset_point(split[0], split[1], ROAD_BEARING, 600.0)
  db = _build(tmp_path, [
    (MOTORWAY_ID, "I-94", "motorway", 70.0, [ORIGIN, split]),
    (4, "I-94", "motorway", 55.0, [split, end]),
  ])
  try:
    v70 = 70.0 * CV.MPH_TO_MS
    before = _offset_point(split[0], split[1], ROAD_BEARING + 180.0, 200.0)
    m = db.lookup(before[0], before[1], bearing_deg=ROAD_BEARING, v_ego_ms=v70, now_s=5000.0)
    assert m is not None
    assert m.way_id == MOTORWAY_ID
    assert abs(m.speed_limit_ms - v70) < 0.3
    assert abs(m.next_speed_limit_ms - 55.0 * CV.MPH_TO_MS) < 0.3, m.next_speed_limit_ms * CV.MS_TO_MPH
    assert m.next_distance_m > 40.0
    eased = anticipatory_limit_ms(
      m.speed_limit_ms, m.next_speed_limit_ms, m.next_distance_m, v70, LOOKAHEAD_NORMAL,
    )
    assert eased is not None
    assert m.next_speed_limit_ms <= eased < m.speed_limit_ms

    after = _offset_point(split[0], split[1], ROAD_BEARING, 40.0)
    m2 = db.lookup(after[0], after[1], bearing_deg=ROAD_BEARING, v_ego_ms=v70, now_s=5000.5)
    assert m2 is not None
    assert m2.way_id == 4
    assert abs(m2.speed_limit_ms - 55.0 * CV.MPH_TO_MS) < 0.3
  finally:
    db.close()


def test_dropout_lower_class_rematch_waits_for_persistence(tmp_path):
  """A minor road appearing after a dropout, far below ego speed, is not instant."""
  db = _build(tmp_path, [
    (MOTORWAY_ID, "I-94", "motorway", 70.0, _line(ORIGIN, ROAD_BEARING, 400.0)),
    (FRONTAGE_ID, "Oak", "residential", 25.0, _line((46.0, -94.0), ROAD_BEARING, 400.0)),
  ])
  try:
    v70 = 70.0 * CV.MPH_TO_MS
    on = _offset_point(ORIGIN[0], ORIGIN[1], ROAD_BEARING, 100.0)
    m = db.lookup(on[0], on[1], bearing_deg=ROAD_BEARING, v_ego_ms=v70, now_s=6000.0)
    assert m is not None and m.way_id == MOTORWAY_ID
    assert db.lookup(44.0, -94.0, bearing_deg=ROAD_BEARING, v_ego_ms=v70, now_s=6001.0) is None
    minor = _offset_point(46.0, -94.0, ROAD_BEARING, 100.0)
    early = db.lookup(minor[0], minor[1], bearing_deg=ROAD_BEARING, v_ego_ms=v70, now_s=6002.0)
    assert early is None or early.way_id != FRONTAGE_ID
    later = db.lookup(minor[0], minor[1], bearing_deg=ROAD_BEARING, v_ego_ms=v70, now_s=6007.0)
    assert later is not None
    assert later.way_id == FRONTAGE_ID
  finally:
    db.close()


def test_cross_street_still_does_not_steal_current_road(tmp_path):
  """#53 intersecting-road case: GPS on the side street, heading along the highway."""
  db = _build(tmp_path, [
    (MOTORWAY_ID, "Main", "primary", 60.0, [(37.0, -122.004), (37.0, -121.996)]),
    (FRONTAGE_ID, "Oak", "residential", 30.0, [(36.997, -122.000), (37.003, -122.000)]),
  ])
  try:
    qlat, qlon = _offset_point(37.0, -122.000, 0.0, 5.0)
    now = 7000.0
    for _ in range(4):
      m = db.lookup(qlat, qlon, bearing_deg=90.0, v_ego_ms=60.0 * CV.MPH_TO_MS, now_s=now)
      now += 0.5
      assert m is not None
      assert m.way_id == MOTORWAY_ID
      assert abs(m.speed_limit_ms - 60.0 * CV.MPH_TO_MS) < 0.3
      assert m.next_speed_limit_ms == 0.0
  finally:
    db.close()
