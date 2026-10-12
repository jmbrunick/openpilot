"""Non-road ways (golf-cart paths etc.) are never the posted speed limit.

Oct 1-3 routes near a golf course: the real road (`highway=residential`) had no
`maxspeed` in OSM, so it was not in the pack. The only stored way within
MAX_MATCH_DISTANCE_M was a `highway=path golf=path maxspeed=5 mph` cart path, and
HUD MAX locked to 5 mph. The class filter lives in `_best_match` (so
`_next_limit` and the min-zone probes inherit it) and NOT in `_candidates()`,
which roundabout ring detection shares. There is deliberately no mph floor.
"""
import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB, _offset_point

ORIGIN = (35.5225, -83.0510)
BEARING = 90.0
ROAD_ID = 10
PATH_ID = 20
NON_ROAD = ("path", "footway", "cycleway", "bridleway", "steps", "pedestrian", "track")


def _build(tmp_path, ways):
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  for way_id, name, highway, mph, coords, *junction in ways:
    OsmSpeedLimitDB.insert_way(
      con, way_id, name, highway, mph * CV.MPH_TO_MS, coords, junction=(junction[0] if junction else ""),
    )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  return db


def _line(start, bearing, length_m, n=8):
  step = float(length_m) / n
  return [_offset_point(start[0], start[1], bearing, step * i) for i in range(n + 1)]


def _beside(lateral_m, length_m=600.0, start_along=0.0):
  """Parallel line `lateral_m` north of the road axis."""
  base = _offset_point(ORIGIN[0], ORIGIN[1], BEARING, start_along)
  return _line(_offset_point(base[0], base[1], 0.0, lateral_m), BEARING, length_m)


def _lookup(db, along_m, v_mph=30.0, now=100.0):
  lat, lon = _offset_point(ORIGIN[0], ORIGIN[1], BEARING, along_m)
  return db.lookup(lat, lon, bearing_deg=BEARING, v_ego_ms=v_mph * CV.MPH_TO_MS, now_s=now)


@pytest.mark.parametrize("highway", NON_ROAD)
def test_non_road_way_alone_is_not_matched(tmp_path, highway):
  """The logged case: only the cart path is stored near the untagged road."""
  db = _build(tmp_path, [(PATH_ID, "", highway, 5.0, _beside(8.0))])
  try:
    assert _lookup(db, 300.0) is None
  finally:
    db.close()


@pytest.mark.parametrize("highway", NON_ROAD)
def test_non_road_way_does_not_beat_a_farther_tagged_road(tmp_path, highway):
  db = _build(tmp_path, [
    (PATH_ID, "", highway, 5.0, _beside(4.0)),
    (ROAD_ID, "Country Club Drive", "residential", 25.0, _beside(24.0)),
  ])
  try:
    m = _lookup(db, 300.0)
    assert m is not None
    assert m.way_id == ROAD_ID
    assert m.speed_limit_ms == pytest.approx(25.0 * CV.MPH_TO_MS)
  finally:
    db.close()


@pytest.mark.parametrize("highway", ["service", "residential", "unclassified", "living_street", "tertiary"])
def test_low_limit_on_a_road_class_is_still_matched(tmp_path, highway):
  """Class filter only: no mph floor. A tagged 10 mph road keeps its limit."""
  db = _build(tmp_path, [(ROAD_ID, "Lot Ln", highway, 10.0, _beside(3.0))])
  try:
    m = _lookup(db, 300.0, v_mph=8.0)
    assert m is not None and m.way_id == ROAD_ID
    assert m.speed_limit_ms == pytest.approx(10.0 * CV.MPH_TO_MS)
  finally:
    db.close()


def _road_then(highway, mph, name=""):
  main = _line(ORIGIN, BEARING, 400.0, n=16)
  cont = _line(main[-1], BEARING, 400.0, n=16)
  return [
    (ROAD_ID, "Main St", "primary", 45.0, main),
    (PATH_ID, name, highway, mph, cont),
  ]


def test_next_limit_follows_a_road_class_continuation(tmp_path):
  """Control: the same geometry with a real road ahead does publish next."""
  db = _build(tmp_path, _road_then("tertiary", 25.0))
  try:
    m = _lookup(db, 250.0, v_mph=45.0)
    assert m is not None and m.way_id == ROAD_ID
    assert m.next_speed_limit_ms == pytest.approx(25.0 * CV.MPH_TO_MS, abs=0.3)
  finally:
    db.close()


@pytest.mark.parametrize("highway", NON_ROAD)
def test_next_limit_ignores_a_non_road_way_ahead(tmp_path, highway):
  """A cart path continuing the road line must not become nextSpeedLimit (early ease)."""
  db = _build(tmp_path, _road_then(highway, 5.0))
  try:
    m = _lookup(db, 250.0, v_mph=45.0)
    assert m is not None and m.way_id == ROAD_ID
    assert m.next_speed_limit_ms == 0.0
    assert m.next_distance_m == 0.0
  finally:
    db.close()


def test_junction_tagged_ring_is_found_even_in_a_non_road_class(tmp_path):
  """Roundabout ring detection shares `_candidates()`; the class filter must not live there."""
  ring_c = _offset_point(ORIGIN[0], ORIGIN[1], BEARING, 400.0)
  ring = [_offset_point(ring_c[0], ring_c[1], 360.0 * i / 20, 22.5) for i in range(20)]
  ring.append(ring[0])
  db = _build(tmp_path, [(PATH_ID, "", "cycleway", 20.0, ring, "roundabout")])
  try:
    lat, lon = _offset_point(ORIGIN[0], ORIGIN[1], BEARING, 300.0)
    hint = db.find_roundabout(lat, lon, bearing_deg=BEARING)
    assert hint.approaching or hint.on_roundabout
    assert hint.way_id == PATH_ID
  finally:
    db.close()
