"""Map pack v4 roundabouts: rings + approach roads without maxspeed (rb_ways).

Fixture: Soco Rd / Dellwood Rd / Jonathan Creek Rd ring (Maggie Valley NC),
Sep 28 PM R126 16:52 / R128 17:38 CT. OSM data © OpenStreetMap contributors,
ODbL 1.0. The ring has no maxspeed and no lanes tag, so pack v3 (maxspeed-only
ways) never had it and the roundabout hint / slow-down never fired.
"""
import json
import math
import sqlite3
from pathlib import Path

import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.local_refresh import merge_ways_into_db
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB
from openpilot.selfdrive.mapd.overpass import overpass_query, ways_from_overpass
from openpilot.selfdrive.mapd.roundabout import roundabout_target_ms
from openpilot.selfdrive.mapd.roundabout_map import (
  PARAM_RING, ROLE_APPROACH, ROLE_RING, RingCache, RingGeometry, local_xy, rb_rows_from_overpass,
  ring_geometry, roundabout_comfort_speed_ms,
)

FIXTURE = Path(__file__).parent / "data" / "soco_roundabout_overpass.json"
SOCO_WAY = 1514082747
SOCO_RING_WAYS = {1514082747, 1517278969, 1517278970, 1517279058, 1517279059, 1517279060}
SOCO_CENTER = (35.52208, -83.02814)
# R128 17:37:59.9 CT, first mapd hint ~192 m east of the ring (westbound on Dellwood Rd).
R128_FIRST_HINT = (35.52224, -83.02602)
# A tagged speed way near the ring (synthetic, not in OSM) to prove speed matching is unchanged.
SPEED_WAY = {"way_id": 900000001, "name": "Test Rd", "highway": "primary", "maxspeed_ms": 45 * CV.MPH_TO_MS,
             "coords": [(35.5240, -83.0330), (35.5240, -83.0230)], "junction": ""}


def _payload():
  return json.loads(FIXTURE.read_text())


def _rows():
  return rb_rows_from_overpass(_payload())


def _ring(rows=None):
  rows = _rows() if rows is None else rows
  return ring_geometry([r for r in rows if r["role"] == ROLE_RING], [r for r in rows if r["role"] == ROLE_APPROACH],
                       seed_id=SOCO_WAY)


def _db(tmp_path, *, rb=True, name="pack.sqlite"):
  path = str(tmp_path / name)
  con = OsmSpeedLimitDB.create(path)
  w = SPEED_WAY
  OsmSpeedLimitDB.insert_way(con, w["way_id"], w["name"], w["highway"], w["maxspeed_ms"], w["coords"], junction="")
  if rb:
    for r in _rows():
      OsmSpeedLimitDB.insert_rb_way(con, r)
    OsmSpeedLimitDB.recount_rb_ways(con)
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  db.open()
  return db, path


def test_fixture_is_attributed_odbl():
  assert "OpenStreetMap contributors" in _payload()["attribution"]
  assert "ODbL" in _payload()["attribution"]


def test_soco_ring_and_approaches_are_extracted_without_maxspeed():
  rows = _rows()
  rings = {r["way_id"] for r in rows if r["role"] == ROLE_RING}
  approaches = {r["way_id"] for r in rows if r["role"] == ROLE_APPROACH}
  assert rings == SOCO_RING_WAYS
  assert SOCO_WAY in rings
  assert {1514082743, 1514082744, 1514082745, 1514082746} <= approaches   # Dellwood / Soco both directions
  assert all(r["maxspeed_ms"] == 0.0 for r in rows)
  # None of these reach the speed-limit ways (no maxspeed): pack v3 never had the ring.
  assert ways_from_overpass(_payload()) == []


def test_soco_ring_fit():
  ring = _ring()
  assert ring is not None
  assert 21.0 < ring.radius_m < 22.3
  assert ring.rms_m < 0.6
  assert ring.ccw                      # US counter-clockwise circulation
  assert ring.coverage_deg > 330.0
  assert set(ring.way_ids) == SOCO_RING_WAYS
  x, y = local_xy(ring.lat, ring.lon, *SOCO_CENTER)
  assert math.hypot(x, y) < 3.0


def test_approaches_are_stored_in_travel_direction():
  ring = _ring()
  inbound = outbound = 0
  for a in ring.approaches:
    r0, r1 = math.hypot(*a[0]), math.hypot(*a[-1])
    inbound += r1 < r0
    outbound += r1 > r0
  # Oneway Dellwood/Soco pairs: one inbound + one outbound each side.
  assert inbound >= 2 and outbound >= 2
  # Westbound Dellwood (1514082743) enters the ring: its polyline must end at the ring.
  rows = [r for r in _rows() if r["way_id"] == 1514082743]
  single = ring_geometry([r for r in _rows() if r["role"] == ROLE_RING], rows, seed_id=SOCO_WAY)
  a = single.approaches[0]
  assert math.hypot(*a[-1]) < math.hypot(*a[0])


def test_ring_json_roundtrip():
  ring = _ring()
  back = RingGeometry.from_json(json.dumps(ring.to_json()))
  assert back is not None
  assert back.way_ids == ring.way_ids and back.ccw == ring.ccw
  assert abs(back.radius_m - ring.radius_m) < 0.01
  assert len(back.approaches) == len(ring.approaches)
  assert RingGeometry.from_json({"v": 999}) is None
  assert RingGeometry.from_json("not json") is None
  assert PARAM_RING == "NAPRoundaboutRing"


def test_comfort_speed_soco_about_15_mph():
  ring = _ring()
  v = roundabout_comfort_speed_ms(ring.radius_m, ring.lanes, ring.maxspeed_ms)
  assert 15.0 * CV.MPH_TO_MS <= v <= 16.5 * CV.MPH_TO_MS
  # Inner lane (2 lanes, lanes tag missing) at that speed: <= 2.5 m/s^2.
  assert v * v / (ring.radius_m - 1.75) <= 2.5 + 1e-6
  # Planner target keeps it (15-20 mph clamp).
  assert abs(roundabout_target_ms(v) - v) < 1e-6
  # OSM maxspeed still caps; bigger rings are faster; bad input safe.
  assert roundabout_comfort_speed_ms(21.6, 2, 5.0) == pytest.approx(5.0)
  assert roundabout_comfort_speed_ms(40.0, 2) > v
  assert roundabout_comfort_speed_ms(0.0, 2) >= 0.0


def test_v4_pack_ring_geometry_from_first_hint_distance(tmp_path):
  db, _ = _db(tmp_path)
  hint = db.find_roundabout(R128_FIRST_HINT[0], R128_FIRST_HINT[1], 270.0, current_highway="primary")
  # Hint fires from the approach, then geometry is fit around the hinted way (not the car ~190 m out).
  ring = db.ring_geometry(SOCO_WAY, *R128_FIRST_HINT)
  assert ring is not None and set(ring.way_ids) == SOCO_RING_WAYS
  assert hint.approaching and int(hint.way_id) in SOCO_RING_WAYS
  assert 150.0 < hint.distance_m < 200.0


def test_v4_pack_find_roundabout_sees_untagged_speed_ring(tmp_path):
  db, _ = _db(tmp_path)
  lat, lon = SOCO_CENTER
  x_east = local_xy(lat, lon + 0.0006, lat, lon)[0]
  assert x_east > 40
  h = db.find_roundabout(lat, lon + 0.0006, 270.0, current_highway="primary")
  assert h.approaching or h.on_roundabout
  assert int(h.way_id) in SOCO_RING_WAYS


def test_v3_pack_without_rb_tables_still_works(tmp_path):
  db, path = _db(tmp_path, rb=False, name="v3.sqlite")
  con = sqlite3.connect(path)
  con.execute("DROP TABLE IF EXISTS rb_ways")
  con.execute("DROP TABLE IF EXISTS rb_ways_rtree")
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  db.open()
  assert db.ring_geometry(SOCO_WAY, *SOCO_CENTER) is None
  m = db.lookup(35.5240, -83.0280, 90.0, v_ego_ms=20.0)
  assert m is not None and m.speed_limit_ms == pytest.approx(SPEED_WAY["maxspeed_ms"])
  h = db.find_roundabout(*SOCO_CENTER, 90.0, current_highway="primary")
  assert not h.on_roundabout


def test_speed_matching_unchanged_by_rb_rows(tmp_path):
  with_rb, _ = _db(tmp_path, rb=True, name="a.sqlite")
  without, _ = _db(tmp_path, rb=False, name="b.sqlite")
  for lat, lon, brg in ((35.5240, -83.0280, 90.0), (35.5240, -83.0300, 270.0), (35.52224, -83.02602, 270.0),
                        (35.52208, -83.02814, 0.0)):
    a = with_rb.lookup(lat, lon, brg, v_ego_ms=15.0)
    b = without.lookup(lat, lon, brg, v_ego_ms=15.0)
    assert (a is None) == (b is None)
    if a is not None:
      assert a.way_id == b.way_id and a.speed_limit_ms == b.speed_limit_ms
  # rb rows never land in the speed `ways` table.
  con = sqlite3.connect(str(tmp_path / "a.sqlite"))
  assert con.execute("SELECT COUNT(*) FROM ways").fetchone()[0] == 1
  assert con.execute("SELECT COUNT(*) FROM rb_ways WHERE role='ring'").fetchone()[0] == 6
  assert con.execute("SELECT value FROM meta WHERE key='rb_ring_way_count'").fetchone()[0] == "6"


def test_overpass_queries_include_rings_and_approaches():
  for unmarked in (False, True):
    q = overpass_query(35.0, -84.0, 36.0, -83.0, include_unmarked=unmarked)
    assert '["junction"~"^(roundabout|circular)$"]' in q
    assert "node(w.rb)->.rbn;" in q
    assert 'way(bn.rbn)["highway"]->.ap;' in q
    assert "\n  .rb;\n  .ap;" in q        # both sets are output in the union
  # Old form still available.
  q = overpass_query(35.0, -84.0, 36.0, -83.0, include_roundabouts=False)
  assert "junction" not in q and '["maxspeed"]' in q


def test_extra_rb_elements_do_not_change_speed_ways():
  base = {"elements": [{"type": "way", "id": SPEED_WAY["way_id"], "tags": {"highway": "primary", "maxspeed": "45 mph"},
                        "geometry": [{"lat": a, "lon": b} for a, b in SPEED_WAY["coords"]]}]}
  combined = {"elements": base["elements"] + _payload()["elements"]}
  assert ways_from_overpass(combined) == ways_from_overpass(base)


def test_refresh_merge_replaces_rb_rows_in_bbox(tmp_path):
  _, path = _db(tmp_path, rb=False, name="installed.sqlite")
  bbox = (35.50, -83.05, 35.55, -83.00)
  merge_ways_into_db(path, [SPEED_WAY], bbox, rb_rows=_rows())
  con = sqlite3.connect(path)
  assert con.execute("SELECT COUNT(*) FROM rb_ways WHERE role='ring'").fetchone()[0] == 6
  # Second refresh replaces (no duplicates); None leaves rb_ways alone.
  merge_ways_into_db(path, [SPEED_WAY], bbox, rb_rows=_rows())
  merge_ways_into_db(path, [SPEED_WAY], bbox, rb_rows=None)
  assert con.execute("SELECT COUNT(*) FROM rb_ways WHERE role='ring'").fetchone()[0] == 6
  con.close()
  db = OsmSpeedLimitDB(path)
  db.open()
  assert db.ring_geometry(SOCO_WAY, *R128_FIRST_HINT) is not None


def test_ring_cache_publishes_once(tmp_path):
  db, _ = _db(tmp_path)
  cache, sent = RingCache(), []
  for t in range(5):
    g = cache.update(db, SOCO_WAY, *R128_FIRST_HINT, float(t), publish=sent.append)
    assert g is not None
  cache.update(db, 1517278969, *SOCO_CENTER, 6.0, publish=sent.append)   # another way of the same ring
  assert len(sent) == 1 and sent[0]["ids"] == sorted(SOCO_RING_WAYS)
  assert cache.update(db, 0, *SOCO_CENTER, 7.0) is None


def test_builder_from_json_writes_pack_v4(tmp_path):
  from scripts.nap.build_osm_speed_limits import main as build_main
  src = tmp_path / "in.json"
  payload = _payload()
  payload["elements"].append({"type": "way", "id": SPEED_WAY["way_id"], "tags": {"highway": "primary", "maxspeed": "45 mph"},
                              "nodes": [1, 2], "geometry": [{"lat": a, "lon": b} for a, b in SPEED_WAY["coords"]]})
  src.write_text(json.dumps(payload))
  out = tmp_path / "out.sqlite"
  assert build_main(["--from-json", str(src), "--out", str(out)]) in (0, None)
  con = sqlite3.connect(str(out))
  assert con.execute("SELECT COUNT(*) FROM ways").fetchone()[0] == 1
  assert con.execute("SELECT COUNT(*) FROM rb_ways WHERE role='ring'").fetchone()[0] == 6
  assert con.execute("SELECT value FROM meta WHERE key='pack_schema'").fetchone()[0] == "4"
  con.close()
  out2 = tmp_path / "out2.sqlite"
  assert build_main(["--from-json", str(src), "--out", str(out2), "--no-roundabouts"]) in (0, None)
  con = sqlite3.connect(str(out2))
  assert con.execute("SELECT COUNT(*) FROM ways").fetchone()[0] == 1
  assert con.execute("SELECT value FROM meta WHERE key='pack_schema'").fetchone() is None
  con.close()
