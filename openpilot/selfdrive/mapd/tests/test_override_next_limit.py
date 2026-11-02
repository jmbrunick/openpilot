"""nextSpeedLimit follows confirmed sign sections (effective limit), not only OSM."""
from openpilot.selfdrive.mapd.speed_overrides import MPH_TO_MS, SpeedOverrides, effective_next
from openpilot.selfdrive.mapd.tests.test_speed_override_sections import BENSON, limit, mk_db, pt, short_ways, way_at, yes


def nxt(o, db, x, bearing=90.0, osm_next=None):
  """(next mph, distance m) mapd publishes standing at x, or None."""
  w = way_at(db, x)
  lat, lon = pt(x)
  ms = o.limit_ms(w["way_id"], bearing, lat, lon, w["coords"])
  cur = ms if ms is not None else w["maxspeed_ms"]
  osm = None if osm_next is None else (osm_next[0] * MPH_TO_MS, osm_next[1])
  r = effective_next(o.entries, db, w["way_id"], w["coords"], lat, lon, bearing, cur, ms is not None, osm)
  return None if r is None else (round(r[0] / MPH_TO_MS), r[1])


def benson(tmp_path):
  # map: 60 to 3000, 40 (wrong) to 5000, 30 after. Posted: 60, 50 @2000, 30 @3200.
  db = mk_db(tmp_path, short_ways(BENSON))
  o = SpeedOverrides(str(tmp_path / "o.json"))
  for i, (x, mph) in enumerate([(500, 60), (2000, 50), (3200, 30)]):
    yes(o, db, x, mph, 90.0, i + 1)
  return o, db


def test_benson_lookahead_targets_confirmed_50_then_30(tmp_path):
  o, db = benson(tmp_path)
  # 400 m before the 50 sign: OSM has no change for 1400 m, effective drops to 50 at the sign.
  m, dist = nxt(o, db, 1600)
  assert m == 50 and abs(dist - 400) <= 5
  # inside 50: OSM's 60 -> 40 at 3000 is masked... the section ends there and hands to OSM 40
  m, dist = nxt(o, db, 2700, osm_next=(40, 300))
  assert m == 40 and abs(dist - 300) <= 5
  # inside the masked 40 (OSM) stretch the 30 section starts at the sign
  m, dist = nxt(o, db, 3050, osm_next=(30, 1950))
  assert m == 30 and abs(dist - 150) <= 5
  # inside the 30 section: OSM 40 -> 30 at 5000 is the same value -> no change ahead
  assert nxt(o, db, 4700, osm_next=(30, 300)) is None


def test_osm_change_masked_by_override(tmp_path):
  db = mk_db(tmp_path, short_ways([(0, 3000, 55)]))
  o = SpeedOverrides(str(tmp_path / "o.json"))
  yes(o, db, 100, 45, 90.0, 1)    # 45 section over a long stretch the map has at 55
  # OSM says 35 at 300 m ahead (inside the section): masked, no change published
  assert nxt(o, db, 1000, osm_next=(35, 300)) is None
  assert limit(o, db, 1300) == 45


def test_section_end_hands_back_to_osm(tmp_path):
  db = mk_db(tmp_path, short_ways([(0, 1609, 25), (1609, 4000, 35)], step=100))
  o = SpeedOverrides(str(tmp_path / "o.json"))
  yes(o, db, 200, 30, 90.0, 1)
  m, dist = nxt(o, db, 1300, osm_next=(35, 309))
  assert m == 35 and abs(dist - 309) <= 5
  # before the sign: next is the confirmed 30 at the sign, not OSM's 35 a mile on
  m, dist = nxt(o, db, 50, osm_next=(35, 1559))
  assert m == 30 and abs(dist - 150) <= 5


def test_westbound_sees_sections(tmp_path):
  o, db = benson(tmp_path)
  # westbound at 3600 (30 section): the 30 section's west end is the 30 sign at 3200 -> map 40 there
  m, dist = nxt(o, db, 3600, bearing=270.0, osm_next=None)
  assert m == 40 and abs(dist - 400) <= 5


def test_no_sections_keeps_osm_next(tmp_path):
  db = mk_db(tmp_path, short_ways(BENSON))
  o = SpeedOverrides(str(tmp_path / "o.json"))
  assert nxt(o, db, 2700, osm_next=(40, 300)) == (40, 300)
  assert nxt(o, db, 2700, osm_next=None) is None
