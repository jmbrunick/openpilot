"""A confirmed sign covers the road from the sign forward, both directions of travel, never behind it.

DeGraff MN, route 000001cd 08:51:55 (Oct 25 2026): a westbound 60 was read on Atlantic Ave (map 30)
before the reader located signs, so its section started at the read point, ~45 m east of the sign
at the town edge where the map turns 60 (US 12). Eastbound, MAX jumped to 59 on that 45 m."""
import json

from openpilot.selfdrive.mapd.speed_overrides import (SpeedOverrides, build_chain, chain_dist, sign_cut_m,
                                                       trim_chain)
from openpilot.selfdrive.mapd.tests.test_speed_override_sections import limit, mk_db, pt, way_at

# x east. US 12 map 60 to x=1000 (town edge), Atlantic Ave map 30 beyond. Ways digitized both ways.
DEGRAFF = [(0, 500, 60, False), (500, 1000, 60, True), (1000, 1100, 30, True), (1100, 1600, 30, False)]
EB, WB = 90.0, 270.0


def _read(o, db, x, mph, heading, seq, **extra):
  w = way_at(db, x)
  lat, lon = pt(x)
  conf = {"mph": mph, "seq": float(seq), "way_id": w["way_id"], "heading": heading, "lat": lat, "lon": lon, **extra}
  return o.consume(json.dumps(conf), None, heading, float(seq), way_coords=db.way_coords, db=db)


def test_degraff_unlocated_read_starts_at_the_town_edge(tmp_path):
  db = mk_db(tmp_path, DEGRAFF)
  o = SpeedOverrides(str(tmp_path / "o.json"))
  e = _read(o, db, 1045, 60, WB, 1)       # read 45 m east of the sign, no sign location (Oct 22 payload)
  assert e["sign_at"] == "map_change" and abs(e["cut_m"] - 45.0) < 1.5
  for x in (1005, 1020, 1040, 1080):
    assert limit(o, db, x, EB) == 30, x    # behind the sign: the eastbound spill is gone
    assert limit(o, db, x, WB) == 30, x    # ... and westbound before reaching it
  for x in (990, 700, 200):
    assert limit(o, db, x, WB) == 60, x    # ahead of the sign in its read direction
    assert limit(o, db, x, EB) == 60, x    # same stretch, opposite travel: still applies


def test_located_sign_starts_exactly_at_the_sign(tmp_path):
  db = mk_db(tmp_path, DEGRAFF)
  o = SpeedOverrides(str(tmp_path / "o.json"))
  e = _read(o, db, 1045, 60, WB, 1, sign_range_m=25.0)   # sign 25 m ahead -> x=1020
  assert e["sign_at"] == "located" and "cut_m" not in e
  assert limit(o, db, 1030, EB) == 30 and limit(o, db, 1030, WB) == 30
  assert limit(o, db, 1010, EB) == 60 and limit(o, db, 1010, WB) == 60


def test_no_map_evidence_keeps_the_read_point(tmp_path):
  db = mk_db(tmp_path, [(0, 500, 60, False), (500, 1000, 60, True)])
  o = SpeedOverrides(str(tmp_path / "o.json"))
  e = _read(o, db, 700, 50, EB, 1)          # map 60 both ways: nothing says where the sign is
  assert e["sign_at"] == "read" and "cut_m" not in e
  assert limit(o, db, 710, EB) == 50 and limit(o, db, 710, WB) == 50
  assert limit(o, db, 690, EB) == 60 and limit(o, db, 690, WB) == 60   # never behind the read


def test_old_row_fixed_at_load_and_logged_once(tmp_path):
  db = mk_db(tmp_path, DEGRAFF)
  w = way_at(db, 1045)
  s0 = abs(1045 - 1100)                     # way 1002 is digitized westward from x=1100
  chain = build_chain(w["way_id"], float(s0), 1, w["coords"], db, mph=60)
  row = {"v": 2, "way_id": w["way_id"], "heading": WB, "mph": 60, "both": True, "from_s": float(s0), "dir": 1,
         "chain": chain, "lat": pt(1045)[0], "lon": pt(1045)[1], "t": 1.0, "source": "sign_yes"}
  path = tmp_path / "o.json"
  path.write_text(json.dumps({"version": 2, "overrides": [row]}))
  o = SpeedOverrides(str(path))
  assert limit(o, db, 1040, EB) == 60       # the Oct 25 spill, before the fix
  fixed = o.fix_starts(db)
  assert len(fixed) == 1 and fixed[0]["chain_read"] == chain and abs(fixed[0]["cut_m"] - 45.0) < 1.5
  assert limit(o, db, 1040, EB) == 30 and limit(o, db, 1040, WB) == 30
  assert limit(o, db, 900, EB) == 60 and limit(o, db, 900, WB) == 60
  assert o.fix_starts(db) == []             # once


def test_located_rows_are_not_touched_at_load(tmp_path):
  db = mk_db(tmp_path, DEGRAFF)
  o = SpeedOverrides(str(tmp_path / "o.json"))
  e = _read(o, db, 1045, 60, WB, 1, sign_range_m=25.0)
  before = [list(sp) for sp in e["chain"]]
  assert o.fix_starts(db) == [] and e["chain"] == before


def test_trim_chain_handles_reversed_pieces():
  ch = [[1, 0.0, 50.0, 50.0, 0.0], [2, 0.0, 100.0, 50.0, 150.0]]
  assert trim_chain(ch, 20.0) == [[1, 0.0, 30.0, 30.0, 0.0], [2, 0.0, 100.0, 30.0, 130.0]]
  assert trim_chain(ch, 60.0) == [[2, 10.0, 100.0, 0.0, 90.0]]
  assert trim_chain(ch, 0.0) == ch


def test_end_tolerance_never_reaches_behind_the_sign():
  e = {"chain": [[7, 100.0, 300.0, 0.0, 200.0]]}
  assert chain_dist(e, 7, 99.5) is None      # 0.5 m behind the sign
  assert chain_dist(e, 7, 100.0) == 0.0
  assert chain_dist(e, 7, 300.5) == 200.0    # far end keeps its 1 m tolerance
  r = {"chain": [[7, 100.0, 300.0, 200.0, 0.0]]}   # read against the digitizing
  assert chain_dist(r, 7, 300.5) is None and chain_dist(r, 7, 99.5) == 200.0


def test_sign_cut_needs_the_map_to_move_toward_the_sign(tmp_path):
  db = mk_db(tmp_path, DEGRAFF)
  w = way_at(db, 1045)
  ch = build_chain(w["way_id"], 55.0, 1, w["coords"], db, mph=60)
  assert sign_cut_m(ch, 60, db)[1] == "map_change"
  assert sign_cut_m(ch, 25, db) == (0.0, "read")   # a 25 read on the 30 street: 60 is away from it
