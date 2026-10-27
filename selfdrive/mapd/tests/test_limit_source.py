"""PR #391: speed limit source toggle, popup manual entry, > 20 mph Tesla-vs-OSM fallback."""
from openpilot.selfdrive.mapd.limit_source import (FALLBACK_DIFF_MPH, SOURCE_OSM_FALLBACK, choose_current,
                                                    manual_confirmed, parse_manual, step_mph)
from openpilot.selfdrive.mapd.speed_overrides import MANUAL_SOURCE, SpeedOverrides, parse_confirmed
from openpilot.selfdrive.mapd.tesla_limit import MPH_TO_MS
from openpilot.selfdrive.mapd.tests.test_speed_override_sections import limit, mk_db, pt, way_at

MPH = MPH_TO_MS
DEGRAFF = [(0, 500, 60, False), (500, 1000, 60, True), (1000, 1100, 30, True), (1100, 1600, 30, False)]
EB, WB = 90.0, 270.0


def mph(ms):
  return round(ms / MPH)


def test_default_order_unchanged():
  assert choose_current(None, 30, 55 * MPH, None, osm_tagged_mph=45)[1] == "tesla"
  assert choose_current(None, 0, 55 * MPH)[1] is None
  assert choose_current(None, 30, 55 * MPH, 35 * MPH, osm_tagged_mph=45)[1] == "tesla_learned"
  assert choose_current(40 * MPH, 30, 55 * MPH)[1:] == ("sign", False)


def test_prefer_osm_skips_tesla_and_learned_but_not_signs():
  ms, src, fb = choose_current(None, 30, 55 * MPH, 35 * MPH, prefer_osm=True, osm_tagged_mph=55)
  assert mph(ms) == 55 and src is None and not fb
  assert choose_current(40 * MPH, 30, 55 * MPH, prefer_osm=True)[1] == "sign"


def test_fallback_over_20_mph():
  # County Road 31 north of DeGraff: Tesla 30, OSM 55 -> OSM, flagged
  ms, src, fb = choose_current(None, 30, 55 * MPH, osm_tagged_mph=55)
  assert mph(ms) == 55 and src == SOURCE_OSM_FALLBACK and fb
  # exactly 20 is not over the threshold
  assert choose_current(None, 35, 55 * MPH, osm_tagged_mph=55)[1] == "tesla"
  assert FALLBACK_DIFF_MPH == 20
  # learned Tesla is checked the same way
  assert choose_current(None, 0, 55 * MPH, 30 * MPH, osm_tagged_mph=55)[1] == SOURCE_OSM_FALLBACK


def test_fallback_off_or_estimated_osm():
  assert choose_current(None, 30, 55 * MPH, fallback_on=False, osm_tagged_mph=55)[1] == "tesla"
  assert choose_current(None, 30, 55 * MPH, osm_tagged_mph=0)[1] == "tesla"   # estimated OSM: no fallback


def test_parse_manual_and_steps():
  assert parse_manual('{"mph": 55, "seq": 1}') == {"mph": 55, "seq": 1.0}
  assert parse_manual({"mph": 52, "seq": 1}) is None
  assert parse_manual({"mph": 90, "seq": 1}) is None
  assert parse_manual(None) is None
  assert step_mph(30, True) == 35 and step_mph(30, False) == 25
  assert step_mph(32, True) == 35 and step_mph(32, False) == 30
  assert step_mph(85, True) == 85 and step_mph(5, False) == 5


def test_manual_payload_needs_a_matched_way():
  assert manual_confirmed({"mph": 55, "seq": 1}, None, 1.0, 2.0, 90.0) is None
  assert manual_confirmed({"mph": 55, "seq": 1}, 7, None, None, 90.0) is None
  c = manual_confirmed({"mph": 55, "seq": 1}, 7, 1.0, 2.0, 90.0)
  assert c["manual"] and c["sign_lat"] == c["lat"] and c["way_id"] == 7
  assert parse_confirmed(c)["manual"] is True


def test_manual_row_covers_the_whole_map_stretch_both_directions(tmp_path):
  # Oct 27 2026: a manual Save covers the current map-limit stretch, behind the car too
  db = mk_db(tmp_path, DEGRAFF)
  o = SpeedOverrides(str(tmp_path / "o.json"))
  w = way_at(db, 1045)
  lat, lon = pt(1045)
  conf = manual_confirmed({"mph": 45, "seq": 5}, w["way_id"], lat, lon, WB)
  e = o.consume(conf, None, WB, 5.0, way_coords=db.way_coords, db=db)
  assert e is not None and e["source"] == MANUAL_SOURCE and e["sign_at"] == "manual_stretch" and "cut_m" not in e
  assert e["back_m"] > 500.0                # walked back to the 60 -> 30 change behind (x = 1600 end)
  for x in (1010, 1035, 1060, 1080, 1300, 1590):
    assert limit(o, db, x, WB) == 45, x      # ahead of and behind the car, up to each map change
    assert limit(o, db, x, EB) == 45, x
  assert limit(o, db, 900, WB) == 60        # past the next map limit change: untouched
  assert limit(o, db, 900, EB) == 60
  assert o.fix_starts(db) == []             # never moved at load
  gone = o.clear(w["way_id"], WB, *pt(1040), w["coords"])
  assert gone and limit(o, db, 1035, WB) == 30 and limit(o, db, 1300, WB) == 30


def test_manual_stretch_stops_at_the_change_behind(tmp_path):
  # 60 | 30 | 30 | 60: set at 1250 heading east, covers only the 30 stretch
  db = mk_db(tmp_path, [(0, 500, 60, False), (500, 1000, 30, True), (1000, 1500, 30, False), (1500, 2000, 60, False)])
  o = SpeedOverrides(str(tmp_path / "o.json"))
  w = way_at(db, 1250)
  e = o.consume(manual_confirmed({"mph": 40, "seq": 1}, w["way_id"], *pt(1250), EB), None, EB, 1.0,
                way_coords=db.way_coords, db=db)
  assert e is not None and e["sign_at"] == "manual_stretch"
  for x in (520, 900, 1250, 1480):
    assert limit(o, db, x, EB) == 40, x
  assert limit(o, db, 400, EB) == 60 and limit(o, db, 1600, EB) == 60


def test_sign_rows_still_never_behind_the_sign(tmp_path):
  from openpilot.selfdrive.mapd.tests.test_speed_override_sections import yes
  db = mk_db(tmp_path, DEGRAFF)
  o = SpeedOverrides(str(tmp_path / "o.json"))
  yes(o, db, 1300, 45, WB, 1)
  assert limit(o, db, 1290, WB) == 45 and limit(o, db, 1400, WB) == 30
