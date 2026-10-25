"""Confirmed sign sections start at the sign and carry past the map's change there.

Oct 23 2026 drives:
  * Clarkfield, US 59 southbound (17:41:57 CDT): a 60 read on '13th Street' (map 30)
    1 m before that way ends; the next way is 'US Highway 59' (map 55). The section
    stopped at the end of 13th Street, so LIMIT showed 55 for 26 s.
  * DeGraff, US 12 eastbound (16:40:24 CDT): a 30 read on 'US 12' (map 60) right
    before 'Atlantic Avenue' (map 30) began; same failure.
"""
import json
import math

from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB
from openpilot.selfdrive.mapd.speed_overrides import (MPH_TO_MS, SpeedOverrides, build_chain, effective_next,
                                                      parse_confirmed, request_override, route_keys)

LAT0, LON0 = 44.7900, -95.8000
M_LAT = 111320.0
M_LON = 111320.0 * math.cos(math.radians(LAT0))


def ew(x):   # x meters east
  return (LAT0, LON0 + x / M_LON)


def ns(y):   # y meters north
  return (LAT0 + y / M_LAT, LON0)


class Road:
  """Ways laid along one axis: [(way_id, a, b, mph, name, ref, reversed)] (a < b, meters)."""

  def __init__(self, tmp_path, axis, pieces, extra=()):
    self.axis = axis
    path = str(tmp_path / "osm.db")
    con = OsmSpeedLimitDB.create(path)
    self.pieces = pieces
    for wid, a, b, mph, name, ref, rev in pieces:
      c = [axis(a), axis((a + b) / 2), axis(b)]
      OsmSpeedLimitDB.insert_way(con, wid, name, "primary", mph * MPH_TO_MS, c[::-1] if rev else c, ref=ref)
    for wid, coords, mph, name in extra:   # side roads
      OsmSpeedLimitDB.insert_way(con, wid, name, "residential", mph * MPH_TO_MS, coords)
    con.commit()
    con.close()
    self.db = OsmSpeedLimitDB(path)
    assert self.db.open()
    self.o = SpeedOverrides(str(tmp_path / "o.json"))

  def way_at(self, u):
    for wid, a, b, *_ in self.pieces:
      if a - 0.5 <= u <= b + 0.5:
        return self.db.way_info(wid)
    raise AssertionError(u)

  def limit(self, u, bearing):
    w = self.way_at(u)
    lat, lon = self.axis(u)
    ms = self.o.limit_ms(w["way_id"], bearing, lat, lon, w["coords"])
    return round((ms if ms is not None else w["maxspeed_ms"]) / MPH_TO_MS)

  def yes(self, u, mph, heading, seq, sign_u=None, sign_range_m=None, auto=None):
    w = self.way_at(u)
    lat, lon = self.axis(u)
    rec = {"lat": lat, "lon": lon, "bearing": heading}
    if sign_u is not None:
      rec["sign_lat"], rec["sign_lon"] = self.axis(sign_u)
    if sign_range_m is not None:
      rec["sign_range_m"] = sign_range_m

    class P:
      def put(self, k, v):
        self.v = v
    p = P()
    assert request_override(mph, rec, float(seq), params=p, way_id=w["way_id"], heading=heading, auto=auto)
    return self.o.consume(json.dumps(p.v), None, heading, float(seq), way_coords=self.db.way_coords, db=self.db)


# Clarkfield, southbound (heading 180). Ways digitized northward (reversed=False means south->north
# here since a < b is south < north). 13th Street (30) from y=0 to y=400 in town, US Highway 59 (55)
# south of town from -3000 to 0 (two ways), then a 45 zone further south.
def clarkfield(tmp_path):
  return Road(tmp_path, ns, [
    (794981138, 0, 400, 30, "13th Street", "US 59", False),
    (18299620, -1500, 0, 55, "US Highway 59", "US 59", True),
    (18299621, -3000, -1500, 55, "US Highway 59", "US 59", False),
    (18299622, -4000, -3000, 45, "US Highway 59", "US 59", False),
  ])


def test_clarkfield_60_at_town_line_carries_onto_us_highway_59(tmp_path):
  r = clarkfield(tmp_path)
  # read 1 m before 13th Street ends, sign located 25 m ahead (on US Highway 59)
  e = r.yes(1.0, 60, 180.0, 1, sign_u=-24.0)
  assert e is not None and e["source"] == "sign_yes"
  assert e["way_id"] == 18299620                      # starts on the next way (the sign is there)
  assert abs(e["lat"] - ns(-24.0)[0]) < 2e-5          # row is keyed to the sign, not the car
  assert e["read_lat"] == ns(1.0)[0]
  assert r.limit(50, 180.0) == 30                     # behind the sign: map
  assert r.limit(-10, 180.0) == 55                    # between the car and the sign: still map
  for y in (-30, -600, -1490, -1510, -2990):
    assert r.limit(y, 180.0) == 60, y                 # was 55 for 26 s on Oct 23
  assert r.limit(-3100, 180.0) == 45                  # stops at the next map change
  assert r.limit(-600, 0.0) == 55                     # other direction: map (Oct 25: direction-aware)


def test_clarkfield_old_payload_without_sign_location_still_carries(tmp_path):
  r = clarkfield(tmp_path)
  e = r.yes(1.0, 60, 180.0, 1)                        # no sign_lat / sign_range_m: starts at the read
  assert e["way_id"] == 794981138
  assert [sp[0] for sp in e["chain"]] == [794981138, 18299620, 18299621]
  assert r.limit(-1000, 180.0) == 60 and r.limit(-3100, 180.0) == 45


def test_sign_range_only_starts_ahead(tmp_path):
  r = clarkfield(tmp_path)
  e = r.yes(1.0, 60, 180.0, 1, sign_range_m=30.0)
  assert e["way_id"] == 18299620 and r.limit(-10, 180.0) == 55 and r.limit(-40, 180.0) == 60


def test_auto_confirm_uses_the_same_path(tmp_path):
  r = clarkfield(tmp_path)
  e = r.yes(1.0, 60, 180.0, 1, sign_u=-24.0, auto="tesla")
  assert e["source"] == "sign_auto_tesla" and r.limit(-1000, 180.0) == 60


def test_same_sign_again_replaces_row(tmp_path):
  r = clarkfield(tmp_path)
  r.yes(1.0, 60, 180.0, 1, sign_u=-24.0)
  r.yes(20.0, 60, 180.0, 2, sign_u=-26.0)            # read earlier, same sign
  assert len(r.o.entries) == 1


# DeGraff, eastbound US 12 (heading 90): US 12 trunk (60) to x=1000, Atlantic Avenue (30) through
# town to 1800, then US 12 (55).
def degraff(tmp_path):
  return Road(tmp_path, ew, [
    (18267060, 0, 1000, 60, "US 12", "US 12", False),
    (1227205098, 1000, 1400, 30, "Atlantic Avenue", "", False),
    (18267487, 1400, 1800, 30, "Atlantic Avenue", "", True),
    (18267488, 1800, 4000, 55, "US 12", "US 12", False),
  ])


def test_degraff_30_carries_onto_atlantic_avenue(tmp_path):
  r = degraff(tmp_path)
  e = r.yes(985.0, 30, 90.0, 1, sign_u=998.0)        # sign just before the way end
  assert [sp[0] for sp in e["chain"]] == [18267060, 1227205098, 18267487]   # was US 12's last 15 m only
  assert r.limit(900, 90.0) == 60
  for x in (1005, 1300, 1700, 1795):
    assert r.limit(x, 90.0) == 30, x
  assert r.limit(1900, 90.0) == 55                    # next map change ends it


def test_degraff_westbound_60_starts_at_its_sign_not_in_town(tmp_path):
  # Oct 22 18:33:40: a westbound 60 read on Atlantic Avenue 31 m before the sign. The row used to
  # start at the car (in the 30 zone) and cover both directions there; Oct 23 eastbound saw 60 on
  # Atlantic Avenue right after the 30 sign.
  r = degraff(tmp_path)
  e = r.yes(1060.0, 60, 270.0, 1, sign_u=1029.0)
  assert e["way_id"] == 1227205098
  assert r.limit(1045, 90.0) == 30 and r.limit(1045, 270.0) == 30   # in town, behind the sign
  assert r.limit(1015, 270.0) == 60 and r.limit(500, 270.0) == 60   # from the sign on (onto US 12)
  r.yes(985.0, 30, 90.0, 2, sign_u=998.0)                          # the eastbound 30 sign
  assert r.limit(1045, 90.0) == 30 and r.limit(1300, 90.0) == 30


def test_degraff_westbound_60_not_applied_eastbound(tmp_path):
  # Oct 25 2026: route 000001cd 08:51:55 CDT, turning eastbound onto Atlantic Avenue the Oct 22
  # westbound 60 section (sign at 1029, covering west onto US 12) applied for ~60 m (SIGN 60 vs
  # Tesla/OSM 30) and MAX jumped to 59. A confirmed sign now applies only in its read direction.
  r = degraff(tmp_path)
  r.yes(1060.0, 60, 270.0, 1, sign_u=1029.0)
  for x in (1028, 1015, 1005):
    assert r.limit(x, 270.0) == 60, x                  # westbound: the sign's section
    for hdg in (90.0, 60.0, 120.0, 45.0):              # eastbound, incl. mid-turn headings
      assert r.limit(x, hdg) == 30, (x, hdg)           # map (was 60)
  assert r.limit(500, 90.0) == 60                     # US 12 eastbound: map 60 anyway
  assert r.limit(1015, 240.0) == 60                   # 30 deg off the westbound way: still applies
  assert r.limit(1015, 200.0) == 30                   # 70 deg off: not the sign's direction


def test_degraff_25_on_atlantic_stops_at_its_end(tmp_path):
  # a 25 read past the town line on Atlantic (map 30): carries nowhere new, stops where the map changes
  r = degraff(tmp_path)
  r.yes(1100.0, 25, 90.0, 1, sign_u=1120.0)
  assert r.limit(1110, 90.0) == 30 and r.limit(1500, 90.0) == 25 and r.limit(1900, 90.0) == 55


def test_carry_once_then_stop_at_next_change(tmp_path):
  # a 45 sign 100 m before a 60 -> 40 map change; the 40 way is carried, the 40 -> 30 change stops it
  r = Road(tmp_path, ew, [
    (1, 0, 1000, 60, "MN 7", "", False),
    (2, 1000, 1500, 40, "MN 7", "", False),
    (3, 1500, 2500, 30, "MN 7", "", False),
  ])
  r.yes(880.0, 45, 90.0, 1, sign_u=900.0)
  assert r.limit(950, 90.0) == 45 and r.limit(1400, 90.0) == 45 and r.limit(1600, 90.0) == 30


def test_far_sign_keeps_stopping_at_map_change(tmp_path):
  r = Road(tmp_path, ew, [
    (1, 0, 1000, 60, "MN 7", "", False),
    (2, 1000, 1500, 40, "MN 7", "", False),
  ])
  r.yes(600.0, 50, 90.0, 1, sign_u=620.0)             # 380 m from the change
  assert r.limit(900, 90.0) == 50 and r.limit(1200, 90.0) == 40


def test_route_number_matches_across_names(tmp_path):
  # same limit, name changes but the route number is the same: continue (was a road change)
  r = Road(tmp_path, ew, [
    (1, 0, 1000, 55, "US 59", "", False),
    (2, 1000, 3000, 55, "US Highway 59", "", False),
    (3, 3000, 5000, 55, "Main Street", "US 59", False),
    (4, 5000, 6000, 55, "Main Street", "", False),
  ])
  r.yes(200.0, 60, 90.0, 1, sign_u=220.0)
  assert r.limit(2000, 90.0) == 60 and r.limit(4000, 90.0) == 60 and r.limit(5500, 90.0) == 60


def test_name_change_without_map_change_is_a_road_change(tmp_path):
  r = Road(tmp_path, ew, [
    (1, 0, 1000, 55, "Elm Street", "", False),
    (2, 1000, 2000, 55, "Oak Street", "", False),
  ])
  r.yes(950.0, 30, 90.0, 1, sign_u=960.0)
  assert r.limit(980, 90.0) == 30 and r.limit(1200, 90.0) == 55


def test_no_carry_into_turn_or_fork(tmp_path):
  # at the map change the way straight ahead is not unique (fork) -> stop at the change
  end = ew(1000)
  y = lambda dx, dy: (end[0] + dy / M_LAT, end[1] + dx / M_LON)   # noqa: E731
  r = Road(tmp_path, ew, [
    (1, 0, 1000, 60, "US 12", "", False),
  ], extra=[(10, [end, y(500, 10)], 30, "Atlantic Avenue"), (11, [end, y(500, -10)], 30, "Depot Street")])
  r.yes(985.0, 30, 90.0, 1, sign_u=990.0)
  assert r.o.entries[0]["chain"][-1][0] == 1 and len(r.o.entries[0]["chain"]) == 1
  # a side road at a right angle is never followed
  r2 = Road(tmp_path / "b", ew, [(1, 0, 1000, 60, "US 12", "", False)],
            extra=[(10, [end, y(0, 500)], 30, "Atlantic Avenue")])
  r2.yes(985.0, 30, 90.0, 1, sign_u=990.0)
  assert len(r2.o.entries[0]["chain"]) == 1


def test_route_keys():
  assert route_keys("US 59") == route_keys("US Highway 59") == route_keys("U.S. Hwy 59") == {"us59"}
  assert route_keys("County Road 31") == route_keys("CR 31") == route_keys("CSAH 31") == {"cr31"}
  assert route_keys("MN 7") == route_keys("State Highway 7") == {"st7"}
  assert route_keys("I 90") == route_keys("Interstate 90") == {"i90"}
  assert route_keys("US 59;MN 7") == {"us59", "st7"}
  assert route_keys("13th Street") == set() and route_keys("") == set()


def test_sign_ahead_clamp():
  f = SpeedOverrides.sign_ahead_m
  lat, lon = ew(0)
  s_lat, s_lon = ew(40)
  assert abs(f(90.0, {"lat": lat, "lon": lon, "sign_lat": s_lat, "sign_lon": s_lon}) - 40) < 0.5
  assert f(270.0, {"lat": lat, "lon": lon, "sign_lat": s_lat, "sign_lon": s_lon}) == 0.0   # never behind
  s_lat, s_lon = ew(900)
  assert f(90.0, {"lat": lat, "lon": lon, "sign_lat": s_lat, "sign_lon": s_lon}) == 150.0
  assert f(90.0, {"lat": lat, "lon": lon, "sign_range_m": 33.0}) == 33.0
  assert f(90.0, {"lat": lat, "lon": lon}) == 0.0


def test_payload_carries_sign_fields():
  class P:
    def put(self, k, v):
      self.v = v
  p = P()
  rec = {"lat": 1.0, "lon": 2.0, "bearing": 3.0, "sign_lat": 1.1, "sign_lon": 2.1, "sign_range_m": 40.0}
  assert request_override(60, rec, 1.0, params=p, way_id=5, heading=180.0)
  c = parse_confirmed(json.dumps(p.v))
  assert c["sign_lat"] == 1.1 and c["sign_lon"] == 2.1 and c["sign_range_m"] == 40.0


def test_build_chain_without_db_is_one_way():
  ch = build_chain(1, 100.0, 1, [ew(0), ew(500)], None)
  assert len(ch) == 1 and ch[0][0] == 1 and abs(ch[0][4] - 400.0) < 1.0


def test_next_limit_follows_section_across_the_town_line(tmp_path):
  # section starts on 13th Street (old payload: at the read); standing on it just past the read,
  # nextSpeedLimit must not offer US Highway 59's map 55 (the section covers it)
  r = clarkfield(tmp_path)
  r.yes(1.0, 60, 180.0, 1)
  w = r.way_at(0.8)
  lat, lon = ns(0.8)
  assert r.limit(0.8, 180.0) == 60
  n = effective_next(r.o.entries, r.db, w["way_id"], w["coords"], lat, lon, 180.0, 60 * MPH_TO_MS, True,
                     (55 * MPH_TO_MS, 0.8))
  assert n is None


def test_no_carry_when_the_map_change_is_another_zone(tmp_path):
  # Montevideo MN 7 westbound (Oct 23 17:22:37): a 35 read 109 m before a map 40 -> 30 change.
  # The map's change is not the one the sign marks (35 is no closer to 30 than to 40): stop there.
  r = Road(tmp_path, ew, [
    (1, 0, 1000, 40, "MN 7", "", False),
    (2, 1000, 1500, 30, "MN 7", "", False),
  ])
  r.yes(850.0, 35, 90.0, 1, sign_u=891.0)
  assert r.limit(950, 90.0) == 35 and r.limit(1200, 90.0) == 30
