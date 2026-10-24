"""Sign sections across many short OSM ways (Benson 60 -> 50 -> 30, 25 -> 30 stop at 35)."""
import math

import pytest

from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB
from openpilot.selfdrive.mapd.speed_overrides import MPH_TO_MS, SpeedOverrides

LAT0, LON0 = 45.3150, -95.6000
M_PER_DEG_LON = 111320.0 * math.cos(math.radians(LAT0))


def pt(x_m):  # x meters east of LON0 along an east-west road
  return (LAT0, LON0 + x_m / M_PER_DEG_LON)


def mk_db(tmp_path, pieces, name="US 12"):
  """pieces: [(x0, x1, mph, reversed)] -> consecutive 100-300 m ways (some digitized westward)."""
  path = str(tmp_path / "osm.db")
  con = OsmSpeedLimitDB.create(path)
  for i, (x0, x1, mph, rev) in enumerate(pieces):
    c = [pt(x0), pt((x0 + x1) / 2), pt(x1)]
    OsmSpeedLimitDB.insert_way(con, 1000 + i, name, "primary", mph * MPH_TO_MS, c[::-1] if rev else c)
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  return db


def short_ways(sections, step=150):
  """sections: [(x0, x1, mph)] split into `step` m ways, alternating digitizing direction."""
  out = []
  for x0, x1, mph in sections:
    x = x0
    while x < x1 - 1:
      out.append((x, min(x + step, x1), mph, len(out) % 2 == 1))
      x += step
  return out


def way_at(db, x):
  for i in range(10000):
    w = db.way_info(1000 + i)
    if w is None:
      return None
    xs = sorted(((p[1] - LON0) * M_PER_DEG_LON) for p in w["coords"])
    if xs[0] - 0.5 <= x <= xs[-1] + 0.5:
      return w


def limit(o, db, x, bearing=90.0):
  """mph mapd publishes at x (override if any, else OSM)."""
  w = way_at(db, x)
  lat, lon = pt(x)
  ms = o.limit_ms(w["way_id"], bearing, lat, lon, w["coords"])
  return round((ms if ms is not None else w["maxspeed_ms"]) / MPH_TO_MS)


def yes(o, db, x, mph, heading, seq):
  w = way_at(db, x)
  lat, lon = pt(x)
  conf = {"mph": mph, "seq": float(seq), "way_id": w["way_id"], "heading": heading, "lat": lat, "lon": lon}
  return o.consume(conf, None, heading, float(seq), way_coords=db.way_coords, db=db)


# Benson on US 12 eastbound: map says 60 to x=3000, then 40 (wrong) to 5000, then 30.
# Posted: 60, 50 at x=2000, 30 at x=3200.
BENSON = [(0, 3000, 60), (3000, 5000, 40), (5000, 8000, 30)]


@pytest.mark.parametrize("order", ["in_turn", "downstream_first"])
def test_benson_60_50_30_eastbound(tmp_path, order):
  db = mk_db(tmp_path, short_ways(BENSON))
  o = SpeedOverrides(str(tmp_path / "o.json"))
  steps = [(500, 60), (2000, 50), (3200, 30)]
  if order == "downstream_first":
    steps = steps[::-1]
  for i, (x, mph) in enumerate(steps):
    assert yes(o, db, x, mph, 90.0, i + 1) is not None
  assert limit(o, db, 1000) == 60
  assert limit(o, db, 1990) == 60          # 50 never applied behind its sign
  assert limit(o, db, 2010) == 50          # 50 applies across many 150 m ways...
  assert limit(o, db, 2900) == 50
  assert limit(o, db, 3100) == 40          # ...and stops where the map's limit changes (60 -> 40)
  assert limit(o, db, 3300) == 30          # 30 section
  assert limit(o, db, 4900) == 30          # map stays 40 until 5000
  assert limit(o, db, 6000) == 30          # map 30 beyond (30 span ended at the map change)
  # both directions of travel: westbound sees the same sections
  assert limit(o, db, 2500, bearing=270.0) == 50
  assert limit(o, db, 3500, bearing=270.0) == 30


def test_benson_westbound_signs(tmp_path):
  db = mk_db(tmp_path, short_ways(BENSON))
  o = SpeedOverrides(str(tmp_path / "o.json"))
  # leaving town westbound: 40 map region gets a 45 sign at x=4800; covers westward to 3000
  yes(o, db, 4800, 45, 270.0, 1)
  assert limit(o, db, 4900, 270.0) == 40   # never behind the sign
  assert limit(o, db, 3100, 270.0) == 45
  assert limit(o, db, 3100, 90.0) == 45    # both directions
  assert limit(o, db, 2900, 270.0) == 60   # map changes 40 -> 60 at 3000: stop


def test_newer_sign_ends_previous_and_upstream_reconfirm_keeps_downstream(tmp_path):
  db = mk_db(tmp_path, short_ways([(0, 6000, 60)]))
  o = SpeedOverrides(str(tmp_path / "o.json"))
  yes(o, db, 3000, 30, 90.0, 1)            # older stored downstream sign
  yes(o, db, 500, 50, 90.0, 2)             # newer upstream sign
  assert limit(o, db, 2000) == 50
  assert limit(o, db, 3500) == 30          # newer upstream YES did not erase or override it
  yes(o, db, 3000, 35, 90.0, 3)            # re-confirm same sign replaces it
  assert limit(o, db, 3500) == 35
  assert sum(1 for e in o.entries if e.get("chain")) == 2


def test_25_to_30_stops_at_35(tmp_path):
  db = mk_db(tmp_path, short_ways([(0, 1609, 25), (1609, 4000, 35)], step=100))
  o = SpeedOverrides(str(tmp_path / "o.json"))
  yes(o, db, 200, 30, 90.0, 1)
  assert limit(o, db, 150) == 25
  assert limit(o, db, 250) == 30
  assert limit(o, db, 1500) == 30          # many 100 m ways
  assert limit(o, db, 1700) == 35          # stops where the map changes to 35


def test_same_value_as_map_still_written(tmp_path):
  db = mk_db(tmp_path, short_ways([(0, 3000, 50)]))
  o = SpeedOverrides(str(tmp_path / "o.json"))
  yes(o, db, 100, 60, 90.0, 1)
  assert yes(o, db, 1000, 50, 90.0, 2) is not None   # matches OSM; still ends the 60 section
  assert limit(o, db, 1500) == 50


def test_cap_road_change_and_fork(tmp_path):
  db = mk_db(tmp_path, short_ways([(0, 15000, 55)], step=300))
  o = SpeedOverrides(str(tmp_path / "o.json"))
  yes(o, db, 0, 45, 90.0, 1)
  assert limit(o, db, 9800) == 45
  assert limit(o, db, 10300) == 55         # ~10 km cap
  path = str(tmp_path / "o2.db")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(con, 1, "Main", "primary", 55 * MPH_TO_MS, [pt(0), pt(200)])
  OsmSpeedLimitDB.insert_way(con, 2, "Oak", "primary", 55 * MPH_TO_MS, [pt(200), pt(400)])   # road change
  OsmSpeedLimitDB.insert_way(con, 3, "Main", "primary", 55 * MPH_TO_MS, [pt(400), pt(600)])
  OsmSpeedLimitDB.insert_way(con, 4, "Main", "primary", 55 * MPH_TO_MS, [pt(600), (LAT0 + 0.002, pt(800)[1])])  # fork
  con.commit()
  con.close()
  db2 = OsmSpeedLimitDB(path)
  db2.open()
  o2 = SpeedOverrides(str(tmp_path / "o2.json"))
  e = o2.consume({"mph": 30, "seq": 1.0, "way_id": 1, "heading": 90.0, "lat": pt(50)[0], "lon": pt(50)[1]},
                 None, 90.0, 1.0, way_coords=db2.way_coords, db=db2)
  assert [sp[0] for sp in e["chain"]] == [1]
  e = o2.consume({"mph": 30, "seq": 2.0, "way_id": 3, "heading": 90.0, "lat": pt(450)[0], "lon": pt(450)[1]},
                 None, 90.0, 2.0, way_coords=db2.way_coords, db=db2)
  assert [sp[0] for sp in e["chain"]] == [3, 4]   # one same-road continuation: follow it
  OsmSpeedLimitDB.insert_way(sq := __import__("sqlite3").connect(path), 5, "Main", "primary", 55 * MPH_TO_MS,
                             [pt(600), (LAT0 - 0.002, pt(800)[1])])
  sq.commit()
  sq.close()
  db3 = OsmSpeedLimitDB(path)
  db3.open()
  e = o2.consume({"mph": 30, "seq": 3.0, "way_id": 3, "heading": 90.0, "lat": pt(450)[0], "lon": pt(450)[1]},
                 None, 90.0, 3.0, way_coords=db3.way_coords, db=db3)
  assert [sp[0] for sp in e["chain"]] == [3]   # fork at 600: stop


def test_legacy_bad_row_superseded(tmp_path):
  db = mk_db(tmp_path, short_ways([(0, 3000, 60)]))
  o = SpeedOverrides(str(tmp_path / "o.json"))
  w = way_at(db, 1500)
  o.entries.append({"way_id": w["way_id"], "heading": 90.0, "mph": 30, "both": True, "from_s": None, "dir": 1})
  assert limit(o, db, 1500) == 30
  yes(o, db, 100, 60, 90.0, 1)
  assert limit(o, db, 1500) == 60


def test_chain_rows_saved_and_legacy_kept_not_applied(tmp_path):
  """Oct 22 hygiene on sign chains: a chain Yes persists, never drops the legacy row, legacy never applies."""
  import json
  db = mk_db(tmp_path, short_ways([(0, 3000, 60)]))
  p = str(tmp_path / "o.json")
  w = way_at(db, 1500)
  legacy = {"way_id": w["way_id"], "heading": 90.0, "mph": 30, "lat": 0.0, "lon": 0.0, "t": 1.0, "source": "sign_yes"}
  with open(p, "w") as f:
    json.dump({"version": 1, "overrides": [legacy]}, f)
  o = SpeedOverrides(p)
  assert limit(o, db, 1500) == 60                      # legacy 30 not applied
  assert o.find_legacy(w["way_id"], 92.0)["mph"] == 30
  e = yes(o, db, 100, 50, 90.0, 1)
  assert e["chain"] and e["v"] == 2
  rows = json.load(open(p))["overrides"]
  assert rows[0] == legacy and any(r.get("chain") for r in rows)
  o2 = SpeedOverrides(p)                               # next drive: chain row came back from the file
  assert limit(o2, db, 1500) == 50 and len(o2.legacy) == 1


def test_clear_removes_chain_row_where_car_is(tmp_path):
  import json
  db = mk_db(tmp_path, short_ways([(0, 3000, 60)]))
  p = str(tmp_path / "o.json")
  legacy = {"way_id": way_at(db, 2500)["way_id"], "heading": 90.0, "mph": 30, "t": 1.0}
  with open(p, "w") as f:
    json.dump({"version": 1, "overrides": [legacy]}, f)
  o = SpeedOverrides(p)
  yes(o, db, 100, 50, 90.0, 1)
  w = way_at(db, 1500)
  lat, lon = pt(1500)
  gone = o.clear(w["way_id"], 90.0, lat, lon, w["coords"])
  assert [g["mph"] for g in gone] == [50]
  assert limit(o, db, 1500) == 60
  assert json.load(open(p))["overrides"] == [legacy]   # legacy row stays
