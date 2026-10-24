"""Speed-sign Temp: this road, this drive, never saved. Each end condition."""
import json
import math
import os
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.mapd import temp_limit as T
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB
from openpilot.selfdrive.mapd.speed_overrides import MPH_TO_MS

LAT0, LON0 = 45.3150, -95.6000
M_PER_DEG_LON = 111320.0 * math.cos(math.radians(LAT0))
M_PER_DEG_LAT = 111320.0


def pt(x_m, y_m=0.0):
  return (LAT0 + y_m / M_PER_DEG_LAT, LON0 + x_m / M_PER_DEG_LON)


_DBS = []


@pytest.fixture(autouse=True)
def _close_dbs():
  yield
  while _DBS:
    _DBS.pop().close()


def mk_db(tmp_path, ways):
  """ways: [(way_id, name, mph, [(x, y), ...])]."""
  path = str(tmp_path / "osm.db")
  con = OsmSpeedLimitDB.create(path)
  for wid, name, mph, xy in ways:
    OsmSpeedLimitDB.insert_way(con, wid, name, "primary", mph * MPH_TO_MS, [pt(*p) for p in xy])
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  _DBS.append(db)
  return db


# US 12 eastbound: three 1 km ways at 60, then a fork at x=3000 (two "US 12" continuations).
MAIN = [
  (1, "US 12", 60, [(0, 0), (500, 0), (1000, 0)]),
  (2, "US 12", 60, [(2000, 0), (1500, 0), (1000, 0)]),   # digitized westward
  (3, "US 12", 60, [(2000, 0), (3000, 0)]),
  (4, "Main St", 30, [(1500, 0), (1500, 800)]),          # side road at x=1500
  (5, "US 12", 60, [(3000, 0), (6000, 0)]),
  (6, "US 12", 60, [(3000, 0), (6000, 300)]),             # fork: two same-road continuations
]


class Car:
  """Drives east along y=0; returns the mapd-like match for x."""

  def __init__(self, db):
    self.db = db

  def match(self, x, way=None, y=0.0):
    if way is None:
      way = 1 if x < 1000 else 2 if x < 2000 else 3 if x < 3000 else 5
    w = self.db.way_info(way)
    return NS(way_id=way, coords=w["coords"], road_name=w["name"], highway=w["highway"]), pt(x, y)


def start(tl, car, x=200.0, mph=45, read_seq=7, seq=1.0, sign=None, **kw):
  m, (lat, lon) = car.match(x)
  raw = {"mph": mph, "seq": seq, "read_seq": read_seq, "way_id": m.way_id, "heading": 90.0,
         "lat": lat, "lon": lon}
  raw.update(kw)
  return tl.consume(json.dumps(raw), m, lat, lon, 90.0, db=car.db, sign_live=sign)


def drive(tl, car, x0, x1, v=25.0, under=60, osm=60, tesla=0, tesla_valid=False, way=None, bearing=90.0, cones=None):
  """2 Hz ticks from x0 to x1 at v. Returns (last x, last result)."""
  x, out = x0, None
  t = getattr(tl, "_t", None) or 0.0
  step = v * 0.5 * (1 if x1 >= x0 else -1)
  while (x <= x1) if x1 >= x0 else (x >= x1):
    t += 0.5
    tl.advance(t, v)
    if cones is not None and cones(x):
      tl.note_cones(NS(active=True, barrier=False, parked=False, leftActive=False, rightActive=False))
    m, (lat, lon) = car.match(x, way=way)
    out = tl.tick(m, m.coords, lat, lon, bearing, v, under * MPH_TO_MS, osm, tesla, tesla_valid)
    if out is None:
      return x, None
    x += step
  return x, out


def test_temp_is_the_limit_until_an_end(tmp_path):
  car = Car(mk_db(tmp_path, MAIN))
  tl = T.TempLimit()
  st = start(tl, car)
  assert st is not None and st.chain       # chain from the sign, same rule as a confirmed sign
  x, out = drive(tl, car, 210, 1900)
  assert out == 45 * MPH_TO_MS and tl.active
  # Same seq again (param re-read) does not restart it.
  assert start(tl, car) is None


def test_apply_to_msg_is_top_priority_and_lookahead():
  tl = T.TempLimit()
  tl.state = T.TempState(mph=45, seq=1.0, read_seq=1, way_id=1, heading=90.0)
  d = NS(speedLimit=60 * MPH_TO_MS, speedLimitValid=True, source="sign", osmRawMph=60, teslaBaseMph=55,
         tempActive=False, tempMph=0, tempEndReason="", tempEndSeq=0, tempNoCueM=0.0)
  nxt = T.apply_to_msg(d, tl, 45 * MPH_TO_MS, (45 * MPH_TO_MS, 300.0))
  assert d.source == "temp" and round(d.speedLimit / MPH_TO_MS) == 45 and d.tempActive and d.tempMph == 45
  assert nxt is None                      # the limit ahead equals the Temp
  assert T.apply_to_msg(d, tl, 45 * MPH_TO_MS, (30 * MPH_TO_MS, 200.0)) == (30 * MPH_TO_MS, 200.0)
  # Auto-confirm reads only raw OSM / Tesla base: untouched.
  assert d.osmRawMph == 60 and d.teslaBaseMph == 55


def test_end_new_sign_different_mph(tmp_path):
  car = Car(mk_db(tmp_path, MAIN))
  tl = T.TempLimit()
  start(tl, car, read_seq=7)
  # The answered read's own prompt still on the wire: not a new sign.
  tl.note_sign(NS(proposedMph=45, proposedSeq=7, signMph=0, signSeq=0))
  assert tl.active
  tl.note_sign(NS(proposedMph=55, proposedSeq=8, signMph=0, signSeq=0))
  assert not tl.active and tl.end_reason == "sign" and tl.end_seq == 1


def test_new_sign_placement_with_other_mph_ends(tmp_path):
  car = Car(mk_db(tmp_path, MAIN))
  tl = T.TempLimit()
  start(tl, car, sign=NS(proposedMph=0, proposedSeq=7, signMph=45, signSeq=3))
  tl.note_sign(NS(proposedMph=0, proposedSeq=7, signMph=45, signSeq=3))
  assert tl.active
  tl.note_sign(NS(proposedMph=0, proposedSeq=7, signMph=65, signSeq=4))
  assert tl.end_reason == "sign"


def test_same_mph_repeat_sign_is_a_cue_not_an_end(tmp_path):
  car = Car(mk_db(tmp_path, MAIN))
  tl = T.TempLimit(no_cue_m=1000.0)
  start(tl, car, x=200)
  drive(tl, car, 210, 900)
  tl.note_sign(NS(proposedMph=45, proposedSeq=9, signMph=0, signSeq=0))
  assert tl.active and tl.no_cue_m_now() < 1.0
  x, out = drive(tl, car, 900, 1800)
  assert out is not None                    # 1000 m counted from the repeat, not the first sign


def test_end_map_limit_change_debounced(tmp_path):
  car = Car(mk_db(tmp_path, MAIN))
  tl = T.TempLimit()
  start(tl, car)
  drive(tl, car, 210, 400)
  # One tick blip of the underlying limit does not end it.
  m, (lat, lon) = car.match(410)
  tl.advance(1000.0, 25.0)
  assert tl.tick(m, m.coords, lat, lon, 90.0, 25.0, 40 * MPH_TO_MS, 60) is not None
  assert tl.tick(m, m.coords, lat, lon, 90.0, 25.0, 60 * MPH_TO_MS, 60) is not None
  x, out = drive(tl, car, 420, 900, under=40, osm=40)
  assert out is None and tl.end_reason == "map"


def test_end_tesla_base_change(tmp_path):
  car = Car(mk_db(tmp_path, MAIN))
  tl = T.TempLimit()
  start(tl, car)
  drive(tl, car, 210, 400, tesla=55, tesla_valid=True)
  assert tl.active
  x, out = drive(tl, car, 410, 900, tesla=45, tesla_valid=True)
  assert out is None and tl.end_reason == "map"


def test_osm_change_ends_even_under_a_confirmed_section(tmp_path):
  car = Car(mk_db(tmp_path, MAIN))
  tl = T.TempLimit()
  start(tl, car)
  drive(tl, car, 210, 400, under=50, osm=60)       # confirmed 50 section on a 60 road
  x, out = drive(tl, car, 410, 900, under=50, osm=45)
  assert out is None and tl.end_reason == "map"


def test_end_road_change_turn_onto_side_road(tmp_path):
  car = Car(mk_db(tmp_path, MAIN))
  tl = T.TempLimit()
  start(tl, car)
  drive(tl, car, 210, 1490)
  # Two ticks on Main St (GPS jitter at the junction) do not end it.
  side = car.db.way_info(4)
  for y in (5.0, 10.0):
    tl.advance(tl._t + 0.5, 10.0)
    lat, lon = pt(1500, y)
    m = NS(way_id=4, coords=side["coords"], road_name="Main St", highway="primary")
    tl.tick(m, m.coords, lat, lon, 0.0, 10.0, 60 * MPH_TO_MS, 60)
  assert tl.active
  m, (lat, lon) = car.match(1500)
  tl.tick(m, m.coords, lat, lon, 90.0, 10.0, 60 * MPH_TO_MS, 60)
  assert tl.active and tl.state.off_ticks == 0
  for y in range(20, 300, 10):   # 2 Hz at 20 m/s: ROAD_OFF_M (100 m) on Main St ends it
    tl.advance(tl._t + 0.5, 20.0)
    lat, lon = pt(1500, y)
    m = NS(way_id=4, coords=side["coords"], road_name="Main St", highway="primary")
    tl.tick(m, m.coords, lat, lon, 0.0, 20.0, 60 * MPH_TO_MS, 60)   # same map limit: only the road ends it
    if not tl.active:
      break
  assert tl.end_reason == "road" and 100 <= y <= 140


def test_end_at_a_fork(tmp_path):
  car = Car(mk_db(tmp_path, MAIN))
  tl = T.TempLimit()
  st = start(tl, car)
  assert {sp[0] for sp in st.chain} == {1, 2, 3}  # chain stops at the fork (ways 5 and 6)
  x, out = drive(tl, car, 210, 3600)
  assert out is None and tl.end_reason == "road" and 3100 <= x < 3200   # ROAD_OFF_M past the fork


def test_junction_mismatch_flicker_does_not_end(tmp_path):
  """Oct 21 15:18:55 CDT: mapd matched 304th Street for ~3 s crossing it, then Ridgewood Road again."""
  car = Car(mk_db(tmp_path, MAIN))
  tl = T.TempLimit()
  start(tl, car)
  drive(tl, car, 210, 1490, v=13.0)
  side = car.db.way_info(4)
  for i in range(6):   # 3 s at 13 m/s (~40 m) matched to the cross street, which has another limit (30)
    tl.advance(tl._t + 0.5, 13.0)
    lat, lon = pt(1500 + 6.5 * i, 2.0)
    m = NS(way_id=4, coords=side["coords"], road_name="Main St", highway="primary")
    tl.tick(m, m.coords, lat, lon, 90.0, 13.0, 30 * MPH_TO_MS, 30)
  assert tl.active and tl.state.off_ticks == 6
  x, out = drive(tl, car, 1540, 2500, v=13.0)
  assert out is not None and tl.active and tl.state.off_ticks == 0


def test_end_turn_around(tmp_path):
  car = Car(mk_db(tmp_path, MAIN))
  tl = T.TempLimit()
  start(tl, car)
  drive(tl, car, 210, 800)
  x, out = drive(tl, car, 800, 400, bearing=270.0)
  assert out is None and tl.end_reason == "road"


def test_no_road_at_read_anchors_on_current_road(tmp_path):
  car = Car(mk_db(tmp_path, MAIN))
  tl = T.TempLimit()
  m, (lat, lon) = car.match(200)
  st = tl.consume({"mph": 45, "seq": 2.0, "read_seq": 3}, m, lat, lon, 90.0, db=None)
  assert st.chain is None and st.road == ("US 12", "primary")
  drive(tl, car, 210, 2500)                  # same name across ways: still on
  assert tl.active
  side = car.db.way_info(4)
  for y in range(20, 300, 10):
    tl.advance(tl._t + 0.5, 20.0)
    la, lo = pt(1500, y)
    tl.tick(NS(way_id=4, coords=side["coords"], road_name="Main St", highway="primary"), side["coords"],
            la, lo, 0.0, 20.0, 60 * MPH_TO_MS, 60)
  assert tl.end_reason == "road"


def test_end_no_construction_seen_measured_from_sign(tmp_path):
  db = mk_db(tmp_path, [(1, "I 94", 70, [(0, 0), (20000, 0)])])
  car = Car(db)
  car.match = lambda x, way=None, y=0.0: (NS(way_id=1, coords=db.way_info(1)["coords"], road_name="I 94",
                                              highway="primary"), pt(x, y))
  tl = T.TempLimit()
  assert T.NO_CUE_END_M == 5000.0
  # Read at x=0, tapped 300 m later: the 5000 m runs from the sign.
  m, (lat, lon) = car.match(0)
  m2, (lat2, lon2) = car.match(300)
  tl.consume({"mph": 55, "seq": 1.0, "read_seq": 1, "way_id": 1, "heading": 90.0, "lat": lat, "lon": lon},
             m2, lat2, lon2, 90.0, db=db)
  x, out = drive(tl, car, 300, 9000, under=70, osm=70)
  assert out is None and tl.end_reason == "clear" and 4950 <= x <= 5100


def test_cones_keep_it_alive_through_a_quiet_gap(tmp_path):
  """Oct 21-like: cones, a 3 km shifted lane with no cue, cones again; ends 5 km after the last cone."""
  db = mk_db(tmp_path, [(1, "I 94", 70, [(0, 0), (30000, 0)])])
  car = Car(db)
  car.match = lambda x, way=None, y=0.0: (NS(way_id=1, coords=db.way_info(1)["coords"], road_name="I 94",
                                              highway="primary"), pt(x, y))
  tl = T.TempLimit()
  m, (lat, lon) = car.match(0)
  tl.consume({"mph": 55, "seq": 1.0, "read_seq": 1, "way_id": 1, "heading": 90.0, "lat": lat, "lon": lon},
             m, lat, lon, 90.0, db=db)
  cones = lambda x: 4000 <= x <= 4600 or 7600 <= x <= 8200    # noqa: E731
  x, out = drive(tl, car, 0, 20000, under=70, osm=70, cones=cones)
  assert out is None and tl.end_reason == "clear" and 13100 <= x <= 13300


def test_parked_or_barrier_is_not_a_cue():
  tl = T.TempLimit()
  tl.state = T.TempState(mph=45, seq=1.0, read_seq=1, way_id=0, heading=None)
  tl.odo = 100.0
  tl.note_cones(NS(active=True, barrier=True, parked=False, leftActive=False, rightActive=False))
  tl.note_cones(NS(active=True, barrier=False, parked=True, leftActive=False, rightActive=False))
  assert tl.state.cues == 0
  tl.note_cones(NS(active=False, barrier=False, parked=False, leftActive=True, rightActive=False))
  assert tl.state.cues == 1 and tl.state.last_cue_odo == 100.0


def test_new_temp_replaces_running_one(tmp_path):
  car = Car(mk_db(tmp_path, MAIN))
  tl = T.TempLimit()
  start(tl, car, mph=45, seq=1.0)
  start(tl, car, mph=35, seq=2.0, read_seq=9)
  assert tl.state.mph == 35


def test_nothing_persisted(tmp_path, monkeypatch):
  """A whole Temp lifecycle writes no override file, no confirmed row, no auto-confirm param."""
  ov = tmp_path / "map_speed_overrides.json"
  monkeypatch.setenv("NAP_MAP_SPEED_OVERRIDES", str(ov))
  monkeypatch.setattr(T, "log_path", lambda: str(tmp_path / "fb" / "temp_limit.jsonl"))
  car = Car(mk_db(tmp_path, MAIN))
  tl = T.TempLimit()
  start(tl, car)
  drive(tl, car, 210, 3600)
  T.log_event({"event": "end", **(tl.last_end or {})})
  assert not ov.exists()
  assert not (tmp_path / "fb" / "confirmed.jsonl").exists()
  assert sorted(os.listdir(tmp_path / "fb")) == ["temp_limit.jsonl"]


def test_request_temp_payload_and_state():
  class Mem(dict):
    def put(self, k, v):
      self[k] = v
  p = Mem()
  assert T.request_temp(45, {"lat": 45.0, "lon": -93.0, "bearing": 90.0}, read_seq=7, way_id=111,
                        heading=91.0, params=p, now=5.0)
  got = T.parse_temp(p[T.PARAM])
  assert got["mph"] == 45 and got["read_seq"] == 7 and got["way_id"] == 111 and got["lat"] == 45.0
  assert not T.request_temp(47, None, params=p)          # not a posted value
  tl = T.TempLimit()
  tl.state = T.TempState(mph=45, seq=1.0, read_seq=1, way_id=0, heading=None)
  T.write_state(p, tl, 10.0)
  assert T.read_state(p, now=11.0) == {"mph": 45}
  tl._end("clear")
  T.write_state(p, tl, 20.0)
  assert T.read_state(p, now=22.0) == {"ended": "no construction seen"}
  assert T.read_state(p, now=20.0 + T.END_NOTE_S + 1) is None


def test_cleared_each_drive():
  root = os.path.join(os.path.dirname(__file__), "..", "..", "..")
  with open(os.path.join(root, "common", "nap_params_keys.h")) as f:
    keys = f.read()
  for k in ("NAPSpeedSignTemp", "NAPSpeedSignTempState"):
    line = next(ln for ln in keys.splitlines() if f'"{k}"' in ln)
    assert "CLEAR_ON_OFFROAD_TRANSITION" in line and "CLEAR_ON_MANAGER_START" in line and "PERSISTENT" not in line
  with open(os.path.join(root, "system", "manager", "process_config.py")) as f:
    pc = f.read()
  line = next(ln for ln in pc.splitlines() if '"mapd"' in ln)
  assert "only_onroad" in line                    # in-memory Temp dies with mapd at offroad
