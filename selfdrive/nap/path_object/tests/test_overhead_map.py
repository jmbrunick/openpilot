import math
import sqlite3
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.nap.path_object import overhead_map as OM
from openpilot.selfdrive.nap.path_object import clearance as C
from openpilot.selfdrive.controls.lib.path_object_response import confirmed_from_msg

LAT, LON = 44.5, -95.0
M_LAT = 1 / 111195.0


def _con(rows):
  con = sqlite3.connect(":memory:")
  OM.ensure_table(con)
  for r in rows:
    OM.insert(con, *r)
  return con


def _north(m):
  return LAT + m * M_LAT


def test_lookup_ahead_heading_and_lag():
  con = _con([(1, _north(120), LON, "bridge", 9, 2.0), (2, _north(60), LON, "bridge", 9, 90.0),
              (3, _north(-80), LON, "signal", 9, None)])
  ov = OM.overhead_ahead(con, LAT, LON, 0.0, v_ego=0.0)
  assert ov.oid == 1 and abs(ov.distance_m - 120) < 1.5    # cross-road bridge (heading 90) skipped, behind skipped
  ov = OM.overhead_ahead(con, LAT, LON, 0.0, v_ego=20.0, lag_s=1.5)
  assert abs(ov.distance_m - 90) < 1.5                      # optional lag correction
  assert abs(OM.overhead_ahead(con, LAT, LON, 0.0, v_ego=20.0).distance_m - 120) < 1.5
  assert OM.overhead_ahead(con, LAT, LON, 180.0, 0.0).oid == 3   # signals: no heading check
  assert OM.overhead_ahead(None, LAT, LON, 0.0) is None
  assert OM.overhead_ahead(sqlite3.connect(":memory:"), LAT, LON, 0.0) is None   # old pack


def test_lateral_window():
  con = _con([(1, _north(100), LON + 30 * M_LAT / math.cos(math.radians(LAT)), "gantry", 9, 0.0)])
  assert OM.overhead_ahead(con, LAT, LON, 0.0) is None


def _obs(**kw):
  base = dict(agree=True, visionEvaluated=True, rangeM=80.0, lateral=0.0, spanM=0.5, vRel=-20.0,
              objectClass="obstacle", overheadIgnore=False, bottomClearanceM=float("nan"),
              roadLights=False, roadLevelSeen=False, daylight=False, modelLead=False)
  base.update(kw)
  return SimpleNamespace(**base)


MD = SimpleNamespace(overheadAheadM=85.0, overheadKind="bridge")


def test_match_all_guards():
  assert OM.overhead_match(_obs(), MD, 20.0)
  assert not OM.overhead_match(_obs(rangeM=40.0), MD, 20.0)                 # outside 25 m window
  sig = SimpleNamespace(overheadAheadM=85.0, overheadKind="signal")
  assert OM.overhead_match(_obs(rangeM=55.0, daylight=True), sig, 20.0)     # signals: 35 m window, daylight
  assert not OM.overhead_match(_obs(rangeM=85.0), sig, 20.0)                # signals never at night
  assert not OM.overhead_match(_obs(vRel=-10.0), MD, 20.0)                  # moving: not stationary
  for cls in ("human", "animal", "unknown"):
    assert not OM.overhead_match(_obs(objectClass=cls), MD, 20.0)
  assert not OM.overhead_match(_obs(modelLead=True), MD, 20.0)
  assert not OM.overhead_match(_obs(roadLights=True), MD, 20.0)
  assert not OM.overhead_match(_obs(), SimpleNamespace(overheadAheadM=0.0, overheadKind=""), 20.0)
  assert not OM.overhead_match(_obs(), None, 20.0)


def test_missing_fields_block():
  obs = SimpleNamespace(rangeM=80.0, vRel=-20.0, objectClass="obstacle")   # old helper: no guard fields
  assert not OM.overhead_match(obs, MD, 20.0)


def test_night_close_range_recheck():
  md = SimpleNamespace(overheadAheadM=50.0, overheadKind="bridge")
  assert OM.overhead_match(_obs(rangeM=70.0, roadLevelSeen=True), SimpleNamespace(overheadAheadM=70.0, overheadKind="bridge"), 20.0)
  assert OM.overhead_match(_obs(rangeM=50.0, roadLevelSeen=False), md, 20.0)
  assert not OM.overhead_match(_obs(rangeM=50.0, roadLevelSeen=True), md, 20.0)   # camera sees it: cancel


def test_daylight_rules():
  assert OM.overhead_match(_obs(daylight=True, rangeM=80.0), MD, 20.0)
  assert not OM.overhead_match(_obs(daylight=True, roadLevelSeen=True), MD, 20.0)
  assert not OM.overhead_match(_obs(daylight=True, bottomClearanceM=0.5), MD, 20.0)
  assert OM.overhead_match(_obs(daylight=True, bottomClearanceM=4.3), MD, 20.0)


def test_confirmed_from_msg_hook():
  obs = _obs()
  assert confirmed_from_msg(obs, 0.1) is not None                # no map: unchanged
  assert confirmed_from_msg(obs, 0.1, MD, 20.0) is None
  assert confirmed_from_msg(_obs(modelLead=True), 0.1, MD, 20.0) is not None


def test_road_level_read_lights_and_contrast():
  img = np.full((600, 800), 40, np.uint8)
  ppm, vg, u = 30.0, 500.0, 400.0
  r = C.road_level_read(img, u, vg, ppm, 60.0)
  assert r is not None and not r.lights and not r.seen
  car = img.copy()
  car[int(vg - 1.4 * ppm):int(vg - 0.3 * ppm), 350:450] = 110      # body
  car[int(vg - 1.0 * ppm):int(vg - 0.9 * ppm), 360:370] = 250      # taillight
  r = C.road_level_read(car, u, vg, ppm, 60.0)
  assert r.lights and r.seen
  red = img.copy()
  red[int(vg - 1.0 * ppm):int(vg - 0.8 * ppm), 380:390] = 180
  v = np.full((300, 400), 128, np.uint8)
  v[int(vg - 1.0 * ppm) // 2:int(vg - 0.8 * ppm) // 2, 190:195] = 200
  assert C.road_level_read(red, u, vg, ppm, 60.0, v).lights
  assert not C.road_level_read(red, u, vg, ppm, 60.0).lights
  assert C.road_level_read(None, u, vg, ppm, 60.0) is None


def test_gate_road_defaults_fail_safe():
  g = C.ClearanceGate()
  assert g.road(5) == (True, True, False, True)
  g.note_road(5, None, True, False)
  assert g.road(5)[:2] == (True, True)
  g.note_road(6, C.RoadRead(False, False, 1.0, 0), False, False)
  assert g.road(6) == (False, False, False, False)
  g.note_road(6, C.RoadRead(True, False, 1.0, 5), False, False)
  g.note_road(6, C.RoadRead(False, False, 1.0, 0), False, False)
  assert g.road(6)[0] is True                                 # lights stay on for the track


def test_overlay_file_preferred_and_reopened(tmp_path, monkeypatch):
  import openpilot.selfdrive.mapd.db_paths as dbp
  monkeypatch.setattr(dbp, "osm_dir", lambda: str(tmp_path))
  OM._OVERLAY.__init__()
  assert OM.overhead_ahead_live(LAT, LON, 0.0, 0.0, None, now=0.0) is None
  con = sqlite3.connect(tmp_path / OM.OVERLAY_FILE)
  OM.ensure_table(con)
  OM.insert(con, 1, _north(100), LON, "confirmed", 0, 0.0, "clearance")
  con.commit()
  con.close()
  ov = OM.overhead_ahead_live(LAT, LON, 0.0, 0.0, None, now=100.0)
  assert ov is not None and ov.kind == "confirmed"
