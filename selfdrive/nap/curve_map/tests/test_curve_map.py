import math
import random

from openpilot.selfdrive.nap.curve_map.blend import MapFeed, merge_map_points
from openpilot.selfdrive.nap.curve_map.gate import MapTrust, match_ok, near_agrees
from openpilot.selfdrive.nap.curve_map.geometry import curvature
from openpilot.selfdrive.nap.curve_map.pack import resample, tile_key, tile_path, write_tile
from openpilot.selfdrive.nap.curve_map.planner_hook import CurveMapSource
from openpilot.selfdrive.nap.curve_map.preview import MapCurvePreview
from openpilot.selfdrive.nap.curve_map.worker import CurveMapWorker

R_E = 6371000.0
LAT0, LON0 = 45.0, -95.0


def _ll(x, y):
  return LAT0 + math.degrees(y / R_E), LON0 + math.degrees(x / (R_E * math.cos(math.radians(LAT0))))


def _arc_xy(R, deg, step=5.0, noise=0.0, seed=1):
  rnd = random.Random(seed)
  n = int(math.radians(deg) * R / step)
  return [(R * math.sin(i * step / R) + rnd.uniform(-noise, noise), R * (1 - math.cos(i * step / R)) + rnd.uniform(-noise, noise))
          for i in range(n + 1)]


def test_curvature_recovers_radius_through_node_noise():
  k = curvature(_arc_xy(100.0, 120.0, noise=0.8), 5.0)
  mid = k[len(k) // 3: 2 * len(k) // 3]
  assert all(0.007 < abs(x) < 0.013 for x in mid)


def test_curvature_straight_is_zero():
  assert max(abs(x) for x in curvature([(0.0, 5.0 * i) for i in range(40)], 5.0)) < 1e-9


def test_resample_spacing_and_end():
  pts = resample([_ll(0, 0), _ll(0, 103)])
  assert len(pts) == 22 and abs(pts[-1][0] - _ll(0, 103)[0]) < 1e-9


def _pack(tmp_path):
  # way 1 runs north 300 m, way 2 continues from its end with a 90 deg left bend (R 60 m), way 3 is a side street
  w1 = [_ll(0, y) for y in range(0, 301, 20)]
  arc = [(-(60 - 60 * math.cos(a)), 300 + 60 * math.sin(a)) for a in [math.radians(d) for d in range(0, 91, 5)]]
  w2 = [_ll(x, y) for x, y in arc] + [_ll(-60 - 200, 360)]
  w3 = [_ll(0, 300), _ll(150, 300)]
  key = tile_key(LAT0, LON0)
  bbox = (LAT0 - 0.1, LON0 - 0.1, LAT0 + 0.1, LON0 + 0.1)
  write_tile([(1, w1), (2, w2), (3, w3)], tile_path(str(tmp_path), key), bbox, "test")
  return str(tmp_path)


def test_preview_walks_the_join_and_finds_the_bend(tmp_path):
  p = MapCurvePreview(_pack(tmp_path))
  la, lo = _ll(0.5, 50)
  q = p.query(la, lo, 0.0, 450.0)
  assert q is not None and q.way_id == 1 and q.ways[:2] == (1, 2) and q.match_m < 2
  near = [k for d, k in q.pts if d < 150]
  bend = [k for d, k in q.pts if 280 < d < 330]
  assert max(near) < 0.002 and 0.010 < max(bend) < 0.022


def test_preview_none_off_map(tmp_path):
  p = MapCurvePreview(_pack(tmp_path))
  assert p.query(LAT0 + 0.05, LON0 + 0.05, 0.0, 300.0) is None
  assert p.query(*_ll(0, 50), 90.0, 300.0) is None  # heading across the road


def test_gate():
  assert match_ok(0.5, 3.0, 5.0, (7, 8), 8, 4.0)
  assert not match_ok(0.5, 3.0, 5.0, (7, 8), 9, 4.0)   # mapd is on another way
  assert not match_ok(3.0, 3.0, 5.0, (7,), 0, 0.0)     # stale
  assert not match_ok(0.5, 20.0, 5.0, (7,), 0, 0.0)    # loose match
  ts = [0.5, 1.0, 1.5, 2.0]
  assert near_agrees(20.0, ts, [0.01] * 4, [(0.0, 0.01), (100.0, 0.01)])
  assert near_agrees(20.0, ts, [0.01] * 4, [(0.0, 0.001), (100.0, 0.001)]) is False
  assert near_agrees(20.0, ts, [0.0] * 4, [(0.0, 0.02), (100.0, 0.02)]) is False
  t = MapTrust()
  assert t.update(0.5, True, None) == 0.25 and t.update(0.5, True, False) == 0.0
  assert t.update(1.0, True, True) == 0.0  # mismatch hold


def test_merge_fades_near_points_and_scales_by_trust():
  ts, ds, ks, w = merge_map_points(20.0, [0.0], [0.0], [0.0], [1.0], [(30.0, 0.02), (200.0, 0.02)], 0.5)
  assert len(ts) == 2 and ds[-1] == 200.0 and w[-1] == 0.5
  assert len(merge_map_points(20.0, [0.0], [0.0], [0.0], [1.0], [(200.0, 0.02)], 0.0)[0]) == 1


def test_feed_moves_points_with_travel(tmp_path):
  p = MapCurvePreview(_pack(tmp_path))
  f = MapFeed()
  f.set(p.query(*_ll(0.5, 50), 0.0, 450.0))
  d0 = f.pts[20][0]  # ~200 m: past the fade
  for _ in range(20):
    f.merge(0.05, 20.0, [0.0, 1.0, 2.0], [0.0, 20.0, 40.0], [0.0, 0.0, 0.0], [1.0, 1.0, 1.0])
  ts2, ds2, _, _ = f.merge(0.05, 20.0, [0.0, 1.0, 2.0], [0.0, 20.0, 40.0], [0.0, 0.0, 0.0], [1.0, 1.0, 1.0])
  assert f.trust > 0.0 and abs(f.travel - 21 * 1.0) < 1e-6
  assert any(abs(d - (d0 - 20.0)) < 1e-6 for d in ds2)


def test_worker_and_hook_inert_without_pack(tmp_path):
  w = CurveMapWorker(str(tmp_path / "missing"), start=False)
  assert not w.available
  feed, way, _ = CurveMapSource(w).update(None, 20.0)
  assert feed is None and way == 0


def test_worker_run_once(tmp_path):
  w = CurveMapWorker(_pack(tmp_path), start=False)
  (q,) = w.run_once((*_ll(0.5, 50), 0.0, 20.0))
  assert q is not None and q.pts
