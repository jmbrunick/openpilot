import math
from openpilot.selfdrive.nap.curve_map.pack import way_profile, tile_key
from openpilot.selfdrive.nap.curve_map.blend import merge_map_points


def _arc(lat0, lon0, R, deg, n=12):
  pts = []
  for i in range(n + 1):
    a = math.radians(deg * i / n)
    x, y = R * math.sin(a), R * (1 - math.cos(a))  # left turn heading east
    pts.append((lat0 + math.degrees(y / 6371000), lon0 + math.degrees(x / (6371000 * math.cos(math.radians(lat0))))))
  return pts


def test_profile_recovers_radius():
  prof = way_profile(_arc(45.0, -95.0, 100.0, 90.0))
  kmax = max(abs(p[3]) for p in prof)
  assert 0.007 < kmax < 0.012  # 1/100 m, smoothed


def test_straight_is_zero():
  prof = way_profile([(45.0, -95.0), (45.0, -94.99)])
  assert all(abs(p[3]) < 1e-6 for p in prof)
  assert len(prof) >= 70


def test_merge_fades_near_points():
  ts, ds, ks, w = merge_map_points(20.0, [0.0], [0.0], [0.0], [1.0], [(30.0, 0.02), (200.0, 0.02)])
  assert len(ts) == 2 and ds[-1] == 200.0 and w[-1] == 1.0  # 1.5 s point dropped, 10 s point full


def test_tile_key():
  assert tile_key(45.1, -95.05) == (180, -381)
