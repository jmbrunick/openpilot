import json
import math

from openpilot.selfdrive.speedsignd import sign_map as M
from openpilot.selfdrive.speedsignd import two_stage as T


def _box(d, u_center):
  h = T.FOCAL_PX * T.SIGN_H_M / d
  return (u_center - h * 0.4, 500.0, h * 0.8, h)


def test_straight_ahead_north():
  loc = M.locate(40.0, -90.0, 0.0, _box(30.0, T.CAM_CX))
  assert abs(loc["range_m"] - 30.0) < 0.1 and abs(loc["offset_m"]) < 0.05
  dn = (loc["lat"] - 40.0) * math.pi / 180 * M.EARTH_R_M
  assert abs(dn - 30.0) < 0.2 and abs(loc["lon"] + 90.0) < 1e-6
  assert loc["heading"] == 0.0


def test_right_offset_heading_east():
  d = 40.0
  u = T.CAM_CX + T.FOCAL_PX * (5.0 / d)          # ~5 m right
  loc = M.locate(40.0, -90.0, 90.0, _box(d, u))
  assert 4.5 < loc["offset_m"] < 5.5
  dn = (loc["lat"] - 40.0) * math.pi / 180 * M.EARTH_R_M   # right of east-facing = south
  assert -5.5 < dn < -4.5


def test_uncertainty_grows_with_range_and_needs_bearing():
  near = M.locate(40.0, -90.0, 0.0, _box(20.0, T.CAM_CX), gps_sigma_m=3.0)
  far = M.locate(40.0, -90.0, 0.0, _box(60.0, T.CAM_CX), gps_sigma_m=3.0)
  assert far["sigma_m"] > near["sigma_m"] >= 3.0
  assert M.locate(40.0, -90.0, None, _box(20.0, T.CAM_CX)) is None


def test_append_jsonl(tmp_path):
  p = str(tmp_path / "nap" / "sign_map.jsonl")
  loc = M.locate(40.0, -90.0, 0.0, _box(30.0, T.CAM_CX))
  rec = M.record(45, 0.91, 67.0, loc, 1.5, 2)
  assert M.append(rec, p) and M.append(rec, p)
  with open(p) as f:
    rows = [json.loads(x) for x in f]
  assert len(rows) == 2 and rows[0]["mph"] == 45
  assert {"lat", "lon", "heading", "sigma_m", "px", "conf"} <= set(rows[0])
  assert not M.append(rec, p, max_bytes=1)
