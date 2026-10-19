import math

import numpy as np

from openpilot.selfdrive.controls.lib.path_obstacle import project_road_point
from openpilot.selfdrive.nap.animal_detect.config import TILE_H, TILE_W
from openpilot.selfdrive.nap.animal_detect.roi import (
  CamModel, bearing_deg, bgr_from_nv12, ground_range, pad_tile, road_tiles, tile_to_source, vanishing_point, wide_tile,
)

# Route 0000019e (New Ulm deer) liveCalibration.
CAL = dict(roll=0.00086, pitch=-0.0738, yaw=-0.0217, height_m=1.228)
ROAD = CamModel(1928, 1208, 2648.0, **CAL)


def test_vanishing_point_matches_project_road_point():
  u, v = vanishing_point(ROAD)
  pu, pv = project_road_point(5000.0, 0.0, 1.228, roll=CAL["roll"], pitch=CAL["pitch"], yaw=CAL["yaw"],
                              height=1.228, focal=2648.0, cx=964.0, cy=604.0)
  assert abs(u - pu) < 2.0 and abs(v - pv) < 2.0
  assert 790 < v < 810   # this car's camera points up: horizon well below centre


def test_road_tiles_native_and_inside_frame():
  tiles = road_tiles(ROAD)
  assert len(tiles) == 2
  for t in tiles:
    assert (t.w, t.h, t.scale) == (TILE_W, TILE_H, 1.0)
    assert 0 <= t.x0 and t.x0 + t.w <= 1928 and 0 <= t.y0 and t.y0 + t.h <= 1208
  assert tiles[0].x0 + TILE_W == tiles[1].x0


def test_deer_inside_right_tile():
  # Hand-marked deer box, frame 788 of seg 93.
  t = road_tiles(ROAD)[1]
  assert t.x0 <= 1325 and 1432 <= t.x0 + t.w and t.y0 <= 825 and 885 <= t.y0 + t.h


def test_tiles_clamped_for_extreme_calibration():
  cam = CamModel(1928, 1208, 2648.0, pitch=0.5, yaw=0.4)
  for t in road_tiles(cam) + [wide_tile(cam)]:
    assert 0 <= t.x0 and t.x0 + t.w <= 1928 and 0 <= t.y0 and t.y0 + t.h <= 1208


def test_ground_range_inverts_projection():
  for d in (15.0, 30.0, 50.0, 100.0):
    _u, v = project_road_point(d, 0.0, 0.0, roll=0.0, pitch=CAL["pitch"], yaw=CAL["yaw"], height=1.228,
                               focal=2648.0, cx=964.0, cy=604.0)
    assert abs(ground_range(ROAD, v) - d) / d < 0.03


def test_above_horizon_is_nan():
  assert math.isnan(ground_range(ROAD, 700.0))


def test_bearing_sign():
  u_vp, _ = vanishing_point(ROAD)
  assert abs(bearing_deg(ROAD, u_vp)) < 1e-6
  assert bearing_deg(ROAD, u_vp + 200) > 0 > bearing_deg(ROAD, u_vp - 200)


def test_wide_tile_scaled_and_mapped_back():
  wide = CamModel(1928, 1208, 567.0, **CAL)
  t = wide_tile(wide)
  assert t.camera == "wide" and abs(t.scale - TILE_W / 1928) < 1e-9
  x0, y0, x1, y1 = tile_to_source(t, (0, 0, TILE_W, TILE_H))
  assert abs(x1 - x0 - 1928) < 1 and abs(y1 - y0 - t.h) < 2


def test_nv12_grey_and_colour():
  y = np.full((1208, 1928), 128, np.uint8)
  uv = np.full((604, 1928), 128, np.uint8)
  t = road_tiles(ROAD)[0]
  img = bgr_from_nv12(y, uv, t)
  assert img.shape == (TILE_H, TILE_W, 3) and np.all(img == 128)
  uv[:, 1::2] = 200   # V up -> red
  img = bgr_from_nv12(y, uv, t)
  assert img[..., 2].mean() > img[..., 0].mean() + 50
  w = bgr_from_nv12(y, uv, wide_tile(CamModel(1928, 1208, 567.0)))
  assert w.shape == (TILE_H, TILE_W, 3)


def test_pad_tile():
  out = pad_tile(np.zeros((100, 200, 3), np.uint8))
  assert out.shape == (TILE_H, TILE_W, 3) and out[0, 0, 0] == 0 and out[-1, -1, 0] == 114
