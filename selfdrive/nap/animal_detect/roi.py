"""Tile selection and image geometry. Pure numpy, no openpilot imports at module load.

Road camera: two native-resolution 640x320 tiles side by side, centred on the
vanishing point, so a deer at 100 m is still ~30 px tall (a full-frame
downscale would leave it ~9 px). Wide camera: the whole frame scaled to 640
wide, 320 rows around its horizon, for near and side objects (< ~40 m).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from openpilot.selfdrive.nap.animal_detect.config import TILE_ABOVE_HORIZON, TILE_H, TILE_W, WIDE_SCALE_W


@dataclass(frozen=True)
class CamModel:
  width: int
  height: int
  focal: float
  roll: float = 0.0
  pitch: float = 0.0
  yaw: float = 0.0
  height_m: float = 1.22

  @property
  def cx(self) -> float:
    return self.width / 2.0

  @property
  def cy(self) -> float:
    return self.height / 2.0


@dataclass(frozen=True)
class Tile:
  camera: str          # "road" or "wide"
  x0: int              # source window, source pixels
  y0: int
  w: int               # source window size, source pixels
  h: int
  scale: float         # tile px per source px (1.0 for native road tiles)


def vanishing_point(cam: CamModel) -> tuple[float, float]:
  """Pixel of a far point straight ahead on the road plane (calibrated frame)."""
  # Same rotation as path_obstacle.project_road_point, for x -> infinity.
  cr, sr = math.cos(cam.roll), math.sin(cam.roll)
  cp, sp = math.cos(cam.pitch), math.sin(cam.pitch)
  cyw, syw = math.cos(cam.yaw), math.sin(cam.yaw)
  rx, ry, rz = 1.0, 0.0, 0.0
  ry, rz = cr * ry - sr * rz, sr * ry + cr * rz
  rx, rz = cp * rx + sp * rz, -sp * rx + cp * rz
  rx, ry = cyw * rx - syw * ry, syw * rx + cyw * ry
  if rx <= 1e-6:
    return cam.cx, cam.cy
  return cam.cx + cam.focal * ry / rx, cam.cy + cam.focal * rz / rx


def _clamp(v: float, lo: int, hi: int) -> int:
  return int(min(max(int(round(v)), lo), hi))


def road_tiles(cam: CamModel) -> list[Tile]:
  u_vp, v_vp = vanishing_point(cam)
  y0 = _clamp(v_vp - TILE_ABOVE_HORIZON, 0, cam.height - TILE_H)
  # Two tiles meeting at the vanishing point, kept inside the frame.
  left = _clamp(u_vp - TILE_W, 0, cam.width - 2 * TILE_W)
  return [Tile("road", left, y0, TILE_W, TILE_H, 1.0), Tile("road", left + TILE_W, y0, TILE_W, TILE_H, 1.0)]


def wide_tile(cam: CamModel) -> Tile:
  scale = WIDE_SCALE_W / float(cam.width)
  src_h = int(round(TILE_H / scale))
  _u, v_vp = vanishing_point(cam)
  y0 = _clamp(v_vp - 0.4 * src_h, 0, cam.height - src_h)
  return Tile("wide", 0, y0, cam.width, src_h, scale)


def tile_to_source(tile: Tile, box) -> tuple[float, float, float, float]:
  x0, y0, x1, y1 = (float(b) for b in box)
  s = tile.scale
  return tile.x0 + x0 / s, tile.y0 + y0 / s, tile.x0 + x1 / s, tile.y0 + y1 / s


def bearing_deg(cam: CamModel, u: float) -> float:
  """Horizontal bearing of column u, degrees, + right of straight ahead."""
  u_vp, _ = vanishing_point(cam)
  return math.degrees(math.atan2(u - cam.cx, cam.focal) - math.atan2(u_vp - cam.cx, cam.focal))


def ground_range(cam: CamModel, v_bottom: float) -> float:
  """Flat-road range (m, camera frame) of a box whose bottom row is v_bottom. NaN above the horizon."""
  _u, v_vp = vanishing_point(cam)
  below = math.atan2(v_bottom - cam.cy, cam.focal) - math.atan2(v_vp - cam.cy, cam.focal)
  if below <= math.radians(0.15):
    return float("nan")
  return cam.height_m / math.tan(below)


def lateral_m(range_m: float, bearing: float) -> float:
  if not math.isfinite(range_m):
    return float("nan")
  return range_m * math.tan(math.radians(bearing))


def bgr_from_nv12(y: np.ndarray, uv: np.ndarray, tile: Tile) -> np.ndarray:
  """BT.601 BGR of one tile window, converted at tile size only.

  y is the (H, W) luma plane, uv the (H/2, W) interleaved chroma plane.
  The model was trained on BGR uint8, values 0..255, no normalisation.
  """
  x0, y0 = tile.x0 & ~1, tile.y0 & ~1
  w, h = tile.w, tile.h
  yy = y[y0:y0 + h, x0:x0 + w].astype(np.float32)
  c = uv[y0 // 2:(y0 + h + 1) // 2, x0:x0 + w + (w & 1)]
  u = c[:, 0::2].astype(np.float32) - 128.0
  v = c[:, 1::2].astype(np.float32) - 128.0
  u = np.repeat(np.repeat(u, 2, axis=0), 2, axis=1)[:yy.shape[0], :yy.shape[1]]
  v = np.repeat(np.repeat(v, 2, axis=0), 2, axis=1)[:yy.shape[0], :yy.shape[1]]
  b = yy + 1.772 * u
  g = yy - 0.344136 * u - 0.714136 * v
  r = yy + 1.402 * v
  out = np.stack([b, g, r], axis=-1)
  if tile.scale != 1.0:
    out = _resize_nearest(out, TILE_H, TILE_W)
  return np.clip(out, 0, 255).astype(np.uint8)


def _resize_nearest(img: np.ndarray, h: int, w: int) -> np.ndarray:
  ys = np.minimum((np.arange(h) + 0.5) * img.shape[0] / h, img.shape[0] - 1).astype(np.int32)
  xs = np.minimum((np.arange(w) + 0.5) * img.shape[1] / w, img.shape[1] - 1).astype(np.int32)
  return img[ys][:, xs]


def pad_tile(img: np.ndarray) -> np.ndarray:
  """Pad a short crop (frame edge) to TILE_H x TILE_W with grey 114, like training."""
  if img.shape[0] == TILE_H and img.shape[1] == TILE_W:
    return img
  out = np.full((TILE_H, TILE_W, 3), 114, np.uint8)
  out[:min(TILE_H, img.shape[0]), :min(TILE_W, img.shape[1])] = img[:TILE_H, :TILE_W]
  return out
