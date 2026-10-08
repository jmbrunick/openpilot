"""ROAD-camera NV12 helpers. Crop+letterbox RGB for YOLO; Y for the numpy fallback.

On-car detect must not convert a full 1928×1208 ROAD frame to RGB. The YOLO
input is 320² of the right-biased short-side crop — convert that, after
downsample, not the whole VisionBuf.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def nv12_uv_offset(buf, stride: int, height: int) -> int:
  """UV plane start. 3X Venus NV12 is page-aligned; not always stride*height.

  camerad sets VisionBuf.uv_offset = stride * ALIGN(height, 32). Using
  stride*visible_height reads the Y pad as chroma and color-shifts RGB
  (white R2-1 → green/yellow) so YOLO never fires.
  """
  raw = getattr(buf, "uv_offset", None)
  try:
    off = int(raw) if raw is not None else 0
  except (TypeError, ValueError):
    off = 0
  if off > 0:
    return off
  return int(stride) * int(height)


def y_plane_from_nv12(buf, copy: bool = False) -> np.ndarray | None:
  """Y plane of an NV12 VisionBuf, cropped to width x height (no padding).

  The default is a view into VisionBuf memory. Pass copy=True before handing
  the plane to a background infer — the next recv can recycle the buffer.
  """
  try:
    width = int(buf.width)
    height = int(buf.height)
    stride = int(buf.stride) if getattr(buf, "stride", 0) else width
    if width < 2 or height < 2 or stride < width:
      return None
    data = buf.data
    n = stride * height
    if data is None or len(data) < n:
      return None
    y = np.frombuffer(data, dtype=np.uint8, count=n).reshape(height, stride)
    plane = y[:, :width]
    return plane.copy() if copy else plane
  except Exception:
    return None


def rgb_from_nv12(buf) -> np.ndarray | None:
  """BT.601 limited-range NV12 → RGB888. Falls back to luma-only if UV is short."""
  y = y_plane_from_nv12(buf)
  if y is None:
    return None
  try:
    height, width = y.shape
    stride = int(buf.stride) if getattr(buf, "stride", 0) else width
    data = buf.data
    uv_off = nv12_uv_offset(buf, stride, height)
    uv_rows = height // 2
    need = uv_off + stride * uv_rows
    if data is None or len(data) < need:
      return np.stack([y, y, y], axis=-1)
    uv = np.frombuffer(data, dtype=np.uint8, count=stride * uv_rows, offset=uv_off)
    uv = uv.reshape(uv_rows, stride)[:, :width]
    u = uv[:, 0::2]
    v = uv[:, 1::2]
    u = np.repeat(np.repeat(u, 2, axis=0), 2, axis=1)[:height, :width]
    v = np.repeat(np.repeat(v, 2, axis=0), 2, axis=1)[:height, :width]
    yf = y.astype(np.float32)
    uf = u.astype(np.float32) - 128.0
    vf = v.astype(np.float32) - 128.0
    r = yf + 1.402 * vf
    g = yf - 0.344136 * uf - 0.714136 * vf
    b = yf + 1.772 * uf
    rgb = np.stack([r, g, b], axis=-1)
    return np.clip(rgb, 0, 255).astype(np.uint8)
  except Exception:
    return np.stack([y, y, y], axis=-1)


def rgb_from_y(y: np.ndarray) -> np.ndarray:
  """Replicate a uint8 Y plane to RGB (tests / missing chroma)."""
  if y.ndim == 3 and y.shape[-1] == 3:
    return y
  return np.stack([y, y, y], axis=-1)


@dataclass
class Nv12DetectCrop:
  """Copied right-biased ROAD crop. Small enough to hand to a background infer."""
  y: np.ndarray
  uv: np.ndarray | None
  frame_w: int
  frame_h: int
  crop: tuple[int, int, int, int]


def detect_crop_rect(h: int, w: int) -> tuple[int, int, int, int]:
  """Same right-biased short-side square as yolo.road_detect_crop_rect."""
  side = min(int(h), int(w))
  x = max(0, int(w) - side)
  y = max(0, (int(h) - side) // 2)
  return x, y, side, side


def copy_nv12_detect_crop(buf) -> Nv12DetectCrop | None:
  """Copy Y+UV for the detect crop only. No full-frame RGB.

  VisionBuf memory is recycled on the next recv — the infer thread needs its
  own copy. A 1208×1208 crop is ~2.2 MB vs a 1928×1208 RGB888 frame (~7 MB)
  plus the float32 BT.601 temporaries.
  """
  y_full = y_plane_from_nv12(buf, copy=False)
  if y_full is None:
    return None
  frame_h, frame_w = int(y_full.shape[0]), int(y_full.shape[1])
  cx, cy, cw, ch = detect_crop_rect(frame_h, frame_w)
  if cw < 2 or ch < 2:
    return None
  y = np.ascontiguousarray(y_full[cy:cy + ch, cx:cx + cw]).copy()
  uv = _copy_uv_crop(buf, frame_w, frame_h, cx, cy, cw, ch)
  return Nv12DetectCrop(y=y, uv=uv, frame_w=frame_w, frame_h=frame_h, crop=(cx, cy, cw, ch))


def _copy_uv_crop(buf, frame_w: int, frame_h: int, cx: int, cy: int, cw: int, ch: int) -> np.ndarray | None:
  try:
    stride = int(buf.stride) if getattr(buf, "stride", 0) else frame_w
    data = buf.data
    uv_off = nv12_uv_offset(buf, stride, frame_h)
    uv_rows = frame_h // 2
    need = uv_off + stride * uv_rows
    if data is None or len(data) < need:
      return None
    uv = np.frombuffer(data, dtype=np.uint8, count=stride * uv_rows, offset=uv_off)
    uv = uv.reshape(uv_rows, stride)[:, :frame_w]
    y0 = cy // 2
    y1 = (cy + ch) // 2
    plane = uv[y0:y1, cx:cx + cw]
    if plane.size == 0 or plane.shape[1] < 2:
      return None
    return np.ascontiguousarray(plane).copy()
  except Exception:
    return None


def _resize_gray(img: np.ndarray, h: int, w: int) -> np.ndarray:
  if img.shape[0] == h and img.shape[1] == w:
    return img
  try:
    import cv2
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
  except Exception:
    return _resize_gray_bilinear(img, h, w)


def _resize_gray_bilinear(img: np.ndarray, h: int, w: int) -> np.ndarray:
  src_h, src_w = img.shape[:2]
  if src_h < 1 or src_w < 1:
    return np.zeros((h, w), dtype=np.uint8)
  ys = (np.arange(h, dtype=np.float32) + 0.5) * (src_h / float(h)) - 0.5
  xs = (np.arange(w, dtype=np.float32) + 0.5) * (src_w / float(w)) - 0.5
  ys = np.clip(ys, 0.0, src_h - 1.0)
  xs = np.clip(xs, 0.0, src_w - 1.0)
  y0 = np.floor(ys).astype(np.int32)
  x0 = np.floor(xs).astype(np.int32)
  y1 = np.minimum(y0 + 1, src_h - 1)
  x1 = np.minimum(x0 + 1, src_w - 1)
  wy = (ys - y0).astype(np.float32)[:, None]
  wx = (xs - x0).astype(np.float32)[None, :]
  img_f = img.astype(np.float32)
  i00 = img_f[y0][:, x0]
  i01 = img_f[y0][:, x1]
  i10 = img_f[y1][:, x0]
  i11 = img_f[y1][:, x1]
  out = (i00 * (1.0 - wx) + i01 * wx) * (1.0 - wy) + (i10 * (1.0 - wx) + i11 * wx) * wy
  return np.clip(np.rint(out), 0, 255).astype(np.uint8)


def _bt601_yuv_to_rgb(y: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
  yf = y.astype(np.float32)
  uf = u.astype(np.float32) - 128.0
  vf = v.astype(np.float32) - 128.0
  r = yf + 1.402 * vf
  g = yf - 0.344136 * uf - 0.714136 * vf
  b = yf + 1.772 * uf
  return np.clip(np.stack([r, g, b], axis=-1), 0, 255).astype(np.uint8)


def letterbox_rgb_from_nv12_crop(crop: Nv12DetectCrop, size: int) -> tuple[np.ndarray, float, int, int]:
  """Downsample the square NV12 crop to size² RGB. No full-frame convert.

  Square crop → letterbox scale is size/side with no pad. YOLO still sees
  chroma (U/V resized with the luma), just not 2.3M RGB pixels.
  """
  y = crop.y
  if y.ndim != 2 or y.shape[0] < 2 or y.shape[1] < 2:
    raise ValueError("letterbox_rgb_from_nv12_crop expects a 2-D Y crop")
  src_h, src_w = y.shape
  scale = min(size / float(src_h), size / float(src_w))
  nh = max(1, int(round(src_h * scale)))
  nw = max(1, int(round(src_w * scale)))
  y_s = _resize_gray(y, nh, nw)
  rgb_s = _rgb_from_resized_nv12(y_s, crop.uv)
  canvas = np.full((size, size, 3), 114, dtype=np.uint8)
  top = (size - nh) // 2
  left = (size - nw) // 2
  canvas[top:top + nh, left:left + nw] = rgb_s
  return canvas, scale, left, top


def _rgb_from_resized_nv12(y_s: np.ndarray, uv: np.ndarray | None) -> np.ndarray:
  nh, nw = y_s.shape
  if uv is None or uv.size == 0 or uv.shape[1] < 2:
    return np.stack([y_s, y_s, y_s], axis=-1)
  u = uv[:, 0::2]
  v = uv[:, 1::2]
  if u.shape[0] < 1 or u.shape[1] < 1:
    return np.stack([y_s, y_s, y_s], axis=-1)
  u_s = _resize_gray(u, nh, nw)
  v_s = _resize_gray(v, nh, nw)
  return _bt601_yuv_to_rgb(y_s, u_s, v_s)
