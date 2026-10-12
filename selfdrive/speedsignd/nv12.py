"""ROAD-camera NV12 helpers. Y for the numpy fallback; RGB for the YOLO ONNX."""
from __future__ import annotations

import numpy as np


def y_plane_from_nv12(buf) -> np.ndarray | None:
  """Y plane of an NV12 VisionBuf, cropped to width x height (no padding)."""
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
    return y[:, :width]
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
    uv_off = stride * height
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
