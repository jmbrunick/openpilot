"""Wet-windshield check from the comma 3X road camera.

The driving model does not publish rainProb. Auto looks at the unwarped ROAD
camera Y plane (the 3X looks through the windshield). Drops on the glass sit
in the near field and show up as out-of-focus blobs in the top of that frame —
the model warp crops them away, so this does not use modelV2.

Wet glass: sparse blob energy in that near-glass band. Dense in-focus texture
(foliage, brick) and large saturated headlamp plates are rejected. Hysteresis
holds while the glass looks wet and releases when it looks dry.
"""
from __future__ import annotations

import time

import numpy as np

# Unwarped ROAD frame. Top band is windshield / near glass.
_NEAR_ROWS = (0.08, 0.32)
_COLS = (0.12, 0.88)

# Hold while EMA is above ON; release below OFF.
# Wet glass: sparse defocused blobs in the near band (score ~2–4).
# Dry sky: ~0. Dense foliage / brick is rejected (not sparse). Saturated
# headlamp plates are rejected (too much of the band is clipped).
SCORE_ON = 1.8
SCORE_OFF = 1.0
EMA_ALPHA = 0.35
STALE_S = 2.0
CONNECT_RETRY_S = 0.5
_SAT_MAX = 0.08
_SPARSE_MIN = 8.0


def y_plane_from_nv12(buf) -> np.ndarray | None:
  """Y plane of an NV12 VisionBuf, cropped to width x height (no padding)."""
  try:
    width = int(buf.width)
    height = int(buf.height)
    stride = int(buf.stride) if getattr(buf, "stride", 0) else width
    if width < 16 or height < 16 or stride < width:
      return None
    data = buf.data
    n = stride * height
    if data is None or len(data) < n:
      return None
    y = np.frombuffer(data, dtype=np.uint8, count=n).reshape(height, stride)
    return y[:, :width]
  except Exception:
    return None


def _band(y: np.ndarray, rows: tuple[float, float], cols: tuple[float, float]) -> np.ndarray:
  h, w = y.shape
  r0, r1 = int(h * rows[0]), int(h * rows[1])
  c0, c1 = int(w * cols[0]), int(w * cols[1])
  r1 = max(r1, r0 + 4)
  c1 = max(c1, c0 + 4)
  return y[r0:r1, c0:c1]


def _near_features(img: np.ndarray) -> tuple[float, float, float, float]:
  """Blob energy, speckle fraction, residual sparsity (p90/p50), sat fraction."""
  if img.size < 25:
    return 0.0, 0.0, 0.0, 0.0
  step = max(1, min(img.shape) // 24)
  x = img[::step, ::step].astype(np.float32)
  if x.shape[0] < 5 or x.shape[1] < 5:
    return 0.0, 0.0, 0.0, 0.0
  k = np.array([1.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float32) / 5.0
  p = np.pad(x, 2, mode="edge")
  acc = np.zeros_like(x)
  for i, w in enumerate(k):
    acc += w * p[2:-2, i:i + x.shape[1]]
  p2 = np.pad(acc, ((2, 2), (0, 0)), mode="edge")
  blur = np.zeros_like(x)
  for i, w in enumerate(k):
    blur += w * p2[i:i + x.shape[0], :]
  resid = np.abs(x - blur)
  blob = float(resid.mean())
  speckle = float(np.mean((x > blur + 18.0) & (resid > 18.0)))
  p90 = float(np.percentile(resid, 90))
  p50 = float(np.median(resid))
  sparse = p90 / (p50 + 0.05)
  sat = float((x > 240.0).mean())
  return blob, speckle, sparse, sat


def windshield_rain_score(y: np.ndarray) -> float:
  """Higher = wetter near glass.

  Drops on the windshield are sparse, slightly defocused blobs in the top of
  the unwarped ROAD frame. Dense in-focus texture (foliage, brick) and large
  saturated headlamp plates are not treated as rain.
  """
  if y is None or y.ndim != 2 or y.shape[0] < 32 or y.shape[1] < 32:
    return 0.0
  blob, speckle, sparse, sat = _near_features(_band(y, _NEAR_ROWS, _COLS))
  if sat > _SAT_MAX:
    return 0.0
  if sparse < _SPARSE_MIN:
    return 0.0
  return blob + 12.0 * speckle


def windshield_looks_rainy(y: np.ndarray, prev_hold: bool = False) -> bool:
  score = windshield_rain_score(y)
  if prev_hold:
    return score >= SCORE_OFF
  return score >= SCORE_ON


class WindshieldRain:
  """Live 3X ROAD-camera wet-glass latch for Wiper Auto."""

  def __init__(self):
    self.hold = False
    self.ema = 0.0
    self.last_score = 0.0
    self._client = None
    self._failed = False
    self._last_frame_t = 0.0
    self._last_connect_t = 0.0

  def update_from_y(self, y: np.ndarray) -> bool:
    score = windshield_rain_score(y)
    self.last_score = score
    self.ema = EMA_ALPHA * score + (1.0 - EMA_ALPHA) * self.ema
    if self.hold:
      self.hold = self.ema >= SCORE_OFF
    else:
      self.hold = self.ema >= SCORE_ON
    self._last_frame_t = time.monotonic()
    return self.hold

  def _recv_y(self) -> np.ndarray | None:
    if self._failed:
      return None
    now = time.monotonic()
    try:
      if self._client is None:
        if now - self._last_connect_t < CONNECT_RETRY_S:
          return None
        self._last_connect_t = now
        from msgq.visionipc import VisionIpcClient, VisionStreamType
        self._client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
      if not self._client.is_connected():
        if now - self._last_connect_t < CONNECT_RETRY_S:
          return None
        self._last_connect_t = now
        if not self._client.connect(False):
          return None
      buf = self._client.recv(timeout_ms=0)
      if buf is None:
        return None
      return y_plane_from_nv12(buf)
    except Exception:
      self._failed = True
      self._client = None
      return None

  def poll(self) -> bool:
    """Non-blocking. Dry until a frame says the glass is wet; stale → dry."""
    y = self._recv_y()
    if y is not None:
      return self.update_from_y(y)
    if self.hold and self._last_frame_t > 0:
      if time.monotonic() - self._last_frame_t > STALE_S:
        self.hold = False
        self.ema = 0.0
        self.last_score = 0.0
    return self.hold


_detector: WindshieldRain | None = None


def windshield_rain_needed() -> bool:
  global _detector
  if _detector is None:
    _detector = WindshieldRain()
  return _detector.poll()


def reset_windshield_rain() -> None:
  """Tests reset the singleton."""
  global _detector
  _detector = None
