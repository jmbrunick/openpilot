"""Windshield rain / ice check from the comma 3X road camera.

There is no rain sensor, and this driving model does not publish rainProb.
Auto looks at the unwarped ROAD camera Y plane (camerad VisionIpc
VISION_STREAM_ROAD). The 3X looks through the windshield; drops, streaks,
frost, and ice sit in the near field of that raw frame — the model warp
crops them away, so this does not use modelV2.

Rain: streaks, speckles, or soft defocused blobs in the near-glass band.
Ice/frost: a milky sheet (collapsed contrast of the scene behind the glass)
or crystals (moderate uniform residual on the glass). Either means the
glass is not clear.

Dense in-focus texture (foliage, brick) and large saturated headlamp plates
are rejected. Hysteresis holds while the glass looks obstructed and releases
when it looks clear.
"""
from __future__ import annotations

import time

import numpy as np

# Unwarped ROAD frame. Top band is windshield / near glass; mid is the scene
# behind the glass (used for ice-sheet haze).
_NEAR_ROWS = (0.08, 0.32)
_MID_ROWS = (0.40, 0.70)
_COLS = (0.12, 0.88)

# Rain (sparse near-glass blobs / speckles / streaks). Wet ~2–4, dry ~0.
SCORE_ON = 1.8
SCORE_OFF = 1.0
# Frost crystals: moderate residual that is not sparse-drop rain and not
# in-focus clutter. Ice sheet: daytime scene contrast collapsed + mottle.
FROST_ON = 1.2
FROST_BLOB_MAX = 10.0
ICE_ON = 1.5
ICE_CONTRAST = 0.085
ICE_LUM = (45.0, 210.0)
# Combined obstruction: 1.0 is the hold line (rain/frost/ice each scaled).
HOLD_ON = 1.0
HOLD_OFF = 0.55
EMA_ALPHA = 0.35
STALE_S = 2.0
CONNECT_RETRY_S = 0.5
_SAT_MAX = 0.08
_SPARSE_MIN = 8.0
_STRUCTURE_FRAC = 0.05


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


def _near_features(img: np.ndarray) -> tuple[float, float, float, float, float]:
  """Blob energy, speckle fraction, residual sparsity (p90/p50), sat, strong-edge frac."""
  if img.size < 25:
    return 0.0, 0.0, 0.0, 0.0, 0.0
  step = max(1, min(img.shape) // 24)
  x = img[::step, ::step].astype(np.float32)
  if x.shape[0] < 5 or x.shape[1] < 5:
    return 0.0, 0.0, 0.0, 0.0, 0.0
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
  structure = float((resid > 40.0).mean())
  return blob, speckle, sparse, sat, structure


def _mid_stats(y: np.ndarray) -> tuple[float, float]:
  mid = _band(y, _MID_ROWS, _COLS).astype(np.float32)
  mean = float(mid.mean())
  contrast = float(mid.std() / (mean + 1.0))
  return mean, contrast


def windshield_rain_score(y: np.ndarray) -> float:
  """Higher = rainier near glass (streaks, speckles, soft blobs).

  Drops on the windshield are sparse, slightly defocused blobs in the top of
  the unwarped ROAD frame. Dense in-focus texture (foliage, brick) and large
  saturated headlamp plates are not treated as rain.
  """
  if y is None or y.ndim != 2 or y.shape[0] < 32 or y.shape[1] < 32:
    return 0.0
  blob, speckle, sparse, sat, _structure = _near_features(_band(y, _NEAR_ROWS, _COLS))
  if sat > _SAT_MAX:
    return 0.0
  if sparse < _SPARSE_MIN:
    return 0.0
  return blob + 12.0 * speckle


def windshield_frost_score(y: np.ndarray) -> float:
  """Crystals on the glass: moderate uniform residual, not foliage, not rain drops."""
  if y is None or y.ndim != 2 or y.shape[0] < 32 or y.shape[1] < 32:
    return 0.0
  blob, _speckle, sparse, sat, structure = _near_features(_band(y, _NEAR_ROWS, _COLS))
  if sat > _SAT_MAX or structure > _STRUCTURE_FRAC:
    return 0.0
  if blob > FROST_BLOB_MAX:
    return 0.0
  if sparse >= _SPARSE_MIN:
    return 0.0
  return blob


def windshield_ice_score(y: np.ndarray) -> float:
  """Ice sheet: daytime view through the glass is milky (contrast collapsed)."""
  if y is None or y.ndim != 2 or y.shape[0] < 32 or y.shape[1] < 32:
    return 0.0
  blob, _speckle, _sparse, sat, structure = _near_features(_band(y, _NEAR_ROWS, _COLS))
  if sat > _SAT_MAX or structure > _STRUCTURE_FRAC or blob > FROST_BLOB_MAX:
    return 0.0
  mean, contrast = _mid_stats(y)
  if not (ICE_LUM[0] < mean < ICE_LUM[1]):
    return 0.0
  haze = ICE_CONTRAST - contrast
  if haze <= 0.0 or blob < 0.5:
    return 0.0
  return 15.0 * haze + blob


def windshield_obstruction_score(y: np.ndarray) -> float:
  """1.0 is the hold line. Rain, frost crystals, or an ice sheet."""
  rain = windshield_rain_score(y) / SCORE_ON
  frost = windshield_frost_score(y) / FROST_ON
  ice = windshield_ice_score(y) / ICE_ON
  return max(rain, frost, ice)


def windshield_looks_rainy(y: np.ndarray, prev_hold: bool = False) -> bool:
  """True when the glass looks rainy or icy/frosted."""
  score = windshield_obstruction_score(y)
  if prev_hold:
    return score >= HOLD_OFF
  return score >= HOLD_ON


windshield_looks_obstructed = windshield_looks_rainy


class WindshieldRain:
  """Live 3X ROAD-camera wet/icy-glass latch for Wiper Auto."""

  def __init__(self):
    self.hold = False
    self.ema = 0.0
    self.last_score = 0.0
    self._client = None
    self._failed = False
    self._last_frame_t = 0.0
    self._last_connect_t = 0.0

  def update_from_y(self, y: np.ndarray) -> bool:
    score = windshield_obstruction_score(y)
    self.last_score = score
    self.ema = EMA_ALPHA * score + (1.0 - EMA_ALPHA) * self.ema
    if self.hold:
      self.hold = self.ema >= HOLD_OFF
    else:
      self.hold = self.ema >= HOLD_ON
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
    """Non-blocking. Clear until a frame says the glass is not; stale → clear."""
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
