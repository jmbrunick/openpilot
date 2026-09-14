"""Windshield rain / ice check from the comma 3X road camera.

There is no rain sensor, and this driving model does not publish rainProb.
Auto looks at the unwarped ROAD camera Y plane (camerad VisionIpc
VISION_STREAM_ROAD). The 3X looks through the windshield, focused on the
far scene, so drops on the glass are large soft defocused bokeh — not
sharp beads. The model warp crops that near field away, so this does not
use modelV2.

Rain: large low-frequency bokeh in the upper/mid ROAD bands, or sparse
    streaks/speckles. Ice/frost: a milky sheet or crystal mottle.
Dense in-focus texture (foliage, brick) and large saturated headlamp
plates are rejected. Hysteresis holds while the glass looks obstructed
and releases when it looks clear.

VisionIpc failures retry; they do not permanently dry Auto.
"""
from __future__ import annotations

import time

import numpy as np

# Unwarped ROAD. Far-focus windshield drops sit as large bokeh across the
# driving view, not only in a thin top strip. Ice-sheet haze uses mid.
_NEAR_ROWS = (0.06, 0.34)
# Mid of the unwarped ROAD view — Justin's live UI shows the soft
# circles of confusion over the scene here. Start below the usual
# headlamp row so saturated plates do not leak into this band.
_BOKEH_ROWS = (0.22, 0.56)
_MID_ROWS = (0.40, 0.70)
_COLS = (0.12, 0.88)
_RAIN_BANDS = (_NEAR_ROWS, _BOKEH_ROWS)

# Rain. Detrended bokeh ~4 on wet ROAD, ~0 on dry sky gradients.
# Speckle-drop path stays ~2–4.
SCORE_ON = 1.8
SCORE_OFF = 1.0
# Justin's heavier-rain ROAD UI: mid-band detrended bokeh ~1.85–3.3.
# Dry sky/road wash is ~0.2 after row-detrend.
BOKEH_ON = 1.5
_BOKEH_RATIO = 0.22
_SPECKLE_MIN = 0.012
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
DEBUG_LOG_S = 1.0
_SAT_MAX = 0.08
_SPARSE_MIN = 8.0
# Dense bead rain on glass is ~6.5–8.5. Justin's upper droplet crop was
# rain-rejected at 6.85. Foliage is still ~1.8.
_SPARSE_RAIN_MIN = 6.0
_SPARSE_MAX = 80.0
STREAM_FALLBACK_S = 3.0
_STRUCTURE_FRAC = 0.05
_STRUCTURE_RAIN = 0.12
_FOLIAGE_BLOB = 18.0
_FINE_R = 2
_MED_R = 4
_COARSE_R = 8


def y_plane_from_nv12(buf) -> np.ndarray | None:
  """Y plane of an NV12 VisionBuf, cropped to width x height (no padding)."""
  try:
    width = int(buf.width)
    height = int(buf.height)
    stride = int(getattr(buf, "stride", 0) or width)
    if width < 16 or height < 16 or stride < width:
      return None
    data = buf.data
    if data is None:
      return None
    n = stride * height
    uv_off = int(getattr(buf, "uv_offset", 0) or 0)
    if uv_off >= n:
      n = uv_off
    raw = np.asarray(memoryview(data) if not isinstance(data, np.ndarray) else data.reshape(-1), dtype=np.uint8)
    if raw.size < stride * height:
      return None
    y = raw[:stride * height].reshape(height, stride)
    return np.ascontiguousarray(y[:, :width])
  except Exception:
    return None


def _band(y: np.ndarray, rows: tuple[float, float], cols: tuple[float, float]) -> np.ndarray:
  h, w = y.shape
  r0, r1 = int(h * rows[0]), int(h * rows[1])
  c0, c1 = int(w * cols[0]), int(w * cols[1])
  r1 = max(r1, r0 + 4)
  c1 = max(c1, c0 + 4)
  return y[r0:r1, c0:c1]


def _box_blur(x: np.ndarray, radius: int) -> np.ndarray:
  if radius < 1:
    return x
  k = np.ones(2 * radius + 1, dtype=np.float32) / float(2 * radius + 1)
  p = np.pad(x, ((0, 0), (radius, radius)), mode="edge")
  acc = np.zeros_like(x)
  for i, w in enumerate(k):
    acc += w * p[:, i:i + x.shape[1]]
  p2 = np.pad(acc, ((radius, radius), (0, 0)), mode="edge")
  out = np.zeros_like(x)
  for i, w in enumerate(k):
    out += w * p2[i:i + x.shape[0], :]
  return out


def _near_features(img: np.ndarray) -> tuple[float, float, float, float, float, float]:
  """Fine blob, speckle, sparsity, sat, strong-edge frac, coarse bokeh energy."""
  if img.size < 25:
    return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  step = max(1, min(img.shape) // 24)
  x = img[::step, ::step].astype(np.float32)
  if x.shape[0] < 5 or x.shape[1] < 5:
    return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  fine = _box_blur(x, _FINE_R)
  med = _box_blur(x, _MED_R)
  coarse = _box_blur(x, _COARSE_R)
  resid = np.abs(x - fine)
  blob = float(resid.mean())
  speckle = float(np.mean((x > fine + 18.0) & (resid > 18.0)))
  p90 = float(np.percentile(resid, 90))
  p50 = float(np.median(resid))
  sparse = p90 / (p50 + 0.05)
  sat = float((x > 240.0).mean())
  structure = float((resid > 40.0).mean())
  # Row-detrend so a vertical sky/road wash is not counted as rain blobs.
  bp = np.abs(med - coarse)
  bp = bp - bp.mean(axis=1, keepdims=True)
  bokeh = float(np.mean(np.abs(bp)))
  return blob, speckle, sparse, sat, structure, bokeh


def _mid_stats(y: np.ndarray) -> tuple[float, float]:
  mid = _band(y, _MID_ROWS, _COLS).astype(np.float32)
  mean = float(mid.mean())
  contrast = float(mid.std() / (mean + 1.0))
  return mean, contrast


def _rain_from_band(img: np.ndarray) -> float:
  """Soft far-focus bokeh and/or sparse speckle drops. Not foliage or lamps."""
  blob, speckle, sparse, sat, structure, bokeh = _near_features(img)
  if sat > _SAT_MAX:
    return 0.0
  if structure > _STRUCTURE_RAIN or blob > _FOLIAGE_BLOB:
    return 0.0
  score = 0.0
  if bokeh >= BOKEH_ON and bokeh / (blob + 0.2) >= _BOKEH_RATIO:
    score = max(score, bokeh)
  if speckle >= _SPECKLE_MIN and _SPARSE_RAIN_MIN <= sparse <= _SPARSE_MAX:
    score = max(score, blob + 12.0 * speckle)
  return score


def windshield_rain_score(y: np.ndarray) -> float:
  """Higher = rainier glass (far-focus bokeh, streaks, speckles).

  3X ROAD is focused on the scene, so windshield drops are large soft
  circles of confusion. Dense in-focus texture (foliage, brick) and large
  saturated headlamp plates are not treated as rain.
  """
  if y is None or y.ndim != 2 or y.shape[0] < 32 or y.shape[1] < 32:
    return 0.0
  return max(_rain_from_band(_band(y, rows, _COLS)) for rows in _RAIN_BANDS)


def windshield_frost_score(y: np.ndarray) -> float:
  """Crystals on the glass: moderate uniform residual, not foliage, not rain drops."""
  if y is None or y.ndim != 2 or y.shape[0] < 32 or y.shape[1] < 32:
    return 0.0
  blob, _speckle, sparse, sat, structure, _bokeh = _near_features(_band(y, _NEAR_ROWS, _COLS))
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
  blob, _speckle, _sparse, sat, structure, _bokeh = _near_features(_band(y, _NEAR_ROWS, _COLS))
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
    self.last_bokeh = 0.0
    self.last_blob = 0.0
    self.last_speckle = 0.0
    self.last_sparse = 0.0
    self.last_sat = 0.0
    self.last_structure = 0.0
    self.n_frames = 0
    self.connected = False
    self.last_err = ""
    self.stream = "ROAD"
    self._client = None
    self._failed = False
    self._last_frame_t = 0.0
    self._last_connect_t = 0.0
    self._last_log_t = 0.0
    self._stream_idx = 0
    self._stream_t0 = 0.0

  def _record_band(self, y: np.ndarray) -> None:
    best = (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    for rows in _RAIN_BANDS:
      blob, speckle, sparse, sat, structure, bokeh = _near_features(_band(y, rows, _COLS))
      if bokeh > best[0]:
        best = (bokeh, blob, speckle, sparse, sat, structure, bokeh)
    _bokeh, blob, speckle, sparse, sat, structure, bokeh = best
    self.last_blob = blob
    self.last_speckle = speckle
    self.last_sparse = sparse
    self.last_sat = sat
    self.last_structure = structure
    self.last_bokeh = bokeh

  def update_from_y(self, y: np.ndarray) -> bool:
    self._record_band(y)
    score = windshield_obstruction_score(y)
    self.last_score = score
    self.ema = EMA_ALPHA * score + (1.0 - EMA_ALPHA) * self.ema
    if self.hold:
      self.hold = self.ema >= HOLD_OFF
    else:
      self.hold = self.ema >= HOLD_ON
    self._last_frame_t = time.monotonic()
    self.n_frames += 1
    self._debug()
    return self.hold

  def _stream_names(self) -> tuple[str, ...]:
    return ("VISION_STREAM_ROAD", "VISION_STREAM_WIDE_ROAD")

  def _maybe_fallback_stream(self, now: float) -> None:
    if self.n_frames > 0:
      return
    if self._stream_t0 <= 0:
      self._stream_t0 = now
      return
    if now - self._stream_t0 < STREAM_FALLBACK_S:
      return
    names = self._stream_names()
    nxt = (self._stream_idx + 1) % len(names)
    if nxt == self._stream_idx:
      return
    self._stream_idx = nxt
    self.stream = "WIDE" if "WIDE" in names[nxt] else "ROAD"
    self._client = None
    self.connected = False
    self._stream_t0 = now
    self.last_err = f"fallback_{self.stream}"

  def _recv_y(self) -> np.ndarray | None:
    now = time.monotonic()
    if self._failed and (now - self._last_connect_t) < CONNECT_RETRY_S:
      return None
    self._failed = False
    self._maybe_fallback_stream(now)
    try:
      if self._client is None:
        if now - self._last_connect_t < CONNECT_RETRY_S and self._last_connect_t > 0:
          return None
        self._last_connect_t = now
        from msgq.visionipc import VisionIpcClient, VisionStreamType
        st = getattr(VisionStreamType, self._stream_names()[self._stream_idx])
        self._client = VisionIpcClient("camerad", st, True)
        self.stream = "WIDE" if "WIDE" in self._stream_names()[self._stream_idx] else "ROAD"
      self.connected = bool(self._client.is_connected())
      if not self.connected:
        if now - self._last_connect_t < CONNECT_RETRY_S:
          self.last_err = self.last_err or "noconnect"
          return None
        self._last_connect_t = now
        if not self._client.connect(False):
          self.connected = False
          self.last_err = "noconnect"
          return None
        self.connected = True
        self.last_err = ""
      buf = self._client.recv(timeout_ms=0)
      if buf is None:
        if not self.last_err:
          self.last_err = "nobuf"
        return None
      y = y_plane_from_nv12(buf)
      if y is None:
        self.last_err = "noyplane"
        return None
      self.last_err = ""
      return y
    except Exception as e:
      self._failed = True
      self.connected = False
      self._client = None
      self._last_connect_t = now
      self.last_err = type(e).__name__
      return None

  def _debug(self, reason: str = "") -> None:
    now = time.monotonic()
    if now - self._last_log_t < DEBUG_LOG_S:
      return
    self._last_log_t = now
    age_ms = (now - self._last_frame_t) * 1000.0 if self._last_frame_t else -1.0
    err = f" err={self.last_err}" if self.last_err else ""
    why = f" {reason}" if reason else ""
    line = (
      "nap wiper rain hold=%d ema=%.2f score=%.2f bokeh=%.2f blob=%.2f "
      "speckle=%.3f sparse=%.1f sat=%.3f struct=%.3f connected=%d failed=%d "
      "frames=%d stream=%s age_ms=%.0f%s%s"
    )
    args = (
      int(self.hold), self.ema, self.last_score, self.last_bokeh, self.last_blob,
      self.last_speckle, self.last_sparse, self.last_sat, self.last_structure,
      int(self.connected), int(self._failed), self.n_frames, self.stream, age_ms, err, why,
    )
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.info(line, *args)
    except Exception:
      pass
    # Do not write the live Auto status param here. The short hold/ema line
    # was clobbering the Auto gate string (setting/on/gear/drive/wipe)
    # Justin cats on device. body_controls owns that param.

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
        self._debug("stale")
    elif not self.hold:
      self._debug("noframe")
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
