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
plates are rejected. Hysteresis holds while the glass looks obstructed.
Dry overcast / scene texture must not count as rain (bokeh bar is above
that false-positive band). A real wipe may look clear for ~0.5 s; HOLD
can ride that dip, then drops quickly once scores stay below rain-level
so clear glass cannot keep the 30 s intermittent forever. Acquire needs
a short run of rain-level scores so a one-frame bokeh flicker cannot
nibble-1 the body into latched Int. Stale ROAD still drops HOLD. Off
still leaves the real stalk (escape). Auto dry extra-forwards rest to
cancel that latch.

Bokeh energy is an 8-bit residual (~0 dry, ~2–5 wet). A live clear-glass
log showed bokeh=49165 — wrong Y scale or a bandpass blowup. Impossible
magnitudes are invalid/dry (they must not latch HOLD or look like rain
returning after a wipe).

VisionIpc is drained on a SCHED_OTHER helper thread (blocking recv,
conflate ROAD then WIDE). card is CTRL_HIGH: stock_cc.update / poll()
only reads the latch. Failures retry; they do not permanently dry Auto.
"""
from __future__ import annotations

import os
import threading
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
# Dry sky/road wash is ~0.2. Live clear/overcast texture was latching at
# 1.5 — raise the bar so that is not rain. Real wet is still ~3+.
BOKEH_ON = 2.2
_BOKEH_RATIO = 0.22
# 8-bit ROAD: wet bokeh ~2–5. Justin's clear-glass log was bokeh=49165.
# Anything this high is a unit/scale bug, not rain.
BOKEH_ABSURD = 24.0
BLOB_ABSURD = 40.0
SCORE_ABSURD = 12.0
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
# Instant looks_rainy hysteresis only. Latch release uses HOLD_ON (below).
HOLD_OFF = 0.70
EMA_ALPHA = 0.35
# Same as acquire. A slow hold-EMA was trapping residual scores.
EMA_HOLD_ALPHA = 0.35
# ROAD ~20 Hz. 12 frames ≈ 0.6 s (≈ 1.2 s at 10 Hz). Bias dry-release:
# stuck 30 s intermittent on clear glass is worse than dropping HOLD on a
# long wipe (rain re-acquires). 48/20 never finished on false bokeh.
CLEAR_RELEASE_N = 12
# Blade-start flash only.
MIN_HOLD_N = 4
# Brief wipe-clear that must keep HOLD (~0.4 s at 20 Hz).
WIPE_CLEAR_N = 8
STALE_S = 2.0
CONNECT_RETRY_S = 0.5
DEBUG_LOG_S = 1.0
HELPER_RECV_MS = 100
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
    # NV12 Y is bytes. np.asarray(..., dtype=uint8) on a uint16/float view
    # wraps or reinterprets and the bokeh residual explodes (49165 vs ~4).
    if isinstance(data, np.ndarray) and data.dtype == np.uint8:
      raw = data.reshape(-1)
    else:
      raw = np.frombuffer(memoryview(data).cast("B"), dtype=np.uint8)
    if raw.size < stride * height:
      return None
    y = raw[:stride * height].reshape(height, stride)
    return np.ascontiguousarray(y[:, :width])
  except Exception:
    return None


def _to_y8(img: np.ndarray) -> np.ndarray | None:
  """8-bit Y, 0–255. 10/16-bit planes are scaled. Garbage dtype/range is None."""
  if img is None or getattr(img, "ndim", 0) != 2 or img.size < 25:
    return None
  a = np.asarray(img)
  if a.dtype == np.uint8:
    return a
  af = a.astype(np.float32, copy=False)
  if not np.isfinite(af).all():
    return None
  mx = float(af.max()) if af.size else 0.0
  mn = float(af.min()) if af.size else 0.0
  if mx > 65535.5 or mn < -1.0:
    return None
  if mx > 255.5:
    if mx <= 1023.5:
      af = af * (255.0 / 1023.0)
    elif mx <= 4095.5:
      af = af * (255.0 / 4095.0)
    else:
      af = af * (255.0 / 65535.0)
  elif mx <= 1.5 and mn >= -0.05:
    af = af * 255.0
  return np.clip(af, 0.0, 255.0).astype(np.uint8)


def _finite_score(score: float) -> float:
  try:
    s = float(score)
  except (TypeError, ValueError):
    return 0.0
  if not np.isfinite(s) or s < 0.0 or s > SCORE_ABSURD:
    return 0.0
  return s


def _band(y: np.ndarray, rows: tuple[float, float], cols: tuple[float, float]) -> np.ndarray:
  h, w = y.shape
  r0, r1 = int(h * rows[0]), int(h * rows[1])
  c0, c1 = int(w * cols[0]), int(w * cols[1])
  r1 = max(r1, r0 + 4)
  c1 = max(c1, c0 + 4)
  return y[r0:r1, c0:c1]


def _box_blur(x: np.ndarray, radius: int) -> np.ndarray:
  x = np.asarray(x, dtype=np.float32)
  if radius < 1:
    return x
  k = np.ones(2 * radius + 1, dtype=np.float32) / float(2 * radius + 1)
  p = np.pad(x, ((0, 0), (radius, radius)), mode="edge")
  acc = np.zeros(x.shape, dtype=np.float32)
  for i, w in enumerate(k):
    acc += w * p[:, i:i + x.shape[1]]
  p2 = np.pad(acc, ((radius, radius), (0, 0)), mode="edge")
  out = np.zeros(x.shape, dtype=np.float32)
  for i, w in enumerate(k):
    out += w * p2[i:i + x.shape[0], :]
  return out


def _near_features(img: np.ndarray) -> tuple[float, float, float, float, float, float]:
  """Fine blob, speckle, sparsity, sat, strong-edge frac, coarse bokeh energy."""
  z = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  y8 = _to_y8(img)
  if y8 is None or y8.size < 25:
    return z
  step = max(1, min(y8.shape) // 24)
  x = y8[::step, ::step].astype(np.float32, copy=False)
  if x.shape[0] < 5 or x.shape[1] < 5:
    return z
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
  bp = np.clip(np.abs(med - coarse), 0.0, 255.0)
  row_mean = bp.mean(axis=1, keepdims=True)
  if not np.isfinite(blob + speckle + sparse + sat + structure + float(row_mean.mean())):
    return z
  bp = bp - row_mean
  bokeh = float(np.mean(np.abs(bp)))
  # Impossible magnitudes: wrong Y scale or bandpass blowup. Not rain.
  if (not np.isfinite(bokeh) or bokeh > BOKEH_ABSURD or blob > BLOB_ABSURD
      or sparse < 0.0 or sparse > 1.0e4):
    return z
  return blob, speckle, sparse, sat, structure, bokeh


def _mid_stats(y: np.ndarray) -> tuple[float, float]:
  y8 = _to_y8(y)
  if y8 is None:
    return 0.0, 0.0
  mid = _band(y8, _MID_ROWS, _COLS).astype(np.float32)
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
  if bokeh >= BOKEH_ON and bokeh <= BOKEH_ABSURD and bokeh / (blob + 0.2) >= _BOKEH_RATIO:
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
  return _finite_score(max(rain, frost, ice))


def windshield_looks_rainy(y: np.ndarray, prev_hold: bool = False) -> bool:
  """True when the glass looks rainy or icy/frosted."""
  score = windshield_obstruction_score(y)
  if prev_hold:
    return score >= HOLD_OFF
  return score >= HOLD_ON


windshield_looks_obstructed = windshield_looks_rainy


def _drop_realtime() -> None:
  """card is CTRL_HIGH. VisionIpc + numpy must not run on that policy."""
  try:
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  except Exception:
    pass
  try:
    os.nice(5)
  except Exception:
    pass


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
    self._lock = threading.Lock()
    self._stop = threading.Event()
    self._helper: threading.Thread | None = None
    self._helper_started = False
    self._poll_recv = 0
    self._clear_n = 0
    self._hold_n = 0

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
    return self._update_score(windshield_obstruction_score(y))

  def _update_score(self, score: float) -> bool:
    """Latch from an obstruction score. Tests inject residual/dry scores here."""
    with self._lock:
      self.last_score = _finite_score(score)
      alpha = EMA_HOLD_ALPHA if self.hold else EMA_ALPHA
      self.ema = alpha * self.last_score + (1.0 - alpha) * self.ema
      if self.hold:
        self._hold_n += 1
        # Bias dry-release: any score below rain-level HOLD_ON counts as
        # clear. False overcast residual must not reset this counter.
        # A swipe-clear is WIPE_CLEAR_N frames, not CLEAR_RELEASE_N in a row.
        if self.last_score < HOLD_ON:
          self._clear_n += 1
        else:
          self._clear_n = 0
        if self._hold_n >= MIN_HOLD_N and self._clear_n >= CLEAR_RELEASE_N:
          self.hold = False
          self._clear_n = 0
          self._hold_n = 0
      else:
        self._clear_n = 0
        # One frame of bokeh~3 with score noise must not latch HOLD (that
        # nibble-1 pulse leaves Pre-AP intermittent until Auto sends rest).
        if self.ema >= HOLD_ON and self.last_score >= HOLD_ON:
          self._hold_n += 1
          if self._hold_n >= MIN_HOLD_N:
            self.hold = True
        else:
          self._hold_n = 0
      self._last_frame_t = time.monotonic()
      self.n_frames += 1
      hold = self.hold
    self._debug()
    return hold

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

  def _recv_y(self, timeout_ms: int = 0) -> np.ndarray | None:
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
      buf = self._client.recv(timeout_ms=int(timeout_ms))
      if buf is None:
        # Helper blocking timeout is normal. Do not stamp nobuf over a live connect.
        if int(timeout_ms) <= 0 and not self.last_err:
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
      f"nap wiper rain hold={int(self.hold)} ema={self.ema:.2f} score={self.last_score:.2f} "
      + f"bokeh={self.last_bokeh:.2f} blob={self.last_blob:.2f} speckle={self.last_speckle:.3f} "
      + f"sparse={self.last_sparse:.1f} sat={self.last_sat:.3f} struct={self.last_structure:.3f} "
      + f"clear={int(self._clear_n)}/{int(CLEAR_RELEASE_N)} connected={int(self.connected)} "
      + f"failed={int(self._failed)} frames={self.n_frames} stream={self.stream} "
      + f"helper={int(self.helper_alive)} age_ms={age_ms:.0f}{err}{why}"
    )
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.info("%s", line)
    except Exception:
      pass
    # Never put a short hold= line. Refresh the full Auto gate string instead.
    try:
      from openpilot.selfdrive.car.tesla import preap_body_controls as body
      if int(body._param_int(body.NAP_WIPER_SPEED, 0)) == body.WIPER_SETTING_AUTO:
        on = body.vehicle_is_on()
        drive = body.in_drive_gear()
        rain = bool(self.hold)
        body._log_auto_status(body.WIPER_SETTING_AUTO, on, drive, rain, bool(on and drive and rain))
    except Exception:
      pass

  @property
  def helper_alive(self) -> bool:
    t = self._helper
    return bool(t is not None and t.is_alive())

  def start_helper(self) -> None:
    """Drain ROAD off the card thread. Idempotent."""
    t = self._helper
    if t is not None and t.is_alive():
      self._helper_started = True
      return
    if t is not None and t is not threading.current_thread():
      self._stop.set()
      t.join(timeout=0.6)
    # New Event so a stuck previous loop cannot resume after we restart.
    self._stop = threading.Event()
    self._helper_started = True
    try:
      self._helper = threading.Thread(target=self._helper_loop, name="nap-wiper-rain", daemon=True)
      self._helper.start()
    except Exception as e:
      self._helper = None
      self._helper_started = False
      self.last_err = type(e).__name__

  def stop_helper(self) -> None:
    self._stop.set()
    t = self._helper
    if t is not None and t.is_alive() and t is not threading.current_thread():
      t.join(timeout=0.6)
    self._helper = None
    self._helper_started = False

  def _helper_loop(self) -> None:
    _drop_realtime()
    while not self._stop.is_set():
      try:
        y = self._recv_y(timeout_ms=HELPER_RECV_MS)
      except Exception as e:
        self._failed = True
        self.last_err = type(e).__name__
        y = None
      if self._stop.is_set():
        break
      if y is not None:
        self.update_from_y(y)
        continue
      self._apply_stale()
      if not self.connected or self._failed:
        self._stop.wait(CONNECT_RETRY_S)

  def _apply_stale(self) -> None:
    """Clear HOLD if ROAD frames stop. Do not wipe dry glass."""
    if self.hold and self._last_frame_t > 0:
      if time.monotonic() - self._last_frame_t > STALE_S:
        with self._lock:
          self.hold = False
          self.ema = 0.0
          self.last_score = 0.0
          self._clear_n = 0
          self._hold_n = 0
        self._debug("stale")
    elif not self.hold:
      self._debug("noframe")

  def poll(self) -> bool:
    """Latch only when the helper is running. Never recv on the card RT thread."""
    if self._helper_started:
      self._apply_stale()
      return self.hold
    self._poll_recv += 1
    y = self._recv_y(timeout_ms=0)
    if y is not None:
      return self.update_from_y(y)
    self._apply_stale()
    return self.hold


_detector: WindshieldRain | None = None


def ensure_windshield_rain_helper() -> WindshieldRain:
  """Start the ROAD drain thread. Called from stock_cc.update / install."""
  global _detector
  if _detector is None:
    _detector = WindshieldRain()
  _detector.start_helper()
  return _detector


def windshield_rain_needed() -> bool:
  det = ensure_windshield_rain_helper()
  return det.poll()


def reset_windshield_rain() -> None:
  """Tests reset the singleton."""
  global _detector
  if _detector is not None:
    _detector.stop_helper()
  _detector = None
