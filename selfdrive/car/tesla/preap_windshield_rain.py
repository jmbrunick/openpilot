"""Windshield rain / ice check from the comma 3X road camera.

There is no rain sensor, and this driving model does not publish rainProb.
Auto looks at the unwarped ROAD camera Y plane (camerad VisionIpc
VISION_STREAM_ROAD). The 3X looks through the windshield, focused on the
far scene, so drops on the glass are large soft defocused bokeh — not
sharp beads. The model warp crops that near field away, so this does not
use modelV2.

Rain: large low-frequency bokeh in the upper/mid ROAD bands, sparse
    streaks/speckles, or a wet sheet (high blob / overlapping milky
    defocus). Ice/frost: a milky sheet or crystal mottle.
Score is monotonic with obstruction: once the glass is past a wetness
floor, more water (blob, bokeh, haze, streaks, dense beads) must score
at least as rainy — never drop to dry because it is "too wet". Do not
require a bokeh/blob ratio sweet spot. Dense beads covering the pane
raise sharp-residual "structure"; that is still rain if bokeh is
rain-like. Foliage/brick is sharp structure *without* rain-scale bokeh.
Headlamp plates are isolated saturated hotspots, not distributed
droplet highlights. Dry overcast / scene texture stays under the
wetness floor. A real wipe may look clear for one idle score tick (~4 s).
Duty cycle (Justin): idle watching assesses every SCORE_PERIOD_S (~4 s)
and does not wipe first on Auto select / helper start. The first
WARMUP_N helper looks are ignored (VisionIpc / first ROAD). First wipe
needs two consecutive *meaningful* wet idle looks (ACQUIRE_ON) after
warmup — light sprinkle / residual beads stay idle (watch every ~4 s).
Dry indoor/garage ROAD and light sprinkle look the same to the old
wet-sheet path (blob≈7–9, live dry score=6.29). That must score well
below acquire. Light mist / film on the glass (highway streaks; ROAD
UI looks through it) must still wipe via the near-glass haze path —
not distant atmospheric fog alone. Then: one blade sweep →
wipe=0 + Auto rest-cancel → wait CLEAR_WAIT_S → assess. Re-wipe only if
still *clearly heavy* (REWIPE_ON, above light-mist film). Light mist
returns to idle 4 s and must full-reacquire. Off still leaves the real stalk.

Bokeh energy is an 8-bit residual (~0 dry, ~2–5 wet). A live clear-glass
log showed bokeh=49165 — wrong Y scale or a bandpass blowup. Impossible
magnitudes are invalid/dry (they must not latch HOLD or look like rain
returning after a wipe). Real heavy-rain scores above ~12 are still wet.

VisionIpc runs only while NAPWiperSpeed is Auto. Off/Int/On stop the
helper, drop HOLD, and do not recv ROAD — Int/On keep working without
the camera. The helper is SCHED_OTHER (nice 10): blocking recv, conflate
ROAD then WIDE. Numpy scoring while idle runs once per SCORE_PERIOD_S
(~4 s, SCORE_HZ ≈ 0.25), not every ROAD frame and not 4 Hz / 1 Hz. A
full-res Y copy plus 20 Hz multi-blur starved card's GIL (age_ms ~30 s,
Selfdrive Process Lagging). Recv downsamples immediately; one feature
pass per scored frame; ice contrast uses the tiny grid. Keep the ROAD client while Auto so we do not
resubscribe every 4 s (Justin: frames stuck at 1–2, age_ms thousands,
stale first Y after reconnect). Recv+numpy still once per period, not
20 Hz. One wipe is WIPE_PULSE_S (~1.5 s of Auto INTERVAL1 / collar=1, one
slow park-to-park), then CLEAR_WAIT_S with HOLD off (cancel + blades out of
FOV), then the helper assesses — it does not wait another idle 4 s.
2 s settle left blades in ROAD FOV on bone-dry Auto (wipe loop). Do not
prime hold_n after a wipe: a marginal post-wipe score must not INTERVAL1
again. (Old LIGHT_REST_N=6 left wet glass ~15 s. BURST_MAX_N=3 sat on
Low for ~7.5 s / several sweeps. Idle used to score every 2.5 s.)
card is CTRL_HIGH: stock_cc.update / poll() only reads the latch, never
recvs, never joins the helper. NAPWiperRainStatus and cloudlog are 1 Hz
or on gate changes — not every 10 ms Params.put. Numpy is imported on a
background thread, not on card's RT path. Helper start waits
RAIN_HELPER_START_DELAY_S after Auto is selected so engage is not
fighting ROAD subscribe. Failures retry; they do not permanently dry Auto.
"""
from __future__ import annotations

import os
import threading
import time

import numpy as np

# Unwarped ROAD. Distant road/sky in the upper-mid view is what made dry
# indoor/garage blob≈7.5 look like a wet sheet (Justin 5f32c450b). Score
# near-bottom glass, US LHD driver side (left). Skip the last rows so
# parked blades / hood edge stay out. Ice haze uses the same near glass.
_NEAR_ROWS = (0.58, 0.90)
_BOKEH_ROWS = (0.48, 0.82)
_MID_ROWS = (0.52, 0.84)
_COLS = (0.06, 0.50)
_RAIN_BANDS = (_NEAR_ROWS, _BOKEH_ROWS)
# Farther driving view (not near-glass). Atmospheric fog collapses this;
# mist-on-glass leaves it relatively sharp (Justin's 59 mph ROAD UI).
_FAR_ROWS = (0.18, 0.42)
_FAR_COLS = (0.22, 0.78)

# Rain. Detrended bokeh ~4 on wet ROAD, ~0 on dry sky gradients.
# Speckle-drop path stays ~2–4.
SCORE_ON = 1.8
SCORE_OFF = 1.0
# Justin's heavier-rain ROAD UI: mid-band detrended bokeh ~1.85–3.3.
# Dry sky/road wash is ~0.2. Live clear/overcast texture was latching at
# 1.5 — raise the bar so that is not rain. Real wet is still ~3+.
# Light sprinkle 5f32c450b: bokeh=3.01 blob=7.40 — not a wet sheet.
# At-or-above: bokeh >= this is rain. Do not also require a ratio vs blob.
BOKEH_ON = 2.2
# Old sweet-spot veto: bokeh/(blob+0.2) >= 0.22 dropped heavy rain to 0
# when blob rose. Kept so tests prove we no longer use it as a reject.
_BOKEH_RATIO = 0.22
# Dry/overcast fine residual on *smooth* fixtures is ~2. Live dry garage
# and light sprinkle both sat blob≈7.0–8.8. Overlapping milky wet is ~12.
# Sheet bar must sit above that dry/sprinkle blob, not at 8.
BLOB_WET = 10.5
# Live 5f32c450b dry + sprinkle (same scorer look):
#   dry last:     blob=7.55 sparse=6.7  struct=0.022 bokeh=3.85 score=6.29
#   sprinkle:     blob=7.40–8.83 sparse=4.7–9.7
# Heavy wet sheet is blob~12 sparse~2. Residual beads can hit sparse~8–10
# with the same mid blob — still not a sheet.
_SCENE_BLOB_MIN = 3.5
_SCENE_SPARSE = (2.5, 12.0)
# 8-bit ROAD: wet bokeh ~2–5. Justin's clear-glass log was bokeh=49165.
# Anything this high is a unit/scale bug, not rain.
BOKEH_ABSURD = 24.0
BLOB_ABSURD = 40.0
# Typical wet rain score is ~2–12. Heavy blob can exceed this and is still wet.
SCORE_ABSURD = 12.0
# 8-bit obstruction never reaches this. 49165 is garbage → dry, not a cap.
SCORE_INVALID = 80.0
_SPECKLE_MIN = 0.012
# Frost crystals: moderate uniform residual. Calm near-glass (blob~1.2)
# is not frost. Live dry blob=7.4 must not be frost either.
FROST_ON = 1.8
FROST_BLOB_MAX = 4.0
ICE_ON = 1.5
ICE_CONTRAST = 0.085
ICE_LUM = (45.0, 210.0)
ICE_BLOB_MIN = 1.8
# Light mist / film on the glass: low near-glass contrast + mild bokeh or
# streaks, while the distant ROAD view stays relatively sharp. Not dry
# calm glass (bokeh~0.1), not garage grain, not distant fog alone.
MIST_ON = 1.0
MIST_BOKEH_MIN = 0.70
MIST_SPECKLE_MIN = 0.010
MIST_NEAR_C = 0.085
MIST_FAR_C = 0.055
MIST_NEAR_OVER_FAR = 0.90
MIST_LUM = (40.0, 220.0)
# Live 2db9e6c30 highway mist that should wipe (lower-left FOV):
#   score=2.15 bokeh=1.42 blob=3.93 sparse=3.2 speckle=0 struct=0
# 48f458dd9 still missed dense fine drizzle (uniform film, ROAD looks
# through it). Status blob/bokeh are the *max-bokeh* band; scoring only
# NEAR dropped the film. Band sits below old-FOV garage (blob~7–9).
# Blob floor is above dry/overcast mid-left (~2.5) and calm glass (~1.2).
MIST_FILM_BLOB = (2.75, 6.2)
MIST_FILM_BOKEH = (0.55, 2.40)
MIST_FILM_SPARSE = (1.8, 5.25)
MIST_FILM_SPECKLE = 0.008
# Combined obstruction: 1.0 is looks_rainy / wetness floor (rain/frost/ice).
HOLD_ON = 1.0
# First wipe: two consecutive idle scores at/above this *after* warmup.
# Mist film live/offline sits ~6–8. Do not "fix" dry garage by raising
# this past old 6.29. Heavy milky / dense beads (~10+) still enter.
ACQUIRE_ON = 4.5
# Post-wipe re-enter and status "heavy": above light-mist film (~6–8.4).
# Justin 48f458dd9: first mist wipe worked, then wipe→3s→rewipe over-fired.
# Light mist exits to idle 4 s (full re-acquire). Heavy can rewipe after settle.
HEAVY_ON = 9.0
REWIPE_ON = HEAVY_ON
HOLD_OFF = 0.70
EMA_ALPHA = 0.35
EMA_HOLD_ALPHA = 0.35
CLEAR_RELEASE_N = 2
MIN_HOLD_N = 2
WIPE_CLEAR_N = 1
# First looks after helper start (VisionIpc / first ROAD) do not acquire.
WARMUP_N = 2
# One slow park-to-park sweep. Camera Auto holds INTERVAL1 (physical Int1),
# not TIPWIPE nibble 1 (BCM Wiper Low / ~32 s latch). Pulse then cancel —
# do not sit on continuous Int. poll() ends the pulse on wall-clock so
# we do not wait for the next idle ROAD score.
WIPE_PULSE_S = 1.5
# After wipe=0 + rest-cancel: ignore ROAD until blades park and streaks
# settle, then assess. 2.0 s still caught blades on bone-dry Auto.
CLEAR_WAIT_S = 3.0
# Compat aliases (old burst/rest names). Duty cycle uses WIPE_PULSE_S / CLEAR_WAIT_S.
BURST_MAX_N = 1
BURST_MAX_S = WIPE_PULSE_S
LIGHT_REST_N = 0
BLADE_BLIND_N = 0
# Must exceed SCORE_PERIOD_S so poll does not drop HOLD between ticks.
STALE_S = 10.0
CONNECT_RETRY_S = 0.5
DEBUG_LOG_S = 1.0
# Live helper while idle: score windshield clarity once every ~4 s, not
# every ROAD frame / not 4 Hz / 1 Hz. Post-wipe assess uses CLEAR_WAIT_S.
SCORE_PERIOD_S = 4.0
SCORE_HZ = 1.0 / SCORE_PERIOD_S  # ≈ 0.25 Hz
HELPER_RECV_MS = 50
HELPER_NICE = 10
# Copy at most this many pixels on the short side from VisionIpc (not full ROAD).
Y_COPY_SIDE = 48
_FEATURE_SIDE = 24
_SAT_MAX = 0.08
# Isolated headlamp plates are sparse hotspots. Dense bead highlights are not.
_LAMP_SPARSE = 200.0
_SPARSE_MIN = 8.0
# Dense bead rain on glass is ~6.5–8.5. Justin's upper droplet crop was
# rain-rejected at 6.85. Foliage is still ~1.8.
_SPARSE_RAIN_MIN = 6.0
_SPARSE_MAX = 80.0
STREAM_FALLBACK_S = 3.0
_STRUCTURE_FRAC = 0.05
# In-focus foliage/brick: sharp residual without rain-scale bokeh.
# Dense beads covering the glass also raise this — do not fail closed.
_STRUCTURE_RAIN = 0.12
_FOLIAGE_BLOB = 18.0
_FINE_R = 2
_MED_R = 4
_COARSE_R = 8


def y_plane_from_nv12(buf, max_side: int = 0) -> np.ndarray | None:
  """Y plane of an NV12 VisionBuf, cropped to width x height (no padding).

  Live helper passes max_side so we never copy a full 3X ROAD frame.
  """
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
    y = raw[:stride * height].reshape(height, stride)[:, :width]
    if max_side > 0 and min(height, width) > max_side:
      step = max(1, min(height, width) // int(max_side))
      y = y[::step, ::step]
    return np.ascontiguousarray(y)
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
  if not np.isfinite(s) or s < 0.0 or s > SCORE_INVALID:
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
  """Separable box mean via cumsum. O(HW), no per-tap Python loop."""
  x = np.asarray(x, dtype=np.float32)
  if radius < 1:
    return x
  k = 2 * radius + 1
  kf = float(k)

  def _mean1d(a: np.ndarray, axis: int) -> np.ndarray:
    pad_width = [(0, 0), (0, 0)]
    pad_width[axis] = (radius, radius)
    p = np.pad(a, pad_width, mode="edge")
    c = np.cumsum(p, axis=axis)
    zshape = list(c.shape)
    zshape[axis] = 1
    c = np.concatenate((np.zeros(zshape, dtype=c.dtype), c), axis=axis)
    sl_hi = [slice(None), slice(None)]
    sl_lo = [slice(None), slice(None)]
    sl_hi[axis] = slice(k, None)
    sl_lo[axis] = slice(None, -k)
    return (c[tuple(sl_hi)] - c[tuple(sl_lo)]) / kf

  return _mean1d(_mean1d(x, 1), 0)


def _near_features(img: np.ndarray) -> tuple[float, float, float, float, float, float]:
  """Fine blob, speckle, sparsity, sat, strong-edge frac, coarse bokeh energy."""
  z = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  y8 = _to_y8(img)
  if y8 is None or y8.size < 25:
    return z
  step = max(1, min(y8.shape) // _FEATURE_SIDE)
  x = y8[::step, ::step].astype(np.float32, copy=False)
  if x.shape[0] < 5 or x.shape[1] < 5:
    return z
  fine = _box_blur(x, _FINE_R)
  med = _box_blur(x, _MED_R)
  coarse = _box_blur(x, _COARSE_R)
  resid = np.abs(x - fine)
  blob = float(resid.mean())
  speckle = float(np.mean((x > fine + 18.0) & (resid > 18.0)))
  flat = resid.reshape(-1)
  p50 = float(np.median(flat))
  k90 = max(0, min(flat.size - 1, int(0.90 * (flat.size - 1))))
  p90 = float(np.partition(flat, k90)[k90])
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


def _band_stats(y: np.ndarray, rows: tuple[float, float], cols: tuple[float, float]) -> tuple[float, float]:
  y8 = _to_y8(y)
  if y8 is None:
    return 0.0, 0.0
  mid = _band(y8, rows, cols)
  step = max(1, min(mid.shape) // _FEATURE_SIDE)
  x = mid[::step, ::step].astype(np.float32, copy=False)
  mean = float(x.mean())
  contrast = float(x.std() / (mean + 1.0))
  return mean, contrast


def _mid_stats(y: np.ndarray) -> tuple[float, float]:
  return _band_stats(y, _MID_ROWS, _COLS)


def _scene_texture(blob: float, sparse: float, structure: float) -> bool:
  """Dry indoor/garage + driveway grain. Not a milky wet sheet.

  Justin 5f32c450b: bone-dry garage blob=7.55 sparse=6.7 score=6.29.
  Same session light sprinkle was blob=7.4–8.83 sparse=4.7–9.7. Those
  are the same look. Heavy overlapping rain is blob~12 sparse~2.
  """
  if structure > _STRUCTURE_RAIN:
    return False
  if blob < _SCENE_BLOB_MIN or blob >= BLOB_WET:
    return False
  return _SCENE_SPARSE[0] <= sparse <= _SCENE_SPARSE[1]


def _rain_from_feats(blob: float, speckle: float, sparse: float, sat: float,
                    structure: float, bokeh: float) -> float:
  if sat > _SAT_MAX and sparse >= _LAMP_SPARSE:
    return 0.0
  if structure > _STRUCTURE_RAIN and bokeh < BOKEH_ON:
    return 0.0
  if _scene_texture(blob, sparse, structure):
    return 0.0
  score = 0.0
  if BOKEH_ON <= bokeh <= BOKEH_ABSURD:
    score = max(score, bokeh)
  if blob >= BLOB_WET:
    score = max(score, blob)
  if speckle >= _SPECKLE_MIN:
    drop = 12.0 * speckle
    # Only a real wet sheet adds blob. Dry garage / residual beads sit
    # blob≈7–9 with sparse~5–10 — that is not a milky sheet.
    if blob >= BLOB_WET:
      drop += blob
    score = max(score, drop)
  return score


def _frost_from_feats(blob: float, _speckle: float, sparse: float, sat: float,
                     structure: float, _bokeh: float) -> float:
  if sat > _SAT_MAX or structure > _STRUCTURE_FRAC:
    return 0.0
  if blob > FROST_BLOB_MAX:
    return 0.0
  if sparse >= _SPARSE_MIN:
    return 0.0
  return blob


def _ice_from_feats(blob: float, _speckle: float, _sparse: float, sat: float,
                   structure: float, _bokeh: float, y: np.ndarray) -> float:
  if sat > _SAT_MAX or structure > _STRUCTURE_FRAC or blob > FROST_BLOB_MAX:
    return 0.0
  mean, contrast = _mid_stats(y)
  if not (ICE_LUM[0] < mean < ICE_LUM[1]):
    return 0.0
  haze = ICE_CONTRAST - contrast
  if haze <= 0.0 or blob < ICE_BLOB_MIN:
    return 0.0
  return 15.0 * haze + blob


def _mist_film_from_feats(blob: float, speckle: float, sparse: float, sat: float,
                         structure: float, bokeh: float) -> float:
  """Soft / uniform fine-droplet film. No Y / contrast needed.

  Live highway mist (2db9e6c30): blob=3.93 bokeh=1.42 sparse=3.2 → 2.15
  and holdn=0. Dense drizzle can pile blob toward 8 with *low* bokeh.
  Old-FOV dry garage blob~7.5 / bokeh~3.8 / sparse~6.7 must stay 0.
  Dry overcast mid-left is blob~2.5 — below the blob floor.
  """
  if sat > _SAT_MAX or structure > _STRUCTURE_FRAC:
    return 0.0
  if blob < MIST_FILM_BLOB[0]:
    return 0.0
  if sparse < MIST_FILM_SPARSE[0] or sparse > MIST_FILM_SPARSE[1]:
    return 0.0
  # Garage/old-FOV wet-sheet grain: high blob *and* high bokeh/sparse.
  if blob > MIST_FILM_BLOB[1] and (bokeh >= BOKEH_ON or sparse >= 5.5):
    return 0.0
  if blob >= BLOB_ABSURD:
    return 0.0
  has_bokeh = MIST_FILM_BOKEH[0] <= bokeh <= MIST_FILM_BOKEH[1]
  has_speckle = speckle >= MIST_FILM_SPECKLE and blob >= 3.2
  has_film = blob >= 3.2 and bokeh < BOKEH_ON
  if not (has_bokeh or has_speckle or has_film):
    return 0.0
  return 5.3 + 0.25 * min(blob, 8.0) + 0.4 * max(0.0, bokeh - 0.4) + 8.0 * speckle


def _mist_from_feats(blob: float, speckle: float, sparse: float, sat: float,
                    structure: float, bokeh: float, y: np.ndarray) -> float:
  """Near-glass mist film. Distant fog alone must not score.

  Justin 2db9e6c30 highway ~59 mph: small drops/streaks should wipe, but
  ROAD looks through the mist (far scene still sharp). Dry garage grain
  and calm dry glass must stay off.
  """
  film = _mist_film_from_feats(blob, speckle, sparse, sat, structure, bokeh)
  if sat > _SAT_MAX or structure > _STRUCTURE_RAIN:
    return film
  if bokeh < MIST_BOKEH_MIN and speckle < MIST_SPECKLE_MIN:
    return film
  near_mean, near_c = _band_stats(y, _NEAR_ROWS, _COLS)
  _far_mean, far_c = _band_stats(y, _FAR_ROWS, _FAR_COLS)
  if not (MIST_LUM[0] < near_mean < MIST_LUM[1]):
    return film
  if near_c > MIST_NEAR_C or far_c < MIST_FAR_C:
    return film
  if far_c <= 0.0 or near_c > far_c * MIST_NEAR_OVER_FAR:
    return film
  haze = MIST_NEAR_C - near_c
  veil = 4.6 + 15.0 * haze + 0.8 * max(0.0, bokeh - MIST_BOKEH_MIN) + 20.0 * speckle
  return max(film, veil)


def _rain_from_band(img: np.ndarray) -> float:
  """Soft far-focus bokeh, wet-sheet blob, and/or speckle beads.

  At-or-above: each cue is a floor, not a band. Extra blob/bokeh/speckle
  raises the score. Dense beads covering the pane look "structured";
  that is still rain when bokeh is rain-like. Foliage is sharp texture
  without that bokeh. Isolated saturated lamps are not rain.
  """
  return _rain_from_feats(*_near_features(img))


def windshield_rain_score(y: np.ndarray) -> float:
  """Higher = rainier glass (far-focus bokeh, streaks, speckles).

  3X ROAD is focused on the scene, so windshield drops are large soft
  circles of confusion. Heavier water must not score drier. Dense
  in-focus texture (foliage, brick) and large saturated headlamp plates
  are not treated as rain.
  """
  if y is None or y.ndim != 2 or y.shape[0] < 32 or y.shape[1] < 32:
    return 0.0
  return max(_rain_from_band(_band(y, rows, _COLS)) for rows in _RAIN_BANDS)


def windshield_frost_score(y: np.ndarray) -> float:
  """Crystals on the glass: moderate uniform residual, not foliage, not rain drops."""
  if y is None or y.ndim != 2 or y.shape[0] < 32 or y.shape[1] < 32:
    return 0.0
  return _frost_from_feats(*_near_features(_band(y, _NEAR_ROWS, _COLS)))


def windshield_ice_score(y: np.ndarray) -> float:
  """Ice sheet: daytime view through the glass is milky (contrast collapsed)."""
  if y is None or y.ndim != 2 or y.shape[1] < 32 or y.shape[0] < 32:
    return 0.0
  feats = _near_features(_band(y, _NEAR_ROWS, _COLS))
  return _ice_from_feats(*feats, y)


def windshield_mist_score(y: np.ndarray) -> float:
  """Light mist / film on near-glass. Distant atmospheric fog scores 0."""
  if y is None or y.ndim != 2 or y.shape[0] < 32 or y.shape[1] < 32:
    return 0.0
  mist = 0.0
  near = None
  best = None
  for rows in _RAIN_BANDS:
    feats = _near_features(_band(y, rows, _COLS))
    mist = max(mist, _mist_film_from_feats(*feats))
    if rows == _NEAR_ROWS:
      near = feats
    if best is None or feats[5] >= best[5]:
      best = feats
  if near is not None:
    mist = max(mist, _mist_from_feats(*near, y))
  if best is not None:
    mist = max(mist, _mist_film_from_feats(*best))
  return mist


def _score_frame(y: np.ndarray) -> tuple[float, tuple[float, float, float, float, float, float]]:
  """One pass: rain + mist film on both lower-left bands; veil/frost/ice on NEAR.

  Status blob/bokeh follow the max-bokeh band. Film must use those same
  bands — scoring only NEAR missed live film that showed up in status.
  Contrast veil stays on NEAR so dry overcast mid-left does not acquire.
  """
  z = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  rain = 0.0
  mist = 0.0
  best = z
  near = z
  for rows in _RAIN_BANDS:
    feats = _near_features(_band(y, rows, _COLS))
    if rows == _NEAR_ROWS:
      near = feats
    rain = max(rain, _rain_from_feats(*feats))
    mist = max(mist, _mist_film_from_feats(*feats))
    if feats[5] >= best[5]:
      best = feats
  frost = _frost_from_feats(*near)
  ice = _ice_from_feats(*near, y)
  mist = max(mist, _mist_from_feats(*near, y), _mist_film_from_feats(*best))
  obs = _finite_score(max(rain / SCORE_ON, frost / FROST_ON, ice / ICE_ON, mist / MIST_ON))
  return obs, best


def windshield_obstruction_score(y: np.ndarray) -> float:
  """1.0 is the hold line. Rain, frost crystals, or an ice sheet."""
  if y is None or y.ndim != 2 or y.shape[0] < 32 or y.shape[1] < 32:
    return 0.0
  obs, _best = _score_frame(y)
  return obs


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
    os.nice(HELPER_NICE)
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
    self._wipe_t0 = 0.0
    self._wait_t0 = 0.0
    self._post_wipe = False
    self._warm_n = 0
    self._owns_client = False
    self._need_flush = False

  def _store_feats(self, feats: tuple[float, float, float, float, float, float]) -> None:
    blob, speckle, sparse, sat, structure, bokeh = feats
    self.last_blob = blob
    self.last_speckle = speckle
    self.last_sparse = sparse
    self.last_sat = sat
    self.last_structure = structure
    self.last_bokeh = bokeh

  def update_from_y(self, y: np.ndarray) -> bool:
    obs, feats = _score_frame(y)
    self._store_feats(feats)
    return self._update_score(obs)

  def _start_wipe(self, now: float) -> None:
    self.hold = True
    self._wipe_t0 = now
    self._wait_t0 = 0.0
    self._post_wipe = False
    self._hold_n = MIN_HOLD_N
    self._clear_n = 0

  def _finish_wipe(self) -> None:
    """End the one-sweep pulse. Auto rest-cancels; wait CLEAR_WAIT_S then assess."""
    self.hold = False
    self._clear_n = 0
    self._wipe_t0 = 0.0
    self._wait_t0 = time.monotonic()
    # Do not prime hold_n. One dry/marginal post-wipe look must exit to idle.
    self._hold_n = 0
    self._post_wipe = True

  def _update_score(self, score: float) -> bool:
    """Latch from an obstruction score. Tests inject residual/dry scores here."""
    with self._lock:
      now = time.monotonic()
      self.last_score = _finite_score(score)
      alpha = EMA_HOLD_ALPHA if self.hold else EMA_ALPHA
      self.ema = alpha * self.last_score + (1.0 - alpha) * self.ema
      waiting = self._wait_t0 > 0.0 and (now - self._wait_t0) < CLEAR_WAIT_S
      if self.hold:
        if self._wipe_t0 > 0.0 and (now - self._wipe_t0) >= WIPE_PULSE_S:
          self._finish_wipe()
      elif waiting:
        # Blade FOV / glass settling. Do not acquire on this score.
        pass
      else:
        self._wait_t0 = 0.0
        self._clear_n = 0
        if self._post_wipe:
          self._post_wipe = False
          if self.last_score >= REWIPE_ON:
            self._start_wipe(now)
          else:
            # Dry / light mist / marginal → idle 4 s. Full re-acquire.
            self._hold_n = 0
        elif self._warm_n < WARMUP_N:
          # First ROAD looks after helper start. Do not acquire.
          self._warm_n += 1
          self._hold_n = 0
        elif self.last_score >= ACQUIRE_ON:
          self._hold_n += 1
          if self._hold_n >= MIN_HOLD_N:
            self._start_wipe(now)
        else:
          # Consecutive idle wet looks only. One below-wet resets.
          self._hold_n = 0
      self._last_frame_t = now
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
    self._owns_client = False
    self._need_flush = False
    self.connected = False
    self._stream_t0 = now
    self.last_err = f"fallback_{self.stream}"

  def _recv_y(self, timeout_ms: int = 0, max_side: int = 0) -> np.ndarray | None:
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
        self._owns_client = True
        self._need_flush = True
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
      y = y_plane_from_nv12(buf, max_side=max_side)
      buf = None
      if y is None:
        self.last_err = "noyplane"
        return None
      self.last_err = ""
      if self._need_flush:
        # First buffer after subscribe is often stale/wrong (blob≈7 dry).
        self._need_flush = False
        flushed = self._client.recv(timeout_ms=int(timeout_ms))
        if flushed is not None:
          y2 = y_plane_from_nv12(flushed, max_side=max_side)
          flushed = None
          if y2 is not None:
            y = y2
      return y
    except Exception as e:
      self._failed = True
      self.connected = False
      self._client = None
      self._owns_client = False
      self._need_flush = False
      self._last_connect_t = now
      self.last_err = type(e).__name__
      return None

  def _debug(self, reason: str = "") -> None:
    """cloudlog at DEBUG_LOG_S. Do not write NAPWiperRainStatus from this thread."""
    now = time.monotonic()
    if now - self._last_log_t < DEBUG_LOG_S:
      return
    self._last_log_t = now
    wait_left = 0.0
    if self._wait_t0 > 0.0:
      wait_left = max(0.0, CLEAR_WAIT_S - (now - self._wait_t0))
    pulse_left = 0.0
    if self.hold and self._wipe_t0 > 0.0:
      pulse_left = max(0.0, WIPE_PULSE_S - (now - self._wipe_t0))
    age_ms = (now - self._last_frame_t) * 1000.0 if self._last_frame_t else -1.0
    err = f" err={self.last_err}" if self.last_err else ""
    why = f" {reason}" if reason else ""
    line = (
      f"nap wiper rain hold={int(self.hold)} ema={self.ema:.2f} score={self.last_score:.2f} "
      + f"bokeh={self.last_bokeh:.2f} blob={self.last_blob:.2f} speckle={self.last_speckle:.3f} "
      + f"sparse={self.last_sparse:.1f} sat={self.last_sat:.3f} struct={self.last_structure:.3f} "
      + f"holdn={int(self._hold_n)}/{int(MIN_HOLD_N)} warm={int(self._warm_n)}/{int(WARMUP_N)} "
      + f"clear={int(self._clear_n)}/{int(CLEAR_RELEASE_N)} "
      + f"heavy={int(self.last_score >= HEAVY_ON)} pulse={pulse_left:.1f}/{WIPE_PULSE_S:.1f} "
      + f"wait={wait_left:.1f}/{CLEAR_WAIT_S:.1f} connected={int(self.connected)} "
      + f"failed={int(self._failed)} frames={self.n_frames} stream={self.stream} "
      + f"helper={int(self.helper_alive)} period_s={SCORE_PERIOD_S:.1f} hz={SCORE_HZ:.1f} age_ms={age_ms:.0f}{err}{why}"
    )
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.info("%s", line)
    except Exception:
      pass

  @property
  def helper_alive(self) -> bool:
    t = self._helper
    return bool(t is not None and t.is_alive())

  def _clear_hold(self) -> None:
    with self._lock:
      self.hold = False
      self.ema = 0.0
      self.last_score = 0.0
      self._clear_n = 0
      self._hold_n = 0
      self._wipe_t0 = 0.0
      self._wait_t0 = 0.0
      self._post_wipe = False
      self._warm_n = 0

  def _release_vision(self) -> None:
    """Drop the live ROAD client (helper stop / thread exit). Tests inject _client."""
    if not self._owns_client:
      return
    self._client = None
    self._owns_client = False
    self._need_flush = False
    self.connected = False

  def start_helper(self) -> None:
    """Drain ROAD off the card thread. Idempotent. Never join (CTRL_HIGH)."""
    t = self._helper
    if t is not None and t.is_alive():
      # Still winding down after stop — next stock_cc.update retries.
      if not self._stop.is_set():
        self._helper_started = True
      return
    self._stop = threading.Event()
    self._helper_started = True
    self._warm_n = 0
    try:
      self._helper = threading.Thread(target=self._helper_loop, name="nap-wiper-rain", daemon=True)
      self._helper.start()
    except Exception as e:
      self._helper = None
      self._helper_started = False
      self.last_err = type(e).__name__

  def stop_helper(self, join: bool = True) -> None:
    """Stop ROAD drain and drop HOLD. join=False from card (do not block RT)."""
    t = self._helper
    if not self._helper_started and not self.hold and (t is None or not t.is_alive()):
      if join:
        self._helper = None
      return
    self._stop.set()
    self._helper_started = False
    self._clear_hold()
    t = self._helper
    if join and t is not None and t.is_alive() and t is not threading.current_thread():
      t.join(timeout=0.6)
    if join:
      self._helper = None
      self._release_vision()

  def _next_score_at(self, last_score_t: float, now: float) -> float:
    """When the helper should next look at ROAD.

    Idle watching: SCORE_PERIOD_S (~4 s) after the last look. First look
    is immediate (assess, do not wipe-first). After a wipe, assess when
    CLEAR_WAIT_S elapses — not another idle 4 s.
    """
    with self._lock:
      if self.hold and self._wipe_t0 > 0.0:
        return self._wipe_t0 + WIPE_PULSE_S + CLEAR_WAIT_S
      if self._wait_t0 > 0.0:
        return self._wait_t0 + CLEAR_WAIT_S
    if last_score_t <= 0.0:
      return now
    return last_score_t + float(SCORE_PERIOD_S)

  def _helper_loop(self) -> None:
    """Recv+score on the idle/wipe-loop cadence. Keep ROAD subscribed while Auto."""
    _drop_realtime()
    last_score_t = 0.0
    try:
      while not self._stop.is_set():
        now = time.monotonic()
        wait = self._next_score_at(last_score_t, now) - now
        if wait > 0.0:
          # Short chunks so poll() ending a pulse can switch to CLEAR_WAIT_S.
          self._stop.wait(min(wait, 0.25))
          continue
        try:
          y = self._recv_y(timeout_ms=HELPER_RECV_MS, max_side=Y_COPY_SIDE)
        except Exception as e:
          self._failed = True
          self.last_err = type(e).__name__
          y = None
        if self._stop.is_set():
          break
        if y is not None:
          self.update_from_y(y)
          last_score_t = time.monotonic()
          y = None
        else:
          self._apply_stale()
          if not self.connected or self._failed:
            self._stop.wait(CONNECT_RETRY_S)
    finally:
      self._release_vision()

  def _apply_stale(self) -> None:
    """Clear HOLD if ROAD frames stop. Do not wipe dry glass."""
    if self.hold and self._last_frame_t > 0:
      if time.monotonic() - self._last_frame_t > STALE_S:
        self._clear_hold()
        self._debug("stale")
    elif not self.hold:
      self._debug("noframe")

  def _apply_wipe_pulse(self) -> None:
    """End the one-sweep nibble-1 pulse on wall-clock. Latch only (no numpy/recv)."""
    with self._lock:
      if not self.hold or self._wipe_t0 <= 0.0:
        return
      if time.monotonic() - self._wipe_t0 < WIPE_PULSE_S:
        return
      self._finish_wipe()

  def poll(self) -> bool:
    """Latch only. Never recv, numpy, Params, or stale-debug on the card RT thread."""
    if not self._helper_started:
      return False
    if self.hold:
      self._apply_stale()
      self._apply_wipe_pulse()
    return self.hold


_detector: WindshieldRain | None = None


def ensure_windshield_rain_helper() -> WindshieldRain:
  """Start the ROAD drain thread. Called from stock_cc.update while Auto."""
  global _detector
  if _detector is None:
    _detector = WindshieldRain()
  _detector.start_helper()
  return _detector


def stop_windshield_rain_helper() -> None:
  """Off/Int/On: no ROAD recv. Does not join (card is CTRL_HIGH)."""
  if _detector is not None:
    _detector.stop_helper(join=False)


def windshield_rain_needed() -> bool:
  """Latch only. stock_cc starts the helper while Auto; do not start from poll."""
  det = _detector
  if det is None:
    return False
  return det.poll()


def reset_windshield_rain() -> None:
  """Tests reset the singleton."""
  global _detector
  if _detector is not None:
    _detector.stop_helper()
  _detector = None
