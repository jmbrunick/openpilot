"""Curve-follow: a continuous longitudinal term from the model path ahead (Pre-AP).

A human does not pick a corner speed and hold it. He eases off a little as a
bend comes into view, keeps adjusting while it holds or tightens, and comes
back on the gas as the road opens. This module is that, in the same style as
the lead-follow law: every cycle the road ahead is a speed envelope
v_env(s) (curve follow's own lateral-accel table, set a little under how
Justin corners; see CF_LAT_TARGET_MS2), and the most
limiting point is a virtual lead that is stationary in speed. The kinematic
required accel to meet it is

  a_i = (v_env_i² - v²) / (2 · room_i)

negative when the car is above the envelope, a small positive allowance when
it is under (so it neither speeds into a bend nor brakes for one it already
meets). The result is a CEILING on acceleration, like the map and speed-cap
terms: it can only lower the command and it is exactly PREVIEW_FREE_A_MS2
when nothing ahead binds. It never writes MAX, keeps no set speed and no
latch. Its only state is sub-second filters, a confidence weight and a slew.

Reliability (how much to believe the forecast, per point and overall):
- a bend must stand out of the path's own position std (signal to noise of
  its lateral displacement), so noisy far points count little and a clear
  bend counts fully, at any distance, without a fixed horizon;
- curvature is filtered per path index: fast to tighten (faster when the
  bend is strong), slower to open, and a single-point spike is clipped;
- a lane change, dropped frames, an invalid model or an active roundabout
  funnel fade the term out (it never steps); below 8 m/s only a turn with
  the blinker on is anticipated, and only out to ~3 s.

Release is proportional: as the path opens the envelope rises and so does
the ceiling; once the path shows the bend has peaked, the near field
(mild lift only, never a brake into the exit) fades to nothing.

Shadow: with NAPCurveFollow = 1 the planner computes and logs this every
cycle and applies nothing. 0 = off, 2 = active (replaces the old preview and
retires the CurveMaxHold MAX cap).
"""
from __future__ import annotations

import json
import math

from openpilot.selfdrive.controls.lib.curve_preview import (
  PREVIEW_FREE_A_MS2,
  PREVIEW_MIN_ROOM_M,
  PREVIEW_MIN_ROOM_S,
  PREVIEW_MIN_V_MS,
)
from openpilot.selfdrive.controls.lib.unified_lead import _smooth01, _softplus, speed_ceiling_accel

PARAM_KEY = "NAPCurveFollow"
MODE_OFF = 0
MODE_SHADOW = 1
MODE_ACTIVE = 2
DEFAULT_MODE = MODE_SHADOW

FREE_A_MS2 = PREVIEW_FREE_A_MS2
MIN_V_MS = PREVIEW_MIN_V_MS

# Curve follow's own cornering table (lateral accel at the tightest point vs
# speed). CurveMaxHold and the old preview keep curve_max_hold's table.
# Fitted (model-path curvature) to 53 curves Justin drove himself, Oct 4-10
# 2026: the median target is ~1.5 mph under his apex speed on curves where
# the bend set his speed (apex lateral accel >= 1.3), a little slower than
# him in town and on highway sweepers alike. His own cornering at one speed
# spans ~2x (1.3 to 3.3 m/s²), so no single table sits 1-5 mph under him
# on every curve; this one is the best balance found (see the PR). The old
# table had the opposite shape (2.4 in town, 1.75 at highway speed).
CF_LAT_TARGET_BP_MS = [8.0, 13.0, 18.0, 22.0, 27.0]
CF_LAT_TARGET_MS2 = [1.45, 1.55, 1.85, 2.05, 2.20]
CF_SPEED_FLOOR_MS = 6.0
CF_SPEED_CEIL_MS = 45.0  # bends gentler than this speed's table are not bends

# Decel the term may ask for, vs speed. Justin's own slowing into a bend
# averages ~0.5 m/s² (lift / regen, peaks ~1.3); the old preview's 0.35 at
# highway speed could not shed a sharp corner after a fast straight. When the
# kinematic demand is beyond CF_URGENT_A0_MS2 the limit grows toward
# CF_URGENT_DECEL_MS2 (still jerk limited: never a wall).
CF_DECEL_BP_MS = [10.0, 25.0]
CF_DECEL_MS2 = [1.2, 0.55]
CF_JERK_MS3 = [1.0, 0.4]
CF_URGENT_DECEL_MS2 = 1.5
CF_URGENT_JERK_MS3 = 0.6
CF_URGENT_A0_MS2 = 0.8
CF_URGENT_BAND_MS2 = 0.5

# Where the car finishes slowing for a bend, as seconds before its tightest
# point. 1.0 s: slowing is done about 1 s before the tightest point. Justin
# himself is still easing at the apex (median: slowing ends ~0.8 s after it,
# Oct 4-10), so 1.0 s early is the conservative side of him. The speed AT the
# tightest point is the table speed either way.
RELEASE_BEFORE_TIGHTEST_S = 1.0
ROOM_SOFT_M = 2.0

# Curvature filter per path index (|k|): tighten fast (faster when strong),
# open slowly so a noisy "it opens" cannot release early.
K_UP_TAU_S = 0.20
K_UP_STRONG_TAU_S = 0.12
K_STRONG = 0.02
K_DOWN_TAU_S = 0.50
SPIKE_MIN_K = 0.01
SPIKE_RATIO = 2.0

# Signal to noise of a bend's lateral displacement against the path's std.
SNR_S0 = 2.0
SNR_S1 = 6.0
STD_FLOOR_M = 0.2
STD_FALLBACK_BASE_M = 0.3   # no yStd in the message
STD_FALLBACK_PER_S_M = 0.15

# Low speed: only a blinkered turn, only ~3 s ahead.
LOW_V0_MS = 6.5
LOW_V1_MS = 8.0
LOW_HORIZON_S = 2.5
LOW_HORIZON_BAND_S = 0.5

# Overall confidence (lead-law style ramp).
CONF_RATE_BASE_1_S = 2.0
CONF_RATE_URGENT_1_S = 14.0
CONF_FALL_1_S = 1.5
CONF_FALL_GATE_1_S = 2.0
CONF_URGENT_LO_MS2 = 0.5
CONF_URGENT_BAND_MS2 = 1.5
FRAME_DROP_MAX_PCT = 20.0

# Near field / apex: lift only, and gone once the path says the bend peaked.
NEAR_BRAKE_MAX_MS2 = 0.35
NEAR_T0_S = 1.0
NEAR_T_BAND_S = 1.0
PEAK_LOOKAHEAD_S = 1.0
PEAK_RATIO0 = 0.9
PEAK_BAND = 0.3
PEAK_MIN_K = 0.004

# Relief: with room to spare the term allows speed up to the profile that can
# still meet the envelope speed at a mild decel, and does not brake until the
# car is above it. That is how a car accelerates out of one bend and eases
# back for the next. Relief fades to nothing as the point gets close.
RELIEF_DECEL_MS2 = 0.30
RELIEF_MAX_MS2 = 0.50
RELIEF_ROOM0_M = 20.0
RELIEF_ROOM_BAND_M = 40.0

# Output slew.
OUT_TAU_S = 0.20
BUILD_PROP_1_S = 2.0
BUILD_MAX_MS3 = 2.0
RELEASE_BP_MS = [10.0, 20.0]
RELEASE_BASE_MS3 = [1.0, 0.45]
RELEASE_OPEN_MS3 = 1.35
RELEASE_PROP_1_S = 2.0
RELEASE_MAX_MS3 = 2.5
SNAP_EPS_MS2 = 0.01

# After a bend that slowed the car has peaked: +a back gently, like Justin
# (back on the gas ~1.4 s after the apex at ~0.2, p75 0.5 m/s²). The ceiling
# holds POST_APEX_A_MS2 for POST_APEX_HOLD_S, then opens over POST_APEX_OPEN_S.
POST_APEX_A_MS2 = 0.50
POST_APEX_HOLD_S = 2.0
POST_APEX_OPEN_S = 1.0
POST_APEX_PASSED = 0.5
POST_APEX_BOUND_S = 3.0  # the term must have been slowing within this long


def read_curve_follow_mode(params) -> int:
  """NAPCurveFollow 0 off / 1 shadow / 2 active. Unknown or unreadable = shadow."""
  try:
    mode = int(params.get(PARAM_KEY, return_default=True))
  except Exception:
    return DEFAULT_MODE
  return mode if mode in (MODE_OFF, MODE_SHADOW, MODE_ACTIVE) else DEFAULT_MODE


def _interp(x: float, xp, fp) -> float:
  if x <= xp[0]:
    return float(fp[0])
  for i in range(1, len(xp)):
    if x <= xp[i]:
      f = (x - xp[i - 1]) / (xp[i] - xp[i - 1])
      return float(fp[i - 1] + f * (fp[i] - fp[i - 1]))
  return float(fp[-1])


def cf_lat_target_ms2(v_ms: float) -> float:
  return _interp(float(v_ms), CF_LAT_TARGET_BP_MS, CF_LAT_TARGET_MS2)


def curve_speed_for_curvature(kappa: float) -> float | None:
  """Curve follow's corner speed for |kappa|: the LOWEST v with v²·|κ| >= A(v).

  A(v) rises with speed here, so a fixed point is not safe; bisection on the
  first crossing is (and picks the conservative root). None on a straight.
  """
  k = abs(float(kappa))
  if not math.isfinite(k) or k <= 1e-6:
    return None
  lo, hi = 0.0, CF_SPEED_CEIL_MS
  if hi * hi * k < cf_lat_target_ms2(hi):
    return None  # gentler than any speed we drive: not a bend
  # first crossing: scan coarse, then bisect
  prev = lo
  v = 0.5
  while v <= hi:
    if v * v * k >= cf_lat_target_ms2(v):
      break
    prev = v
    v += 0.5
  lo, hi = prev, min(v, hi)
  for _ in range(20):
    mid = 0.5 * (lo + hi)
    if mid * mid * k >= cf_lat_target_ms2(mid):
      hi = mid
    else:
      lo = mid
  return max(CF_SPEED_FLOOR_MS, hi)


def decel_limit_ms2(v_ego: float, demand_ms2: float = 0.0) -> float:
  """Most decel the term may ask for: speed table, plus the urgent allowance."""
  base = _interp(float(v_ego), CF_DECEL_BP_MS, CF_DECEL_MS2)
  u = _smooth01((-float(demand_ms2) - CF_URGENT_A0_MS2) / CF_URGENT_BAND_MS2)
  return base + max(0.0, CF_URGENT_DECEL_MS2 - base) * u


def build_jerk_ms3(v_ego: float, urgent: float) -> float:
  base = _interp(float(v_ego), CF_DECEL_BP_MS, CF_JERK_MS3)
  return base + max(0.0, CF_URGENT_JERK_MS3 - base) * float(urgent)


def interp_at(ts, vals, t: float) -> float | None:
  """Linear interpolation of vals(ts) at t, None outside the path."""
  try:
    n = min(len(ts), len(vals))
  except TypeError:
    return None
  if n < 1 or t < float(ts[0]) or t > float(ts[n - 1]):
    return None
  for i in range(1, n):
    if float(ts[i]) >= t:
      a, b = float(ts[i - 1]), float(ts[i])
      f = 0.0 if b <= a else (t - a) / (b - a)
      return float(vals[i - 1]) + f * (float(vals[i]) - float(vals[i - 1]))
  return float(vals[0])


def curve_follow_raw(v_ego: float, ts, ds, ks, weights, passed: float = 0.0):
  """Unslewed ceiling from the (filtered) path ahead.

  Returns (a, limiting_index, limiting_envelope_speed, near_field_a). `a` is
  FREE_A_MS2 exactly when no point binds. `weights` in [0, 1] scale each
  point's demand toward "no demand"; `passed` in [0, 1] says the bend has
  peaked and removes the near-field demand.
  """
  v = float(v_ego)
  if v < MIN_V_MS or ts is None:
    return FREE_A_MS2, -1, None, None
  min_room = max(PREVIEW_MIN_ROOM_M, PREVIEW_MIN_ROOM_S * v)
  best = FREE_A_MS2
  best_i = -1
  best_v = None
  for i, (t, d, k, w) in enumerate(zip(ts, ds, ks, weights, strict=True)):
    if w <= 0.0 or t < 0.0:
      continue
    v_env = curve_speed_for_curvature(k)
    if v_env is None:
      continue
    room = min_room + _softplus(float(d) - RELEASE_BEFORE_TIGHTEST_S * v - min_room, ROOM_SOFT_M)
    a_kin = (v_env * v_env - v * v) / (2.0 * room)
    v_prof = math.sqrt(v_env * v_env + 2.0 * RELIEF_DECEL_MS2 * room)
    r0 = RELIEF_MAX_MS2 * _smooth01((room - RELIEF_ROOM0_M) / RELIEF_ROOM_BAND_M)
    relief = r0 * (1.0 - _smooth01((v - v_env) / max(v_prof - v_env, 0.5)))
    a_i = min(FREE_A_MS2, a_kin + relief)
    near = 1.0 - _smooth01((float(t) - NEAR_T0_S) / NEAR_T_BAND_S)
    scale = float(w) * (1.0 - float(passed) * near)
    a_eff = FREE_A_MS2 + scale * (a_i - FREE_A_MS2)
    if a_eff < best:
      best, best_i, best_v = a_eff, i, v_env
  a_near = None
  v0 = curve_speed_for_curvature(ks[0]) if len(ks) else None
  if v0 is not None and weights[0] > 0.0:
    over = speed_ceiling_accel(v, v0)
    if over < 0.0:
      a_near = FREE_A_MS2 + float(weights[0]) * (1.0 - float(passed)) * (max(over, -NEAR_BRAKE_MAX_MS2) - FREE_A_MS2)
      if a_near < best:
        best, best_i, best_v = a_near, 0, v0
  return best, best_i, best_v, a_near


class CurveFollow:
  """Filters, confidence and slew around `curve_follow_raw`."""

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self._kf: list[float] | None = None
    self._conf = 1.0
    self._f = FREE_A_MS2
    self._brake = 0.0
    self.a = FREE_A_MS2
    self.raw = FREE_A_MS2
    self.conf = 1.0
    self.limit_idx = -1
    self.limit_t = None
    self.limit_d = None
    self.limit_v = None
    self.near_a = None
    self.passed = 0.0
    self.gate = 1.0
    self.trusted = False
    self.urgent = 0.0
    self._post_t = -1.0       # time since the post-apex hold began (<0: none)
    self._since_bound = 1e9   # time since the term last slowed the car

  # ---- path preparation -------------------------------------------------
  def _filter_k(self, ks, dt: float) -> list[float]:
    kc = [abs(float(k)) for k in ks]
    n = len(kc)
    # A single point far above both neighbours is a spike, not a bend.
    clipped = list(kc)
    for i in range(n):
      nb = [kc[j] for j in (i - 1, i + 1) if 0 <= j < n]
      if nb and kc[i] > SPIKE_MIN_K and kc[i] > SPIKE_RATIO * max(nb):
        clipped[i] = SPIKE_RATIO * max(nb)
    if self._kf is None or len(self._kf) != n:
      self._kf = list(clipped)
      return list(self._kf)
    for i in range(n):
      cur = self._kf[i]
      x = clipped[i]
      tau = (K_UP_STRONG_TAU_S if x > K_STRONG else K_UP_TAU_S) if x > cur else K_DOWN_TAU_S
      self._kf[i] = cur + dt / (tau + dt) * (x - cur)
    return list(self._kf)

  @staticmethod
  def _snr_weights(ts, ds, ks, y_std) -> list[float]:
    """Lateral displacement of the (signed) path from straight-ahead, in sigmas of its own std.

    Signed on purpose: noise around zero walks, a real bend (or the first arc
    of an S) accumulates.
    """
    n = len(ks)
    w = [1.0] * n
    theta = 0.0
    y = 0.0
    have_std = y_std is not None and len(y_std) == n
    for i in range(1, n):
      dd = max(0.0, float(ds[i]) - float(ds[i - 1]))
      k_mid = 0.5 * (float(ks[i]) + float(ks[i - 1]))
      y += (theta + 0.5 * k_mid * dd) * dd
      theta += k_mid * dd
      if have_std:
        std = max(STD_FLOOR_M, float(y_std[i]))
      else:
        std = STD_FALLBACK_BASE_M + STD_FALLBACK_PER_S_M * max(0.0, float(ts[i]))
      w[i] = _smooth01((abs(y) / std - SNR_S0) / (SNR_S1 - SNR_S0))
    return w

  # ---- one cycle --------------------------------------------------------
  def step(self, *, dt: float, v_ego: float, ts, ds, ks, y_std=None, lane_change: bool = False,
           blinker: bool = False, frame_drop_pct: float = 0.0, rb_active: bool = False,
           valid: bool = True) -> float:
    dt = max(1e-3, float(dt))
    v = float(v_ego)
    path_ok = bool(valid) and ts is not None and len(ts) >= 2 and len(ts) == len(ds) == len(ks)
    ctx_ok = not (lane_change or rb_active or float(frame_drop_pct) > FRAME_DROP_MAX_PCT)

    a_raw = FREE_A_MS2
    self.limit_idx, self.limit_t, self.limit_d, self.limit_v, self.near_a = -1, None, None, None, None
    self.passed = 0.0
    slow = 1.0 - _smooth01((v - LOW_V0_MS) / (LOW_V1_MS - LOW_V0_MS))
    gate = 1.0 - (slow if not blinker else 0.0)
    self.gate = gate
    if path_ok:
      kf = self._filter_k(ks, dt)
      snr = self._snr_weights(ts, ds, ks, y_std)
      weights = []
      for t, s in zip(ts, snr, strict=True):
        far_cut = slow * _smooth01((float(t) - LOW_HORIZON_S) / LOW_HORIZON_BAND_S)
        weights.append(s * (1.0 - far_cut) * gate)
      k_now = kf[0]
      k_ahead = interp_at(ts, kf, PEAK_LOOKAHEAD_S)
      if k_ahead is not None and k_now >= PEAK_MIN_K:
        self.passed = _smooth01((PEAK_RATIO0 - k_ahead / k_now) / PEAK_BAND)
      a_raw, idx, v_lim, self.near_a = curve_follow_raw(v, ts, ds, kf, weights, self.passed)
      if idx >= 0:
        self.limit_idx, self.limit_v = idx, v_lim
        self.limit_t, self.limit_d = float(ts[idx]), float(ds[idx])
    else:
      self._kf = None

    # Confidence: rises like a lead's, fades out on a lane change, dropped
    # frames, an invalid model or the roundabout funnel. Never a step.
    target = 1.0 if (path_ok and ctx_ok) else 0.0
    if target >= self._conf:
      urgency = _smooth01((-a_raw - CONF_URGENT_LO_MS2) / CONF_URGENT_BAND_MS2)
      rate = CONF_RATE_BASE_1_S + CONF_RATE_URGENT_1_S * urgency * self._conf
    else:
      rate = CONF_FALL_GATE_1_S if (path_ok and not ctx_ok) else CONF_FALL_1_S
    self._conf = min(1.0, max(0.0, self._conf + min(1.0, rate * dt) * (target - self._conf)))
    self.conf = self._conf
    self.trusted = bool(path_ok and ctx_ok and self._conf >= 0.2 and v >= MIN_V_MS)

    a_c = FREE_A_MS2 + self._conf * (a_raw - FREE_A_MS2)
    self.urgent = _smooth01((-a_c - CF_URGENT_A0_MS2) / CF_URGENT_BAND_MS2)
    a_c = min(FREE_A_MS2, max(a_c, -decel_limit_ms2(v, a_c)))
    a_c = min(a_c, self._post_apex_cap(dt))
    self.raw = a_c
    out = self._slew(v, a_c, dt)
    self._since_bound = 0.0 if out < 0.0 else self._since_bound + dt
    return out

  def _post_apex_cap(self, dt: float) -> float:
    """+a ceiling right after a bend that slowed the car has peaked."""
    if self._post_t < 0.0:
      if self.passed >= POST_APEX_PASSED and self._since_bound <= POST_APEX_BOUND_S and self.trusted:
        self._post_t = 0.0
      else:
        return FREE_A_MS2
    else:
      self._post_t += dt
    if self._post_t <= POST_APEX_HOLD_S:
      return POST_APEX_A_MS2
    f = (self._post_t - POST_APEX_HOLD_S) / POST_APEX_OPEN_S
    if f >= 1.0:
      self._post_t = -1.0
      return FREE_A_MS2
    return POST_APEX_A_MS2 + (FREE_A_MS2 - POST_APEX_A_MS2) * _smooth01(f)

  def _slew(self, v: float, target: float, dt: float) -> float:
    target = min(FREE_A_MS2, float(target))
    self._f += (dt / (OUT_TAU_S + dt)) * (target - self._f)
    if self._f > FREE_A_MS2 - SNAP_EPS_MS2:
      self._f = FREE_A_MS2
    tb = min(0.0, self._f)
    err = tb - self._brake
    rel_base = _interp(v, RELEASE_BP_MS, RELEASE_BASE_MS3)
    rel_base += max(0.0, RELEASE_OPEN_MS3 - rel_base) * self.passed
    release = min(RELEASE_MAX_MS3, rel_base + RELEASE_PROP_1_S * max(0.0, err)) * dt
    if err < 0.0:
      base = build_jerk_ms3(v, self.urgent)
      build = min(BUILD_MAX_MS3, base + BUILD_PROP_1_S * (-err)) * dt
      delta = max(err, -build)
    else:
      delta = min(err, release)
    self._brake = min(0.0, self._brake + delta)
    out = self._brake + max(0.0, self._f)
    # While the ceiling is still braking it rises no faster than the release.
    if self.a < 0.0:
      out = min(out, self.a + release)
    if out > FREE_A_MS2 - SNAP_EPS_MS2:
      out = FREE_A_MS2
    self.a = out
    return self.a


# ---- logging (plain JSON text, no capnp) ----------------------------------
LOG_PERIOD_CYCLES = 10      # 2 Hz at the 20 Hz planner rate
LOG_QUIET_PERIOD_CYCLES = 50  # arrays on a straight road, every 2.5 s
LOG_SUMMARY_T = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0)
LOG_STD_T = (3.0, 5.0, 8.0)
LOG_PREFIX = "curvefollow "


def _r(x, nd: int):
  return None if x is None else round(float(x), nd)


def path_is_relevant(ks, cf_a: float, old_a: float, conf: float) -> bool:
  kmax = max((abs(float(k)) for k in ks), default=0.0)
  return kmax > 0.003 or cf_a < FREE_A_MS2 - 0.05 or old_a < FREE_A_MS2 - 0.05 or conf < 0.99


def format_log_line(*, mode: int, t: float, v: float, a_ego: float, v_cruise_kph: float, a_target: float,
                    cf: CurveFollow, old_a: float, ts, ds, ks, y_std, meta: dict, arrays: bool) -> str:
  """One compact JSON line. Summary always; the full 33-point arrays on request."""
  rec: dict = {
    "m": int(mode), "t": round(float(t), 2), "v": round(float(v), 2), "ae": round(float(a_ego), 3),
    "mx": round(float(v_cruise_kph), 1), "at": round(float(a_target), 3),
    "cf": round(float(cf.a), 3), "cfr": round(float(cf.raw), 3), "w": round(float(cf.conf), 3),
    "g": round(float(cf.gate), 2), "pk": round(float(cf.passed), 2), "tr": int(cf.trusted),
    "li": int(cf.limit_idx), "lt": _r(cf.limit_t, 2), "ld": _r(cf.limit_d, 1), "lv": _r(cf.limit_v, 2),
    "nf": _r(cf.near_a, 3), "old": round(float(old_a), 3),
  }
  rec.update(meta)
  try:
    rec["k"] = [None if (x := interp_at(ts, ks, s)) is None else round(x * 1e4) for s in LOG_SUMMARY_T]
    if y_std is not None and len(y_std) == len(ts):
      rec["ys"] = [None if (x := interp_at(ts, y_std, s)) is None else round(x, 2) for s in LOG_STD_T]
    if arrays:
      rec["T"] = [round(float(x), 2) for x in ts]
      rec["D"] = [round(float(x), 1) for x in ds]
      rec["K"] = [round(float(x) * 1e4) for x in ks]
      if y_std is not None and len(y_std) == len(ts):
        rec["Y"] = [round(float(x), 2) for x in y_std]
  except (TypeError, ValueError):
    rec["path"] = "unavailable"
  return LOG_PREFIX + json.dumps(rec, separators=(",", ":"))
