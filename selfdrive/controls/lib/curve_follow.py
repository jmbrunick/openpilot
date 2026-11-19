"""Curve-follow: a continuous longitudinal term from the model path ahead (Pre-AP).

A human eases off as soon as a bend is visible that cannot be taken
comfortably at the current speed, then keeps a smooth, gradual deceleration
and comes back on the gas as the road opens — never pulsing
(brake-accelerate-brake) through a curve or an S. This module is that.

Every cycle the road ahead is a speed envelope v_safe(s) (Justin's median
apex speed for that radius, capped at 2.8 m/s² lateral; see
selfdrive/nap/curve_speed). For each visible point the kinematic accel
to meet it over the full remaining distance is

  a_i = (v_safe_i² - v²) / (2 · d_i)

The command is the minimum of those (the tightest constraint), spread over
the full visible distance, with a small ease floor so a tiny required
decel still lifts off the gas instead of waiting. The result is a CEILING
on acceleration: it can only lower the command and is exactly
PREVIEW_FREE_A_MS2 when nothing ahead binds. It never writes MAX and keeps
no set speed.

A curve event latches when trusted braking starts. While the event is
active the ceiling may deepen or ease toward zero, but never flips to +a
and back. Back-to-back / S-curves still visible in the path stay one
event: the car coasts or holds between them instead of re-accelerating.
Exit is confirmed only once the path is opening and no remaining point
requires braking; +a then blends back at the comfort jerk so Accel 1
(unchanged) can climb toward MAX.

Reliability (how much to believe the forecast, per point and overall):
- a bend must stand out of the path's own position std (signal to noise of
  its lateral displacement), so noisy far points count little and a clear
  bend counts fully, at any distance, without a fixed horizon;
- curvature is filtered per path index: fast to tighten (faster when the
  bend is strong), slower to open, and a single-point spike is clipped;
- a lane change, dropped frames, an invalid model or an active roundabout
  funnel fade the term out (it never steps); below 8 m/s only a turn with
  the blinker on is anticipated, and only out to ~3 s.

Shadow: with NAPCurveFollow = 1 the planner computes and logs this every
cycle and applies nothing. 0 = off, 2 = active (replaces the old preview and
retires the CurveMaxHold MAX cap).
"""
from __future__ import annotations

import json

from openpilot.selfdrive.controls.lib.curve_preview import (
  PREVIEW_FREE_A_MS2,
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
from openpilot.selfdrive.nap.curve_speed.emergency import (
  EMERGENCY_DECEL_MS2 as NAP_EMERGENCY_DECEL_MS2, EMERGENCY_JERK_MS3 as NAP_EMERGENCY_JERK_MS3,
  NORMAL_DECEL_MS2 as NAP_NORMAL_DECEL_MS2, needs_emergency,
)
from openpilot.selfdrive.nap.curve_speed.median_table import (
  URGENT_DECEL_MS2 as NAP_URGENT_DECEL_MS2, URGENT_JERK_MS3 as NAP_URGENT_JERK_MS3, corner_speed,
)
CF_LAT_TARGET_BP_MS = [8.0, 13.0, 18.0, 22.0, 27.0]
CF_LAT_TARGET_MS2 = [1.45, 1.55, 1.85, 2.05, 2.20]
CF_SPEED_FLOOR_MS = 6.0
CF_SPEED_CEIL_MS = 45.0  # bends gentler than this speed's table are not bends

# Comfort / urgent decel caps vs speed. Justin's own slowing into a bend
# averages ~0.5 m/s² (lift / regen, peaks ~1.3). When the kinematic demand
# is beyond CF_URGENT_A0_MS2 the limit grows toward CF_URGENT_DECEL_MS2
# (still jerk limited). The emergency tier may go to 3.4 m/s² when a sharp
# bend is seen too late for the normal cap.
CF_DECEL_BP_MS = [10.0, 25.0]
CF_DECEL_MS2 = [1.2, 0.55]
CF_URGENT_DECEL_MS2 = NAP_URGENT_DECEL_MS2
CF_URGENT_JERK_MS3 = NAP_URGENT_JERK_MS3
# Comfort covers the gradual-lift band (a sweeper seen early stays
# around 0.5–0.8 m/s²). Urgent is for a late or tightening bite past that.
CF_URGENT_A0_MS2 = 0.8
CF_URGENT_BAND_MS2 = 0.5

# Room is the full distance ahead, plus a small numeric / actuator-lag floor.
# There is no time-based "finish slowing Xs before the apex" term: that
# made the required decel later and harder. Softplus keeps room >= min_room.
# Smoothness floor so a tiny leftover deficit does not spike the
# kinematic a over the last few meters. This is not a finish-before-apex
# buffer: onset still uses the full remaining visible distance.
MIN_ROOM_M = 43.0
MIN_ROOM_S = 2.15
ROOM_SOFT_M = 2.0

# A tiny but real required decel still eases off the gas instead of waiting.
EASE_FLOOR_MS2 = 0.05
# Ignore wobble around the current demand or around zero (no gas/brake toggle).
DEADBAND_MS2 = 0.03
EXIT_MARGIN_MS = 0.45          # ~1 mph: v_safe must clear this to allow +a
EXIT_CONFIRM_S = 0.30
# Exit without a passed apex (gentle sweeper, or the road is straight): the
# whole trusted path must clear v by this much, for this long. Wider and
# slower than the apex exit so a near-envelope sweeper stays one event.
EXIT_OPEN_MARGIN_MS = 2.0      # ~4.5 mph
EXIT_OPEN_MIN_W = 0.3
EXIT_OPEN_CONFIRM_S = 1.5
EVENT_START_S = 0.15

# Comfort-path jerk (onset and release). Urgent / emergency keep their
# existing firmer jerks for a late-revealed sharp bend. Gate-off (lane
# change / dropped frames) may release a little faster so we do not keep
# braking after the term is supposed to fade.
COMFORT_JERK_MS3 = 0.45
GATE_RELEASE_JERK_MS3 = 1.00

# Per-point required-decel filter: tighten reasonably quickly, release slowly,
# so model-path noise cannot oscillate the demand.
DEMAND_TIGHTEN_TAU_S = 0.25
DEMAND_RELEASE_TAU_S = 0.80

# Curvature filter per path index (|k|): tighten fast (faster when strong),
# open slowly so a noisy "it opens" cannot release early.
K_UP_TAU_S = 0.20
K_UP_STRONG_TAU_S = 0.12
K_STRONG = 0.02
K_DOWN_TAU_S = 0.50
SPIKE_MIN_K = 0.01
SPIKE_RATIO = 2.0

# Signal to noise of a bend's lateral displacement against the path's std.
SNR_S0 = 1.0
SNR_S1 = 4.0
STD_FLOOR_M = 0.2
SNR_LOOK_AHEAD = True
SHARP_K0 = 0.010
SHARP_K_BAND = 0.006
SHARP_T_S = 6.0
STD_FALLBACK_BASE_M = 0.3   # no yStd in the message
STD_FALLBACK_PER_S_M = 0.15

# Low speed: only a blinkered turn, only ~3 s ahead.
LOW_V0_MS = 6.5
LOW_V1_MS = 8.0
LOW_HORIZON_S = 2.5
LOW_HORIZON_BAND_S = 0.5

# Overall confidence (lead-law style ramp).
CONF_RATE_BASE_1_S = 2.0
CONF_RATE_URGENT_1_S = 30.0
CONF_FALL_1_S = 1.5
CONF_FALL_GATE_1_S = 2.0
CONF_URGENT_LO_MS2 = 0.5
CONF_URGENT_BAND_MS2 = 1.5
FRAME_DROP_MAX_PCT = 20.0

# Opening / apex: the path ahead is loosening relative to now.
PEAK_LOOKAHEAD_S = 1.0
PEAK_RATIO0 = 0.9
PEAK_BAND = 0.3
PEAK_MIN_K = 0.004
NEAR_BRAKE_MAX_MS2 = 0.35

# Output low-pass around the jerk-limited command.
OUT_TAU_S = 0.20
SNAP_EPS_MS2 = 0.01


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
  """Curve follow's corner speed: Justin's median for this radius, capped by steering (selfdrive/nap/curve_speed)."""
  v = corner_speed(kappa)
  if v is None or v >= CF_SPEED_CEIL_MS:
    return None  # gentler than any speed we drive: not a bend
  return max(CF_SPEED_FLOOR_MS, v)


def decel_limit_ms2(v_ego: float, demand_ms2: float = 0.0) -> float:
  """Most decel the term may ask for: speed table, plus the urgent allowance."""
  base = _interp(float(v_ego), CF_DECEL_BP_MS, CF_DECEL_MS2)
  u = _smooth01((-float(demand_ms2) - CF_URGENT_A0_MS2) / CF_URGENT_BAND_MS2)
  return base + max(0.0, CF_URGENT_DECEL_MS2 - base) * u


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


def _point_accel(v: float, d: float, v_env: float, min_room: float) -> float:
  """Kinematic a to meet v_env over the full remaining distance."""
  room = min_room + _softplus(float(d) - min_room, ROOM_SOFT_M)
  a_kin = (v_env * v_env - v * v) / (2.0 * room)
  # Tiny but real deficits still ease off; ignore sub-mph wobble.
  if -EASE_FLOOR_MS2 < a_kin < 0.0 and (v - v_env) > EXIT_MARGIN_MS:
    a_kin = -EASE_FLOOR_MS2
  return min(FREE_A_MS2, a_kin)


def curve_follow_points(v_ego: float, ts, ds, ks, weights):
  """Per-point unslewed ceiling and the tightest constraint.

  Returns (a, limiting_index, limiting_envelope_speed, near_field_a, a_pts).
  `a` is FREE_A_MS2 exactly when no point binds. `weights` in [0, 1] scale
  each point's demand toward "no demand".
  """
  v = float(v_ego)
  n = 0 if ts is None else len(ts)
  a_pts = [FREE_A_MS2] * n
  if v < MIN_V_MS or ts is None:
    return FREE_A_MS2, -1, None, None, a_pts
  min_room = max(MIN_ROOM_M, MIN_ROOM_S * v)
  best = FREE_A_MS2
  best_i = -1
  best_v = None
  for i, (t, d, k, w) in enumerate(zip(ts, ds, ks, weights, strict=True)):
    if w <= 0.0 or t < 0.0:
      continue
    v_env = curve_speed_for_curvature(k)
    if v_env is None:
      continue
    a_i = _point_accel(v, float(d), v_env, min_room)
    # A point at or under its envelope must not pull the +a ceiling off
    # FREE. The previous FREE+w*(a_i-FREE) blend turned a mild (or even
    # positive) a_i into a coast cap, so path noise looked like a bend.
    if a_i >= -EASE_FLOOR_MS2:
      a_eff = FREE_A_MS2
    else:
      a_eff = FREE_A_MS2 + float(w) * (a_i - FREE_A_MS2)
    a_pts[i] = a_eff
    if a_eff < best:
      best, best_i, best_v = a_eff, i, v_env
  a_near = None
  v0 = curve_speed_for_curvature(ks[0]) if len(ks) else None
  if v0 is not None and weights[0] > 0.0:
    over = speed_ceiling_accel(v, v0)
    if over < 0.0:
      a_near = FREE_A_MS2 + float(weights[0]) * (max(over, -NEAR_BRAKE_MAX_MS2) - FREE_A_MS2)
      a_pts[0] = min(a_pts[0], a_near)
      if a_near < best:
        best, best_i, best_v = a_near, 0, v0
  return best, best_i, best_v, a_near, a_pts


def hold_constraint_ahead(v_ego: float, ts, ds, ks, weights, margin: float = EXIT_MARGIN_MS) -> bool:
  """True if any trusted point ahead is at or below the current speed.

  Accelerating into that point would require braking again, so an active
  event should coast or hold instead of climbing.
  """
  v = float(v_ego)
  if ts is None:
    return False
  for t, k, w in zip(ts, ks, weights, strict=True):
    if w < 0.6 or t < 0.0:
      continue
    v_env = curve_speed_for_curvature(k)
    if v_env is not None and v_env <= v + margin:
      return True
  return False


def bend_open_at_car(v_ego: float, ts, ks, weights, margin: float | None = None) -> bool:
  """True when the road ahead is clearly no longer a bend at this speed.

  Every point the path still trusts (and the first PEAK_LOOKAHEAD_S under
  the car regardless of trust) must have a corner speed at least `margin`
  above v. The margin is wider than the hold margin so a long sweeper
  driven near its envelope stays one event (no brake / +a / brake).
  """
  if ts is None or len(ts) == 0:
    return False
  m = EXIT_OPEN_MARGIN_MS if margin is None else float(margin)
  v = float(v_ego)
  for t, k, w in zip(ts, ks, weights, strict=True):
    if float(t) < 0.0:
      continue
    if float(t) > PEAK_LOOKAHEAD_S and w < EXIT_OPEN_MIN_W:
      continue
    v_env = curve_speed_for_curvature(k)
    if v_env is not None and v_env < v + m:
      return False
  return True


def curve_follow_raw(v_ego: float, ts, ds, ks, weights, passed: float = 0.0):
  """Unslewed ceiling from the (filtered) path ahead.

  `passed` is accepted for call-site compatibility and is not used: exit
  handling lives in CurveFollow (event latch), not in the raw kinematics.
  """
  _ = passed
  best, best_i, best_v, a_near, _a_pts = curve_follow_points(v_ego, ts, ds, ks, weights)
  return best, best_i, best_v, a_near


class CurveFollow:
  """Filters, confidence, event latch and comfort-jerk slew around the raw ceiling."""

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self._kf: list[float] | None = None
    self._af: list[float] | None = None
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
    self.limit_k = None
    self.emergency = False
    self.near_a = None
    self.passed = 0.0
    self.gate = 1.0
    self.trusted = False
    self.urgent = 0.0
    self.event = False
    self._hold_ahead = False
    self._open_at_car = False
    self._open_t = 0.0
    self._exit_t = 0.0
    self._demand_t = 0.0
    self._desired = FREE_A_MS2
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

  def _filter_demand(self, a_pts: list[float], dt: float) -> list[float]:
    """Low-pass each point's required accel: faster to tighten, slower to release."""
    n = len(a_pts)
    if self._af is None or len(self._af) != n:
      # Ignore a first-frame cap unless it is already a real brake demand.
      # Path noise that only trims +a must not snap the filter in; a visible
      # bend still starts immediately.
      self._af = [FREE_A_MS2 if float(a) >= -0.25 else float(a) for a in a_pts]
    for i in range(n):
      cur = self._af[i]
      x = float(a_pts[i])
      if x < cur:
        tau = DEMAND_TIGHTEN_TAU_S
      elif x >= FREE_A_MS2 - SNAP_EPS_MS2:
        # Path no longer asks: drop a stale cap as fast as we would tighten.
        tau = DEMAND_TIGHTEN_TAU_S
      else:
        tau = DEMAND_RELEASE_TAU_S
      self._af[i] = cur + dt / (tau + dt) * (x - cur)
    return list(self._af)

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
    # A sharp bend within SHARP_T_S is credible on its own curvature: its
    # displacement only builds after it, too late for the std test.
    # Signed 3-point mean: a real bend spans several points, noise flips sign.
    for i in range(1, n - 1):
      k3 = abs(float(ks[i - 1]) + float(ks[i]) + float(ks[i + 1])) / 3.0
      sharp = _smooth01((k3 - SHARP_K0) / SHARP_K_BAND)
      if sharp <= 0.0:
        continue
      # Full model horizon (~10 s) may start a light ease as soon as a
      # sharp bend is visible, but only if the path std is not huge.
      std_ok = (not have_std) or float(y_std[i]) < 4.0
      if float(ts[i]) <= SHARP_T_S or (float(ts[i]) <= 10.0 and std_ok):
        w[i] = max(w[i], sharp)
    if SNR_LOOK_AHEAD:
      # A bend at point i only shows as displacement after it: credit point i
      # with the best SNR at or beyond it (the path really does bend away).
      for i in range(n - 2, -1, -1):
        w[i] = max(w[i], w[i + 1])
    return w

  # ---- one cycle --------------------------------------------------------
  def step(self, *, dt: float, v_ego: float, ts, ds, ks, y_std=None, lane_change: bool = False,
           blinker: bool = False, frame_drop_pct: float = 0.0, rb_active: bool = False,
           valid: bool = True, long_active: bool = True) -> float:
    """One planner cycle.

    `long_active` False (disengaged, or the driver's gas override) ends any
    curve event: the path filters and confidence keep tracking so a
    re-engage sees the road at once, but the latch, its timers and the
    jerk-limited output start fresh and the term is free while inactive.
    """
    dt = max(1e-3, float(dt))
    v = float(v_ego)
    path_ok = bool(valid) and ts is not None and len(ts) >= 2 and len(ts) == len(ds) == len(ks)
    ctx_ok = not (lane_change or rb_active or float(frame_drop_pct) > FRAME_DROP_MAX_PCT)

    a_raw = FREE_A_MS2
    self.limit_idx, self.limit_t, self.limit_d, self.limit_v, self.near_a = -1, None, None, None, None
    self.limit_k = None
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
      _a0, idx, v_lim, self.near_a, a_pts = curve_follow_points(v, ts, ds, kf, weights)
      a_filt = self._filter_demand(a_pts, dt)
      a_raw = min(a_filt) if a_filt else FREE_A_MS2
      # Keep the limiting index aligned with the filtered demand.
      if a_raw < FREE_A_MS2 - SNAP_EPS_MS2:
        idx = min(range(len(a_filt)), key=lambda i: a_filt[i])
        v_lim = curve_speed_for_curvature(kf[idx])
      if idx >= 0:
        self.limit_idx, self.limit_v = idx, v_lim
        self.limit_t, self.limit_d = float(ts[idx]), float(ds[idx])
        self.limit_k = float(kf[idx])
      self._hold_ahead = hold_constraint_ahead(v, ts, ds, kf, weights)
      self._open_at_car = bend_open_at_car(v, ts, kf, weights)
    else:
      self._hold_ahead = False
      self._open_at_car = False
      self._kf = None
      self._af = None

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

    if rb_active and not ctx_ok:
      # The map roundabout funnel owns the entry (it brakes for the ring
      # itself, and the planner takes the lower of the two): hand over at once.
      self._conf = self.conf = 0.0
      return self._end_event()
    if not long_active:
      # Not in control (disengaged / gas override): no event survives it.
      return self._end_event()

    a_c = FREE_A_MS2 + self._conf * (a_raw - FREE_A_MS2)
    self.urgent = _smooth01((-a_c - CF_URGENT_A0_MS2) / CF_URGENT_BAND_MS2)
    room = None if self.limit_d is None else float(self.limit_d)
    self.emergency = bool(a_c < -NAP_NORMAL_DECEL_MS2 and self._conf > 0.5 and needs_emergency(v, room, self.limit_k))
    a_demand = a_c
    a_c = min(FREE_A_MS2, max(a_c, -decel_limit_ms2(v, a_c)))
    if self.emergency:
      a_c = min(a_c, max(a_demand, -NAP_EMERGENCY_DECEL_MS2))

    a_c = self._event_ceiling(a_c, v, dt)
    a_c = self._deadband(a_c)
    self.raw = a_c
    out = self._slew(v, a_c, dt, gated_off=not ctx_ok)
    self._since_bound = 0.0 if out < -DEADBAND_MS2 else self._since_bound + dt
    return out

  def _end_event(self) -> float:
    """Drop the event latch and the jerk-limited output state; the term is free."""
    self._brake, self._f, self.a = 0.0, FREE_A_MS2, FREE_A_MS2
    self.event = False
    self._exit_t = 0.0
    self._open_t = 0.0
    self._demand_t = 0.0
    self._desired = FREE_A_MS2
    self.raw = FREE_A_MS2
    self.urgent = 0.0
    self.emergency = False
    return self.a

  def _event_ceiling(self, a_c: float, v: float, dt: float) -> float:
    """Latch a curve event so the ceiling cannot pulse +a / -a mid-curve.

    While the event is active the ceiling stays <= 0 unless exit is confirmed
    (path opening, and no remaining point still asks to brake or would if we
    started climbing). A later S-bend still on the path keeps the event, so
    the car coasts or holds instead of re-accelerating.
    """
    brake_demand = a_c < -EASE_FLOOR_MS2
    hold_for_next = bool(self._hold_ahead)
    open_clear = self._open_at_car and self.trusted and not brake_demand and not hold_for_next
    self._open_t = self._open_t + dt if open_clear else 0.0
    # The bend is behind us when either the apex has been passed (curvature
    # ahead falling off a real bend) or the whole trusted path has cleared v
    # by EXIT_OPEN_MARGIN_MS for EXIT_OPEN_CONFIRM_S. The apex test alone can
    # never fire on a gentle sweeper (|k| under PEAK_MIN_K) or once the road
    # is straight (k_now ~ 0), which left the event latched with a 0 ceiling
    # for 28 min (Nov 18 2026 drive).
    opening = self.passed >= 0.5 or self._open_t >= EXIT_OPEN_CONFIRM_S
    # Exit only once the path is opening and nothing ahead still requires a
    # hold or a brake. Do not use a "we've been at v_safe for a moment"
    # shortcut: that lets cruise climb mid-arc and then pulse back.
    can_exit = (not brake_demand) and (not hold_for_next) and opening and self.trusted

    if not self.event:
      # Need a clear, trusted, sustained deficit so path noise cannot latch.
      if brake_demand and self.trusted and self._conf >= 0.5:
        self._demand_t += dt
        if self._demand_t >= EVENT_START_S:
          self.event = True
          self._exit_t = 0.0
      else:
        self._demand_t = 0.0
        if a_c > -EASE_FLOOR_MS2:
          # On a straight or with only tiny wobble, stay fully free (not 0).
          return a_c if a_c > 0.0 else FREE_A_MS2
      return a_c

    # Event active: never go positive until exit is confirmed.
    if brake_demand or hold_for_next or not can_exit:
      self._exit_t = 0.0
      return min(0.0, a_c)

    self._exit_t += dt
    if self._exit_t < EXIT_CONFIRM_S:
      return min(0.0, a_c)

    # Confirmed opening, no remaining constraint: blend +a (jerk-limited in _slew).
    if a_c >= -DEADBAND_MS2 and self.a >= -DEADBAND_MS2:
      self.event = False
      self._exit_t = 0.0
      self._demand_t = 0.0
    return a_c

  def _deadband(self, a_c: float) -> float:
    """Hold the demand still when the change is tiny, especially around zero.

    Never snap a free/positive ceiling down to 0: that would block Accel 1
    on a straight. Inside an event, wobble around 0 stays 0 (coast/hold).
    """
    if self.event:
      if abs(a_c) < DEADBAND_MS2:
        a_c = 0.0
    elif a_c > -EASE_FLOOR_MS2 and a_c < DEADBAND_MS2:
      a_c = FREE_A_MS2
    if abs(a_c - self._desired) < DEADBAND_MS2:
      return self._desired
    self._desired = a_c
    return a_c

  def _slew(self, v: float, target: float, dt: float, gated_off: bool = False) -> float:
    target = min(FREE_A_MS2, float(target))
    self._f += (dt / (OUT_TAU_S + dt)) * (target - self._f)
    if self._f > FREE_A_MS2 - SNAP_EPS_MS2:
      self._f = FREE_A_MS2
    tb = min(0.0, self._f)
    err = tb - self._brake
    if err < 0.0:
      # Building decel: comfort jerk, or the urgent / emergency allowance.
      if self.emergency:
        jerk = NAP_EMERGENCY_JERK_MS3
      elif self.urgent > 0.0:
        jerk = COMFORT_JERK_MS3 + (CF_URGENT_JERK_MS3 - COMFORT_JERK_MS3) * float(self.urgent)
      else:
        jerk = COMFORT_JERK_MS3
      delta = max(err, -jerk * dt)
    else:
      # Once the envelope is met, drop a leftover firm brake quickly so
      # the car does not run well under the table. Mid-curve easing of a
      # still-active demand stays at comfort jerk.
      met = (not gated_off) and target >= -EASE_FLOOR_MS2 and self._brake < -0.3
      jerk = GATE_RELEASE_JERK_MS3 if (gated_off or met) else COMFORT_JERK_MS3
      delta = min(err, jerk * dt)
    self._brake = min(0.0, self._brake + delta)
    out = self._brake + max(0.0, self._f)
    # Bound a rising ceiling so a coast at 0 cannot snap to FREE in one
    # frame when the event ends (_f may already have climbed). Exit +a
    # uses the gate-release jerk (soft blend into Accel 1); a still-braking
    # ceiling eases at comfort unless the path dropped.
    if out > self.a:
      rising_off_brake = self.a < 0.0 and not gated_off
      rel = (COMFORT_JERK_MS3 if rising_off_brake else GATE_RELEASE_JERK_MS3) * dt
      out = min(out, self.a + rel)
    if out > FREE_A_MS2 - SNAP_EPS_MS2:
      out = FREE_A_MS2
    # Event monotonicity: once we are braking or coasting, do not go
    # positive until exit is confirmed. Do not snap a still-free ceiling
    # down to 0 at event start — that is a jerk and delays a gradual ease.
    if self.event and self._exit_t < EXIT_CONFIRM_S and self.a <= DEADBAND_MS2:
      out = min(0.0, out)
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
    "ev": int(cf.event),
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
