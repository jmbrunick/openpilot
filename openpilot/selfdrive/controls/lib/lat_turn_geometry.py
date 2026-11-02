"""Turn geometry correction: when the model plan's heading is sampled.

modeld samples the plan heading at `lat_action_t = lagd + 0.075 s`. lagd
only learns above 15 m/s, so at turn speeds the plan was sampled ~0.14 s
too far ahead (measured desired->yaw lag 0.26 s at 3-8 m/s vs 0.40 s
learned). The model was also trained on cars whose camera sits ~1.55 m ahead
of the rear axle; on the Model S it is 1.90 m (measured), so turn-in is
anchored ~0.35 m early in space. Both are the same knob:

  lookahead = lagd + dly_offset(v) + 0.075 - ref_offset_m / max(v, 3)

Reduction is capped at 0.2 s, the lookahead never goes below 0.2 s (or
below the stock value if that is already lower), it fades in from 1.5 to
2.5 m/s (no effect below 1.5 m/s), and it is held off (slewed back to stock)
while the steering angle is pinned by the 250 deg/s rate limit. Highway
speeds are unchanged apart from ref_offset / v (0.012 s at 30 m/s).

Low-speed reach (Sep 28 PM drive: 5-8 mph turns still clipped the inside by
~0.6-0.8 m). drive_helpers.get_curvature_from_plan treats any action_t below
MIN_STABLE_DELAY (0.3 s) as 0.3 s, so the effective reduction was only
0.167 s, and the 3-4 m/s fade meant ~no correction at 5-8 mph. Below 8 mph
the cap rises to 0.25 s and the plan is sampled directly down to 0.2 s
(`plan_curvature`); both fade back to exactly the 7c49a8e behavior by
10 mph (4.47 m/s). Below 10 mph a rate-limited wheel freezes the reduction
instead of slewing back to stock. At >= 10 mph every output is identical to
7c49a8e.

Turn-in delay trim (NAPTurnInDelay, integer steps -2..+3, default 0). A
user trim on top of the low-speed correction above, for "turn-in is good but
still a little early": every step moves the lookahead by TURN_IN_STEP_S
(30 ms), positive = shorter lookahead = later turn-in, negative = earlier. It
is scaled by the same low-speed band as the reach (full from 3-4 mph up to
8 mph, gone by 10 mph, nothing below 1.5 m/s), so >= 10 mph is untouched.
Step 0 returns the existing output unchanged, bit for bit. The shifted
lookahead never goes below TURN_IN_MIN_LOOKAHEAD_S (0.10 s) or above stock,
and the trim only acts while Turn Geometry Correction is on.

Off, or not Pre-AP, returns the stock lookahead unchanged.
"""
from __future__ import annotations

import numpy as np

from openpilot.selfdrive.controls.lib.drive_helpers import MIN_STABLE_DELAY, curv_from_psis, get_curvature_from_plan

PARAM_TURN_GEOMETRY = "NAPLatTurnGeom"
PARAM_REF_OFFSET = "NAPLatRefOffset"
PARAM_TURN_IN_DELAY = "NAPTurnInDelay"

REF_OFFSET_DEFAULT_M = 0.35
REF_OFFSET_MIN_M = 0.0
REF_OFFSET_MAX_M = 1.0

# Measured desired->yaw lag minus the highway (lagd) lag, per speed.
DELAY_OFFSET_BP_MS = (5.0, 10.0, 14.0, 19.0, 25.0)
DELAY_OFFSET_S = (-0.13, -0.10, -0.08, -0.05, 0.0)

MIN_SPEED_MS = 1.5          # no correction at or below
FADE_IN_SPEED_MS = 2.5      # full correction from here
REF_SPEED_FLOOR_MS = 3.0    # offset / max(v, 3)
MAX_REDUCTION_S = 0.20
MIN_LOOKAHEAD_S = 0.20

# Low-speed reach: full below 8 mph, gone (== 7c49a8e) from 10 mph.
MPH = 0.44704
LOW_SPEED_REACH_FULL_MS = 8.0 * MPH
LOW_SPEED_REACH_END_MS = 10.0 * MPH
LOW_SPEED_MAX_REDUCTION_S = 0.25
LOW_SPEED_SAMPLE_FLOOR_S = MIN_LOOKAHEAD_S   # plan sampled directly down to 0.2 s

# Turn-in delay trim (user step, low-speed band only).
TURN_IN_STEP_MIN = -2
TURN_IN_STEP_MAX = 3
TURN_IN_STEP_S = 0.030            # lookahead change per step at full weight
TURN_IN_MIN_LOOKAHEAD_S = 0.10    # trimmed lookahead / sample time never below

# Steering rate limit hold-off. Pre-AP MAX_ANGLE_RATE is 5 deg per 20 ms.
STEER_RATE_LIMIT_DEG_S = 250.0
RATE_LIMIT_FRACTION = 0.8        # output moving >= 200 deg/s
RATE_LIMIT_GAP_DEG = 2.5         # and still short of the command
RATE_LIMIT_HOLD_S = 0.5          # keep the stock lookahead this long after
REDUCTION_SLEW_S_PER_S = 0.5     # 0.2 s of correction moves in 0.4 s


def clamp_ref_offset(offset_m) -> float:
  try:
    v = float(offset_m)
  except (TypeError, ValueError):
    return REF_OFFSET_DEFAULT_M
  if not np.isfinite(v):
    return REF_OFFSET_DEFAULT_M
  return float(min(REF_OFFSET_MAX_M, max(REF_OFFSET_MIN_M, v)))


def clamp_turn_in_step(step) -> int:
  """NAPTurnInDelay as an int in [TURN_IN_STEP_MIN, TURN_IN_STEP_MAX]; junk -> 0."""
  try:
    v = float(step)
  except (TypeError, ValueError):
    return 0
  if not np.isfinite(v):
    return 0
  return int(min(TURN_IN_STEP_MAX, max(TURN_IN_STEP_MIN, round(v))))


def delay_offset_s(v_ego: float) -> float:
  """Low-speed lag correction (<= 0). 0 at >= 25 m/s."""
  return float(np.interp(float(v_ego), DELAY_OFFSET_BP_MS, DELAY_OFFSET_S))


def ref_offset_s(v_ego: float, ref_offset_m: float) -> float:
  """Rear reference offset as time: offset / max(v, 3)."""
  return clamp_ref_offset(ref_offset_m) / max(float(v_ego), REF_SPEED_FLOOR_MS)


def speed_fade(v_ego: float) -> float:
  return float(np.clip((float(v_ego) - MIN_SPEED_MS) / (FADE_IN_SPEED_MS - MIN_SPEED_MS), 0.0, 1.0))


def low_speed_reach_weight(v_ego: float) -> float:
  """1 below 8 mph, 0 from 10 mph (linear between)."""
  w = (LOW_SPEED_REACH_END_MS - float(v_ego)) / (LOW_SPEED_REACH_END_MS - LOW_SPEED_REACH_FULL_MS)
  return float(np.clip(w, 0.0, 1.0))


def reduction_cap_s(v_ego: float) -> float:
  """0.25 s below 8 mph, exactly MAX_REDUCTION_S from 10 mph."""
  w = low_speed_reach_weight(v_ego)
  if w <= 0.0:
    return MAX_REDUCTION_S
  return MAX_REDUCTION_S + (LOW_SPEED_MAX_REDUCTION_S - MAX_REDUCTION_S) * w


def sample_floor_s(v_ego: float) -> float:
  """Shortest plan sample time: 0.2 s below 8 mph, MIN_STABLE_DELAY (stock) from 10 mph."""
  w = low_speed_reach_weight(v_ego)
  if w <= 0.0:
    return MIN_STABLE_DELAY
  return MIN_STABLE_DELAY - (MIN_STABLE_DELAY - LOW_SPEED_SAMPLE_FLOOR_S) * w


def plan_curvature(yaws, yaw_rates, t_idxs, v_ego: float, action_t: float,
                   sample_floor: float = MIN_STABLE_DELAY) -> float:
  """get_curvature_from_plan, but the plan may be sampled directly below 0.3 s.

  With the stock floor (or action_t >= 0.3 s) this IS get_curvature_from_plan.
  """
  floor = float(sample_floor)
  if floor >= MIN_STABLE_DELAY or float(action_t) >= MIN_STABLE_DELAY:
    return get_curvature_from_plan(yaws, yaw_rates, t_idxs, v_ego, action_t)
  if floor < LOW_SPEED_SAMPLE_FLOOR_S:   # turn-in trim: sampled below 0.2 s
    t = max(float(action_t), floor, TURN_IN_MIN_LOOKAHEAD_S)
  else:
    t = max(float(action_t), floor, LOW_SPEED_SAMPLE_FLOOR_S)
  psi_target = np.interp(t, t_idxs, yaws)
  return curv_from_psis(psi_target, yaw_rates[0], v_ego, t)


def target_reduction_s(v_ego: float, ref_offset_m: float) -> float:
  """Seconds to take off the stock lookahead (>= 0, <= reduction_cap_s(v))."""
  v = max(0.0, float(v_ego))
  red = min(reduction_cap_s(v), max(0.0, -delay_offset_s(v) + ref_offset_s(v, ref_offset_m)))
  return red * speed_fade(v)


def apply_reduction(stock_lookahead_s: float, reduction_s: float, cap_s: float = MAX_REDUCTION_S) -> float:
  """Stock minus reduction, floored at MIN_LOOKAHEAD_S (never above stock)."""
  stock = float(stock_lookahead_s)
  red = min(float(cap_s), max(0.0, float(reduction_s)))
  if red <= 0.0:
    return stock
  return max(min(stock, MIN_LOOKAHEAD_S), stock - red)


def turn_in_weight(v_ego: float) -> float:
  """Low-speed band of the trim: 0 below 1.5 m/s, full from 2.5 m/s to 8 mph, 0 from 10 mph."""
  return low_speed_reach_weight(v_ego) * speed_fade(v_ego)


def turn_in_shift_s(step, v_ego: float) -> float:
  """Lookahead shift (s) the trim asks for at this speed. + = later turn-in."""
  return clamp_turn_in_step(step) * TURN_IN_STEP_S * turn_in_weight(v_ego)


def apply_turn_in_trim(lookahead_s: float, stock_lookahead_s: float, shift_s: float) -> float:
  """Shift the lookahead by shift_s; 0 returns it unchanged. Bounded, never above stock."""
  L = float(lookahead_s)
  shift = float(shift_s)
  if shift == 0.0:
    return L
  if shift > 0.0:
    return max(min(L, TURN_IN_MIN_LOOKAHEAD_S), L - shift)
  return min(max(L, float(stock_lookahead_s)), L - shift)


def steer_rate_limited(cmd_angle_deg: float, out_angle_deg: float, out_angle_prev_deg: float,
                       dt: float, lat_active: bool) -> bool:
  """True while the output angle is running at the rate limit and still short of the command."""
  if not lat_active or dt <= 0.0:
    return False
  rate = abs(float(out_angle_deg) - float(out_angle_prev_deg)) / dt
  gap = abs(float(cmd_angle_deg) - float(out_angle_deg))
  return rate >= RATE_LIMIT_FRACTION * STEER_RATE_LIMIT_DEG_S and gap > RATE_LIMIT_GAP_DEG


class TurnGeometryCorrection:
  """Stateful wrapper: hold-off on rate limit + slewed reduction."""

  def __init__(self, dt: float):
    self.dt = float(dt)
    self.reduction_s = 0.0
    self._hold_s = 0.0
    self._out_prev: float | None = None
    self.rate_limited = False
    self.sample_floor_s = MIN_STABLE_DELAY
    self.trim_s = 0.0

  def reset(self) -> None:
    self.trim_s = 0.0
    self.reduction_s = 0.0
    self._hold_s = 0.0
    self._out_prev = None
    self.rate_limited = False
    self.sample_floor_s = MIN_STABLE_DELAY

  def update(self, *, enabled: bool, stock_lookahead_s: float, v_ego: float, ref_offset_m: float,
             lat_active: bool = True, cmd_angle_deg: float = 0.0, out_angle_deg: float | None = None,
             turn_in_step: int = 0) -> float:
    """Return the lookahead to use. enabled=False returns stock_lookahead_s exactly."""
    if not enabled:
      self.reset()
      return stock_lookahead_s
    limited = False
    if out_angle_deg is not None:
      if self._out_prev is not None:
        limited = steer_rate_limited(cmd_angle_deg, out_angle_deg, self._out_prev, self.dt, lat_active)
      self._out_prev = float(out_angle_deg)
    self.rate_limited = limited
    self._hold_s = RATE_LIMIT_HOLD_S if limited else max(0.0, self._hold_s - self.dt)
    if self._hold_s > 0.0 and low_speed_reach_weight(v_ego) > 0.0:
      target = self.reduction_s   # below 10 mph: freeze, keep the turn-in timing
    else:
      target = 0.0 if self._hold_s > 0.0 else target_reduction_s(v_ego, ref_offset_m)
    step = REDUCTION_SLEW_S_PER_S * self.dt
    self.reduction_s += float(np.clip(target - self.reduction_s, -step, step))
    self.sample_floor_s = sample_floor_s(v_ego)
    # User turn-in trim: slewed so a menu change is not a step in the steering.
    shift_target = turn_in_shift_s(turn_in_step, v_ego)
    trim_step = REDUCTION_SLEW_S_PER_S * self.dt
    self.trim_s += float(np.clip(shift_target - self.trim_s, -trim_step, trim_step))
    if self.trim_s != 0.0:
      trim_floor = self.sample_floor_s - self.trim_s
      self.sample_floor_s = float(min(MIN_STABLE_DELAY, max(TURN_IN_MIN_LOOKAHEAD_S, trim_floor)))
      lookahead = apply_reduction(stock_lookahead_s, self.reduction_s, reduction_cap_s(v_ego))
      return apply_turn_in_trim(lookahead, stock_lookahead_s, self.trim_s)
    return apply_reduction(stock_lookahead_s, self.reduction_s, reduction_cap_s(v_ego))


def turn_geometry_active(is_preap: bool, param_on: bool) -> bool:
  return bool(is_preap) and bool(param_on)


def is_preap_car(CP) -> bool:
  return getattr(CP, "brand", "") == "tesla" and getattr(CP, "carFingerprint", "") == "TESLA_MODEL_S_PREAP"
