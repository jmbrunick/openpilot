"""Turn geometry correction: when the model plan's heading is sampled.

modeld samples the plan heading at `lat_action_t = lagd + 0.075 s`. lagd
only learns above 15 m/s, so at turn speeds the plan was sampled ~0.14 s
too far ahead (measured desired->yaw lag 0.26 s at 3-8 m/s vs 0.40 s
learned). The model was also trained on cars whose camera sits ~1.55 m ahead
of the rear axle; on the Model S it is 1.90 m (measured), so turn-in is
anchored ~0.35 m early in space. Both are the same knob:

  lookahead = lagd + dly_offset(v) + 0.075 - ref_offset_m / max(v, 3)

Reduction is capped at 0.2 s, the lookahead never goes below 0.2 s (or
below the stock value if that is already lower), it fades in from 3 to
4 m/s (no effect below 3 m/s), and it is held off (slewed back to stock)
while the steering angle is pinned by the 250 deg/s rate limit. Highway
speeds are unchanged apart from ref_offset / v (0.012 s at 30 m/s).

Off, or not Pre-AP, returns the stock lookahead unchanged.
"""
from __future__ import annotations

import numpy as np

PARAM_TURN_GEOMETRY = "NAPLatTurnGeom"
PARAM_REF_OFFSET = "NAPLatRefOffset"

REF_OFFSET_DEFAULT_M = 0.35
REF_OFFSET_MIN_M = 0.0
REF_OFFSET_MAX_M = 1.0

# Measured desired->yaw lag minus the highway (lagd) lag, per speed.
DELAY_OFFSET_BP_MS = (5.0, 10.0, 14.0, 19.0, 25.0)
DELAY_OFFSET_S = (-0.13, -0.10, -0.08, -0.05, 0.0)

MIN_SPEED_MS = 3.0          # no correction at or below
FADE_IN_SPEED_MS = 4.0      # full correction from here
REF_SPEED_FLOOR_MS = 3.0    # offset / max(v, 3)
MAX_REDUCTION_S = 0.20
MIN_LOOKAHEAD_S = 0.20

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


def delay_offset_s(v_ego: float) -> float:
  """Low-speed lag correction (<= 0). 0 at >= 25 m/s."""
  return float(np.interp(float(v_ego), DELAY_OFFSET_BP_MS, DELAY_OFFSET_S))


def ref_offset_s(v_ego: float, ref_offset_m: float) -> float:
  """Rear reference offset as time: offset / max(v, 3)."""
  return clamp_ref_offset(ref_offset_m) / max(float(v_ego), REF_SPEED_FLOOR_MS)


def speed_fade(v_ego: float) -> float:
  return float(np.clip((float(v_ego) - MIN_SPEED_MS) / (FADE_IN_SPEED_MS - MIN_SPEED_MS), 0.0, 1.0))


def target_reduction_s(v_ego: float, ref_offset_m: float) -> float:
  """Seconds to take off the stock lookahead (>= 0, <= MAX_REDUCTION_S)."""
  v = max(0.0, float(v_ego))
  red = min(MAX_REDUCTION_S, max(0.0, -delay_offset_s(v) + ref_offset_s(v, ref_offset_m)))
  return red * speed_fade(v)


def apply_reduction(stock_lookahead_s: float, reduction_s: float) -> float:
  """Stock minus reduction, floored at MIN_LOOKAHEAD_S (never above stock)."""
  stock = float(stock_lookahead_s)
  red = min(MAX_REDUCTION_S, max(0.0, float(reduction_s)))
  if red <= 0.0:
    return stock
  return max(min(stock, MIN_LOOKAHEAD_S), stock - red)


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

  def reset(self) -> None:
    self.reduction_s = 0.0
    self._hold_s = 0.0
    self._out_prev = None
    self.rate_limited = False

  def update(self, *, enabled: bool, stock_lookahead_s: float, v_ego: float, ref_offset_m: float,
             lat_active: bool = True, cmd_angle_deg: float = 0.0, out_angle_deg: float | None = None) -> float:
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
    target = 0.0 if self._hold_s > 0.0 else target_reduction_s(v_ego, ref_offset_m)
    step = REDUCTION_SLEW_S_PER_S * self.dt
    self.reduction_s += float(np.clip(target - self.reduction_s, -step, step))
    return apply_reduction(stock_lookahead_s, self.reduction_s)


def turn_geometry_active(is_preap: bool, param_on: bool) -> bool:
  return bool(is_preap) and bool(param_on)


def is_preap_car(CP) -> bool:
  return getattr(CP, "brand", "") == "tesla" and getattr(CP, "carFingerprint", "") == "TESLA_MODEL_S_PREAP"
