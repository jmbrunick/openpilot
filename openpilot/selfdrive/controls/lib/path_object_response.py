"""Oct 14 path-object response (pathObstacleVisionNAP, radar AND vision agree).

Zones, measured from the edge of our predicted swept path (car half width
around the model path) to the near edge of the object:
  brake   gap <= BRAKE_GAP_M (~1 ft) or in the path -> strong brake up to the
          stack's max decel. LOG-ONLY for now (wouldBrake / decel).
  steer   beyond that but inside the existing chime zones (human / other
          ~3 ft past the 1.7 m path half, animal ~20 ft) -> gentle in-lane
          shift away, no braking.
Gates: radar and vision agree, >= 15 mph, gas not pressed, message fresh.
Frames: lateral is path-relative, +right. A positive shift moves right.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.path_obstacle import (
  ANIMAL_OUTER_M,
  HUMAN_OUTER_M,
)

MIN_SPEED_MPS = 6.7          # 15 mph
CAR_HALF_M = 1.0             # Model S body half width (0.98 m), our swept path edge
BRAKE_GAP_M = 0.3            # ~1 ft
MAX_RANGE_M = 90.0
STALE_S = 0.6
# Steer: gentle, in-lane.
AVOID_CLEAR_M = 1.2          # aim for this gap between our edge and the object
AVOID_CAP_M = 0.6            # in-lane cap
AVOID_RATE_MPS = 0.4
AVOID_RELEASE_S = 2.0
AVOID_RESUME_S = 1.5
AVOID_HOLD_S = 1.0           # keep the shift this long after the last fresh confirm
# Brake (log only).
STOP_MARGIN_M = 4.0
ACCEL_MIN_FALLBACK = -3.5    # longitudinal stack floor (ACCEL_MIN) when none is passed


@dataclass(frozen=True)
class Confirmed:
  object_class: str
  range_m: float
  lateral: float
  span_m: float
  v_rel: float


def _f(v, d=float("nan")) -> float:
  try:
    out = float(v)
  except (TypeError, ValueError):
    return d
  return out if math.isfinite(out) else d


def confirmed_from_msg(obs, age_s: float) -> Confirmed | None:
  """The vision-confirmed object, or None (not agreed, stale, or no geometry)."""
  if obs is None or not math.isfinite(age_s) or age_s > STALE_S:
    return None
  if not bool(getattr(obs, "agree", False)) or not bool(getattr(obs, "visionEvaluated", False)):
    return None
  rng, lat = _f(getattr(obs, "rangeM", None)), _f(getattr(obs, "lateral", None))
  if not (math.isfinite(rng) and math.isfinite(lat)) or rng <= 0.0 or rng > MAX_RANGE_M:
    return None
  cls = str(getattr(obs, "objectClass", "unknown"))
  return Confirmed(cls, rng, lat, max(0.0, _f(getattr(obs, "spanM", 0.0), 0.0)), _f(getattr(obs, "vRel", None), 0.0))


def edge_gap(obj: Confirmed) -> float:
  """Gap from our swept path edge to the object's near edge, m. <= 0 is in the path."""
  return abs(obj.lateral) - 0.5 * obj.span_m - CAR_HALF_M


def zone(obj: Confirmed | None) -> str:
  """'brake', 'steer' or ''."""
  if obj is None:
    return ""
  gap = edge_gap(obj)
  if gap <= BRAKE_GAP_M:
    return "brake"
  outer = ANIMAL_OUTER_M if obj.object_class == "animal" else HUMAN_OUTER_M
  if abs(obj.lateral) - 0.5 * obj.span_m <= outer:
    return "steer"
  return ""


def gates_ok(v_ego: float, gas_pressed: bool) -> bool:
  return math.isfinite(v_ego) and v_ego >= MIN_SPEED_MPS and not gas_pressed


def avoid_target(obj: Confirmed | None, lane_room: float | None = None) -> float:
  """Signed in-lane shift away from a steer-zone object, +right. 0 otherwise."""
  if zone(obj) != "steer":
    return 0.0
  need = AVOID_CLEAR_M - edge_gap(obj)
  if need <= 0.0:
    return 0.0
  mag = min(AVOID_CAP_M, need)
  if lane_room is not None and math.isfinite(lane_room):
    mag = min(mag, max(0.0, lane_room))
  # Object to the right (lateral > 0): move left (negative).
  return -math.copysign(mag, obj.lateral) if obj.lateral != 0.0 else 0.0


def brake_decel(obj: Confirmed | None, v_ego: float, accel_min: float = ACCEL_MIN_FALLBACK) -> float:
  """Decel (m/s^2, <= 0) to stop STOP_MARGIN_M short, clipped to the stack floor.

  Strong by design: never gentler than half the floor once in the brake zone.
  """
  if zone(obj) != "brake" or not math.isfinite(v_ego) or v_ego <= 0.0:
    return 0.0
  floor = min(-0.5, _f(accel_min, ACCEL_MIN_FALLBACK))
  closing = v_ego  # radar+vision confirmed objects are near-stationary on the ground
  room = max(0.5, obj.range_m - STOP_MARGIN_M)
  need = -(closing * closing) / (2.0 * room)
  return max(floor, min(need, 0.5 * floor))


def _smooth(t: float) -> float:
  t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
  return t * t * (3.0 - 2.0 * t)


class AvoidShift:
  """Rate-limited avoid shift with override drop, smooth resume and release."""

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.shift = 0.0
    self._target = 0.0
    self._hold_s = 0.0
    self._gain = 1.0
    self._release_from = 0.0
    self._release_s = 0.0
    self._releasing = False

  def update(self, obj: Confirmed | None, *, v_ego: float, gas_pressed: bool, steering_pressed: bool,
             lat_active: bool, dt: float, lane_room: float | None = None) -> float:
    """Applied shift, +right, after gain. 0 when nothing to do."""
    dt = max(0.0, min(0.2, _f(dt, 0.0)))
    if not gates_ok(_f(v_ego, 0.0), gas_pressed) or not lat_active:
      self.reset()
      self._gain = 0.0 if lat_active is False else self._gain
      return 0.0
    tgt = avoid_target(obj, lane_room)
    if tgt != 0.0:
      self._target = tgt
      self._hold_s = AVOID_HOLD_S
      self._releasing = False
    else:
      self._hold_s = max(0.0, self._hold_s - dt)
    if self._hold_s > 0.0:
      step = AVOID_RATE_MPS * dt
      if self.shift * self._target < 0.0:
        self.shift = 0.0
      self.shift += max(-step, min(step, self._target - self.shift))
    else:
      if abs(self.shift) > 1e-4 and not self._releasing:
        self._releasing, self._release_from, self._release_s = True, self.shift, 0.0
      if self._releasing:
        self._release_s += dt
        u = min(1.0, self._release_s / AVOID_RELEASE_S)
        self.shift = self._release_from * (1.0 - _smooth(u))
        if u >= 1.0:
          self._releasing, self.shift = False, 0.0
    if steering_pressed:
      self._gain = 0.0
    else:
      self._gain = min(1.0, self._gain + dt / AVOID_RESUME_S)
    return self.shift * _smooth(self._gain)
