"""Ring hold for the Roundabout Steering Assist (Pre-AP, NAPRoundaboutAssist, default Off).

Oct 1 pm (0000013c, 9819620): the model's curvature fell to ~0 at the ring edge (it follows the exit branch, not the ring) and the
latched cap (1.15x the LIVE model) pinned the assist to it; steeringPressed flicker zeroed the weight 45% of the time. Pure functions
used by roundabout_guide.RoundaboutGuide (curvature + = right; sense = +1 CCW, circulation side is -sense * k):

  * widen_cap: model circulation-side curvature below CIRCLE_CAP_FRAC x the circle's -> cap widens toward the circle target, by <= CIRCLE_CAP_EXTRA
    over the model (still <= 2.2 m/s², 0.035 slew)
  * exit windows: EXIT_BEFORE_DEG before each exit branch (to EXIT_AFTER_DEG past it), no stalk -> output >= the circle target
  * a same-side add (any magnitude) does not drop the curl; an exit-side pull (>= 1.5 Nm) or a held stalk wins at once.
    steeringPressed with no same-side torque still debounces, and a short flicker keeps the correction state
  * early latch at the ring edge, release on a right blinker while on the exit arm
A right blinker only releases the guide to the model (stalk held, or blinker + exit arm); it never routes the path onto the exit.
"""
from __future__ import annotations

import math

CIRCLE_CAP_FRAC = 0.5        # model's circulation-side curvature < this x the circle's: the cap widens toward the circle target
CIRCLE_CAP_EXTRA = 0.015     # 1/m: ...by at most this much over the model (R128: GNSS reads the car 2-3 m outside; uncapped it rides the island curb)
EXIT_BEFORE_DEG = 20.0       # exit window: this far before an exit branch (along the circulation) ...
EXIT_AFTER_DEG = 5.0         # ... to this far past it (the mouth of the exit arm)
EXIT_ARM_NEAR_DEG = 30.0     # blinker release: within this of an exit branch ...
EXIT_ARM_EDGE_M = 3.0        # ... and no more than this far inside the outer edge of the ring band
EXIT_ON_RING_M = 6.0         # an approach end this close to the ring radius touches the ring
EXIT_MERGE_DEG = 10.0        # branches closer than this are one exit
PRESS_DEBOUNCE_S = 0.3       # steeringPressed must hold this long before the assist yields
LIGHT_TORQUE_NM = 1.5        # |torque| below this toward the circulation is a resting hand, not a driver input
LATCH_ALIGN_EDGE_DEG = 45.0  # latch at the ring edge when within this of the tangent (was 25 deg: waited until tangent)
LATCH_EDGE_M = 1.0


def _wrap_deg(a: float) -> float:
  return (a + 180.0) % 360.0 - 180.0


def exit_angles(ring) -> list[float]:
  """Polar angles (deg, 0..360) of the exit branches: approach roads whose first point (travel direction) is on the ring."""
  out: list[float] = []
  if ring is None:
    return out
  for a in ring.approaches:
    if len(a) < 2:
      continue
    x0, y0 = a[0]
    x1, y1 = a[-1]
    if abs(math.hypot(x0, y0) - ring.radius_m) > EXIT_ON_RING_M or abs(math.hypot(x1, y1) - ring.radius_m) <= EXIT_ON_RING_M:
      continue
    deg = math.degrees(math.atan2(y0, x0)) % 360.0
    if all(abs(_wrap_deg(deg - o)) > EXIT_MERGE_DEG for o in out):
      out.append(deg)
  return sorted(out)


def ahead_deg(theta_rad: float, sense: float, exit_deg: float) -> float:
  """Degrees along the circulation from the car (polar angle theta) to the exit branch, in (-180, 180]."""
  return _wrap_deg(sense * (exit_deg - math.degrees(theta_rad)))


def in_exit_window(theta_rad: float, sense: float, exits: list[float]) -> bool:
  return any(-EXIT_AFTER_DEG <= ahead_deg(theta_rad, sense, e) <= EXIT_BEFORE_DEG for e in exits)


def near_exit(theta_rad: float, sense: float, exits: list[float]) -> bool:
  return any(abs(ahead_deg(theta_rad, sense, e)) <= EXIT_ARM_NEAR_DEG for e in exits)


def widen_cap(cap: float, s_model: float, s_circle: float, v: float, a_lat_max: float) -> float:
  """Latched circulation-side cap (-sense * k). Model under CIRCLE_CAP_FRAC x the circle: the cap widens toward the circle target, by up to
  CIRCLE_CAP_EXTRA over the model (full at a model of 0, fading to nothing at the half-circle point, so it never steps), <= a_lat_max at
  the actual speed. Otherwise unchanged (1.15 x the model)."""
  if s_circle <= 0.0 or s_model >= CIRCLE_CAP_FRAC * s_circle:
    return cap
  fade = min(max(1.0 - s_model / (CIRCLE_CAP_FRAC * s_circle), 0.0), 1.0)
  return max(cap, min(s_circle, s_model + CIRCLE_CAP_EXTRA * fade, a_lat_max / max(v * v, 1.0)))


def ring_limits(cap: float, floor: float, s_model: float, s_circle: float, v: float, a_lat_max: float,
                in_window: bool, plain: bool) -> tuple[float, float]:
  """(cap, floor) on the latched circulation-side curvature -s*k. `plain` (right stalk pending, off the ring band, over ring speed): the old 1.15x-model limits.
  Otherwise the cap widens to the circle when the model is under half of it, and inside an exit window (no stalk) the floor is the
  circle target, both bounded by a_lat_max at the actual speed; the cap never drops below the floor."""
  if plain:
    return cap, floor
  cap = widen_cap(cap, s_model, s_circle, v, a_lat_max)
  if in_window:
    floor = max(floor, min(s_circle, a_lat_max / max(v * v, 1.0)))
    cap = max(cap, floor)
  return cap, floor


def step_cap(extra: float, base: float, want_cap: float, v: float, a_lat_max: float, slew_dt: float, fall: float,
             plain: bool, holding: bool) -> tuple[float, float]:
  """(extra, cap): the widening over the 1.15x cap `base`. It rises at once and is given up at `fall` x the slew (at the full slew when
  `plain`, over the ring speed, or past a_lat_max at the actual speed); the cap never passes a_lat_max at today's speed via the widening."""
  if holding:
    return 0.0, base
  want = max(want_cap - base, 0.0)
  a_cap = a_lat_max / max(v * v, 1.0)
  rate = 1.0 if plain or base + extra > a_cap else fall
  extra = want if want >= extra else max(want, extra - rate * slew_dt)
  return extra, base + min(extra, max(a_cap - base, 0.0))


def on_exit_arm(theta_rad: float, sense: float, exits: list[float], d_edge: float) -> bool:
  """Near an exit branch and at the outer edge of the ring band or beyond: the car is on (or entering) the exit arm."""
  return near_exit(theta_rad, sense, exits) and d_edge >= -EXIT_ARM_EDGE_M


def counts_as_press(pressed: bool, torque: float, sense: float) -> bool:
  """steeringPressed that is not torque toward the circulation.

  A same-side add (sense * torque > 0), light or hard, is the driver
  helping the ring and must not count. Zero torque and an exit-side press do.
  """
  return bool(pressed) and not (sense * torque > 0.0)


def driver_wins(stalk_held: bool, pressed: bool, torque: float, sense: float, drv_s: float, dt: float) -> tuple[bool, float]:
  """(weight 0 now?, debounce timer). At once: a held stalk, or an exit-side pull
  (-sense * torque >= LIGHT_TORQUE_NM). A same-side add never zeroes the curl.
  steeringPressed with no same-side torque still advances the timer (a flicker
  shorter than PRESS_DEBOUNCE_S changes nothing) but does not itself zero the weight."""
  exit_pull = (-sense * float(torque)) >= LIGHT_TORQUE_NM
  drv_s = drv_s + dt if counts_as_press(pressed, torque, sense) else 0.0
  return bool(stalk_held or exit_pull), drv_s


def latch_ready(align_deg: float, d_edge: float, tangent_deg: float) -> bool:
  """Tangent at the edge as before, or within LATCH_ALIGN_EDGE_DEG of it as the car reaches the ring edge."""
  return d_edge < LATCH_EDGE_M and align_deg < max(tangent_deg, LATCH_ALIGN_EDGE_DEG)
