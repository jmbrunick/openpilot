"""Crossing and turning vehicles in the longitudinal stack.

Radar sees a side-on car as a glint on the body panel that is square to
the bumper. While that panel covers our lane the point sits near the
centreline, so the lead looks stopped. The motion shows up at the
corners: an entry sweep when the nose reaches us, and an exit sweep
when the tail starts to leave.

A return moving sideways in the ground frame, faster than about 2 mph
after ego yaw is removed, is a crosser. The follow law then sees a
smaller closing speed, so it eases off instead of stopping for a parked
car. The weight on that input is filtered. When the tail's exit sweep
begins, the same weight hands the command back to the speed-ceiling
term and the car starts accelerating again. That handover runs only
while longitudinal is still on. A gas touch clears it until the stalk,
and lifting off the pedal does not bring it back.

A stopped car, a return the model still calls a lead, and anything
close enough that we could not stop stays on the normal law.
"""
from __future__ import annotations

import math
from collections import deque

# About 2 mph. E1's entry sweep was 1.5–1.6 m/s. A stopped car's
# ground-frame wander over a couple of seconds stays under this.
CROSS_VY_MS = 0.89
# Net ground-frame shift in the first moments, same direction as the sweep.
CROSS_DY_M = 0.8
CROSS_AGE_S = 0.8
CROSS_ENTRY_S = 1.5
# Not traffic. A lead still moving along the road is not crossing.
CROSS_ALONG_MS = 3.0
CROSS_ALONG_DROP_MS = 3.5
# The radar fit already covers this much motion before it publishes a speed.
CROSS_FIT_MIN_S = 0.6
CROSS_FIT_WINDOW_S = 1.0

# Tail starts to leave. The entry sweep is ignored: the point has to
# come back toward the lane (the glint) and then move out again, with
# a real lateral speed, and stay there briefly so a wobble does not.
CROSS_EXIT_DY_M = 0.35
CROSS_EXIT_VY_MS = 0.40
CROSS_EXIT_HOLD_S = 0.12
CROSS_COMEBACK_M = 0.30
CROSS_Y_TAU_S = 0.08

# Model still says this is a lead in our lane: it is stopped, not crossing.
CROSS_VISION_RANGE_M = 55.0
CROSS_VISION_PROB = 0.7

# Still overlapping the lane and too close to keep easing off.
CROSS_IMMINENT_GAP_M = 10.0
CROSS_LANE_HALF_M = 1.6

# Share of the real closing speed the follow law keeps seeing while the
# crosser is in the lane. The rest is filtered out so the stop term does
# not run. The fraction itself is not a second command.
CROSS_CLOSE_FRAC = 0.28
CROSS_GENTLE_TAU_S = 0.25
CROSS_RESUME_TAU_S = 0.12

# Stationary-target room cap (X2). Fast closes shorten the match distance
# to about 0.38 of the gap; a stopped car at long range was then asked for
# about twice the decel a stop needs. D stays at least three quarters of
# the gap beyond the standstill keep.
X2_V_LEAD_MS = 2.0
X2_ROOM_FRAC = 0.75


def crossing_may_resume(long_on: bool) -> bool:
  """Re-accelerate only while longitudinal is still engaged.

  Any gas touch drops longitudinal until the driver pulls the stalk.
  Letting the pedal up does not turn it back on, including when the
  crosser is already leaving.
  """
  return bool(long_on)


def _filt(prev: float | None, sample: float, dt: float, tau: float) -> float:
  if prev is None or tau <= 1e-4:
    return float(sample)
  a = float(dt) / (float(tau) + float(dt))
  return float(prev) + a * (float(sample) - float(prev))


def _slope(samples: deque, now: float) -> float:
  """Least-squares lateral speed over the recent window. 0 until it spans the fit."""
  pts = [p for p in samples if now - p[0] <= CROSS_FIT_WINDOW_S + 1e-9]
  if len(pts) < 3:
    return 0.0
  span = pts[-1][0] - pts[0][0]
  if span < CROSS_FIT_MIN_S:
    return 0.0
  n = float(len(pts))
  t_mean = sum(p[0] for p in pts) / n
  y_mean = sum(p[1] for p in pts) / n
  var_t = sum((p[0] - t_mean) ** 2 for p in pts)
  if var_t < 1e-8:
    return 0.0
  return sum((p[0] - t_mean) * (p[1] - y_mean) for p in pts) / var_t


class GroundLateral:
  """Ground-frame lateral speed of radar returns, ego yaw removed.

  y_rel and the published speed are +left, metres and m/s. yaw_rate_left
  is +left, rad/s. The radar's own lateral velocity is not used: at range
  it is mostly our yaw, and on a side-on car it tracks the glint.
  """

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self._t: float | None = None
    self._heading = 0.0
    self._x = 0.0
    self._y = 0.0
    self._tracks: dict[int, deque] = {}

  def update(self, t: float, v_ego: float, yaw_rate_left: float,
             points: list[tuple[int, float, float]]) -> dict[int, float]:
    """points are (track_id, d_rel, y_rel). Returns track_id -> v_lat."""
    now = float(t)
    if self._t is not None:
      dt = now - self._t
      if dt < 0.0 or dt > 0.5:
        self.reset()
      elif dt > 0.0:
        self._heading += float(yaw_rate_left) * dt
        c = math.cos(self._heading)
        s = math.sin(self._heading)
        self._x += float(v_ego) * c * dt
        self._y += float(v_ego) * s * dt
    self._t = now
    c = math.cos(self._heading)
    s = math.sin(self._heading)
    live: dict[int, float] = {}
    seen: set[int] = set()
    for track_id, d_rel, y_rel in points:
      tid = int(track_id)
      seen.add(tid)
      world_y = self._y + float(d_rel) * s + float(y_rel) * c
      hist = self._tracks.get(tid)
      if hist is None:
        hist = deque(maxlen=48)
        self._tracks[tid] = hist
      hist.append((now, world_y))
      live[tid] = _slope(hist, now)
    for tid in list(self._tracks):
      if tid not in seen:
        self._tracks.pop(tid, None)
    return live


class CrossingFollow:
  """Smoothed inputs that tell the follow law a crosser will clear.

  close_scale is the fraction of the real closing speed the law should
  see. 1 is the normal law. It falls once the entry sweep qualifies and
  rises again for a stopped car, a vision lead, or an imminent overlap.
  resume_w rises when the exit sweep begins and longitudinal is still
  on, and the controller then blends toward the speed-ceiling command.
  """

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.lead_id = None
    self.age = 0.0
    self.dy = 0.0
    self.credited = False
    self.speed_ok = False
    self.entry_closed = False
    self.classified = False
    self.direction = 0.0
    self.exit = False
    self.exit_s = 0.0
    self.imminent = False
    self.y_f: float | None = None
    self.peak_y: float | None = None
    self.came_back = False
    self.lock_y: float | None = None
    self.gentle_w = 0.0
    self.resume_w = 0.0
    self._along_s = 0.0

  def update(self, *, dt: float, lead_id, gap: float, y_rel: float, v_lead: float,
             v_lat: float, model_prob: float | None, long_on: bool) -> None:
    frame = 0.05 if dt is None or float(dt) <= 1e-6 else float(dt)
    if lead_id != self.lead_id:
      self.reset()
      self.lead_id = lead_id
    self.age += frame
    vlat = float(v_lat)
    along = abs(float(v_lead))
    if not self.credited and abs(vlat) >= CROSS_VY_MS:
      # The published speed is already a fit over the window, so the
      # motion that produced it counts. Later frames add themselves.
      self.dy += vlat * CROSS_FIT_MIN_S
      self.credited = True
    self.dy += vlat * frame
    if (not self.classified) and abs(vlat) >= CROSS_VY_MS and vlat * self.dy >= 0.0:
      self.speed_ok = True

    prob = 0.0 if model_prob is None or not math.isfinite(float(model_prob)) else float(model_prob)
    vision_stopped = float(gap) <= CROSS_VISION_RANGE_M and prob >= CROSS_VISION_PROB
    if along >= CROSS_ALONG_DROP_MS:
      self._along_s += frame
    else:
      self._along_s = 0.0

    if not self.classified and not self.entry_closed:
      if self.age > CROSS_ENTRY_S:
        self.entry_closed = True
      elif (self.age + 1e-9 >= CROSS_AGE_S and abs(self.dy) >= CROSS_DY_M
            and self.speed_ok and along < CROSS_ALONG_MS and not vision_stopped):
        self.classified = True
        self.direction = 1.0 if self.dy >= 0.0 else -1.0

    if self.classified and (vision_stopped or self._along_s >= 0.30):
      # It is traffic, or the model sees a lead where the glint sits.
      # Back to the normal law. The exit handover does not fire.
      self.classified = False
      self.exit = False
      self.exit_s = 0.0

    prev_y = self.y_f
    self.y_f = _filt(self.y_f, float(y_rel), frame, CROSS_Y_TAU_S)
    vy = 0.0 if prev_y is None else (float(self.y_f) - float(prev_y)) / frame
    if self.classified and not self.exit and self.direction != 0.0:
      y = float(self.y_f)
      # The nose sweep is not the tail. Remember how far out the point
      # got, and only look for the exit after it has come back in.
      if self.peak_y is None or y * self.direction > self.peak_y * self.direction:
        self.peak_y = y
      elif self.peak_y is not None and (self.peak_y - y) * self.direction >= CROSS_COMEBACK_M:
        self.came_back = True
      if self.came_back:
        if self.lock_y is None or y * self.direction < self.lock_y * self.direction:
          self.lock_y = y
          self.exit_s = 0.0
        moved = 0.0 if self.lock_y is None else (y - self.lock_y) * self.direction
        if moved >= CROSS_EXIT_DY_M and vy * self.direction >= CROSS_EXIT_VY_MS:
          self.exit_s += frame
        else:
          self.exit_s = 0.0
        if self.exit_s + 1e-9 >= CROSS_EXIT_HOLD_S:
          self.exit = True

    y_abs = abs(float(y_rel))
    self.imminent = bool(
      self.classified and not self.exit
      and float(gap) <= CROSS_IMMINENT_GAP_M
      and y_abs <= CROSS_LANE_HALF_M
    )

    gentle_target = 1.0 if self.classified and not self.imminent else 0.0
    resume_target = 1.0 if self.exit and self.classified and crossing_may_resume(long_on) else 0.0
    self.gentle_w = _filt(self.gentle_w, gentle_target, frame, CROSS_GENTLE_TAU_S)
    self.resume_w = _filt(self.resume_w, resume_target, frame, CROSS_RESUME_TAU_S)

  @property
  def close_scale(self) -> float:
    """1 = full closing speed. Lower while a crosser is still in the lane."""
    ease = self.gentle_w * (1.0 - CROSS_CLOSE_FRAC)
    scale = (1.0 - ease) * (1.0 - self.resume_w)
    if scale < 0.0:
      return 0.0
    if scale > 1.0:
      return 1.0
    return float(scale)
