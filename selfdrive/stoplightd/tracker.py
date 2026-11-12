"""Red -> green state machine for the stoplight chime. No numpy, no cereal.

Fed one list of lamps (detect.Lamp) per camera frame while the car is
stopped in Drive. The camera does not move while the car is stopped, so a
signal lamp keeps its pixel position. That is the whole trick:

  red track   a red lamp seen in >= RED_CONFIRM_FRAMES frames spread over
              >= RED_CONFIRM_S at one spot. Raises the red flag.
  red off     a confirmed red track not seen for >= RED_OFF_S.
  new green   a green lamp that was not there while that red was lit,
              seen in >= GREEN_CONFIRM_FRAMES frames, that came on within
              GREEN_AFTER_RED_S of the red going out, in the same signal
              head: straight below it (vertical head) or beside it
              (horizontal head).

Only that red-off + new-green pair dings, and only once per stop. A green
sign, another lane's green that stays green, a lead car's brake lights
going out, a right turn on red, or a digital billboard flipping colours
never pair up. The caller owns the gate (Drive, ~0 m/s) and calls reset()
on every un-gate, so the next stop starts clean.

Pixel constants are for the 3X road camera (1928 px wide). `scale` is
frame width / 1928 for other sensors.
"""
from __future__ import annotations

from dataclasses import dataclass, field

REF_WIDTH = 1928.0

RED_CONFIRM_FRAMES = 4
RED_CONFIRM_S = 0.6
RED_OFF_S = 0.35
RED_FLAG_HOLD_S = 1.2       # the red flag outlives a missed frame or two
# Mean |V-128| of a lamp's coloured pixels. Real lamps on Scallywag's drives:
# red 50..115 (dull daytime LEDs sit at 50..60), green 62..98. Green street
# signs sit at 45..58, so green needs the higher bar.
RED_MIN_CHROMA = 48.0
GREEN_MIN_CHROMA = 60.0
GREEN_CONFIRM_FRAMES = 2
GREEN_CONFIRM_S = 0.15
GREEN_AFTER_RED_S = 1.0     # the green comes on within this of the red going out
GREEN_PAIR_S = 2.5          # a red that went out longer ago than this is stale
TRACK_GATE_PX = 18.0        # min association radius, full-res px
TRACK_GATE_K = 0.6          # ... or this many lamp sizes
TRACK_DROP_S = 4.0
MIN_LAMP_PX = 8.0           # a 30 cm lamp ~100 m away
MAX_TRACKS = 32
# Same head. Scallywag's 24 real pairs: |dx| <= 5 px, dy 27..79 px, dy/size
# 0.5..2.6 (night halos inflate size). A digital billboard flipping red ->
# green content (route 000001f7, Nov 9) was 15 px sideways, 3 px down.
HEAD_DX_PX = 10.0
HEAD_DX_K = 0.5             # |dx| <= max(HEAD_DX_PX, this * red size)
HEAD_DY_PX = 20.0
HEAD_DY_MIN_K = 0.4         # dy >= max(HEAD_DY_PX, this * red size): green below red
HEAD_DY_MAX_K = 4.0         # dy <= this * red size + HEAD_DY_PX
SIDE_DY_PX = 8.0
SIDE_DY_K = 0.4
SIDE_DX_PX = 20.0
SIDE_DX_MIN_K = 1.2
SIDE_DX_MAX_K = 4.0


@dataclass
class Track:
  color: str
  x: float
  y: float
  size: float
  first_t: float
  last_t: float
  hits: int = 1
  confirmed_t: float | None = None

  def near(self, lamp, scale: float = 1.0) -> bool:
    gate = max(TRACK_GATE_PX * scale, TRACK_GATE_K * max(self.size, lamp.size))
    return abs(lamp.x - self.x) <= gate and abs(lamp.y - self.y) <= gate

  def absorb(self, lamp, t: float) -> None:
    a = 0.3
    self.x += a * (lamp.x - self.x)
    self.y += a * (lamp.y - self.y)
    self.size += a * (lamp.size - self.size)
    self.last_t = t
    self.hits += 1


@dataclass
class Output:
  red: bool = False             # a confirmed red is lit now (UI flag)
  green: bool = False           # red -> green seen this stop (UI flash)
  chime: bool = False           # True on the one update the ding fires
  red_count: int = 0            # red lamps that passed lamp_ok this frame
  green_count: int = 0
  red_x: float = -1.0           # full-res px of the strongest lit red track
  red_y: float = -1.0
  reason: str = ""


def lamp_ok(lamp, scale: float = 1.0) -> bool:
  if lamp.size < MIN_LAMP_PX * scale:
    return False
  if lamp.color == "red":
    return lamp.chroma >= RED_MIN_CHROMA
  return lamp.chroma >= GREEN_MIN_CHROMA


def same_head(red: Track, green: Track, scale: float = 1.0) -> bool:
  s = max(red.size, MIN_LAMP_PX * scale)
  dx = green.x - red.x
  dy = green.y - red.y
  if abs(dx) <= max(HEAD_DX_PX * scale, HEAD_DX_K * s) and \
     max(HEAD_DY_PX * scale, HEAD_DY_MIN_K * s) <= dy <= HEAD_DY_MAX_K * s + HEAD_DY_PX * scale:
    return True
  return abs(dy) <= max(SIDE_DY_PX * scale, SIDE_DY_K * s) and \
    max(SIDE_DX_PX * scale, SIDE_DX_MIN_K * s) <= abs(dx) <= SIDE_DX_MAX_K * s


@dataclass
class StoplightTracker:
  scale: float = 1.0
  reds: list[Track] = field(default_factory=list)
  greens: list[Track] = field(default_factory=list)
  chimed: bool = False
  green_t: float | None = None

  def reset(self) -> None:
    self.reds.clear()
    self.greens.clear()
    self.chimed = False
    self.green_t = None

  def _assoc(self, tracks: list[Track], lamps, t: float, color: str) -> None:
    for lamp in lamps:
      best = None
      best_d = 0.0
      for tr in tracks:
        if tr.last_t < t and tr.near(lamp, self.scale):
          d = abs(tr.x - lamp.x) + abs(tr.y - lamp.y)
          if best is None or d < best_d:
            best, best_d = tr, d
      if best is not None:
        best.absorb(lamp, t)
      elif len(tracks) < MAX_TRACKS:
        tracks.append(Track(color, lamp.x, lamp.y, lamp.size, t, t))
    tracks[:] = [tr for tr in tracks if t - tr.last_t <= TRACK_DROP_S]

  def update(self, t: float, lamps) -> Output:
    """One camera frame at monotonic time t (s). Lamps from detect.find_lamps."""
    reds = [lp for lp in lamps if lp.color == "red" and lamp_ok(lp, self.scale)]
    greens = [lp for lp in lamps if lp.color == "green" and lamp_ok(lp, self.scale)]
    self._assoc(self.reds, reds, t, "red")
    # Every green is tracked, including ones lit while a red is still lit
    # (a sign, another approach's signal). A tracked green is never "new".
    self._assoc(self.greens, greens, t, "green")

    for tr in self.reds:
      if tr.confirmed_t is None and tr.hits >= RED_CONFIRM_FRAMES and tr.last_t - tr.first_t >= RED_CONFIRM_S:
        tr.confirmed_t = t
    confirmed = [tr for tr in self.reds if tr.confirmed_t is not None]
    lit = [tr for tr in confirmed if t - tr.last_t <= RED_FLAG_HOLD_S]
    out = Output(red_count=len(reds), green_count=len(greens))
    if lit:
      out.red = True
      top = max(lit, key=lambda tr: tr.hits)
      out.red_x, out.red_y = top.x, top.y

    if not self.chimed:
      off = [tr for tr in confirmed if RED_OFF_S <= t - tr.last_t <= GREEN_PAIR_S + RED_OFF_S]
      for g in self.greens:
        if g.hits < GREEN_CONFIRM_FRAMES or g.last_t != t or g.last_t - g.first_t < GREEN_CONFIRM_S:
          continue
        for r in off:
          if not (r.last_t - 0.25 <= g.first_t <= r.last_t + GREEN_AFTER_RED_S):
            continue
          if same_head(r, g, self.scale):
            self.chimed = True
            self.green_t = t
            out.chime = True
            out.reason = f"red({r.x:.0f},{r.y:.0f})->green({g.x:.0f},{g.y:.0f})"
            break
        if self.chimed:
          break
    if self.chimed:
      out.green = True
      out.red = False
    return out
