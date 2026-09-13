"""Hold speed for ~1 s after Pre-AP long engage if the accelerator just came off.

Justin: press gas, engage software long, lift — the car often regens / dips
before the planner climbs toward a higher MAX. Two stacked causes:

1. Gas override sets ``longActive`` false. The planner resets every frame
   from live ``aEgo`` (often +1 m/s²+). ``get_accel_from_plan`` then does
   ``2*dv/t - a_now``; a still-high ``a_now`` with only a small climb
   inverts to a soft negative command.
2. Tesla regen on a passthrough pedal (enable=0) already pulls ``aEgo``
   negative on lift. The next reset seeds MPC from that hole, and jerk
   limits keep commanded ``a`` negative while it climbs back through 0.

This overlay does **not** rewrite MAX / vCruise / sticky. It zeros *soft*
negative accel for one second after ``enableLongControl`` rises, but only
if the accelerator was down during that window and then came off.

Leave these paths alone:
- brake pedal (digital Applied — Pre-AP forces ``brakePressed`` false)
- FCW / should-stop / AEB-class
- any valid radar lead that is already asking to slow
- a lead already asking ``LEAD_KEEP_DECEL_MS2`` (0.55) or harder
- after the 1 s window (normal regen toward MAX / lead)
- engage with no gas in the window (no clamp)
"""
from __future__ import annotations

# Full second after software-long engage. Pedal-layer engage grace is 0.5 s
# and only starts on longActive rising (gas release), which is too late /
# too short for the planner inversion.
POST_ENGAGE_COAST_S = 1.0

# Lead-approach comfort peak. A lead already asking this (or harder)
# is not the throttle-lift dip — keep it. Softer −a with only a far
# `leadOne.status` is still the inversion and must be zeroed.
LEAD_KEEP_DECEL_MS2 = 0.55

# MPC / AEB-class. Passed through only with a safety flag (FCW, stop,
# brake, or a lead already at/above this). A lone −1.2 from
# get_accel_from_plan inversion is still clamped.
HARD_DECEL_MS2 = 1.0


class PostEngageCoast:
  """Frame-by-frame 1 s post-engage coast / hold."""

  def __init__(self, dt: float):
    self.dt = float(dt)
    self._remain_s = 0.0
    self._gas_in_window = False
    self._prev_long = False

  @property
  def active(self) -> bool:
    """Window still open *and* the accelerator was down in it."""
    return self._remain_s > 0.0 and self._gas_in_window

  def reset(self) -> None:
    self._remain_s = 0.0
    self._gas_in_window = False
    self._prev_long = False

  def update(self, *, long_engaged: bool, gas_pressed: bool, dt: float | None = None) -> None:
    step = self.dt if dt is None else float(dt)
    if not long_engaged:
      self.reset()
      return

    if not self._prev_long:
      self._remain_s = POST_ENGAGE_COAST_S
      self._gas_in_window = bool(gas_pressed)
    elif self._remain_s > 0.0:
      self._remain_s = max(0.0, self._remain_s - step)
      if gas_pressed:
        self._gas_in_window = True
      if self._remain_s <= 0.0:
        self._gas_in_window = False

    self._prev_long = True

  def apply(self, a_cmd, *, brake_pressed: bool = False, fcw: bool = False,
            should_stop: bool = False, has_lead: bool = False) -> float:
    """Zero soft negative accel; pass positives and safety through."""
    a = float(a_cmd)
    if not self.active:
      return a
    if brake_pressed or fcw or should_stop:
      return a
    if a >= 0.0:
      return a
    if has_lead and a <= -LEAD_KEEP_DECEL_MS2:
      return a
    return 0.0
