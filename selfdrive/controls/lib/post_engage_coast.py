"""Hold last pressed pedal for ~1 s after Pre-AP long engage.

Justin: press gas, engage software long, lift — Pre-AP gas override hands
authority back with ``vdas.reset(commanded_accel=0)`` and a 0.5 s engage
grace that floors accel at 0. That is a *speed* / coast clamp: the
interceptor still ramps down from the pressed DI toward zero-torque, and
``get_accel_from_plan`` (``2*dv/t - a_now``) can invert leftover +aEgo
into a soft negative. Tesla passthrough regen (``enable=0``) already
pulls ``aEgo`` down on the way.

This overlay does **not** rewrite MAX / vCruise / sticky. After
``enableLongControl`` rises, watch the accelerator. As soon as the pedal
starts decreasing, freeze the last *pressed* position (and the last
non-negative ``aEgo`` it was producing) for the rest of the 1 s window.
The carcontroller rewrite holds ``GAS_COMMAND`` at that DI once panda
allows ``ENABLE=1`` (not while the driver is still on the gas).

Leave these paths alone:
- brake pedal (digital Applied — Pre-AP forces ``brakePressed`` false)
- FCW / should-stop / AEB-class
- a radar lead already asking ``LEAD_KEEP_DECEL_MS2`` (0.55) or harder
- after the 1 s window (normal OP long toward MAX / lead)
- engage with no gas / pedal never decreases (no hold)
"""
from __future__ import annotations

# Full second after software-long engage. Pedal-layer engage grace is 0.5 s
# and only starts on longActive rising (gas release). The hold window is
# the engage edge; the freeze is the first lift inside it.
POST_ENGAGE_COAST_S = 1.0

# Lead-approach comfort peak. A lead already asking this (or harder)
# is not the throttle-lift dip — keep it.
LEAD_KEEP_DECEL_MS2 = 0.55

# MPC / AEB-class. Passed through only with a safety flag (FCW, stop,
# brake, or a lead already at/above this). A lone −1.2 from
# get_accel_from_plan inversion is still held.
HARD_DECEL_MS2 = 1.0

# Comma Pedal / DI_pedalPos units. Match opendbc PEDAL_DI_PRESSED.
PEDAL_PRESSED_DI = 2.0
# Lift vs interceptor / DI noise. ~0.4 DI is a real decrease, not chatter.
PEDAL_DROP_DI = 0.40
# When only gasPressed is available (planner has not seen analog DI yet).
BINARY_PRESSED_DI = 8.0

# Sticky / cruise MAX is a hard cap. Holding a pressed pedal must not
# keep accelerating once ego is at/above vCruise.
MAX_HOLD_SLACK_MS = 0.15


def cs_pedal_di(CS, gas_pressed: bool | None = None) -> float:
  """Measured or last-known accelerator DI. Binary fallback if analog is 0."""
  for name in ("pedal_interceptor_value", "pedalCommandDi", "pedal_command_di"):
    val = getattr(CS, name, None)
    if val is not None:
      try:
        di = float(val)
      except (TypeError, ValueError):
        continue
      if di > 0.0:
        return di
  pressed = bool(getattr(CS, "gasPressed", False) if gas_pressed is None else gas_pressed)
  return BINARY_PRESSED_DI if pressed else 0.0


class PostEngageCoast:
  """Frame-by-frame 1 s post-engage pedal / accel hold."""

  def __init__(self, dt: float):
    self.dt = float(dt)
    self._remain_s = 0.0
    self._prev_long = False
    self._last_pressed_di = 0.0
    self._last_good_a = 0.0
    self._saw_pressed = False
    self._holding = False
    self._hold_di = 0.0
    self._hold_a = 0.0

  @property
  def active(self) -> bool:
    """Window still open *and* a lift has frozen a pressed pedal."""
    return self._remain_s > 0.0 and self._holding

  @property
  def hold_pedal(self) -> float | None:
    return self._hold_di if self.active else None

  @property
  def hold_accel(self) -> float:
    return self._hold_a if self.active else 0.0

  def reset(self) -> None:
    self._remain_s = 0.0
    self._prev_long = False
    self._last_pressed_di = 0.0
    self._last_good_a = 0.0
    self._saw_pressed = False
    self._holding = False
    self._hold_di = 0.0
    self._hold_a = 0.0

  def update(self, *, long_engaged: bool, gas_pressed: bool = False,
             pedal_pos: float | None = None, a_ego: float = 0.0,
             dt: float | None = None) -> None:
    step = self.dt if dt is None else float(dt)
    if not long_engaged:
      self.reset()
      return

    if pedal_pos is None:
      di = BINARY_PRESSED_DI if gas_pressed else 0.0
    else:
      di = float(pedal_pos)
      if di <= 0.0 and gas_pressed:
        di = BINARY_PRESSED_DI

    if not self._prev_long:
      self._remain_s = POST_ENGAGE_COAST_S
      self._holding = False
      self._hold_di = 0.0
      self._hold_a = 0.0
      self._last_pressed_di = di if di > PEDAL_PRESSED_DI else 0.0
      self._saw_pressed = self._last_pressed_di > PEDAL_PRESSED_DI
      self._last_good_a = max(float(a_ego), 0.0) if self._saw_pressed else 0.0
    elif self._remain_s > 0.0:
      self._remain_s = max(0.0, self._remain_s - step)
      if self._remain_s <= 0.0:
        self._holding = False
        self._saw_pressed = False
      else:
        self._track_pedal(di, float(a_ego))

    self._prev_long = True

  def _track_pedal(self, di: float, a_ego: float) -> None:
    lifting = self._saw_pressed and di <= self._last_pressed_di - PEDAL_DROP_DI
    if di > PEDAL_PRESSED_DI:
      self._saw_pressed = True
      # Do not recapture aEgo on the lift frame — that sample is already
      # the regen hole. Keep the last non-negative a from the pressed peak.
      if a_ego >= 0.0 and not lifting:
        self._last_good_a = a_ego
      if (not self._holding) or di > self._hold_di + PEDAL_DROP_DI:
        if di + 1e-9 >= self._last_pressed_di:
          self._last_pressed_di = di
        if self._holding and di > self._hold_di + PEDAL_DROP_DI:
          # Re-press inside the window: recapture so the next lift
          # holds the new peak, not the earlier (lower) freeze.
          self._holding = False
          self._last_pressed_di = di

    if not self._saw_pressed or self._holding:
      return
    if self._last_pressed_di <= PEDAL_PRESSED_DI:
      return
    if di <= self._last_pressed_di - PEDAL_DROP_DI:
      # Last pressed / start of decrease — not the already-dipped sample.
      self._hold_di = self._last_pressed_di
      self._hold_a = self._last_good_a
      self._holding = True

  def at_or_above_max(self, v_ego, v_cruise) -> bool:
    """True when a pedal hold would push past sticky/cruise MAX."""
    if v_ego is None or v_cruise is None:
      return False
    try:
      ego = float(v_ego)
      cruise = float(v_cruise)
    except (TypeError, ValueError):
      return False
    if cruise <= 0.0:
      return False
    return ego >= cruise - MAX_HOLD_SLACK_MS

  def safety_overrides(self, a_cmd, *, brake_pressed: bool = False,
                       fcw: bool = False, should_stop: bool = False,
                       has_lead: bool = False, v_ego=None, v_cruise=None) -> bool:
    """True when a real hazard or MAX cap must win over the pedal freeze."""
    a = float(a_cmd)
    if brake_pressed or fcw or should_stop:
      return True
    if has_lead and a <= -LEAD_KEEP_DECEL_MS2:
      return True
    if self.at_or_above_max(v_ego, v_cruise):
      return True
    return False

  def apply(self, a_cmd, *, brake_pressed: bool = False, fcw: bool = False,
            should_stop: bool = False, has_lead: bool = False,
            v_ego=None, v_cruise=None) -> float:
    """Hold last pressed accel; pass safety, MAX, and unused window through."""
    a = float(a_cmd)
    if not self.active:
      return a
    if self.safety_overrides(a, brake_pressed=brake_pressed, fcw=fcw,
                             should_stop=should_stop, has_lead=has_lead,
                             v_ego=v_ego, v_cruise=v_cruise):
      return a
    return self._hold_a

  def should_hold_pedal(self, a_cmd, *, brake_pressed: bool = False,
                        fcw: bool = False, should_stop: bool = False,
                        has_lead: bool = False, v_ego=None, v_cruise=None) -> bool:
    """Carcontroller: freeze GAS_COMMAND only when apply() would hold."""
    if not self.active or self.hold_pedal is None:
      return False
    return not self.safety_overrides(
      a_cmd, brake_pressed=brake_pressed, fcw=fcw,
      should_stop=should_stop, has_lead=has_lead,
      v_ego=v_ego, v_cruise=v_cruise,
    )
