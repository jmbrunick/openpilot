"""Climb toward MAX on the first pedal drop after Pre-AP long engage.

Justin: press gas, engage software long, lift — Pre-AP gas override hands
authority back with ``vdas.reset(commanded_accel=0)`` and a 0.5 s engage
grace that floors accel at 0. That is a sit / coast: Tesla passthrough
regen (``enable=0``) already pulls ``aEgo`` down on the way, then grace
keeps commanded accel at 0 before OP long climbs to MAX.

On the **first detectable pedal decrease** after long engage, take over
with **positive accel toward MAX** and **keep that climb** until ego is
at MAX (or brake / FCW / should-stop / hard lead). The same latch covers
**every later gas-override → OP long handoff** while software long stays
on (not a one-time post-engage window). Do not freeze last-pressed DI
on the wire (a peak-DI stab retriggers interceptor ``gasPressed`` →
ENABLE 0↔1 → ACQUIRE wipe → pulse / net decel). Planner +a that is
already climbing harder passes through. MAX remains a hard cap.

Leave these paths alone:
- brake pedal (digital Applied — Pre-AP forces ``brakePressed`` false)
- FCW / should-stop / AEB-class
- a radar lead already asking ``LEAD_KEEP_DECEL_MS2`` (0.55) or harder
- at/above sticky MAX (normal OP long owns the cap)
- engage with no gas / pedal never decreases (no overlay)
"""
from __future__ import annotations

# Legacy 1 s window from #144. Climb is now latched until MAX / safety —
# expiring at 1 s dropped the floor while ACQUIRE/VDAS were still in the
# regen hole (38→28 mph). Kept so docs/tests can name the old contract.
POST_ENGAGE_COAST_S = 1.0

# Lead-approach comfort peak. A lead already asking this (or harder)
# is not the throttle-lift dip — keep it.
LEAD_KEEP_DECEL_MS2 = 0.55

# MPC / AEB-class. Passed through only with a safety flag (FCW, stop,
# brake, or a lead already at/above this). A lone −1.2 from
# get_accel_from_plan inversion is still replaced by the climb.
HARD_DECEL_MS2 = 1.0

# Comma Pedal / DI_pedalPos units. Match opendbc PEDAL_DI_PRESSED.
PEDAL_PRESSED_DI = 2.0
# First detectable decrease. 0.40 left the early drop unprotected
# (ENABLE=0 passthrough + Tesla regen) before the overlay latched.
PEDAL_DROP_DI = 0.05
# Real driver re-press while climbing. 0.05 treated interceptor chatter
# and ENABLE=1 command echo as a new press → unlatch / re-latch pulse.
REPRESS_DI = 2.0
# When only gasPressed is available (planner has not seen analog DI yet).
BINARY_PRESSED_DI = 8.0

# Minimum +a toward MAX while the handoff is active. last +aEgo from the
# pressed peak is used when it is higher; planner +a may still exceed this.
CLIMB_FLOOR_MS2 = 0.40

# Sticky / cruise MAX is a hard cap. The climb must not push past vCruise.
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


def cs_driver_pedal_di(CS, gas_pressed: bool | None = None) -> float:
  """Driver / interceptor DI only — never the ENABLE=1 command echo.

  ``cs_pedal_di`` prefers ``pedal_command_di``. After a climb seed that
  feedback looks like a still-pressed pedal and re-arms the overlay.
  """
  val = getattr(CS, "pedal_interceptor_value", None)
  if val is not None:
    try:
      di = float(val)
    except (TypeError, ValueError):
      di = 0.0
    if di > 0.0:
      return di
  pressed = bool(getattr(CS, "gasPressed", False) if gas_pressed is None else gas_pressed)
  return BINARY_PRESSED_DI if pressed else 0.0


def cs_lift_pedal_di(CS, gas_pressed: bool | None = None) -> float:
  """Pedal for lift detection: interceptor when present, else analog fallback."""
  if getattr(CS, "pedal_interceptor_value", None) is not None:
    return cs_driver_pedal_di(CS, gas_pressed=gas_pressed)
  return cs_pedal_di(CS, gas_pressed=gas_pressed)


class PostEngageCoast:
  """Frame-by-frame post-engage climb-to-MAX handoff on first pedal drop."""

  def __init__(self, dt: float):
    self.dt = float(dt)
    self._prev_long = False
    self._last_pressed_di = 0.0
    self._last_good_a = 0.0
    self._saw_pressed = False
    self._holding = False
    self._hold_di = 0.0
    self._hold_a = 0.0
    self._prev_gas = False

  @property
  def active(self) -> bool:
    """A lift has latched the climb (stays until MAX / safety / long off)."""
    return self._holding

  @property
  def hold_pedal(self) -> float | None:
    """Last pressed DI — diagnostics only, never a GAS_COMMAND stab."""
    return self._hold_di if self.active else None

  @property
  def hold_accel(self) -> float:
    """Minimum +a toward MAX while the handoff is active."""
    return self._hold_a if self.active else 0.0

  def reset(self) -> None:
    self._prev_long = False
    self._last_pressed_di = 0.0
    self._last_good_a = 0.0
    self._saw_pressed = False
    self._holding = False
    self._hold_di = 0.0
    self._hold_a = 0.0
    self._prev_gas = False

  def update(self, *, long_engaged: bool, gas_pressed: bool = False,
             pedal_pos: float | None = None, a_ego: float = 0.0,
             dt: float | None = None) -> None:
    if dt is not None:
      self.dt = float(dt)
    if not long_engaged:
      self.reset()
      return

    if pedal_pos is None:
      di = BINARY_PRESSED_DI if gas_pressed else 0.0
    else:
      di = float(pedal_pos)
      if di <= 0.0 and gas_pressed:
        di = BINARY_PRESSED_DI

    gas = bool(gas_pressed)
    gas_falling = self._prev_gas and not gas

    if not self._prev_long:
      # Watch from engage. Do not start a 1 s timer — holding the pedal
      # past 1 s must still hand off, and the climb must last until MAX.
      self._holding = False
      self._hold_di = 0.0
      self._hold_a = 0.0
      self._last_pressed_di = di if di > PEDAL_PRESSED_DI else 0.0
      self._saw_pressed = self._last_pressed_di > PEDAL_PRESSED_DI
      self._last_good_a = max(float(a_ego), 0.0) if self._saw_pressed else 0.0
    else:
      self._track_pedal(di, float(a_ego), gas, gas_falling)

    self._prev_long = True
    self._prev_gas = gas

  def _climb_floor(self) -> float:
    return max(float(self._last_good_a), CLIMB_FLOOR_MS2)

  def _latch_climb(self) -> None:
    # Remember the last pressed peak for re-press detection only.
    # apply() floors accel; the carcontroller must not put this DI on
    # GAS_COMMAND (peak stab → interceptor gasPressed → ENABLE chatter).
    self._hold_di = self._last_pressed_di
    self._hold_a = self._climb_floor()
    self._holding = True

  def _track_pedal(self, di: float, a_ego: float, gas_pressed: bool,
                   gas_falling: bool) -> None:
    # Once latched this long session, stay latched until MAX / long off.
    # Unlatching on a re-press re-armed on every gasPressed falling edge
    # (manual accel → OP long) and pulsed again.
    if self._holding:
      return

    lifting = self._saw_pressed and (
      gas_falling or di <= self._last_pressed_di - PEDAL_DROP_DI)
    if di > PEDAL_PRESSED_DI:
      self._saw_pressed = True
      # Do not recapture aEgo on the lift / gas-falling frame — that
      # sample is already the regen hole. Keep the last non-negative a.
      if a_ego >= 0.0 and not lifting:
        self._last_good_a = a_ego
      if di + 1e-9 >= self._last_pressed_di:
        self._last_pressed_di = di

    if not self._saw_pressed:
      return
    if self._last_pressed_di <= PEDAL_PRESSED_DI:
      return
    # Analog drop *or* gasPressed falling edge (override → OP long).
    if gas_falling or di <= self._last_pressed_di - PEDAL_DROP_DI:
      self._latch_climb()

  def at_or_above_max(self, v_ego, v_cruise) -> bool:
    """True when a climb would push past sticky/cruise MAX."""
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
    """True when a real hazard or MAX cap must win over the climb."""
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
    """Floor coast/regen to a climb toward MAX; pass safety and higher +a."""
    a = float(a_cmd)
    if not self.active:
      return a
    if self.safety_overrides(a, brake_pressed=brake_pressed, fcw=fcw,
                             should_stop=should_stop, has_lead=has_lead,
                             v_ego=v_ego, v_cruise=v_cruise):
      return a
    climb = self._hold_a
    return a if a > climb else climb

  def should_hold_pedal(self, a_cmd, *, brake_pressed: bool = False,
                        fcw: bool = False, should_stop: bool = False,
                        has_lead: bool = False, v_ego=None, v_cruise=None) -> bool:
    """Carcontroller: expire grace / seed VDAS when apply() would climb."""
    if not self.active or self.hold_pedal is None:
      return False
    return not self.safety_overrides(
      a_cmd, brake_pressed=brake_pressed, fcw=fcw,
      should_stop=should_stop, has_lead=has_lead,
      v_ego=v_ego, v_cruise=v_cruise,
    )
