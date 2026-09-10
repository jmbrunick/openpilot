"""Keep Pre-AP cruiseEnabled while a blinker lamp pauses lateral.

opendbc's handle_steering_disengage tears down the FSM on hands-on ≥ 2.
That drop is EventName.pcmDisable — Pre-AP HUD "Steering Disengaged".
GTW lamp bits flash, so BlinkerLateralHold latches turn-active through
those gaps. During that turn we have already released steering, so a
wheel input must not drop cruiseEnabled. After ~1s of continuous dark,
keep that suppression until torque is released so a finishing hand-steer
does not fully disengage NAP. Stalk cancel is unchanged.

A latched *driver turn* (not ALC tip/keep-alive) also drops longitudinal
the same way brake does: enableLongControl=False, cruiseEnabled stays.
Drop while a turn lamp or held LEFT/RIGHT is showing. After lamps/stalk
go idle, long stays off until SET — including during the ~1s dark latch
and while lat is still paused for hand-on. One stalk SET restores long
whether lat is still paused or already active. Default double-pull would
otherwise treat that SET as a first pull (lat-only) and require a second
pull inside the window; that path is skipped while a drop-long-keep-lat
resume is pending. SET while a turn lamp/stalk is still showing does not
stick.

card.py imports tesla.carstate (binding update_preap) before this install.
Patch both the source module and that imported name, or the live path
never sets _nap_* lamps/stalk and a faster corner still full-cancels.

ALC keep-alive flashes are not a driver turn: do not latch turn-active,
do not drop long, and do not tear down cruise. DAS_bodyControls
turn-indicator TX stays in teslacan / carcontroller from CC.leftBlinker;
this helper does not touch it.

This follows the same install-from-card pattern as preap_body_controls.

Panda tesla_preap still has its own hands-on >= 2 path. Matching flash-latch
in tesla_preap.h keeps controls_allowed; selfdrived also hides the
controlsMismatch that would otherwise full-cancel after 2s.
"""

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import (
  BlinkerLateralHold,
  blinker_pauses_lateral,
  blinker_turn_blocks_steering_disengage,
  stalk_is_left_or_right,
)

_ORIG_HANDLE = None
_ORIG_UPDATE = None
_ORIG_PROCESS = None
_ORIG_DROP = None
_installed = False


def _peek_blinker_lamps(can_parsers):
  try:
    from opendbc.car import Bus
    gtw = can_parsers[Bus.chassis].vl["GTW_carState"]
    return gtw["BC_indicatorLStatus"] == 1, gtw["BC_indicatorRStatus"] == 1
  except Exception:
    return False, False


def _peek_steering_override(can_parsers):
  """(steering_pressed, steering_disengage) from EPAS.

  A slightly faster corner hits EPAS_handsOnLevel 2 (steeringDisengage)
  before the 5-frame steeringPressed debounce. Counting only the torque
  threshold let a flash-gap expire the latch, then the next hands-on
  rising edge tore down cruiseEnabled.
  """
  try:
    from opendbc.car import Bus
    from opendbc.car.tesla.values import STEER_THRESHOLD
    epas = can_parsers[Bus.chassis].vl["EPAS_sysStatus"]
    torque = abs(epas["EPAS_torsionBarTorque"]) > STEER_THRESHOLD
    hands = int(epas.get("EPAS_handsOnLevel", 0) or 0) >= 2
    return bool(torque), bool(hands)
  except Exception:
    return False, False


def _peek_steering_pressed(can_parsers):
  pressed, disengage = _peek_steering_override(can_parsers)
  return pressed or disengage


def _peek_stalk_and_speed(can_parsers):
  try:
    from opendbc.car import Bus
    stw = can_parsers[Bus.chassis].vl["STW_ACTN_RQ"]
    stalk = int(stw.get("TurnIndLvr_Stat", 0) or 0)
    if stalk == 3:  # SNA
      stalk = 0
    v_kph = can_parsers[Bus.chassis].vl["ESP_B"]["ESP_vehicleSpeed"]
    return stalk, float(v_kph) * CV.KPH_TO_MS
  except Exception:
    return 0, 0.0


def _hold_for(engagement):
  hold = getattr(engagement, "_nap_lat_hold", None)
  if hold is None:
    hold = BlinkerLateralHold()
    engagement._nap_lat_hold = hold
  return hold


def _hold_kwargs(engagement, dt=None):
  kwargs = dict(
    alc_active=bool(getattr(engagement, "_nap_alc_active", False)),
    v_ego=float(getattr(engagement, "_nap_v_ego", 0.0) or 0.0),
    stalk_state=int(getattr(engagement, "_nap_stalk_state", 0) or 0),
  )
  if dt is not None:
    kwargs["dt"] = dt
  return kwargs


def _driver_turn_active(engagement) -> bool:
  hold = getattr(engagement, "_nap_lat_hold", None)
  return hold is not None and bool(hold.turn_active)


def _should_drop_long_for_turn(engagement) -> bool:
  """Drop long while a driver turn is *showing*, not during the dark latch.

  turn_active stays up through Tesla flash gaps and ~1s after the last
  flash. Blocking SET for that whole latch is what forced a second stalk
  pull: the driver sees the blinker off, SETs, and long does not stick.
  Keep dropping only while one lamp is lit or the stalk is still LEFT/RIGHT.
  After lamps/stalk are idle, one SET restores long even if turn_active
  or lat holding is still true. A later flash (still the same turn)
  drops long again.
  """
  if not _driver_turn_active(engagement):
    return False
  left = bool(getattr(engagement, "_nap_left_blinker", False))
  right = bool(getattr(engagement, "_nap_right_blinker", False))
  if blinker_pauses_lateral(left, right):
    return True
  stalk = int(getattr(engagement, "_nap_stalk_state", 0) or 0)
  return stalk_is_left_or_right(stalk)


def _drop_long_if_driver_turn(engagement):
  """Brake-style long drop while a latched driver turn is showing.

  ALC tip/keep-alive never sets turn_active, so those flashes keep long.
  Post-turn hand-on (holding) does not keep dropping: one SET can restore
  long while lat is still paused.
  """
  if _should_drop_long_for_turn(engagement):
    engagement._drop_longitudinal_keep_lateral()


def _drop_longitudinal_keep_lateral(self):
  # Remember that long was released while staying engaged so the next SET
  # can restore it in one pull (double-pull's first pull would otherwise
  # stay lat-only and demand a second pull inside the window).
  if self.cruiseEnabled:
    self._nap_long_resume_pending = True
  return _ORIG_DROP(self)


def _process_buttons(self, cruise_buttons, prev_cruise_buttons, *args, **kwargs):
  from opendbc.car.tesla.values import CruiseButtons

  # Full disengage (cancel, door, previous cycle) must not leave a stale
  # resume latch that would skip double-pull on the next first SET.
  if not self.cruiseEnabled:
    self._nap_long_resume_pending = False

  _drop_long_if_driver_turn(self)

  # One SET restores long after brake/blinker drop-keep-lat. Do not start
  # a new double-pull window. While a turn lamp or held stalk is still
  # showing, SET must not stick — drop again after the FSM so long stays
  # off. After lamps/stalk go idle, SET sticks even if the ~1s flash
  # latch has not expired yet.
  resume = (
    cruise_buttons == CruiseButtons.MAIN
    and prev_cruise_buttons != CruiseButtons.MAIN
    and bool(self.cruiseEnabled)
    and bool(getattr(self, "_nap_long_resume_pending", False))
    and not _should_drop_long_for_turn(self)
  )
  saved_double = self.enableDoublePull
  if resume:
    self.enableDoublePull = False
  try:
    result = _ORIG_PROCESS(self, cruise_buttons, prev_cruise_buttons, *args, **kwargs)
  finally:
    self.enableDoublePull = saved_double

  _drop_long_if_driver_turn(self)

  if not self.cruiseEnabled or self.enableLongControl:
    self._nap_long_resume_pending = False
  return result


def _handle_steering_disengage(self, steering_disengage):
  left = getattr(self, "_nap_left_blinker", False)
  right = getattr(self, "_nap_right_blinker", False)
  stalk = int(getattr(self, "_nap_stalk_state", 0) or 0)
  pressed = bool(getattr(self, "_nap_steering_pressed", False) or steering_disengage)
  hold = _hold_for(self)
  # dt=0: _update_preap already advanced the dark timer this cycle.
  hold.update(
    left, right, pressed, engaged=bool(getattr(self, "cruiseEnabled", False)),
    steering_disengage=bool(steering_disengage),
    **_hold_kwargs(self, dt=0.0))
  _drop_long_if_driver_turn(self)
  # Hard gate: one lamp or physical LEFT/RIGHT, not only hold state.
  # Dropping cruiseEnabled here is EventName.pcmDisable on Pre-AP —
  # the 3X HUD string "Steering Disengaged".
  if blinker_turn_blocks_steering_disengage(left, right, stalk, hold):
    # Keep prev in sync so lamp-off / hand-release is not a rising edge.
    self.prev_steering_disengage = steering_disengage
    return
  _ORIG_HANDLE(self, steering_disengage)
  if not self.cruiseEnabled:
    self._nap_long_resume_pending = False


def _update_preap(cs, can_parsers):
  left, right = _peek_blinker_lamps(can_parsers)
  pressed, disengage = _peek_steering_override(can_parsers)
  stalk, v_ego = _peek_stalk_and_speed(can_parsers)
  engagement = getattr(cs, "engagement", None)
  if engagement is not None:
    engagement._nap_left_blinker = left
    engagement._nap_right_blinker = right
    engagement._nap_steering_pressed = pressed or disengage
    engagement._nap_stalk_state = stalk
    engagement._nap_v_ego = v_ego
    _hold_for(engagement).update(
      left, right, pressed, engaged=bool(getattr(engagement, "cruiseEnabled", False)),
      steering_disengage=disengage,
      **_hold_kwargs(engagement))
    _drop_long_if_driver_turn(engagement)
  return _ORIG_UPDATE(cs, can_parsers)


def _rewire_tesla_carstate_update():
  """card.py builds CarInterface before install, which binds update_preap.

  tesla/carstate.py does `from ...preap.carstate import update_preap` and
  calls that local name. Patching only the source module leaves the live
  path on the original, so _nap_* lamp/stalk never land and a faster
  corner tears down cruiseEnabled → pcmDisable / "Steering Disengaged".
  """
  try:
    from opendbc.car.tesla import carstate as tesla_carstate
    tesla_carstate.update_preap = _update_preap
  except Exception:
    pass


def install_blinker_lat_pause():
  """Patch Pre-AP engagement so a lamp-on turn does not tear down cruise."""
  global _installed, _ORIG_HANDLE, _ORIG_UPDATE, _ORIG_PROCESS, _ORIG_DROP
  from opendbc.car.tesla.preap import carstate as preap_carstate
  from opendbc.car.tesla.preap.engagement import PreAPEngagement

  if not _installed:
    _ORIG_HANDLE = PreAPEngagement.handle_steering_disengage
    _ORIG_UPDATE = preap_carstate.update_preap
    _ORIG_PROCESS = PreAPEngagement.process_buttons
    _ORIG_DROP = PreAPEngagement._drop_longitudinal_keep_lateral
    PreAPEngagement.handle_steering_disengage = _handle_steering_disengage
    PreAPEngagement.process_buttons = _process_buttons
    PreAPEngagement._drop_longitudinal_keep_lateral = _drop_longitudinal_keep_lateral
    preap_carstate.update_preap = _update_preap
    _installed = True
  _rewire_tesla_carstate_update()
