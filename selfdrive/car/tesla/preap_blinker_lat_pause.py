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
whether lat is still paused or already active, and keeps the held MAX
(which may already have rebased if posted changed). Default double-pull
would otherwise treat that SET as a first pull (lat-only) and require a
second pull inside the window; that path is skipped while a drop-long-keep-lat
resume is pending. A second SET in the double-pull window forgets sticky
and takes posted (maps on + known) or current traveled speed (maps off /
unknown posted). SET while a turn lamp/stalk is still showing does not
stick. Tip ALC never uses this long-pause path.

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


def _peek_hands_on_level(can_parsers) -> int:
  """Raw EPAS_handsOnLevel 0/1/2/3. 1 = still on the rim (soft-yield hold)."""
  try:
    from opendbc.car import Bus
    epas = can_parsers[Bus.chassis].vl["EPAS_sysStatus"]
    return int(epas.get("EPAS_handsOnLevel", 0) or 0)
  except Exception:
    return 0


def _publish_hands_on_level(ret, hands: int) -> None:
  """Expose EPAS hands-on to controlsd for soft lat hold.

  Prefer cereal handsOnLevel when the schema has it. Also stash the
  discrete 0/1/2/3 on steeringTorqueEps (unused on Pre-AP angle control)
  so a nap-dev-only PR works without an opendbc cereal bump.
  """
  hands = int(max(0, min(3, hands)))
  if hasattr(ret, 'handsOnLevel'):
    try:
      ret.handsOnLevel = hands
    except Exception:
      pass
  if hasattr(ret, 'steeringTorqueEps'):
    try:
      ret.steeringTorqueEps = float(hands)
    except Exception:
      pass


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
  # Orig zeros pedal_speed_kph — restore held MAX so one SET does not
  # recapture ego and so card can keep sticky across the pause.
  if self.cruiseEnabled:
    self._nap_long_resume_pending = True
    held = float(getattr(self, "pedal_speed_kph", 0.0) or 0.0)
    prev_held = getattr(self, "_nap_held_max_kph", None)
    if held > 0:
      self._nap_held_max_kph = held
    elif prev_held is None:
      self._nap_held_max_kph = held
  result = _ORIG_DROP(self)
  held = getattr(self, "_nap_held_max_kph", None)
  if self.cruiseEnabled and held is not None:
    self.pedal_speed_kph = float(held)
  return result


def _curr_time_ms(args, kwargs):
  if args:
    return args[0]
  return kwargs.get("curr_time_ms", 0)


def _use_pedal(args, kwargs):
  if len(args) >= 4:
    return bool(args[3])
  return bool(kwargs.get("use_pedal", False))


def _clear_session_max_flags(engagement):
  engagement._nap_long_resume_pending = False
  engagement._nap_held_max_kph = None
  engagement._nap_set_resume_long = False
  engagement._nap_set_take_speed_now = False


def _process_buttons(self, cruise_buttons, prev_cruise_buttons, *args, **kwargs):
  from opendbc.car.tesla.values import CruiseButtons

  self._nap_set_resume_long = False
  self._nap_set_take_speed_now = False

  # Full disengage (cancel, door, previous cycle) must not leave a stale
  # resume latch that would skip double-pull on the next first SET, or a
  # held MAX from the previous session.
  if not self.cruiseEnabled:
    _clear_session_max_flags(self)

  _drop_long_if_driver_turn(self)

  curr_time_ms = _curr_time_ms(args, kwargs)
  use_pedal = _use_pedal(args, kwargs)
  set_edge = (
    cruise_buttons == CruiseButtons.MAIN
    and prev_cruise_buttons != CruiseButtons.MAIN
  )
  window_ms = float(getattr(self, "double_pull_window_ms", 0) or 0)
  dt_ms = curr_time_ms - float(getattr(self, "stalk_pull_time_ms", 0) or 0)
  in_double_window = (
    window_ms > 0
    and float(getattr(self, "stalk_pull_time_ms", 0) or 0) > 0
    and 0 <= dt_ms < window_ms
  )

  # One SET restores long after brake/blinker drop-keep-lat. Do not start
  # a new double-pull window. While a turn lamp or held stalk is still
  # showing, SET must not stick — drop again after the FSM so long stays
  # off. After lamps/stalk go idle, SET sticks even if the ~1s flash
  # latch has not expired yet.
  resume = (
    set_edge
    and bool(self.cruiseEnabled)
    and bool(getattr(self, "_nap_long_resume_pending", False))
    and not _should_drop_long_for_turn(self)
  )
  # Already lat+long in pedal mode: first SET only arms the double-SET
  # window (do not drop long). Second SET in the window forgets sticky /
  # take speed now. No-pedal keeps the stock first-pull pending_enable path.
  in_session_long_set = (
    set_edge
    and bool(use_pedal)
    and bool(self.cruiseEnabled)
    and bool(self.enableLongControl)
    and bool(self.enableDoublePull)
    and not resume
    and not _should_drop_long_for_turn(self)
  )
  swallow_set = in_session_long_set and not in_double_window
  take_now_in_session = in_session_long_set and in_double_window

  saved_double = self.enableDoublePull
  if resume:
    self.enableDoublePull = False
  orig_buttons = prev_cruise_buttons if swallow_set else cruise_buttons
  was_long = bool(self.enableLongControl)
  try:
    result = _ORIG_PROCESS(self, orig_buttons, prev_cruise_buttons, *args, **kwargs)
  finally:
    self.enableDoublePull = saved_double

  if swallow_set:
    self.stalk_pull_time_ms = curr_time_ms
    self.last_stalk_non_cancel_ms = curr_time_ms

  _drop_long_if_driver_turn(self)

  if not self.cruiseEnabled:
    _clear_session_max_flags(self)
    return result

  if resume and self.enableLongControl:
    self._nap_set_resume_long = True
    held = getattr(self, "_nap_held_max_kph", None)
    if held is not None:
      self.pedal_speed_kph = float(held)
    self.stalk_pull_time_ms = curr_time_ms
    self.last_stalk_non_cancel_ms = curr_time_ms
  elif take_now_in_session and self.enableLongControl:
    self._nap_set_take_speed_now = True
    self._nap_held_max_kph = None
  elif self.enableLongControl and not was_long and not resume:
    # Initial double-pull (or single-pull) engage from disengaged / lat-only.
    self._nap_set_take_speed_now = True

  if self.enableLongControl:
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
    _clear_session_max_flags(self)


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
  ret = _ORIG_UPDATE(cs, can_parsers)
  hands = int(getattr(cs, 'hands_on_level', 0) or 0)
  if hands <= 0:
    hands = _peek_hands_on_level(can_parsers)
  _publish_hands_on_level(ret, hands)
  return ret


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
