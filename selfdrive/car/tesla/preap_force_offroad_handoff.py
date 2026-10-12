"""Pre-AP stock-CC handoff for Force Offroad, and kill CC on OP long engage.

Force Offroad (`NAPForceOffroad`) used to flip `deviceState.started` false
immediately. Cancel / SET spoofs live in the onroad carcontroller stack, so
that dumped pedal long with nothing holding speed (hard regen).

When NAP is actively using software long (`enableLongControl`), hold
`started` until this FSM finishes:

  1. Drive DI toward STANDBY (CANCEL from ENABLED / STANDSTILL / OVERRIDE;
     MAIN arm from OFF — SET_ACCEL is ignored on an unarmed DI).
  2. Once STANDBY (or DI already ENABLED from MAIN), drop OP long.
  3. SET_ACCEL at current ego (`preap_cc_engage_needed`).
  4. After DI ENABLED (or a short timeout), write
     `NAPForceOffroadHandoffReady` so hardwared may set started=false.

If NAP is not in software long, write ready immediately (today's path).

On OP long engage, if stock CC is ENABLED or STANDBY, keep canceling until
it is neither so pedal long and stock CC do not fight. Force Offroad
handoff suppresses that kill so it can park DI in STANDBY and SET.

Install-from-card, same pattern as preap_blinker_lat_pause / body_controls.
"""

from __future__ import annotations

from dataclasses import dataclass

# Mirror opendbc StockCCSpoofer so this module stays importable without capnp.
# Keep in sync with opendbc_repo/opendbc/car/tesla/preap/stock_cc_spoofer.py.
CANCEL_DELAY_FRAMES = 10  # 100 ms at 100 Hz
CC_ENGAGE_TIMEOUT_FRAMES = 50  # 500 ms ENGAGING retries

PARAM = "NAPForceOffroad"
HANDOFF_READY_PARAM = "NAPForceOffroadHandoffReady"
CONFIRMED_PARAM = "NAPForceOffroadConfirmed"

# Card-side overall budget. CANCEL_DELAY (100 ms) + slot align (~100 ms) +
# DI lag + ENGAGING retries (500 ms) + another DI lag. Prefer ENABLED;
# after this, write ready anyway so maps still unlock.
HANDOFF_TIMEOUT_S = 2.5

# Re-issue a one-shot spoof only after the spoofer has had time to TX
# (CANCEL_DELAY_FRAMES + one 10 Hz slot) plus a little DI lag.
SPOOF_RETRY_S = 0.25

# Wait for STANDBY after cancel/arm before giving up that phase.
STANDBY_WAIT_S = 0.80

# Wait for ENABLED after SET. StockCCSpoofer's ENGAGING timeout (500 ms)
# plus DI_state lag.
ENABLED_WAIT_S = (CC_ENGAGE_TIMEOUT_FRAMES / 100.0) + 0.30

# Tesla DI_cruiseState values that are holding / about to hold speed.
_HOLDING = frozenset({"ENABLED", "STANDSTILL", "OVERRIDE", "PRE_CANCEL"})
_READY_TO_SET = frozenset({"STANDBY"})
_ENABLED_OK = frozenset({"ENABLED", "STANDSTILL"})
_KILL = frozenset({"ENABLED", "STANDBY", "STANDSTILL", "OVERRIDE", "PRE_CANCEL"})

IDLE = "idle"
CANCELING = "canceling"
ARMING = "arming"
DROP_AND_SET = "drop_and_set"
WAIT_ENABLED = "wait_enabled"
READY = "ready"


@dataclass(frozen=True)
class HandoffCommand:
  """One-tick intent for engagement + StockCCSpoofer flags."""
  cancel: bool = False
  arm: bool = False
  engage: bool = False
  drop_long: bool = False
  ready: bool = False
  ignore_stalk: bool = False
  suppress_cancel: bool = False
  state: str = IDLE


def di_is_holding(di_state: str) -> bool:
  return (di_state or "OFF") in _HOLDING


def di_ready_to_set(di_state: str) -> bool:
  return (di_state or "OFF") in _READY_TO_SET


def di_enabled_ok(di_state: str) -> bool:
  return (di_state or "OFF") in _ENABLED_OK


def di_needs_kill(di_state: str) -> bool:
  return (di_state or "OFF") in _KILL


def software_long_active(enable_long_control: bool) -> bool:
  """Gate for the Force Offroad handoff. Software long, not pedal handshake."""
  return bool(enable_long_control)


class StockCCKillOnLong:
  """Cancel stock CC while OP long is coming up, until it is not ENABLED/STANDBY.

  One-shot CANCELs with a cooldown so we do not reset StockCCSpoofer's
  cancel_frame every tick (that would prevent the TX). Rising long starts
  the sequence; ENABLED while long is on re-arms it (they are fighting).
  """

  def __init__(self):
    self.active = False
    self.prev_long = False
    self.last_cancel_t: float | None = None

  def reset(self):
    self.active = False
    self.prev_long = False
    self.last_cancel_t = None

  def update(self, enable_long: bool, di_state: str, now: float) -> bool:
    if not enable_long:
      self.active = False
      self.prev_long = False
      return False

    rising = enable_long and not self.prev_long
    self.prev_long = True
    if rising or di_is_holding(di_state):
      self.active = True

    if not self.active:
      return False
    if not di_needs_kill(di_state):
      self.active = False
      return False
    if self.last_cancel_t is not None and (now - self.last_cancel_t) < SPOOF_RETRY_S:
      return False
    self.last_cancel_t = now
    return True


class ForceOffroadHandoff:
  """Pure FSM. No Params / CAN. card + controller wraps apply the command."""

  def __init__(self, timeout_s: float = HANDOFF_TIMEOUT_S):
    self.timeout_s = timeout_s
    self.state = IDLE
    self.started_t: float | None = None
    self.phase_t: float | None = None
    self.last_spoof_t: float | None = None
    self.last_cmd = HandoffCommand()
    self.kill = StockCCKillOnLong()
    self.dropped_long = False

  def reset(self):
    self.state = IDLE
    self.started_t = None
    self.phase_t = None
    self.last_spoof_t = None
    self.dropped_long = False
    self.last_cmd = HandoffCommand()
    self.kill.reset()

  def _enter(self, state: str, now: float):
    self.state = state
    self.phase_t = now

  def _overall_timeout(self, now: float) -> bool:
    return self.started_t is not None and (now - self.started_t) >= self.timeout_s

  def _phase_timeout(self, now: float, budget: float) -> bool:
    return self.phase_t is not None and (now - self.phase_t) >= budget

  def _emit_spoof(self, now: float) -> bool:
    if self.last_spoof_t is not None and (now - self.last_spoof_t) < SPOOF_RETRY_S:
      return False
    self.last_spoof_t = now
    return True

  def update(self, *, force_offroad: bool, enable_long: bool, di_state: str,
             now: float, confirmed: bool = True) -> HandoffCommand:
    di_state = di_state or "OFF"

    if force_offroad and not confirmed:
      # Waiting for the on-road Yes/No. Do not cancel, drop long, or SET.
      self.state = IDLE
      self.started_t = None
      self.phase_t = None
      self.last_spoof_t = None
      self.dropped_long = False
      self.last_cmd = HandoffCommand(ready=False, state=IDLE)
      return self.last_cmd

    if not force_offroad:
      self.state = IDLE
      self.started_t = None
      self.phase_t = None
      self.last_spoof_t = None
      self.dropped_long = False
      cancel = self.kill.update(enable_long, di_state, now)
      self.last_cmd = HandoffCommand(cancel=cancel, ready=False, state=IDLE)
      return self.last_cmd

    # Force Offroad is on. Stop the engage-kill so we can park in STANDBY.
    self.kill.reset()
    self.kill.prev_long = bool(enable_long)

    if self.state == IDLE:
      if not software_long_active(enable_long):
        self.last_cmd = HandoffCommand(ready=True, state=READY)
        return self.last_cmd
      self.started_t = now
      self.dropped_long = False
      if di_is_holding(di_state):
        self._enter(CANCELING, now)
      elif di_ready_to_set(di_state) or di_enabled_ok(di_state):
        self._enter(DROP_AND_SET, now)
      else:
        self._enter(ARMING, now)

    if self._overall_timeout(now) and self.state != READY:
      self._enter(READY, now)

    cmd = self._step(enable_long=enable_long, di_state=di_state, now=now)
    self.last_cmd = cmd
    return cmd

  def _step(self, *, enable_long: bool, di_state: str, now: float) -> HandoffCommand:
    if self.state == CANCELING:
      if di_ready_to_set(di_state):
        self._enter(DROP_AND_SET, now)
      elif not di_is_holding(di_state):
        # CANCEL went to OFF (or fault). Arm, then SET.
        self._enter(ARMING, now)
      elif self._phase_timeout(now, STANDBY_WAIT_S):
        self._enter(READY, now)
      else:
        return HandoffCommand(
          cancel=self._emit_spoof(now),
          ignore_stalk=True,
          state=CANCELING,
        )

    if self.state == ARMING:
      if di_ready_to_set(di_state):
        self._enter(DROP_AND_SET, now)
      elif di_enabled_ok(di_state):
        # MAIN from OFF sometimes SETs in one step. Drop OP; CC already holds.
        self._enter(DROP_AND_SET, now)
      elif self._phase_timeout(now, STANDBY_WAIT_S):
        self._enter(READY, now)
      else:
        return HandoffCommand(
          arm=self._emit_spoof(now),
          ignore_stalk=True,
          state=ARMING,
        )

    if self.state == DROP_AND_SET:
      drop = not self.dropped_long
      self.dropped_long = True
      self._enter(WAIT_ENABLED, now)
      # If DI is already ENABLED (MAIN took it), do not SET — just finish.
      if di_enabled_ok(di_state):
        self._enter(READY, now)
        return HandoffCommand(
          drop_long=drop,
          ready=True,
          ignore_stalk=True,
          suppress_cancel=True,
          state=READY,
        )
      return HandoffCommand(
        engage=True,
        drop_long=drop,
        ignore_stalk=True,
        suppress_cancel=True,
        state=WAIT_ENABLED,
      )

    if self.state == WAIT_ENABLED:
      if di_enabled_ok(di_state):
        self._enter(READY, now)
      elif self._phase_timeout(now, ENABLED_WAIT_S) or self._overall_timeout(now):
        self._enter(READY, now)
      else:
        # StockCCSpoofer retries SET while ENGAGING. Re-issue only if it
        # dropped back to IDLE (timeout / cancel) and DI is still STANDBY.
        return HandoffCommand(
          engage=di_ready_to_set(di_state) and self._emit_spoof(now),
          ignore_stalk=True,
          suppress_cancel=True,
          state=WAIT_ENABLED,
        )

    if self.state == READY:
      return HandoffCommand(
        ready=True,
        ignore_stalk=True,
        suppress_cancel=True,
        drop_long=False,
        state=READY,
      )

    return HandoffCommand(state=self.state)


_handoff = ForceOffroadHandoff()
_ready_written: bool | None = None
_ORIG_PREAP_LONG_UPDATE = None
_ORIG_STOCK_CC_UPDATE = None
_ORIG_PROCESS_BUTTONS = None
_installed = False


def _params():
  try:
    from openpilot.common.params import Params
    return Params()
  except Exception:
    return None


def _read_force_offroad(params=None) -> bool:
  try:
    p = params if params is not None else _params()
    return bool(p.get_bool(PARAM)) if p is not None else False
  except Exception:
    return False


def _read_confirmed(params=None) -> bool:
  try:
    p = params if params is not None else _params()
    return bool(p.get_bool(CONFIRMED_PARAM)) if p is not None else False
  except Exception:
    return False


def _write_ready(ready: bool) -> None:
  global _ready_written
  ready = bool(ready)
  if _ready_written is ready:
    return
  p = _params()
  if p is None:
    return
  try:
    p.put_bool(HANDOFF_READY_PARAM, ready)
    _ready_written = ready
  except Exception:
    pass


def _drop_long_silent(inner_cs) -> None:
  """Stop commanding long. Keep cruiseEnabled so lat stays until offroad."""
  eng = getattr(inner_cs, "engagement", None)
  if eng is not None:
    eng.enableLongControl = False
    if getattr(eng, "cruiseEnabled", False):
      eng.enableJustCC = True
  if hasattr(inner_cs, "enableLongControl"):
    inner_cs.enableLongControl = False
    if getattr(inner_cs, "cruiseEnabled", False):
      inner_cs.enableJustCC = True


def _apply_cmd_flags(inner_cs, cmd: HandoffCommand) -> None:
  if inner_cs is None:
    return
  if cmd.cancel:
    inner_cs.preap_cc_cancel_needed = True
  if cmd.engage:
    inner_cs.preap_cc_engage_needed = True
  if cmd.arm:
    inner_cs.preap_cc_arm_needed = True
  eng = getattr(inner_cs, "engagement", None)
  if eng is not None:
    eng._nap_handoff_ignore_stalk = bool(cmd.ignore_stalk)
    if cmd.cancel:
      eng.preap_cc_cancel_needed = True
    if cmd.engage:
      eng.preap_cc_engage_needed = True


def apply_controller_flags(CS, cmd: HandoffCommand | None = None) -> None:
  """Re-assert spoof flags after PreAPLongController may have overwritten them.

  Falling enableLongControl otherwise sets cancel_needed and StockCCSpoofer
  would abort ENGAGING (cancel beats engage).
  """
  if cmd is None:
    cmd = _handoff.last_cmd
  if cmd.suppress_cancel:
    CS.preap_cc_cancel_needed = False
  if cmd.cancel:
    CS.preap_cc_cancel_needed = True
  if cmd.engage:
    CS.preap_cc_engage_needed = True
  if cmd.arm:
    CS.preap_cc_arm_needed = True


def update_force_offroad_handoff(inner_cs, public_cs=None, *, now: float | None = None,
                                 force_offroad: bool | None = None,
                                 confirmed: bool | None = None,
                                 params=None) -> HandoffCommand:
  """card.state_update hook. Runs at 100 Hz after CI.update()."""
  import time
  if now is None:
    now = time.monotonic()
  if force_offroad is None:
    force_offroad = _read_force_offroad(params)
  if confirmed is None:
    confirmed = _read_confirmed(params) if force_offroad else False

  if inner_cs is None and public_cs is None:
    ready = bool(force_offroad) and confirmed
    _write_ready(ready)
    return HandoffCommand(ready=ready, state=READY if ready else IDLE)

  enable_long = bool(
    getattr(inner_cs, "enableLongControl", False)
    if inner_cs is not None else False
  )
  if public_cs is not None:
    enable_long = enable_long or bool(getattr(public_cs, "enableLongControl", False))
  di_state = "OFF"
  if inner_cs is not None:
    di_state = getattr(inner_cs, "di_cruise_state", None) or "OFF"

  cmd = _handoff.update(
    force_offroad=force_offroad,
    enable_long=enable_long,
    di_state=di_state,
    now=now,
    confirmed=confirmed,
  )
  if cmd.drop_long and inner_cs is not None:
    _drop_long_silent(inner_cs)
    if public_cs is not None and hasattr(public_cs, "enableLongControl"):
      try:
        public_cs.enableLongControl = False
      except Exception:
        pass
  _apply_cmd_flags(inner_cs, cmd)
  _write_ready(bool(cmd.ready) if force_offroad else False)
  return cmd


def _preap_long_update_with_handoff(self, CC, CS, frame, tesla_can, can_bus_party, now_nanos=0):
  orig = _ORIG_PREAP_LONG_UPDATE
  sends = orig(self, CC, CS, frame, tesla_can, can_bus_party, now_nanos)
  apply_controller_flags(CS)
  return sends


def _stock_cc_update_with_arm(self, CS, frame, tesla_can, can_bus_party):
  """MAIN arm spoof. StockCCSpoofer only knows cancel + SET_ACCEL."""
  orig = _ORIG_STOCK_CC_UPDATE
  if getattr(CS, "preap_cc_arm_needed", False):
    self._nap_arm_pending = True
    self._nap_arm_frame = frame
    CS.preap_cc_arm_needed = False
  can_sends = orig(self, CS, frame, tesla_can, can_bus_party)
  if getattr(self, "_nap_arm_pending", False):
    from opendbc.car.tesla.values import CruiseButtons
    ready = (frame - int(getattr(self, "_nap_arm_frame", frame))) >= CANCEL_DELAY_FRAMES
    if ready and frame % 10 == 0:
      sent = self._send(CS, tesla_can, can_bus_party, CruiseButtons.MAIN)
      if sent is not None:
        can_sends.append(sent)
        self._nap_arm_pending = False
  return can_sends


def _process_buttons_with_handoff(self, cruise_buttons, prev_cruise_buttons, *args, **kwargs):
  """Spoofed MAIN / CANCEL / SET must not look like a driver stalk pull."""
  from opendbc.car.tesla.values import CruiseButtons
  orig = _ORIG_PROCESS_BUTTONS
  if getattr(self, "_nap_handoff_ignore_stalk", False):
    cruise_buttons = CruiseButtons.IDLE
    prev_cruise_buttons = CruiseButtons.IDLE
  return orig(self, cruise_buttons, prev_cruise_buttons, *args, **kwargs)


def install_force_offroad_handoff():
  """Patch pedal-long + stock-cc + engagement. Call after the other Pre-AP installs."""
  global _installed, _ORIG_PREAP_LONG_UPDATE, _ORIG_STOCK_CC_UPDATE, _ORIG_PROCESS_BUTTONS
  from opendbc.car.tesla.preap.carcontroller import PreAPLongController
  from opendbc.car.tesla.preap.engagement import PreAPEngagement
  from opendbc.car.tesla.preap.stock_cc_spoofer import StockCCSpoofer

  if _installed:
    return
  _ORIG_PREAP_LONG_UPDATE = PreAPLongController.update
  _ORIG_STOCK_CC_UPDATE = StockCCSpoofer.update
  _ORIG_PROCESS_BUTTONS = PreAPEngagement.process_buttons
  PreAPLongController.update = _preap_long_update_with_handoff
  StockCCSpoofer.update = _stock_cc_update_with_arm
  PreAPEngagement.process_buttons = _process_buttons_with_handoff
  _installed = True


def reset_handoff_for_tests():
  """Unit tests: clear singleton state. Does not uninstall patches."""
  global _ready_written
  _handoff.reset()
  _ready_written = None
