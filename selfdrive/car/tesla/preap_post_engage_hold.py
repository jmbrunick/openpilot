"""Sustained post-engage climb handoff for Pre-AP GAS_COMMAND.

Panda blocks ENABLE=1 while gas_pressed (interceptor raw > 650 /
``get_longitudinal_allowed`` = controls_allowed && !gas_pressed_prev), so
an enabled command cannot go on the wire during the analog lift. Tesla
passthrough regen still runs until override ends.

#144 expired grace and rewrote the first ENABLE=1 frame with **last-pressed
peak DI**, then dropped. That stab:

- Looks like a still-pressed pedal on 0x552 (raw > 650) → panda/Python
  ``gasPressed`` → PedalAuthority RELEASE (ENABLE=0) → ACQUIRE
  ``vdas.reset(commanded_accel=0)`` → another peak rewrite → pulse.
- Sets ``prev_pedal_di`` to the peak so the next VDAS update rate-limits
  *down* toward the climb (accel then coast).
- Feeds ``pedal_command_di`` back into ``cs_pedal_di`` so the overlay
  re-arms on command echo.

On-car that was "immediate takeover" then accelerator pulses and
38→28 mph instead of a climb to sticky MAX.

This wrap keeps a **sustained** climb until MAX / safety:

- Expire ``ENGAGE_GRACE_FRAMES`` on every climb frame (ACQUIRE restarts
  grace — kill it again).
- Re-seed VirtualDAS with +a after every ACQUIRE wipe.
- Rewrite an ACQUIRE / still-in-grace ENABLE=1 with the **VDAS climb DI**
  (zero-torque + climb step), never last-pressed peak.
- Later ENABLE=1 frames: grace is dead, VDAS follows planner climb.
- Do not clear the seed flag on a one-frame safety flicker.

Install-from-card, after force-offroad (wraps the live
``PreAPLongController.update``).
"""
from __future__ import annotations

from openpilot.selfdrive.controls.lib.post_engage_coast import (
  PostEngageCoast,
  cs_lift_pedal_di,
)

# card / controlsd rate. Avoid importing openpilot.common.realtime here
# (setproctitle) so helper tests stay import-light.
DT_CTRL = 0.01

GAS_COMMAND_ID = 0x551
# opendbc ENGAGE_GRACE_FRAMES = 50 at 100 Hz. Expire it so grace cannot
# floor the next VDAS update at a=0. The 0.5 s coast must not stay in
# the handoff path.
ENGAGE_GRACE_FRAMES = 50

_installed = False
_ORIG_PREAP_LONG_UPDATE = None


def _hold_for(controller) -> PostEngageCoast:
  hold = getattr(controller, "_nap_post_engage_hold", None)
  if hold is None:
    hold = PostEngageCoast(dt=DT_CTRL)
    controller._nap_post_engage_hold = hold
  return hold


def expire_engage_grace(controller, frame=None) -> None:
  """Point ``preap_long_engage_frame`` so ``elapsed < 50`` is false."""
  if frame is None:
    frame = int(getattr(controller, "preap_long_engage_frame", 0) or 0)
  controller.preap_long_engage_frame = int(frame) - ENGAGE_GRACE_FRAMES
  controller.preap_long_handoff_slew_active = False


def seed_vdas_climb(controller, *, climb_a: float, pedal_di: float,
                    a_ego: float = 0.0) -> None:
  """Replace ACQUIRE's ``vdas.reset(commanded_accel=0)`` with a climb."""
  vdas = getattr(controller, "vdas", None)
  if vdas is not None and hasattr(vdas, "reset"):
    try:
      vdas.reset(
        measured_accel=float(a_ego),
        commanded_accel=max(float(climb_a), 0.0),
        pedal_di_init=float(pedal_di),
        preserve_grade=True,
      )
    except TypeError:
      vdas.reset(
        measured_accel=float(a_ego),
        commanded_accel=max(float(climb_a), 0.0),
      )
    if hasattr(vdas, "prev_pedal_di"):
      vdas.prev_pedal_di = float(pedal_di)
  controller.prev_pedal_di = float(pedal_di)


def _enabled_gas_idx(can_sends) -> int | None:
  for i in range(len(can_sends) - 1, -1, -1):
    msg = can_sends[i]
    if not msg or msg[0] != GAS_COMMAND_ID:
      continue
    try:
      if msg[1][4] & 0x80:
        return i
    except (IndexError, TypeError):
      continue
  return None


def apply_held_pedal_command(controller, CS, tesla_can, can_sends, hold_di: float,
                             di_to_pedal=None) -> bool:
  """Replace the last ENABLE=1 GAS_COMMAND. Counter stays consecutive."""
  if hold_di is None:
    return False
  if di_to_pedal is None:
    try:
      from opendbc.car.tesla.preap.nap_conf import nap_conf
      di_to_pedal = nap_conf.di_to_pedal
    except ImportError:
      return False

  enabled_idx = _enabled_gas_idx(can_sends)
  if enabled_idx is None:
    return False

  try:
    pedal_cmd = di_to_pedal(float(hold_di))
    replacement = tesla_can.create_pedal_command(pedal_cmd, enable=1)
  except Exception:
    return False

  can_sends[enabled_idx] = replacement
  controller.prev_pedal_di = float(hold_di)
  if hasattr(controller, "vdas"):
    controller.vdas.prev_pedal_di = float(hold_di)
  expire_engage_grace(controller)
  CS.pedal_command_di = float(hold_di)
  try:
    CS.pedal_command_counter = replacement[1][4] & 0x0F
  except (IndexError, TypeError):
    pass
  return True


def _current_command_di(controller, CS) -> float:
  """Live VDAS / interceptor DI — not last-pressed peak."""
  prev = float(getattr(controller, "prev_pedal_di", 0.0) or 0.0)
  if prev > 0.0:
    return prev
  vdas = getattr(controller, "vdas", None)
  if vdas is not None:
    vprev = float(getattr(vdas, "prev_pedal_di", 0.0) or 0.0)
    if vprev > 0.0:
      return vprev
  interceptor = getattr(CS, "pedal_interceptor_value", None)
  try:
    idi = float(interceptor) if interceptor is not None else 0.0
  except (TypeError, ValueError):
    idi = 0.0
  return idi if idi > 0.0 else 0.0


def climb_command_di(controller, CS, *, climb_a: float) -> float:
  """DI that realizes the climb floor. Never last-pressed peak.

  After ACQUIRE, ``prev_pedal_di`` is zero-torque / interceptor (foot up).
  One VDAS step from that seed is a modest climb, not a 14 DI stab.
  """
  seed = _current_command_di(controller, CS)
  vdas = getattr(controller, "vdas", None)
  if vdas is not None and hasattr(vdas, "update"):
    try:
      di = vdas.update(
        float(climb_a),
        _v_ego(CS),
        seed,
        a_ego=_a_ego(CS),
        freeze_integrator=False,
        orientation_ned=list(getattr(CS, "orientationNED", None) or []),
      )
      return float(di)
    except TypeError:
      try:
        di = vdas.update(float(climb_a), _v_ego(CS), seed, a_ego=_a_ego(CS))
        return float(di)
      except (TypeError, ValueError):
        pass
    except (TypeError, ValueError):
      pass
  return seed


def apply_climb_handoff(controller, CS, tesla_can, can_sends, hold,
                        frame=None, di_to_pedal=None) -> bool:
  """Expire grace, re-seed after ACQUIRE, rewrite only a coast ENABLE=1."""
  if frame is None:
    frame = int(getattr(controller, "preap_long_engage_frame", 0) or 0)
  engage_frame = int(getattr(controller, "preap_long_engage_frame", 0) or 0)
  was_grace = (int(frame) - engage_frame) < ENGAGE_GRACE_FRAMES
  acquire_now = int(frame) == engage_frame

  expire_engage_grace(controller, frame=frame)

  climb_a = float(hold.hold_accel)
  seed_di = _current_command_di(controller, CS)

  need_seed = was_grace or acquire_now or not getattr(controller, "_nap_climb_seeded", False)
  if need_seed:
    seed_vdas_climb(controller, climb_a=climb_a, pedal_di=float(seed_di),
                    a_ego=_a_ego(CS))
    controller._nap_climb_seeded = True

  # Only the ACQUIRE / still-in-grace ENABLE=1 frame is floored at 0.
  # Rewrite that coast with a *climb* DI (VDAS step), not last-pressed peak.
  # Later frames: grace is dead, VDAS follows planner climb to MAX.
  # ENABLE=0 passthrough is left alone (panda still blocks ENABLE=1
  # while gas_pressed). Expire + seed still succeed.
  if was_grace or acquire_now:
    tx_di = climb_command_di(controller, CS, climb_a=climb_a)
    apply_held_pedal_command(
      controller, CS, tesla_can, can_sends, tx_di, di_to_pedal=di_to_pedal)
  return True


def _a_ego(CS) -> float:
  out = getattr(CS, "out", None)
  if out is not None:
    return float(getattr(out, "aEgo", 0.0) or 0.0)
  return float(getattr(CS, "aEgo", 0.0) or 0.0)


def _gas_pressed(CS) -> bool:
  out = getattr(CS, "out", None)
  if out is not None and bool(getattr(out, "gasPressed", False)):
    return True
  return bool(getattr(CS, "gasPressed", False))


def _v_ego(CS) -> float:
  out = getattr(CS, "out", None)
  if out is not None:
    return float(getattr(out, "vEgo", 0.0) or 0.0)
  return float(getattr(CS, "vEgo", 0.0) or 0.0)


def _v_cruise_ms(CS) -> float:
  """HUD / sticky MAX in m/s. Pedal mode owns speed via pedal_speed_kph."""
  kph = getattr(CS, "pedal_speed_kph", None)
  if kph:
    return float(kph) / 3.6
  out = getattr(CS, "out", None)
  src = out if out is not None else CS
  cruise = getattr(src, "cruiseState", None)
  if cruise is not None:
    spd = getattr(cruise, "speed", 0.0) or 0.0
    if spd:
      return float(spd)
  vc = getattr(src, "vCruise", 0.0) or 0.0
  return float(vc) / 3.6 if vc else 0.0


def _preap_long_update_with_pedal_hold(self, CC, CS, frame, tesla_can, can_bus_party, now_nanos=0):
  orig = _ORIG_PREAP_LONG_UPDATE
  hold = _hold_for(self)
  gas = _gas_pressed(CS)
  # Lift detection must not see the ENABLE=1 command echo.
  pedal_di = cs_lift_pedal_di(CS, gas_pressed=gas)
  hold.update(
    long_engaged=bool(getattr(CS, "enableLongControl", False)),
    gas_pressed=gas,
    pedal_pos=pedal_di,
    a_ego=_a_ego(CS),
  )

  sends = orig(self, CC, CS, frame, tesla_can, can_bus_party, now_nanos)

  a_cmd = float(getattr(getattr(CC, "actuators", None), "accel", 0.0) or 0.0)
  brake = bool(getattr(CS, "real_brake_pressed", False))
  if hold.should_hold_pedal(a_cmd, brake_pressed=brake,
                            v_ego=_v_ego(CS), v_cruise=_v_cruise_ms(CS)):
    apply_climb_handoff(self, CS, tesla_can, sends, hold, frame=frame)
  elif not hold.active:
    self._nap_climb_seeded = False
    if gas and pedal_di > 0.0:
      # Analog lift for planner/controlsd next frame (command is 0 in passthrough).
      CS.pedal_command_di = pedal_di
  return sends


def install_post_engage_hold():
  """Patch pedal TX after the other Pre-AP installs."""
  global _installed, _ORIG_PREAP_LONG_UPDATE
  from opendbc.car.tesla.preap.carcontroller import PreAPLongController

  if _installed:
    return
  _ORIG_PREAP_LONG_UPDATE = PreAPLongController.update
  PreAPLongController.update = _preap_long_update_with_pedal_hold
  _installed = True
