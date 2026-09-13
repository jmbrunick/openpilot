"""Post-engage climb handoff for Pre-AP GAS_COMMAND.

Panda blocks ENABLE=1 while gas_pressed (interceptor raw > 650 /
``get_longitudinal_allowed`` = controls_allowed && !gas_pressed_prev), so
an enabled command cannot go on the wire during the analog lift. Tesla
passthrough regen still runs until override ends.

On the first pedal decrease we still close the sit-then-go gap:
- Expire ``ENGAGE_GRACE_FRAMES`` immediately so ACQUIRE cannot floor
  commanded accel at 0 for 0.5 s.
- Seed VirtualDAS with a positive climb accel (not ``commanded_accel=0``).
- On the first ENABLE=1 frame, rewrite the grace-floored command to a
  climb seed. After that, grace is dead and normal OP long climbs to MAX.

Do not freeze last-pressed DI for the rest of the window.

Install-from-card, after force-offroad (wraps the live
``PreAPLongController.update``).
"""
from __future__ import annotations

from openpilot.selfdrive.controls.lib.post_engage_coast import (
  PostEngageCoast,
  cs_pedal_di,
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
  """Replace the last ENABLE=1 GAS_COMMAND with a climb seed. Counter stays consecutive."""
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


def apply_climb_handoff(controller, CS, tesla_can, can_sends, hold,
                        frame=None, di_to_pedal=None) -> bool:
  """Expire grace, seed VDAS climb, rewrite only the grace-floored ENABLE=1."""
  if frame is None:
    frame = int(getattr(controller, "preap_long_engage_frame", 0) or 0)
  engage_frame = int(getattr(controller, "preap_long_engage_frame", 0) or 0)
  was_grace = (int(frame) - engage_frame) < ENGAGE_GRACE_FRAMES

  expire_engage_grace(controller, frame=frame)

  seed_di = hold.hold_pedal
  if seed_di is None:
    seed_di = float(getattr(controller, "prev_pedal_di", 0.0) or 0.0)
  climb_a = float(hold.hold_accel)

  need_seed = was_grace or not getattr(controller, "_nap_climb_seeded", False)
  if need_seed:
    seed_vdas_climb(controller, climb_a=climb_a, pedal_di=float(seed_di),
                    a_ego=_a_ego(CS))
    controller._nap_climb_seeded = True

  # Only the ACQUIRE / still-in-grace ENABLE=1 frame was floored at 0.
  # Later frames: grace is dead, VDAS follows planner climb to MAX.
  # ENABLE=0 passthrough is left alone (panda still blocks ENABLE=1
  # while gas_pressed). Expire + seed still succeed.
  if was_grace:
    apply_held_pedal_command(
      controller, CS, tesla_can, can_sends, seed_di, di_to_pedal=di_to_pedal)
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
  pedal_di = cs_pedal_di(CS, gas_pressed=gas)
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
  else:
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
