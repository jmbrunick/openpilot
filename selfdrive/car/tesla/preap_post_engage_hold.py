"""Rewrite Pre-AP GAS_COMMAND to the frozen post-engage pedal DI.

Panda blocks ENABLE=1 while gas_pressed, so the freeze can only go on
the wire after override ends. Until then we publish the interceptor DI
on ``pedal_command_di`` so planner/controlsd see analog lift.

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
# floor the next VDAS update at a=0 while we are holding pedal.
ENGAGE_GRACE_FRAMES = 50

_installed = False
_ORIG_PREAP_LONG_UPDATE = None


def _hold_for(controller) -> PostEngageCoast:
  hold = getattr(controller, "_nap_post_engage_hold", None)
  if hold is None:
    hold = PostEngageCoast(dt=DT_CTRL)
    controller._nap_post_engage_hold = hold
  return hold


def apply_held_pedal_command(controller, CS, tesla_can, can_sends, hold_di: float,
                             di_to_pedal=None) -> bool:
  """Replace the last ENABLE=1 GAS_COMMAND with the frozen DI. Counter stays consecutive."""
  if hold_di is None:
    return False
  if di_to_pedal is None:
    try:
      from opendbc.car.tesla.preap.nap_conf import nap_conf
      di_to_pedal = nap_conf.di_to_pedal
    except ImportError:
      return False

  enabled_idx = None
  for i in range(len(can_sends) - 1, -1, -1):
    msg = can_sends[i]
    if not msg or msg[0] != GAS_COMMAND_ID:
      continue
    try:
      if msg[1][4] & 0x80:
        enabled_idx = i
        break
    except (IndexError, TypeError):
      continue
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
  frame = int(getattr(controller, "preap_long_engage_frame", 0) or 0)
  # Expire grace relative to the last known engage frame. Using a large
  # offset is safe: in_engage_grace is strictly ``elapsed < 50``.
  controller.preap_long_engage_frame = frame - ENGAGE_GRACE_FRAMES
  controller.preap_long_handoff_slew_active = False
  CS.pedal_command_di = float(hold_di)
  try:
    CS.pedal_command_counter = replacement[1][4] & 0x0F
  except (IndexError, TypeError):
    pass
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
  if hold.should_hold_pedal(a_cmd, brake_pressed=brake):
    apply_held_pedal_command(self, CS, tesla_can, sends, hold.hold_pedal)
  elif gas and pedal_di > 0.0:
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
