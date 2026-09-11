"""Pre-AP driver-wheel temporary lateral handoff.

Light torsion-bar input yields NAP lateral without tearing down the OP
session or longitudinal. Blinker-turn pause / ALC / panda hands-on >= 2
are separate paths and are not reused as the trigger.

Signal (Pre-AP EPAS_sysStatus 0x370, tesla_preap.dbc):
  EPAS_torsionBarTorque  — continuous, Nm, factor 0.01, offset −20.5
  EPAS_handsOnLevel      — discrete 0/1/2/3
  StW_AnglHP_Spd         — steering-angle rate, deg/s, factor 0.5

Existing software thresholds (unchanged):
  steeringPressed  = |torsion| > STEER_THRESHOLD (1.0 Nm), 5-frame debounce
  steerOverride    = EventName from steeringPressed (OVERRIDE_LATERAL, stays enabled)
  steeringDisengage / panda PREAP_HANDS_ON_DISENGAGE_LEVEL = handsOnLevel >= 2
                   (hard USER_DISABLE / pcmDisable / controls_allowed drop)

This module is software-only. It does not change STEER_THRESHOLD, panda
safety, or handle_steering_disengage. Soft-yield trigger is 50% of the
software steeringPressed threshold (0.5 Nm) with hysteresis + debounce so
road vibration does not latch.

handsOnLevel >= 1 is NOT the soft-yield trigger: stock EPAS already uses
~0.5 Nm for 0.25 s to raise level 1, which is slower than a dodge, and
level 1 is a normal hands-on-the-wheel posture while OP is engaged.
handsOnLevel >= 2 stays the hard/safety path.

Quiet (start the 0.25 s hand-back timer) uses low torsion AND low
steering-angle rate. Rate is ignored during the 1 s blend because OP's
own return-to-lane moves the wheel.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from cereal import log
from opendbc.car.tesla.values import STEER_THRESHOLD

# Match controlsd / card (openpilot.common.realtime.DT_CTRL).
DT_CTRL = 0.01

State = log.SelfdriveState.OpenpilotState

# --- thresholds (from real Pre-AP signals; do not invent) ---
# Software steeringPressed / steerOverride: |EPAS_torsionBarTorque| > 1.0 Nm.
SOFT_YIELD_TRIGGER_NM = 0.5 * float(STEER_THRESHOLD)  # 0.5 Nm, half of steeringPressed
SOFT_YIELD_RELEASE_NM = 0.3 * float(STEER_THRESHOLD)  # 0.3 Nm release hysteresis

# steeringPressed uses 5 frames at 1 Nm. Half-threshold needs a bit more
# persistence so texture / rumble does not latch.
SOFT_YIELD_DEBOUNCE_FRAMES = 8  # 80 ms at 100 Hz
SOFT_YIELD_RELEASE_FRAMES = 5   # 50 ms below release before clearing latch

# Held-correction / still-turning while yielded. DBC resolution is 0.5 deg/s.
STEER_RATE_QUIET_DEG_S = 25.0

# --- timing / UI ---
QUIET_WAIT_S = 0.25
BLEND_TIME_S = 1.0
UI_LATERAL_RETURN_AUTHORITY = 0.70  # do not show lat-engaged below this

# Authority is dropped in one control cycle. The physical yield is the
# existing Tesla VM angle limiter (CarControllerParams.ANGLE_LIMITS):
# MAX_ANGLE_RATE = 5 deg / 20 ms (50 Hz STEER_STEP) = 250 deg/s, plus the
# ~3.6 m/s^3 lateral-jerk cap. We do not set latActive=False (that snaps
# apply_angle to measured in apply_steer_angle_limits_vm) and we do not
# bypass those slew limits. 0.08 s is ~20 deg at that rate — typical dodge.
YIELD_AUTHORITY_TIME_S = 0.0
# CarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE (tesla/values.py)
TESLA_MAX_ANGLE_RATE_DEG_PER_20MS = 5.0

# smoothstep(t) = t^2 (3-2t); max |ds/dt| on t in [0,1] is 1.5.
SMOOTHSTEP_MAX_SLOPE = 1.5

PREAP_FINGERPRINT = "TESLA_MODEL_S_PREAP"


def smoothstep(t: float) -> float:
  t = float(np.clip(t, 0.0, 1.0))
  return t * t * (3.0 - 2.0 * t)


def apply_lat_authority(authority: float, torque: float, desired_angle: float,
                        measured_angle: float, desired_curvature: float,
                        measured_curvature: float) -> tuple[float, float, float]:
  """Scale lateral actuator authority. 0 follows the driver; 1 is full NAP."""
  a = float(np.clip(authority, 0.0, 1.0))
  return (
    float(torque) * a,
    a * float(desired_angle) + (1.0 - a) * float(measured_angle),
    a * float(desired_curvature) + (1.0 - a) * float(measured_curvature),
  )


def hud_engaged_status(*, enabled: bool, op_state, lat_active: bool,
                       lat_handoff_paused: bool) -> str:
  """HUD chrome while the OP session may still be enabled.

  'override' is the stable paused / driver-control indication. 'engaged'
  is the normal lateral-on indication and must not return below 70%
  authority (the caller latches lat_handoff_paused). No sounds.
  """
  if lat_handoff_paused and enabled:
    return "override"
  if op_state in (State.preEnabled, State.overriding):
    if op_state == State.overriding and lat_active:
      return "engaged"
    return "override"
  return "engaged" if enabled else "disengaged"


@dataclass(frozen=True)
class HandoffOutput:
  authority: float
  ui_paused: bool
  yielded: bool
  blending: bool


class DriverLateralHandoff:
  """Process-local latch: light wheel input yields lat; quiet + 1 s S-curve hands it back."""

  def __init__(self, enabled: bool = True):
    self.enabled = bool(enabled)
    self._reset()

  def _reset(self):
    self.authority = 1.0
    self.ui_paused = False
    self._yielded = False
    self._blending = False
    self._quiet_s = 0.0
    self._blend_s = 0.0
    self._press_cnt = 0
    self._release_cnt = 0
    self._pressed = False

  def reset(self):
    self._reset()

  def _update_soft_pressed(self, torque_nm: float) -> bool:
    mag = abs(float(torque_nm))
    if mag >= SOFT_YIELD_TRIGGER_NM:
      self._press_cnt = min(self._press_cnt + 1, SOFT_YIELD_DEBOUNCE_FRAMES * 2 + 1)
      self._release_cnt = 0
    elif mag <= SOFT_YIELD_RELEASE_NM:
      self._release_cnt = min(self._release_cnt + 1, SOFT_YIELD_RELEASE_FRAMES * 2 + 1)
      self._press_cnt = max(self._press_cnt - 1, 0)
    # hysteresis band: hold latched state, decay neither way hard

    if self._press_cnt > SOFT_YIELD_DEBOUNCE_FRAMES:
      self._pressed = True
    if self._pressed and self._release_cnt > SOFT_YIELD_RELEASE_FRAMES:
      self._pressed = False
      self._press_cnt = 0
    return self._pressed

  def _set_ui_paused(self):
    # Latch at 70%: never claim lateral is back at 1–69%. Falling through
    # 70% (renewed input) pauses immediately. Rising crosses 70% once on
    # the monotonic blend, so the latch does not chatter.
    if self.authority >= UI_LATERAL_RETURN_AUTHORITY and not self._yielded:
      self.ui_paused = False
    else:
      self.ui_paused = self._yielded or self._blending or (
        self.authority < UI_LATERAL_RETURN_AUTHORITY)

  def _enter_yield(self):
    self._yielded = True
    self._blending = False
    self._quiet_s = 0.0
    self._blend_s = 0.0
    self.authority = 0.0
    self.ui_paused = True

  def update(self, *, engaged: bool, lat_would_be_active: bool,
             steering_torque: float, steering_rate_deg: float,
             alc_active: bool = False, dt: float | None = None) -> HandoffOutput:
    if dt is None:
      dt = DT_CTRL

    if not self.enabled:
      return HandoffOutput(1.0, False, False, False)

    # Blinker pause / standstill / faults already cleared lat_would_be_active.
    # ALC wheel-nudge uses steeringPressed at 1 Nm and must not be softened.
    # Full disengage (cancel / door / hands-on >= 2) clears engaged.
    if not engaged or not lat_would_be_active or alc_active:
      self._reset()
      return HandoffOutput(1.0, False, False, False)

    mag = abs(float(steering_torque))
    pressed = self._update_soft_pressed(steering_torque)
    turning = abs(float(steering_rate_deg)) > STEER_RATE_QUIET_DEG_S
    holding_torque = mag > SOFT_YIELD_RELEASE_NM

    if not self._yielded and not self._blending:
      if pressed:
        self._enter_yield()
    elif self._yielded:
      # Quiet is the live signal, not the trigger latch, so the 0.25 s
      # timer starts as soon as torsion and rate are low.
      if mag >= SOFT_YIELD_TRIGGER_NM or turning or holding_torque:
        self._enter_yield()
      else:
        self._quiet_s += dt
        if self._quiet_s + 1e-12 >= QUIET_WAIT_S:
          self._yielded = False
          self._blending = True
          self._blend_s = 0.0
          self.authority = 0.0
          self.ui_paused = True
    elif self._blending:
      # Immediate on renewed effort. Rate is ignored here: the blend itself
      # turns the wheel. Use the live 0.5 Nm trigger, not the press latch
      # (the latch can still be true for a few quiet frames after release).
      if mag >= SOFT_YIELD_TRIGGER_NM:
        self._enter_yield()
      else:
        self._blend_s += dt
        t = self._blend_s / BLEND_TIME_S
        self.authority = smoothstep(t)
        if self._blend_s + 1e-12 >= BLEND_TIME_S:
          self.authority = 1.0
          self._blending = False
          self._blend_s = 0.0
          self._quiet_s = 0.0

    self._set_ui_paused()
    return HandoffOutput(self.authority, self.ui_paused, self._yielded, self._blending)
