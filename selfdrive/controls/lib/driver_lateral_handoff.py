"""Pre-AP driver-wheel temporary lateral handoff.

DEFAULT OFF. On-car gravel / crosswind produced gray 3X chrome
(latHandoffPaused) and OP stopped fighting the wind — false soft-yield
cutting lat authority. NAPDriverLatHandoff and this constructor default
Off. Do not enable for driving. Settings → NAP is opt-in for testing
only. Resume-fix / firmer detector must not ship enabled.

Signal (Pre-AP EPAS_sysStatus 0x370, tesla_preap.dbc):
  EPAS_torsionBarTorque  — continuous, Nm, factor 0.01, offset −20.5
  EPAS_handsOnLevel      — discrete 0/1/2/3
  StW_AnglHP_Spd         — steering-angle rate, deg/s, factor 0.5

Existing software / safety (unchanged):
  steeringPressed  = |torsion| > STEER_THRESHOLD (1.0 Nm), 5-frame debounce
  steerOverride    = EventName from steeringPressed (OVERRIDE_LATERAL)
  steeringDisengage / panda PREAP_HANDS_ON_DISENGAGE_LEVEL = handsOnLevel >= 2

Soft-yield is 0.85 Nm (85% of STEER_THRESHOLD) with 250 ms of *consecutive*
frames above that — not 0.5 Nm / 80 ms. Gravel spikes and short rumble
do not accumulate. Steady crosswind torsion is typically well below
0.85 Nm; a constant 0.85+ for 250 ms is treated as a hand push.
Release hysteresis is 0.40 Nm (wider gap). handsOnLevel is not the
soft trigger. >= 2 stays the hard/safety path.

Quiet while yielded aborts only on a renewed >= 0.85 Nm trigger. The
0.40 Nm band and CS.steeringRateDeg must not zero _quiet_s.
Pre-AP CS.steeringRateDeg is -STW_ANGLHP_STAT.StW_AnglHP_Spd (0x0E,
14-bit, factor 0.5, offset −4096, deg/s; SNA → ~4095 deg/s). Caster /
road / SNA stay above the old 25 deg/s gate. Rate is ignored while
yielded and during the 1 s blend.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from cereal import log
from opendbc.car.tesla.values import STEER_THRESHOLD

# Match controlsd / card (openpilot.common.realtime.DT_CTRL).
DT_CTRL = 0.01

State = log.SelfdriveState.OpenpilotState

# --- thresholds (from real Pre-AP STEER_THRESHOLD = 1.0 Nm) ---
# 0.5 Nm / 80 ms false-yielded on gravel rumble + crosswind. 0.85 Nm is a
# firm hand push: above rumble/wind, at or just below steeringPressed
# (1.0 Nm / 5 frames) so we do not require a yank. Hard path stays
# handsOnLevel >= 2.
SOFT_YIELD_TRIGGER_NM = 0.85 * float(STEER_THRESHOLD)  # 0.85 Nm
SOFT_YIELD_RELEASE_NM = 0.40 * float(STEER_THRESHOLD)  # 0.40 Nm, wider gap

# Consecutive frames above trigger. Gaps reset the count (spike reject).
SOFT_YIELD_DEBOUNCE_FRAMES = 25  # 250 ms at 100 Hz
SOFT_YIELD_RELEASE_FRAMES = 8    # 80 ms below release before clearing latch

# Historical rate gate (25 deg/s). Intentionally unused while yielded:
# on-car measured rate after release is caster / road, not driver intent.
# Kept so tests record the old number and that it must not block resume.
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
# Settings → NAP. Default Off. Gray chrome on gravel/wind = false yield.
PARAM_DRIVER_LAT_HANDOFF = "NAPDriverLatHandoff"


def handoff_enabled(*, fingerprint: str, param_on: bool) -> bool:
  """Opt-in only. Param defaults Off; Pre-AP alone must not arm this."""
  return bool(param_on) and fingerprint == PREAP_FINGERPRINT


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

  def __init__(self, enabled: bool = False):
    # Default Off: vibration / aero load on the torsion bar can
    # false-yield. Enable only via NAPDriverLatHandoff for testing.
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
      self._press_cnt = min(self._press_cnt + 1, SOFT_YIELD_DEBOUNCE_FRAMES + 1)
      self._release_cnt = 0
    else:
      # Any gap below trigger is a rumble impulse, not a sustained push.
      self._press_cnt = 0
      if mag <= SOFT_YIELD_RELEASE_NM:
        self._release_cnt = min(self._release_cnt + 1, SOFT_YIELD_RELEASE_FRAMES + 1)

    if self._press_cnt >= SOFT_YIELD_DEBOUNCE_FRAMES:
      self._pressed = True
    if self._pressed and self._release_cnt >= SOFT_YIELD_RELEASE_FRAMES:
      self._pressed = False
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
      self._reset()
      return HandoffOutput(1.0, False, False, False)

    # Blinker pause / standstill / faults already cleared lat_would_be_active.
    # ALC wheel-nudge uses steeringPressed at 1 Nm and must not be softened.
    # Full disengage (cancel / door / hands-on >= 2) clears engaged.
    if not engaged or not lat_would_be_active or alc_active:
      self._reset()
      return HandoffOutput(1.0, False, False, False)

    mag = abs(float(steering_torque))
    pressed = self._update_soft_pressed(steering_torque)
    # steering_rate_deg is CS.steeringRateDeg (−StW_AnglHP_Spd, deg/s).
    # Not a quiet/yield signal — see module docstring.
    _ = steering_rate_deg

    if not self._yielded and not self._blending:
      if pressed:
        self._enter_yield()
    elif self._yielded:
      # Only a renewed ≥ trigger (0.85 Nm) aborts quiet. The 0.40 Nm
      # hysteresis band and rate must not call _enter_yield().
      if mag >= SOFT_YIELD_TRIGGER_NM:
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
      # Immediate on renewed ≥ trigger. Rate is ignored: the blend turns
      # the wheel. Use live torque, not the press latch.
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
