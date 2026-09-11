"""Pre-AP driver-wheel temporary lateral handoff.

Default On (NAPDriverLatHandoff=1). Settings → NAP can turn it Off if
gravel / crosswind still false-yields (gray chrome = latHandoffPaused,
lat authority cut).

History (on-car):
  #71  0.50 Nm / 80 ms   — gravel / rumble false-yield
  #73  0.85 Nm / 250 ms + 0.25 s quiet — rumble-safe, but a gentle
                           dodge push felt too slow/firm (risked
                           hands-on >= 2). 0.25 s quiet then felt
                           late on hand-back.
  now  0.70 Nm / 140 ms, quiet wait 0 — quicker, lighter push; 1 s
                           blend starts as soon as input is gone.

Signal (Pre-AP EPAS_sysStatus 0x370, tesla_preap.dbc):
  EPAS_torsionBarTorque  — continuous, Nm, factor 0.01, offset −20.5
  EPAS_handsOnLevel      — discrete 0/1/2/3
  StW_AnglHP_Spd         — steering-angle rate, deg/s, factor 0.5

Existing software / safety (unchanged):
  steeringPressed  = |torsion| > STEER_THRESHOLD (1.0 Nm), 5-frame debounce
  steerOverride    = EventName from steeringPressed (OVERRIDE_LATERAL)
  steeringDisengage / panda PREAP_HANDS_ON_DISENGAGE_LEVEL = handsOnLevel >= 2

Soft-yield is 0.70 Nm (70% of STEER_THRESHOLD) with 140 ms of *consecutive*
frames above that. Gaps reset the count (gravel spike trains do not
accumulate). 0.70 is well above the old 0.5 Nm rumble floor and still
below software steeringPressed (1.0 Nm). Release hysteresis stays
0.40 Nm (press-latch + "input still present" while yielded).
handsOnLevel is not the soft trigger. >= 2 stays the hard/safety path.

QUIET_WAIT_S is 0. As soon as soft-yield input is gone (below release /
not pressed) the 1 s smoothstep starts — no 0.25 s patience. While
yielded, torque above release is still on the rim (finish the dodge);
that is not a timer. Rate is ignored. During the blend only a renewed
>= 0.70 Nm firm push re-yields.

Pre-AP CS.steeringRateDeg is -STW_ANGLHP_STAT.StW_AnglHP_Spd (0x0E,
14-bit, factor 0.5, offset −4096, deg/s; SNA → ~4095 deg/s). Caster /
road / SNA stay above the old 25 deg/s gate.

While yielded, controlsd pins desired_curvature to measured (stock
inactive target) so LaC cannot run ahead of the wheel. Blend / resume
then clip_curvature from that pin onto the model with latActive still
true.

Blinker lat-pause is a different path: latActive is False, this module
resets to identity (no soft-yield). After the turn, stock resume used
to restore full latActive onto the model immediately — a firm grab
when the plan is wrong (parking lot / no lanes → grass). We pin to
measured while lat is down, and on blinker-pause rising edge start
the same 1 s blend (no quiet wait) so post-turn return is smooth.
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
# #71 0.5 Nm / 80 ms false-yielded on gravel. #73 0.85 Nm / 250 ms was
# rumble-safe but too slow/firm for a gentle dodge (hands-on >= 2 risk).
# 0.70 Nm / 140 ms is a quicker light push: still above rumble, still
# below steeringPressed (1.0 Nm / 5 frames). Hard path stays
# handsOnLevel >= 2.
SOFT_YIELD_TRIGGER_NM = 0.70 * float(STEER_THRESHOLD)  # 0.70 Nm
SOFT_YIELD_RELEASE_NM = 0.40 * float(STEER_THRESHOLD)  # 0.40 Nm, wider gap

# Consecutive frames above trigger. Gaps reset the count (spike reject).
SOFT_YIELD_DEBOUNCE_FRAMES = 14  # 140 ms at 100 Hz (was 25 / 250 ms)
SOFT_YIELD_RELEASE_FRAMES = 8    # 80 ms below release before clearing latch

# Historical rate gate (25 deg/s). Intentionally unused while yielded:
# on-car measured rate after release is caster / road, not driver intent.
# Kept so tests record the old number and that it must not block resume.
STEER_RATE_QUIET_DEG_S = 25.0

# --- timing / UI ---
# Justin: hand-back earlier. No quiet delay — 1 s blend starts the first
# frame input is gone (below release). Do not add patience.
QUIET_WAIT_S = 0.0
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
# Settings → NAP. Default On. Turn Off if gravel / wind still false-yields.
PARAM_DRIVER_LAT_HANDOFF = "NAPDriverLatHandoff"


def handoff_enabled(*, fingerprint: str, param_on: bool) -> bool:
  """Pre-AP and Settings toggle (param defaults On)."""
  return bool(param_on) and fingerprint == PREAP_FINGERPRINT


def smoothstep(t: float) -> float:
  t = float(np.clip(t, 0.0, 1.0))
  return t * t * (3.0 - 2.0 * t)


def apply_lat_authority(authority: float, torque: float, desired_angle: float,
                        measured_angle: float, desired_curvature: float,
                        measured_curvature: float) -> tuple[float, float, float]:
  """Scale lateral actuator authority. 0 follows the driver; 1 is full NAP.

  Identity at authority=1 (same as skipping the call). Do not write the
  blended curvature back into the planner state — that would hold LaC
  near measured after resume. controlsd skips this when authority is 1.
  """
  a = float(np.clip(authority, 0.0, 1.0))
  return (
    float(torque) * a,
    a * float(desired_angle) + (1.0 - a) * float(measured_angle),
    a * float(desired_curvature) + (1.0 - a) * float(measured_curvature),
  )


def handoff_new_desired_curvature(*, yielded: bool, lat_active: bool,
                                  model_curvature: float,
                                  measured_curvature: float) -> float:
  """clip_curvature target. Pin to the wheel while yielded.

  latActive stays true through yield so apply_steer_angle_limits_vm does
  not snap. If we still fed the model/plan as new_desired, desired_curvature
  would run to the lane while apply_lat_authority(0) commanded measured.
  After authority→1, LaC then jumped to that diverged angle and the Tesla
  VM / EPAS path felt firm-holding (green HUD, on-screen path back, wheel
  not tracking). Same target as stock latActive=False; snap the pin in
  controlsd (do not slew toward measured via clip_curvature).
  """
  if yielded or not lat_active:
    return float(measured_curvature)
  return float(model_curvature)


def pin_desired_curvature_to_measured(yielded: bool, lat_active: bool = True) -> bool:
  """True: assign desired_curvature = measured, skip clip_curvature.

  Pin while soft-yielded *and* while latActive is false (blinker pause /
  standstill / fault). Blinker pause used to only clip toward measured,
  so a fast lot turn left desired lagged; resume then slewed from that
  lag onto a bad model path (grass). Snap-pin matches stock inactive.
  """
  return bool(yielded) or not bool(lat_active)


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
  """Process-local latch: light wheel input yields lat; 1 s S-curve hands it back."""

  def __init__(self, enabled: bool = True):
    # Product default On. controlsd still requires Pre-AP + param
    # (NAPDriverLatHandoff defaults true). Toggle Off to disable.
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
    self._blinker_was_paused = False

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

  def _start_blend(self):
    self._yielded = False
    self._blending = True
    self._blend_s = 0.0
    self._quiet_s = 0.0
    self.authority = 0.0
    self.ui_paused = True

  def update(self, *, engaged: bool, lat_would_be_active: bool,
             steering_torque: float, steering_rate_deg: float,
             alc_active: bool = False, blinker_paused: bool = False,
             dt: float | None = None) -> HandoffOutput:
    if dt is None:
      dt = DT_CTRL

    if not self.enabled:
      self._reset()
      return HandoffOutput(1.0, False, False, False)

    # Blinker pause / standstill / faults already cleared lat_would_be_active.
    # ALC wheel-nudge uses steeringPressed at 1 Nm and must not be softened.
    # Full disengage (cancel / door / hands-on >= 2) clears engaged.
    # Soft-yield stays gated (identity) while lat is down — do not steal
    # the blinker-turn path. Remember a driver-turn pause so the rising
    # edge can 1 s blend instead of restoring authority=1 onto the model.
    if not engaged or alc_active:
      self._reset()
      return HandoffOutput(1.0, False, False, False)
    if not lat_would_be_active:
      remember = self._blinker_was_paused or bool(blinker_paused)
      self._reset()
      self._blinker_was_paused = remember
      return HandoffOutput(1.0, False, False, False)
    mag = abs(float(steering_torque))
    pressed = self._update_soft_pressed(steering_torque)
    # steering_rate_deg is CS.steeringRateDeg (−StW_AnglHP_Spd, deg/s).
    # Not a quiet/yield signal — see module docstring.
    _ = steering_rate_deg

    if self._blinker_was_paused:
      self._blinker_was_paused = False
      self._start_blend()
    elif not self._yielded and not self._blending:
      if pressed:
        self._enter_yield()
    elif self._yielded:
      # Stay yielded while a firm push is back, or while torque is still
      # above release (rim still loaded — finish the dodge). That is not
      # a quiet timer. Input gone (below release / not pressed) → 1 s
      # blend this frame. QUIET_WAIT_S is 0; do not add patience.
      if mag >= SOFT_YIELD_TRIGGER_NM:
        self._enter_yield()
      elif mag > SOFT_YIELD_RELEASE_NM:
        pass
      else:
        # QUIET_WAIT_S is 0: blend this frame (no patience).
        self._start_blend()
    elif self._blending:
      # Immediate on renewed ≥ trigger. Rate is ignored: the blend turns
      # the wheel. Use live torque, not the press latch. Mid-band torque
      # during return is OP/caster, not a new push.
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
