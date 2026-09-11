"""Pre-AP driver-wheel temporary lateral handoff.

Default On (NAPDriverLatHandoff=1). Settings → NAP can turn it Off.

Product (2026-09-11): yield lateral only on *driver intent to turn the
wheel* (typically avoiding something). Gravel / wind / road-crown
pressure must not gray the chrome.

Intent (Pre-AP EPAS) — all required to *enter* yield:
  EPAS_torsionBarTorque / CS.steeringTorque
      sustained directional torque (not short spikes)
  EPAS_handsOnLevel / hands stash
      hands on the rim (>= 1)
  CS.steeringRateDeg aligned with torsion
      same sign, above a quiet floor, not SNA
  Optional veto: high path/tracking error without matching sustained
      torsion → disturbance, do not yield

History (on-car):
  #71  0.50 Nm / 80 ms   — gravel / rumble false-yield
  #73  0.85 Nm / 250 ms + 0.25 s quiet — rumble-safe, too slow/firm
  #74  0.70 Nm / 140 ms, quiet wait 0 — mid-dodge torsion dip blended
  #75  stay yielded while handsOnLevel >= 1; blend after hands off
  now  same hold / 1 s resume, but *entry* is intent (torsion+rate+hands),
       not torsion-only. Emergency/hard brake while yielded (or shortly
       after yield entry) fully cancels OP — not the silent long pause.

Signal (Pre-AP EPAS_sysStatus 0x370, tesla_preap.dbc):
  EPAS_torsionBarTorque  — continuous, Nm, factor 0.01, offset −20.5
  EPAS_handsOnLevel      — discrete 0/1/2/3
  StW_AnglHP_Spd         — steering-angle rate, deg/s, factor 0.5
                           CS.steeringRateDeg is negated (same as torsion)

Existing software / safety (unchanged):
  steeringPressed  = |torsion| > STEER_THRESHOLD (1.0 Nm), 5-frame debounce
  steerOverride    = EventName from steeringPressed (OVERRIDE_LATERAL)
  steeringDisengage / panda PREAP_HANDS_ON_DISENGAGE_LEVEL = handsOnLevel >= 2

Soft-yield torsion floor stays 0.70 Nm with 140 ms of *consecutive*
intent frames (gaps reset). 0.70 is above the old 0.5 Nm rumble floor
and still below software steeringPressed (1.0 Nm). Release hysteresis
stays 0.40 Nm (press-latch only). handsOnLevel is not the soft *trigger*
by itself — it is required for intent and is the hold while yielded.
>= 2 stays the hard/safety path (panda unchanged).

QUIET_WAIT_S stays 0. The 1 s smoothstep starts only after
handsOnLevel == 0 for HANDS_OFF_CONFIRM_S (~80 ms). Renewed hands-on
or a firm >= 0.70 Nm push cancels the blend and re-yields.

Emergency / hard brake (see emergency_brake() and docs-nap/engagement.md):
  Pre-AP has no analog brake pressure on parsed buses — only digital
  Applied (DI_brakePedal / BrakeMessage.driverBrakeStatus). Light brake
  is that digital bit and still uses the existing sticky-MAX silent long
  pause + one SET. Emergency is digital Applied AND measured aEgo at or
  below EMERGENCY_DECEL_MPS2 (−3.5 m/s², ~0.36 g) for
  EMERGENCY_DECEL_FRAMES (80 ms), at vEgo >= 1 m/s, while yielded /
  blending / within YIELD_EMERGENCY_WINDOW_S of yield entry.
  That cannot come from NAP long (planner comfort Accel-5 is −0.80 m/s²;
  pre-AP clip is −1.5 m/s²). Full cancel is a flag here; card tears down
  the session (cruiseEnabled) so pcmDisable / disengage chime fire.
  This module does not call the silent long-pause path.

While yielded, controlsd pins desired_curvature to measured so LaC
cannot run ahead of the wheel. Blinker lat-pause is a different path:
latActive is False, this module resets to identity (no soft-yield).
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
# Torsion floor is unchanged from #74/#75. Intent adds rate agreement
# and hands-on so gravel spikes / crown pressure do not count.
SOFT_YIELD_TRIGGER_NM = 0.70 * float(STEER_THRESHOLD)  # 0.70 Nm
SOFT_YIELD_RELEASE_NM = 0.40 * float(STEER_THRESHOLD)  # 0.40 Nm, wider gap

# Consecutive *intent* frames (torsion + hands + aligned rate). Gaps reset.
SOFT_YIELD_DEBOUNCE_FRAMES = 14  # 140 ms at 100 Hz
SOFT_YIELD_RELEASE_FRAMES = 8    # 80 ms below release before clearing latch

# Rate agreement. Pre-AP CS.steeringRateDeg is −StW_AnglHP_Spd (deg/s).
# SNA decodes to ~4095 deg/s and must never count as intent.
# Historical 25 deg/s gate was a resume quiet check (unused since #75).
STEER_RATE_QUIET_DEG_S = 25.0
RATE_INTENT_MIN_DEG_S = 10.0
RATE_SNA_ABS_DEG_S = 400.0

# High |desired−measured| curvature without matching sustained torsion
# is wind / tracking fight, not a driver dodge.
DISTURBANCE_CURVATURE_ERR = 0.0025

# --- emergency / hard brake (digital Applied + measured decel) ---
# Light brake = digital Applied only → sticky-MAX silent long pause.
# Emergency cannot use pressure (Pre-AP DBC has none on parsed buses).
EMERGENCY_DECEL_MPS2 = -3.5
EMERGENCY_DECEL_FRAMES = 8  # 80 ms; reject pothole aEgo spikes
EMERGENCY_MIN_V_EGO = 1.0   # m/s; standstill KF chatter
YIELD_EMERGENCY_WINDOW_S = 2.0

# --- timing / UI ---
QUIET_WAIT_S = 0.0
HANDS_ON_HOLD_LEVEL = 1
HANDS_OFF_CONFIRM_S = 0.08  # 80 ms at 100 Hz
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


def hands_still_on(hands_on_level: int) -> bool:
  """True while EPAS still sees a hand on the rim (level 1+)."""
  return int(hands_on_level or 0) >= HANDS_ON_HOLD_LEVEL


def cs_hands_on_level(CS) -> int:
  """EPAS_handsOnLevel 0/1/2/3 from cereal CarState.

  Prefer CS.handsOnLevel when the schema has it. Pre-AP card also stashes
  the discrete level on steeringTorqueEps (unused on this angle car) so
  controlsd works without an opendbc cereal bump. steeringDisengage is
  hands >= 2.
  """
  vals: list[int] = []
  if hasattr(CS, 'handsOnLevel'):
    try:
      vals.append(int(CS.handsOnLevel or 0))
    except (TypeError, ValueError):
      pass
  try:
    ev = int(round(float(getattr(CS, 'steeringTorqueEps', 0.0) or 0.0)))
    if 0 <= ev <= 3:
      vals.append(ev)
  except (TypeError, ValueError):
    pass
  if getattr(CS, 'steeringDisengage', False):
    vals.append(2)
  return max(vals) if vals else 0


def cs_real_brake_pressed(CS) -> bool:
  """Digital driver brake without using CS.brakePressed.

  Pre-AP forces brakePressed=False so generic OP brake-to-disengage never
  fires. Card publishes the Applied flag on CS.brake (1.0 / 0.0).
  """
  if bool(getattr(CS, 'realBrakePressed', False)):
    return True
  try:
    return float(getattr(CS, 'brake', 0.0) or 0.0) >= 0.5
  except (TypeError, ValueError):
    return False


def torque_rate_aligned(torque_nm: float, rate_deg: float) -> bool:
  """True when the driver is turning the wheel in the torsion direction."""
  torque = float(torque_nm)
  rate = float(rate_deg)
  if not np.isfinite(torque) or not np.isfinite(rate):
    return False
  if abs(rate) >= RATE_SNA_ABS_DEG_S:
    return False
  if abs(rate) < RATE_INTENT_MIN_DEG_S:
    return False
  if abs(torque) < SOFT_YIELD_TRIGGER_NM:
    return False
  return (torque * rate) > 0.0


def is_disturbance(*, torque_nm: float, rate_deg: float,
                   tracking_error: float) -> bool:
  """High tracking effort without matching sustained driver torsion.

  Wind / crown: LaC fights the path (error high) while torsion is low or
  not rate-aligned. A real dodge has matching sustained torsion — that
  is not a disturbance even if error is large (driver is leaving the path).
  """
  if abs(float(tracking_error)) < DISTURBANCE_CURVATURE_ERR:
    return False
  return not torque_rate_aligned(torque_nm, rate_deg)


def emergency_brake(*, brake_applied: bool, a_ego: float, v_ego: float,
                    decel_frames: int) -> bool:
  """Hard brake: digital Applied + sustained emergency-level aEgo.

  Not light brake. Not OP map-track decel. Not a lone aEgo pothole spike.
  """
  if not brake_applied:
    return False
  if float(v_ego) < EMERGENCY_MIN_V_EGO:
    return False
  if float(a_ego) > EMERGENCY_DECEL_MPS2:
    return False
  return int(decel_frames) >= EMERGENCY_DECEL_FRAMES


def emergency_cancel_active(*, yielded: bool, blending: bool,
                            yield_age_s: float | None,
                            hard_brake: bool) -> bool:
  """Full OP cancel when hard-braking in the soft-yield takeover window."""
  if not hard_brake:
    return False
  if yielded or blending:
    return True
  if yield_age_s is None:
    return False
  return 0.0 <= float(yield_age_s) <= YIELD_EMERGENCY_WINDOW_S


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
  emergency_cancel: bool = False


class DriverLateralHandoff:
  """Process-local latch: driver-intent yield; 1 s S-curve hands it back."""

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
    self._hands_off_s = 0.0
    self._decel_cnt = 0
    self._yield_age_s: float | None = None

  def reset(self):
    self._reset()

  def _update_intent(self, torque_nm: float, rate_deg: float,
                     hands_on: bool, tracking_error: float) -> bool:
    """Sustained directional torsion + hands + rate agreement.

    Gaps reset the count (gravel spike trains do not accumulate). High
    tracking error without matching torsion is a disturbance, not intent.
    """
    mag = abs(float(torque_nm))
    aligned = hands_on and torque_rate_aligned(torque_nm, rate_deg)
    if is_disturbance(torque_nm=torque_nm, rate_deg=rate_deg,
                      tracking_error=tracking_error):
      aligned = False
    if mag >= SOFT_YIELD_TRIGGER_NM and aligned:
      self._press_cnt = min(self._press_cnt + 1, SOFT_YIELD_DEBOUNCE_FRAMES + 1)
      self._release_cnt = 0
    else:
      self._press_cnt = 0
      if mag <= SOFT_YIELD_RELEASE_NM:
        self._release_cnt = min(self._release_cnt + 1, SOFT_YIELD_RELEASE_FRAMES + 1)

    if self._press_cnt >= SOFT_YIELD_DEBOUNCE_FRAMES:
      self._pressed = True
    if self._pressed and self._release_cnt >= SOFT_YIELD_RELEASE_FRAMES:
      self._pressed = False
    return self._pressed

  def _update_emergency(self, *, brake_applied: bool, a_ego: float,
                        v_ego: float, dt: float) -> bool:
    hard = (
      bool(brake_applied)
      and float(v_ego) >= EMERGENCY_MIN_V_EGO
      and float(a_ego) <= EMERGENCY_DECEL_MPS2
    )
    if hard:
      self._decel_cnt = min(self._decel_cnt + 1, EMERGENCY_DECEL_FRAMES + 1)
    else:
      self._decel_cnt = 0

    if self._yielded or self._blending:
      if self._yield_age_s is None:
        self._yield_age_s = 0.0
      self._yield_age_s += dt
    elif self._yield_age_s is not None:
      self._yield_age_s += dt
      if self._yield_age_s > YIELD_EMERGENCY_WINDOW_S:
        self._yield_age_s = None

    return emergency_cancel_active(
      yielded=self._yielded,
      blending=self._blending,
      yield_age_s=self._yield_age_s,
      hard_brake=emergency_brake(
        brake_applied=brake_applied, a_ego=a_ego, v_ego=v_ego,
        decel_frames=self._decel_cnt),
    )

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
    self._hands_off_s = 0.0
    self.authority = 0.0
    self.ui_paused = True
    if self._yield_age_s is None:
      self._yield_age_s = 0.0

  def _start_blend(self):
    self._yielded = False
    self._blending = True
    self._blend_s = 0.0
    self._quiet_s = 0.0
    self._hands_off_s = 0.0
    self.authority = 0.0
    self.ui_paused = True

  def _identity(self, *, emergency_cancel: bool = False) -> HandoffOutput:
    return HandoffOutput(1.0, False, False, False, emergency_cancel)

  def update(self, *, engaged: bool, lat_would_be_active: bool,
             steering_torque: float, steering_rate_deg: float,
             alc_active: bool = False, blinker_paused: bool = False,
             hands_on_level: int = 0, tracking_error: float = 0.0,
             brake_applied: bool = False, a_ego: float = 0.0,
             v_ego: float = 15.0, dt: float | None = None) -> HandoffOutput:
    if dt is None:
      dt = DT_CTRL

    if not self.enabled:
      self._reset()
      return self._identity()

    # Blinker pause / standstill / faults already cleared lat_would_be_active.
    # ALC wheel-nudge uses steeringPressed at 1 Nm and must not be softened.
    # Full disengage (cancel / door / hands-on >= 2) clears engaged.
    # Soft-yield stays gated (identity) while lat is down — do not steal
    # the blinker-turn path. Remember a driver-turn pause so the rising
    # edge can 1 s blend instead of restoring authority=1 onto the model.
    if not engaged or alc_active:
      self._reset()
      return self._identity()
    if not lat_would_be_active:
      remember = self._blinker_was_paused or bool(blinker_paused)
      self._reset()
      self._blinker_was_paused = remember
      return self._identity()

    mag = abs(float(steering_torque))
    pressed = self._update_intent(
      steering_torque, steering_rate_deg,
      hands_still_on(hands_on_level), tracking_error)
    hands_on = hands_still_on(hands_on_level)
    firm_push = mag >= SOFT_YIELD_TRIGGER_NM

    if self._blinker_was_paused:
      self._blinker_was_paused = False
      # Blinker pause already waited for steeringPressed release. Keep
      # the #74 1 s re-entry. If a hand is still on the rim, yield
      # instead of blending toward the model.
      if hands_on or firm_push:
        self._enter_yield()
      else:
        self._start_blend()
    elif not self._yielded and not self._blending:
      if pressed:
        self._enter_yield()
    elif self._yielded:
      # Stay yielded while still maneuvering: hands on the rim OR a
      # renewed firm push. Mid-dodge torsion dips (below release) must
      # not start the blend. Hands 0 for ~80 ms → 1 s smoothstep.
      if hands_on or firm_push:
        self._enter_yield()
      else:
        self._hands_off_s += dt
        if self._hands_off_s + 1e-12 >= HANDS_OFF_CONFIRM_S:
          self._start_blend()
    elif self._blending:
      # Hands back on or a firm push cancels the return. Mid-band
      # torque during the blend is OP/caster, not a new push. Re-yield
      # does not re-require rate agreement (already in the maneuver).
      if hands_on or firm_push:
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

    emergency = self._update_emergency(
      brake_applied=brake_applied, a_ego=a_ego, v_ego=v_ego, dt=dt)
    self._set_ui_paused()
    return HandoffOutput(
      self.authority, self.ui_paused, self._yielded, self._blending,
      emergency)
