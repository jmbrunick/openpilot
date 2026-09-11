"""Pre-AP driver-wheel temporary lateral handoff.

Default On (NAPDriverLatHandoff=1). Settings → NAP can turn it Off.

Product: yield = *free the EPS*, not follow-the-rim angle control.
A light purposeful push with a hand on the rim should let the wheel
go. Gravel spike trains / wind / road-crown must not free-yield.

On-car (#79): entry still required a 0.70 Nm / 140 ms fight, *and*
after yield OP kept latActive=True with apply_lat_authority(0)
commanding measured angle (DAS_steeringControlType still 1). That is
closed-loop follow-the-rim — Justin could nudge off-path but the car
kept holding/steering. Wrestling was full authority until the debounce
finished, then more holding after "yield."

Yield now reuses the blinker lat-pause EPS release: latActive false →
carcontroller sends DAS_steeringControlType=0. Hands stay detected
via EPAS_handsOnLevel (and a light probe). Resume is still pin-to-
wheel while lat is down + the same 1 s blend; hands still on the rim
stay yielded (do not blend onto the model). Blinker rising-edge
re-entry is unchanged.

Intent (Pre-AP EPAS) — required to *enter* yield:
  EPAS_torsionBarTorque / CS.steeringTorque
      sustained directional torque (not short spikes)
  EPAS_handsOnLevel / hands stash
      hands on the rim (>= 1)
  Steer rate is *not* an entry gate. A sustained push yields even
      when the wheel is not moving (isometric fight). Rate is only a
      weak wind filter at *low* torsion — it never blocks a firm push
      and never promotes low torsion to intent.
  Disturbance veto only while torsion is *below* the yield trigger
      (wind / crown: high path error, low torsion). High torsion is
      intent even if tracking error is large (driver is leaving the path).

History (on-car):
  #71  0.50 Nm / 80 ms   — gravel / rumble false-yield (no hands gate)
  #73  0.85 Nm / 250 ms + 0.25 s quiet — rumble-safe, too slow/firm
  #74  0.70 Nm / 140 ms, quiet wait 0 — mid-dodge torsion dip blended
  #75  stay yielded while handsOnLevel >= 1; blend after hands off
  #78  entry = torsion + aligned rate + hands; blocked isometric fight
  #79  entry = 0.70 Nm / 140 ms + hands; still follow-measured hold
  now  entry = ~0.55 Nm / 90 ms + hands; yield frees the EPS.

Signal (Pre-AP EPAS_sysStatus 0x370, tesla_preap.dbc):
  EPAS_torsionBarTorque  — continuous, Nm, factor 0.01, offset −20.5
  EPAS_handsOnLevel      — discrete 0/1/2/3
  StW_AnglHP_Spd         — steering-angle rate, deg/s, factor 0.5
                           CS.steeringRateDeg is negated (same as torsion)

Existing software / safety (unchanged):
  steeringPressed  = |torsion| > STEER_THRESHOLD (1.0 Nm), 5-frame debounce
  steerOverride    = EventName from steeringPressed (OVERRIDE_LATERAL)
  steeringDisengage / panda PREAP_HANDS_ON_DISENGAGE_LEVEL = handsOnLevel >= 2

Soft-yield torsion floor is 0.55 Nm (closer to #71 gentleness, with
the hands gate #71 lacked). Consecutive *push* frames (gaps reset):
90 ms at the 0.55 floor, fewer as torsion approaches STEER_THRESHOLD
so the soft path beats hands-on >= 2. Never so soft that rumble with
hands resting (level 0, or ~0.50 Nm with level 1) free-yields.
Release hysteresis stays 0.40 Nm (press-latch only). handsOnLevel is
not the soft *trigger* by itself — it is required for intent and is
the hold while yielded. >= 2 stays the hard/safety path (panda
unchanged).

QUIET_WAIT_S stays 0. The 1 s smoothstep starts only after
handsOnLevel == 0 for HANDS_OFF_CONFIRM_S (~80 ms). Renewed hands-on
or a firm >= 0.55 Nm push cancels the blend and re-yields.

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

While yielded, latActive is false (EPS released) and controlsd pins
desired_curvature to measured so resume clips from the wheel. Blinker
lat-pause is a different path: this module resets to identity and
remembers the pause so the rising edge can 1 s blend (or stay yielded
if hands are still on).
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
# Gentler than #79's 0.70 / 140 ms so a light purposeful push yields
# before the driver wrestles. Hands-on is required so gravel / crown
# with hands off or resting do not count. Rate is not an entry gate.
SOFT_YIELD_TRIGGER_NM = 0.55 * float(STEER_THRESHOLD)  # 0.55 Nm
SOFT_YIELD_RELEASE_NM = 0.40 * float(STEER_THRESHOLD)  # 0.40 Nm, wider gap

# Consecutive *push* frames (torsion + hands). Gaps reset.
# 90 ms at the 0.55 floor; fewer near STEER_THRESHOLD so soft yield
# wins before hands-on >= 2 / steeringPressed. Fast floor stays above
# the 5-frame gravel-spike bursts used in tests (~50 ms).
SOFT_YIELD_DEBOUNCE_FRAMES = 9   # 90 ms at 100 Hz (0.55 Nm)
SOFT_YIELD_FAST_DEBOUNCE_FRAMES = 6  # 60 ms as |torsion| → 1.0 Nm
SOFT_YIELD_RELEASE_FRAMES = 8    # 80 ms below release before clearing latch

# Rate agreement. Pre-AP CS.steeringRateDeg is −StW_AnglHP_Spd (deg/s).
# SNA decodes to ~4095 deg/s. Kept as a *weak low-torsion* wind filter
# only — never required to enter yield on a firm sustained push.
# Historical 25 deg/s gate was a resume quiet check (unused since #75).
STEER_RATE_QUIET_DEG_S = 25.0
RATE_INTENT_MIN_DEG_S = 10.0
RATE_SNA_ABS_DEG_S = 400.0

# High |desired−measured| curvature with torsion *below* the yield
# trigger is wind / crown, not a driver dodge. High torsion is intent.
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

# Yield drops latActive so the EPS is free (DAS_steeringControlType=0).
# apply_steer_angle_limits_vm then snaps apply_angle to measured — that
# is the stock inactive path, same as blinker pause. Resume slews from
# that pin via the 1 s blend + Tesla VM limiter.
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
  """True when the wheel is turning in the torsion direction.

  Not an entry gate. Used only as a weak low-torsion wind filter —
  aligned rate below the yield trigger never counts as intent, and a
  firm push yields even when this is False (isometric fight / SNA).
  """
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


def required_press_frames(torque_nm: float) -> int:
  """Consecutive push frames needed to enter yield.

  90 ms at the 0.55 Nm floor. Linearly fewer toward 60 ms as |torsion|
  approaches STEER_THRESHOLD so the soft path beats hands-on >= 2.
  Never below SOFT_YIELD_FAST_DEBOUNCE_FRAMES (gravel 5-frame spikes).
  """
  mag = abs(float(torque_nm))
  lo = SOFT_YIELD_TRIGGER_NM
  hi = float(STEER_THRESHOLD)
  if mag <= lo + 1e-12:
    return SOFT_YIELD_DEBOUNCE_FRAMES
  if mag >= hi - 1e-12:
    return SOFT_YIELD_FAST_DEBOUNCE_FRAMES
  t = (mag - lo) / (hi - lo)
  frames = SOFT_YIELD_DEBOUNCE_FRAMES + t * (
    SOFT_YIELD_FAST_DEBOUNCE_FRAMES - SOFT_YIELD_DEBOUNCE_FRAMES)
  return int(round(frames))


def is_disturbance(*, torque_nm: float, rate_deg: float,
                   tracking_error: float) -> bool:
  """Wind / crown: high path error while torsion is below the yield trigger.

  High torsion is driver intent even if tracking error is large (leaving
  the path) and even if steer rate is low (isometric fight). Rate is a
  weak filter only at low torsion: aligned rate below the trigger never
  clears this and never promotes low torsion to a yield.
  """
  if abs(float(tracking_error)) < DISTURBANCE_CURVATURE_ERR:
    return False
  if abs(float(torque_nm)) >= SOFT_YIELD_TRIGGER_NM:
    return False
  # Low torsion + high error. Rate is unused here on purpose: aligned
  # rate below the trigger must not clear a disturbance (#78 required
  # alignment to *not* be a disturbance) and cannot promote this to yield.
  _ = rate_deg
  return True


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


def lat_active_after_handoff(lat_would_be_active: bool, yielded: bool) -> bool:
  """EPS request bit after blinker / standstill / soft-yield.

  Blinker pause and standstill already cleared lat_would_be_active.
  Soft yield does the same: latActive false so Pre-AP carcontroller
  sends DAS_steeringControlType=0 and apply_steer_angle_limits_vm
  tracks the wheel without commanding it. That is free-wheel, not
  follow-measured angle control with latActive still true.
  """
  return bool(lat_would_be_active) and not bool(yielded)


def apply_lat_authority(authority: float, torque: float, desired_angle: float,
                        measured_angle: float, desired_curvature: float,
                        measured_curvature: float) -> tuple[float, float, float]:
  """Scale lateral actuator authority. Resume blend only; not the yield.

  Yield frees the EPS (latActive false). This blend runs when lat is
  back and authority < 1. Identity at authority=1 (same as skipping).
  Do not write blended curvature back into the planner state.
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
  """clip_curvature target. Pin to the wheel while lat is down.

  Yield and blinker pause both clear latActive (EPS released). Pin so
  resume clips from the wheel, not a model path that ran ahead. Same
  target as stock latActive=False; snap the pin in controlsd.
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
  """Process-local latch: free-wheel yield; 1 s S-curve hands it back."""

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
    """Sustained firm torsion + hands. Rate does not gate a firm push.

    Gaps reset the count (gravel spike trains do not accumulate). High
    tracking error is a disturbance only while torsion is below the
    yield trigger (wind / crown). High torsion is intent even if the
    path error is large and steer rate is near zero (isometric fight).
    Near STEER_THRESHOLD the consecutive-frame bar drops so soft yield
    beats hands-on >= 2.
    """
    mag = abs(float(torque_nm))
    firm = hands_on and mag >= SOFT_YIELD_TRIGGER_NM
    if is_disturbance(torque_nm=torque_nm, rate_deg=rate_deg,
                      tracking_error=tracking_error):
      firm = False
    if firm:
      self._press_cnt = min(self._press_cnt + 1, SOFT_YIELD_DEBOUNCE_FRAMES + 1)
      self._release_cnt = 0
    else:
      self._press_cnt = 0
      if mag <= SOFT_YIELD_RELEASE_NM:
        self._release_cnt = min(self._release_cnt + 1, SOFT_YIELD_RELEASE_FRAMES + 1)

    if self._press_cnt >= required_press_frames(torque_nm):
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

    # lat_would_be_active is the *pre-yield* request (blinker / standstill
    # / faults). Caller drops CC.latActive after this update when yielded
    # so the EPS is free — do not feed that dropped bit back in or we
    # reset to identity and forget we yielded.
    # ALC wheel-nudge uses steeringPressed at 1 Nm and must not be softened.
    # Full disengage (cancel / door / hands-on >= 2) clears engaged.
    # Soft-yield stays gated (identity) while blinker already paused lat.
    # Remember a driver-turn pause so the rising edge can 1 s blend
    # instead of restoring authority=1 onto the model.
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
