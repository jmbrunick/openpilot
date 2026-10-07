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
stay yielded (do not blend onto the model).

Driver-turn blinker (soft-lat On): lamp latch must not strip lat.
Keep control if we still have it. Soft-lat may still yield on a
driver push. While the driver-turn blinker is latched, do not
re-enable (stay yielded / do not finish a take-back blend). After
it clears, land in yield and use the normal 0.15 s hands-off
confirm + 1 s blend — no dedicated blinker rising-edge blend.
The same re-enable inhibit also applies whenever v_ego is strictly
below 10 mph (LAT_REENABLE_MIN_V_EGO_MPH), blinker or not. Above
10 mph with the blinker off, resume is unchanged. Blinker-on still
blocks take-back at any speed. Soft-lat Off keeps the stock
lamp-latch lat pause so turns still free the wheel.

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
  #80  entry = ~0.55 Nm / 90 ms + hands; yield frees the EPS
  now  hands-off confirm 0.15 s before the 1 s blend (crossover gap)

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

QUIET_WAIT_S stays 0 (no torsion-quiet patience — that blended on
mid-dodge dips before #75). The 1 s smoothstep starts only after
handsOnLevel == 0 for HANDS_OFF_CONFIRM_S (~0.15 s). A brief
hands-off or direction-change during the dodge must not start
take-back. Renewed hands-on or a firm >= 0.55 Nm push during that
wait or the blend cancels and re-yields (delay resets).

Fight hold (one-sided tug). A single dodge still resumes on the
0.15 s confirm + 1 s blend. After FIGHT_YIELD_COUNT same-direction
yields inside FIGHT_WINDOW_S, or one-sided torque at/above
FIGHT_TORQUE_NM for FIGHT_TORQUE_HOLD_S while a yield is recent,
resume waits until torque has been below FIGHT_QUIET_NM for
FIGHT_QUIET_S and |model curvature − measured| has been under
FIGHT_CURVATURE_ERR for FIGHT_CURVATURE_HOLD_S. That re-take uses
FIGHT_BLEND_RATE_PER_S (still finished inside BLEND_TIME_S, so the
inference grace is unchanged). Callers that omit model/measured
curvature keep the previous resume. BLEND_TIME_S and
HANDS_OFF_CONFIRM_S are not changed.
Fight evidence does not accumulate while a driver-turn blinker is
on, while v_ego is under 10 mph, or for FIGHT_POST_INHIBIT_S after
either ends. Thresholds are relative to a resting offset learned
only while hands are off and the wheel is quiet, clamped to
±REST_BIAS_CLAMP_NM. Hands-quiet is hands-on level 0, steeringPressed
false, and |torque − offset| < FIGHT_QUIET_NM. After FIGHT_MAX_HOLD_S
of that quiet the slow re-take starts even if curvature has not
agreed. An in-session stalk pull clears the latch and starts the
normal 1 s blend.

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
desired_curvature to measured so resume clips from the wheel. A
driver-turn blinker is not a lat-down by itself when soft-lat is On:
this module treats it as a re-enable inhibit and, on the falling
edge after a yield / lat-down, enters yield so the normal resume
owns the take-back. v_ego below 10 mph uses that same inhibit
(OR, not a parallel path). Soft-lat Off still uses BlinkerLateralHold
to clear latActive on lamp latch.

INFERENCE CONTRACT (lateral yield is inferred, never announced):
  Panda and the card decide "OP has yielded lateral" (a wheel yank is a
  driver maneuver, not a cancel) from the 0x488 DAS_steeringControlType=1
  send history alone: opendbc preap/lat_yield.py and
  safety/modes/tesla_preap_latyield.h. That inference assumes, about THIS
  module and controlsd:
    * latActive (hence type 1) is False for the whole yield, and True when
      OP steers, including the take-back blend at authority < 1;
    * a yield lasts at least HANDS_OFF_CONFIRM_S before the blend starts;
    * the blend lasts BLEND_TIME_S and the re-arm grace covers it.
  ANY change to yield / blend / confirm timing, to what makes latActive
  false (new release path, inhibit, pause) or true (new early re-arm), or
  to what authority does MUST update the inference windows and the panda
  header. selfdrive/controls/lib/tests/test_lat_yield_inference_guard.py
  fails when BLEND_TIME_S / HANDS_OFF_CONFIRM_S and the lat_yield windows
  diverge; fix lat_yield.py + tesla_preap_latyield.h first, then the pins.

WHEEL-RESUME GATE (rapid back-and-forth turns). A driver who lets the wheel
uncoil through light / open hands (roundabout, switchback, parking lot) has
handsOnLevel 0 while the rim is still far from straight. The hands-off
confirm used to start the 1 s take-back blend there, so OP grabbed lateral
mid-uncoil and fought the next turn. Now, once the wheel angle since the
yield began has exceeded WHEEL_GATE_ARM_DEG, neither resume path may start
the blend until |angle| <= WHEEL_RESUME_STRAIGHT_DEG (named, 10-15 deg) for
WHEEL_STRAIGHT_DWELL_S (a swing through centre into the next turn is not
"straight").
Speed does not matter (lots, and a 10 mph crossing with the wheel at lock).
The yield simply lasts longer (latActive stays false the whole time), which
the inference contract allows (only a *minimum* yield length is assumed).
Hands back on, or a firm push, behave exactly as before. Safety valve:
after WHEEL_GATE_MAX_HOLD_S of hands-off hold the gate lets go so a wheel
that never centres cannot leave lateral off forever. steering_angle_deg=None
(not wired / old callers) disables the gate: bit-identical to before.
ALC / lane-change turn / disengage reset the whole latch (gate included).

Early yield (steeringPressed, no hands gate): EPAS_handsOnLevel lags the
torsion bar, so a firm push can reach hands >= 2 before the hands-gated
soft yield above has fired. EARLY_YIELD_FRAMES consecutive steeringPressed
frames of same-sign torque >= EARLY_YIELD_NM yield without the hands gate,
so the wheel input lands on a yielded lateral (B) rather than a full one (A).
Tuned conservatively against Oct 1-3 qlogs: 1 Nm / 50 ms false-yielded on
road chatter; a true first-contact yank (<= 0.2 s to level 3) still reaches
level 3 on full lateral and cancels by design.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from cereal import log
from opendbc.car.tesla.values import STEER_THRESHOLD
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.lane_change_nudge import (
  tipped_lane_change_driver_release,
)

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

# Early yield on steeringPressed, no EPAS hands gate (see module docstring).
# Same-sign torsion at/above EARLY_YIELD_NM while steeringPressed for
# EARLY_YIELD_FRAMES consecutive frames (100 Hz) enters the same yield.
EARLY_YIELD_NM = 2.0
EARLY_YIELD_FRAMES = 8  # 80 ms

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

# Soft-lat re-enable inhibit floor (strictly below). Same gate as a
# driver-turn blinker latch: keep control if we still have it; already
# yielded / blending stay yielded; do not finish a take-back blend.
# Blinker-on still inhibits at any speed. Soft-lat Off is identity
# (BlinkerLateralHold still pauses lat on lamp latch only). Long /
# ALC tip-hold / FCW / AEB are not this path.
LAT_REENABLE_MIN_V_EGO_MPH = 10.0
LAT_REENABLE_MIN_V_EGO = LAT_REENABLE_MIN_V_EGO_MPH * CV.MPH_TO_MS

# --- wheel-resume gate (see WHEEL-RESUME GATE in the module docstring) ---
# No resume while |wheel| is above this after a big-angle handoff. 10-15 deg.
WHEEL_RESUME_STRAIGHT_DEG = 12.0
# Peak |wheel| since the yield began that arms the gate (a real turn, not a
# highway nudge). Below this the resume is unchanged.
WHEEL_GATE_ARM_DEG = 30.0
# The wheel must stay within WHEEL_RESUME_STRAIGHT_DEG this long (not just
# sweep through centre between two turns) before the blend may start.
WHEEL_STRAIGHT_DWELL_S = 0.15
# Hands-off time the gate may hold the resume before it lets go.
WHEEL_GATE_MAX_HOLD_S = 6.0

# --- timing / UI ---
QUIET_WAIT_S = 0.0
HANDS_ON_HOLD_LEVEL = 1
# After hands are fully off: ~0.15 s before the 1 s blend.
# Not the old 0.08 s confirm (crossover snatch) and not torsion-quiet.
# Shorter than the 0.25 s confirm — Justin wanted less than a quarter second.
HANDS_OFF_CONFIRM_S = 0.15
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
# With BLEND_TIME_S == 1 that is also the max authority rise per second.
SMOOTHSTEP_MAX_SLOPE = 1.5
BLEND_AUTHORITY_RATE_PER_S = SMOOTHSTEP_MAX_SLOPE / BLEND_TIME_S

# One-sided fight: stay yielded instead of sawtoothing back onto a
# path the driver is refusing. Timing of a normal blend is unchanged.
FIGHT_YIELD_COUNT = 3
FIGHT_WINDOW_S = 20.0
FIGHT_TORQUE_NM = 0.30
FIGHT_TORQUE_HOLD_S = 5.0
FIGHT_QUIET_NM = 0.20
FIGHT_QUIET_S = 2.0
FIGHT_CURVATURE_ERR = 0.0015
FIGHT_CURVATURE_HOLD_S = 0.30
# Gentler than the smoothstep peak (1.5/s) and still reaches 1 inside
# BLEND_TIME_S, so panda's re-arm grace still covers the re-take.
FIGHT_BLEND_RATE_PER_S = 1.15
# No fight evidence while a turn blinker is on, under 10 mph, or for
# this long after either ends. A held turn is not a tug-of-war.
FIGHT_POST_INHIBIT_S = 1.5
# Hands-quiet this long starts the slow re-take even if curvature
# never agrees. The blend is still FIGHT_BLEND_RATE_PER_S.
FIGHT_MAX_HOLD_S = 6.0
# Resting torsion (this EPAS reads about +0.25 Nm hands-off). Learned
# only while clearly hands-off and quiet, then clamped.
REST_BIAS_TAU_S = 3.0
REST_BIAS_CLAMP_NM = 0.5
REST_BIAS_MAX_RATE_DEG_S = 5.0

PREAP_FINGERPRINT = "TESLA_MODEL_S_PREAP"
# Settings → NAP. Default On. Turn Off if gravel / wind still false-yields.
PARAM_DRIVER_LAT_HANDOFF = "NAPDriverLatHandoff"


def handoff_enabled(*, fingerprint: str, param_on: bool) -> bool:
  """Pre-AP and Settings toggle (param defaults On)."""
  return bool(param_on) and fingerprint == PREAP_FINGERPRINT


def lat_reenable_inhibited(*, blinker_paused: bool, v_ego: float) -> bool:
  """True when soft-lat must not take the wheel back.

  Driver-turn blinker latch (any speed) OR v_ego strictly below 10 mph.
  Keep control if we still have it; already yielded / blending stay
  yielded. Falling edge (blinker clear or speed crosses 10 mph) lands
  in yield so the normal 0.15 s confirm + 1 s blend owns resume.
  """
  return bool(blinker_paused) or float(v_ego) < LAT_REENABLE_MIN_V_EGO


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


def limit_authority_step(prev: float, target: float, dt: float, rate_per_s: float) -> float:
  """Rise toward ``target`` no faster than ``rate_per_s``. Falls immediately.

  The normal 1 s blend uses BLEND_AUTHORITY_RATE_PER_S, which matches
  the smoothstep peak, so a 100 Hz smoothstep is unchanged. The fight
  re-take passes FIGHT_BLEND_RATE_PER_S.
  """
  prev_a = float(np.clip(prev, 0.0, 1.0))
  target_a = float(np.clip(target, 0.0, 1.0))
  if target_a <= prev_a:
    return target_a
  return float(min(target_a, prev_a + max(0.0, float(rate_per_s)) * max(0.0, float(dt))))


def lat_active_after_handoff(lat_would_be_active: bool, yielded: bool) -> bool:
  """EPS request bit after blinker / standstill / soft-yield.

  INFERENCE CONTRACT: this bit becomes DAS_steeringControlType (1 = OP is
  steering) and is the ONLY thing panda / the card use to infer "lateral
  yielded". Any new reason for latActive to go false or come back true,
  or any change to handoff / blend timing, must update opendbc
  preap/lat_yield.py + tesla_preap_latyield.h (see the module docstring
  and test_lat_yield_inference_guard.py).

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
    # Survives _reset so a disengage does not forget the sensor offset.
    self._rest_bias = 0.0
    self._reset()

  @property
  def rest_bias(self) -> float:
    return float(self._rest_bias)

  def _reset(self):
    self.authority = 1.0
    self.ui_paused = False
    self._yielded = False
    self._blending = False
    self._quiet_s = 0.0
    self._blend_s = 0.0
    self._press_cnt = 0
    self._release_cnt = 0
    self._early_cnt = 0
    self._early_sign = 0
    self._pressed = False
    self._blinker_was_paused = False
    self._hands_off_s = 0.0
    self._decel_cnt = 0
    self._yield_age_s: float | None = None
    self._peak_angle_deg = 0.0
    self._gate_hold_s = 0.0
    self._straight_s = 0.0
    self._clock_s = 0.0
    self._yield_log: list[tuple[float, int]] = []
    self._fight = False
    self._fight_sign = 0
    self._fight_release = False
    self._torque_hold_s = 0.0
    self._torque_hold_sign = 0
    self._fight_quiet_s = 0.0
    self._fight_agree_s = 0.0
    # Eligible immediately. Only a blinker / low-speed inhibit zeros this.
    self._post_inhibit_s = FIGHT_POST_INHIBIT_S
    self._last_driver_hold = False
    self._last_inhibited = False

  def reset(self):
    self._reset()

  def driver_resume_request(self) -> None:
    """In-session stalk pull: drop the tug-of-war latch.

    If the wheel is already free (hands off, under the yield trigger, not
    in a turn or under 10 mph), start the normal 1 s blend. A hand still
    on the rim stays yielded; the latch is gone, so the usual 0.15 s
    confirm owns the take-back once they let go. No chime lives here.
    """
    self._fight = False
    self._fight_release = False
    self._fight_sign = 0
    self._fight_quiet_s = 0.0
    self._fight_agree_s = 0.0
    self._torque_hold_s = 0.0
    self._torque_hold_sign = 0
    self._yield_log.clear()
    if not self.enabled or not self._yielded:
      return
    if self._last_inhibited or self._last_driver_hold:
      return
    self._start_blend()

  def _compensated_torque(self, steering_torque: float) -> float:
    try:
      tq = float(steering_torque)
    except (TypeError, ValueError):
      return 0.0
    if not np.isfinite(tq):
      return 0.0
    return tq - self._rest_bias

  def _update_rest_bias(self, torque, rate_deg, hands_on_level, steering_pressed,
                        _lat_would_be_active, steering_angle_deg, dt: float) -> None:
    """Slow EMA of hands-off, nearly-straight torsion. Ignores a real push."""
    if int(hands_on_level or 0) != 0 or bool(steering_pressed):
      return
    try:
      rate = abs(float(rate_deg))
      tq = float(torque)
    except (TypeError, ValueError):
      return
    if not np.isfinite(rate) or not np.isfinite(tq):
      return
    if rate >= REST_BIAS_MAX_RATE_DEG_S or abs(tq) > REST_BIAS_CLAMP_NM:
      return
    # Anything at or above the fight floor is the driver's push, not this
    # EPAS's resting reading. A steady +0.25 Nm still learns; 0.40 Nm does not.
    if abs(tq) >= FIGHT_TORQUE_NM:
      return
    # Only a straight wheel. A yield with no angle (older callers, unit
    # tests) is a maneuver, not a resting sample. A known-straight wheel
    # may learn while yielded: that is the hands-off torsion.
    if steering_angle_deg is None:
      if self._yielded or self._blending:
        return
    else:
      try:
        if abs(float(steering_angle_deg)) > WHEEL_RESUME_STRAIGHT_DEG:
          return
      except (TypeError, ValueError):
        return
    alpha = 1.0 - float(np.exp(-max(float(dt), 0.0) / REST_BIAS_TAU_S))
    self._rest_bias += alpha * (tq - self._rest_bias)
    self._rest_bias = float(min(REST_BIAS_CLAMP_NM, max(-REST_BIAS_CLAMP_NM, self._rest_bias)))

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

  def _update_early(self, torque_nm: float, steering_pressed: bool) -> bool:
    """Same-sign firm torsion while steeringPressed, no hands gate."""
    mag = abs(float(torque_nm))
    sign = 1 if float(torque_nm) > 0.0 else -1
    if steering_pressed and mag >= EARLY_YIELD_NM and (
        self._early_cnt == 0 or sign == self._early_sign):
      self._early_sign = sign
      self._early_cnt = min(self._early_cnt + 1, EARLY_YIELD_FRAMES + 1)
    else:
      self._early_cnt = 0
      self._early_sign = 0
    return self._early_cnt >= EARLY_YIELD_FRAMES

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

  def _note_angle(self, steering_angle_deg: float | None):
    """Track peak |wheel| while a yield / blend / lat-down is in play."""
    if steering_angle_deg is None:
      return
    a = abs(float(steering_angle_deg))
    if np.isfinite(a) and a > self._peak_angle_deg:
      self._peak_angle_deg = a

  def _wheel_gate_holds(self, steering_angle_deg: float | None, dt: float) -> bool:
    """True: do not start the take-back blend yet (wheel still uncoiling)."""
    if steering_angle_deg is None:
      return False
    a = abs(float(steering_angle_deg))
    if not np.isfinite(a):
      return False
    if self._peak_angle_deg < WHEEL_GATE_ARM_DEG:
      return False
    self._gate_hold_s += dt
    if self._gate_hold_s >= WHEEL_GATE_MAX_HOLD_S:
      return False
    if a <= WHEEL_RESUME_STRAIGHT_DEG:
      self._straight_s += dt
      return self._straight_s + 1e-12 < WHEEL_STRAIGHT_DWELL_S
    self._straight_s = 0.0
    return True

  def _enter_yield(self):
    self._yielded = True
    self._blending = False
    self._fight_release = False
    self._quiet_s = 0.0
    self._blend_s = 0.0
    self._hands_off_s = 0.0
    self._gate_hold_s = 0.0
    self._straight_s = 0.0
    self.authority = 0.0
    self.ui_paused = True
    if self._yield_age_s is None:
      self._yield_age_s = 0.0

  def _start_blend(self):
    # A fight re-take is rate-limited. Leave the latch up so a curvature
    # jump during the blend re-yields instead of snatching the wheel.
    # A normal dodge (_fight False) keeps the 1 s smoothstep.
    self._fight_release = bool(self._fight)
    self._yielded = False
    self._blending = True
    self._blend_s = 0.0
    self._quiet_s = 0.0
    self._hands_off_s = 0.0
    self._gate_hold_s = 0.0
    self._straight_s = 0.0
    self.authority = 0.0
    self.ui_paused = True

  def _note_yield(self, torque_nm: float):
    sign = 1 if float(torque_nm) > 0.05 else -1 if float(torque_nm) < -0.05 else 0
    if sign == 0:
      sign = self._fight_sign or self._torque_hold_sign
    now = self._clock_s
    self._yield_log.append((now, sign))
    self._yield_log = [(t, s) for t, s in self._yield_log if now - t <= FIGHT_WINDOW_S]
    if sign == 0:
      return
    same = sum(1 for _t, s in self._yield_log if s == sign)
    if same >= FIGHT_YIELD_COUNT:
      self._fight = True
      self._fight_sign = sign

  def _update_sustained_fight(self, torque_nm: float, dt: float):
    """Arm stay-yielded on a long one-sided hold after a recent yield."""
    mag = abs(float(torque_nm))
    sign = 1 if float(torque_nm) > 0.0 else -1 if float(torque_nm) < 0.0 else 0
    # A hold already in progress stays "recent" after a 1 s blend finishes.
    # Otherwise a 0.3–0.5 Nm push (under the 0.55 yield trigger) would
    # take lateral back and then forget the push that was still on the wheel.
    recent = (
      self._yielded or self._blending or self._torque_hold_s > 0.0
      or any(self._clock_s - t <= FIGHT_WINDOW_S for t, _s in self._yield_log)
    )
    same_sign = self._torque_hold_sign in (0, sign)
    if recent and mag >= FIGHT_TORQUE_NM and sign != 0 and same_sign:
      if self._torque_hold_sign == 0:
        self._torque_hold_sign = sign
      self._torque_hold_s += dt
      if self._torque_hold_s >= FIGHT_TORQUE_HOLD_S:
        self._fight = True
        self._fight_sign = sign
    elif mag < FIGHT_QUIET_NM or (sign != 0 and self._torque_hold_sign not in (0, sign)):
      self._torque_hold_s = 0.0
      self._torque_hold_sign = 0

  def _fight_resume_allowed(self, torque_nm: float, curv_err: float, dt: float, *,
                            hands_on_level: int = 0, steering_pressed: bool = False) -> bool:
    """True when a fight hold may start the normal hands-off confirm.

    ``torque_nm`` is already relative to the resting offset. Quiet also
    requires the car's hands-off level and steeringPressed, because this
    EPAS never sits under FIGHT_QUIET_NM in raw torsion.
    """
    if not self._fight:
      return True
    quiet = (
      int(hands_on_level or 0) == 0
      and not bool(steering_pressed)
      and abs(float(torque_nm)) < FIGHT_QUIET_NM
    )
    if quiet:
      self._fight_quiet_s += dt
    else:
      self._fight_quiet_s = 0.0
    # Wider bar once the re-take has started so one noisy frame
    # does not cancel a blend that already agreed.
    err_limit = FIGHT_CURVATURE_ERR * (2.0 if self._fight_release else 1.0)
    if float(curv_err) < err_limit:
      self._fight_agree_s += dt
    else:
      self._fight_agree_s = 0.0
    agreed = (
      self._fight_quiet_s + 1e-12 >= FIGHT_QUIET_S
      and self._fight_agree_s + 1e-12 >= FIGHT_CURVATURE_HOLD_S
    )
    # Ceiling: genuinely hands-quiet this long starts the same slow
    # re-take even when curvature never comes home.
    capped = self._fight_quiet_s + 1e-12 >= FIGHT_MAX_HOLD_S
    return agreed or capped

  def _identity(self, *, emergency_cancel: bool = False) -> HandoffOutput:
    return HandoffOutput(1.0, False, False, False, emergency_cancel)

  def update(self, *, engaged: bool, lat_would_be_active: bool,
             steering_torque: float, steering_rate_deg: float,
             alc_active: bool = False, blinker_paused: bool = False,
             hands_on_level: int = 0, tracking_error: float = 0.0,
             brake_applied: bool = False, a_ego: float = 0.0,
             v_ego: float = 15.0, dt: float | None = None,
             emergency_yank: bool = False,
             lane_change_confirm: bool = False,
             steering_pressed: bool = False,
             steering_angle_deg: float | None = None,
             model_curvature: float | None = None,
             measured_curvature: float | None = None) -> HandoffOutput:
    if dt is None:
      dt = DT_CTRL

    if not self.enabled:
      self._reset()
      return self._identity()

    self._update_rest_bias(
      steering_torque, steering_rate_deg, hands_on_level, bool(steering_pressed),
      bool(lat_would_be_active), steering_angle_deg, dt)
    tq = self._compensated_torque(steering_torque)

    # lat_would_be_active is the *pre-yield* request (standstill / faults;
    # and lamp-latch pause only when soft-lat is Off). Caller drops
    # CC.latActive after this update when yielded so the EPS is free —
    # do not feed that dropped bit back in or we reset to identity and
    # forget we yielded.
    # A same-direction confirm during tipped ALC is not a soft-yield.
    # An emergency yank still frees the EPS immediately, same as a yield
    # that today's takeover follows with a full disengage.
    tipped = bool(alc_active or lane_change_confirm)
    if tipped and tipped_lane_change_driver_release(
        same_direction=bool(lane_change_confirm), emergency=bool(emergency_yank)):
      self._enter_yield()
      self._set_ui_paused()
      return HandoffOutput(self.authority, self.ui_paused, True, False, False)

    # ALC wheel-nudge uses steeringPressed at 1 Nm and must not be softened.
    # Full disengage (cancel / door / hands-on >= 2) clears engaged.
    # blinker_paused is a latched *driver-turn* (not ALC tip/keep-alive).
    # Soft-lat On: do not strip lat on lamp latch. Inhibit re-enable
    # while the turn blinker is on *or* v_ego is below 10 mph; after
    # that clears, enter yield so the normal 0.15 s confirm + 1 s blend
    # owns resume (no dedicated blinker rising-edge blend). Soft-lat
    # Off never reaches here (enabled=False → identity);
    # BlinkerLateralHold still frees lat on lamp latch only.
    if not engaged or alc_active or lane_change_confirm:
      self._reset()
      return self._identity()
    inhibited = lat_reenable_inhibited(blinker_paused=blinker_paused, v_ego=v_ego)
    if not lat_would_be_active:
      remember = self._blinker_was_paused or inhibited
      peak = self._peak_angle_deg if remember else 0.0
      self._reset()
      self._blinker_was_paused = remember
      self._peak_angle_deg = peak
      return self._identity()

    self._clock_s += dt
    # A turn (blinker or under 10 mph) and a short grace after it are
    # not a tug-of-war. An already-armed fight still uses the quiet test.
    if inhibited:
      self._post_inhibit_s = 0.0
    else:
      self._post_inhibit_s += dt
    fight_evidence = (
      not inhibited and self._post_inhibit_s + 1e-12 >= FIGHT_POST_INHIBIT_S)
    fight_on = model_curvature is not None and measured_curvature is not None
    if fight_on:
      try:
        curv_err = abs(float(model_curvature) - float(measured_curvature))
      except (TypeError, ValueError):
        curv_err = 0.0
        fight_on = False
      if not np.isfinite(curv_err):
        curv_err = 0.0
        fight_on = False
    else:
      curv_err = 0.0
    if fight_on and fight_evidence:
      self._update_sustained_fight(tq, dt)
    elif fight_on:
      self._torque_hold_s = 0.0
      self._torque_hold_sign = 0
    if fight_on:
      resume_allowed = self._fight_resume_allowed(
        tq, curv_err, dt, hands_on_level=hands_on_level,
        steering_pressed=bool(steering_pressed))
    else:
      resume_allowed = True

    mag = abs(float(tq))
    pressed = self._update_intent(
      tq, steering_rate_deg,
      hands_still_on(hands_on_level), tracking_error)
    if self._update_early(tq, bool(steering_pressed)):
      pressed = self._pressed = True
    hands_on = hands_still_on(hands_on_level)
    firm_push = mag >= SOFT_YIELD_TRIGGER_NM
    if (self._yielded or self._blending or self._blinker_was_paused
        or pressed):
      self._note_angle(steering_angle_deg)

    driver_hold = hands_on or firm_push
    # Fight hold blocks the take-back only. A normal single dodge
    # (resume_allowed True) is the same state machine as before.
    if inhibited:
      # Keep control if we still have it. A driver push may still yield.
      # Already yielded / blending / coming back from lat-down: stay
      # yielded. Do not finish a take-back blend while the turn lamp
      # is latched or speed is below 10 mph.
      if self._yielded or self._blending or self._blinker_was_paused:
        self._enter_yield()
        self._blinker_was_paused = True
      elif pressed:
        self._enter_yield()
        self._blinker_was_paused = True
    elif self._blinker_was_paused:
      # Inhibit just cleared (blinker off and/or speed crossed 10 mph,
      # or lat came back after a blinker / low-speed standstill/fault).
      # No dedicated 1 s blend shortcut. Land in yield; normal
      # hands-off confirm owns the resume.
      self._blinker_was_paused = False
      self._enter_yield()
      if not driver_hold and resume_allowed:
        self._hands_off_s += dt
        if (self._hands_off_s + 1e-12 >= HANDS_OFF_CONFIRM_S
            and not self._wheel_gate_holds(steering_angle_deg, dt)):
          self._start_blend()
    elif not self._yielded and not self._blending:
      if pressed or (fight_on and self._fight and not resume_allowed):
        rising = not self._yielded
        self._enter_yield()
        if fight_on and fight_evidence and rising and pressed:
          self._note_yield(tq)
    elif self._yielded:
      # Stay yielded while still maneuvering: hands on the rim OR a
      # renewed firm push. Mid-dodge torsion dips (below release) must
      # not start the blend. Hands 0 for ~0.15 s → 1 s smoothstep.
      # A fight hold also stays yielded until torque is quiet and the
      # model path is close to the wheel.
      if driver_hold or not resume_allowed:
        self._enter_yield()
      else:
        self._hands_off_s += dt
        if (self._hands_off_s + 1e-12 >= HANDS_OFF_CONFIRM_S
            and not self._wheel_gate_holds(steering_angle_deg, dt)):
          self._start_blend()
    elif self._blending:
      # Hands back on or a firm push cancels the return. Mid-band
      # torque during the blend is OP/caster, not a new push. Re-yield
      # does not re-require rate agreement (already in the maneuver).
      # Fight hold cancels a re-take that the model is still fighting.
      if driver_hold or not resume_allowed:
        self._enter_yield()
        if fight_on and fight_evidence and driver_hold:
          self._note_yield(tq)
      else:
        self._blend_s += dt
        t_norm = self._blend_s / BLEND_TIME_S
        target = 1.0 if t_norm >= 1.0 else smoothstep(t_norm)
        rate = FIGHT_BLEND_RATE_PER_S if self._fight_release else BLEND_AUTHORITY_RATE_PER_S
        self.authority = limit_authority_step(self.authority, target, dt, rate)
        if self._blend_s + 1e-12 >= BLEND_TIME_S and self.authority + 1e-9 >= 1.0:
          self.authority = 1.0
          self._blending = False
          self._fight = False
          self._fight_release = False
          self._fight_sign = 0
          self._fight_quiet_s = 0.0
          self._fight_agree_s = 0.0
          # Leave the sustained-torque timer alone. A clean re-take is
          # quiet, so the timer is already 0. A push still under the
          # yield trigger must keep counting toward the 5 s stay-yielded.
          self._yield_log.clear()
          self._blend_s = 0.0
          self._quiet_s = 0.0
          self._peak_angle_deg = 0.0

    self._last_driver_hold = bool(hands_on or firm_push)
    self._last_inhibited = bool(inhibited)
    emergency = self._update_emergency(
      brake_applied=brake_applied, a_ego=a_ego, v_ego=v_ego, dt=dt)
    self._set_ui_paused()
    return HandoffOutput(
      self.authority, self.ui_paused, self._yielded, self._blending,
      emergency)
