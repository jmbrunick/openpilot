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
wheel while lat is down, then a fast taper back onto the model.
Hands resting on the rim do not keep lateral yielded.

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
  compensated torque tau = steeringTorque − learned offset
      same sign, |tau| >= 0.9 Nm for 150 ms, or >= 1.4 Nm for 80 ms
  A sign flip restarts the count. EPAS_handsOnLevel is not required
      (it lags the torsion bar).
  The 2.0 Nm / 80 ms steeringPressed early-yield is the same fast
      counter, so a firm grab lands on yielded lateral.
  Anything lighter or shorter: OP keeps steering. Resting hands
      (about 0.15–0.35 Nm on this car) do not yield.

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

Push to take, release to give back. Entry is the compensated torque
above (0.9 Nm / 150 ms or 1.4 Nm / 80 ms). Give-back ignores hands-on
level. Any one of these starts the taper, unless a blinker or speed
below 10 mph is inhibiting, or the post-turn wheel gate is holding:
  R1  |tau| < 0.40 Nm for 0.30 s (hands may stay on the rim)
  R2  |model curvature − actual| < 0.0008 1/m for 0.5 s, with
      |tau| < 1.2 Nm and not rising
  R3  |tau| < 0.8 Nm for 2.5 s
The taper ramps authority 0 → 1 at
clamp(0.002 / |model k − actual k|, 1.0, 2.0) per second, starting
from the wheel. That finishes inside BLEND_TIME_S (1.0 s), which is
what GRACE_S / the panda 1.2 s window already cover. The shortest
automatic yield is RELEASE_HOLD_S (0.30 s), above GAP. A stalk pull
forces the taper (no chime) once that minimum has elapsed.
Re-yield during the taper only on a deliberate push: |tau| rises
0.6 Nm above its down-tracking taper-start baseline for 150 ms, or
tau opposes OP's wheel motion at >= 0.9 Nm for 150 ms. Torque in
OP's direction never cancels the taper.
There is no tug-of-war latch. Two or more re-yields inside 15 s make
the next taper use the 1.0/s floor. That softener clears after 10 s
of full lateral and never blocks a return.
tau subtracts one of two resting offsets (OP steering vs wheel free),
each an EMA with a 3 s time constant, learned at hands level 0,
|angle| < 12 deg, |raw| < 0.35 Nm, and wheel-still from
steeringAngleDeg (steeringRateDeg is SNA on this car). Clamped ±0.5 Nm.

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
    * a yield lasts at least HANDS_OFF_CONFIRM_S before the blend starts
      (the automatic release is RELEASE_HOLD_S, which is longer);
    * the blend lasts at most BLEND_TIME_S and the re-arm grace covers it.
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
A light hand on the rim does not reset the gate. Torque back above the
release band does (the driver is still in the maneuver). Safety valve:
after WHEEL_GATE_MAX_HOLD_S (3 s) of ready-but-not-straight hold the gate
lets go so a wheel that never centres cannot leave lateral off forever.
steering_angle_deg=None (not wired / old callers) disables the gate.
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
from opendbc.car.tesla.preap.lat_yield import GRACE_DELAY_S
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
# Ready-to-return time the gate may hold the resume before it lets go.
WHEEL_GATE_MAX_HOLD_S = 3.0

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

# Push to take. Same-sign compensated torque. Hands level is not required.
YIELD_SOFT_NM = 0.9
YIELD_SOFT_S = 0.15
YIELD_FIRM_NM = 1.4
# 80 ms, same counter as EARLY_YIELD_FRAMES.
YIELD_FIRM_FRAMES = EARLY_YIELD_FRAMES

# Release to give back. Hands-on level is ignored.
RELEASE_TAU_NM = 0.40
RELEASE_HOLD_S = 0.30  # R1. Also the shortest yield the inference allows.
AGREE_CURVATURE_ERR = 0.0008  # 1/m, R2
AGREE_HOLD_S = 0.50
AGREE_TAU_NM = 1.2
AGREE_RISE_EPS_NM = 0.02
LIGHT_HOLD_TAU_NM = 0.8  # R3
LIGHT_HOLD_S = 2.5

# Fast taper. At BLEND_TIME_S == 1 these are 1.0 and 2.0 per second, so the
# ramp finishes inside the grace the panda / card already assume.
TAPER_RATE_MIN_PER_S = 1.0
TAPER_RATE_MAX_PER_S = 2.0
TAPER_CURVATURE_SCALE = 0.002
TAPER_RISE_NM = 0.6
TAPER_RISE_S = 0.15
TAPER_OPPOSE_NM = 0.9
TAPER_OPPOSE_S = 0.15

# Softener: never blocks a return. Two re-yields inside the window make the
# next taper use the slow floor. Clears after this long at full lateral.
REYIELD_COUNT = 2
REYIELD_WINDOW_S = 15.0
FULL_LATERAL_RESET_S = 10.0

# Two resting offsets (OP steering vs wheel free). steeringRateDeg is SNA
# on this car, so wheel-still is ΔsteeringAngleDeg/Δt.
REST_BIAS_TAU_S = 3.0
REST_BIAS_CLAMP_NM = 0.5
REST_BIAS_MAX_RATE_DEG_S = 5.0
REST_BIAS_MAX_ANGLE_DEG = WHEEL_RESUME_STRAIGHT_DEG  # |angle| < 12 deg
REST_BIAS_MAX_RAW_NM = 0.35

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
  """Consecutive 100 Hz frames of this |tau| needed to enter yield.

  150 ms at 0.9 Nm, 80 ms at 1.4 Nm and above. Below 0.9 Nm this returns
  the old debounce length so a caller can spin that many frames and still
  not yield (resting hands, road noise).
  """
  mag = abs(float(torque_nm))
  if mag + 1e-12 >= YIELD_FIRM_NM:
    return YIELD_FIRM_FRAMES
  if mag + 1e-12 >= YIELD_SOFT_NM:
    return int(round(YIELD_SOFT_S / DT_CTRL))
  return SOFT_YIELD_DEBOUNCE_FRAMES


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


def taper_rate_per_s(curv_err: float | None, *, soften: bool = False) -> float:
  """Authority rise per second for the take-back taper.

  clamp(0.002 / |model k − actual k|, 1, 2) when BLEND_TIME_S is 1.
  Both ends scale with 1/BLEND_TIME_S so a longer blend still outlasts
  the inference grace (the guard's tripwire). Unknown curvature, or the
  re-yield softener, uses the slow floor. The ramp still finishes in at
  most BLEND_TIME_S.
  """
  floor = TAPER_RATE_MIN_PER_S / BLEND_TIME_S
  ceiling = TAPER_RATE_MAX_PER_S / BLEND_TIME_S
  if soften or curv_err is None:
    return float(floor)
  try:
    err = abs(float(curv_err))
  except (TypeError, ValueError):
    return float(floor)
  if not np.isfinite(err):
    return float(floor)
  raw = ceiling if err < 1e-9 else TAPER_CURVATURE_SCALE / err
  return float(min(ceiling, max(floor, raw)))


def limit_authority_step(prev: float, target: float, dt: float, rate_per_s: float) -> float:
  """Rise toward ``target`` no faster than ``rate_per_s``. Falls immediately.

  The take-back taper passes taper_rate_per_s. A rate of 1/BLEND_TIME_S
  reaches 1 inside the grace window; agreement is allowed to be faster.
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
  """Process-local latch: push to take the wheel, release to give it back."""

  def __init__(self, enabled: bool = True):
    # Product default On. controlsd still requires Pre-AP + param
    # (NAPDriverLatHandoff defaults true). Toggle Off to disable.
    self.enabled = bool(enabled)
    # Survive _reset so a disengage does not forget the sensor offsets.
    self._rest_op = 0.0
    self._rest_free = 0.0
    self._applied_bias = 0.0
    self._prev_angle: float | None = None
    self._reset()

  @property
  def rest_bias(self) -> float:
    """Offset currently subtracted from steeringTorque (the live tau bias)."""
    return float(self._applied_bias)

  @property
  def rest_bias_op(self) -> float:
    return float(self._rest_op)

  @property
  def rest_bias_free(self) -> float:
    return float(self._rest_free)

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
    self._soft_s = 0.0
    self._soft_sign = 0
    self._pressed = False
    self._blinker_was_paused = False
    self._hands_off_s = 0.0
    self._decel_cnt = 0
    self._yield_age_s: float | None = None
    self._peak_angle_deg = 0.0
    self._gate_hold_s = 0.0
    self._straight_s = 0.0
    self._clock_s = 0.0
    self._r1_s = 0.0
    self._r2_s = 0.0
    self._r3_s = 0.0
    self._r2_base = 0.0
    self._taper_base = 0.0
    self._taper_soften = False
    self._rise_s = 0.0
    self._oppose_s = 0.0
    self._soften = False
    self._reyields: list[float] = []
    self._full_s = 0.0
    self._pull_pending = False
    self._last_driver_hold = False
    self._last_inhibited = False

  def reset(self):
    self._reset()

  def driver_resume_request(self) -> None:
    """In-session stalk pull: force the take-back taper. No chime.

    Clears the re-yield softener. A blinker or sub-10 mph inhibit still
    holds the yield; the pull is remembered and applied when that clears,
    once the yield has lasted RELEASE_HOLD_S. Hands on the rim do not block it.
    """
    self._soften = False
    self._reyields.clear()
    self._taper_soften = False
    if not self.enabled or not (self._yielded or self._blending):
      self._pull_pending = False
      return
    self._pull_pending = True

  def _wheel_rate_deg_s(self, steering_angle_deg, dt: float) -> float | None:
    """|Δangle/Δt|. steeringRateDeg is SNA on this car and is not used."""
    if steering_angle_deg is None:
      return None
    try:
      angle = float(steering_angle_deg)
    except (TypeError, ValueError):
      return None
    if not np.isfinite(angle):
      self._prev_angle = None
      return None
    prev = self._prev_angle
    self._prev_angle = angle
    if prev is None or float(dt) <= 1e-6:
      return None
    return abs(angle - prev) / float(dt)

  def _learn_rest(self, torque, hands_on_level, steering_angle_deg, dt: float, *,
                  engaged: bool, lat_would_be_active: bool) -> None:
    """EMA of hands-off, straight, wheel-still torsion. Two offsets."""
    if int(hands_on_level or 0) != 0:
      return
    try:
      tq = float(torque)
    except (TypeError, ValueError):
      return
    if not np.isfinite(tq) or abs(tq) >= REST_BIAS_MAX_RAW_NM:
      return
    rate = self._wheel_rate_deg_s(steering_angle_deg, dt)
    if rate is None or rate >= REST_BIAS_MAX_RATE_DEG_S:
      return
    try:
      if abs(float(steering_angle_deg)) >= REST_BIAS_MAX_ANGLE_DEG:
        return
    except (TypeError, ValueError):
      return
    if self._blending:
      return
    wheel_free = bool(self._yielded) or (not engaged) or (not lat_would_be_active)
    op_steering = bool(engaged and lat_would_be_active and not self._yielded)
    if wheel_free:
      slot = "_rest_free"
    elif op_steering:
      slot = "_rest_op"
    else:
      return
    alpha = 1.0 - float(np.exp(-max(float(dt), 0.0) / REST_BIAS_TAU_S))
    cur = float(getattr(self, slot))
    cur += alpha * (tq - cur)
    cur = float(min(REST_BIAS_CLAMP_NM, max(-REST_BIAS_CLAMP_NM, cur)))
    setattr(self, slot, cur)

  def _compensated_torque(self, steering_torque: float, *, use_free: bool) -> float:
    try:
      tq = float(steering_torque)
    except (TypeError, ValueError):
      tq = 0.0
    if not np.isfinite(tq):
      tq = 0.0
    # Free-wheel offset while the EPS is released, OP is disengaged, or
    # the take-back taper is in progress, so tau does not jump at the start.
    if use_free or self._yielded or self._blending:
      bias = self._rest_free
    else:
      bias = self._rest_op
    self._applied_bias = float(bias)
    return tq - bias

  def _update_soft(self, torque_nm: float, dt: float) -> bool:
    """Same-sign |tau| >= 0.9 Nm for 150 ms. A sign flip or a gap resets."""
    mag = abs(float(torque_nm))
    sign = 1 if float(torque_nm) > 0.0 else -1 if float(torque_nm) < 0.0 else 0
    if mag >= YIELD_SOFT_NM and sign != 0 and self._soft_sign in (0, sign):
      self._soft_sign = sign
      self._soft_s += max(float(dt), 0.0)
    else:
      self._soft_s = 0.0
      self._soft_sign = 0
    return self._soft_s + 1e-12 >= YIELD_SOFT_S

  def _update_early(self, torque_nm: float, steering_pressed: bool) -> bool:
    """Same-sign fast torsion. 1.4 Nm / 80 ms, or 2.0 Nm with steeringPressed."""
    mag = abs(float(torque_nm))
    sign = 1 if float(torque_nm) > 0.0 else -1
    fast = mag >= YIELD_FIRM_NM or (bool(steering_pressed) and mag >= EARLY_YIELD_NM)
    if fast and (
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
    self._quiet_s = 0.0
    self._blend_s = 0.0
    self._hands_off_s = 0.0
    self._r1_s = 0.0
    self._r2_s = 0.0
    self._r3_s = 0.0
    self._rise_s = 0.0
    self._oppose_s = 0.0
    self._gate_hold_s = 0.0
    self._straight_s = 0.0
    self.authority = 0.0
    self.ui_paused = True
    if self._yield_age_s is None:
      self._yield_age_s = 0.0

  def _rehold_yield(self):
    """Stay yielded and reset the wheel-straight dwell, keeping give-back timers."""
    r1, r2, r3, base = self._r1_s, self._r2_s, self._r3_s, self._r2_base
    self._enter_yield()
    self._r1_s, self._r2_s, self._r3_s, self._r2_base = r1, r2, r3, base

  def _start_blend(self, tau: float):
    self._taper_soften = bool(self._soften)
    self._taper_base = abs(float(tau))
    self._rise_s = 0.0
    self._oppose_s = 0.0
    self._yielded = False
    self._blending = True
    self._blend_s = 0.0
    self._quiet_s = 0.0
    self._hands_off_s = 0.0
    self._r1_s = 0.0
    self._r2_s = 0.0
    self._r3_s = 0.0
    self._gate_hold_s = 0.0
    self._straight_s = 0.0
    self.authority = 0.0
    self.ui_paused = True

  def _min_yield_met(self, dt: float) -> bool:
    age = 0.0 if self._yield_age_s is None else float(self._yield_age_s)
    return age + max(float(dt), 0.0) + 1e-12 >= RELEASE_HOLD_S

  def _integrate_giveback(self, tau: float, curv_err: float, dt: float,
                          have_curv: bool) -> bool:
    """Advance R1/R2/R3. True when any one has held long enough."""
    mag = abs(float(tau))
    step = max(float(dt), 0.0)
    if mag < RELEASE_TAU_NM:
      self._r1_s += step
    else:
      self._r1_s = 0.0
    if mag < LIGHT_HOLD_TAU_NM:
      self._r3_s += step
    else:
      self._r3_s = 0.0
    if have_curv and float(curv_err) < AGREE_CURVATURE_ERR and mag < AGREE_TAU_NM:
      if self._r2_s <= 0.0:
        self._r2_base = mag
      if mag <= self._r2_base + AGREE_RISE_EPS_NM:
        self._r2_s += step
        if mag < self._r2_base:
          self._r2_base = mag
      else:
        self._r2_s = 0.0
        self._r2_base = mag
    else:
      self._r2_s = 0.0
      self._r2_base = mag
    return (
      self._r1_s + 1e-12 >= RELEASE_HOLD_S
      or self._r2_s + 1e-12 >= AGREE_HOLD_S
      or self._r3_s + 1e-12 >= LIGHT_HOLD_S
    )

  def _note_reyield(self):
    now = self._clock_s
    self._reyields.append(now)
    self._reyields = [t for t in self._reyields if now - t <= REYIELD_WINDOW_S]
    if len(self._reyields) >= REYIELD_COUNT:
      self._soften = True

  def _note_full(self, dt: float):
    if (not self._yielded and not self._blending
        and self.authority + 1e-9 >= 1.0):
      self._full_s += max(float(dt), 0.0)
      if self._full_s + 1e-12 >= FULL_LATERAL_RESET_S:
        self._soften = False
        self._reyields.clear()
    else:
      self._full_s = 0.0

  def _taper_reyield(self, tau: float, model_k, measured_k, dt: float) -> bool:
    """Deliberate push during the taper. Helping torque never cancels."""
    mag = abs(float(tau))
    if mag < self._taper_base:
      self._taper_base = mag
    helping = False
    opposing = False
    dk = None
    if model_k is not None and measured_k is not None:
      try:
        dk = float(model_k) - float(measured_k)
      except (TypeError, ValueError):
        dk = None
      if dk is None or not np.isfinite(dk):
        dk = None
    # Positive curvature and positive steeringTorque are both left.
    if dk is not None and abs(dk) > 1e-6:
      if float(tau) * dk > 0.0:
        helping = True
      elif float(tau) * dk < 0.0 and mag >= TAPER_OPPOSE_NM:
        opposing = True
    if helping:
      self._rise_s = 0.0
      self._oppose_s = 0.0
      return False
    if mag >= self._taper_base + TAPER_RISE_NM:
      self._rise_s += max(float(dt), 0.0)
    else:
      self._rise_s = 0.0
    if opposing:
      self._oppose_s += max(float(dt), 0.0)
    else:
      self._oppose_s = 0.0
    return (
      self._rise_s + 1e-12 >= TAPER_RISE_S
      or self._oppose_s + 1e-12 >= TAPER_OPPOSE_S
    )

  def _curv_pair(self, model_curvature, measured_curvature):
    if model_curvature is None or measured_curvature is None:
      return False, 0.0
    try:
      err = abs(float(model_curvature) - float(measured_curvature))
    except (TypeError, ValueError):
      return False, 0.0
    if not np.isfinite(err):
      return False, 0.0
    return True, err

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
    # steering_rate_deg is SNA (-4095.5) on this Pre-AP car. Wheel-still
    # for the resting offset uses steeringAngleDeg. The argument stays so
    # callers do not change.
    _ = steering_rate_deg

    if not self.enabled:
      self._reset()
      return self._identity()

    self._learn_rest(
      steering_torque, hands_on_level, steering_angle_deg, dt,
      engaged=bool(engaged), lat_would_be_active=bool(lat_would_be_active))
    use_free = (
      bool(self._yielded or self._blending)
      or (not engaged)
      or (not lat_would_be_active))
    tq = self._compensated_torque(steering_torque, use_free=use_free)

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
    # that clears, enter yield so the release rules own resume.
    # Soft-lat Off never reaches here (enabled=False → identity);
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
    have_curv, curv_err = self._curv_pair(model_curvature, measured_curvature)
    # Push-to-take counters run only while OP has the wheel. A helping
    # torque during the taper must not be pre-armed as the next yield.
    if self._yielded or self._blending:
      self._soft_s = 0.0
      self._soft_sign = 0
      self._early_cnt = 0
      self._early_sign = 0
      wants = False
    else:
      wants = self._update_soft(tq, dt) or self._update_early(tq, bool(steering_pressed))
    if (self._yielded or self._blending or self._blinker_was_paused or wants):
      self._note_angle(steering_angle_deg)

    if inhibited:
      # Keep control if we still have it. A driver push may still yield.
      # Already yielded / blending / coming back from lat-down: stay
      # yielded. Do not finish a take-back blend while the turn lamp
      # is latched or speed is below 10 mph.
      if self._yielded or self._blending or self._blinker_was_paused:
        self._enter_yield()
        self._blinker_was_paused = True
      elif wants:
        self._enter_yield()
        self._blinker_was_paused = True
    elif self._blinker_was_paused:
      # Inhibit just cleared (blinker off and/or speed crossed 10 mph).
      # Land in yield; R1/R2/R3 own the resume. A pending stalk pull
      # skips the wheel gate once the minimum yield has elapsed.
      self._blinker_was_paused = False
      self._enter_yield()
      pull_now = self._pull_pending and self._min_yield_met(dt)
      ready = self._integrate_giveback(tq, curv_err, dt, have_curv)
      if pull_now or (ready and not self._wheel_gate_holds(steering_angle_deg, dt)):
        self._pull_pending = False
        self._start_blend(tq)
      elif not ready:
        self._rehold_yield()
    elif not self._yielded and not self._blending:
      if wants:
        self._enter_yield()
    elif self._yielded:
      pull_now = self._pull_pending and self._min_yield_met(dt)
      ready = self._integrate_giveback(tq, curv_err, dt, have_curv)
      if pull_now or (ready and not self._wheel_gate_holds(steering_angle_deg, dt)):
        self._pull_pending = False
        self._start_blend(tq)
      elif not ready:
        self._rehold_yield()
    elif self._blending:
      # Helping torque (OP's direction) never cancels. A deliberate push does.
      if self._pull_pending:
        self._pull_pending = False
      if self._taper_reyield(tq, model_curvature, measured_curvature, dt):
        self._note_reyield()
        self._enter_yield()
      else:
        rate = taper_rate_per_s(
          curv_err if have_curv else None, soften=self._taper_soften)
        self._blend_s += dt
        # full_control stays true for GRACE_DELAY_S after type-1 resumes.
        # Hold authority at 0 through that hole, and compress the spec
        # rate into the rest of BLEND_TIME_S so the ramp still ends
        # inside the grace (no safety-window change).
        # 50 Hz stamps can land up to one frame after the blend starts,
        # so the grace-delay hole runs a little past GRACE_DELAY_S.
        if self._blend_s + 1e-12 < GRACE_DELAY_S + 0.03:
          self.authority = 0.0
        else:
          usable = max(BLEND_TIME_S - (GRACE_DELAY_S + 0.03), DT_CTRL)
          self.authority = limit_authority_step(
            self.authority, 1.0, dt, rate * (BLEND_TIME_S / usable))
        if self.authority + 1e-9 >= 1.0:
          self.authority = 1.0
          self._blending = False
          self._quiet_s = 0.0
          self._peak_angle_deg = 0.0
          self._blend_s = 0.0

    self._note_full(dt)
    self._last_driver_hold = False
    self._last_inhibited = bool(inhibited)
    emergency = self._update_emergency(
      brake_applied=brake_applied, a_ego=a_ego, v_ego=v_ego, dt=dt)
    self._set_ui_paused()
    return HandoffOutput(
      self.authority, self.ui_paused, self._yielded, self._blending,
      emergency)
