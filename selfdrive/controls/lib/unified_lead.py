"""Continuous lead-follow controller.

One equation replaces the stacked follow overrides when NAPLongUnified is on.
The planner still runs MPC. This module is a few arithmetic ops per frame.

  a_cmd = k_g·e + k_v(v_lead - v_ego) + k_a·a_lead,   bounded by a_req

e = gap - gap_set is one signed gap error. gap_set is the selected follow
gap (t_follow·v_lead + standstill). There is no separate inside-gap or
outside-gap formula: every term is a smooth function of e, so nothing steps
at the follow distance.

The kinematic required decel

  a_req = -v_close² / (2·D(e))

is applied while closing (Sep 25 10:07:59 / 10:10:13). D(e) is the room to
match speed: outside the gap the gap error shortened with closing speed,
plus the part of the follow gap we may eat into (it fades in over the last
few seconds of approach and shrinks as e goes negative), minus a short
reaction distance. Slow closes give small values; a fast close from 60 m is
firm straight away; a small miss at the gap edge is not a -3.5 spike
(Sep 25 09:21:44). There is no far soft cap and no far comfort cap.

A pulling-away lead near the gap gets a trickle, not a catch-up lunge
(Sep 25 09:47 / 10:54). Far from the gap, with speed already matched, the
proportional gap term would still demand a highway pull; that term fades
out between 15 and 50 m of slack and a soft desired closing speed
(about 0.10–0.95 m/s, matched command still under 0.15 m/s²) takes its
place. Braking, the near gap, and the follow distance are unchanged. Speed ceilings (MAX, curve) are a
speed-error term: never accelerate above the cap, and above it start
near the EV's mild -0.22 m/s² settle, firmer only when well over
(Sep 25 10:43:56).

A lead is weighted by a continuous confidence: consistent, on-path, radar
or confident-vision readings ramp to full weight; a fast close ramps within
a fraction of a second; a one-frame blip barely moves the command. A new
lead (cut-in) ramps in the same way from the command already in force.

A jerk limit slews the result. Build and release rates grow with how far
the command is from the target: fast when far too firm, gentle when close.

Driver-likeness (Justin's fingerprint, Sep 20–27 qlogs):
- A lead that starts braking is anticipated: its filtered decel onset rate
  extends a_lead ahead by up to 0.8 m/s², weighted continuously by how hard
  it already brakes, how short the headway is, and how fast the decel builds.
- Mild lead slowing is mirrored under 1:1 (0.75×) so the ease stays gradual;
  real braking is over-matched (1.10×). One smooth blend, no threshold.
- Inside the gap the glide keeps a small ease until the gap recovers.
- A far, slow close coasts: the closing gain fades with time-to-gap 12→30 s.
- A far, nearly matched gap eases in instead of k_g·slack. The desired
  close is fast enough to finish a 100 m settle inside 240 s, and the
  matched command stays under 0.15 m/s² so it does not pulse.

How hard a lead may brake us depends on how much we trust it (Sep 28
11:07:22, a half-lane-over lead 80–120 m ahead drove −3.5 for ~1 s). The
physical need a_need is the decel that stops the gap closing inside a
keep distance, lead braking-to-stop included. Lead-driven decel past
min(a_need·margin, −0.22) is scaled by a continuous trust: 1 for a close
or short-time-to-collision lead (every #222 / cut-in / rapid-close case),
falling to a fraction for a far, long-TTC lead, lower still when it sits
off our path or is a weak vision read. The same bound applies to the
command a new lead blends from, so a firmer inherited command is not
carried into an ambiguous far lead either. A far lead whose kinematics
really need full regen (stopped car at 100 m and 30 m/s) still gets it.
"""
from __future__ import annotations

import math

from openpilot.selfdrive.controls.lib.lead_leaving import CLEAR_MARGIN_M, EGO_HALF_WIDTH_M, LEAD_HALF_WIDTH_M

# Same standstill gap long_mpc uses for the follow obstacle.
STOP_DISTANCE_M = 6.0

# Hard plant bounds. Full regen is ACCEL_MIN.
A_MIN_MS2 = -3.5
A_MAX_MS2 = 2.0

# Gap-error gain. Severity in [0, 1] interpolates low → high.
K_G_LO = 0.012  # 1/s²
K_G_HI = 0.030
# Relative-speed gain while closing. Small closes land near mild regen.
K_V_LO = 0.16   # 1/s
K_V_HI = 0.10   # Large closes let a_lead and a_req own the depth.
# a_lead weight: 1 next to the gap, easing off when the lead is still far.
K_A_NEAR = 1.10  # slightly over 1: a closing ego must shed more than the lead brakes
K_A_FAR = 0.45
K_A_NEAR_M = 18.0
K_A_FAR_M = 70.0
# Braking a_lead fades out when the lead is still far in both meters and
# time (15:56: ~83 m of slack, ~38 s to the gap, command was k_a·aLead).
# Full weight near the gap, on a short time-to-gap, or on a hard lead decel.
ALEAD_FADE_SLACK_LO_M = 20.0
ALEAD_FADE_SLACK_HI_M = 55.0
ALEAD_FADE_TTG_LO_S = 12.0
ALEAD_FADE_TTG_HI_S = 30.0
ALEAD_FADE_EMERG_LO_MS2 = 2.6
ALEAD_FADE_EMERG_HI_MS2 = 3.4
# The a_lead weight also grows with how hard the lead brakes: mild slowing is
# under-matched (a gradual, matched ease), real braking over-matched.
K_A_MILD = 0.75
K_A_FIRM = 1.10
K_A_FIRM_A0_MS2 = 0.30
K_A_FIRM_BAND_MS2 = 0.50
# Far, slow closes coast: the closing gain fades with time-to-gap.
K_V_FAR_TTG_LO_S = 12.0
K_V_FAR_TTG_HI_S = 30.0
K_V_FAR_MIN = 0.40

GAP_SEV_M = 16.0
CLOSE_SEV_MS = 3.2

# Glide: nearly matched at the setpoint contributes nothing you can feel.
GLIDE_GAP_M = 3.5
GLIDE_V_MS = 0.40
GLIDE_KEEP = 0.90  # fraction of the gap/speed term removed at perfect glide
# Inside the gap the glide removes less, so a small ease persists until the
# gap recovers instead of holding speed a few meters too close.
GLIDE_KEEP_INSIDE = 0.60

# Kinematic room D(e): outside the gap the gap error shortened with closing
# speed (e / (1 + v/V0)); at and inside the setpoint a share of the follow
# gap is intrusion room that fades in over the last seconds of approach and
# shrinks one-for-one as e goes negative, down to a floor. Minus reaction.
ARRIVE_V0_MS = 8.0  # outside room = e / (1 + v_close / V0)
ARRIVE_SOFT_M = 2.0
INSIDE_ALLOW_FRAC = 0.65  # of min(t_follow, cap)·v_lead
INSIDE_ALLOW_TF_MAX_S = 1.0  # follow time beyond ~1 s is reserve, not room
ALLOW_FADE_TTG_S = 4.0  # intrusion room fades in over the last seconds to the setpoint
ALLOW_FADE_V_MIN_MS = 0.3
INSIDE_ALLOW_MIN_M = 3.0
REACTION_S = 0.3
KIN_ROOM_MIN_M = 1.5
# Closing speed below this contributes no kinematic bound (fades in).
KIN_GATE_MS = 0.40
# Far-gap widening of that gate (F3). A slow close at a far gap with the PD
# term wanting to accelerate made the gate flip the command between the PD
# and a near-zero kinematic bound (accCmd pulses at 15a 11:27, 159 08:03).
# While slack is past WIDEN_SLACK_LO_M (full weight at +WIDEN_SLACK_RAMP_M)
# and the bound is not a real brake (weight 1 at >= 0, 0 at <= WIDEN_A_BOUND_LO),
# the gate width grows with the PD accel, so the bound fades in over a wider
# closing speed. Never firmer than before; unchanged for a real bound
# (<= -0.10), a non-accelerating PD, or slack under 15 m.
KIN_GATE_WIDEN_SLACK_LO_M = 15.0
KIN_GATE_WIDEN_SLACK_RAMP_M = 5.0
KIN_GATE_WIDEN_A_BOUND_LO = -0.10
KIN_GATE_WIDEN_A_BOUND_RAMP = 0.10
KIN_GATE_WIDEN_A_CAP = 0.50
KIN_GATE_WIDEN_K = 1.0  # 1/s: gate width = a_pd / K, capped at A_CAP / K

# Far-gap soft closing speed. k_g·slack grows without bound, so a nearly
# matched lead 40–60 m out asks for +1.2…+1.8 m/s² and the kinematic gate
# bangs that into ~3 s catch-up pulses (15a 11:27). Fade the classic gap
# attraction from 15 → 50 m and replace the faded portion with a desired
# closing speed that saturates near 0.95 m/s. The fade finishes at 50 m,
# not 40: a Follow-7 recovery still has ~30 m of slack when it needs to
# come back, and ending the fade at 40 m left that catch-up too weak to
# finish inside 83.5 m. Past 50 m the highway matched case is fully soft.
# The blend is on only where the classic PD term would accelerate and the
# kinematic bound is not a real brake, so near-gap follow, #222, and
# braking stay on today's path.
GAP_FADE_LO_M = 15.0
GAP_FADE_HI_M = 50.0
V_DES_CLOSE_LO = 0.10           # m/s at the start of the fade
# The ramp still finishes 30 m above the fade floor, but the ceiling is
# 0.95 m/s rather than 0.35. A 0.35 m/s ease from a 100 m start is still
# inside the set gap at 240 s (Follow 2 landed 1.6 m short). The higher
# desired close gets every Follow setting through that settle in time.
# K_CATCH is lower so the matched command stays about +0.13, under the
# 0.15 soft cap: no highway pull, no gate pulses.
V_DES_CLOSE_SPAN = 0.85         # → 0.95 m/s once the ramp is done
V_DES_CLOSE_RAMP_M = 30.0
K_CATCH = 0.14                  # 1/s
GAP_CATCH_A_ON = 0.15           # classic a_pd at which the rewrite is full
GAP_CATCH_A_BOUND_LO = -0.10    # at or below this the bound is a real brake
GAP_CATCH_A_BOUND_RAMP = 0.10

# Short headway: pull firmer below ~0.6 s even at small closing.
SHORT_HW_S = 0.60
SHORT_HW_BAND_S = 0.30
SHORT_HW_A = 0.60

# Pulling away near the gap: positive command compresses to a trickle.
# The compression fades out once the slack is large (real catch-up).
TRICKLE_A = 0.12
TRICKLE_SCALE = 0.15
TRICKLE_FAR_LO_M = 25.0
TRICKLE_FAR_HI_M = 60.0
TRICKLE_IN_M = 6.0
# Opening lead: the relative-speed gain is small near the gap and grows
# with slack, so a lead pulling away is not chased.
K_V_OPEN_NEAR = 0.07
K_V_OPEN_FAR = 0.16
OPEN_INSIDE_M = 6.0  # inside by this much: opening uses the closing gain

# Speed ceiling (MAX, curve). Below the cap: a <= (v_cap - v)/tau (0 at cap).
# Above it: start at the EV mild settle, firmer only when well over.
SPEED_CEILING_TAU_S = 4.0
OVERSPEED_SETTLE_MS2 = 0.22
OVERSPEED_RAMP_MS = 0.45
OVERSPEED_K = 0.12       # m/s² per m/s beyond the ramp
OVERSPEED_MAX_MS2 = 1.20  # soft saturation

# Filters. One 50 ms blip moves a_lead by ~1/4 of the spike.
TAU_V_S = 0.20
TAU_A_S = 0.15

# Slow-close comfort lives on the follow inputs, not a post-planner clamp.
# Closing speed and gap are low-passed. The old 6 m/s "rapid" line is a
# regime with enter/exit hysteresis and a dwell, so a noisy sample cannot
# flip the input source. The published command is never softer than the
# unfiltered law when physics still needs more than the EV mild settle.
APPROACH_TAU_CLOSE_S = 0.25
APPROACH_TAU_GAP_S = 0.20
APPROACH_RAPID_ENTER_MS = 6.0
APPROACH_RAPID_EXIT_MS = 5.4
APPROACH_RAPID_DWELL_S = 0.30
APPROACH_COMFORT_CLOSE_MS = 3.0
APPROACH_NEED_MARGIN_M = 2.0
APPROACH_NEED_ROOM_MIN_M = 3.0
APPROACH_MILD_MS2 = 0.22
# A lead already braking this hard is not the slow-close comfort path
# (#222 stays on the unfiltered law).
APPROACH_BRAKE_A_MS2 = -0.35

# Faster cut-in that is still pulling away. Enter on a new lead id only.
# While latched the lead cannot cause decel (hold / coast; the existing
# trickle still caps a chase inside the follow window). MAX, curve, and
# map limits still apply. Handover is one-way.
CUTIN_PULL_ENTER_MS = 2.0          # v_lead - v_ego at acquire
CUTIN_CLOSE_MS = 0.3               # confirmed closing (ego faster than lead)
CUTIN_CLOSE_HOLD_S = 0.50
CUTIN_PRED_T_S = 1.5               # lead will be slower than ego within this
CUTIN_PRED_HOLD_S = 0.30
CUTIN_VREL_TAU_S = 0.30
CUTIN_ALEAD_IGNORE_S = 0.75
CUTIN_HARD_A_MS2 = -1.5
CUTIN_HARD_FRAMES = 2
CUTIN_PROJ_T_S = 3.0
CUTIN_KEEP_M = 6.0
CUTIN_KEEP_HW_S = 0.3              # keep = 6 m + 0.3 s * v_ego
CUTIN_HEADWAY_S = 0.4

# Lead-decel anticipation: a_eff = a_lead - min(MAX, T·max(0, -da_lead/dt)·w).
# w grows continuously with the lead's decel (A0/BAND), short headway
# (HW/HW_BAND) and the decel onset rate (J0/JBAND). A steady lead adds nothing.
LEAD_ANTICIPATE_T_S = 2.2
LEAD_ANTICIPATE_MAX_MS2 = 0.80
LEAD_JERK_TAU_S = 0.25
LEAD_ANTICIPATE_A0_MS2 = 0.10
LEAD_ANTICIPATE_ABAND_MS2 = 0.35
LEAD_ANTICIPATE_HW_S = 2.5
LEAD_ANTICIPATE_HW_BAND_S = 1.0
LEAD_ANTICIPATE_J0_MS3 = 0.20
LEAD_ANTICIPATE_JBAND_MS3 = 0.40

# Jerk. Rates grow with the distance between command and target.
JERK_RELEASE_MS3 = 0.45
JERK_RELEASE_OPEN_MS3 = 1.35
JERK_BUILD_MS3 = 1.15
JERK_PHYS_MS3 = 2.50
JERK_PROP_1_S = 2.0  # m/s³ extra per m/s² of error
UNIFIED_JERK_LIMIT_MS3 = JERK_PHYS_MS3

# Lead confidence. dw/dt = (R0 + R1·urgency·w)·(target - w) while rising.
CONF_RATE_BASE_1_S = 2.0
CONF_RATE_URGENT_1_S = 14.0
CONF_RATE_FALL_1_S = 4.0
CONF_URGENT_LO_MS2 = 0.5
CONF_URGENT_BAND_MS2 = 1.5
CONF_VISION_MIN = 0.35
# Kept for callers that referenced the old fixed cut-in blend.
CUTIN_S = 0.45
# Toggle engage/disengage blend lives in the planner (~1 s).
MODE_BLEND_S = 1.0

# Static lateral fade from the predicted travel path. It starts only where
# a lead that is not moving sideways already clears our body: our half-width
# + its half-width + margin (lead_leaving). A stalled half-out lead keeps
# full weight. A lead that is *moving* out is released earlier, and only
# when its projection clears us at our arrival, by the planner's leaving
# weight (`leave_w`), combined with this fade below.
DEPART_Y0_M = EGO_HALF_WIDTH_M + LEAD_HALF_WIDTH_M + CLEAR_MARGIN_M
DEPART_Y1_M = DEPART_Y0_M + 0.7

# Achievable hard decel vs lead trust (Sep 28 11:07:22). A lead is remote
# when it is both far and a long time-to-collision away; a remote lead's
# trust tops out at TRUST_REMOTE and shrinks with lateral offset and a weak
# read. Decel past the physical need (with margin) is scaled by trust.
# The path-collapse gate does not cover this. It only blocks a *new*
# association while lane lines are down and the near path is absurd, and
# it leaves an already-followed lead alone. A far lead's rapid decel is
# a different path: the emergency a_lead restore below. That restore, and
# the lead-brake share of the kinematic floor, require the lead to sit on
# the predicted path. Close and short-TTC leads (#222) do not.
TRUST_RANGE_LO_M = 60.0
TRUST_RANGE_HI_M = 110.0
TRUST_TTC_LO_S = 4.0
TRUST_TTC_HI_S = 8.0
TRUST_REMOTE = 0.35
TRUST_LAT_Y0_M = 0.9   # within our own half-width: fully on path
TRUST_LAT_MIN = 0.35   # at the static depart line
NEED_KEEP_TF_FRAC = 0.5  # keep distance: standstill + half the follow time
NEED_MARGIN = 1.25
NEED_MILD_MS2 = 0.22  # an untrusted lead may always ease at the EV's mild settle

# Far rapid-brake path. Full credit inside our own half-width. None once
# the lead is past a lane line (drifting out, adjacent lane, or a lateral
# ghost). Smooth between, so a noisy yRel does not step the command.
# The controller filters this offset; one frame cannot flip it.
RAPID_PATH_Y_IN_M = 0.9
RAPID_PATH_Y_OUT_M = 1.8
TAU_RAPID_PATH_S = 0.25


def _smooth01(x: float) -> float:
  """Smoothstep on [0, 1]. Constant outside. Value and slope match at the ends."""
  if x <= 0.0:
    return 0.0
  if x >= 1.0:
    return 1.0
  return x * x * (3.0 - 2.0 * x)


def _softplus(x: float, scale: float) -> float:
  """Smooth max(x, 0). Equals x well above 0, 0 well below."""
  s = max(1e-6, float(scale))
  z = float(x) / s
  if z > 30.0:
    return float(x)
  return s * math.log1p(math.exp(z))


def _filt(prev: float | None, sample: float, dt: float, tau: float) -> float:
  if prev is None or tau <= 1e-4:
    return float(sample)
  a = float(dt) / (float(tau) + float(dt))
  return float(prev) + a * (float(sample) - float(prev))


def gap_set_m(v_lead: float, t_follow: float) -> float:
  """Selected follow gap, including standstill distance."""
  return float(t_follow) * max(0.0, float(v_lead)) + STOP_DISTANCE_M


def _severity(slack: float, v_err: float) -> float:
  g = float(slack) / GAP_SEV_M
  v = float(v_err) / CLOSE_SEV_MS
  return 1.0 - math.exp(-(g * g) - (v * v))


def _k_a(slack: float) -> float:
  t = _smooth01((max(float(slack), 0.0) - K_A_NEAR_M) / max(1e-3, K_A_FAR_M - K_A_NEAR_M))
  return K_A_NEAR + (K_A_FAR - K_A_NEAR) * t


def _k_a_brake_scale(a_lead: float) -> float:
  """0.75× for mild lead slowing, 1.10× for real braking, smooth in between."""
  t = _smooth01((-float(a_lead) - K_A_FIRM_A0_MS2) / K_A_FIRM_BAND_MS2)
  return K_A_MILD + (K_A_FIRM - K_A_MILD) * t


def rapid_path_offset_m(y_rel: float, curvature: float, gap: float,
                        path_lat: float | None = None) -> float:
  """Lateral distance from the predicted path, in meters.

  `path_lat` wins: a lead in a bend stays on-path even when raw yRel is
  large, and an adjacent-lane car at yRel ≈ 0 does not. Without a path,
  curve geometry is taken back out of yRel. A missing reading is 0, which
  is on-path — do not invent an off-path block.
  """
  if path_lat is not None and math.isfinite(float(path_lat)):
    return abs(float(path_lat))
  try:
    y = abs(float(y_rel))
  except (TypeError, ValueError):
    return 0.0
  if not math.isfinite(y):
    return 0.0
  lane = abs(0.5 * float(curvature) * float(gap) * float(gap))
  return max(0.0, y - lane)


def rapid_path_credit(y_abs: float) -> float:
  """1 on the predicted path, 0 once the lead has left it."""
  if not math.isfinite(float(y_abs)):
    return 1.0
  span = max(1e-3, RAPID_PATH_Y_OUT_M - RAPID_PATH_Y_IN_M)
  return 1.0 - _smooth01((float(y_abs) - RAPID_PATH_Y_IN_M) / span)


def _braking_alead_weight(slack: float, v_close: float, a_lead: float,
                          path_credit: float = 1.0) -> float:
  """How much of a braking lead to copy, in [0, 1].

  0 when there is a lot of slack and a long time to the gap, and the lead
  is not braking hard. 1 near the gap, on a short time-to-gap, or when the
  lead decel is emergency-sized. The emergency restore is the far
  rapid-brake path: it is scaled by path credit, so a far lead that is
  drifting out, off the predicted path, or a lateral ghost does not firm
  ego braking. A close or short-TTC lead ignores that credit. Positive
  a_lead is not scaled here.
  """
  if float(a_lead) >= 0.0:
    return 1.0
  slack_f = float(slack)
  near = 1.0 - _smooth01(
    (slack_f - ALEAD_FADE_SLACK_LO_M) / (ALEAD_FADE_SLACK_HI_M - ALEAD_FADE_SLACK_LO_M))
  if slack_f <= 0.0:
    short = 1.0
  else:
    ttg = slack_f / max(float(v_close), 0.3)
    short = 1.0 - _smooth01(
      (ttg - ALEAD_FADE_TTG_LO_S) / (ALEAD_FADE_TTG_HI_S - ALEAD_FADE_TTG_LO_S))
  credit = 1.0 if path_credit is None or not math.isfinite(float(path_credit)) else min(
    1.0, max(0.0, float(path_credit)))
  emerg = _smooth01(
    (-float(a_lead) - ALEAD_FADE_EMERG_LO_MS2) / (ALEAD_FADE_EMERG_HI_MS2 - ALEAD_FADE_EMERG_LO_MS2))
  emerg *= credit
  return 1.0 - (1.0 - near) * (1.0 - short) * (1.0 - emerg)


def credited_rapid_lead(a_lead: float, slack: float, v_close: float,
                        path_credit: float) -> float:
  """Lead accel the kinematic floor may treat as a real in-path stop.

  Near the gap or on a short time-to-gap the whole decel counts (#222).
  A far rapid decel counts only in proportion to path credit. Off-path,
  that decel does not deepen the floor, so an inherited firm command is
  not justified by a lead that is leaving or a radar ghost.
  """
  a = float(a_lead)
  if a >= 0.0:
    return a
  base = _braking_alead_weight(slack, v_close, a, path_credit=0.0)
  credit = 0.0 if path_credit is None or not math.isfinite(float(path_credit)) else min(
    1.0, max(0.0, float(path_credit)))
  keep = base + (1.0 - base) * credit
  return a * keep


def lead_decel_anticipation(a_lead: float, lead_jerk: float, gap: float, v_ego: float) -> float:
  """Extra decel (>= 0) ahead of a lead whose braking is still building.

  Continuous in every input: 0 for a steady or recovering lead, growing with
  how hard the lead already brakes, how short the headway is, and how fast
  its decel builds. Capped so a gradual slowdown cannot become a wall.
  """
  building = max(0.0, -float(lead_jerk))
  if building <= 0.0:
    return 0.0
  hw = float(gap) / max(float(v_ego), 1.0)
  w = _smooth01((-float(a_lead) - LEAD_ANTICIPATE_A0_MS2) / LEAD_ANTICIPATE_ABAND_MS2)
  w *= _smooth01((LEAD_ANTICIPATE_HW_S - hw) / LEAD_ANTICIPATE_HW_BAND_S)
  w *= _smooth01((building - LEAD_ANTICIPATE_J0_MS3) / LEAD_ANTICIPATE_JBAND_MS3)
  return min(LEAD_ANTICIPATE_MAX_MS2, LEAD_ANTICIPATE_T_S * building * w)


def _glide(slack: float, v_err: float, a_lead: float) -> float:
  """1 when matched and the lead is not braking. A real lead brake kills it."""
  g = math.exp(-((float(slack) / GLIDE_GAP_M) ** 2) - ((float(v_err) / GLIDE_V_MS) ** 2))
  brake = min(float(a_lead), 0.0)
  g *= math.exp(-((brake / 0.30) ** 2))
  return g


def kinematic_room_m(slack: float, v_close: float, t_follow: float, v_lead: float) -> float:
  """Room to match speed. One smooth function of the signed gap error.

  Outside the gap the room is the gap error itself, shortened as the
  closing speed grows (plan to be matched earlier on a fast close). At and
  inside the setpoint part of the follow gap is usable intrusion room; it
  fades in across the last few meters of approach and shrinks one-for-one
  as the gap error goes negative, down to a floor.
  """
  e = float(slack)
  v = max(0.0, float(v_close))
  outside = _softplus(e, ARRIVE_SOFT_M) / (1.0 + v / ARRIVE_V0_MS)
  tf_room = min(float(t_follow), INSIDE_ALLOW_TF_MAX_S)
  allow = max(INSIDE_ALLOW_MIN_M, INSIDE_ALLOW_FRAC * tf_room * max(0.0, float(v_lead)))
  # min(e, 0) has a kink at 0 but its value is continuous.
  inside = _softplus(allow + min(e, 0.0) - INSIDE_ALLOW_MIN_M, 1.0) + INSIDE_ALLOW_MIN_M
  # The intrusion room fades in over the last ALLOW_FADE_TTG_S of approach
  # (time to reach the setpoint at the current closing speed).
  ttg = e / max(v, ALLOW_FADE_V_MIN_MS)
  inside *= 1.0 - _smooth01(ttg / ALLOW_FADE_TTG_S)
  room = outside + inside - REACTION_S * v
  return max(KIN_ROOM_MIN_M, room)


def kinematic_required_accel(slack: float, v_close: float, t_follow: float, v_lead: float) -> float:
  """a_req = -v_close² / (2·D). 0 when not closing."""
  v = max(0.0, float(v_close))
  if v <= 0.0:
    return 0.0
  room = kinematic_room_m(slack, v, t_follow, v_lead)
  return max(A_MIN_MS2, -(v * v) / (2.0 * room))


def _a_kin(slack: float, v_close: float, gap: float, v_ego: float,
           t_follow: float, v_lead: float) -> float:
  """Kinematic bound used by the build-rate choice. Same term as the command."""
  _ = gap, v_ego
  return kinematic_required_accel(slack, v_close, t_follow, v_lead)


def _short_headway(gap: float, v_ego: float, v_close: float) -> float:
  """Firmer pull below ~0.6 s headway unless the lead is pulling away."""
  hw = float(gap) / max(float(v_ego), 1.0)
  w = _smooth01((SHORT_HW_S - hw) / SHORT_HW_BAND_S)
  if w <= 0.0:
    return 0.0
  closing_w = _smooth01((float(v_close) + 0.5) / 1.0)
  return -SHORT_HW_A * w * closing_w


def _depart_fade(y_rel: float, curvature: float, gap: float,
                 path_lat: float | None = None) -> float:
  """0 on the travel path, 1 when the lead has left it.

  `path_lat` is the lateral offset from the model's predicted path at the
  lead's distance, so a lead in a bend stays on-path and an adjacent-lane
  car is off-path. Without a path, the curve geometry is removed from yRel.
  """
  if path_lat is not None and math.isfinite(float(path_lat)):
    y = abs(float(path_lat))
  else:
    lane = abs(0.5 * float(curvature) * float(gap) * float(gap))
    y = max(0.0, abs(float(y_rel)) - lane)
  return _smooth01((y - DEPART_Y0_M) / max(1e-3, DEPART_Y1_M - DEPART_Y0_M))


def _leave_fade(y_rel: float, curvature: float, gap: float,
                path_lat: float | None = None, leave_w: float = 0.0) -> float:
  """Static off-path fade combined with the planner's leaving weight."""
  static = _depart_fade(y_rel, curvature, gap, path_lat)
  w = 0.0 if leave_w is None or not math.isfinite(float(leave_w)) else min(1.0, max(0.0, float(leave_w)))
  return 1.0 - (1.0 - static) * (1.0 - w)


def kinematic_need_accel(gap: float, v_ego: float, v_lead: float, a_lead: float,
                         t_follow: float) -> float:
  """Physical decel (<= 0) that keeps the gap above a keep distance.

  Constant-accel lead, braking to a stop if a_lead < 0. If our speed can
  match the lead's before it stops, a = a_lead - v_close²/(2·room);
  otherwise we stop behind where it stops. The two agree at the boundary.
  """
  g = max(0.0, float(gap))
  v_e = max(0.0, float(v_ego))
  v_l = max(0.0, float(v_lead))
  a_l = min(0.0, float(a_lead))
  keep = STOP_DISTANCE_M + NEED_KEEP_TF_FRAC * max(0.0, float(t_follow)) * v_l
  room = g - keep
  if room <= KIN_ROOM_MIN_M:
    return A_MIN_MS2 if v_e > v_l or a_l < 0.0 else min(0.0, a_l)
  v_close = max(0.0, v_e - v_l)
  a_match = a_l - (v_close * v_close) / (2.0 * room)
  if a_l < 0.0:
    t_match = 2.0 * room / max(v_close, 1e-3)
    t_stop = v_l / -a_l
    if t_match > t_stop:
      a_match = -(v_e * v_e) / (2.0 * (room + (v_l * v_l) / (2.0 * -a_l)))
  return max(A_MIN_MS2, min(0.0, a_match))


def lead_trust(gap: float, v_ego: float, v_lead: float, y_rel: float = 0.0,
               curvature: float = 0.0, path_lat: float | None = None,
               model_prob: float | None = None, radar: bool | None = None) -> float:
  """How much hard braking this lead may command, in [0, 1].

  1 for a close or short-TTC lead. A remote lead (far AND long TTC) keeps
  TRUST_REMOTE, scaled by how on-path and how good the reading is.
  """
  g = max(0.0, float(gap))
  v_close = max(0.0, float(v_ego) - max(0.0, float(v_lead)))
  ttc = g / max(v_close, 0.1)
  far = _smooth01((g - TRUST_RANGE_LO_M) / (TRUST_RANGE_HI_M - TRUST_RANGE_LO_M))
  slow = _smooth01((ttc - TRUST_TTC_LO_S) / (TRUST_TTC_HI_S - TRUST_TTC_LO_S))
  remote = far * slow
  if remote <= 0.0:
    return 1.0
  if path_lat is not None and math.isfinite(float(path_lat)):
    y = abs(float(path_lat))
  else:
    lane = abs(0.5 * float(curvature) * g * g)
    y = max(0.0, abs(float(y_rel)) - lane)
  lat_w = 1.0 - (1.0 - TRUST_LAT_MIN) * _smooth01((y - TRUST_LAT_Y0_M) / (DEPART_Y0_M - TRUST_LAT_Y0_M))
  q = lead_quality(model_prob, radar)
  return 1.0 - remote * (1.0 - TRUST_REMOTE * lat_w * q)


def hard_decel_floor(gap: float, v_ego: float, v_lead: float, a_lead: float,
                     t_follow: float) -> float:
  """Firmest decel an untrusted lead may command: the need with margin, or mild."""
  need = kinematic_need_accel(gap, v_ego, v_lead, a_lead, t_follow)
  return max(A_MIN_MS2, min(-NEED_MILD_MS2, NEED_MARGIN * need))


def trust_bound(a: float, floor: float, trust: float) -> float:
  """Decel past `floor` scaled by trust. Continuous; identity at trust 1."""
  a = float(a)
  if a >= floor:
    return a
  return float(floor) + min(1.0, max(0.0, float(trust))) * (a - float(floor))


def speed_ceiling_accel(v_ego: float, v_cap: float) -> float:
  """Speed-error term for MAX / curve caps.

  Below the cap it only limits acceleration (0 at the cap). Above it the
  decel starts at the mild settle and grows with overspeed, saturating
  softly well short of full regen.
  """
  over = float(v_ego) - float(v_cap)
  if over <= 0.0:
    return -over / SPEED_CEILING_TAU_S
  mild = OVERSPEED_SETTLE_MS2 * _smooth01(over / OVERSPEED_RAMP_MS)
  extra = OVERSPEED_K * max(0.0, over - OVERSPEED_RAMP_MS)
  raw = mild + extra
  return -OVERSPEED_MAX_MS2 * math.tanh(raw / OVERSPEED_MAX_MS2)


def _kin_gate_widen(slack: float, a_pd: float, a_bound: float) -> float:
  """Extra kinematic-gate width (m/s) at a far gap while the PD term accelerates."""
  # a_pd <= 0 (not accelerating) never clears the base width: wide < 0.
  wide = min(a_pd, KIN_GATE_WIDEN_A_CAP) / KIN_GATE_WIDEN_K - KIN_GATE_MS
  if wide <= 0.0:
    return 0.0
  w_slack = _smooth01((slack - KIN_GATE_WIDEN_SLACK_LO_M) / KIN_GATE_WIDEN_SLACK_RAMP_M)
  w_bound = _smooth01((a_bound - KIN_GATE_WIDEN_A_BOUND_LO) / KIN_GATE_WIDEN_A_BOUND_RAMP)
  return wide * w_slack * w_bound


def _far_gap_catchup_gap(a_gap_classic: float, slack: float, v_err: float, keep: float,
                         a_pd_classic: float, a_bound: float) -> float:
  """Gap term with the far positive catch-up replaced by a soft closing speed.

  Classic k_g·slack below GAP_FADE_LO_M. Above that, and only while the
  classic PD term is a catch-up and the kinematic bound is not a real
  brake, blend toward K_CATCH·(v_des + v_err). v_des rises from 0.10 to
  0.95 m/s. A negative soft term is clipped, so closing faster than v_des
  does not add gap braking. The blend never exceeds the classic gap term.
  """
  w_slack = _smooth01((slack - GAP_FADE_LO_M) / (GAP_FADE_HI_M - GAP_FADE_LO_M))
  w_accel = _smooth01(a_pd_classic / GAP_CATCH_A_ON)
  w_bound = _smooth01((a_bound - GAP_CATCH_A_BOUND_LO) / GAP_CATCH_A_BOUND_RAMP)
  w = w_slack * w_accel * w_bound
  v_des_close = V_DES_CLOSE_LO + V_DES_CLOSE_SPAN * _smooth01(
    (slack - GAP_FADE_LO_M) / V_DES_CLOSE_RAMP_M)
  a_soft = K_CATCH * (v_des_close + v_err) * keep
  # min keeps a lead that is already faster from getting a harder gap pull
  # than classic: a_soft includes v_err, which can exceed k_g·slack.
  a_gap = (1.0 - w) * a_gap_classic + w * max(0.0, a_soft)
  return min(a_gap, a_gap_classic)


def unified_follow_desired(gap: float, v_ego: float, v_lead: float, a_lead: float,
                           t_follow: float, *, v_ceiling: float | None = None,
                           a_map: float | None = None, y_rel: float = 0.0,
                           curvature: float = 0.0,
                           path_lat: float | None = None,
                           leave_w: float = 0.0,
                           model_prob: float | None = None,
                           radar: bool | None = None,
                           gap_set_override_m: float | None = None,
                           v_curve_cap: float | None = None,
                           rapid_path_w: float | None = None) -> float:
  """Unslewed follow accel from filtered lead signals. Continuous in its inputs.

  `gap_set_override_m` replaces only the meter setpoint used for slack
  (gap lock). `t_follow` still sizes intrusion room, trust, and FCW.
  A lock is still this law: the near-gap "do not chase" opening gain,
  the positive-accel trickle, and the far soft-close are skipped so a
  speed deficit left by the pre-lock ease tracks the locked meters
  instead of opening the gap and creeping back. Cruise MAX is not applied
  while the lock is the setpoint, so the car may go above MAX to hold
  those meters. `v_curve_cap` still slows the car for a bend.
  """
  gap_f = max(0.0, float(gap))
  v_l = max(0.0, float(v_lead))
  v_e = max(0.0, float(v_ego))
  a_l = float(a_lead)
  gap_set = gap_set_m(v_l, t_follow)
  if gap_set_override_m is not None:
    gap_set = float(gap_set_override_m)
  slack = gap_f - gap_set
  v_err = v_l - v_e  # lead faster is positive, matching k_v(v_lead - v_ego)
  v_close = max(0.0, -v_err)

  sev = _severity(slack, v_err)
  k_g = K_G_LO + (K_G_HI - K_G_LO) * sev
  # High severity uses the smaller velocity gain (K_V_HI < K_V_LO).
  k_v_close = K_V_LO + (K_V_HI - K_V_LO) * sev
  locked = gap_set_override_m is not None
  if locked:
    # Closing gain on both sides, and not the small near-gap opening
    # gain. A leftover speed deficit has to fold back onto the locked
    # meters in a few seconds. 0.40 /s is a 1 m/s miss at 0.40 m/s²,
    # still well under the follow ceiling.
    k_v = max(k_v_close, 0.40)
  else:
    # A far, slow close coasts: the closing gain fades with time-to-gap.
    ttg_far = slack / max(v_close, 0.3)
    k_v_close *= 1.0 - (1.0 - K_V_FAR_MIN) * _smooth01(
      (ttg_far - K_V_FAR_TTG_LO_S) / max(1e-3, K_V_FAR_TTG_HI_S - K_V_FAR_TTG_LO_S))
    # Opening: small gain at/over the setpoint (a lead pulling away is not
    # chased), growing with slack. Inside the gap the opening speed keeps the
    # closing gain so braking unwinds as soon as the gap starts recovering.
    open_far = _smooth01((slack - 5.0) / 30.0)
    k_v_open = K_V_OPEN_NEAR + (K_V_OPEN_FAR - K_V_OPEN_NEAR) * open_far
    inside_w = _smooth01(-slack / OPEN_INSIDE_M)
    k_v_open = inside_w * k_v_close + (1.0 - inside_w) * k_v_open
    # Blend the two across v_err = 0 so the gain has no step.
    side = _smooth01((v_err + 0.3) / 0.6)
    k_v = (1.0 - side) * k_v_close + side * k_v_open
  k_a = _k_a(slack) * _k_a_brake_scale(a_l)
  glide = _glide(slack, v_err, a_l)
  glide_keep = GLIDE_KEEP + (GLIDE_KEEP_INSIDE - GLIDE_KEEP) * _smooth01(-slack / GLIDE_GAP_M)
  keep = 1.0 - glide_keep * glide
  a_gv = (k_g * slack + k_v * v_err) * keep
  a_gv += _short_headway(gap_f, v_e, v_close)
  a_pd = a_gv + k_a * a_l

  a_kin = _a_kin(slack, v_close, gap_f, v_e, t_follow, v_l)
  # Braking-lead contribution on the bound. Positive a_lead stays in a_pd only.
  a_bound = a_kin + min(a_l, 0.0) * k_a
  # Command copy. Far slack and a long time-to-gap drop a mild or mid lead
  # brake. a_pd and a_bound stay unfaded: catch-up and the kinematic gate
  # still see a real lead brake, so they do not open just because the
  # command stopped copying it. Near the gap, a short time-to-gap, or an
  # emergency lead decel keeps the full term on the command too.
  if rapid_path_w is None:
    path_credit = rapid_path_credit(rapid_path_offset_m(y_rel, curvature, gap_f, path_lat))
  elif not math.isfinite(float(rapid_path_w)):
    path_credit = 1.0
  else:
    path_credit = min(1.0, max(0.0, float(rapid_path_w)))
  # Far rapid decel is copied only while the lead is on the predicted path.
  # Near / short-TTC weight does not read path_credit.
  brake_w = _braking_alead_weight(slack, v_close, a_l, path_credit)
  if a_l < 0.0 and brake_w < 1.0:
    a_pd_cmd = a_gv + k_a * a_l * brake_w
    a_bound_cmd = a_kin + a_l * k_a * brake_w
  else:
    a_pd_cmd = a_pd
    a_bound_cmd = a_bound
  # Swap only the command's gap piece. The gate below still reads a_pd.
  a_gap_classic = (k_g * slack) * keep
  if not locked:
    a_gap = _far_gap_catchup_gap(a_gap_classic, slack, v_err, keep, a_pd, a_bound)
    a_pd_cmd = a_pd_cmd + (a_gap - a_gap_classic)
  gate_w = KIN_GATE_MS + _kin_gate_widen(slack, a_pd, a_bound)
  gate = _smooth01(v_close / gate_w) if v_close > 0.0 else 0.0
  limited = min(a_pd_cmd, a_bound_cmd)
  a_cmd = ((1.0 - gate) * a_pd_cmd) + (gate * limited)

  # Pulling away near the gap: trickle, not a catch-up lunge.
  # A locked setpoint is the distance the driver asked to hold, so a
  # positive error is not trickled down to the 0.12 m/s² cap.
  if a_cmd > 0.0 and not locked:
    # Compression weight rises across the setpoint (recovering from inside
    # is not compressed) and fades once the slack is a real catch-up.
    far = _smooth01((slack - TRICKLE_FAR_LO_M) / (TRICKLE_FAR_HI_M - TRICKLE_FAR_LO_M))
    w = _smooth01((slack + TRICKLE_IN_M) / TRICKLE_IN_M) * (1.0 - far)
    trickle = TRICKLE_A * math.tanh(a_cmd / TRICKLE_SCALE)
    a_cmd = w * min(a_cmd, trickle) + (1.0 - w) * a_cmd

  # Off-path / leaving lead: its command (brake included) fades toward 0.
  # Curve still applies below. MAX applies only when the gap is not locked.
  fade = _leave_fade(y_rel, curvature, gap_f, path_lat, leave_w)
  if fade > 0.0:
    a_cmd *= (1.0 - fade)

  # Achievable hard decel follows trust: a remote, off-path, or weak lead
  # cannot brake us past what its kinematics need (Sep 28 11:07:22).
  trust = lead_trust(gap_f, v_e, v_l, y_rel, curvature, path_lat, model_prob, radar)
  # Floor only. The command above already used the unscaled lead accel.
  # Off-path far rapid decel must not keep a deep floor that re-firms the blend.
  a_l = credited_rapid_lead(a_l, slack, v_close, path_credit)
  a_cmd = trust_bound(a_cmd, hard_decel_floor(gap_f, v_e, v_l, a_l, t_follow), trust)

  # A locked gap is the driver's setpoint. MAX does not hold it back.
  # A curve cap, when the planner passes one, still does.
  if (not locked) and v_ceiling is not None and float(v_ceiling) > 0.5:
    a_cmd = min(a_cmd, speed_ceiling_accel(v_e, float(v_ceiling)))
  if v_curve_cap is not None and float(v_curve_cap) > 0.5:
    a_cmd = min(a_cmd, speed_ceiling_accel(v_e, float(v_curve_cap)))
  if a_map is not None:
    a_cmd = min(a_cmd, float(a_map))
  if a_cmd < A_MIN_MS2:
    a_cmd = A_MIN_MS2
  elif a_cmd > A_MAX_MS2:
    a_cmd = A_MAX_MS2
  return a_cmd


def lead_quality(model_prob: float | None = None, radar: bool | None = None) -> float:
  """Per-frame reading quality in [0, 1]. Radar-associated reads full.

  Vision-only reads scale with model probability, never below a floor so
  a real vision lead still owns the command after it has been consistent.
  """
  if radar is True or model_prob is None:
    return 1.0
  p = float(model_prob)
  if not math.isfinite(p):
    return 1.0
  return max(CONF_VISION_MIN, min(1.0, p / 0.6))


def _limit_jerk(prev: float, target: float, dt: float, *, down: float, up: float) -> float:
  delta = float(target) - float(prev)
  if delta > up:
    return float(prev) + up
  if delta < -down:
    return float(prev) - down
  return float(target)


def approach_physics_need(gap: float, v_ego: float, v_lead: float, t_follow: float) -> float:
  """Decel (<= 0) that stops the close before eating the margin.

  v_close² / (2 · max(slack − margin, room floor)). 0 when not closing.
  Comfort may not publish a softer command than this when it is firmer
  than the EV mild settle.
  """
  slack = float(gap) - gap_set_m(v_lead, t_follow)
  v_close = max(0.0, float(v_ego) - float(v_lead))
  if v_close <= 0.0:
    return 0.0
  room = max(slack - APPROACH_NEED_MARGIN_M, APPROACH_NEED_ROOM_MIN_M)
  return max(A_MIN_MS2, -(v_close * v_close) / (2.0 * room))


def lead_free_accel(v_ego: float, v_ceiling: float | None, a_map: float | None,
                    v_curve_cap: float | None) -> float:
  """Accel with no lead: hold speed, unless MAX, curve, or map brakes.

  Under a speed cap the ceiling term is positive (accel is allowed). The
  cut-in hold mins this with the trickled follow command, so a lead that
  is pulling away is not chased inside the follow window.
  """
  a = A_MAX_MS2
  limited = False
  if v_ceiling is not None and float(v_ceiling) > 0.5:
    a = min(a, speed_ceiling_accel(float(v_ego), float(v_ceiling)))
    limited = True
  if v_curve_cap is not None and float(v_curve_cap) > 0.5:
    a = min(a, speed_ceiling_accel(float(v_ego), float(v_curve_cap)))
    limited = True
  if a_map is not None:
    a = min(a, float(a_map))
    limited = True
  if not limited:
    return 0.0
  return float(a)


def projected_gap_m(gap: float, v_ego: float, v_lead: float, a_lead: float,
                    horizon: float = CUTIN_PROJ_T_S) -> float:
  """Minimum gap over the next `horizon` seconds at the lead's decel.

  A positive lead accel is not credited: the lead is not assumed to pull
  away harder than it is pulling away now. With non-positive lead accel
  the gap parabola opens downward, so the minimum on the window is at an
  endpoint.
  """
  a = min(0.0, float(a_lead))
  v = float(v_lead) - float(v_ego)
  g0 = max(0.0, float(gap))
  g1 = g0 + v * float(horizon) + 0.5 * a * float(horizon) * float(horizon)
  return min(g0, g1)


class ApproachInputs:
  """Low-pass and hysteresis on the slow-close follow inputs.

  Closing speed and gap are filtered. Crossing the old 6 m/s rapid line
  takes a dwell, and the exit is lower than the enter, so one noisy
  sample cannot flip the regime. `use_comfort` is the slow, non-braking
  close: that is where the filtered gap and closing speed replace the
  raw sample. A braking lead and a confirmed rapid close stay on the
  unfiltered law (#222, fast dump).
  """

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.gap_f: float | None = None
    self.v_close_f: float | None = None
    self.rapid = False
    self._enter_s = 0.0
    self._exit_s = 0.0

  def update(self, gap: float, v_ego: float, v_lead: float, dt: float) -> None:
    frame = 0.05 if dt is None or float(dt) <= 1e-6 else float(dt)
    v_close = float(v_ego) - float(v_lead)
    first = self.v_close_f is None
    self.gap_f = _filt(self.gap_f, float(gap), frame, APPROACH_TAU_GAP_S)
    self.v_close_f = _filt(self.v_close_f, v_close, frame, APPROACH_TAU_CLOSE_S)
    if first:
      # An obvious rapid close is rapid on the first sample. The dwell
      # is for the boundary, not for a 13 m/s acquire.
      self.rapid = v_close >= APPROACH_RAPID_ENTER_MS
      self._enter_s = 0.0
      self._exit_s = 0.0
      return
    if not self.rapid:
      if v_close >= APPROACH_RAPID_ENTER_MS:
        self._enter_s += frame
        if self._enter_s + 1e-9 >= APPROACH_RAPID_DWELL_S:
          self.rapid = True
          self._enter_s = 0.0
      else:
        self._enter_s = 0.0
      self._exit_s = 0.0
    else:
      if v_close <= APPROACH_RAPID_EXIT_MS:
        self._exit_s += frame
        if self._exit_s + 1e-9 >= APPROACH_RAPID_DWELL_S:
          self.rapid = False
          self._exit_s = 0.0
      else:
        self._exit_s = 0.0
      self._enter_s = 0.0

  def use_comfort(self, a_lead: float) -> bool:
    """True when the filtered inputs should feed the follow law."""
    if float(a_lead) <= APPROACH_BRAKE_A_MS2:
      return False
    if self.rapid or self.v_close_f is None:
      return False
    return float(self.v_close_f) >= APPROACH_COMFORT_CLOSE_MS


class UnifiedLeadController:
  """Stateful filters, lead confidence, and jerk limit around unified_follow_desired."""

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self._v_f: float | None = None
    self._a_f: float | None = None
    self._lead_jerk = 0.0
    self._prev: float | None = None
    self._lead_id = None
    self._conf = 1.0
    self._cutin_from = 0.0
    self.last_desired = 0.0
    self.last_v_lead = 0.0
    self.last_a_lead = 0.0
    self.last_a_lead_eff = 0.0
    self.last_confidence = 0.0
    self.last_trust = 1.0
    self.last_floor = A_MIN_MS2
    self._rapid_y: float | None = None
    self.last_rapid_path_credit = 1.0
    self._approach = ApproachInputs()
    self._track_age = 0.0
    self._vrel_f: float | None = None  # v_lead - v_ego, positive opens
    self._pullaway = False
    self._close_s = 0.0
    self._pred_s = 0.0
    self._hard_n = 0
    self.pulling_away = False

  @property
  def _cutin_w(self) -> float:
    return self._conf

  def step(self, *, dt: float, present: bool, gap: float, v_ego: float,
           v_lead: float, a_lead: float, t_follow: float, lead_id,
           seed_a: float = 0.0, v_ceiling: float | None = None,
           a_map: float | None = None, y_rel: float = 0.0,
           curvature: float = 0.0, a_max: float | None = None,
           path_lat: float | None = None, model_prob: float | None = None,
           radar: bool | None = None, leave_w: float = 0.0,
           gap_set_override_m: float | None = None,
           v_curve_cap: float | None = None) -> float:
    """One planner frame. Returns the slewed road-relative accel, or 0 with no lead."""
    frame_dt = 0.05 if dt is None or float(dt) <= 1e-6 else float(dt)
    if not present:
      self.reset()
      return 0.0

    # No command of our own yet: the first frame starts from seed_a.
    fresh = self._prev is None
    if lead_id != self._lead_id:
      # New lead or cut-in: blend in from the command already in force.
      self._cutin_from = float(seed_a if self._prev is None else self._prev)
      self._conf = 0.0
      self._lead_id = lead_id
      # Do not inherit the previous lead's accel. Speed starts at this sample.
      # A pulling-away cut-in is seeded from the first radar accel so the
      # 0 → aLeadK step is not read as brake onset. Every other new lead
      # still ramps from 0: copying a closing lead's decel on the first
      # frame is the Sep 28 far-lead flash, and a mild opening lead
      # (about +0.5 m/s, aLeadK near −0.3) keeps the onset the plant
      # already follows down to the −0.22 settle.
      self._v_f = float(v_lead)
      opening0 = float(v_lead) - float(v_ego)
      if opening0 >= CUTIN_PULL_ENTER_MS:
        self._a_f = float(a_lead)
      else:
        self._a_f = 0.0
      self._lead_jerk = 0.0
      self._rapid_y = None
      self._approach.reset()
      self._track_age = 0.0
      opening = float(v_lead) - float(v_ego)
      self._vrel_f = opening
      # A new lead that is already pulling away does not get lead braking.
      # An existing lead that later becomes faster does not enter here.
      self._pullaway = opening >= CUTIN_PULL_ENTER_MS
      self._close_s = 0.0
      self._pred_s = 0.0
      self._hard_n = 0
      if self._prev is None:
        self._prev = float(seed_a)

    # Path consistency for the far rapid-brake path. A one-frame yRel
    # spike moves this about a fifth of the way, so a ghost flicker does
    # not firm ego braking and a true in-path lead does not drop out.
    y_abs = rapid_path_offset_m(y_rel, curvature, gap, path_lat)
    self._rapid_y = _filt(self._rapid_y, y_abs, frame_dt, TAU_RAPID_PATH_S)
    path_credit = rapid_path_credit(self._rapid_y)
    self.last_rapid_path_credit = float(path_credit)

    self._track_age += frame_dt
    self._approach.update(gap, v_ego, v_lead, frame_dt)
    opening = float(v_lead) - float(v_ego)
    self._vrel_f = _filt(self._vrel_f, opening, frame_dt, CUTIN_VREL_TAU_S)
    self._update_pullaway(frame_dt, float(gap), float(v_ego), float(v_lead), float(a_lead))

    self._v_f = _filt(self._v_f, float(v_lead), frame_dt, TAU_V_S)
    a_prev = self._a_f
    self._a_f = _filt(self._a_f, float(a_lead), frame_dt, TAU_A_S)
    jerk_raw = (self._a_f - a_prev) / frame_dt if a_prev is not None else 0.0
    self._lead_jerk = _filt(self._lead_jerk, jerk_raw, frame_dt, LEAD_JERK_TAU_S)
    # While a pulling-away cut-in is still new, a mild aLeadK is not a
    # brake we should copy. A hard brake (<= −1.5) is used immediately.
    ignore_alead = (
      self._pullaway
      and self._track_age < CUTIN_ALEAD_IGNORE_S
      and float(a_lead) > CUTIN_HARD_A_MS2
    )
    if ignore_alead:
      a_eff = 0.0
    else:
      a_eff = self._a_f - lead_decel_anticipation(self._a_f, self._lead_jerk, gap, v_ego)
    self.last_v_lead = float(self._v_f)
    self.last_a_lead = float(self._a_f)
    self.last_a_lead_eff = float(a_eff)

    law_kw = dict(
      v_ceiling=v_ceiling, a_map=a_map, y_rel=y_rel, curvature=curvature,
      path_lat=path_lat, leave_w=leave_w, model_prob=model_prob, radar=radar,
      gap_set_override_m=gap_set_override_m, v_curve_cap=v_curve_cap,
      rapid_path_w=path_credit,
    )
    desired = unified_follow_desired(gap, v_ego, self._v_f, a_eff, t_follow, **law_kw)
    # Slow non-braking close: feed the low-passed gap and closing speed.
    # If that removes braking physics still needs, keep the unfiltered law.
    if self._approach.use_comfort(float(a_lead)) and self._approach.v_close_f is not None:
      v_lead_f = float(v_ego) - float(self._approach.v_close_f)
      gap_f = float(self._approach.gap_f if self._approach.gap_f is not None else gap)
      filtered = unified_follow_desired(gap_f, v_ego, v_lead_f, a_eff, t_follow, **law_kw)
      need = approach_physics_need(gap, v_ego, v_lead, t_follow)
      # Filtered inputs may ease a command only when physics does not need
      # it. They must not add braking the unfiltered law did not ask for,
      # and they must not drop a brake that is still required.
      if filtered > desired and need < -APPROACH_MILD_MS2:
        pass
      elif filtered <= desired:
        pass
      else:
        desired = filtered
    if a_max is not None:
      desired = min(desired, float(a_max))
    if self._pullaway:
      # Zero lead-induced decel. Positive follow stays, already trickled
      # inside the window, and is capped by the lead-free limit. MAX,
      # curve, and map braking still apply through that limit. Pin the
      # blend source so a rising confidence cannot fade the brake back in.
      free = lead_free_accel(float(v_ego), v_ceiling, a_map, v_curve_cap)
      desired = min(free, max(float(desired), 0.0))
      # Inside the follow window a pulling-away lead is not chased.
      # The existing trickle is the positive cap; a limit brake stays.
      gap_set = gap_set_m(float(v_lead), t_follow)
      if gap_set_override_m is not None:
        gap_set = float(gap_set_override_m)
      if desired > TRICKLE_A and float(gap) <= gap_set:
        desired = TRICKLE_A
      if a_max is not None:
        desired = min(desired, float(a_max))
      self._cutin_from = float(desired)
    self.pulling_away = bool(self._pullaway)
    self.last_desired = desired

    v_err = float(self._v_f) - float(v_ego)
    gap_set = gap_set_m(self._v_f, t_follow)
    if gap_set_override_m is not None:
      gap_set = float(gap_set_override_m)
    slack = float(gap) - gap_set
    a_kin = _a_kin(slack, max(0.0, -v_err), max(0.0, float(gap)), float(v_ego),
                   t_follow, float(self._v_f))

    # Confidence: consistent, on-path, good-quality readings ramp to 1.
    # A fast close (deep kinematic demand) ramps within a fraction of a
    # second; a one-frame reading barely moves it.
    # Confidence reads the static off-path geometry only. A leaving lead is
    # released through `desired` (leave_w above), toward the lead-free
    # command; a confidence drop would blend toward the command in force,
    # which is the lead's brake itself.
    fade = _leave_fade(y_rel, curvature, gap, path_lat, leave_w)
    on_path = 1.0 - _depart_fade(y_rel, curvature, gap, path_lat)
    target_conf = on_path * lead_quality(model_prob, radar)
    urgency = _smooth01((-a_kin - CONF_URGENT_LO_MS2) / CONF_URGENT_BAND_MS2)
    if target_conf >= self._conf:
      rate = CONF_RATE_BASE_1_S + CONF_RATE_URGENT_1_S * urgency * self._conf
    else:
      rate = CONF_RATE_FALL_1_S
    self._conf = self._conf + min(1.0, rate * frame_dt) * (target_conf - self._conf)
    self._conf = min(1.0, max(0.0, self._conf))
    self.last_confidence = float(self._conf)

    # Off-path leads already fade inside `desired`; confidence blends a new
    # or weak reading from the command in force before this lead.
    w = self._conf if lead_id == self._lead_id else 0.0
    # The command a lead blends from obeys a trust bound too: an inherited
    # command firmer than both this lead's own demand and its kinematic
    # floor (legacy seed after a gas override, a previous lead) is kept only
    # in proportion to trust × confidence. A new or remote lead cannot carry
    # it (Sep 28 11:07:22). MAX / map terms keep their own depth.
    trust = lead_trust(gap, v_ego, self._v_f, y_rel, curvature, path_lat, model_prob, radar)
    a_floor_lead = credited_rapid_lead(
      float(self._a_f), slack, max(0.0, -v_err), path_credit)
    floor = hard_decel_floor(gap, v_ego, self._v_f, a_floor_lead, t_follow)
    if gap_set_override_m is None and v_ceiling is not None and float(v_ceiling) > 0.5:
      floor = min(floor, speed_ceiling_accel(float(v_ego), float(v_ceiling)))
    if v_curve_cap is not None and float(v_curve_cap) > 0.5:
      floor = min(floor, speed_ceiling_accel(float(v_ego), float(v_curve_cap)))
    if a_map is not None:
      floor = min(floor, float(a_map))
    carry = min(desired, floor)
    carry_trust = trust * self._conf
    self.last_trust = float(trust)
    self.last_floor = float(floor)
    self._cutin_from = trust_bound(self._cutin_from, carry, carry_trust)
    target = ((1.0 - w) * self._cutin_from) + (w * desired)
    if self._pullaway:
      # The held command is the blend. Confidence must not fade a lead
      # brake back in, and the handover later slews from this command.
      target = float(desired)
    prev = float(self._prev if self._prev is not None else seed_a)
    if fresh:
      prev = trust_bound(prev, carry, carry_trust)

    opening = v_err > 0.25 and slack > 1.0
    release_base = JERK_RELEASE_OPEN_MS3 if (opening or fade > 0.35) else JERK_RELEASE_MS3
    # The frame a lock replaces a time-gap ease, the command in force is
    # still that brake. Release it at the physical jerk so the gap does
    # not keep opening while the slew catches up. Braking (target below
    # the command) is unchanged.
    if gap_set_override_m is not None and float(target) > prev:
      release_base = JERK_PHYS_MS3
    err = float(target) - prev
    # Proportional: far too firm releases fast, near the target gently.
    up = min(JERK_PHYS_MS3, release_base + JERK_PROP_1_S * max(0.0, err)) * frame_dt
    build_base = JERK_PHYS_MS3 if a_kin < prev - 0.08 else JERK_BUILD_MS3
    down = min(JERK_PHYS_MS3, build_base + JERK_PROP_1_S * max(0.0, -err)) * frame_dt
    nxt = _limit_jerk(prev, target, frame_dt, down=down, up=up)
    self._prev = nxt
    # Once a lead is trusted, the blend-from value follows the published
    # command so a later confidence dip fades toward current behavior.
    self._cutin_from += (nxt - self._cutin_from) * self._conf * min(1.0, frame_dt / MODE_BLEND_S)
    if self._pullaway:
      # Pin to the published command so leaving the hold cannot step.
      self._cutin_from = float(nxt)
    return nxt

  def _update_pullaway(self, dt: float, gap: float, v_ego: float, v_lead: float,
                       a_lead: float) -> None:
    """Hold or release the pulling-away cut-in. Release is one-way."""
    if not self._pullaway:
      return
    if float(a_lead) <= CUTIN_HARD_A_MS2:
      self._hard_n += 1
    else:
      self._hard_n = 0
    v_open = 0.0 if self._vrel_f is None else float(self._vrel_f)
    if -v_open >= CUTIN_CLOSE_MS:
      self._close_s += float(dt)
    else:
      self._close_s = 0.0
    pred = v_open + min(float(a_lead), 0.0) * CUTIN_PRED_T_S
    if pred <= -CUTIN_CLOSE_MS:
      self._pred_s += float(dt)
    else:
      self._pred_s = 0.0
    keep = CUTIN_KEEP_M + CUTIN_KEEP_HW_S * max(0.0, float(v_ego))
    headway = float(gap) / max(float(v_ego), 1.0)
    release = (
      self._hard_n >= CUTIN_HARD_FRAMES
      or self._close_s + 1e-9 >= CUTIN_CLOSE_HOLD_S
      or self._pred_s + 1e-9 >= CUTIN_PRED_HOLD_S
      or projected_gap_m(gap, v_ego, v_lead, a_lead) < keep
      or headway < CUTIN_HEADWAY_S
    )
    if release:
      self._pullaway = False
      self._close_s = 0.0
      self._pred_s = 0.0
