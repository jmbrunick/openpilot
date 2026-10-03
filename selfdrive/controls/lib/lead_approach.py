"""Shared lead / follow helpers kept after the unified controller became the sole path.

The earlier (#214-#244) lead-approach overlay chain is gone: the planner now
hands every live or held lead to `unified_lead.UnifiedLeadController`. What
remains here is consumed outside that overlay:

- lead hold tracking and approach gating for the planner
  (`resolve_lead_close_hold`, `lead_close_should_cap`, `lead_approach_track_ok`)
- LongControl's follow actuator guard and residual window
  (`guard_follow_actuator_regen`, `LeadResidualWindow`)
- the Pre-AP plant regen guard installed by card.py
  (`install_preap_plant_regen_guard`, `plant_regen_effort_limits`)
- follow-distance helpers (`nap_t_follow`, `NAP_T_FOLLOW`, `lead_follow_slack_m`)
"""

from __future__ import annotations

import math
from collections import deque

from openpilot.selfdrive.mapd.constants import TRACK_DEADBAND_MS



# Keep in sync with long_mpc.STOP_DISTANCE (acados cruise/lead obstacle).
STOP_DISTANCE = 6.0


# Comfort peak |a|. Early map brake, not Normal 0.80 — Tesla VirtualDAS
# regen at 0.80 then rematch chatters on a slight grade. Do not raise.
# MPC 2.5 / FCW still own danger. Rapid closes only.
LEAD_APPROACH_A_MS2 = 0.55


# Usable Bosch ceiling. Old 140 m waited until late in the gap; 200 m is
# still inside typical Pre-AP Bosch reports. Anti-flicker is quality +
# hysteresis, not a short ceiling — see lead_approach_track_ok.
LEAD_APPROACH_MAX_START_M = 200.0


# Hold a few meters past the ceiling so a track at 199–201 m does not chatter.
LEAD_APPROACH_MAX_HOLD_M = 8.0


# Inside this, leadOne.status is enough. Beyond it, require radar
# association so vision-only far flicker cannot own the plan. Radar
# tracks do not wait on modelProb. LeadData has no track age on Pre-AP.
LEAD_APPROACH_RELIABLE_M = 140.0


LEAD_APPROACH_MODEL_PROB_MIN = 0.50  # radard association gate


# Mild-close comfort ceiling. Kinematics used to hit 0.55 on a 3–10 mph
# close right at the gap (hard let-off). Light regen / ease-off only.
# Rapid (high closing rate) keeps the 0.55 path.
LEAD_APPROACH_MILD_A_MS2 = 0.22


# Rapid / dumping: closing rate high (much faster than lead). ~13 mph.
# Short TTC at a mild v_rel is "almost at the gap", not dumping — do not
# promote that to 0.55.
LEAD_APPROACH_RAPID_DV_MS = 6.0


# Closing this fast can release the MILD floor, but only with brake
# evidence: firm aLead (below −0.35), #222 residual / rising close,
# confirmed rapid, FCW, shouldStop, or near-bumper. 1.5 m/s alone is
# a non-braking catch-up (22:12:04) and stays on MILD.
LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS = 1.5


# Lead clearly braking. −0.2 is a real coast/brake, not aLeadK noise at 0.
LEAD_APPROACH_SOFT_LIMIT_ALEAD_MS2 = -0.2


# Owned-lead rising close. Above rematch-enter jitter (0.55) so a
# matched follow whose closing starts climbing is the brake signal
# before 1.5. One-frame radar chatter must not count as a rise.
LEAD_SOFT_LIMIT_RISE_MS = 0.15


# Unexplained closing accel after subtracting ego's own a. Same
# spirit as aLead ≤ −0.2: residual ≥ 0.2 means the lead is braking.
LEAD_SOFT_LIMIT_RESIDUAL_MS2 = 0.2


# #222 residual / rising close, de-noised. One radar LSB (1/16 m/s) over
# a planner frame is 1.25 m/s² and used to cancel every comfort floor.
# Require a ~0.5 s window of measured accel, then two frames of it.
LEAD_RESIDUAL_WINDOW_S = 0.50


LEAD_RESIDUAL_SUSTAIN_MS2 = 0.45


LEAD_RESIDUAL_SUSTAIN_N = 2


LEAD_RESIDUAL_WINDOW_RISE_MS = 0.25


# Closing ≥ 1 m/s and mild (−0.22) cannot finish at Follow Distance:
# brake the kinematic need, capped short of the regen rail.
# The planned approach is sized for LEAD_APPROACH_A_MS2 (0.55). A gap
# at that slack stays mild. Deepen only once slack is clearly inside it.
LEAD_KIN_APPROACH_CAP_MS2 = 0.60


# While slack is still above the 3 m floor, the late bite stops at −0.45
# so a hot approach does not land on the regen rail at Follow Distance.
# Inside 3 m the −0.60 cap remains.
LEAD_KIN_APPROACH_OPEN_CAP_MS2 = 0.45


LEAD_KIN_APPROACH_BIAS_MS2 = 0.05


LEAD_KIN_APPROACH_SLACK_FLOOR_M = 3.0


LEAD_KIN_APPROACH_PAST_MILD_MS2 = 0.15


LEAD_KIN_APPROACH_DESIGN_MARGIN_M = 0.5


# Bias-corrected IMU: orientationNED[1] reads about +0.02 rad uphill.
# Grade effort is sin(pitch − bias) · g. Plant opens below −0.08 m/s².
# Profile sizing opens below −1.5% (~−0.147 m/s²) or a stuck-positive shortfall.
PLANT_PITCH_BIAS_RAD = 0.02


PLANT_GRAVITY_MS2 = 9.81


PLANT_DESCENT_GRADE_MS2 = -0.08


PLANT_DESCENT_SHORTFALL_MS2 = 0.15


PLANT_DESCENT_SHORTFALL_HOLD_S = 1.5


PLANT_PID_WINDOW_S = 1.0


PLANT_PID_SHORTFALL_MS2 = 0.10


# Extra effort room so the inner PID can trim down. Not the regen rail.
PLANT_PID_TRIM_CAP_MS2 = 0.30


# Firm lead (aLead < −0.35), closing ≥ 1.5, slack still 20–50 m: start
# the kinematic brake now instead of sitting on mild until the gap is
# short. Cap ~−1.0. Inside 20 m, confirmed rapid, and near-bumper stay
# on full #222 authority.
LEAD_FIRM_EARLY_CAP_MS2 = 1.0


# Never rematch / cruise +a into a live or held closing gap. 1.0 m/s
# (~2.2 mph) is above rematch-enter jitter; 1.5 is match-speed / ownership.
LEAD_CLOSING_REMATCH_BLOCK_MS = 1.0


LEAD_CLOSING_ALEAD_MS2 = LEAD_APPROACH_SOFT_LIMIT_ALEAD_MS2


# aLead-only ownership / match-aLeadK. Opening or slack ≳ 20 m must not
# force aTarget ≤ aLeadK (far braking lead, gap opening). Near-gap
# braking-lead protection still matches. 20 m is inside the 15–25 m tune.
LEAD_ALEAD_MATCH_SLACK_M = 20.0


# Enter / exit (hysteresis). A single v_rel / slack gate chatters around
# the follow gap on a slight incline (regen ↔ accel). First pass was
# 0.50 / 0.20; leftover bump-pull was rematch re-crossing 0.50. Raise
# enter only — a 0.12 exit held ease too long and parked far back.
LEAD_APPROACH_DV_MS = 0.55         # enter: ~1.2 mph closing; ignore radar jitter


NAP_T_FOLLOW = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)


# Same Bosch ceiling as ease. A 160–180 m same-speed lead used to skip the
# cap and punch cruise / MAX-rise to close Follow Distance. Do not.
LEAD_CLOSE_MAX_M = LEAD_APPROACH_MAX_START_M


# Near Follow Distance, rematch after ease must trickle. Large-gap
# catch-up uses Mannerisms Accel (same as open-road).
LEAD_CLOSE_OPENING_A_MS2 = 0.08


# Mid-gap rematch while still slowly closing (ef 10:18). Slack above
# the near-gap rematch band and not past 50 m, closing ~0.8–2.0 m/s:
# trickle, not Accel ceil. Slack > 50 is large-gap catch-up (Accel),
# even while still closing in-band. Opening / same-speed is Accel.
# Rapid / match-speed ≥ 1.5 still owns −a.
LEAD_MID_GAP_SLACK_M = 50.0


LEAD_MID_GAP_CLOSE_HI_MS = 2.0


# Map FOLLOW above MAX with a steady mid-gap lead (Scallywag 16:22:
# slack ~19 m, dRel ~60 m, closing ~0.5). Do not pass a −1.4…−3.5
# cruise/map cliff through. Comfort floor is the early map brake
# (−0.55), inside −0.4…−0.6, so the limit still comes back.
# Slack ~8–50 m. Farther than that, map decel still mins in.
LEAD_MAP_MIDGAP_SLACK_LO_M = 8.0


LEAD_MAP_MIDGAP_FLOOR_MS2 = 0.55


LEAD_MAP_MIDGAP_DREL_HI_M = 80.0


# Inside this dRel, never soften MPC −a (near bumper).
LEAD_MPC_SOFT_NEAR_M = 12.0


# Near-gap small ±a slew (rematch trickle ↔ leftover mild).
LEAD_NEAR_GAP_SLACK_M = 15.0


# Plant regen dump while the planner is coasting or easing. The steady
# band used to be |a| ≤ 0.08, so a commanded MILD (−0.22) unlocked the
# Pre-AP regen rail (18:10:08: aTarget −0.218, act −1.32). Cover
# |aTarget| ≤ MILD + ε so slight lift cannot full-lift. Planner ≤ −0.5
# / FCW / confirmed rapid / near-bumper still pass through.
LEAD_FOLLOW_STEADY_EPS_MS2 = 0.03


LEAD_FOLLOW_STEADY_A_MS2 = LEAD_APPROACH_MILD_A_MS2 + LEAD_FOLLOW_STEADY_EPS_MS2


LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2 = -LEAD_APPROACH_MILD_A_MS2


LEAD_FOLLOW_ACT_REGEN_CMD_MS2 = -0.50


# Cruise / weak-lead0 ACCEL_MIN where mid-gap and near-gap overlap.
# 0000010e 20:10:48 / 20:11:30 / 20:18:23: source cruise, slack ~12 m,
# close ~0.7, aLead ~−0.25, plan_min ≥ 0, aTarget −3. Near-gap match
# owns aLead ≤ −0.2, so the coast floor bailed. That aLead is not a
# firm brake (≲ −0.4 with a real close). aLead at or above this still
# floors; anything firmer stays on the raw near-gap path.
LEAD_CRUISE_CLIFF_ALEAD_MS2 = -0.35


# Internal v_rel = v_ego − v_lead (positive closes). Radar vRel is the
# opposite sign. Clearly opening, past match noise: 10:05 onset was
# about −0.56 (radar +0.56) with slack ~9 m and aLeadK only −0.32.
# A single sample milder than this can still be near-gap noise; a
# run of any opening vRel latches the release below.
LEAD_OPENING_VREL_MS = -0.25


# Brief hold of the last in-window lead when `leadOne.status` drops so
# cruise punch cannot leak through a radar flicker. ~10 planner frames.
LEAD_CLOSE_HOLD_S = 0.50


def nap_t_follow(nap_follow_dist: int | None) -> float | None:
  if nap_follow_dist in range(1, len(NAP_T_FOLLOW) + 1):
    return NAP_T_FOLLOW[nap_follow_dist - 1]
  return None


def lead_follow_slack_m(d_rel, v_lead, t_follow):
  """Meters above the selected Follow Distance, or None if unknown."""
  if d_rel is None or t_follow is None or float(t_follow) <= 0:
    return None
  d_follow = float(t_follow) * max(0.0, float(v_lead)) + STOP_DISTANCE
  return float(d_rel) - d_follow


def lead_midgap_comfort_excluded(v_rel, d_rel, slack, fcw=False, crash_cnt=0,
                                allow_rapid=False, a_lead=None,
                                prev_v_rel=None, a_ego=None, dt=None,
                                should_stop=False) -> bool:
  """True when a mid-gap comfort cap must not apply.

  FCW, shouldStop, rapid close, and near-bumper stay raw. Close ≥ 1.5
  alone does not: a non-braking catch-up stays in the cap. Firm aLead
  (below −0.35) or #222 residual / rising close still excludes.
  Outside the slack ~8–50 m band (or the dRel stand-in) is not this
  settle. Fast opening (|v_rel| ≥ ~2 m/s) is not a slow close.
  """
  _ = allow_rapid
  if should_stop or fcw or int(crash_cnt) > 0:
    return True
  if d_rel is not None and float(d_rel) <= LEAD_MPC_SOFT_NEAR_M:
    return True
  if d_rel is not None and (float(d_rel) - STOP_DISTANCE) <= 0.0:
    return True
  if v_rel is not None and lead_approach_is_rapid(float(v_rel)):
    return True
  if not lead_mid_gap_map_band(slack, d_rel):
    return True
  if v_rel is None:
    return False
  v = float(v_rel)
  if v >= LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS:
    # Catch-up speed is not a brake. 22:12:04 stayed on MILD until
    # slack crossed 20 m, then this gate unlocked the regen rail.
    return lead_firm_alead(a_lead) or lead_rising_or_residual_brake(
      v, prev_v_rel, a_ego, dt,
    )
  if abs(v) >= LEAD_MID_GAP_CLOSE_HI_MS:
    return True
  return False


def lead_near_gap_alead_raw(v_rel, a_lead, slack, owned=False, prev_v_rel=None) -> bool:
  """True when near-gap lead braking must stay raw.

  Slack at or under the near-gap band with a meaningful negative aLead
  is match-aLead (dRel = follow + 8 m, closing ~0.4 m/s, aLeadK −0.80)
  even while the MPC horizon is still cruise. Hold-owned is the same
  brake. A slack-~19 m cliff (−1.4…−3.5) with a coasting plan is not
  this path. Clearly opening with positive slack is not this path
  either — filtered aLeadK does not keep the cliff.
  """
  if slack is None or float(slack) > LEAD_NEAR_GAP_SLACK_M:
    return False
  if lead_kinematics_opening(v_rel, slack):
    return False
  line = lead_weak_alead_line(v_rel, prev_v_rel)
  if lead_alead_owns_match(v_rel, a_lead, slack, a_lead_ms2=line):
    return True
  if not owned or a_lead is None:
    return False
  return float(a_lead) <= line


def guard_follow_actuator_regen(actuator_a, planner_a, v_rel=None, d_rel=None,
                                slack=None, fcw=False, crash_cnt=0,
                                allow_rapid=False, v_ego=None, v_cruise=None,
                                a_lead=None, owned=False, prev_v_rel=None,
                                a_ego=None, dt=None, should_stop=False):
  """Do not dump firm plant regen on a coast or a mild ease.

  ef 10:18:42: aTarget ≈ 0 while actuators.accel hit −1.23. 18:09 /
  18:10: a coasting or MILD aTarget still reached the Pre-AP regen rail.
  When |aTarget| is inside the steady band (through MILD + ε), clip
  the actuator to the MILD slight-lift floor. Mid-gap slow close /
  settle hard-caps to that floor even if the planner command is
  already a cliff — PID / feedforward windup must not full-lift.
  Near-gap match-aLead is a real brake and is not that hard cap.
  Planner ≤ −0.5 outside that settle, FCW, confirmed rapid, and
  near-bumper pass through.
  """
  if actuator_a is None or planner_a is None:
    return actuator_a
  a = float(actuator_a)
  p = float(planner_a)
  floor = LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2
  emergency = (
    fcw or int(crash_cnt) > 0
    or (d_rel is not None and float(d_rel) <= LEAD_MPC_SOFT_NEAR_M)
    or (allow_rapid and v_rel is not None and lead_approach_is_rapid(float(v_rel)))
  )
  if emergency:
    return a
  # Large-slack firm lead: track the early kinematic brake, not the rail.
  # Confirmed rapid still falls through to full authority below.
  if not (v_rel is not None and lead_approach_is_rapid(float(v_rel))):
    firm_early = lead_firm_large_slack_a(v_rel, slack, a_lead)
    if firm_early is not None:
      return a if a >= firm_early else firm_early
  if (not lead_near_gap_alead_raw(
        v_rel, a_lead, slack, owned=owned, prev_v_rel=prev_v_rel)
      and not lead_midgap_comfort_excluded(
        v_rel, d_rel, slack, fcw=False, crash_cnt=0, allow_rapid=False,
        a_lead=a_lead, prev_v_rel=prev_v_rel, a_ego=a_ego, dt=dt,
        should_stop=should_stop,
      )):
    # Under the map, slight lift. Over the map, keep the #231 comfort
    # brake (−0.55) so a limit return is not lifted back to MILD.
    if lead_map_decel_above_max(v_ego, v_cruise):
      floor = -LEAD_MAP_MIDGAP_FLOOR_MS2
    else:
      kin = lead_kinematic_approach_a(v_rel, slack, a_lead)
      if kin is not None:
        floor = min(floor, kin)
    return a if a >= floor else floor
  if p <= LEAD_FOLLOW_ACT_REGEN_CMD_MS2:
    return a
  if abs(p) > LEAD_FOLLOW_STEADY_A_MS2:
    return a
  # Coast allows a slight lift. A command already at or below that
  # floor is tracked, but the plant may not go past it to the rail.
  allowed = p if p < floor else floor
  return a if a >= allowed else allowed


def plant_regen_effort_limits(a_cmd, limits, *, steady_grade=0.0, transient=0.0,
                              descent=False, pid_room=0.0):
  """VirtualDAS effort bounds while the command is coast or mild.

  Returns `limits` unchanged when the command is a firm brake. A
  coasting / MILD command cannot use the regen rail: the lower bound
  rises to the slight-lift floor. An existing tighter bound (engage
  grace) is kept.

  On a descent the floor applies to the net command, so the downhill
  grade term is not clipped off, and a sustained positive aEgo error
  may pull the inner PID through. Uphill and flat leave the floor on
  the command alone. Full regen stays reserved for cmd ≤ −0.60.
  """
  if a_cmd is None:
    return limits
  p = float(a_cmd)
  # Deeper than map comfort is a real brake: leave the regen rail open.
  if p < -LEAD_MAP_MIDGAP_FLOOR_MS2 - 0.05:
    return limits
  mild = LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2
  if p >= 0.0 or abs(p) <= LEAD_FOLLOW_STEADY_A_MS2:
    # Coast / throttle: slight lift only. A mild negative command is
    # tracked if it is already under that floor.
    floor = p if p < mild else mild
  else:
    # Between MILD and map comfort: track the command, do not open the rail.
    floor = p
  if descent:
    grade_sum = float(steady_grade) + float(transient)
    floor = floor + min(0.0, grade_sum)
    room = max(0.0, float(pid_room))
    if room > 0.0:
      floor -= min(room, PLANT_PID_TRIM_CAP_MS2)
    # nap_conf.REGEN_MAX. Do not import it here: that pulls capnp, and
    # the effort floor must stay inside the physical rail.
    floor = max(float(floor), -1.5)
  if limits is None:
    from opendbc.car.tesla.preap.nap_conf import ACCEL_MAX
    return (floor, float(ACCEL_MAX))
  lo, hi = float(limits[0]), float(limits[1])
  return (max(lo, floor), hi)


def bias_corrected_grade_ms2(pitch_rad):
  """IMU grade effort with the +0.02 rad uphill bias removed.

  None when pitch was not measured. A missing orientation must not
  look like a descent just because the bias term is negative.
  """
  if pitch_rad is None:
    return None
  return math.sin(float(pitch_rad) - PLANT_PITCH_BIAS_RAD) * PLANT_GRAVITY_MS2


def _preview_grade_compensation(estimator, orientation_ned):
  """Next-sample steady and transient grade, without stepping the filters.

  VirtualDAS.update applies the real step. The effort floor has to use
  the same terms or the downhill grade is clipped before it is added.
  """
  if estimator is None or orientation_ned is None or len(orientation_ned) < 2:
    return 0.0, 0.0
  from numpy import clip

  from opendbc.car.tesla.preap.virtual_das import (
    GRAVITY,
    MAX_PITCH_COMPENSATION,
    MAX_STEADY_GRADE_COMPENSATION,
    TRANSIENT_GRADE_GAIN,
  )

  maximum_pitch = math.asin(MAX_STEADY_GRADE_COMPENSATION / GRAVITY)
  pitch = float(clip(float(orientation_ned[1]), -maximum_pitch, maximum_pitch))
  lp = estimator.pitch_lp
  lp_x = (1.0 - lp._alpha) * lp.x + lp._alpha * pitch
  f1 = estimator.pitch_hp._f1
  f2 = estimator.pitch_hp._f2
  hp_x = (
    (1.0 - f1._alpha) * f1.x + f1._alpha * pitch
    - ((1.0 - f2._alpha) * f2.x + f2._alpha * pitch)
  )
  steady = float(clip(
    math.sin(lp_x) * GRAVITY,
    -MAX_STEADY_GRADE_COMPENSATION,
    MAX_STEADY_GRADE_COMPENSATION,
  ))
  transient = float(clip(
    math.sin(hp_x) * GRAVITY * TRANSIENT_GRADE_GAIN,
    -MAX_PITCH_COMPENSATION,
    MAX_PITCH_COMPENSATION,
  ))
  return steady, transient


def _plant_descent_allowance(vdas, a_cmd, a_ego, orientation_ned, dt):
  """Descent gate and PID room for one VirtualDAS sample.

  Grade gate is the bias-corrected IMU. The shortfall gate is aEgo − cmd
  above 0.15 for 1.5 s. PID room opens when the 1 s mean aEgo stays more
  than 0.10 above the command. State lives on the controller instance.
  """
  frame_dt = 0.02 if dt is None or float(dt) <= 1e-6 else float(dt)
  cmd = 0.0 if a_cmd is None else float(a_cmd)
  ego = 0.0 if a_ego is None else float(a_ego)
  hist = getattr(vdas, "_nap_descent_ego", None)
  if hist is None:
    hist = deque()
    vdas._nap_descent_ego = hist
  short_s = float(getattr(vdas, "_nap_descent_short_s", 0.0))
  if ego - cmd > PLANT_DESCENT_SHORTFALL_MS2:
    short_s += frame_dt
  else:
    short_s = 0.0
  vdas._nap_descent_short_s = short_s
  hist.append((frame_dt, ego, cmd))
  span = sum(sample_dt for sample_dt, _e, _c in hist)
  while span > PLANT_PID_WINDOW_S + 1e-9 and len(hist) > 1:
    drop_dt, _e, _c = hist.popleft()
    span -= drop_dt
  pitch = None
  if orientation_ned is not None and len(orientation_ned) >= 2:
    pitch = float(orientation_ned[1])
  grade = bias_corrected_grade_ms2(pitch)
  descent = (
    (grade is not None and grade < PLANT_DESCENT_GRADE_MS2)
    or short_s + 1e-9 >= PLANT_DESCENT_SHORTFALL_HOLD_S
  )
  pid_room = 0.0
  if descent and span + 1e-9 >= PLANT_PID_WINDOW_S:
    mean_ego = sum(sample_dt * sample_e for sample_dt, sample_e, _c in hist) / span
    mean_cmd = sum(sample_dt * sample_c for sample_dt, _e, sample_c in hist) / span
    mean_short = mean_ego - mean_cmd
    if mean_short > PLANT_PID_SHORTFALL_MS2:
      pid_room = min(PLANT_PID_TRIM_CAP_MS2, mean_short)
  return descent, pid_room


def install_preap_plant_regen_guard():
  """Keep Pre-AP pedal effort on a slight lift when the command is mild.

  LongControl is pure feedforward. The regen rail is VirtualDAS effort
  (grade + inner integral) after that seam. Card and the controller
  share a process, so the cap is installed on VirtualDAS.update.
  """
  from opendbc.car.tesla.pedal.controller import PEDAL_RAMP_RATE_UP
  from opendbc.car.tesla.preap.virtual_das import VirtualDAS

  current = VirtualDAS.update
  if getattr(current, "_nap_plant_regen_guard", False):
    return

  def update(self, a_cmd, v_ego, prev_pedal_di, a_ego=0.0, freeze_integrator=False,
             orientation_ned=None, accel_effort_limits=None,
             pedal_ramp_rate_up=PEDAL_RAMP_RATE_UP):
    steady, transient = _preview_grade_compensation(self.grade_estimator, orientation_ned)
    descent, pid_room = _plant_descent_allowance(
      self, a_cmd, a_ego, orientation_ned, self.dt,
    )
    return current(
      self, a_cmd, v_ego, prev_pedal_di, a_ego=a_ego,
      freeze_integrator=freeze_integrator, orientation_ned=orientation_ned,
      accel_effort_limits=plant_regen_effort_limits(
        a_cmd, accel_effort_limits,
        steady_grade=steady, transient=transient,
        descent=descent, pid_room=pid_room,
      ),
      pedal_ramp_rate_up=pedal_ramp_rate_up,
    )

  update._nap_plant_regen_guard = True
  update._nap_plant_regen_raw = current
  VirtualDAS.update = update


def lead_map_decel_above_max(v_ego, v_cruise) -> bool:
  """True when map is commanding over-MAX decel (past the deadband).

  lead_at_or_above_max includes sitting *at* MAX. First-latch acquire
  slew and the MILD floor must still apply there (e4 09:53:19). Map
  brake only starts once ego is faster than MAX by TRACK_DEADBAND —
  same edge as map_track_decel_ms2.
  """
  if v_ego is None or v_cruise is None:
    return False
  if float(v_ego) <= 0.0 or float(v_cruise) <= 0.0:
    return False
  return float(v_ego) > float(v_cruise) + TRACK_DEADBAND_MS


def lead_mid_gap_map_band(slack, d_rel=None) -> bool:
  """True when a live lead is in the mid-gap band for map comfort.

  Slack ~8–50 m when known (dig slack ~19 m). Without slack, dRel
  past the bumper and not a far lock (dig ~60 m) still qualifies.
  Near-bumper and large-gap catch-up stay out.
  """
  if slack is not None:
    s = float(slack)
    return LEAD_MAP_MIDGAP_SLACK_LO_M <= s <= LEAD_MID_GAP_SLACK_M
  if d_rel is None:
    return False
  d = float(d_rel)
  return LEAD_MPC_SOFT_NEAR_M < d <= LEAD_MAP_MIDGAP_DREL_HI_M


def lead_close_should_cap(d_rel, model_prob=None, radar=None, active=False) -> bool:
  """True when a detected lead is in the close-cap window.

  Bosch ceiling matches ease (200 m). Far tracks need radar association
  so a 160 m vision flicker cannot cap empty-road climb. Radar-only
  far locks do cap. Missing quality args (unit tests) are ok.
  """
  if d_rel is None:
    return False
  d = float(d_rel)
  d_max = LEAD_CLOSE_MAX_M + (LEAD_APPROACH_MAX_HOLD_M if active else 0.0)
  if d <= 0.0 or d > d_max:
    return False
  return lead_approach_track_ok(d, model_prob, radar, active=active)


def resolve_lead_close_hold(status, d_rel, v_lead, held_d, held_v, held_age, dt,
                            hold_s=LEAD_CLOSE_HOLD_S, model_prob=None, radar=None):
  """Lead used for the +a close cap, with a brief hold on status flicker.

  Live in-window lead wins, including a far Bosch track (160–200 m).
  A dropped `leadOne.status` keeps the last in-window lead for `hold_s`
  so cruise 1.6 cannot punch through a flicker. Past Bosch, or a
  vision-only far flicker with no hold, drops so empty-road MAX-rise
  is not stuck capped.

  Returns `(d_use, v_use, held_d, held_v, held_age)`. `d_use` is None
  when the cap should not apply.
  """
  holding = held_d is not None
  if status and lead_close_should_cap(d_rel, model_prob, radar, active=holding) and v_lead is not None:
    d = float(d_rel)
    v = float(v_lead)
    return d, v, d, v, 0.0

  past_bosch = (
    d_rel is not None and float(d_rel) > LEAD_CLOSE_MAX_M + LEAD_APPROACH_MAX_HOLD_M
  )
  if past_bosch or held_d is None or held_v is None:
    return None, None, None, None, 0.0

  age = float(held_age) + float(dt)
  if age > float(hold_s):
    return None, None, None, None, 0.0
  return float(held_d), float(held_v), float(held_d), float(held_v), age


def lead_approach_track_ok(d_rel, model_prob=None, radar=None, active=False) -> bool:
  """Far-track anti-flicker. LeadData has modelProb + radar; no track age.

  Inside RELIABLE_M, `leadOne.status` is enough (planner already gated).
  Beyond it, enter needs a radar-associated lead (`radar=True`) so a
  160–200 m Bosch lock can ease / show immediately. Vision-only far
  flicker stays off. Missing quality args (unit kinematics) are ok.
  Once active, hold through a brief quality dip so far tracks do not
  chatter.
  """
  if d_rel is None:
    return False
  d = float(d_rel)
  if d <= LEAD_APPROACH_RELIABLE_M or active:
    return True
  if radar is False:
    return False
  # Solid Bosch association: ease / cap from first far lock. Do not wait
  # on modelProb — that delayed yellow-arrow reaction on radar-only tracks.
  # Vision-only far flicker still stays off.
  if radar is True:
    return True
  if model_prob is not None and float(model_prob) < LEAD_APPROACH_MODEL_PROB_MIN:
    return False
  return True


def lead_opening_sample(v_rel, slack) -> bool:
  """True when internal v_rel is opening and slack is still positive.

  Inside the follow gap (slack ≤ 0) is a near-bumper / too-close bite,
  not this release. Radar vRel positive is this sign flipped.
  """
  if v_rel is None or slack is None:
    return False
  return float(v_rel) < 0.0 and float(slack) > 0.0


def lead_kinematics_opening(v_rel, slack) -> bool:
  """True when the gap is clearly opening, not match-noise.

  aLeadK alone must not own firm −a here. A milder negative v_rel can
  still be one noisy sample; `update_opening_release` latches a run
  of those.
  """
  if not lead_opening_sample(v_rel, slack):
    return False
  return float(v_rel) < LEAD_OPENING_VREL_MS


def lead_alead_owns_match(v_rel, a_lead, slack=None,
                         a_lead_ms2=LEAD_CLOSING_ALEAD_MS2) -> bool:
  """True when aLead alone may own match-speed −a / closing.

  aLead ≲ −0.2 used to own regardless of gap. A far or opening lead
  (da 2026-09-18: dRel~64 m, v_rel~−1.44 opening, aLeadK~−1.46) then
  snapped cruise +a to aLeadK. Gate: slack ≳ 20 m never owns from
  aLead alone. Clearly opening with positive slack never owns either
  (09:57 / 10:05: aLeadK ~−0.3…−0.4 on an opening gap must not skip
  the mild floor). Slight opening noise near the gap, and a real
  brake inside the follow gap, still match. Real closing ≳ 1.0–1.5
  is a separate v_rel gate.
  """
  if a_lead is None or float(a_lead) > float(a_lead_ms2):
    return False
  if slack is not None and float(slack) >= LEAD_ALEAD_MATCH_SLACK_M:
    return False
  if lead_kinematics_opening(v_rel, slack):
    return False
  if v_rel is not None and float(v_rel) < 0.0:
    return slack is not None
  return True


def lead_close_is_rising(v_rel, prev_v_rel, rise_ms=LEAD_SOFT_LIMIT_RISE_MS) -> bool:
  """True when closing rate increased vs the last owned-lead sample."""
  if v_rel is None or prev_v_rel is None:
    return False
  return float(v_rel) > float(prev_v_rel) + float(rise_ms)


def lead_residual_close_ms2(v_rel, prev_v_rel, a_ego, dt):
  """Unexplained closing accel after subtracting ego's own a.

  Δv_rel = a_ego·dt − a_lead·dt. Residual (Δv_rel − a_ego·dt) / dt
  is −a_lead: positive means the lead is braking harder than ego's
  command accounts for. None when a pair of samples is missing.
  """
  if v_rel is None or prev_v_rel is None or dt is None or float(dt) <= 1e-6:
    return None
  expected = 0.0 if a_ego is None else float(a_ego) * float(dt)
  return (float(v_rel) - float(prev_v_rel) - expected) / float(dt)


def lead_residual_window_hit(v_rel, prev_v_rel, a_ego, dt) -> bool:
  """True when a ~0.5 s window shows a real #222 residual or rise.

  One radar step over 0.05 s is not this. The legacy two-sample helper
  stays available for the Sep 23 fixtures; callers that arm the comfort
  floors use this window and then require two consecutive hits.
  """
  if v_rel is None or prev_v_rel is None or dt is None:
    return False
  if float(dt) + 1e-9 < LEAD_RESIDUAL_WINDOW_S * 0.5:
    return False
  rise = float(v_rel) - float(prev_v_rel)
  if (float(v_rel) >= LEAD_CLOSING_REMATCH_BLOCK_MS
      and rise >= LEAD_RESIDUAL_WINDOW_RISE_MS):
    return True
  residual = lead_residual_close_ms2(v_rel, prev_v_rel, a_ego, dt)
  if residual is None or residual < LEAD_RESIDUAL_SUSTAIN_MS2:
    return False
  return (float(v_rel) >= LEAD_APPROACH_DV_MS
          and float(v_rel) > float(prev_v_rel))


def update_residual_sustain(count, hit, need_n=LEAD_RESIDUAL_SUSTAIN_N):
  """Hold a window hit for `need_n` frames. Returns `(armed, count)`."""
  if not hit:
    return False, 0
  nxt = int(count) + 1
  return nxt >= int(need_n), nxt


class LeadResidualWindow:
  """0.5 s v_rel / measured-aEgo history for the #222 residual gate.

  `update` returns `(prev_v_rel, dt, a_ego)` for the existing floor
  helpers. `prev_v_rel` stays None until the window has been a real
  brake for two frames, so a one-LSB radar tick cannot cancel a
  comfort floor. When it is set, `dt` is the window length and `a_ego`
  is the measured average over that window — not one commanded frame.
  """

  def __init__(self) -> None:
    self.hist: deque[tuple[float, float]] = deque()
    self.count = 0

  def reset(self) -> None:
    self.hist.clear()
    self.count = 0

  def update(self, v_rel, a_ego, dt) -> tuple[float | None, float, float]:
    frame_dt = 0.05 if dt is None or float(dt) <= 1e-6 else float(dt)
    a = 0.0 if a_ego is None else float(a_ego)
    if v_rel is None:
      self.reset()
      return None, frame_dt, a
    self.hist.append((float(v_rel), a))
    n = max(1, int(round(LEAD_RESIDUAL_WINDOW_S / frame_dt)))
    while len(self.hist) > n + 1:
      self.hist.popleft()
    if len(self.hist) <= n:
      self.count = 0
      return None, frame_dt, a
    old_v = float(self.hist[0][0])
    window = list(self.hist)
    a_avg = sum(sample_a for _v, sample_a in window) / float(len(window))
    dt_w = n * frame_dt
    hit = lead_residual_window_hit(float(v_rel), old_v, a_avg, dt_w)
    armed, self.count = update_residual_sustain(self.count, hit)
    if not armed:
      return None, frame_dt, a
    return old_v, dt_w, a_avg


def lead_weak_alead_line(v_rel, prev_v_rel=None) -> float:
  """aLead line that may unlock raw near the gap.

  −0.35 while closing stays under 1 m/s and is not rising. At or above
  1 m/s, or when closing is rising, keep the −0.2 line so a real close
  still owns the match.
  """
  v = 0.0 if v_rel is None else float(v_rel)
  if v < LEAD_CLOSING_REMATCH_BLOCK_MS and not lead_close_is_rising(v_rel, prev_v_rel):
    return LEAD_CRUISE_CLIFF_ALEAD_MS2
  return LEAD_CLOSING_ALEAD_MS2


def lead_kinematic_approach_a(v_rel, slack, a_lead=None):
  """Decel when mild cannot finish inside the planned approach distance.

  Non-braking close ≥ 1 m/s only. The planned ease is sized for
  LEAD_APPROACH_A_MS2: a gap at that slack stays mild, even though
  0.22 cannot stop there. Deepen only once slack is clearly inside
  that distance and the required decel is past mild by a clear
  margin. Capped at −0.45 while slack is still above 3 m, and at −0.6
  once inside that, so this never becomes a regen rail. A firm
  braking lead is #222, not this floor.
  """
  if lead_firm_alead(a_lead):
    return None
  if v_rel is None or slack is None:
    return None
  v = float(v_rel)
  if v < LEAD_CLOSING_REMATCH_BLOCK_MS or LEAD_APPROACH_A_MS2 <= 0.0:
    return None
  design = (v * v) / (2.0 * LEAD_APPROACH_A_MS2)
  if float(slack) >= design - LEAD_KIN_APPROACH_DESIGN_MARGIN_M:
    return None
  s = max(float(slack), LEAD_KIN_APPROACH_SLACK_FLOOR_M)
  pure = -(v * v) / (2.0 * s)
  if pure >= -LEAD_APPROACH_MILD_A_MS2 - LEAD_KIN_APPROACH_PAST_MILD_MS2:
    return None
  cap = LEAD_KIN_APPROACH_CAP_MS2
  if float(slack) > LEAD_KIN_APPROACH_SLACK_FLOOR_M:
    cap = LEAD_KIN_APPROACH_OPEN_CAP_MS2
  return max(pure - LEAD_KIN_APPROACH_BIAS_MS2, -cap)


def lead_firm_large_slack_a(v_rel, slack, a_lead):
  """Earlier kinematic brake for a firm lead that is still 20–50 m out.

  None unless aLead is below −0.35, closing ≥ 1.5, and slack is in that
  band. The result is the kinematic need capped at about −1.0. It does
  not apply inside 20 m, where #222 keeps full authority.
  """
  if not lead_firm_alead(a_lead):
    return None
  if v_rel is None or float(v_rel) < LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS:
    return None
  if slack is None:
    return None
  s = float(slack)
  if s <= LEAD_ALEAD_MATCH_SLACK_M or s > LEAD_MID_GAP_SLACK_M:
    return None
  kin = float(a_lead) - (float(v_rel) ** 2) / (2.0 * max(s, 1.0))
  return max(kin, -LEAD_FIRM_EARLY_CAP_MS2)


def lead_firm_alead(a_lead, firm_ms2=LEAD_CRUISE_CLIFF_ALEAD_MS2) -> bool:
  """True when measured aLead is below the #237 firm-brake line.

  Equal to −0.35 is not firm. 22:14:14 (aLead −0.6…−0.7) is.
  """
  return a_lead is not None and float(a_lead) < float(firm_ms2)


def lead_rising_or_residual_brake(v_rel, prev_v_rel=None, a_ego=None, dt=None) -> bool:
  """#222: closing rate rose, or residual close beyond ego's own a.

  A steady close (v_rel not rising) is not residual brake. Same tests
  the mild-floor skip uses after the static close gate.
  """
  if (v_rel is not None and float(v_rel) >= LEAD_CLOSING_REMATCH_BLOCK_MS
      and lead_close_is_rising(v_rel, prev_v_rel)):
    return True
  residual = lead_residual_close_ms2(v_rel, prev_v_rel, a_ego, dt)
  if residual is not None and residual >= LEAD_SOFT_LIMIT_RESIDUAL_MS2:
    if (v_rel is not None and prev_v_rel is not None
        and float(v_rel) >= LEAD_APPROACH_DV_MS
        and float(v_rel) > float(prev_v_rel)):
      return True
  return False


def lead_approach_is_rapid(v_rel, ttc=None) -> bool:
  """True when closing rate is high enough for the firm 0.55 path.

  Short TTC at a mild `v_rel` is arriving at Follow Distance, not dumping.
  `ttc` is for callers; it does not promote a 5–10 mph close to 0.55.
  Planner still requires consecutive samples via lead_approach_rapid_gate.
  """
  _ = ttc
  return float(v_rel) >= LEAD_APPROACH_RAPID_DV_MS
