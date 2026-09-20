#!/usr/bin/env python3
import math
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalMpc,
  LongitudinalPlanSource,
  get_safe_obstacle_distance,
  get_stopped_equivalence_factor,
  get_T_FOLLOW,
)
from openpilot.selfdrive.controls.lib.lead_approach import (
  LEAD_APPROACH_MILD_A_MS2,
  apply_lead_approach_overlay,
  apply_lead_glide_a,
  cap_closing_lead_accel,
  lead_approach_decel_ms2,
  lead_approach_rapid_gate,
  lead_approach_track_ok,
  lead_close_accel_ms2,
  lead_close_should_cap,
  lead_mid_gap_catchup_latch,
  lead_follow_slack_m,
  lead_owns_plan,
  lead_remaining_close_a_ms2,
  resolve_lead_close_hold,
  slew_follow_chatter_a,
  slew_lead_acquire_a,
  slew_lead_approach_a,
  slew_near_gap_small_a,
  soft_limit_mpc_a_target,
  update_lead_acquire,
  update_lead_glide,
  update_lead_settle,
)
from openpilot.selfdrive.controls.lib.follow_distance import FollowDistanceBlend, NAP_FOLLOW_DISTANCE_RANGE
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.selfdrive.mapd.constants import MODE_CAP, MODE_FOLLOW, map_accel_a_ms2, map_brake_a_ms2
from openpilot.selfdrive.mapd.map_speed_policy import (
  cap_planner_v_cruise_ms, map_climb_replaces_mpc, map_in_track_deadband, map_track_accel_ms2,
  map_track_decel_ms2, read_map_speed_params,
)
from openpilot.selfdrive.controls.lib.hill_climb import (
  apply_hill_climb, read_hypermile_hill_climb,
)
from openpilot.selfdrive.controls.lib.hypermile import (
  effective_nap_follow_dist, read_hypermile_params,
)
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]

# Pre-AP follow-mode accel cap: imported lazily to avoid circular deps
_preap_follow_cache = None
def _get_preap_follow_limit(v_ego):
  global _preap_follow_cache
  if _preap_follow_cache is None:
    try:
      from opendbc.car.tesla.preap.constants import ACCEL_PREAP_BP, ACCEL_PREAP_FOLLOW
      _preap_follow_cache = (ACCEL_PREAP_BP, ACCEL_PREAP_FOLLOW)
    except ImportError:
      _preap_follow_cache = (None, None)
  bp, v = _preap_follow_cache
  if bp is None:
    return None
  return float(np.interp(v_ego, bp, v))


def get_preap_follow_cap_strength(v_ego, lead_distance, lead_speed, t_follow):
  lead_obstacle_distance = lead_distance + get_stopped_equivalence_factor(max(lead_speed, 0.0))
  safe_obstacle_distance = get_safe_obstacle_distance(v_ego, t_follow)
  equivalent_ratio = lead_obstacle_distance / max(safe_obstacle_distance, 1.0)
  return float(np.clip(1.0 - (equivalent_ratio - 1.2) / 0.3, 0.0, 1.0))


CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns.

  Pre-AP HUD MAX snapshot/restore through a bend lives in card.py
  (CurveMaxHold). This clip is temporary +a only — it must not rebase
  vCruise / sticky MAX.
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL, params=None):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self._is_preap = (CP.brand == "tesla" and CP.carFingerprint == "TESLA_MODEL_S_PREAP"
                       and CP.openpilotLongitudinalControl and not CP.pcmCruise)
    self._params = Params() if params is None else params
    self.nap_follow_dist = self._params.get("NAPFollowDistance", return_default=True) if self._is_preap else None
    self.nap_adaptive_accel = self._params.get_bool("NAPAdaptiveAccel") if self._is_preap else False
    self._hypermile_on = read_hypermile_params(self._params) if self._is_preap else False
    self._hypermile_hill_climb = read_hypermile_hill_climb(self._params) if self._is_preap else False
    self._hill_pitch = 0.0
    self._map_speed_mode, self._map_speed_offset_kph, self._map_speed_lookahead, self._map_speed_accel = (
      read_map_speed_params(self._params) if self._is_preap else (0, 0.0, 0, 5)
    )
    self._follow_blend = FollowDistanceBlend()
    if self._is_preap:
      self._follow_blend.read_setpoints(self._params)
    self.active_nap_follow_dist = effective_nap_follow_dist(self._is_preap, self.nap_follow_dist)
    self.t_follow = get_T_FOLLOW(nap_follow_dist=self.active_nap_follow_dist)
    self._frame = 0
    self._lead_approach_active = False
    self._lead_approach_a = None
    self._lead_approach_rapid_count = 0
    self._lead_close_hold_d = None
    self._lead_close_hold_v = None
    self._lead_close_hold_a = None
    self._lead_close_hold_age = 0.0
    self._lead_close_hold_owned = False
    self._lead_close_a_cap = None
    self._lead_mid_gap_catchup = False
    self._lead_mid_gap_slack = None
    self._follow_open_a = None
    self._lead_settle_age = 0.0
    self._lead_settled = False
    self._lead_acquire_age = 0.0
    self._lead_glide_active = False
    self._lead_soft_limit_floored = False
    self._lead_soft_limit_v_rel = None

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm):
    self._frame += 1
    if self._is_preap:
      # Stalk Follow Distance 1–7 must land on the next plan.
      self.nap_follow_dist = self._params.get("NAPFollowDistance", return_default=True)
      self._hypermile_on = read_hypermile_params(self._params)
      self._hypermile_hill_climb = read_hypermile_hill_climb(self._params)
      self.nap_adaptive_accel = self._params.get_bool("NAPAdaptiveAccel")
      self._map_speed_mode, self._map_speed_offset_kph, self._map_speed_lookahead, self._map_speed_accel = (
        read_map_speed_params(self._params)
      )
      self._follow_blend.read_setpoints(self._params)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    # HUD MAX after card's map overlay (seed / sticky / Follow override).
    v_hud_ms = v_cruise
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]
    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
    accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])
      self._lead_approach_active = False
      self._lead_approach_a = None
      self._lead_approach_rapid_count = 0
      self._lead_close_hold_d = None
      self._lead_close_hold_v = None
      self._lead_close_hold_a = None
      self._lead_close_hold_age = 0.0
      self._lead_close_hold_owned = False
      self._lead_close_a_cap = None
      self._lead_mid_gap_catchup = False
      self._lead_mid_gap_slack = None
      self._follow_open_a = None
      self._lead_settle_age = 0.0
      self._lead_settled = False
      self._lead_acquire_age = 0.0
      self._lead_soft_limit_v_rel = None

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    _, _, _, _, throttle_prob = self.parse_model(sm['modelV2'])
    # Don't clip at low speeds since throttle_prob doesn't account for creep
    # VDAS takes acceleration targets literally, so the model's negative coast
    # ceiling commands regen even when the MPC is trying to hold cruise speed.
    self.allow_throttle = self._is_preap or throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0

    # OSM map speed: trust card HUD MAX (eased decreases, lag-corrected raises).
    # Do not min() with posted — that snapped when GPS entered a lower zone.
    # Lead still wins via mpc.update(radarState, v_cruise).
    if (not force_slow_decel) and self._is_preap and self._map_speed_mode in (MODE_CAP, MODE_FOLLOW):
      v_cruise = cap_planner_v_cruise_ms(v_hud_ms, None, mode=self._map_speed_mode)

    self.active_nap_follow_dist = effective_nap_follow_dist(self._is_preap, self.nap_follow_dist)
    self.t_follow = get_T_FOLLOW(sm['selfdriveState'].personality, self.active_nap_follow_dist)
    self._follow_open_a = None
    long_engaged = self._is_preap and (not reset_state)
    has_lead_status = bool(sm['radarState'].leadOne.status)
    if self._is_preap:
      lead_tf = sm['radarState'].leadOne
      t_blended, active_dist, self._follow_open_a = self._follow_blend.update(
        v_ego, self.dt,
        engaged=long_engaged,
        has_lead=has_lead_status,
        v_lead=float(lead_tf.vLead) if has_lead_status else None,
        v_cruise=v_hud_ms,
      )
      if active_dist in NAP_FOLLOW_DISTANCE_RANGE:
        self.active_nap_follow_dist = active_dist
        self.nap_follow_dist = active_dist
        self.t_follow = float(t_blended)

    # Pre-AP adaptive accel: only limit accel when the lead's obstacle-equivalent
    # distance is close. Above 1.5x, Adaptive Accel used the full cruise profile
    # to close the gap — that punch. The lead-close cap below owns large-gap
    # +a (Mannerisms Accel, never higher). Below 1.2x, cap to follow limits
    # to prevent overshoot → regen → overshoot oscillation. Blend in between.
    if self.CP.carFingerprint == "TESLA_MODEL_S_PREAP" and self.nap_adaptive_accel and sm['radarState'].leadOne.status:
      follow_limit = _get_preap_follow_limit(v_ego)
      if follow_limit is not None:
        lead = sm['radarState'].leadOne
        cap_strength = get_preap_follow_cap_strength(v_ego, lead.dRel, lead.vLead, self.t_follow)
        if cap_strength > 0:
          blended = accel_clip[1] * (1.0 - cap_strength) + follow_limit * cap_strength
          accel_clip[1] = min(accel_clip[1], blended)

    # Coming up behind a radar lead: cap +a to the same Accel 1–10
    # envelope as open-road / MAX climb (including last-mph baby-step).
    # Cruise 1.6 / Adaptive full-profile used to punch a 160–200 m lead.
    # Near-gap rematch trickles. After match-settle, Accel-ceil rematch
    # is deadbanded (trickle / hunt, not +0.323 for 7 s while opening).
    # At/above MAX the envelope is 0 — do not chase a faster lead past
    # set. Hold last in-window lead on a brief status drop. Does not
    # change MPC danger / −a.
    self._lead_close_a_cap = None
    if self._is_preap:
      lead_close = sm['radarState'].leadOne
      d_cap, v_cap, self._lead_close_hold_d, self._lead_close_hold_v, self._lead_close_hold_age = (
        resolve_lead_close_hold(
          lead_close.status, lead_close.dRel, lead_close.vLead,
          self._lead_close_hold_d, self._lead_close_hold_v, self._lead_close_hold_age,
          self.dt, model_prob=lead_close.modelProb, radar=lead_close.radar,
        )
      )
      if d_cap is not None:
        v_rel_lead = v_ego - float(v_cap)
        slack = lead_follow_slack_m(d_cap, v_cap, self.t_follow)
        a_peak = map_accel_a_ms2(self._map_speed_lookahead, self._map_speed_accel)
        a_grad = map_track_accel_ms2(
          v_ego, v_hud_ms, a_peak, accel_level=self._map_speed_accel,
        )
        # At/above MAX, map_track_accel is None (deadband). That must be
        # a 0 ceiling, not full Accel peak — otherwise rematch +a chases
        # a faster lead past set (ea 11:46: Accel-1 at 62 on a 60 MAX).
        a_env = 0.0 if a_grad is None else min(a_peak, float(a_grad))
        if lead_close.status and lead_close_should_cap(
          lead_close.dRel, lead_close.modelProb, lead_close.radar, active=True,
        ):
          self._lead_close_hold_a = float(lead_close.aLeadK)
          self._lead_close_hold_owned = lead_owns_plan(
            v_rel_lead, self._lead_close_hold_a, slack,
          )
        self._lead_settle_age, self._lead_settled = update_lead_settle(
          self._lead_settle_age, self._lead_settled, v_rel_lead, slack, self.dt,
          present=True, closing_hard=lead_owns_plan(
            v_rel_lead, self._lead_close_hold_a, slack,
          ),
        )
        self._lead_mid_gap_catchup = lead_mid_gap_catchup_latch(
          self._lead_mid_gap_catchup, v_rel_lead, slack,
          prev_slack=self._lead_mid_gap_slack,
          settled=self._lead_settled,
        )
        self._lead_mid_gap_slack = slack
        if self._lead_mid_gap_catchup:
          self._lead_settled = False
          self._lead_settle_age = 0.0
        self._lead_close_a_cap = lead_close_accel_ms2(
          self._map_speed_accel, v_rel=v_rel_lead, slack=slack, a_personality=a_env,
          settled=self._lead_settled, v_ego=v_ego, v_cruise=v_hud_ms,
          catchup=self._lead_mid_gap_catchup,
        )
        if self._lead_close_hold_owned or lead_owns_plan(
          v_rel_lead, self._lead_close_hold_a, slack,
        ):
          self._lead_close_a_cap = min(float(self._lead_close_a_cap), 0.0)
        accel_clip[1] = min(accel_clip[1], self._lead_close_a_cap)
      else:
        self._lead_close_hold_a = None
        self._lead_close_hold_owned = False
        self._lead_settle_age = 0.0
        self._lead_settled = False
        self._lead_mid_gap_catchup = False
        self._lead_mid_gap_slack = None
        self._lead_glide_active = False
        self._lead_soft_limit_floored = False
        self._lead_soft_limit_v_rel = None

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['radarState'], v_cruise, t_follow=self.t_follow)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=self.CP.vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    if sm['selfdriveState'].experimentalMode:
      output_a_target = min(output_a_target_e2e, output_a_target_mpc)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
      if output_a_target < output_a_target_mpc:
        self.mpc.source = LongitudinalPlanSource.e2e
    else:
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc

    # Map MAX is a set speed; MPC cruise_obstacle will not track it.
    # Climb at Accel 1–10 until the deadband, then hold so we do not surge
    # past MAX and map_track_decel below it. Brake is locked Accel 5.
    # A valid radar lead owns follow: do not replace ~0 / slight+ MPC with
    # map climb toward MAX (punch + overshoot + sluggish re-match).
    # map_track_decel when above MAX still mins in. Lead-approach min()
    # and MPC stay as-is.
    # Hypermile Hill Climb (IMU pitch only — no maps-elevation lookahead)
    # then raises +a on a real uphill *under* MAX when there is no lead.
    # Crest / downhill ease only at or above MAX (not TRACK_TAPER).
    # Deadband leaves hold 0 — no +g·sin past MAX. It never writes vCruise / MAX.
    has_valid_lead = bool(sm['radarState'].leadOne.status)
    if len(sm['carControl'].orientationNED) == 3:
      hill_pitch = float(sm['carControl'].orientationNED[1])
    else:
      hill_pitch = 0.0
    if self._is_preap and self._map_speed_mode in (MODE_CAP, MODE_FOLLOW):
      if map_in_track_deadband(v_ego, v_hud_ms):
        if float(output_a_target) >= 0.0:
          output_a_target = 0.0
        pre_hill = float(output_a_target)
        output_a_target = apply_hill_climb(
          pitch_rad=hill_pitch,
          prev_pitch_rad=self._hill_pitch,
          v_ego_ms=v_ego,
          v_cruise_ms=v_hud_ms,
          a_cmd=float(output_a_target),
          in_deadband=True,
          hypermile_on=self._hypermile_on,
          hill_climb_on=self._hypermile_hill_climb,
        )
        if has_valid_lead and float(output_a_target) > pre_hill:
          output_a_target = pre_hill
      else:
        a_brake = map_track_decel_ms2(
          v_ego, v_hud_ms, map_brake_a_ms2(self._map_speed_lookahead),
        )
        if a_brake is not None:
          output_a_target = min(float(output_a_target), a_brake)
        else:
          a_up = map_track_accel_ms2(
            v_ego, v_hud_ms, map_accel_a_ms2(self._map_speed_lookahead, self._map_speed_accel),
            accel_level=self._map_speed_accel,
          )
          if map_climb_replaces_mpc(a_up, output_a_target, has_valid_lead):
            # min() alone never created climb (MPC holds ~0). Command Accel 1–10
            # toward MAX only when no radar lead is following.
            output_a_target = a_up
        pre_hill = float(output_a_target)
        output_a_target = apply_hill_climb(
          pitch_rad=hill_pitch,
          prev_pitch_rad=self._hill_pitch,
          v_ego_ms=v_ego,
          v_cruise_ms=v_hud_ms,
          a_cmd=float(output_a_target),
          in_deadband=False,
          hypermile_on=self._hypermile_on,
          hill_climb_on=self._hypermile_hill_climb,
        )
        if has_valid_lead and float(output_a_target) > pre_hill:
          # Do not add +g·sin punch toward MAX while a lead constrains.
          output_a_target = pre_hill
    self._hill_pitch = hill_pitch

    # Slower radar lead: early light ease as soon as radar feedback is
    # reasonable (200 m Bosch ceiling, 24 s head-start, clear-close skips
    # the late-gap need). Far radar-associated tracks are accepted without
    # a modelProb wait; vision-only far flicker is not. Mild closes stay
    # at MILD; rapid / dumping still uses kinematics up to 0.55 after 4
    # agreeing in-window samples (~0.20 s). Hysteresis + slew both ways
    # keep regen from slamming rematch. Map's +110 m is road distance to
    # a sign and must not be used here. Rapid 0.55 needs consecutive high
    # v_rel frames (a single closing-rate blip stays on mild). A far
    # same-speed nibble must not steal cruise +a; a closing lock never
    # rematches +a. Hold last in-window lead through status flicker so
    # cruise cannot reclaim +a. Near the follow gap, a nibble can mesh.
    # MPC −a is floored at MILD as anti-chatter (gap opening /
    # small |v_rel|, far aLead) and for large-slack small adjustments
    # (e4 40 m / 9.5 m/s −2.33). Closing *near the follow gap* or a
    # near-gap braking lead keeps full match-speed −a. First latch
    # slews both ways so a cruise/MPC punch cannot yo-yo regen→accel
    # (e4 09:53:19). After acquire, matched-speed near the gap glides
    # (a≈0) and inside-FD slow close commands the MILD floor.
    # FCW / rapid / near-bumper / a real stop still own danger.
    # Map MAX cannot cancel this.
    if self._is_preap:
      lead = sm['radarState'].leadOne
      allow_rapid = False
      lead_held = self._lead_close_hold_d is not None and self._lead_close_hold_v is not None
      live_ok = bool(lead.status) and lead_close_should_cap(
        lead.dRel, lead.modelProb, lead.radar, active=False,
      )
      if live_ok:
        overlay_v_rel = v_ego - float(lead.vLead)
        overlay_d = float(lead.dRel)
        overlay_v = float(lead.vLead)
        overlay_radar = lead.radar
        overlay_prob = lead.modelProb
      elif lead_held:
        overlay_v_rel = v_ego - float(self._lead_close_hold_v)
        overlay_d = float(self._lead_close_hold_d)
        overlay_v = float(self._lead_close_hold_v)
        overlay_radar = True
        overlay_prob = None
      else:
        overlay_v_rel = None
        overlay_d = None
        overlay_v = None
        overlay_radar = None
        overlay_prob = None
      overlay_slack = lead_follow_slack_m(overlay_d, overlay_v, self.t_follow)
      self._lead_acquire_age, acquiring = update_lead_acquire(
        self._lead_acquire_age, live_ok or lead_held, self.dt,
      )
      keep_overlay = live_ok or (
        lead_held and (self._lead_close_hold_owned or lead_owns_plan(
          overlay_v_rel, self._lead_close_hold_a, overlay_slack,
        ))
      )
      if keep_overlay:
        a_lead = lead_approach_decel_ms2(
          v_ego, overlay_v, overlay_d, self.t_follow, active=self._lead_approach_active or lead_held,
          model_prob=overlay_prob, radar=overlay_radar, allow_rapid=False,
        )
        allow_rapid, self._lead_approach_rapid_count = lead_approach_rapid_gate(
          overlay_v_rel, self._lead_approach_rapid_count, sample_ok=a_lead is not None,
        )
        if allow_rapid:
          a_lead = lead_approach_decel_ms2(
            v_ego, overlay_v, overlay_d, self.t_follow, active=self._lead_approach_active or lead_held,
            model_prob=overlay_prob, radar=overlay_radar, allow_rapid=allow_rapid,
          )
        self._lead_approach_active = a_lead is not None
      else:
        self._lead_approach_active = False
        self._lead_approach_rapid_count = 0
        a_lead = None
      if live_ok:
        lead_v_hold = float(lead.vLead)
        lead_d_hold = float(lead.dRel)
        lead_a_k = float(lead.aLeadK)
      elif lead_held:
        lead_v_hold = self._lead_close_hold_v
        lead_d_hold = self._lead_close_hold_d
        lead_a_k = self._lead_close_hold_a
      else:
        lead_v_hold = None
        lead_d_hold = None
        lead_a_k = None
      # Floor MPC before overlay so a confirmed rapid 0.55 path is not
      # also clamped. Owned / path-synced lead: residual close (beyond
      # ego a) or aLead skips MILD. One-frame v_rel spikes stay at MILD.
      # Large-slack e4 stays floored. Over MAX (past the deadband),
      # map decel still mins in on a same-speed lead. Firm 0.55 /
      # hard dump still waits on the rapid confirm.
      raw_mpc_a = float(output_a_target)
      prev_close_v_rel = self._lead_soft_limit_v_rel
      output_a_target = soft_limit_mpc_a_target(
        output_a_target, v_ego, lead_v_hold, lead_d_hold,
        fcw=self.fcw, crash_cnt=self.mpc.crash_cnt, allow_rapid=allow_rapid,
        a_lead=lead_a_k, slack=overlay_slack,
        prev_floored=self._lead_soft_limit_floored,
        owned=bool(self._lead_close_hold_owned) or (
          (not acquiring) and (live_ok or lead_held)
        ),
        acquiring=acquiring,
        prev_v_rel=prev_close_v_rel,
        a_ego=self.output_a_target,
        dt=self.dt,
        v_cruise=v_hud_ms,
      )
      self._lead_soft_limit_floored = (
        raw_mpc_a < -LEAD_APPROACH_MILD_A_MS2
        and float(output_a_target) > raw_mpc_a + 1e-9
      )
      self._lead_soft_limit_v_rel = overlay_v_rel if (live_ok or lead_held) else None
      a_lead = slew_lead_approach_a(a_lead, self._lead_approach_a)
      self._lead_approach_a = a_lead
      if a_lead is not None:
        output_a_target = apply_lead_approach_overlay(
          output_a_target, a_lead, v_rel=overlay_v_rel, slack=overlay_slack,
        )
      if self._follow_open_a is not None and float(output_a_target) > float(self._follow_open_a):
        output_a_target = float(self._follow_open_a)
      # Hard block rematch / cruise +a while closing on a live or held
      # lead. Prefer match-speed −a; never positive into a shrinking gap.
      # aLead-only match is near-gap only (not a far / opening lead).
      output_a_target = cap_closing_lead_accel(
        output_a_target, overlay_v_rel, a_lead=lead_a_k,
        lead_present=live_ok or lead_held, owned=self._lead_close_hold_owned,
        slack=overlay_slack, d_rel=overlay_d, acquiring=acquiring,
        prev_v_rel=prev_close_v_rel, a_ego=self.output_a_target, dt=self.dt,
      )
      output_a_target = lead_remaining_close_a_ms2(
        output_a_target, overlay_v_rel, overlay_slack,
        v_ego=v_ego, v_cruise=v_hud_ms,
      )
      if live_ok or lead_held:
        output_a_target = slew_lead_acquire_a(
          output_a_target, self.output_a_target, overlay_v_rel,
          d_rel=overlay_d, slack=overlay_slack, acquiring=acquiring,
          allow_rapid=allow_rapid, fcw=self.fcw, crash_cnt=self.mpc.crash_cnt,
          a_lead=lead_a_k, v_ego=v_ego, v_cruise=v_hud_ms,
        )
      # After acquire: matched-speed glide (no felt ±a), slower
      # near-gap small-bite slew, and mid-gap rematch↔~0 chatter
      # slew. First-latch smoothness stays #214.
      self._lead_glide_active = update_lead_glide(
        self._lead_glide_active, overlay_v_rel, overlay_slack,
        d_rel=overlay_d, fcw=self.fcw, crash_cnt=self.mpc.crash_cnt,
        allow_rapid=allow_rapid, acquiring=acquiring, a_lead=lead_a_k,
      )
      if not acquiring:
        output_a_target = apply_lead_glide_a(output_a_target, self._lead_glide_active)
        output_a_target = slew_near_gap_small_a(
          output_a_target, self.output_a_target, overlay_v_rel,
          d_rel=overlay_d, slack=overlay_slack, allow_rapid=allow_rapid,
          fcw=self.fcw, crash_cnt=self.mpc.crash_cnt,
        )
        output_a_target = slew_follow_chatter_a(
          output_a_target, self.output_a_target, overlay_v_rel,
          d_rel=overlay_d, slack=overlay_slack, allow_rapid=allow_rapid,
          fcw=self.fcw, crash_cnt=self.mpc.crash_cnt,
          catchup=self._lead_mid_gap_catchup,
        )

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    # Hard +a ceiling: the 0.05 clip slew must not leak cruise punch for a
    # second, and a status flicker must not restore 1.6.
    if self._lead_close_a_cap is not None:
      accel_clip[1] = min(float(accel_clip[1]), float(self._lead_close_a_cap))
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    # Chevron / HUD lead: live status, or the planner still owns a held
    # in-window lead through a brief flicker.
    live_lead = bool(sm['radarState'].leadOne.status)
    if self._is_preap and live_lead:
      live_lead = lead_approach_track_ok(
        sm['radarState'].leadOne.dRel,
        sm['radarState'].leadOne.modelProb,
        sm['radarState'].leadOne.radar,
        active=False,
      )
    longitudinalPlan.hasLead = live_lead or (
      self._is_preap and self._lead_close_hold_d is not None
    )
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)
    longitudinalPlan.napFollowDistance = self.active_nap_follow_dist or 0
    longitudinalPlan.tFollow = self.t_follow

    pm.send('longitudinalPlan', plan_send)
