import pytest

from openpilot.selfdrive.controls.lib.lead_approach import (
  LEAD_APPROACH_A_MS2,
  LEAD_APPROACH_CLEAR_DV_MS,
  LEAD_APPROACH_DV_MS,
  LEAD_APPROACH_DV_OFF_MS,
  LEAD_APPROACH_HEADSTART_S,
  LEAD_APPROACH_MAX_HOLD_M,
  LEAD_APPROACH_MAX_START_M,
  LEAD_APPROACH_MILD_A_MS2,
  LEAD_APPROACH_MODEL_PROB_MIN,
  LEAD_APPROACH_NIBBLE_MS2,
  LEAD_APPROACH_NEED_HOLD_M,
  LEAD_APPROACH_RAPID_CONFIRM_N,
  LEAD_APPROACH_RAPID_DV_MS,
  LEAD_APPROACH_RAPID_TTC_S,
  LEAD_APPROACH_SOFT_LIMIT_ALEAD_MS2,
  LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS,
  LEAD_CLOSING_ALEAD_MS2,
  LEAD_CLOSING_MATCH_GAIN,
  LEAD_CLOSING_MATCH_MS,
  LEAD_CLOSING_REMATCH_BLOCK_MS,
  LEAD_ALEAD_MATCH_SLACK_M,
  LEAD_APPROACH_RELEASE_SLEW_MS2,
  LEAD_APPROACH_RELIABLE_M,
  LEAD_APPROACH_SLACK_OFF_M,
  LEAD_APPROACH_SLACK_ON_M,
  LEAD_APPROACH_SLEW_MS2,
  LEAD_APPROACH_TTC_START_S,
  LEAD_CLOSE_A_BASE_MS2,
  LEAD_CLOSE_A_MAX_MS2,
  LEAD_CLOSE_A_MIN_MS2,
  LEAD_CLOSE_HOLD_S,
  LEAD_CLOSE_MAX_M,
  LEAD_CLOSE_OPENING_A_MS2,
  LEAD_CLOSE_REMATCH_A_MS2,
  LEAD_CLOSE_REMATCH_SLACK_M,
  LEAD_SETTLE_FINISH_SLACK_M,
  LEAD_SETTLE_HOLD_S,
  LEAD_SETTLE_SLACK_M,
  LEAD_SETTLE_VREL_MS,
  LEAD_HUNT_A_FRAC,
  LEAD_HUNT_SLACK_M,
  LEAD_HUNT_SLACK_SPAN_M,
  LEAD_REMATCH_PULL_SLACK_M,
  LEAD_ACQUIRE_HOLD_S,
  LEAD_ACQUIRE_SLEW_MS2,
  LEAD_ATARGET_SLEW_MS2,
  LEAD_MPC_SOFT_NEAR_M,
  LEAD_SETTLE_GAP_BIAS_M,
  LEAD_SLOW_CLOSE_MS,
  LEAD_GLIDE_VREL_MS,
  LEAD_GLIDE_VREL_OFF_MS,
  LEAD_GLIDE_SLACK_M,
  LEAD_GLIDE_SLACK_OFF_M,
  LEAD_GLIDE_A_MS2,
  LEAD_GLIDE_CHATTER_LO_MS2,
  LEAD_GLIDE_CHATTER_HI_MS2,
  LEAD_NEAR_GAP_SLACK_M,
  LEAD_NEAR_GAP_SLEW_MS2,
  NAP_T_FOLLOW,
  STOP_DISTANCE,
  apply_lead_approach_overlay,
  apply_lead_glide_a,
  cap_closing_lead_accel,
  lead_approach_decel_ms2,
  lead_alead_owns_match,
  lead_inside_slow_close_a_ms2,
  lead_is_closing,
  lead_is_glide_sample,
  lead_kinematic_slack_m,
  lead_mpc_needs_full_authority,
  lead_owns_plan,
  lead_approach_is_rapid,
  lead_approach_need_m,
  lead_approach_rapid_gate,
  lead_approach_track_ok,
  lead_approach_ttc_s,
  lead_at_or_above_max,
  lead_close_accel_ms2,
  lead_close_should_cap,
  lead_follow_slack_m,
  lead_hunt_accel_ms2,
  lead_is_settled_sample,
  lead_remaining_close_a_ms2,
  nap_t_follow,
  resolve_lead_close_hold,
  slew_follow_plus_a,
  slew_lead_acquire_a,
  slew_lead_approach_a,
  slew_near_gap_small_a,
  soft_limit_mpc_a_target,
  update_lead_acquire,
  update_lead_glide,
  update_lead_settle,
)
from openpilot.selfdrive.mapd.constants import (
  DECREASE_START_MARGIN_M,
  LOOKAHEAD_EARLY,
  LOOKAHEAD_NORMAL,
  TRACK_DEADBAND_MS,
  map_brake_a_ms2,
)


def test_nap_t_follow_matches_follow_distance_slider():
  assert nap_t_follow(4) == 1.3
  assert nap_t_follow(1) == 0.7
  assert nap_t_follow(7) == 1.9
  assert nap_t_follow(None) is None
  assert nap_t_follow(0) is None
  assert list(NAP_T_FOLLOW) == [0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9]


def test_lead_approach_keeps_early_map_brake_not_map_110m_margin():
  """0.55 overlay like map Early. Map's +110 m is a sign drop, not a lead."""
  assert abs(LEAD_APPROACH_A_MS2 - 0.55) < 1e-9
  assert abs(LEAD_APPROACH_A_MS2 - map_brake_a_ms2(LOOKAHEAD_EARLY)) < 1e-9
  assert LEAD_APPROACH_A_MS2 < map_brake_a_ms2(LOOKAHEAD_NORMAL)
  assert abs(DECREASE_START_MARGIN_M - 110.0) < 1e-9
  assert abs(LEAD_APPROACH_HEADSTART_S - 24.0) < 1e-9
  assert abs(LEAD_APPROACH_MAX_START_M - 200.0) < 1e-9
  assert abs(LEAD_APPROACH_RELIABLE_M - 140.0) < 1e-9
  assert abs(LEAD_APPROACH_CLEAR_DV_MS - 1.5) < 1e-9
  assert abs(LEAD_APPROACH_MODEL_PROB_MIN - 0.50) < 1e-9
  assert LEAD_APPROACH_RELIABLE_M < LEAD_APPROACH_MAX_START_M
  assert LEAD_APPROACH_CLEAR_DV_MS > LEAD_APPROACH_DV_MS
  assert LEAD_APPROACH_CLEAR_DV_MS >= LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS
  assert LEAD_APPROACH_A_MS2 < 0.80
  assert LEAD_APPROACH_A_MS2 < 1.0
  assert LEAD_APPROACH_A_MS2 < 2.5
  # Tiny comfort tune: raise enter only. Exit stays 0.20 so we still close.
  assert abs(LEAD_APPROACH_DV_MS - 0.55) < 1e-9
  assert abs(LEAD_APPROACH_DV_OFF_MS - 0.20) < 1e-9
  assert LEAD_APPROACH_DV_OFF_MS < LEAD_APPROACH_DV_MS
  assert LEAD_APPROACH_DV_MS > 0.50  # harder rematch re-enter than #122
  assert LEAD_APPROACH_SLACK_OFF_M < LEAD_APPROACH_SLACK_ON_M
  assert LEAD_APPROACH_NEED_HOLD_M > 0.0
  assert LEAD_APPROACH_MAX_HOLD_M > 0.0
  assert abs(LEAD_APPROACH_SLEW_MS2 - 0.05) < 1e-9
  assert abs(LEAD_APPROACH_RELEASE_SLEW_MS2 - 0.025) < 1e-9
  assert LEAD_APPROACH_RELEASE_SLEW_MS2 < LEAD_APPROACH_SLEW_MS2
  assert abs(LEAD_APPROACH_NIBBLE_MS2 - 0.15) < 1e-9
  assert LEAD_APPROACH_NIBBLE_MS2 > 0.13  # covers matching-traffic |a|
  assert abs(LEAD_APPROACH_A_MS2 - 0.55) < 1e-9  # rapid peak unchanged
  assert abs(LEAD_APPROACH_MILD_A_MS2 - 0.22) < 1e-9
  assert LEAD_APPROACH_MILD_A_MS2 < LEAD_APPROACH_A_MS2
  assert LEAD_APPROACH_MILD_A_MS2 > LEAD_APPROACH_NIBBLE_MS2
  assert abs(LEAD_APPROACH_RAPID_DV_MS - 6.0) < 1e-9
  assert abs(LEAD_APPROACH_TTC_START_S - 20.0) < 1e-9
  assert abs(LEAD_APPROACH_RAPID_TTC_S - 8.0) < 1e-9
  assert LEAD_APPROACH_RAPID_DV_MS > LEAD_APPROACH_CLEAR_DV_MS
  assert LEAD_APPROACH_RAPID_CONFIRM_N >= 4
  # Floor skip is below rapid, above rematch-enter jitter.
  assert abs(LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS - 1.5) < 1e-9
  assert LEAD_APPROACH_DV_MS < LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS < LEAD_APPROACH_RAPID_DV_MS
  assert abs(LEAD_APPROACH_SOFT_LIMIT_ALEAD_MS2 + 0.2) < 1e-9
  assert LEAD_APPROACH_SOFT_LIMIT_ALEAD_MS2 < 0.0
  assert abs(LEAD_CLOSING_REMATCH_BLOCK_MS - 1.0) < 1e-9
  assert abs(LEAD_CLOSING_MATCH_MS - 1.5) < 1e-9
  assert abs(LEAD_CLOSING_ALEAD_MS2 + 0.2) < 1e-9
  assert abs(LEAD_CLOSING_MATCH_GAIN - 0.25) < 1e-9
  assert LEAD_CLOSING_REMATCH_BLOCK_MS < LEAD_CLOSING_MATCH_MS
  assert abs(LEAD_ALEAD_MATCH_SLACK_M - 20.0) < 1e-9
  assert 15.0 <= LEAD_ALEAD_MATCH_SLACK_M <= 25.0
  assert LEAD_HUNT_SLACK_M <= LEAD_ALEAD_MATCH_SLACK_M <= LEAD_SETTLE_SLACK_M
  assert abs(LEAD_CLOSE_OPENING_A_MS2 - 0.08) < 1e-9
  assert abs(LEAD_CLOSE_REMATCH_A_MS2 - 0.12) < 1e-9
  assert LEAD_CLOSE_OPENING_A_MS2 < LEAD_CLOSE_REMATCH_A_MS2
  assert LEAD_CLOSE_REMATCH_SLACK_M > 0.0
  assert abs(LEAD_SETTLE_VREL_MS - 0.5) < 1e-9
  assert abs(LEAD_SETTLE_HOLD_S - 0.75) < 1e-9
  assert 0.5 <= LEAD_SETTLE_HOLD_S <= 1.0
  assert abs(LEAD_SETTLE_SLACK_M - 25.0) < 1e-9
  assert abs(LEAD_SETTLE_FINISH_SLACK_M - 4.0) < 1e-9
  assert 2.0 <= LEAD_SETTLE_FINISH_SLACK_M < LEAD_HUNT_SLACK_M
  assert abs(LEAD_HUNT_SLACK_M - 15.0) < 1e-9
  assert abs(LEAD_REMATCH_PULL_SLACK_M - 20.0) < 1e-9
  assert 18.0 <= LEAD_REMATCH_PULL_SLACK_M <= 25.0
  assert LEAD_HUNT_SLACK_M < LEAD_REMATCH_PULL_SLACK_M < LEAD_SETTLE_SLACK_M
  assert abs(LEAD_HUNT_A_FRAC - 0.35) < 1e-9
  assert LEAD_HUNT_A_FRAC < 1.0
  assert abs(LEAD_HUNT_SLACK_SPAN_M - 25.0) < 1e-9
  assert abs(LEAD_ATARGET_SLEW_MS2 - 0.05) < 1e-9
  assert abs(LEAD_ACQUIRE_HOLD_S - 0.75) < 1e-9
  assert 0.5 <= LEAD_ACQUIRE_HOLD_S <= 1.0
  assert abs(LEAD_ACQUIRE_SLEW_MS2 - LEAD_ATARGET_SLEW_MS2) < 1e-9
  assert abs(LEAD_MPC_SOFT_NEAR_M - 12.0) < 1e-9
  assert STOP_DISTANCE < LEAD_MPC_SOFT_NEAR_M <= LEAD_CLOSE_REMATCH_SLACK_M
  # e8 settle / glide: keep acquire, add gap bias + matched-speed deadband.
  assert abs(LEAD_SETTLE_GAP_BIAS_M - 3.0) < 1e-9
  assert 2.0 <= LEAD_SETTLE_GAP_BIAS_M <= 4.0
  assert abs(LEAD_SLOW_CLOSE_MS - 0.8) < 1e-9
  assert LEAD_GLIDE_VREL_MS < LEAD_SLOW_CLOSE_MS < LEAD_CLOSING_MATCH_MS
  assert LEAD_GLIDE_VREL_MS < LEAD_SETTLE_VREL_MS
  assert abs(LEAD_GLIDE_VREL_OFF_MS - 0.40) < 1e-9
  assert LEAD_GLIDE_VREL_MS < LEAD_GLIDE_VREL_OFF_MS < LEAD_SLOW_CLOSE_MS
  assert abs(LEAD_GLIDE_SLACK_M - LEAD_SETTLE_FINISH_SLACK_M) < 1e-9
  assert abs(LEAD_GLIDE_SLACK_OFF_M - 8.0) < 1e-9
  assert LEAD_GLIDE_SLACK_M < LEAD_GLIDE_SLACK_OFF_M
  assert abs(LEAD_GLIDE_A_MS2 - LEAD_CLOSE_OPENING_A_MS2) < 1e-9
  assert 0.0 < LEAD_GLIDE_A_MS2 <= LEAD_CLOSE_OPENING_A_MS2
  assert LEAD_GLIDE_CHATTER_LO_MS2 < -LEAD_APPROACH_MILD_A_MS2
  assert LEAD_GLIDE_CHATTER_HI_MS2 > LEAD_CLOSE_REMATCH_A_MS2
  assert abs(LEAD_NEAR_GAP_SLACK_M - 15.0) < 1e-9
  assert abs(LEAD_NEAR_GAP_SLEW_MS2 - 0.02) < 1e-9
  assert LEAD_NEAR_GAP_SLEW_MS2 < LEAD_ATARGET_SLEW_MS2
  assert abs(LEAD_CLOSE_HOLD_S - 0.50) < 1e-9
  assert abs(LEAD_CLOSE_MAX_M - LEAD_APPROACH_MAX_START_M) < 1e-9
  import openpilot.selfdrive.controls.lib.lead_approach as lead_approach
  assert not hasattr(lead_approach, "LEAD_APPROACH_MARGIN_M")


def test_lead_approach_eases_before_mpc_comfort_brake_window():
  """Slower lead: relative Early head-start. Mild peak is light regen, not 0.55 / MPC 2.5."""
  v_ego = 60.0 * 0.44704
  v_lead = 50.0 * 0.44704
  v_rel = v_ego - v_lead
  t4 = nap_t_follow(4)
  t7 = nap_t_follow(7)
  d_follow = t4 * v_lead + STOP_DISTANCE
  need = lead_approach_need_m(v_ego, v_lead, t_follow=t4)
  rel_need = (v_rel * v_rel) / (2.0 * LEAD_APPROACH_A_MS2)
  mpc_need = (v_rel * v_rel) / (2.0 * 2.5)
  map_road_need = (v_ego * v_ego - v_lead * v_lead) / (2.0 * LEAD_APPROACH_A_MS2) + 110.0
  assert abs(need - (rel_need + v_rel * LEAD_APPROACH_HEADSTART_S)) < 1e-6
  assert need > mpc_need
  # Map's road-distance +110 m is much farther (radar-edge hang). Do not use it.
  assert need < map_road_need - 50.0
  assert d_follow + need <= LEAD_APPROACH_MAX_START_M + 1e-6
  assert d_follow + need > 140.0  # old 140 m / 12 s window
  at_open = lead_approach_decel_ms2(v_ego, v_lead, d_follow + need - 1.0, t4)
  assert at_open is not None
  # Longer head-start: open is lighter than the old 12 s ~0.15.
  assert -0.12 < at_open < -0.05
  mid = lead_approach_decel_ms2(v_ego, v_lead, d_follow + 0.45 * need, t4)
  assert mid is not None and -LEAD_APPROACH_A_MS2 <= mid < 0.0
  assert at_open > mid  # more slack → gentler a (both negative)
  peak = lead_approach_decel_ms2(v_ego, v_lead, d_follow + rel_need, t4)
  assert peak is not None
  # 10 mph is not dumping: light ceiling, not the 0.55 bite.
  assert abs(peak + LEAD_APPROACH_MILD_A_MS2) < 0.05
  assert abs(peak) < LEAD_APPROACH_A_MS2 - 0.20
  assert lead_approach_decel_ms2(v_ego, v_ego, 80.0, t4) is None
  assert lead_approach_decel_ms2(v_ego, v_ego + 2.0, 80.0, t4) is None
  # Follow 7 vs 4: marginal close still uses need (v_rel below CLEAR_DV).
  v_slow = v_lead + LEAD_APPROACH_DV_MS + 0.05
  assert v_slow - v_lead < LEAD_APPROACH_CLEAR_DV_MS
  d_open_4 = t4 * v_lead + STOP_DISTANCE + lead_approach_need_m(v_slow, v_lead, t_follow=t4)
  d_open_7 = t7 * v_lead + STOP_DISTANCE + lead_approach_need_m(v_slow, v_lead, t_follow=t7)
  assert d_open_7 > d_open_4 + 5.0
  assert lead_approach_decel_ms2(v_slow, v_lead, d_open_4 + 3.0, t7) is not None
  assert lead_approach_decel_ms2(v_slow, v_lead, d_open_4 + 3.0, t4) is None


def test_lead_approach_starts_much_earlier_with_lighter_open():
  """24 s vs 12 s head-start: earlier and lighter, still closes onto Follow Distance."""
  v_ego = 60.0 * 0.44704
  v_lead = 50.0 * 0.44704
  v_rel = v_ego - v_lead
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  rel_need = (v_rel * v_rel) / (2.0 * LEAD_APPROACH_A_MS2)
  need_12 = rel_need + v_rel * 12.0
  need_24 = lead_approach_need_m(v_ego, v_lead, t_follow=t4)
  assert abs(need_24 - (rel_need + v_rel * 24.0)) < 1e-6
  extra_m = need_24 - need_12
  assert 50.0 < extra_m < 58.0  # ~54 m / ~12 s of 10 mph closing
  d_open_12 = d_follow + need_12
  d_open_24 = d_follow + need_24
  assert d_open_24 > d_open_12
  assert d_open_24 > 140.0
  assert d_open_24 <= LEAD_APPROACH_MAX_START_M + 1e-6
  assert lead_approach_decel_ms2(v_ego, v_lead, d_open_12 + 3.0, t4) is not None
  a_old_open = -(v_rel * v_rel) / (2.0 * need_12)
  a_new_open = lead_approach_decel_ms2(v_ego, v_lead, d_open_24 - 1.0, t4)
  assert a_new_open is not None
  assert a_old_open < a_new_open < 0.0  # new open is lighter (less negative)
  assert abs(a_new_open) < 0.12
  assert abs(a_old_open) > abs(a_new_open) + 0.03
  peak = lead_approach_decel_ms2(v_ego, v_lead, d_follow + rel_need, t4)
  assert peak is not None
  assert abs(peak + LEAD_APPROACH_MILD_A_MS2) < 0.05


def test_far_closing_lead_enters_where_old_140m_would_not():
  """60→50 at 150–180 m now eases. Old 140 m / short need stayed off."""
  v_ego = 60.0 * 0.44704
  v_lead = 50.0 * 0.44704
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  old_need = (v_ego - v_lead) ** 2 / (2.0 * LEAD_APPROACH_A_MS2) + (v_ego - v_lead) * 12.0
  assert d_follow + old_need < 140.0
  for d_rel in (150.0, 160.0, 180.0):
    assert d_rel > 140.0
    a = lead_approach_decel_ms2(
      v_ego, v_lead, d_rel, t4, model_prob=1.0, radar=True,
    )
    assert a is not None and -LEAD_APPROACH_A_MS2 <= a < 0.0
    assert abs(a) < 0.20  # far slack → gentle
  assert lead_approach_decel_ms2(
    v_ego, v_lead, 210.0, t4, model_prob=1.0, radar=True,
  ) is None


def test_lead_approach_closes_onto_follow_distance_not_hang_at_radar():
  """Integrate relative ease: arrive near Follow Distance instead of matching at ~150 m."""
  v_ego = 60.0 * 0.44704
  v_lead = 50.0 * 0.44704
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  d_rel = 180.0
  dt = 0.05
  min_d_rel = d_rel
  active = False
  for _ in range(int(80.0 / dt)):
    a = lead_approach_decel_ms2(
      v_ego, v_lead, d_rel, t4, active=active, model_prob=1.0, radar=True,
    )
    active = a is not None
    if a is None:
      a = 0.0
    v_ego = max(0.0, v_ego + a * dt)
    d_rel -= (v_ego - v_lead) * dt
    min_d_rel = min(min_d_rel, d_rel)
    if d_rel <= d_follow + 2.0:
      break
  assert min_d_rel <= d_follow + 8.0
  assert d_rel <= d_follow + 8.0
  assert v_ego <= v_lead + 1.5
  # Map-style road formula would have matched near 150 m; we must not.
  assert d_rel < 80.0


def test_t_follow_table_stays_in_sync_with_mpc():
  from pathlib import Path
  mpc = (Path(__file__).resolve().parents[1] / "lib/longitudinal_mpc_lib/long_mpc.py").read_text()
  assert "NAP_T_FOLLOW = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)" in mpc
  assert "STOP_DISTANCE = 6.0" in mpc


def test_stopped_lead_still_plans_a_comfortable_stop_gap():
  v_ego = 20.0
  t4 = nap_t_follow(4)
  need = lead_approach_need_m(v_ego, 0.0, t_follow=t4)
  d_follow = STOP_DISTANCE
  a = lead_approach_decel_ms2(v_ego, 0.0, d_follow + 0.5 * need, t4)
  assert a is not None and a < 0.0
  assert abs(a) <= LEAD_APPROACH_A_MS2 + 1e-9
  assert lead_approach_decel_ms2(v_ego, 0.0, d_follow + need + 20.0, t4) is None


def test_lead_close_accel_matches_mannerisms_not_cruise_punch():
  """Accel 1–10 owns catch-up +a. Same envelope as MAX climb, never a 1.6 punch."""
  from openpilot.selfdrive.mapd.constants import LOOKAHEAD_NORMAL, map_accel_a_ms2
  a1 = lead_close_accel_ms2(1)
  a5 = lead_close_accel_ms2(5)
  a10 = lead_close_accel_ms2(10)
  assert abs(a1 - LEAD_CLOSE_A_MIN_MS2) < 1e-9
  assert abs(a5 - LEAD_CLOSE_A_BASE_MS2) < 1e-9
  assert abs(a10 - LEAD_CLOSE_A_MAX_MS2) < 1e-9
  assert a1 == pytest.approx(map_accel_a_ms2(LOOKAHEAD_NORMAL, 1))
  assert a10 == pytest.approx(map_accel_a_ms2(LOOKAHEAD_NORMAL, 10))
  assert a1 < a5 < a10
  assert a1 < 1.6
  assert a10 <= 1.60 + 1e-9
  assert lead_close_should_cap(80.0)
  assert lead_close_should_cap(LEAD_CLOSE_MAX_M)
  assert lead_close_should_cap(160.0, model_prob=1.0, radar=True)
  assert lead_close_should_cap(160.0, model_prob=0.2, radar=True)
  assert not lead_close_should_cap(160.0, model_prob=0.2, radar=False)
  assert not lead_close_should_cap(LEAD_APPROACH_MAX_START_M + 20.0)
  assert not lead_close_should_cap(0.0)
  assert not lead_close_should_cap(None)


def test_lead_close_accel_still_closes_onto_follow_distance():
  """Capped +a still closes a large same-speed gap; no far-back hang."""
  v_ego = 22.0
  v_lead = 22.0
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  d_rel = d_follow + 40.0
  a_cap = lead_close_accel_ms2(1)
  dt = 0.05
  min_d_rel = d_rel
  for _ in range(int(80.0 / dt)):
    a = a_cap
    decel = lead_approach_decel_ms2(v_ego, v_lead, d_rel, t4)
    if decel is not None:
      a = min(a, decel)
    v_ego = max(0.0, v_ego + a * dt)
    d_rel -= (v_ego - v_lead) * dt
    min_d_rel = min(min_d_rel, d_rel)
    if d_rel <= d_follow + 2.0:
      break
  assert a_cap == pytest.approx(LEAD_CLOSE_A_MIN_MS2)
  assert min_d_rel <= d_follow + 8.0
  assert d_rel <= d_follow + 8.0
  assert d_rel < d_follow + 20.0


def test_lead_approach_hysteresis_holds_through_v_rel_and_slack_noise():
  """Slight-grade follow: ±noise around the old 0.5 / slack=1 gates must not chatter."""
  v_lead = 22.0
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  v_ego = v_lead + LEAD_APPROACH_DV_MS + 0.05
  d_rel = d_follow + 3.0
  assert lead_approach_decel_ms2(v_ego, v_lead, d_rel, t4, active=False) is not None

  v_jitter = v_lead + LEAD_APPROACH_DV_MS - 0.08
  assert v_jitter - v_lead > LEAD_APPROACH_DV_OFF_MS
  assert lead_approach_decel_ms2(v_jitter, v_lead, d_rel, t4, active=False) is None
  held = lead_approach_decel_ms2(v_jitter, v_lead, d_rel, t4, active=True)
  assert held is not None and held < 0.0

  d_near = d_follow + 0.4
  assert lead_approach_decel_ms2(v_ego, v_lead, d_near, t4, active=False) is None
  near = lead_approach_decel_ms2(v_ego, v_lead, d_near, t4, active=True)
  assert near is not None and near < 0.0
  assert abs(near) <= LEAD_APPROACH_A_MS2 + 1e-9

  need = lead_approach_need_m(v_ego, v_lead, t_follow=t4)
  just_out = d_follow + need + 1.5
  assert lead_approach_decel_ms2(v_ego, v_lead, just_out, t4, active=False) is None
  assert lead_approach_decel_ms2(v_ego, v_lead, just_out, t4, active=True) is not None
  far_out = d_follow + need + LEAD_APPROACH_NEED_HOLD_M + 1.0
  assert lead_approach_decel_ms2(v_ego, v_lead, far_out, t4, active=True) is None

  assert lead_approach_decel_ms2(v_lead, v_lead, d_rel, t4, active=True) is None
  assert lead_approach_decel_ms2(v_lead - 0.5, v_lead, d_rel, t4, active=True) is None
  assert lead_approach_decel_ms2(v_ego, v_lead, d_follow - 0.5, t4, active=True) is None

  # Exit stays 0.20 so a 0.16 rematch-adjacent close can finish onto the gap.
  v_finish = v_lead + 0.16
  assert v_finish - v_lead < LEAD_APPROACH_DV_OFF_MS
  assert lead_approach_decel_ms2(v_finish, v_lead, d_rel, t4, active=True) is None
  # Rematch at the old 0.50 enter must not re-bite.
  v_old_enter = v_lead + 0.52
  assert 0.50 < 0.52 < LEAD_APPROACH_DV_MS
  assert lead_approach_decel_ms2(v_old_enter, v_lead, d_rel, t4, active=False) is None
  assert lead_approach_decel_ms2(v_lead + LEAD_APPROACH_DV_MS + 0.01, v_lead, d_rel, t4, active=False) is not None
  assert lead_approach_decel_ms2(v_lead + LEAD_APPROACH_DV_OFF_MS - 0.02, v_lead, d_rel, t4, active=True) is None


def test_lead_approach_gap_edge_rematch_does_not_chatter():
  """Slight-grade rematch: rematch after a 0.20 exit used to re-cross 0.50.

  Enter 0.55 blocks that re-bite. Exit stays 0.20 so we still drop and close.
  """
  v_lead = 22.0
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  d_rel = d_follow + 3.0

  v_on = v_lead + LEAD_APPROACH_DV_MS + 0.05
  assert lead_approach_decel_ms2(v_on, v_lead, d_rel, t4, active=False) is not None

  # Hold just above the 0.20 exit; drop at/under it so rematch can finish.
  assert lead_approach_decel_ms2(v_lead + 0.21, v_lead, d_rel, t4, active=True) is not None
  assert lead_approach_decel_ms2(v_lead + 0.19, v_lead, d_rel, t4, active=True) is None
  assert lead_approach_decel_ms2(v_lead + 0.08, v_lead, d_rel, t4, active=True) is None

  # After drop, old enter 0.50–0.54 stays off so rematch does not re-bite.
  for dv in (0.50, 0.52, 0.54):
    assert 0.50 <= dv < LEAD_APPROACH_DV_MS
    assert lead_approach_decel_ms2(v_lead + dv, v_lead, d_rel, t4, active=False) is None

  assert lead_approach_decel_ms2(v_lead + LEAD_APPROACH_DV_MS + 0.01, v_lead, d_rel, t4, active=False) is not None


def test_lead_approach_slew_softens_onset_and_release():
  """Regen onset is gradual; milder / off slews toward 0 so rematch does not slam."""
  target = -LEAD_APPROACH_A_MS2
  a = slew_lead_approach_a(target, None)
  assert a == pytest.approx(-LEAD_APPROACH_SLEW_MS2)
  prev = None
  reached = 0
  for n in range(1, 20):
    prev = slew_lead_approach_a(target, prev)
    assert prev is not None
    assert prev >= target - 1e-9
    reached = n
    if abs(prev - target) < 1e-9:
      break
  else:
    raise AssertionError("slew did not reach comfort peak")
  assert abs(prev + LEAD_APPROACH_A_MS2) < 1e-9
  assert reached == int(round(LEAD_APPROACH_A_MS2 / LEAD_APPROACH_SLEW_MS2))

  assert slew_lead_approach_a(-0.10, -0.40) == pytest.approx(-0.40 + LEAD_APPROACH_RELEASE_SLEW_MS2)
  released = slew_lead_approach_a(None, -0.40)
  assert released == pytest.approx(-0.40 + LEAD_APPROACH_RELEASE_SLEW_MS2)
  assert released is not None and released < 0.0
  fading = -0.40
  frames = 0
  while fading is not None:
    fading = slew_lead_approach_a(None, fading)
    frames += 1
    if frames > 40:
      raise AssertionError("release slew did not reach off")
  assert frames == int(round(0.40 / LEAD_APPROACH_RELEASE_SLEW_MS2))


def test_lead_approach_peak_stays_at_early_comfort_not_mpc():
  """Rapid overlay caps at 0.55. Mild stays at 0.22. Does not own MPC 2.5."""
  t4 = nap_t_follow(4)
  v_lead = 22.4
  d_follow = t4 * v_lead + STOP_DISTANCE

  v_mild = v_lead + 4.4  # ~10 mph — light ceiling
  mild_need = (4.4 * 4.4) / (2.0 * LEAD_APPROACH_A_MS2)
  mild = lead_approach_decel_ms2(v_mild, v_lead, d_follow + mild_need, t4)
  assert mild is not None
  assert abs(mild + LEAD_APPROACH_MILD_A_MS2) < 1e-9
  assert abs(mild) < 0.30
  tight_mild = lead_approach_decel_ms2(v_mild, v_lead, d_follow + 1.05, t4)
  assert tight_mild == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)

  v_ego = v_lead + LEAD_APPROACH_RAPID_DV_MS + 0.5
  v_rel = v_ego - v_lead
  rel_need = (v_rel * v_rel) / (2.0 * LEAD_APPROACH_A_MS2)
  peak = lead_approach_decel_ms2(v_ego, v_lead, d_follow + rel_need, t4, allow_rapid=True)
  assert peak is not None
  assert abs(peak + LEAD_APPROACH_A_MS2) < 1e-9
  assert abs(peak) <= 0.55 + 1e-9
  assert abs(peak) < 0.80
  assert abs(peak) < 2.5
  tight = lead_approach_decel_ms2(v_ego, v_lead, d_follow + 1.05, t4, allow_rapid=True)
  assert tight is not None
  assert tight == pytest.approx(-LEAD_APPROACH_A_MS2)


def test_map_climb_still_does_not_replace_mpc_when_lead_present():
  """#118: valid lead → no map a_up replace. Soft overlay must not reopen that."""
  from openpilot.selfdrive.mapd.constants import map_accel_a_ms2
  from openpilot.selfdrive.mapd.map_speed_policy import map_climb_replaces_mpc, map_track_accel_ms2

  a_up = map_track_accel_ms2(21.5, 24.6, map_accel_a_ms2(LOOKAHEAD_EARLY, 1))
  assert a_up is not None and a_up > 0.05
  assert map_climb_replaces_mpc(a_up, 0.05, has_valid_lead=True) is False
  assert map_climb_replaces_mpc(a_up, 0.0, has_valid_lead=True) is False
  assert map_climb_replaces_mpc(a_up, 0.05, has_valid_lead=False) is True
  assert map_climb_replaces_mpc(a_up, -0.4, has_valid_lead=False) is False


def test_far_flicker_rejected_without_radar_or_model_prob():
  """Beyond 140 m: vision-only stays off. Radar-associated enters without modelProb."""
  v_ego = 60.0 * 0.44704
  v_lead = 50.0 * 0.44704
  t4 = nap_t_follow(4)
  d_far = 180.0
  assert d_far > LEAD_APPROACH_RELIABLE_M
  assert lead_approach_track_ok(80.0, model_prob=0.0, radar=False) is True
  assert lead_approach_track_ok(d_far, model_prob=0.2, radar=True) is True
  assert lead_approach_track_ok(d_far, model_prob=1.0, radar=False) is False
  assert lead_approach_track_ok(d_far, model_prob=1.0, radar=True) is True
  assert lead_approach_track_ok(d_far) is True
  a_radar = lead_approach_decel_ms2(
    v_ego, v_lead, d_far, t4, model_prob=0.2, radar=True,
  )
  assert a_radar is not None and a_radar < 0.0
  assert lead_approach_decel_ms2(
    v_ego, v_lead, d_far, t4, model_prob=1.0, radar=False,
  ) is None
  held = lead_approach_decel_ms2(
    v_ego, v_lead, d_far, t4, active=True, model_prob=0.2, radar=True,
  )
  assert held is not None and held < 0.0
  just_past = LEAD_APPROACH_MAX_START_M + 3.0
  assert lead_approach_decel_ms2(
    v_ego, v_lead, just_past, t4, active=False, model_prob=1.0, radar=True,
  ) is None
  assert lead_approach_decel_ms2(
    v_ego, v_lead, just_past, t4, active=True, model_prob=1.0, radar=True,
  ) is not None
  assert lead_approach_decel_ms2(
    v_ego, v_lead, LEAD_APPROACH_MAX_START_M + LEAD_APPROACH_MAX_HOLD_M + 1.0,
    t4, active=True, model_prob=1.0, radar=True,
  ) is None


def test_accel1_catchup_at_one_ms_is_not_clear_close():
  """v_rel=1.0 / 35 m slack is Accel-1 catch-up. Overlay must stay off."""
  v_ego = 25.0
  v_lead = 24.0
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  assert v_ego - v_lead < LEAD_APPROACH_CLEAR_DV_MS
  assert lead_approach_decel_ms2(v_ego, v_lead, d_follow + 35.0, t4) is None
  assert lead_approach_decel_ms2(v_lead + LEAD_APPROACH_CLEAR_DV_MS + 0.2, v_lead, d_follow + 35.0, t4) is not None


def test_clear_close_allows_large_slack_still_capped():
  """v_rel clearly positive + valid lead: ease even with slack past need."""
  v_ego = 60.0 * 0.44704
  v_lead = 50.0 * 0.44704
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  need = lead_approach_need_m(v_ego, v_lead, t_follow=t4)
  far_slack = d_follow + need + 20.0
  assert far_slack < LEAD_APPROACH_MAX_START_M
  assert v_ego - v_lead > LEAD_APPROACH_CLEAR_DV_MS
  a = lead_approach_decel_ms2(
    v_ego, v_lead, far_slack, t4, model_prob=1.0, radar=True,
  )
  assert a is not None and a < 0.0
  assert abs(a) <= LEAD_APPROACH_A_MS2 + 1e-9
  assert abs(a) < 0.15
  v_slow = v_lead + LEAD_APPROACH_DV_MS + 0.05
  need_slow = lead_approach_need_m(v_slow, v_lead, t_follow=t4)
  assert lead_approach_decel_ms2(
    v_slow, v_lead, d_follow + need_slow + 20.0, t4,
  ) is None


def test_far_mild_close_overlay_does_not_kill_cruise_plus_a():
  """Same-speed far slack may keep cruise +a. Closing ≥ 1.5 must not.

  First-acquire nibble used to keep rematch +a into a shrinking gap (d7
  20:33). Closing locks min() the overlay; same-speed catch-up may climb.
  """
  v_lead = 22.0
  v_rel = 1.6
  v_ego = v_lead + v_rel
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  slack = 80.0
  d_rel = d_follow + slack
  need = lead_approach_need_m(v_ego, v_lead, t_follow=t4)
  assert slack > need
  assert v_rel >= LEAD_APPROACH_CLEAR_DV_MS
  assert v_rel >= LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS
  # Closing first lock past the old need window now eases immediately.
  a_close = lead_approach_decel_ms2(
    v_ego, v_lead, d_rel, t4, active=False, model_prob=1.0, radar=True,
  )
  assert a_close is not None and a_close < 0.0
  cruise = 0.80
  first = apply_lead_approach_overlay(cruise, -0.08, v_rel=v_rel, slack=slack)
  assert first < 0.0
  assert first == pytest.approx(-0.08)
  assert apply_lead_approach_overlay(
    cruise, -0.08, v_rel=v_rel, slack=160.0,
  ) == pytest.approx(-0.08)
  assert apply_lead_approach_overlay(
    cruise, -0.08, v_rel=4.0, slack=120.0,
  ) == pytest.approx(-0.08)
  # Same-speed / Accel catch-up (closing 1.0–1.5) far slack keeps +a.
  assert apply_lead_approach_overlay(
    cruise, -0.08, v_rel=0.3, slack=80.0,
  ) == pytest.approx(cruise)
  assert apply_lead_approach_overlay(
    cruise, -0.08, v_rel=1.2, slack=80.0,
  ) == pytest.approx(cruise)
  # Vision flicker must not early-start past need (even above CLEAR_DV).
  v_clear = v_lead + LEAD_APPROACH_CLEAR_DV_MS + 0.2
  far_slack = lead_approach_need_m(v_clear, v_lead, t_follow=t4) + 20.0
  assert lead_approach_decel_ms2(
    v_clear, v_lead, d_follow + far_slack, t4, model_prob=1.0, radar=False,
  ) is None


def test_near_follow_gap_mesh_applies_light_negative():
  """Inside need / near set gap: overlay may apply light −a to match v_lead."""
  v_lead = 22.0
  v_rel = 1.6
  v_ego = v_lead + v_rel
  t4 = nap_t_follow(4)
  t1 = nap_t_follow(1)
  t7 = nap_t_follow(7)
  cruise = LEAD_CLOSE_A_MIN_MS2
  for t_follow in (t1, t4, t7):
    d_follow = t_follow * v_lead + STOP_DISTANCE
    need = lead_approach_need_m(v_ego, v_lead, t_follow=t_follow)
    slack_in = max(LEAD_CLOSE_REMATCH_SLACK_M - 2.0, 3.0)
    assert slack_in < need
    a_over = lead_approach_decel_ms2(
      v_ego, v_lead, d_follow + slack_in, t_follow, model_prob=1.0, radar=True,
    )
    assert a_over is not None and a_over < 0.0
    assert abs(a_over) <= LEAD_APPROACH_MILD_A_MS2 + 1e-9
    meshed = apply_lead_approach_overlay(
      cruise, a_over, v_rel=v_rel, slack=slack_in,
    )
    assert meshed < 0.0
    assert meshed == pytest.approx(a_over)
    # City vs hwy setpoints differ; both mesh at their own gap.
    assert d_follow == pytest.approx(t_follow * v_lead + STOP_DISTANCE)


def test_nibble_overlay_does_not_steal_catchup_plus_a():
  """Far/gentle overlay must not beat rematch / cruise +a. Near gap can mesh."""
  assert apply_lead_approach_overlay(0.20, -0.05) == pytest.approx(0.20)
  assert apply_lead_approach_overlay(0.20, -0.13) == pytest.approx(0.20)
  assert apply_lead_approach_overlay(0.20, -LEAD_APPROACH_NIBBLE_MS2) == pytest.approx(-LEAD_APPROACH_NIBBLE_MS2)
  assert apply_lead_approach_overlay(0.20, -0.20) == pytest.approx(-0.20)
  assert apply_lead_approach_overlay(0.0, -0.05) == pytest.approx(-0.05)
  assert apply_lead_approach_overlay(-0.30, -0.05) == pytest.approx(-0.30)
  assert apply_lead_approach_overlay(-0.10, -0.20) == pytest.approx(-0.20)
  assert apply_lead_approach_overlay(0.20, None) == pytest.approx(0.20)
  # Large-gap catch-up keeps +a while closing 1.0–1.5; match-speed min()s.
  assert apply_lead_approach_overlay(0.20, -0.08, v_rel=0.3, slack=40.0) == pytest.approx(0.20)
  assert apply_lead_approach_overlay(0.20, -0.08, v_rel=1.2, slack=80.0) == pytest.approx(0.20)
  assert apply_lead_approach_overlay(0.20, -0.08, v_rel=1.6, slack=80.0) == pytest.approx(-0.08)
  assert apply_lead_approach_overlay(0.20, -0.08, v_rel=4.0, slack=40.0) == pytest.approx(-0.08)
  # Near the follow gap, a fading nibble still min()s (mesh, no Accel slam).
  assert apply_lead_approach_overlay(0.20, -0.08, v_rel=0.1, slack=3.0) == pytest.approx(-0.08)
  assert apply_lead_approach_overlay(0.20, -0.08, v_rel=1.2, slack=8.0) == pytest.approx(-0.08)
  assert apply_lead_approach_overlay(0.20, -0.08, v_rel=1.6, slack=5.0) == pytest.approx(-0.08)


def test_planner_wires_hysteresis_and_slew():
  """Overlay stays after map climb / Hill Climb; MPC hard path is still a min()."""
  from pathlib import Path
  planner = (Path(__file__).resolve().parents[1] / "lib/longitudinal_planner.py").read_text()
  assert "active=self._lead_approach_active or lead_held" in planner
  assert "model_prob=overlay_prob" in planner
  assert "radar=overlay_radar" in planner
  assert "slew_lead_approach_a(a_lead, self._lead_approach_a)" in planner
  assert "apply_lead_approach_overlay(" in planner
  assert "lead_follow_slack_m(" in planner
  assert "resolve_lead_close_hold(" in planner
  assert "lead_close_accel_ms2(" in planner
  assert "lead_approach_rapid_gate(" in planner
  assert "soft_limit_mpc_a_target(" in planner
  assert "cap_closing_lead_accel(" in planner
  assert "d_rel=overlay_d" in planner
  assert "lead_remaining_close_a_ms2(" in planner
  assert "v_ego=v_ego, v_cruise=v_hud_ms" in planner
  assert "a_env = 0.0 if a_grad is None else min(a_peak, float(a_grad))" in planner
  assert "live_ok = bool(lead.status) and lead_close_should_cap(" in planner
  assert "lead_owns_plan(" in planner
  assert "slack=overlay_slack" in planner
  assert "owned=self._lead_close_hold_owned" in planner
  assert "lead_approach_track_ok(" in planner
  assert "update_lead_settle(" in planner
  assert "settled=self._lead_settled" in planner
  assert "slew_lead_acquire_a(" in planner
  assert "update_lead_acquire(" in planner
  assert "acquiring=acquiring" in planner
  assert "update_lead_glide(" in planner
  assert "apply_lead_glide_a(" in planner
  assert "slew_near_gap_small_a(" in planner
  assert "apply_matched_inside_fd_a" not in planner
  assert "prev_floored=self._lead_soft_limit_floored" in planner
  assert "v_cruise=v_hud_ms" in planner
  assert "allow_rapid=allow_rapid" in planner
  assert "a_lead=lead_a_k" in planner
  assert "self._lead_approach_active = a_lead is not None" in planner
  assert "map_climb_replaces_mpc" in planner
  hill = (Path(__file__).resolve().parents[1] / "lib/hill_climb.py").read_text()
  assert "PITCH_CLIMB_RAD" in hill
  assert "lead_approach" not in hill or "Caller still" in hill


def test_mild_close_stays_light_regen():
  """Normal / mild close: earlier light |a|, not the 0.55 let-off."""
  t4 = nap_t_follow(4)
  v_lead = 50.0 * 0.44704
  d_follow = t4 * v_lead + STOP_DISTANCE

  # 10 mph close — kinematics wanted 0.55 at rel_need; stay at MILD.
  v_ego = 60.0 * 0.44704
  v_rel = v_ego - v_lead
  assert v_rel < LEAD_APPROACH_RAPID_DV_MS
  assert not lead_approach_is_rapid(v_rel, ttc=5.0)
  rel_need = (v_rel * v_rel) / (2.0 * LEAD_APPROACH_A_MS2)
  a = lead_approach_decel_ms2(v_ego, v_lead, d_follow + rel_need, t4, model_prob=1.0, radar=True)
  assert a is not None
  assert abs(a) <= LEAD_APPROACH_MILD_A_MS2 + 1e-9
  assert abs(a) < 0.30
  # Short TTC at this mild v_rel is "at the gap", not dumping.
  slack_short = v_rel * LEAD_APPROACH_RAPID_TTC_S
  ttc_short = lead_approach_ttc_s(slack_short, v_rel)
  assert ttc_short == pytest.approx(LEAD_APPROACH_RAPID_TTC_S)
  a_short = lead_approach_decel_ms2(v_ego, v_lead, d_follow + slack_short, t4)
  assert a_short is not None
  assert abs(a_short) <= LEAD_APPROACH_MILD_A_MS2 + 1e-9

  # 5 mph close — even lighter kinematics, still under the mild ceiling.
  v_slow = v_lead + 2.24
  slack = 8.0
  a_slow = lead_approach_decel_ms2(v_slow, v_lead, d_follow + slack, t4)
  a_kin = -(2.24 * 2.24) / (2.0 * lead_kinematic_slack_m(slack, 2.24))
  assert a_slow is not None
  assert a_slow == pytest.approx(max(a_kin, -LEAD_APPROACH_MILD_A_MS2), abs=1e-6)
  assert abs(a_slow) < LEAD_APPROACH_A_MS2 - 0.20
  # Gap bias makes the last meters firmer than raw slack, still mild.
  a_raw = -(2.24 * 2.24) / (2.0 * slack)
  assert lead_kinematic_slack_m(slack, 2.24) < slack
  assert a_kin <= a_raw + 1e-9


def test_rapid_close_allows_stronger_early_decel():
  """High closing-rate dump: kinematics up to 0.55, above the mild ceiling."""
  v_ego = 70.0 * 0.44704
  v_lead = 50.0 * 0.44704
  v_rel = v_ego - v_lead
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  assert v_rel >= LEAD_APPROACH_RAPID_DV_MS
  assert lead_approach_is_rapid(v_rel)

  a_far = lead_approach_decel_ms2(v_ego, v_lead, 180.0, t4, model_prob=1.0, radar=True, allow_rapid=True)
  slack_far = 180.0 - d_follow
  a_kin = -(v_rel * v_rel) / (2.0 * lead_kinematic_slack_m(slack_far, v_rel))
  assert a_far == pytest.approx(max(a_kin, -LEAD_APPROACH_A_MS2), abs=1e-6)
  assert abs(a_far) > LEAD_APPROACH_MILD_A_MS2 or abs(a_kin) <= LEAD_APPROACH_MILD_A_MS2

  d_dump = d_follow + v_rel * LEAD_APPROACH_RAPID_TTC_S
  a_dump = lead_approach_decel_ms2(v_ego, v_lead, d_dump, t4, model_prob=1.0, radar=True, allow_rapid=True)
  assert a_dump == pytest.approx(-LEAD_APPROACH_A_MS2, abs=0.08)
  assert abs(a_dump) > LEAD_APPROACH_MILD_A_MS2 + 0.20


def test_gap_opening_rematch_is_a_trickle():
  """When the gap is opening / just rematching near Follow Distance: small +a.

  Large-gap catch-up uses the Mannerisms Accel envelope (same as open-road).
  """
  assert lead_close_accel_ms2(1) == pytest.approx(LEAD_CLOSE_A_MIN_MS2)
  assert lead_close_accel_ms2(1, v_rel=0.0, slack=40.0) == pytest.approx(LEAD_CLOSE_A_MIN_MS2)
  assert lead_close_accel_ms2(5, v_rel=0.0, slack=40.0) == pytest.approx(LEAD_CLOSE_A_BASE_MS2)

  a_open = lead_close_accel_ms2(1, v_rel=-0.4, slack=3.0)
  assert a_open == pytest.approx(LEAD_CLOSE_OPENING_A_MS2)
  assert a_open < 0.10
  a_open_5 = lead_close_accel_ms2(5, v_rel=-0.2, slack=2.0)
  assert a_open_5 == pytest.approx(LEAD_CLOSE_OPENING_A_MS2)

  a_near = lead_close_accel_ms2(1, v_rel=0.30, slack=4.0)
  assert a_near == pytest.approx(LEAD_CLOSE_REMATCH_A_MS2)
  assert a_near < LEAD_CLOSE_A_MIN_MS2
  # Strong close near the gap: never rematch +a into a shrinking gap.
  a_closing = lead_close_accel_ms2(1, v_rel=2.0, slack=4.0)
  assert a_closing == pytest.approx(0.0)

  # Release slew is slower than onset: gap may keep opening while +a trickles.
  step = slew_lead_approach_a(None, -LEAD_APPROACH_MILD_A_MS2)
  assert step == pytest.approx(-LEAD_APPROACH_MILD_A_MS2 + LEAD_APPROACH_RELEASE_SLEW_MS2)
  assert abs(step - (-LEAD_APPROACH_MILD_A_MS2)) < LEAD_APPROACH_SLEW_MS2
  t4 = nap_t_follow(4)
  v_lead = 22.0
  d_follow = t4 * v_lead + STOP_DISTANCE
  slack = lead_follow_slack_m(d_follow + 40.0, v_lead, t4)
  assert slack == pytest.approx(40.0)
  assert slack > LEAD_CLOSE_REMATCH_SLACK_M
  near_slack = lead_follow_slack_m(d_follow + 5.0, v_lead, t4)
  assert near_slack == pytest.approx(5.0)
  assert near_slack < LEAD_CLOSE_REMATCH_SLACK_M


def test_settled_rematch_deadbands_accel_ceil_while_gap_ok_or_opening():
  """d7 20:25:33: after match, do not pin Accel-ceil +a while opening.

  Unsettled large same-speed gaps still use Accel catch-up.
  """
  a2 = lead_close_accel_ms2(2)
  assert a2 > 0.20
  # Evidence shape: Accel 2 ceil (~0.32) for 7 s as dRel 49→59 opening.
  a_open_ok = lead_close_accel_ms2(2, v_rel=-0.3, slack=10.0, settled=True)
  assert a_open_ok == pytest.approx(0.0)
  a_open_mid = lead_close_accel_ms2(2, v_rel=-0.3, slack=18.0, settled=True)
  assert a_open_mid == pytest.approx(0.0)
  assert a_open_mid < 0.05
  # Slack clearly large *and* lead pulling away: hunt, not Accel ceil.
  a_pull = lead_close_accel_ms2(2, v_rel=-0.4, slack=22.0, settled=True)
  assert 0.0 < a_pull <= LEAD_CLOSE_OPENING_A_MS2 + 0.05
  assert a_pull < a2 * 0.5
  a_pull_40 = lead_close_accel_ms2(2, v_rel=-0.4, slack=40.0, settled=True)
  assert a_pull_40 == pytest.approx(lead_hunt_accel_ms2(a2, 40.0))
  assert a_pull_40 < a2 * LEAD_HUNT_A_FRAC + 1e-9
  assert a_pull_40 < a2 - 0.10
  # Matched, gap OK: trickle if slightly closing, 0 if opening.
  a_ok = lead_close_accel_ms2(2, v_rel=0.2, slack=8.0, settled=True)
  assert a_ok == pytest.approx(LEAD_CLOSE_OPENING_A_MS2)
  assert lead_close_accel_ms2(2, v_rel=-0.1, slack=8.0, settled=True) == pytest.approx(0.0)
  # Ego clearly slower: Accel so grade / cruise can recover speed.
  assert lead_close_accel_ms2(2, v_rel=-0.6, slack=10.0, settled=True) == pytest.approx(a2)
  assert lead_close_accel_ms2(2, v_rel=-0.6, slack=18.0, settled=True) == pytest.approx(a2)
  # Remaining Follow Distance (≲ 4 m) still trickles, even if slightly opening.
  a_finish = lead_close_accel_ms2(2, v_rel=-0.1, slack=3.0, settled=True)
  assert a_finish == pytest.approx(LEAD_CLOSE_OPENING_A_MS2)
  assert lead_close_accel_ms2(2, v_rel=0.2, slack=3.0, settled=True) == pytest.approx(
    LEAD_CLOSE_OPENING_A_MS2
  )
  # At Follow Distance, matched / slight sag: Accel so grade can hold speed.
  assert lead_close_accel_ms2(2, v_rel=-0.1, slack=0.0, settled=True) == pytest.approx(a2)
  assert lead_close_accel_ms2(2, v_rel=0.0, slack=0.3, settled=True) == pytest.approx(a2)
  # Large-gap unsettled catch-up still Accel while closing ≳ 1.0 (#187).
  assert lead_close_accel_ms2(1, v_rel=1.2, slack=80.0, settled=False) == pytest.approx(
    LEAD_CLOSE_A_MIN_MS2
  )
  # Large same-speed gap that never matched: Accel-owned catch-up.
  assert lead_close_accel_ms2(2, v_rel=0.0, slack=40.0, settled=False) == pytest.approx(a2)
  assert lead_close_accel_ms2(5, v_rel=0.0, slack=80.0) == pytest.approx(LEAD_CLOSE_A_BASE_MS2)
  # #190: closing ≳ 1.0 still 0 after settle on a large gap.
  assert lead_close_accel_ms2(2, v_rel=1.2, slack=22.0, settled=True) == pytest.approx(0.0)
  assert lead_close_accel_ms2(2, v_rel=1.6, slack=40.0, settled=True) == pytest.approx(0.0)
  # Last meters may still trickle while closing ≳ 1.0 (finish FD).
  assert lead_close_accel_ms2(2, v_rel=1.2, slack=3.0, settled=True) == pytest.approx(
    LEAD_CLOSE_OPENING_A_MS2
  )


def test_remaining_close_commands_trickle_when_mpc_sits_at_zero():
  assert lead_remaining_close_a_ms2(0.0, 0.0, 3.0) == pytest.approx(LEAD_CLOSE_OPENING_A_MS2)
  assert lead_remaining_close_a_ms2(-0.20, 0.2, 3.0) == pytest.approx(-0.20)
  assert lead_remaining_close_a_ms2(-0.05, 0.0, 3.0) == pytest.approx(LEAD_CLOSE_OPENING_A_MS2)
  assert lead_remaining_close_a_ms2(0.0, 0.2, 8.0) == pytest.approx(0.0)
  assert lead_remaining_close_a_ms2(0.0, -0.4, 0.0) == pytest.approx(LEAD_CLOSE_OPENING_A_MS2)
  assert lead_remaining_close_a_ms2(0.0, -0.4, -2.0) == pytest.approx(0.0)
  assert lead_remaining_close_a_ms2(0.0, 1.6, 3.0) == pytest.approx(0.0)
  assert lead_remaining_close_a_ms2(0.0, -0.6, 10.0) == pytest.approx(LEAD_CLOSE_A_MIN_MS2)
  assert lead_remaining_close_a_ms2(0.0, -0.3, 18.0) == pytest.approx(0.0)


def test_max_gates_lead_rematch_and_remaining_close_plus_a():
  """MAX is a hard ceiling: faster lead must not command +a to keep up.

  ea 11:46: speed-sag rematch Accel-1 (+0.36) while already 1–2 mph over
  a 60 MAX. Under MAX, grade sag / same-speed catch-up still rematch.
  Overlay ease / emergency −a still win at MAX.
  """
  v_max = 60.0 * 0.44704
  v_at = v_max
  v_over = v_max + 1.0 * 0.44704
  v_under = v_max - 5.0 * 0.44704
  assert lead_at_or_above_max(v_at, v_max)
  assert lead_at_or_above_max(v_over, v_max)
  assert lead_at_or_above_max(v_max - TRACK_DEADBAND_MS, v_max)
  assert not lead_at_or_above_max(v_under, v_max)
  assert not lead_at_or_above_max(None, v_max)
  assert not lead_at_or_above_max(v_at, None)

  # Remaining-close: no +a for speed-sag / finish at or above MAX.
  assert lead_remaining_close_a_ms2(
    0.0, -0.6, 10.0, v_ego=v_at, v_cruise=v_max,
  ) == pytest.approx(0.0)
  assert lead_remaining_close_a_ms2(
    0.0, -0.8, 56.0, v_ego=v_over, v_cruise=v_max,
  ) == pytest.approx(0.0)
  assert lead_remaining_close_a_ms2(
    0.0, 0.0, 3.0, v_ego=v_at, v_cruise=v_max,
  ) == pytest.approx(0.0)
  assert lead_remaining_close_a_ms2(
    -0.05, -0.4, 3.0, v_ego=v_over, v_cruise=v_max,
  ) == pytest.approx(-0.05)
  # Real ease / emergency −a still pass through at MAX.
  assert lead_remaining_close_a_ms2(
    -0.20, -0.6, 10.0, v_ego=v_at, v_cruise=v_max,
  ) == pytest.approx(-0.20)
  assert lead_remaining_close_a_ms2(
    -2.0, 8.0, 12.0, v_ego=v_at, v_cruise=v_max,
  ) == pytest.approx(-2.0)
  # Under MAX, speed-sag rematch and finish trickle still arm.
  assert lead_remaining_close_a_ms2(
    0.0, -0.6, 10.0, v_ego=v_under, v_cruise=v_max,
  ) == pytest.approx(LEAD_CLOSE_A_MIN_MS2)
  assert lead_remaining_close_a_ms2(
    0.0, 0.0, 3.0, v_ego=v_under, v_cruise=v_max,
  ) == pytest.approx(LEAD_CLOSE_OPENING_A_MS2)

  # Close-cap / settled rematch: at MAX the +a envelope is 0.
  a2 = lead_close_accel_ms2(2)
  assert lead_close_accel_ms2(
    2, v_rel=-0.6, slack=10.0, settled=True, v_ego=v_at, v_cruise=v_max,
  ) == pytest.approx(0.0)
  assert lead_close_accel_ms2(
    1, v_rel=-0.8, slack=40.0, settled=False, v_ego=v_over, v_cruise=v_max,
  ) == pytest.approx(0.0)
  assert lead_close_accel_ms2(
    5, v_rel=0.0, slack=80.0, v_ego=v_at, v_cruise=v_max,
  ) == pytest.approx(0.0)
  # Under MAX, same-speed catch-up and speed-sag rematch still Accel.
  assert lead_close_accel_ms2(
    2, v_rel=0.0, slack=80.0, v_ego=v_under, v_cruise=v_max,
  ) == pytest.approx(a2)
  assert lead_close_accel_ms2(
    2, v_rel=-0.6, slack=10.0, settled=True, v_ego=v_under, v_cruise=v_max,
  ) == pytest.approx(a2)


def test_settled_gap_hunt_is_small_accel_proportional_not_ceil():
  """Gap error ≳ 15 m and |closing| < 1: small Accel-proportional close."""
  a5 = lead_close_accel_ms2(5)
  a_hunt = lead_close_accel_ms2(5, v_rel=0.3, slack=30.0, settled=True)
  assert a_hunt == pytest.approx(lead_hunt_accel_ms2(a5, 30.0))
  assert a_hunt < a5 * LEAD_HUNT_A_FRAC + 1e-9
  assert a_hunt < a5 * 0.5
  assert a_hunt >= LEAD_CLOSE_OPENING_A_MS2 - 1e-9
  a_15 = lead_close_accel_ms2(5, v_rel=0.2, slack=15.0, settled=True)
  assert a_15 == pytest.approx(LEAD_CLOSE_OPENING_A_MS2)
  # Full Accel ceil must not pulse at ~30 m after settle.
  assert lead_close_accel_ms2(2, v_rel=0.2, slack=30.0, settled=True) < lead_close_accel_ms2(2) * 0.5


def test_lead_settle_arms_near_gap_not_far_same_speed():
  age, settled = 0.0, False
  while age + 0.05 < LEAD_SETTLE_HOLD_S - 1e-9:
    age, settled = update_lead_settle(age, settled, 0.2, 10.0, 0.05)
    assert not settled
  age, settled = update_lead_settle(age, settled, 0.2, 10.0, 0.05)
  assert settled
  # Hold through a growing gap (dRel 49→59).
  age, settled = update_lead_settle(age, True, -0.4, 22.0, 0.05)
  assert settled
  # Far same-speed never arms.
  age, settled = update_lead_settle(0.0, False, 0.0, 80.0, 0.05)
  assert not settled and age == pytest.approx(0.0)
  for _ in range(20):
    age, settled = update_lead_settle(age, settled, 0.0, 80.0, 0.05)
  assert not settled
  # Inside Follow Distance is a too-close recovery, not a matched follow.
  assert not lead_is_settled_sample(0.1, -5.0)
  age, settled = 0.0, False
  for _ in range(20):
    age, settled = update_lead_settle(age, settled, 0.1, -20.0, 0.05)
  assert not settled and age == pytest.approx(0.0)
  # Closing ≳ 1.5 clears settle so match-speed −a owns.
  age, settled = update_lead_settle(1.0, True, 1.6, 10.0, 0.05, closing_hard=True)
  assert not settled and age == pytest.approx(0.0)
  age, settled = update_lead_settle(0.4, False, 0.2, 10.0, 0.05, present=False)
  assert not settled


def test_follow_plus_a_slew_rate_limits_ownership_flips_not_closing():
  """Cruise↔lead +a flips slew; closing ≳ 1.0 / −a is immediate."""
  a = slew_follow_plus_a(0.323, 0.0, v_rel=0.2)
  assert a == pytest.approx(LEAD_ATARGET_SLEW_MS2)
  nxt = slew_follow_plus_a(0.323, a, v_rel=-0.3)
  assert nxt == pytest.approx(2 * LEAD_ATARGET_SLEW_MS2)
  # Closing block / match-speed −a is not delayed.
  assert slew_follow_plus_a(0.0, 0.323, v_rel=1.2) == pytest.approx(0.0)
  assert slew_follow_plus_a(-0.40, 0.20, v_rel=1.6) == pytest.approx(-0.40)
  assert slew_follow_plus_a(-0.22, 0.0, v_rel=0.3) == pytest.approx(-0.22)


def test_lead_acquire_slew_kills_first_latch_yoyo():
  """e4 09:53:19: first latch −0.46 must not punch, then rematch +0.05.

  Instant −a (slew_follow_plus_a) applied the spike; rematch slewed
  back over ~0.5 s. Acquire slews both ways so the first frame is a
  nibble, not a regen→accel yo-yo.
  """
  d_rel = 118.0
  slack = 80.0
  v_rel = 1.2
  punched = slew_lead_acquire_a(
    -0.46, 0.0, v_rel, d_rel=d_rel, slack=slack, acquiring=True,
  )
  assert punched == pytest.approx(-LEAD_ACQUIRE_SLEW_MS2)
  assert punched > -0.20
  # Rematch +a also steps; no slam from −0.05 to +0.05.
  rematch = slew_lead_acquire_a(
    0.05, punched, 0.3, d_rel=d_rel, slack=slack, acquiring=True,
  )
  assert rematch == pytest.approx(0.0, abs=1e-9)
  # After the window, −a is immediate again (established lead).
  assert slew_lead_acquire_a(
    -0.46, 0.0, v_rel, d_rel=d_rel, slack=slack, acquiring=False,
  ) == pytest.approx(-0.46)
  age, acquiring = 0.0, False
  while True:
    age, acquiring = update_lead_acquire(age, True, 0.05)
    if age + 1e-9 >= LEAD_ACQUIRE_HOLD_S:
      break
    assert acquiring
  age, acquiring = update_lead_acquire(age, True, 0.05)
  assert not acquiring
  age, acquiring = update_lead_acquire(0.4, False, 0.05)
  assert not acquiring and age == pytest.approx(0.0)


def test_lead_acquire_slew_keeps_rapid_and_near_bumper_authority():
  """Dumping / near-bumper / FCW first latch must not wait."""
  assert lead_mpc_needs_full_authority(8.0, 40.0, slack=20.0, confirm_rapid=False)
  assert slew_lead_acquire_a(
    -2.0, 0.0, 8.0, d_rel=40.0, slack=20.0, acquiring=True,
  ) == pytest.approx(-2.0)
  assert lead_mpc_needs_full_authority(2.0, 10.0, slack=-4.0)
  assert slew_lead_acquire_a(
    -2.0, 0.0, 2.0, d_rel=10.0, slack=-4.0, acquiring=True,
  ) == pytest.approx(-2.0)
  assert slew_lead_acquire_a(
    -2.0, 0.0, 3.0, d_rel=LEAD_MPC_SOFT_NEAR_M, slack=2.0, acquiring=True,
  ) == pytest.approx(-2.0)
  assert slew_lead_acquire_a(
    -2.0, 0.0, 1.2, d_rel=118.0, slack=80.0, acquiring=True, fcw=True,
  ) == pytest.approx(-2.0)
  # Near-gap aLead / closing ≳ 1.5 at range: slew (EV slight lift).
  assert slew_lead_acquire_a(
    -0.80, 0.0, 0.4, d_rel=40.0, slack=8.0, acquiring=True, a_lead=-0.80,
  ) == pytest.approx(-LEAD_ACQUIRE_SLEW_MS2)
  # 10:48 class: −0.996 at 80–130 m must not punch on first latch.
  assert slew_lead_acquire_a(
    -0.996, 0.0, 2.0, d_rel=110.0, slack=60.0, acquiring=True,
  ) == pytest.approx(-LEAD_ACQUIRE_SLEW_MS2)
  assert soft_limit_mpc_a_target(-0.996, 25.0, 23.0, 110.0, slack=60.0) == pytest.approx(
    -LEAD_APPROACH_MILD_A_MS2
  )
  # Same-speed slack ≤ 0 is a too-close recovery, not a dump.
  assert not lead_mpc_needs_full_authority(0.2, 40.0, slack=-2.0)
  assert soft_limit_mpc_a_target(-2.5, 25.0, 24.8, 40.0, slack=-2.0) == pytest.approx(
    -LEAD_APPROACH_MILD_A_MS2
  )


def test_one_outlier_rapid_v_rel_does_not_commit_hard_regen():
  """One dump-shaped v_rel blip stays at 0.22; four consecutive dump samples reach 0.55."""
  t4 = nap_t_follow(4)
  v_lead = 22.4
  d_follow = t4 * v_lead + STOP_DISTANCE
  v_rel = LEAD_APPROACH_RAPID_DV_MS + 0.5
  v_ego = v_lead + v_rel
  d_rel = d_follow + 8.0
  assert lead_approach_is_rapid(v_rel)
  ungated = lead_approach_decel_ms2(v_ego, v_lead, d_rel, t4, allow_rapid=False)
  assert ungated == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  n = 0
  for _ in range(LEAD_APPROACH_RAPID_CONFIRM_N - 1):
    allow, n = lead_approach_rapid_gate(v_rel, n)
    assert not allow
  allow, n = lead_approach_rapid_gate(v_rel, n)
  assert allow
  assert n == LEAD_APPROACH_RAPID_CONFIRM_N
  gated = lead_approach_decel_ms2(v_ego, v_lead, d_rel, t4, allow_rapid=True)
  assert gated == pytest.approx(-LEAD_APPROACH_A_MS2)
  allow, n = lead_approach_rapid_gate(v_rel, 0)
  assert not allow
  allow, n = lead_approach_rapid_gate(1.0, n)
  assert not allow and n == 0
  allow, n = lead_approach_rapid_gate(v_rel, n)
  assert not allow and n == 1
  allow, n = lead_approach_rapid_gate(v_rel, n, sample_ok=False)
  assert not allow and n == 0


def test_lead_close_hold_covers_status_flicker_and_far_bosch():
  d_use, v_use, held_d, held_v, age = resolve_lead_close_hold(
    True, 80.0, 22.0, None, None, 0.0, 0.05,
  )
  assert d_use == pytest.approx(80.0)
  assert v_use == pytest.approx(22.0)
  for _ in range(8):
    d_use, v_use, held_d, held_v, age = resolve_lead_close_hold(
      False, 0.0, 0.0, held_d, held_v, age, 0.05,
    )
    assert d_use == pytest.approx(80.0)
  assert age < LEAD_CLOSE_HOLD_S
  d_use, v_use, held_d, held_v, age = resolve_lead_close_hold(
    False, 0.0, 0.0, 80.0, 22.0, LEAD_CLOSE_HOLD_S, 0.05,
  )
  assert d_use is None
  assert held_d is None
  d_use, v_use, held_d, held_v, age = resolve_lead_close_hold(
    True, 160.0, 22.0, 80.0, 22.0, 0.10, 0.05, model_prob=1.0, radar=True,
  )
  assert d_use == pytest.approx(160.0)
  assert lead_close_should_cap(160.0, model_prob=1.0, radar=True)
  d_use, v_use, held_d, held_v, age = resolve_lead_close_hold(
    True, 160.0, 22.0, None, None, 0.0, 0.05, model_prob=0.2, radar=False,
  )
  assert d_use is None
  assert not lead_close_should_cap(160.0, model_prob=0.2, radar=False)
  d_use, v_use, held_d, held_v, age = resolve_lead_close_hold(
    True, 160.0, 22.0, None, None, 0.0, 0.05, model_prob=0.2, radar=True,
  )
  assert d_use == pytest.approx(160.0)
  assert lead_close_should_cap(160.0, model_prob=0.2, radar=True)
  d_use, v_use, held_d, held_v, age = resolve_lead_close_hold(
    True, 220.0, 22.0, 80.0, 22.0, 0.10, 0.05, model_prob=1.0, radar=True,
  )
  assert d_use is None
  assert not lead_close_should_cap(220.0, model_prob=1.0, radar=True)


def test_mpc_soft_limit_floors_chatter_not_match_speed():
  """Mild floor clamps non-emergency MPC −a to slight-lift.

  Closing ≳ 1.5 / near-gap aLead / 10 mph town entry stay at MILD.
  Rapid / FCW / near-bumper / crash still dump. Rematch still floors.
  """
  v_ego = 25.0
  d_rel = 40.0
  # Same-speed / tiny v_rel noise, lead not braking: anti-chatter floor.
  assert soft_limit_mpc_a_target(-2.5, v_ego, v_ego, d_rel, a_lead=0.0) == pytest.approx(
    -LEAD_APPROACH_MILD_A_MS2
  )
  v_lead_jitter = v_ego - 0.5
  assert (v_ego - v_lead_jitter) < LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS
  assert soft_limit_mpc_a_target(-2.5, v_ego, v_lead_jitter, d_rel, a_lead=0.05) == pytest.approx(
    -LEAD_APPROACH_MILD_A_MS2
  )
  # Commands already milder than MILD pass through.
  assert soft_limit_mpc_a_target(-0.10, v_ego, v_lead_jitter, d_rel) == pytest.approx(-0.10)
  # Gap opening rematch: still floor (do not undo rematch trickle).
  assert soft_limit_mpc_a_target(-2.5, v_ego, v_ego + 0.4, d_rel, a_lead=0.0) == pytest.approx(
    -LEAD_APPROACH_MILD_A_MS2
  )
  # Just below the close skip, lead not braking: still chatter, still floor.
  v_lead_under = v_ego - (LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS - 0.1)
  assert soft_limit_mpc_a_target(-2.5, v_ego, v_lead_under, d_rel, a_lead=0.0) == pytest.approx(
    -LEAD_APPROACH_MILD_A_MS2
  )

  # Closing at/above 1.5 is still a mild EV close — slight-lift floor.
  v_lead_close = v_ego - LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS
  assert (v_ego - v_lead_close) < LEAD_APPROACH_RAPID_DV_MS
  assert soft_limit_mpc_a_target(-2.5, v_ego, v_lead_close, d_rel, a_lead=0.0) == pytest.approx(
    -LEAD_APPROACH_MILD_A_MS2
  )
  assert soft_limit_mpc_a_target(
    -2.5, v_ego, v_lead_close, d_rel, a_lead=0.0, slack=8.0,
  ) == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  # e4 40 m / 9.5 m/s: large slack, small adjustment — do not dump −2.33.
  v_e4 = 9.5
  v_lead_e4 = v_e4 - 1.6
  slack_e4 = 22.0
  assert slack_e4 > LEAD_ALEAD_MATCH_SLACK_M
  a_e4 = soft_limit_mpc_a_target(
    -2.33, v_e4, v_lead_e4, 40.0, a_lead=0.0, slack=slack_e4,
  )
  assert a_e4 == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  assert a_e4 > -0.60
  # 10 mph-class close under the rapid gate stays slight-lift.
  v_lead_town = v_ego - 4.4
  assert (v_ego - v_lead_town) < LEAD_APPROACH_RAPID_DV_MS
  assert soft_limit_mpc_a_target(-1.5, v_ego, v_lead_town, 17.0) == pytest.approx(
    -LEAD_APPROACH_MILD_A_MS2
  )
  # Lead braking / near-gap aLead: still MILD (not match-speed dump).
  assert LEAD_APPROACH_SOFT_LIMIT_ALEAD_MS2 <= -0.2
  assert soft_limit_mpc_a_target(
    -2.5, v_ego, v_ego - 0.4, d_rel, a_lead=LEAD_APPROACH_SOFT_LIMIT_ALEAD_MS2,
  ) == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  assert soft_limit_mpc_a_target(
    -2.5, v_ego, v_ego - 0.4, d_rel, a_lead=-0.4, slack=8.0,
  ) == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  # Far opening slack: aLead alone does not skip the chatter floor.
  assert soft_limit_mpc_a_target(
    -2.5, v_ego, v_ego + 1.44, 64.0, a_lead=-1.46, slack=40.0,
  ) == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  # aLead just milder than the skip still floors when not closing.
  assert soft_limit_mpc_a_target(
    -2.5, v_ego, v_ego - 0.4, d_rel, a_lead=LEAD_APPROACH_SOFT_LIMIT_ALEAD_MS2 + 0.05,
  ) == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)

  # Route-shaped town entry: closing 1.6 m/s, gap 41 m, lead decelerating.
  v_route = 20.0
  v_lead_route = v_route - 1.6
  assert 1.6 >= LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS
  assert 1.6 < LEAD_APPROACH_RAPID_DV_MS
  assert soft_limit_mpc_a_target(-1.2, v_route, v_lead_route, 41.0, a_lead=-0.8) == pytest.approx(
    -LEAD_APPROACH_MILD_A_MS2
  )
  assert soft_limit_mpc_a_target(-1.2, v_route, v_lead_route, 41.0) == pytest.approx(
    -LEAD_APPROACH_MILD_A_MS2
  )

  # Confirmed rapid / FCW / crash still dump.
  v_lead_rapid = v_ego - LEAD_APPROACH_RAPID_DV_MS - 0.5
  d_far = 80.0
  assert soft_limit_mpc_a_target(
    -2.5, v_ego, v_lead_rapid, d_far, allow_rapid=True,
  ) == pytest.approx(-2.5)
  assert soft_limit_mpc_a_target(-2.5, v_ego, v_ego, d_rel, fcw=True) == pytest.approx(-2.5)
  assert soft_limit_mpc_a_target(-2.5, v_ego, v_ego, d_rel, crash_cnt=1) == pytest.approx(-2.5)
  # Already inside the stop gap: do not floor.
  assert soft_limit_mpc_a_target(-2.5, 5.0, 0.0, STOP_DISTANCE - 1.0) == pytest.approx(-2.5)
  # Closing under the skip, but kinematics to stop need more than 0.55.
  v_mild_stop = 1.2
  slack_stop = 1.0
  assert v_mild_stop < LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS
  a_need = -(v_mild_stop * v_mild_stop) / (2.0 * slack_stop)
  assert a_need < -LEAD_APPROACH_A_MS2
  assert soft_limit_mpc_a_target(-2.5, v_mild_stop, 0.0, STOP_DISTANCE + slack_stop) == pytest.approx(-2.5)


def test_closing_lead_hard_blocks_rematch_plus_a():
  """d7 20:33: closing ≥ 1.5 must never rematch +a; prefer match-speed −a."""
  assert not lead_is_closing(0.8)
  assert lead_is_closing(1.0)
  assert lead_is_closing(0.4, a_lead=-0.25)
  assert not lead_owns_plan(1.2)
  assert lead_owns_plan(1.5)
  assert lead_owns_plan(0.4, a_lead=-0.25)

  # Same-speed / below the block: cruise +a may stand.
  assert cap_closing_lead_accel(0.20, 0.8, a_lead=0.0, lead_present=True) == pytest.approx(0.20)
  # Closing 1.0–1.5: coast ≤0, no match-speed yet.
  assert cap_closing_lead_accel(0.20, 1.2, a_lead=0.0, lead_present=True) == pytest.approx(0.0)
  # Closing ≥ 1.5: never rematch +a. Extra match-speed −a is emergency only.
  a_match = cap_closing_lead_accel(0.201, 2.68, a_lead=-0.20, lead_present=True)
  assert a_match == pytest.approx(0.0)
  # Route-shaped rematch: +0.20 while closing 2.2–3.8 at ~85 m → coast.
  a_d7 = cap_closing_lead_accel(0.215, 3.79, a_lead=-0.29, lead_present=True)
  assert a_d7 == pytest.approx(0.0)
  # Overlay-MILD pin while closing: match-speed wins (secondary −0.22 path).
  assert cap_closing_lead_accel(-0.22, 6.8, a_lead=0.0, lead_present=True) == pytest.approx(
    -LEAD_CLOSING_MATCH_GAIN * 6.8
  )
  # No lead: rematch block does not fire.
  assert cap_closing_lead_accel(0.80, 3.0, lead_present=False) == pytest.approx(0.80)
  # Hold-owned even if this frame's v_rel dipped.
  assert cap_closing_lead_accel(0.20, 0.4, a_lead=0.0, lead_present=True, owned=True) <= 0.0
  # Past Bosch: no match-speed crawl on a 215 m closing lock.
  past = LEAD_CLOSE_MAX_M + LEAD_APPROACH_MAX_HOLD_M + 15.0
  assert cap_closing_lead_accel(
    0.0, 4.4, a_lead=0.0, lead_present=True, d_rel=past,
  ) == pytest.approx(0.0)
  assert cap_closing_lead_accel(
    0.20, 4.4, a_lead=0.0, lead_present=True, d_rel=160.0,
  ) == pytest.approx(0.0)
  # Last meters: closing 1.0–1.5 may keep trickle (finish FD).
  assert cap_closing_lead_accel(
    LEAD_CLOSE_OPENING_A_MS2, 1.2, a_lead=0.0, lead_present=True, slack=3.0,
  ) == pytest.approx(LEAD_CLOSE_OPENING_A_MS2)


def test_alead_only_does_not_own_opening_or_far_slack():
  """da 2026-09-18: far opening aLeadK must not snap cruise +a to aLead.

  Keep #190 real closing ≳ 1.0–1.5 and near-gap braking-lead match.
  """
  assert 15.0 <= LEAD_ALEAD_MATCH_SLACK_M <= 25.0
  # Incident-shaped: v_rel opening, slack ~40 m, aLeadK −1.46.
  assert not lead_alead_owns_match(-1.44, -1.46, slack=40.0)
  assert not lead_is_closing(-1.44, a_lead=-1.46, slack=40.0)
  assert not lead_owns_plan(-1.44, a_lead=-1.46, slack=40.0)
  a_keep = cap_closing_lead_accel(
    0.47, -1.44, a_lead=-1.46, lead_present=True, slack=40.0,
  )
  assert a_keep == pytest.approx(0.47)
  # Far slack even if not opening: aLead alone does not match.
  assert not lead_alead_owns_match(0.4, -1.46, slack=40.0)
  assert cap_closing_lead_accel(
    0.47, 0.4, a_lead=-1.46, lead_present=True, slack=40.0,
  ) == pytest.approx(0.47)
  # Opening with unknown slack: do not aLead-own.
  assert not lead_alead_owns_match(-1.44, -1.46, slack=None)
  assert cap_closing_lead_accel(
    0.47, -1.44, a_lead=-1.46, lead_present=True,
  ) == pytest.approx(0.47)

  # Real closing ≳ 1.5 still never rematch +a (even with large slack).
  # Match-speed extra −a is near-gap / rapid only so it cannot undo
  # the large-slack MILD floor (e4 40 m / 9.5 m/s).
  a_close = cap_closing_lead_accel(
    0.47, 1.6, a_lead=-0.40, lead_present=True, slack=40.0,
  )
  assert a_close == pytest.approx(0.0)
  a_mild = cap_closing_lead_accel(
    -LEAD_APPROACH_MILD_A_MS2, 1.6, a_lead=0.0, lead_present=True, slack=40.0,
  )
  assert a_mild == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  # Planner-computed slack at 160 m / 4.4 m/s (under rapid): nibble only.
  assert cap_closing_lead_accel(
    -0.05, 4.4, a_lead=0.0, lead_present=True, slack=100.0, d_rel=160.0,
  ) == pytest.approx(-0.05)
  # Large-gap catch-up still Accel while closing 1.0–1.5 (#187).
  assert cap_closing_lead_accel(
    0.47, 1.2, a_lead=0.0, lead_present=True, slack=40.0,
  ) == pytest.approx(0.47)
  # Near-gap rematch-block still zeros +a at closing ≳ 1.0.
  assert cap_closing_lead_accel(
    0.47, 1.2, a_lead=0.0, lead_present=True, slack=8.0,
  ) == pytest.approx(0.0)

  # Near-gap braking lead: block +a, no match-speed dump (EV slight lift).
  assert lead_alead_owns_match(-0.20, -0.80, slack=8.0)
  assert lead_owns_plan(0.4, a_lead=-0.25, slack=8.0)
  a_near = cap_closing_lead_accel(
    0.47, 0.4, a_lead=-0.80, lead_present=True, slack=8.0,
  )
  assert a_near == pytest.approx(0.0)
  a_open_near = cap_closing_lead_accel(
    0.47, -0.20, a_lead=-0.80, lead_present=True, slack=8.0,
  )
  assert a_open_near == pytest.approx(0.0)


def test_lead_flicker_hold_still_blocks_plus_a():
  """Dropped leadOne.status keeps the last in-window closing lead for ownership."""
  d_use, v_use, held_d, held_v, age = resolve_lead_close_hold(
    True, 110.0, 25.0, None, None, 0.0, 0.05, model_prob=0.4, radar=True,
  )
  assert d_use == pytest.approx(110.0)
  v_ego = 28.2
  v_rel = v_ego - v_use
  assert v_rel >= LEAD_CLOSING_MATCH_MS
  a_live = cap_closing_lead_accel(0.20, v_rel, a_lead=-0.18, lead_present=True)
  assert a_live <= 0.0
  for _ in range(8):
    d_use, v_use, held_d, held_v, age = resolve_lead_close_hold(
      False, 0.0, 0.0, held_d, held_v, age, 0.05,
    )
    assert d_use == pytest.approx(110.0)
    a_held = cap_closing_lead_accel(
      0.20, v_ego - v_use, a_lead=-0.18, lead_present=True, owned=True,
    )
    assert a_held <= 0.0
  assert age < LEAD_CLOSE_HOLD_S


def test_far_radar_lead_eases_without_model_prob():
  """Radar-associated 160–180 m lead is accepted; vision-only far flicker is not."""
  v_ego = 60.0 * 0.44704
  v_lead = 50.0 * 0.44704
  t4 = nap_t_follow(4)
  for d_rel in (150.0, 160.0, 180.0):
    a = lead_approach_decel_ms2(
      v_ego, v_lead, d_rel, t4, model_prob=0.15, radar=True,
    )
    assert a is not None and a < 0.0
    assert lead_approach_track_ok(d_rel, model_prob=0.15, radar=True)
    assert not lead_approach_track_ok(d_rel, model_prob=0.9, radar=False)


def test_matched_speed_glide_deadbands_chatter_with_hysteresis():
  """e8 settle yo-yo: |v_rel| small near the gap → a≈0, no rematch↔mild flip.

  Acquire / far catch-up / rapid / bumper stay out of the deadband.
  """
  assert lead_is_glide_sample(0.15, 3.0)
  # Slight sag at the gap: kill leftover −a so grade does not chew.
  assert lead_is_glide_sample(-0.30, 3.0)
  # Still closing in the finish band must keep −a (do not coast through FD).
  assert not lead_is_glide_sample(0.30, 3.0)
  # Real speed sag is Accel-owned.
  assert not lead_is_glide_sample(-0.55, 3.0)
  # Already inside FD is a too-close recovery — rematch +a / mild −a stand.
  assert not lead_is_glide_sample(-0.15, -8.0)
  assert not lead_is_glide_sample(0.15, -8.0)
  assert not lead_is_glide_sample(0.15, 20.0)
  assert not lead_is_glide_sample(0.8, 3.0)
  assert not lead_is_glide_sample(0.15, None)

  assert update_lead_glide(False, 0.15, 3.0) is True
  # Hysteresis: hold a slightly larger close than enter, not a 0.55 close.
  assert LEAD_GLIDE_VREL_MS < 0.30 < LEAD_GLIDE_VREL_OFF_MS
  assert update_lead_glide(True, 0.30, 3.0) is True
  assert update_lead_glide(True, -0.30, 3.0) is True
  assert update_lead_glide(True, 0.45, 3.0) is False
  assert update_lead_glide(True, -0.55, 3.0) is False
  assert update_lead_glide(True, 0.15, 16.0) is False
  assert update_lead_glide(True, 0.15, -2.0) is False
  # First-latch / danger never glide.
  assert update_lead_glide(False, 0.15, 3.0, acquiring=True) is False
  assert update_lead_glide(False, 0.15, 3.0, fcw=True) is False
  assert update_lead_glide(False, 0.15, 3.0, d_rel=LEAD_MPC_SOFT_NEAR_M) is False
  assert update_lead_glide(False, 8.0, 3.0) is False

  # Rematch trickle ↔ mild floor snaps to coast / slight lift.
  assert apply_lead_glide_a(-LEAD_APPROACH_MILD_A_MS2, True) == pytest.approx(0.0)
  assert apply_lead_glide_a(-0.17, True) == pytest.approx(0.0)
  assert apply_lead_glide_a(LEAD_CLOSE_OPENING_A_MS2, True) == pytest.approx(LEAD_GLIDE_A_MS2)
  assert apply_lead_glide_a(0.0, True) == pytest.approx(0.0)
  assert apply_lead_glide_a(-0.22, False) == pytest.approx(-0.22)
  assert apply_lead_glide_a(LEAD_CLOSE_OPENING_A_MS2, False) == pytest.approx(
    LEAD_CLOSE_OPENING_A_MS2
  )
  # Accel-ceil grade hold and MPC dump sit outside the chatter band.
  assert apply_lead_glide_a(0.32, True) == pytest.approx(0.32)
  assert apply_lead_glide_a(-2.0, True) == pytest.approx(-2.0)


def test_inside_fd_slow_close_commands_mild_not_dump():
  """e8 gap-cross: closing ~1.25 m/s inside FD commands MILD, not rematch or dump."""
  # Same-speed inside FD is still a too-close recovery, not a dump.
  assert not lead_mpc_needs_full_authority(0.2, 40.0, slack=-2.0)
  assert lead_inside_slow_close_a_ms2(0.2, -2.0) is None
  # Below the slow-close gate: coast, no rematch +a.
  a_coast = cap_closing_lead_accel(
    0.20, 0.6, a_lead=0.0, lead_present=True, slack=-2.0,
  )
  assert a_coast == pytest.approx(0.0)

  a_e8 = lead_inside_slow_close_a_ms2(1.25, -2.0)
  assert a_e8 == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  assert cap_closing_lead_accel(
    0.20, 1.25, a_lead=0.0, lead_present=True, slack=-2.0,
  ) == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  # Closing ≳ 1.5 inside FD is still slight-lift (#216), not dump.
  assert lead_inside_slow_close_a_ms2(1.6, -2.0) == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  assert not lead_mpc_needs_full_authority(1.6, 30.0, slack=-2.0)
  assert lead_mpc_needs_full_authority(8.0, 40.0, slack=20.0, confirm_rapid=False)
  assert lead_mpc_needs_full_authority(2.0, LEAD_MPC_SOFT_NEAR_M, slack=2.0)


def test_near_gap_small_a_slews_chatter_not_authority():
  """Soft-limit −0.22 ↔ −0.55 near the gap must not step in one frame."""
  stepped = slew_near_gap_small_a(
    -0.55, -LEAD_APPROACH_MILD_A_MS2, 1.4, d_rel=40.0, slack=8.0,
  )
  assert stepped == pytest.approx(-LEAD_APPROACH_MILD_A_MS2 - LEAD_NEAR_GAP_SLEW_MS2)
  assert stepped > -0.40
  # Sign flip rematch → mild also slews through 0.
  flip = slew_near_gap_small_a(
    -0.17, LEAD_CLOSE_OPENING_A_MS2, 0.2, d_rel=40.0, slack=4.0,
  )
  assert flip == pytest.approx(LEAD_CLOSE_OPENING_A_MS2 - LEAD_NEAR_GAP_SLEW_MS2)
  # First onset of mild ease is immediate (do not delay earlier settle).
  assert slew_near_gap_small_a(
    -LEAD_APPROACH_MILD_A_MS2, 0.0, 1.4, d_rel=40.0, slack=8.0,
  ) == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  # Inside FD mild command is immediate (do not delay the settle).
  assert slew_near_gap_small_a(
    -LEAD_APPROACH_MILD_A_MS2, 0.0, 1.25, d_rel=30.0, slack=-2.0,
  ) == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  # Rapid / bumper / FCW stay immediate.
  assert slew_near_gap_small_a(
    -2.0, 0.0, 8.0, d_rel=40.0, slack=8.0, allow_rapid=True,
  ) == pytest.approx(-2.0)
  assert slew_near_gap_small_a(
    -2.0, 0.0, 1.2, d_rel=LEAD_MPC_SOFT_NEAR_M, slack=2.0,
  ) == pytest.approx(-2.0)
  assert slew_near_gap_small_a(
    -2.0, 0.0, 0.2, d_rel=40.0, slack=4.0, fcw=True,
  ) == pytest.approx(-2.0)
  # Far slack: no extra slew (acquire / large-gap path unchanged).
  assert slew_near_gap_small_a(
    -0.46, 0.0, 1.2, d_rel=118.0, slack=80.0,
  ) == pytest.approx(-0.46)


def test_soft_limit_always_mild_on_non_emergency():
  """#216: 1.4↔1.6 flicker near the gap stays at the mild floor."""
  v_ego = 25.0
  d_rel = 40.0
  v_flicker = v_ego - 1.6
  assert soft_limit_mpc_a_target(
    -0.55, v_ego, v_flicker, d_rel, a_lead=0.0, slack=8.0,
  ) == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  assert soft_limit_mpc_a_target(
    -0.55, v_ego, v_ego - 2.0, d_rel, a_lead=0.0, slack=8.0, prev_floored=True,
  ) == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)


def test_settle_gap_bias_firms_last_meters_not_hud_follow():
  """Aim ~3 m long while closing so we are not still at −1.2 m/s at slack=0."""
  assert lead_kinematic_slack_m(20.0, 1.25) == pytest.approx(17.0)
  assert lead_kinematic_slack_m(6.0, 1.25) == pytest.approx(3.0)
  assert lead_kinematic_slack_m(2.0, 1.25) == pytest.approx(0.75)
  # Speeds matched: no bias (glide owns).
  assert lead_kinematic_slack_m(6.0, 0.10) == pytest.approx(6.0)
  t2 = nap_t_follow(2)
  v_lead = 30.0
  d_follow = t2 * v_lead + STOP_DISTANCE
  slack = 6.0
  v_rel = 1.25
  a = lead_approach_decel_ms2(v_lead + v_rel, v_lead, d_follow + slack, t2)
  a_raw = -(v_rel * v_rel) / (2.0 * slack)
  assert a is not None
  assert a <= a_raw + 1e-9
  assert a >= -LEAD_APPROACH_MILD_A_MS2 - 1e-9
