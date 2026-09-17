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
  LEAD_APPROACH_RAPID_DV_MS,
  LEAD_APPROACH_RAPID_TTC_S,
  LEAD_APPROACH_RELEASE_SLEW_MS2,
  LEAD_APPROACH_RELIABLE_M,
  LEAD_APPROACH_SLACK_OFF_M,
  LEAD_APPROACH_SLACK_ON_M,
  LEAD_APPROACH_SLEW_MS2,
  LEAD_APPROACH_TTC_START_S,
  LEAD_CLOSE_A_BASE_MS2,
  LEAD_CLOSE_A_MAX_MS2,
  LEAD_CLOSE_A_MIN_MS2,
  LEAD_CLOSE_MAX_M,
  LEAD_CLOSE_OPENING_A_MS2,
  LEAD_CLOSE_REMATCH_A_MS2,
  LEAD_CLOSE_REMATCH_SLACK_M,
  LEAD_CLOSE_HOLD_S,
  NAP_T_FOLLOW,
  STOP_DISTANCE,
  apply_lead_approach_overlay,
  lead_approach_decel_ms2,
  lead_approach_is_rapid,
  lead_approach_need_m,
  lead_approach_track_ok,
  lead_approach_ttc_s,
  lead_close_accel_ms2,
  lead_close_should_cap,
  lead_follow_slack_m,
  nap_t_follow,
  resolve_lead_close_hold,
  slew_lead_approach_a,
)
from openpilot.selfdrive.mapd.constants import (
  DECREASE_START_MARGIN_M,
  LOOKAHEAD_EARLY,
  LOOKAHEAD_NORMAL,
  map_accel_a_ms2,
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
  assert abs(LEAD_APPROACH_CLEAR_DV_MS - 2.5) < 1e-9
  assert abs(LEAD_APPROACH_MODEL_PROB_MIN - 0.50) < 1e-9
  assert LEAD_APPROACH_RELIABLE_M < LEAD_APPROACH_MAX_START_M
  assert LEAD_APPROACH_CLEAR_DV_MS > LEAD_APPROACH_DV_MS
  assert LEAD_APPROACH_A_MS2 < 0.80
  assert LEAD_APPROACH_A_MS2 < 1.0
  assert LEAD_APPROACH_A_MS2 < 2.5
  # Tiny comfort tune: raise enter only. Exit stays 0.20 so we still close.
  assert abs(LEAD_APPROACH_DV_MS - 0.65) < 1e-9
  assert abs(LEAD_APPROACH_DV_OFF_MS - 0.20) < 1e-9
  assert LEAD_APPROACH_DV_OFF_MS < LEAD_APPROACH_DV_MS
  assert LEAD_APPROACH_DV_MS > 0.50  # harder rematch re-enter than #122
  assert LEAD_APPROACH_SLACK_OFF_M < LEAD_APPROACH_SLACK_ON_M
  assert LEAD_APPROACH_NEED_HOLD_M > 0.0
  assert LEAD_APPROACH_MAX_HOLD_M > 0.0
  assert abs(LEAD_APPROACH_SLEW_MS2 - 0.03) < 1e-9
  assert abs(LEAD_APPROACH_RELEASE_SLEW_MS2 - 0.012) < 1e-9
  assert LEAD_APPROACH_RELEASE_SLEW_MS2 < LEAD_APPROACH_SLEW_MS2
  assert abs(LEAD_APPROACH_NIBBLE_MS2 - 0.15) < 1e-9
  assert LEAD_APPROACH_NIBBLE_MS2 > 0.13  # covers matching-traffic |a|
  assert abs(LEAD_APPROACH_A_MS2 - 0.55) < 1e-9  # rapid peak unchanged
  assert abs(LEAD_APPROACH_MILD_A_MS2 - 0.18) < 1e-9
  assert LEAD_APPROACH_MILD_A_MS2 < LEAD_APPROACH_A_MS2
  assert LEAD_APPROACH_MILD_A_MS2 > LEAD_APPROACH_NIBBLE_MS2
  assert abs(LEAD_APPROACH_RAPID_DV_MS - 6.0) < 1e-9
  assert abs(LEAD_APPROACH_TTC_START_S - 20.0) < 1e-9
  assert abs(LEAD_APPROACH_RAPID_TTC_S - 8.0) < 1e-9
  assert LEAD_APPROACH_RAPID_DV_MS > LEAD_APPROACH_CLEAR_DV_MS
  assert abs(LEAD_CLOSE_OPENING_A_MS2 - 0.05) < 1e-9
  assert abs(LEAD_CLOSE_REMATCH_A_MS2 - 0.08) < 1e-9
  assert LEAD_CLOSE_OPENING_A_MS2 < LEAD_CLOSE_REMATCH_A_MS2 < LEAD_CLOSE_A_MIN_MS2
  assert LEAD_CLOSE_REMATCH_SLACK_M > 12.0
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


def test_lead_close_accel_is_well_below_cruise_and_scales_with_accel():
  """Accel 1–10 gates catch-up +a. Min Accel is a nudge, not cruise 1.6–0.6."""
  a1 = lead_close_accel_ms2(1)
  a5 = lead_close_accel_ms2(5)
  a10 = lead_close_accel_ms2(10)
  assert abs(a1 - LEAD_CLOSE_A_MIN_MS2) < 1e-9
  assert abs(a5 - LEAD_CLOSE_A_BASE_MS2) < 1e-9
  assert abs(a10 - LEAD_CLOSE_A_MAX_MS2) < 1e-9
  assert a1 < a5 < a10
  assert a1 < 0.15
  assert a10 <= 0.30
  # City / highway cruise clip is the old large-gap punch (1.2 at 10 m/s, 0.8 at 25).
  assert a1 < 1.2 / 3.0
  assert a1 < 0.8 / 2.0
  assert a5 < LEAD_APPROACH_A_MS2
  assert lead_close_should_cap(80.0)
  assert lead_close_should_cap(LEAD_CLOSE_MAX_M)
  # Old 140 m hole: a 160–200 m lead still counted as open-road cruise punch.
  assert lead_close_should_cap(160.0)
  assert lead_close_should_cap(LEAD_APPROACH_MAX_START_M)
  assert not lead_close_should_cap(LEAD_APPROACH_MAX_START_M + LEAD_APPROACH_MAX_HOLD_M + 1.0)
  assert not lead_close_should_cap(0.0)
  assert not lead_close_should_cap(None)
  assert not lead_close_should_cap(160.0, model_prob=0.2, radar=False)
  assert lead_close_should_cap(160.0, model_prob=1.0, radar=True)


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
  assert a_cap == LEAD_CLOSE_A_MIN_MS2
  assert min_d_rel <= d_follow + 8.0
  assert d_rel <= d_follow + 8.0
  assert d_rel < d_follow + 20.0


def test_lead_close_accel_never_exceeds_mannerisms_personality():
  """Catch-up +a ≤ Accel 1–10 open-road. Not a higher punch profile."""
  for accel in range(1, 11):
    personality = map_accel_a_ms2(LOOKAHEAD_NORMAL, accel)
    a = lead_close_accel_ms2(accel, v_rel=0.0, slack=80.0, a_personality=personality)
    assert a <= personality + 1e-9
    assert a <= lead_close_accel_ms2(accel) + 1e-9
    assert a < 0.30
  # Explicit ceiling: a lower personality wins over the catch-up curve.
  assert lead_close_accel_ms2(10, a_personality=0.10) == pytest.approx(0.10)
  assert lead_close_accel_ms2(1, a_personality=0.36) == pytest.approx(LEAD_CLOSE_A_MIN_MS2)
  # Slack close stays on the gentler curve, not MAX-rise 0.36–1.60.
  assert lead_close_accel_ms2(1, v_rel=0.0, slack=80.0) < map_accel_a_ms2(LOOKAHEAD_NORMAL, 1)
  assert lead_close_accel_ms2(5, v_rel=0.0, slack=80.0) < map_accel_a_ms2(LOOKAHEAD_NORMAL, 5)


def test_large_gap_lead_closes_gradually_without_cruise_punch():
  """Lead a ways out (past old 140 m): +a stays ≤ Accel 1, slack still shrinks."""
  v_ego = 22.0
  v_lead = 22.0
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  d_rel = 180.0
  slack0 = d_rel - d_follow
  assert slack0 > 100.0
  assert d_rel > 140.0
  assert lead_close_should_cap(d_rel, model_prob=1.0, radar=True)
  personality = map_accel_a_ms2(LOOKAHEAD_NORMAL, 1)
  a_cap = lead_close_accel_ms2(1, v_rel=0.0, slack=slack0, a_personality=personality)
  assert a_cap == pytest.approx(LEAD_CLOSE_A_MIN_MS2)
  assert a_cap <= personality + 1e-9
  assert a_cap < 0.20
  dt = 0.05
  max_plus_a = 0.0
  for _ in range(int(25.0 / dt)):
    slack = d_rel - d_follow
    a = lead_close_accel_ms2(1, v_rel=v_ego - v_lead, slack=slack, a_personality=personality)
    assert a <= personality + 1e-9
    assert a <= LEAD_CLOSE_A_MIN_MS2 + 1e-9
    decel = lead_approach_decel_ms2(v_ego, v_lead, d_rel, t4, model_prob=1.0, radar=True)
    if decel is not None:
      a = min(a, decel)
    if a > 0.0:
      max_plus_a = max(max_plus_a, a)
    v_ego = max(0.0, v_ego + a * dt)
    d_rel -= (v_ego - v_lead) * dt
  slack1 = d_rel - d_follow
  assert slack1 < slack0 - 10.0
  assert max_plus_a == pytest.approx(LEAD_CLOSE_A_MIN_MS2, abs=1e-9)
  assert max_plus_a <= personality + 1e-9
  assert max_plus_a < 0.20


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

  Enter 0.65 blocks that re-bite. Exit stays 0.20 so we still drop and close.
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
  for n in range(1, 40):
    prev = slew_lead_approach_a(target, prev)
    assert prev is not None
    assert prev >= target - 1e-9
    reached = n
    if abs(prev - target) < 1e-9:
      break
  else:
    raise AssertionError("slew did not reach comfort peak")
  assert abs(prev + LEAD_APPROACH_A_MS2) < 1e-9
  assert reached >= int(LEAD_APPROACH_A_MS2 / LEAD_APPROACH_SLEW_MS2)

  assert slew_lead_approach_a(-0.10, -0.40) == pytest.approx(-0.40 + LEAD_APPROACH_RELEASE_SLEW_MS2)
  released = slew_lead_approach_a(None, -0.40)
  assert released == pytest.approx(-0.40 + LEAD_APPROACH_RELEASE_SLEW_MS2)
  assert released is not None and released < 0.0
  fading = -0.40
  frames = 0
  while fading is not None:
    fading = slew_lead_approach_a(None, fading)
    frames += 1
    if frames > 80:
      raise AssertionError("release slew did not reach off")
  assert frames >= int(0.40 / LEAD_APPROACH_RELEASE_SLEW_MS2)


def test_lead_approach_peak_stays_at_early_comfort_not_mpc():
  """Rapid overlay caps at 0.55. Mild stays at 0.18. Does not own MPC 2.5."""
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
  peak = lead_approach_decel_ms2(v_ego, v_lead, d_follow + rel_need, t4)
  assert peak is not None
  assert abs(peak + LEAD_APPROACH_A_MS2) < 1e-9
  assert abs(peak) <= 0.55 + 1e-9
  assert abs(peak) < 0.80
  assert abs(peak) < 2.5
  tight = lead_approach_decel_ms2(v_ego, v_lead, d_follow + 1.05, t4)
  assert tight is not None
  assert tight == pytest.approx(-LEAD_APPROACH_A_MS2)


def test_far_flicker_rejected_without_radar_or_model_prob():
  """Beyond 140 m: vision-only / low modelProb stay off. Radar+prob enters."""
  v_ego = 60.0 * 0.44704
  v_lead = 50.0 * 0.44704
  t4 = nap_t_follow(4)
  d_far = 180.0
  assert d_far > LEAD_APPROACH_RELIABLE_M
  assert lead_approach_track_ok(80.0, model_prob=0.0, radar=False) is True
  assert lead_approach_track_ok(d_far, model_prob=0.2, radar=True) is False
  assert lead_approach_track_ok(d_far, model_prob=1.0, radar=False) is False
  assert lead_approach_track_ok(d_far, model_prob=1.0, radar=True) is True
  assert lead_approach_track_ok(d_far) is True
  assert lead_approach_decel_ms2(
    v_ego, v_lead, d_far, t4, model_prob=0.2, radar=True,
  ) is None
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


def test_nibble_overlay_does_not_steal_catchup_plus_a():
  """Far/gentle overlay must not beat rematch +a. Real ease / MPC 0/−a still min()."""
  a_min = LEAD_CLOSE_A_MIN_MS2
  assert apply_lead_approach_overlay(a_min, -0.05) == pytest.approx(a_min)
  assert apply_lead_approach_overlay(a_min, -0.13) == pytest.approx(a_min)
  assert apply_lead_approach_overlay(a_min, -LEAD_APPROACH_NIBBLE_MS2) == pytest.approx(-LEAD_APPROACH_NIBBLE_MS2)
  assert apply_lead_approach_overlay(a_min, -0.20) == pytest.approx(-0.20)
  assert apply_lead_approach_overlay(0.0, -0.05) == pytest.approx(-0.05)
  assert apply_lead_approach_overlay(-0.30, -0.05) == pytest.approx(-0.30)
  assert apply_lead_approach_overlay(-0.10, -0.20) == pytest.approx(-0.20)
  assert apply_lead_approach_overlay(a_min, None) == pytest.approx(a_min)
  # Large-gap rematch still keeps catch-up. A real close eases off throttle.
  assert apply_lead_approach_overlay(a_min, -0.08, v_rel=0.3, slack=40.0) == pytest.approx(a_min)
  assert apply_lead_approach_overlay(a_min, -0.08, v_rel=4.0, slack=40.0) == pytest.approx(0.0)
  # Near the follow gap, a fading nibble still min()s (no Accel slam).
  assert apply_lead_approach_overlay(a_min, -0.08, v_rel=0.1, slack=3.0) == pytest.approx(-0.08)


def test_planner_wires_hysteresis_and_slew():
  """Overlay stays after map track; MPC hard path is still a min()."""
  from pathlib import Path
  planner = (Path(__file__).resolve().parents[1] / "lib/longitudinal_planner.py").read_text()
  assert "active=self._lead_approach_active" in planner
  assert "model_prob=lead.modelProb" in planner
  assert "radar=lead.radar" in planner
  assert "slew_lead_approach_a(a_lead, self._lead_approach_a)" in planner
  assert "apply_lead_approach_overlay(" in planner
  assert "v_rel=overlay_v_rel" in planner
  assert "lead_follow_slack_m(" in planner
  assert "resolve_lead_close_hold(" in planner
  assert "lead_close_accel_ms2(" in planner
  assert "a_personality=a_personality" in planner
  assert "model_prob=lead_close.modelProb" in planner
  assert "radar=lead_close.radar" in planner
  assert "self._lead_close_a_cap" in planner
  assert "self._lead_approach_active = a_lead is not None" in planner
  slew_at = planner.find("self.prev_accel_clip[idx] - 0.05")
  recap_at = planner.find("min(float(accel_clip[1]), float(self._lead_close_a_cap))")
  assert 0 <= slew_at < recap_at
  assert "hypermile" not in planner.lower()
  assert "hill_climb" not in planner.lower()


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
  a_kin = -(2.24 * 2.24) / (2.0 * slack)
  assert a_slow is not None
  assert a_slow == pytest.approx(max(a_kin, -LEAD_APPROACH_MILD_A_MS2), abs=1e-6)
  assert abs(a_slow) < LEAD_APPROACH_A_MS2 - 0.20


def test_rapid_close_allows_stronger_early_decel():
  """High closing-rate dump: kinematics up to 0.55, above the mild ceiling."""
  v_ego = 70.0 * 0.44704
  v_lead = 50.0 * 0.44704
  v_rel = v_ego - v_lead
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  assert v_rel >= LEAD_APPROACH_RAPID_DV_MS
  assert lead_approach_is_rapid(v_rel)

  a_far = lead_approach_decel_ms2(v_ego, v_lead, 180.0, t4, model_prob=1.0, radar=True)
  slack_far = 180.0 - d_follow
  a_kin = -(v_rel * v_rel) / (2.0 * slack_far)
  assert a_far == pytest.approx(max(a_kin, -LEAD_APPROACH_A_MS2), abs=1e-6)
  assert abs(a_far) > LEAD_APPROACH_MILD_A_MS2 or abs(a_kin) <= LEAD_APPROACH_MILD_A_MS2

  d_dump = d_follow + v_rel * LEAD_APPROACH_RAPID_TTC_S
  a_dump = lead_approach_decel_ms2(v_ego, v_lead, d_dump, t4, model_prob=1.0, radar=True)
  assert a_dump == pytest.approx(-LEAD_APPROACH_A_MS2, abs=0.08)
  assert abs(a_dump) > LEAD_APPROACH_MILD_A_MS2 + 0.20


def test_gap_opening_rematch_is_a_trickle():
  """When the gap is opening / just rematching near Follow Distance: small +a.

  Large-gap catch-up (lead-close cap) is 0.12/0.18/0.28.
  """
  assert lead_close_accel_ms2(1) == pytest.approx(LEAD_CLOSE_A_MIN_MS2)
  assert lead_close_accel_ms2(1, v_rel=0.0, slack=40.0) == pytest.approx(LEAD_CLOSE_A_MIN_MS2)
  assert lead_close_accel_ms2(5, v_rel=0.0, slack=40.0) == pytest.approx(LEAD_CLOSE_A_BASE_MS2)

  a_open = lead_close_accel_ms2(1, v_rel=-0.4, slack=3.0)
  assert a_open == pytest.approx(LEAD_CLOSE_OPENING_A_MS2)
  assert a_open < 0.07
  a_open_5 = lead_close_accel_ms2(5, v_rel=-0.2, slack=2.0)
  assert a_open_5 == pytest.approx(LEAD_CLOSE_OPENING_A_MS2)

  a_near = lead_close_accel_ms2(1, v_rel=0.30, slack=4.0)
  assert a_near == pytest.approx(LEAD_CLOSE_REMATCH_A_MS2)
  assert a_near < LEAD_CLOSE_A_MIN_MS2
  # Strong close near the gap keeps the normal catch-up cap.
  a_closing = lead_close_accel_ms2(1, v_rel=2.0, slack=4.0)
  assert a_closing == pytest.approx(LEAD_CLOSE_A_MIN_MS2)

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
  mid_slack = lead_follow_slack_m(d_follow + 15.0, v_lead, t4)
  assert mid_slack == pytest.approx(15.0)
  assert mid_slack < LEAD_CLOSE_REMATCH_SLACK_M
  a_mid = lead_close_accel_ms2(1, v_rel=-0.2, slack=mid_slack)
  assert a_mid == pytest.approx(LEAD_CLOSE_OPENING_A_MS2)


def test_lead_close_hold_keeps_cap_through_status_flicker():
  """Dropped leadOne.status keeps the last in-window lead; a valid far lead does not."""
  d_live, v_live, held_d, held_v, age = resolve_lead_close_hold(
    True, 80.0, 22.0, None, None, 0.0, 0.05,
  )
  assert d_live == pytest.approx(80.0)
  assert v_live == pytest.approx(22.0)
  assert held_d == pytest.approx(80.0)
  assert age == pytest.approx(0.0)

  d_use, v_use, held_d, held_v, age = resolve_lead_close_hold(
    False, 80.0, 22.0, held_d, held_v, age, 0.05,
  )
  assert d_use == pytest.approx(80.0)
  assert v_use == pytest.approx(22.0)
  assert age == pytest.approx(0.05)

  # Hold through several planner frames, then expire.
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
  assert age == pytest.approx(0.0)

  # Far Bosch radar lead stays capped — that was the cruise punch at 160 m.
  d_use, v_use, held_d, held_v, age = resolve_lead_close_hold(
    True, 160.0, 22.0, 80.0, 22.0, 0.10, 0.05, model_prob=1.0, radar=True,
  )
  assert d_use == pytest.approx(160.0)
  assert v_use == pytest.approx(22.0)
  assert lead_close_should_cap(160.0, model_prob=1.0, radar=True)
  assert lead_close_should_cap(80.0)

  # Vision-only far flicker does not start a cap (empty-road MAX-rise).
  d_use, v_use, held_d, held_v, age = resolve_lead_close_hold(
    True, 160.0, 22.0, None, None, 0.0, 0.05, model_prob=0.2, radar=False,
  )
  assert d_use is None
  assert held_d is None
  assert not lead_close_should_cap(160.0, model_prob=0.2, radar=False)

  # Past usable Bosch: drop the hold so MAX-rise can climb.
  d_use, v_use, held_d, held_v, age = resolve_lead_close_hold(
    True, 220.0, 22.0, 80.0, 22.0, 0.10, 0.05, model_prob=1.0, radar=True,
  )
  assert d_use is None
  assert held_d is None
  assert not lead_close_should_cap(220.0, model_prob=1.0, radar=True)
