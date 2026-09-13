import pytest

from openpilot.selfdrive.controls.lib.lead_approach import (
  LEAD_APPROACH_A_MS2,
  LEAD_APPROACH_CLEAR_DV_MS,
  LEAD_APPROACH_DV_MS,
  LEAD_APPROACH_DV_OFF_MS,
  LEAD_APPROACH_HEADSTART_S,
  LEAD_APPROACH_MAX_HOLD_M,
  LEAD_APPROACH_MAX_START_M,
  LEAD_APPROACH_MODEL_PROB_MIN,
  LEAD_APPROACH_NEED_HOLD_M,
  LEAD_APPROACH_RELIABLE_M,
  LEAD_APPROACH_SLACK_OFF_M,
  LEAD_APPROACH_SLACK_ON_M,
  LEAD_APPROACH_SLEW_MS2,
  LEAD_CLOSE_A_BASE_MS2,
  LEAD_CLOSE_A_MAX_MS2,
  LEAD_CLOSE_A_MIN_MS2,
  LEAD_CLOSE_MAX_M,
  NAP_T_FOLLOW,
  STOP_DISTANCE,
  lead_approach_decel_ms2,
  lead_approach_need_m,
  lead_approach_track_ok,
  lead_close_accel_ms2,
  lead_close_should_cap,
  nap_t_follow,
  slew_lead_approach_a,
)
from openpilot.selfdrive.mapd.constants import (
  DECREASE_START_MARGIN_M,
  LOOKAHEAD_EARLY,
  LOOKAHEAD_NORMAL,
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
  assert abs(LEAD_APPROACH_CLEAR_DV_MS - 1.05) < 1e-9
  assert abs(LEAD_APPROACH_MODEL_PROB_MIN - 0.50) < 1e-9
  assert abs(LEAD_CLOSE_MAX_M - 140.0) < 1e-9
  assert LEAD_APPROACH_RELIABLE_M < LEAD_APPROACH_MAX_START_M
  assert LEAD_APPROACH_CLEAR_DV_MS > LEAD_APPROACH_DV_MS
  assert LEAD_APPROACH_A_MS2 < 0.80
  assert LEAD_APPROACH_A_MS2 < 1.0
  assert LEAD_APPROACH_A_MS2 < 2.5
  # Tiny comfort tune: raise enter only. Exit stays 0.20 so we still close.
  assert abs(LEAD_APPROACH_DV_MS - 0.55) < 1e-9
  assert abs(LEAD_APPROACH_DV_OFF_MS - 0.20) < 1e-9
  assert LEAD_APPROACH_DV_OFF_MS < LEAD_APPROACH_DV_MS
  assert LEAD_APPROACH_DV_MS > 0.50  # harder rematch re-enter than #121
  assert LEAD_APPROACH_SLACK_OFF_M < LEAD_APPROACH_SLACK_ON_M
  assert LEAD_APPROACH_NEED_HOLD_M > 0.0
  assert LEAD_APPROACH_MAX_HOLD_M > 0.0
  assert abs(LEAD_APPROACH_SLEW_MS2 - 0.05) < 1e-9
  assert abs(LEAD_APPROACH_A_MS2 - 0.55) < 1e-9  # peak unchanged
  import openpilot.selfdrive.controls.lib.lead_approach as lead_approach
  assert not hasattr(lead_approach, "LEAD_APPROACH_MARGIN_M")


def test_lead_approach_eases_before_mpc_comfort_brake_window():
  """Slower lead: relative Early 0.55 + head-start, peak overlay 0.55, not MPC 2.5."""
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
  assert abs(peak + LEAD_APPROACH_A_MS2) < 0.05
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
  assert abs(peak + LEAD_APPROACH_A_MS2) < 0.05


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
  # Past usable Bosch: still off.
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
  assert a1 < 0.25
  assert a10 <= 0.50
  # City / highway cruise clip is the old large-gap punch (1.2 at 10 m/s, 0.8 at 25).
  assert a1 < 1.2 / 3.0
  assert a1 < 0.8 / 2.0
  assert a5 < LEAD_APPROACH_A_MS2
  assert lead_close_should_cap(80.0)
  assert lead_close_should_cap(LEAD_CLOSE_MAX_M)
  assert not lead_close_should_cap(160.0)
  assert not lead_close_should_cap(LEAD_APPROACH_MAX_START_M)
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
  assert a_cap == LEAD_CLOSE_A_MIN_MS2
  assert min_d_rel <= d_follow + 8.0
  assert d_rel <= d_follow + 8.0
  assert d_rel < d_follow + 20.0


def test_lead_approach_hysteresis_holds_through_v_rel_and_slack_noise():
  """Slight-grade follow: ±noise around the old 0.5 / slack=1 gates must not chatter."""
  v_lead = 22.0
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  # Just inside the enter window: closing ~1.1 mph, slack a few meters.
  v_ego = v_lead + LEAD_APPROACH_DV_MS + 0.05
  d_rel = d_follow + 3.0
  assert lead_approach_decel_ms2(v_ego, v_lead, d_rel, t4, active=False) is not None

  # Drop v_rel just below the old enter gate — stay on.
  v_jitter = v_lead + LEAD_APPROACH_DV_MS - 0.08
  assert v_jitter - v_lead > LEAD_APPROACH_DV_OFF_MS
  assert lead_approach_decel_ms2(v_jitter, v_lead, d_rel, t4, active=False) is None
  held = lead_approach_decel_ms2(v_jitter, v_lead, d_rel, t4, active=True)
  assert held is not None and held < 0.0

  # Slack chatters through the old 1.0 m off gate — stay on until at the gap.
  d_near = d_follow + 0.4
  assert lead_approach_decel_ms2(v_ego, v_lead, d_near, t4, active=False) is None
  near = lead_approach_decel_ms2(v_ego, v_lead, d_near, t4, active=True)
  assert near is not None and near < 0.0
  assert abs(near) <= LEAD_APPROACH_A_MS2 + 1e-9

  # Need-edge buffer: a few meters past open stays latched, then drops.
  need = lead_approach_need_m(v_ego, v_lead, t_follow=t4)
  just_out = d_follow + need + 1.5
  assert lead_approach_decel_ms2(v_ego, v_lead, just_out, t4, active=False) is None
  assert lead_approach_decel_ms2(v_ego, v_lead, just_out, t4, active=True) is not None
  far_out = d_follow + need + LEAD_APPROACH_NEED_HOLD_M + 1.0
  assert lead_approach_decel_ms2(v_ego, v_lead, far_out, t4, active=True) is None

  # Matched / opening: always off, even if the previous frame was active.
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
  """Slight-grade rematch: Accel-1 after a 0.20 exit used to re-cross 0.50.

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


def test_lead_approach_slew_softens_onset_and_releases_immediately():
  """Regen onset is gradual; milder / off is not held in regen."""
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

  # Milder (closing speed dropped) and off: no leftover regen.
  assert slew_lead_approach_a(-0.10, -0.40) == pytest.approx(-0.10)
  assert slew_lead_approach_a(None, -0.40) is None


def test_lead_approach_peak_stays_at_early_comfort_not_mpc():
  """Comfort overlay caps at 0.55. Does not own MPC 2.5 / hard brake."""
  v_ego = 26.8
  v_lead = 22.4
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  v_rel = v_ego - v_lead
  rel_need = (v_rel * v_rel) / (2.0 * LEAD_APPROACH_A_MS2)
  peak = lead_approach_decel_ms2(v_ego, v_lead, d_follow + rel_need, t4)
  assert peak is not None
  assert abs(peak + LEAD_APPROACH_A_MS2) < 1e-9
  assert abs(peak) <= 0.55 + 1e-9
  assert abs(peak) < 0.80
  assert abs(peak) < 2.5
  # Tight slack still comfort-capped — MPC may min() harder later.
  tight = lead_approach_decel_ms2(v_ego, v_lead, d_follow + 1.05, t4)
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
  # Missing quality (unit kinematics) is ok; planner always passes both.
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
  # Distance hysteresis: a couple meters past 200 m stays on, then drops.
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
  # A truly faster close still skips need.
  assert lead_approach_decel_ms2(v_ego + 0.2, v_lead, d_follow + 35.0, t4) is not None


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
  # Marginal close (below CLEAR_DV) still uses the need window.
  v_slow = v_lead + LEAD_APPROACH_DV_MS + 0.05
  need_slow = lead_approach_need_m(v_slow, v_lead, t_follow=t4)
  assert lead_approach_decel_ms2(
    v_slow, v_lead, d_follow + need_slow + 20.0, t4,
  ) is None


def test_planner_wires_hysteresis_and_slew_after_map_climb():
  """Overlay stays after map climb / Hill Climb; MPC hard path is still a min()."""
  from pathlib import Path
  planner = (Path(__file__).resolve().parents[1] / "lib/longitudinal_planner.py").read_text()
  assert "active=self._lead_approach_active" in planner
  assert "model_prob=lead.modelProb" in planner
  assert "radar=lead.radar" in planner
  assert "slew_lead_approach_a(a_lead, self._lead_approach_a)" in planner
  assert "min(float(output_a_target), a_lead)" in planner
  assert "map_climb_replaces_mpc" in planner
  hill = (Path(__file__).resolve().parents[1] / "lib/hill_climb.py").read_text()
  assert "PITCH_CLIMB_RAD" in hill
  assert "lead_approach" not in hill or "Caller still" in hill
