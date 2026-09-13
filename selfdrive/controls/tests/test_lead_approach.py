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
  NAP_T_FOLLOW,
  STOP_DISTANCE,
  lead_approach_decel_ms2,
  lead_approach_need_m,
  lead_approach_track_ok,
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
  assert abs(LEAD_APPROACH_CLEAR_DV_MS - 1.0) < 1e-9
  assert abs(LEAD_APPROACH_MODEL_PROB_MIN - 0.50) < 1e-9
  assert LEAD_APPROACH_RELIABLE_M < LEAD_APPROACH_MAX_START_M
  assert LEAD_APPROACH_CLEAR_DV_MS > LEAD_APPROACH_DV_MS
  assert LEAD_APPROACH_A_MS2 < 0.80
  assert LEAD_APPROACH_A_MS2 < 1.0
  assert LEAD_APPROACH_A_MS2 < 2.5
  # Tiny comfort tune: leftover bump-pull was gap rematch, not the 0.55 peak.
  assert abs(LEAD_APPROACH_DV_MS - 0.55) < 1e-9
  assert abs(LEAD_APPROACH_DV_OFF_MS - 0.12) < 1e-9
  assert LEAD_APPROACH_DV_OFF_MS < LEAD_APPROACH_DV_MS
  assert (LEAD_APPROACH_DV_MS - LEAD_APPROACH_DV_OFF_MS) > 0.30  # wider than 0.50/0.20
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

  # Rematch-adjacent leftover chatter after #122: v_rel 0.16 (below old 0.20
  # exit) must stay on; v_rel 0.52 (old enter, below new 0.55) must not re-enter.
  v_hold = v_lead + 0.16
  assert LEAD_APPROACH_DV_OFF_MS < 0.16 < 0.20
  assert lead_approach_decel_ms2(v_hold, v_lead, d_rel, t4, active=False) is None
  assert lead_approach_decel_ms2(v_hold, v_lead, d_rel, t4, active=True) is not None
  v_old_enter = v_lead + 0.52
  assert 0.50 < 0.52 < LEAD_APPROACH_DV_MS
  assert lead_approach_decel_ms2(v_old_enter, v_lead, d_rel, t4, active=False) is None
  assert lead_approach_decel_ms2(v_lead + LEAD_APPROACH_DV_MS + 0.01, v_lead, d_rel, t4, active=False) is not None
  assert lead_approach_decel_ms2(v_lead + LEAD_APPROACH_DV_OFF_MS - 0.02, v_lead, d_rel, t4, active=True) is None


def test_lead_approach_gap_edge_rematch_does_not_chatter():
  """Slight-grade rematch: rematch after a 0.20 exit used to re-cross 0.50.

  Live band (0.55 / 0.12) holds through rematch-adjacent v_rel and does not
  re-enter at the old 0.50 gate. Slack-off / matched still drop so we close.
  """
  v_lead = 22.0
  t4 = nap_t_follow(4)
  d_follow = t4 * v_lead + STOP_DISTANCE
  d_rel = d_follow + 3.0

  v_on = v_lead + LEAD_APPROACH_DV_MS + 0.05
  assert lead_approach_decel_ms2(v_on, v_lead, d_rel, t4, active=False) is not None

  # Hold through the old 0.20 exit (rematch used to punch here).
  for dv in (0.19, 0.16, 0.14, 0.13):
    assert LEAD_APPROACH_DV_OFF_MS < dv < 0.20
    assert lead_approach_decel_ms2(v_lead + dv, v_lead, d_rel, t4, active=True) is not None

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
  frames = 0
  for frames in range(1, 20):
    prev = slew_lead_approach_a(target, prev)
    assert prev is not None
    assert prev >= target - 1e-9
    if abs(prev - target) < 1e-9:
      break
  else:
    raise AssertionError("slew did not reach comfort peak")
  assert abs(prev + LEAD_APPROACH_A_MS2) < 1e-9
  assert frames == int(round(LEAD_APPROACH_A_MS2 / LEAD_APPROACH_SLEW_MS2))

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


def test_planner_wires_hysteresis_and_slew():
  """Overlay stays after map track; MPC hard path is still a min()."""
  from pathlib import Path
  planner = (Path(__file__).resolve().parents[1] / "lib/longitudinal_planner.py").read_text()
  assert "active=self._lead_approach_active" in planner
  assert "model_prob=lead.modelProb" in planner
  assert "radar=lead.radar" in planner
  assert "slew_lead_approach_a(a_lead, self._lead_approach_a)" in planner
  assert "min(float(output_a_target), a_lead)" in planner
  assert "hypermile" not in planner.lower()
  assert "hill_climb" not in planner.lower()
