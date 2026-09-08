from openpilot.selfdrive.controls.lib.lead_approach import (
  LEAD_APPROACH_A_MS2,
  NAP_T_FOLLOW,
  STOP_DISTANCE,
  lead_approach_decel_ms2,
  nap_t_follow,
)


def test_nap_t_follow_matches_follow_distance_slider():
  assert nap_t_follow(4) == 1.3
  assert nap_t_follow(1) == 0.7
  assert nap_t_follow(7) == 1.9
  assert nap_t_follow(None) is None
  assert nap_t_follow(0) is None
  assert list(NAP_T_FOLLOW) == [0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9]


def test_lead_approach_eases_before_mpc_comfort_brake_window():
  """Slower lead: start at 1.0 m/s² + Follow Distance, not MPC's late 2.5 m/s²."""
  v_ego = 60.0 * 0.44704
  v_lead = 50.0 * 0.44704
  t4 = nap_t_follow(4)
  t7 = nap_t_follow(7)
  assert abs(LEAD_APPROACH_A_MS2 - 1.0) < 1e-9
  d_follow = t4 * v_lead + STOP_DISTANCE
  need = (v_ego * v_ego - v_lead * v_lead) / (2.0 * LEAD_APPROACH_A_MS2)
  mpc_need = (v_ego * v_ego - v_lead * v_lead) / (2.0 * 2.5)
  assert need > mpc_need + 40.0
  far = d_follow + need + 15.0
  assert lead_approach_decel_ms2(v_ego, v_lead, far, t4) is None
  at_open = lead_approach_decel_ms2(v_ego, v_lead, d_follow + need - 1.0, t4)
  assert at_open is not None
  assert abs(at_open + LEAD_APPROACH_A_MS2) < 0.05
  mid = lead_approach_decel_ms2(v_ego, v_lead, d_follow + 0.45 * need, t4)
  assert mid is not None and -LEAD_APPROACH_A_MS2 <= mid < 0.0
  assert lead_approach_decel_ms2(v_ego, v_ego, 80.0, t4) is None
  assert lead_approach_decel_ms2(v_ego, v_ego + 2.0, 80.0, t4) is None
  d_open_4 = d_follow + need
  d_open_7 = t7 * v_lead + STOP_DISTANCE + (v_ego * v_ego - v_lead * v_lead) / (2.0 * LEAD_APPROACH_A_MS2)
  assert d_open_7 > d_open_4 + 5.0
  assert lead_approach_decel_ms2(v_ego, v_lead, d_open_4 + 3.0, t7) is not None
  assert lead_approach_decel_ms2(v_ego, v_lead, d_open_4 + 3.0, t4) is None


def test_t_follow_table_stays_in_sync_with_mpc():
  from pathlib import Path
  mpc = (Path(__file__).resolve().parents[1] / "lib/longitudinal_mpc_lib/long_mpc.py").read_text()
  assert "NAP_T_FOLLOW = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)" in mpc
  assert "STOP_DISTANCE = 6.0" in mpc


def test_stopped_lead_still_plans_a_comfortable_stop_gap():
  v_ego = 20.0
  t4 = nap_t_follow(4)
  need = (v_ego * v_ego) / (2.0 * LEAD_APPROACH_A_MS2)
  d_follow = STOP_DISTANCE
  a = lead_approach_decel_ms2(v_ego, 0.0, d_follow + 0.5 * need, t4)
  assert a is not None and a < 0.0
  assert lead_approach_decel_ms2(v_ego, 0.0, d_follow + need + 20.0, t4) is None
