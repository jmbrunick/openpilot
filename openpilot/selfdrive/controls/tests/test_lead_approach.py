import pytest
from openpilot.selfdrive.controls.lib.lead_approach import (
  LEAD_APPROACH_MILD_A_MS2,
  LEAD_CLOSE_HOLD_S,
  LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2,
  NAP_T_FOLLOW,
  STOP_DISTANCE,
  LeadResidualWindow,
  lead_approach_track_ok,
  guard_follow_actuator_regen,
  plant_regen_effort_limits,
  lead_close_should_cap,
  lead_follow_slack_m,
  nap_t_follow,
  resolve_lead_close_hold,

)
from openpilot.selfdrive.mapd.constants import (
  LOOKAHEAD_EARLY,
)


def test_nap_t_follow_matches_follow_distance_slider():
  assert nap_t_follow(4) == 1.3
  assert nap_t_follow(1) == 0.7
  assert nap_t_follow(7) == 1.9
  assert nap_t_follow(None) is None
  assert nap_t_follow(0) is None
  assert list(NAP_T_FOLLOW) == [0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9]


def test_t_follow_table_stays_in_sync_with_mpc():
  from pathlib import Path
  mpc = (Path(__file__).resolve().parents[1] / "lib/longitudinal_mpc_lib/long_mpc.py").read_text()
  assert "NAP_T_FOLLOW = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)" in mpc
  assert "STOP_DISTANCE = 6.0" in mpc


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


def test_guard_follow_actuator_regen_when_planner_near_zero():
  """ef 10:18:42: aTarget ≈ 0 must not dump plant regen to −1.2.

  A steady command also must not open the mild settle. That settle is
  for a command that is already there.
  """
  # Planner coasting: clip firm regen to the command, not to −0.22.
  assert guard_follow_actuator_regen(-1.23, 0.0) == pytest.approx(0.0)
  assert guard_follow_actuator_regen(-1.50, 0.02) == pytest.approx(0.0)
  assert guard_follow_actuator_regen(-0.10, 0.0) == pytest.approx(0.0)
  assert guard_follow_actuator_regen(0.0, 0.0) == pytest.approx(0.0)
  # Planner asked for firm / rapid / FCW −a: full authority.
  assert guard_follow_actuator_regen(-1.23, -0.50) == pytest.approx(-1.23)
  assert guard_follow_actuator_regen(-2.0, -2.0) == pytest.approx(-2.0)
  assert guard_follow_actuator_regen(-1.50, -0.80) == pytest.approx(-1.50)
  # Commanded MILD is inside the steady band: plant cannot full-lift.
  assert guard_follow_actuator_regen(-0.22, -0.22) == pytest.approx(-0.22)
  assert guard_follow_actuator_regen(-0.40, -0.22) == pytest.approx(
    LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2
  )
  assert guard_follow_actuator_regen(-1.32, -0.218) == pytest.approx(
    LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2
  )
  # LongControl only applies this on Pre-AP PID, not stopping.
  from pathlib import Path
  longcontrol = (Path(__file__).resolve().parents[1] / "lib/longcontrol.py").read_text()
  assert "guard_follow_actuator_regen(" in longcontrol
  assert "LongCtrlState.pid" in longcontrol
  assert "TESLA_MODEL_S_PREAP" in longcontrol


def test_card_does_not_subscribe_to_controlsstate_for_curve():
  """Curve MAX curvature comes from carControl, not a 100 Hz controlsState poll."""
  from pathlib import Path
  card = (Path(__file__).resolve().parents[3] / "selfdrive/car/card.py").read_text()
  sm = card.split("messaging.SubMaster([", 1)[1].split("])", 1)[0]
  assert "controlsState" not in sm
  assert "currentCurvature" in card


def test_descent_plant_effort_keeps_grade_flat_and_uphill_do_not():
  """A descent −0.22 effort includes the grade term. Flat and uphill do not move."""
  import math

  from opendbc.car.tesla.preap.virtual_das import GRAVITY, VirtualDAS

  from openpilot.selfdrive.controls.lib.lead_approach import (
    install_preap_plant_regen_guard,
  )

  grade = -0.42
  down = plant_regen_effort_limits(
    -0.22, (-1.5, 2.0), steady_grade=grade, transient=0.0, descent=True,
  )
  flat = plant_regen_effort_limits(
    -0.22, (-1.5, 2.0), steady_grade=0.0, transient=0.0, descent=False,
  )
  up = plant_regen_effort_limits(
    -0.22, (-1.5, 2.0), steady_grade=0.27, transient=0.0, descent=False,
  )
  assert down[0] == pytest.approx(-0.22 + grade)
  assert flat[0] == pytest.approx(-0.22)
  assert up[0] == pytest.approx(-0.22)
  assert up[0] == flat[0]

  def raw_update():
    update = VirtualDAS.update
    while getattr(update, "_nap_plant_regen_guard", False):
      update = update._nap_plant_regen_raw
    return update

  before = raw_update()
  install_preap_plant_regen_guard()
  try:
    def settle(update, pitch, a_ego, a_cmd=-0.22, limits=None):
      vdas = VirtualDAS(dt=0.02)
      ori = [0.0, pitch, 0.0]
      for _ in range(400):
        vdas.observe(a_ego=0.0, orientation_ned=ori)
      vdas.reset(
        measured_accel=a_ego, commanded_accel=a_cmd, pedal_di_init=8.0,
        preserve_grade=True,
      )
      pedal = 8.0
      for _ in range(80):
        pedal = update(
          vdas, a_cmd, 30.0, pedal, a_ego=a_ego, orientation_ned=ori,
          accel_effort_limits=limits,
        )
      return vdas.prev_accel_effort, vdas.grade_estimator._steady_grade_compensation()

    guarded = VirtualDAS.update
    raw = raw_update()
    # Previous floor: mild command, no descent allowance. Flat and uphill
    # must still land on that effort. aEgo tracks, so the shortfall gate
    # stays shut.
    old_limits = plant_regen_effort_limits(-0.22, (-1.5, 2.0), descent=False)
    flat_g, _flat_grade = settle(guarded, 0.02, -0.22)
    flat_old, _ = settle(raw, 0.02, -0.22, limits=old_limits)
    assert flat_g == pytest.approx(flat_old, abs=0.02)

    up_pitch = 0.02 + math.asin(0.27 / GRAVITY)
    up_g, up_grade = settle(guarded, up_pitch, -0.22)
    up_old, _ = settle(raw, up_pitch, -0.22, limits=old_limits)
    assert up_g == pytest.approx(up_old, abs=0.02)
    assert up_grade > 0.2

    down_pitch = math.asin(grade / GRAVITY)
    down_g, down_grade = settle(guarded, down_pitch, 0.05)
    down_old, _ = settle(raw, down_pitch, 0.05, limits=old_limits)
    assert down_grade < -0.3
    assert down_old == pytest.approx(-0.22, abs=0.05)
    assert down_g <= -0.22 + down_grade + 0.05
    assert down_g < down_old - 0.15
  finally:
    VirtualDAS.update = before



def test_lead_close_should_cap_and_track_gate():
  assert lead_close_should_cap(80.0)
  assert lead_close_should_cap(160.0, model_prob=1.0, radar=True)
  assert lead_close_should_cap(160.0, model_prob=0.2, radar=True)
  assert not lead_close_should_cap(160.0, model_prob=0.2, radar=False)
  assert not lead_close_should_cap(0.0)
  assert not lead_close_should_cap(None)
  d_far = 180.0
  assert lead_approach_track_ok(80.0, model_prob=0.0, radar=False) is True
  assert lead_approach_track_ok(d_far, model_prob=0.2, radar=True) is True
  assert lead_approach_track_ok(d_far, model_prob=1.0, radar=False) is False
  assert lead_approach_track_ok(d_far, model_prob=1.0, radar=True) is True
  assert lead_approach_track_ok(d_far) is True


def test_lead_follow_slack_is_gap_minus_follow_distance():
  t4 = nap_t_follow(4)
  v_lead = 22.0
  d_follow = t4 * v_lead + STOP_DISTANCE
  assert lead_follow_slack_m(d_follow + 40.0, v_lead, t4) == pytest.approx(40.0)
  assert lead_follow_slack_m(d_follow + 5.0, v_lead, t4) == pytest.approx(5.0)


def test_steady_plant_does_not_leak_into_mild_without_a_dwell():
  """23:16 / 23:22: a steady command must not settle at −0.22.

  The mild settle still applies once the command has dwelled there, and a
  brief return to coast does not drop it. A firm brake is not mild.
  """
  from openpilot.selfdrive.controls.lib.lead_approach import (
    LEAD_FOLLOW_STEADY_ONLY_MS2,
    PlantDecelClassifier,
    plant_follow_floor,
    plant_regen_effort_limits,
  )

  clf = PlantDecelClassifier()
  for _ in range(30):
    mode = clf.update(0.0, 0.02)
  assert mode == "steady"
  assert plant_regen_effort_limits(0.0, (-1.5, 2.0), decel_mode=mode)[0] == pytest.approx(0.0)
  for _ in range(10):
    mode = clf.update(-0.22, 0.02)
  assert mode == "steady"
  assert plant_follow_floor(-0.22, mode) == pytest.approx(-LEAD_FOLLOW_STEADY_ONLY_MS2)
  for _ in range(20):
    mode = clf.update(-0.22, 0.02)
  assert mode == "mild"
  assert plant_follow_floor(-0.22, mode) == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  held = plant_regen_effort_limits(-0.22, (-1.5, 2.0), decel_mode=mode)
  assert held[0] == pytest.approx(-0.22)
  for _ in range(10):
    mode = clf.update(0.0, 0.02)
  assert mode == "mild"
  assert plant_follow_floor(0.0, mode) == pytest.approx(-0.22)
  for _ in range(25):
    mode = clf.update(0.01, 0.02)
  assert mode == "steady"

  firm = PlantDecelClassifier()
  for _ in range(40):
    mode = firm.update(-1.4, 0.02)
  assert mode == "steady"
  # A sustained mild sample, with no classifier, is still the settle.
  assert plant_regen_effort_limits(-0.22, (-1.5, 2.0))[0] == pytest.approx(-0.22)
  assert plant_regen_effort_limits(0.008, (-1.5, 2.0))[0] == pytest.approx(0.0)


def test_residual_window_ignores_one_radar_lsb_and_arms_on_sustained_close():
  """One vRel quantum over a 0.05 s frame must not arm; a sustained 07:55-shaped close must."""
  lsb = 1.0 / 16.0
  window = LeadResidualWindow()
  v = 1.20
  armed_prev = None
  for _ in range(40):
    prev, _dt, _a = window.update(v, -0.22, 0.05)
    if prev is not None:
      armed_prev = prev
    v = 1.20 + lsb
  assert armed_prev is None

  window = LeadResidualWindow()
  prev = None
  dt_w = 0.05
  v0 = 1.40
  for i in range(14):
    prev, dt_w, _a_w = window.update(v0 + 0.04 * i, -0.22, 0.05)
  assert prev is not None
  assert dt_w == pytest.approx(0.50, abs=0.02)
