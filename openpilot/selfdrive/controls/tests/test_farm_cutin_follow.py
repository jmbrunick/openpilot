"""Farm-vehicle approach and faster cut-in.

Oct 7 2026, route 00000172--2a94c25d70. A 25 mph tractor was held at the
EV mild settle by the post-planner regen guard once closing speed fell
under 6 m/s, then braked late. A faster cut-in was braked while it was
still pulling away. #222 exemplar timings are pinned to the stock law.
"""

import pytest

from openpilot.selfdrive.controls.lib.lead_approach import (
  LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2,
  guard_follow_actuator_regen,
)
from openpilot.selfdrive.controls.lib.unified_lead import (
  A_MIN_MS2,
  APPROACH_MILD_MS2,
  APPROACH_RAPID_DWELL_S,
  APPROACH_RAPID_ENTER_MS,
  APPROACH_RAPID_EXIT_MS,
  CUTIN_HARD_FRAMES,
  TRICKLE_A,
  ApproachInputs,
  UnifiedLeadController,
  approach_physics_need,
  gap_set_m,
)
from openpilot.selfdrive.controls.tests.test_unified_lead import (
  SEP20_0755_ROWS,
  SEP22_2331_ROWS,
  SEP23_2214_ROWS,
  _first_at_or_below,
  _log_replay,
)

DT = 0.05
MPH = 0.44704
PLANT_LAG_S = 0.35
PLANT_FLOOR = -1.5

# Stock unified replay of the three #222 logs, before this change.
# t(−0.5), t(−1.0), min command. These must not move.
_222_STOCK = {
  "sep20": (SEP20_0755_ROWS, -0.02, 1.70, 1.90, -2.352904),
  "sep23": (SEP23_2214_ROWS, 0.37, 1.05, 5.50, -1.086998),
  "sep22": (SEP22_2331_ROWS, -0.02, 2.60, 2.90, -2.242028),
}


def _physics_need(gap, v_ego, v_lead, t_follow):
  return approach_physics_need(gap, v_ego, v_lead, t_follow)


def _farm_replay():
  """Closed loop of the 14:34 tractor approach. Returns gap, commands, needs."""
  ctrl = UnifiedLeadController()
  v_lead = 11.3  # 25 mph
  t_follow = 0.7
  v_ceil = 55 * MPH
  gap = 111.6
  v_ego = v_lead + 12.9  # 54.3 mph at acquire
  a_act = 0.0
  min_gap = gap
  rows = []
  prev = 0.0
  late_slam = False
  eased = False
  for k in range(int(round(45.0 / DT))):
    t = k * DT
    plan = ctrl.step(
      dt=DT, present=True, gap=gap, v_ego=v_ego, v_lead=v_lead, a_lead=0.0,
      t_follow=t_follow, lead_id=2110, seed_a=0.0, v_ceiling=v_ceil,
      radar=True, model_prob=0.9, y_rel=0.5,
    )
    slack = gap - gap_set_m(v_lead, t_follow)
    v_close = v_ego - v_lead
    act = guard_follow_actuator_regen(
      plan, plan, v_rel=v_close, d_rel=gap, slack=slack, a_lead=0.0,
      v_ego=v_ego, v_cruise=v_ceil,
    )
    act = max(PLANT_FLOOR, float(act))
    # A late slam is a return to the regen rail after the approach has
    # already eased. Early firm braking, then a smooth release, is not one.
    if act > -0.55:
      eased = True
    if eased and prev > -0.7 and act <= -1.3:
      late_slam = True
    prev = act
    need = _physics_need(gap, v_ego, v_lead, t_follow)
    rows.append((t, gap, v_close, slack, float(plan), act, need))
    a_act += (act - a_act) * DT / PLANT_LAG_S
    v_ego = max(0.0, v_ego + a_act * DT)
    gap += (v_lead - v_ego) * DT
    min_gap = min(min_gap, gap)
  return min_gap, late_slam, rows


def test_farm_approach_keeps_at_least_11m_and_meets_the_need():
  """14:34 tractor: early progressive brake, no late −1.5, min gap ≥ 11 m.

  Stock closed loop of this geometry stopped at 7.1 m because the regen
  guard held −0.22 and then −0.45. The follow law, with that cap retired,
  starts braking while the gap is still long.
  """
  min_gap, late_slam, rows = _farm_replay()
  assert min_gap >= 11.0
  assert not late_slam
  # After the jerk ramp, a close that still needs more than a mild settle
  # is not held above that need.
  checked = 0
  for t, _gap, v_close, slack, plan, act, need in rows:
    if t < 1.5 or need >= -0.30 or v_close <= 0.0 or slack <= 3.0:
      continue
    checked += 1
    assert plan <= need + 0.08
    assert act <= need + 0.08
  assert checked > 50
  # Braking is underway before the gap is inside 60 m, and it eases as
  # the close does. It does not sit on the mild settle and then dive.
  early = [act for t, gap, _v, _s, _p, act, _n in rows if gap > 60.0 and t > 0.4]
  assert early and min(early) <= -0.8
  mild_settle_while_hot = [
    act for t, gap, v_close, slack, _p, act, need in rows
    if 8.0 <= slack <= 50.0 and v_close >= 4.0 and need < -0.40 and act > -0.30
  ]
  assert mild_settle_while_hot == []


def test_nonbraking_slow_close_follows_the_planner_not_the_mild_cap():
  """The farm geometry: planner −0.87 is not rewritten to −0.22 or −0.45."""
  v_lead = 11.3
  t_follow = 0.7
  gap = 51.5
  v_ego = v_lead + 5.88
  slack = gap - gap_set_m(v_lead, t_follow)
  plan = -0.87
  act = guard_follow_actuator_regen(
    plan, plan, v_rel=5.88, d_rel=gap, slack=slack, a_lead=0.0,
    v_ego=v_ego, v_cruise=55 * MPH,
  )
  assert act == pytest.approx(plan)
  # Windup still cannot outrun the planner command.
  wind = guard_follow_actuator_regen(
    -1.5, plan, v_rel=5.88, d_rel=gap, slack=slack, a_lead=0.0,
    v_ego=v_ego, v_cruise=55 * MPH,
  )
  assert wind == pytest.approx(plan)


def test_mild_settle_and_firm_lead_cap_are_unchanged():
  """EV −0.22 still catches plant windup. A firm #222-style lead still clips."""
  assert guard_follow_actuator_regen(
    -1.32, -0.22, v_rel=2.0, d_rel=40.0, slack=20.0, a_lead=0.0,
  ) == pytest.approx(LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2)
  assert guard_follow_actuator_regen(
    -1.5, -1.5, v_rel=1.2, d_rel=38.0, slack=18.0, a_lead=-0.6,
  ) == pytest.approx(-APPROACH_MILD_MS2)
  # Coast with no lead geometry still does not open the settle.
  assert guard_follow_actuator_regen(-1.23, 0.0) == pytest.approx(0.0)


def test_approach_rapid_regime_has_enter_exit_hysteresis():
  """A sample chatter around 6 m/s does not flip the input regime."""
  inp = ApproachInputs()
  v_ego = 20.0
  # Just under the enter line: not rapid, and one frame over does not enter.
  inp.update(80.0, v_ego, v_ego - (APPROACH_RAPID_ENTER_MS - 0.2), DT)
  assert inp.rapid is False
  inp.update(80.0, v_ego, v_ego - (APPROACH_RAPID_ENTER_MS + 0.2), DT)
  assert inp.rapid is False
  frames = int(APPROACH_RAPID_DWELL_S / DT)
  for _ in range(frames):
    inp.update(80.0, v_ego, v_ego - (APPROACH_RAPID_ENTER_MS + 0.3), DT)
  assert inp.rapid is True
  # One frame under the exit line does not leave.
  inp.update(70.0, v_ego, v_ego - (APPROACH_RAPID_EXIT_MS - 0.2), DT)
  assert inp.rapid is True
  for _ in range(frames):
    inp.update(70.0, v_ego, v_ego - (APPROACH_RAPID_EXIT_MS - 0.3), DT)
  assert inp.rapid is False
  # An obvious rapid acquire is rapid on the first sample.
  hot = ApproachInputs()
  hot.update(111.6, 24.2, 11.3, DT)
  assert hot.rapid is True


@pytest.mark.parametrize("name", list(_222_STOCK))
def test_222_log_exemplar_numbers_are_unchanged(name):
  rows, seed, t05, t10, mn = _222_STOCK[name]
  cmds = _log_replay(rows, seed)
  assert _first_at_or_below(cmds, -0.5) == pytest.approx(t05, abs=1e-9)
  assert _first_at_or_below(cmds, -1.0) == pytest.approx(t10, abs=1e-9)
  assert min(cmds) == pytest.approx(mn, abs=1e-6)
  assert min(cmds) >= A_MIN_MS2


def test_new_lead_alead_is_seeded_from_the_first_radar_value():
  """A new track must not start the accel filter at 0 (false lead jerk)."""
  ctrl = UnifiedLeadController()
  v_ego = 24.5
  ctrl.step(
    dt=DT, present=True, gap=23.4, v_ego=v_ego, v_lead=v_ego + 8.0, a_lead=-0.54,
    t_follow=0.7, lead_id=2205, seed_a=0.0, v_ceiling=30.0, radar=True, model_prob=0.9,
  )
  assert ctrl.last_a_lead == pytest.approx(-0.54, abs=0.02)
  assert abs(ctrl._lead_jerk) < 0.25


def test_faster_cutin_does_not_brake_while_pulling_away():
  """14:37:35 cut-in: 72.8 mph into a 54.8 mph ego at 23.4 m. Zero brake.

  The passer eases toward 65 mph and is still faster at the end, so the
  hold never releases and the gap opens. Inside the follow window the
  positive command stays on the trickle.
  """
  ctrl = UnifiedLeadController()
  v_ego = 54.8 * MPH
  v_fast = 72.8 * MPH
  v_ease = 65.0 * MPH
  gap = 23.4
  t_follow = 0.7
  v_ceil = 55.0 * MPH
  cmds = []
  for k in range(int(round(5.0 / DT))):
    t = k * DT
    if t < 4.0:
      v_lead = v_fast + (v_ease - v_fast) * (t / 4.0)
      a_lead = (v_ease - v_fast) / 4.0
    else:
      v_lead = v_ease
      a_lead = 0.0
    cmd = ctrl.step(
      dt=DT, present=True, gap=gap, v_ego=v_ego, v_lead=v_lead, a_lead=a_lead,
      t_follow=t_follow, lead_id=2205, seed_a=0.0, v_ceiling=v_ceil,
      radar=True, model_prob=0.9, y_rel=1.4,
    )
    cmds.append(cmd)
    assert ctrl.pulling_away
    gap_set = gap_set_m(v_lead, t_follow)
    if gap <= gap_set:
      assert cmd <= TRICKLE_A + 1e-6
    gap += (v_lead - v_ego) * DT
    v_ego = max(0.0, v_ego + cmd * DT)
  assert min(cmds) >= -1e-6
  assert gap > 23.4


def test_cutin_hard_brake_releases_and_full_regen_is_available():
  """+3 m/s at 15 m, then the lead brakes at −3. Release on that brake."""
  ctrl = UnifiedLeadController()
  v_ego, v_lead, gap = 25.0, 28.0, 15.0
  hard_at = None
  release_at = None
  cmds = []
  for k in range(int(round(3.0 / DT))):
    t = k * DT
    a_lead = -3.0 if t >= 0.5 else 0.0
    cmd = ctrl.step(
      dt=DT, present=True, gap=gap, v_ego=v_ego, v_lead=v_lead, a_lead=a_lead,
      t_follow=0.7, lead_id=3, seed_a=0.0, v_ceiling=30.0, radar=True, model_prob=1.0,
    )
    cmds.append(cmd)
    if hard_at is None and a_lead <= -1.5:
      hard_at = t
    if release_at is None and not ctrl.pulling_away:
      release_at = t
    v_lead = max(0.0, v_lead + a_lead * DT)
    v_ego = max(0.0, v_ego + cmd * DT)
    gap += (v_lead - v_ego) * DT
  assert hard_at == pytest.approx(0.5)
  # Two hard frames, or the projected-gap backstop on the first one.
  assert release_at is not None
  assert release_at <= hard_at + CUTIN_HARD_FRAMES * DT + 1e-9
  assert min(cmds) <= -3.0
  assert min(cmds) >= A_MIN_MS2


def test_cutin_easing_releases_on_predicted_closing_without_an_early_brake():
  """+2.5 m/s at 20 m, lead eases at −0.6. No brake until the handover."""
  ctrl = UnifiedLeadController()
  v_ego, v_lead, gap = 25.0, 27.5, 20.0
  release_at = None
  before = []
  after = []
  for k in range(int(round(8.0 / DT))):
    t = k * DT
    cmd = ctrl.step(
      dt=DT, present=True, gap=gap, v_ego=v_ego, v_lead=v_lead, a_lead=-0.6,
      t_follow=0.7, lead_id=4, seed_a=0.0, v_ceiling=30.0, radar=True, model_prob=1.0,
    )
    if ctrl.pulling_away:
      before.append(cmd)
    else:
      if release_at is None:
        release_at = t
      after.append(cmd)
    # Ego holds speed aside from the command; the lead is slowing.
    v_lead = max(0.0, v_lead - 0.6 * DT)
    v_ego = max(0.0, v_ego + min(cmd, 0.0) * DT)
    gap += (v_lead - v_ego) * DT
  assert release_at == pytest.approx(3.75, abs=0.15)
  assert before and min(before) >= -1e-6
  assert after and min(after) <= -0.4
  # Not the stock copy of the passer's decel (−0.94). The handover is milder.
  assert min(after) > -0.9


def test_cutin_inside_9m_releases_immediately():
  """+2.2 m/s at 9 m is already inside the keep distance. No hold."""
  ctrl = UnifiedLeadController()
  cmd = ctrl.step(
    dt=DT, present=True, gap=9.0, v_ego=22.0, v_lead=24.2, a_lead=0.0,
    t_follow=0.7, lead_id=5, seed_a=0.0, v_ceiling=28.0, radar=True, model_prob=1.0,
  )
  assert ctrl.pulling_away is False
  assert cmd < 0.0


def test_cutin_becomes_a_normal_lead_once_it_is_slower_than_ego():
  """Closing that persists hands the cut-in to the normal follow law."""
  ctrl = UnifiedLeadController()
  v_ego, v_lead, gap = 22.0, 24.5, 30.0
  release_at = None
  for k in range(int(round(6.0 / DT))):
    t = k * DT
    # Strong enough to be slower than ego inside 1.5 s, not a −1.5 spike.
    cmd = ctrl.step(
      dt=DT, present=True, gap=gap, v_ego=v_ego, v_lead=v_lead, a_lead=-1.2,
      t_follow=0.7, lead_id=6, seed_a=0.0, v_ceiling=26.0, radar=True, model_prob=1.0,
    )
    if release_at is None and not ctrl.pulling_away:
      release_at = t
      assert cmd <= 0.05
    v_lead = max(0.0, v_lead - 1.2 * DT)
    gap += (v_lead - v_ego) * DT
  assert release_at is not None and release_at < 3.0
  # One-way: once released, a still-positive filtered speed does not re-latch.
  assert ctrl.pulling_away is False


def test_slightly_faster_new_lead_does_not_enter_the_hold():
  """SEP23 starts only +1.6 m/s. The +2.0 entry must not latch."""
  ctrl = UnifiedLeadController()
  ctrl.step(
    dt=DT, present=True, gap=41.5, v_ego=29.58, v_lead=31.18, a_lead=-0.08,
    t_follow=0.7, lead_id=1, seed_a=0.37, v_ceiling=40.0, radar=True, model_prob=0.9,
  )
  assert ctrl.pulling_away is False
