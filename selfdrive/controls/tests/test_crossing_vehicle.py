"""Crossing / turning vehicle in the longitudinal law.

E1, 14:44:34–40, track 2842: a side-on car turned across the lane. Radar
held the glint near the centreline, the car braked as if it were stopped,
and long did not move again until the stalk. A crosser slows gently and
starts accelerating when the tail's exit sweep begins. A stopped car in
the lane still gets the normal brake. Gas touched during the brake stays
paused. Stationary roadside returns (E2) are not crossers.
"""
import math

import pytest

from openpilot.selfdrive.controls.lib.crossing_vehicle import (
  CROSS_VY_MS,
  X2_ROOM_FRAC,
  GroundLateral,
  crossing_may_resume,
)
from openpilot.selfdrive.controls.lib.unified_lead import (
  UnifiedLeadController,
  kinematic_required_accel,
  kinematic_room_m,
)
from openpilot.selfdrive.controls.radard import KalmanParams, Track
from openpilot.selfdrive.controls.tests.test_radard import RadarScenario

DT = 0.05
RADAR_DT = 0.125
V_EGO = 13.2
V_CEIL = 20.0
T_FOLLOW = 0.7
GAP0 = 80.0


def _e1_y(t: float) -> float:
  """Ego-frame y, +left. Entry, glint on the centreline, then the tail leaves."""
  if t < 0.90:
    return 0.10 + (1.70 / 0.90) * t
  if t < 1.40:
    u = (t - 0.90) / 0.50
    return 1.80 + (0.45 - 1.80) * u
  if t < 3.40:
    return 0.45
  if t < 4.30:
    u = (t - 3.40) / 0.90
    return 0.45 + 1.05 * u
  return 1.50


def _vlat_at(t: float, y_of) -> float:
  """Ground-frame lateral speed from the yaw-corrected fit, held between radar frames."""
  ground = GroundLateral()
  last = 0.0
  n = int(math.floor(t / RADAR_DT + 1e-9))
  for i in range(n + 1):
    tr = i * RADAR_DT
    if tr > t + 1e-9:
      break
    gap = max(8.0, GAP0 - V_EGO * tr)
    speeds = ground.update(tr, V_EGO, 0.0, [(2842, gap, y_of(tr))])
    last = float(speeds.get(2842, 0.0))
  return last


def _run(y_of, *, v_lat_of=None, model_prob=0.25, long_on_of=None, v_lead=0.2,
         seconds=5.0, v_ego=V_EGO, gap0=GAP0):
  ctrl = UnifiedLeadController()
  rows = []
  for k in range(int(round(seconds / DT))):
    t = k * DT
    gap = max(8.0, gap0 - v_ego * t)
    y = float(y_of(t))
    v_lat = 0.0 if v_lat_of is None else float(v_lat_of(t))
    long_on = True if long_on_of is None else bool(long_on_of(t))
    a = ctrl.step(
      dt=DT, present=True, gap=gap, v_ego=v_ego, v_lead=v_lead, a_lead=0.0,
      t_follow=T_FOLLOW, lead_id=2842, seed_a=0.0, v_ceiling=V_CEIL,
      y_rel=y, radar=True, model_prob=model_prob, v_lat=v_lat, long_on=long_on,
    )
    rows.append((t, y, a, bool(ctrl.crossing_classified), bool(ctrl._cross.exit)))
  return rows


def _median(vals):
  ordered = sorted(vals)
  return ordered[len(ordered) // 2]


def test_ground_lateral_ignores_ego_yaw_and_keeps_a_real_sweep():
  """A stopped point while we yaw stays near 0. A 1.5 m/s sweep does not."""
  yaw = 0.05
  spinning = GroundLateral()
  moving = GroundLateral()
  last_spin = 0.0
  last_move = 0.0
  for i in range(16):
    t = i * RADAR_DT
    heading = yaw * t
    d_rel = 80.0 * math.cos(heading)
    y_rel = -80.0 * math.sin(heading)
    last_spin = spinning.update(t, 0.0, yaw, [(1, d_rel, y_rel)]).get(1, 0.0)
    last_move = moving.update(t, V_EGO, 0.0, [(2, 70.0, 1.5 * t)]).get(2, 0.0)
  assert abs(last_spin) < 0.35
  assert last_move == pytest.approx(1.5, abs=0.25)
  assert last_move > CROSS_VY_MS


def test_e1_turning_car_slows_gently_and_reaccelerates_as_the_tail_leaves():
  rows = _run(_e1_y, v_lat_of=lambda t: _vlat_at(t, _e1_y), model_prob=0.25)
  glint = [a for t, y, a, classified, _exit in rows if 2.2 <= t <= 3.2]
  assert glint, "no glint samples"
  assert all(classified for t, y, a, classified, _exit in rows if 2.2 <= t <= 3.2)
  glint_a = _median(glint)
  # Slowing, and not the stopped-object brake.
  assert -0.95 < glint_a < -0.05, glint_a

  stopped = _run(lambda t: 0.40, v_lat_of=lambda t: 0.0, model_prob=0.85)
  stopped_glint = [a for t, y, a, classified, _exit in stopped if 2.2 <= t <= 3.2]
  assert stopped_glint
  assert not any(classified for _, _, _, classified, _ in stopped)
  stopped_a = _median(stopped_glint)
  assert stopped_a < -1.15, stopped_a
  assert glint_a > stopped_a + 0.40

  # Exit sweep starts at t=3.40, y=0.45, and the point is not clear until
  # y=1.50 at t=4.30. The flag is the start of that sweep, and the command
  # is already rising while the return is still in the lane.
  exits = [(t, y, a) for t, y, a, classified, exited in rows if exited and t >= 3.4]
  assert exits, "exit sweep never started"
  t0, y0, a0 = exits[0]
  assert t0 < 4.05, (t0, y0, a0)
  assert y0 < 1.15, (t0, y0, a0)
  before = [a for t, y, a, classified, exited in rows if 3.4 <= t < t0 and not exited]
  assert before
  rising = [(t, y, a) for t, y, a in exits if t0 + 0.20 <= t <= t0 + 0.40 and a >= before[-1] + 0.25]
  assert rising, [(round(t, 2), round(y, 2), round(a, 2)) for t, y, a in exits[:6]]
  t_rel, y_rel, a_rel = rising[0]
  assert y_rel < 1.40, (t_rel, y_rel, a_rel)


def test_stopped_car_in_the_lane_gets_full_braking():
  rows = _run(lambda t: 0.30, v_lat_of=lambda t: 0.0, model_prob=0.90, v_lead=0.0)
  assert not any(classified for _, _, _, classified, _ in rows)
  early = [a for t, _, a, _, _ in rows if 1.0 <= t <= 2.5]
  assert _median(early) < -1.20


def test_crosser_with_gas_touched_stays_paused():
  """Gas during the crossing brake: no resume when the tail leaves, and lift is not a resume."""
  assert crossing_may_resume(True)
  assert not crossing_may_resume(False)

  def long_on(t):
    # Touched at 2.0 s. Still false after a lift; only the stalk would set it.
    return t < 2.0

  paused = _run(_e1_y, v_lat_of=lambda t: _vlat_at(t, _e1_y), long_on_of=long_on)
  live = _run(_e1_y, v_lat_of=lambda t: _vlat_at(t, _e1_y), long_on_of=lambda t: True)
  after = [a for t, _, a, _, _ in paused if t >= 2.0]
  assert after
  assert max(after) <= 0.05
  # The exit does not release the brake while longitudinal is paused.
  at_touch = [a for t, _, a, _, _ in paused if abs(t - 2.0) < DT * 0.5][0]
  late = [a for t, _, a, _, _ in paused if t >= 4.2]
  assert max(late) <= at_touch + 0.15
  live_late = [a for t, _, a, _, _ in live if t >= 4.2]
  assert _median(live_late) > _median(late) + 0.30


def test_stationary_roadside_object_is_not_a_crosser():
  """E2: stopped returns beside the lane, including while we yaw. Not a crosser."""
  yaw = 0.04
  world_x, world_y = 90.0, 1.3
  # Same bicycle step GroundLateral uses, so a fixed world point has no lateral speed.
  poses = []
  heading = 0.0
  x_ego = 0.0
  y_ego = 0.0
  for i in range(24):
    if i:
      heading += yaw * RADAR_DT
      x_ego += V_EGO * math.cos(heading) * RADAR_DT
      y_ego += V_EGO * math.sin(heading) * RADAR_DT
    poses.append((heading, x_ego, y_ego))

  def measure(t):
    i = min(len(poses) - 1, int(math.floor(t / RADAR_DT + 1e-9)))
    h, xe, ye = poses[i]
    dx, dy = world_x - xe, world_y - ye
    d_rel = dx * math.cos(h) + dy * math.sin(h)
    y_rel = -dx * math.sin(h) + dy * math.cos(h)
    return d_rel, y_rel

  ground = GroundLateral()
  speeds = []
  ys = []
  for i in range(24):
    t = i * RADAR_DT
    d_rel, y_rel = measure(t)
    ys.append(y_rel)
    speeds.append(ground.update(t, V_EGO, yaw, [(2895, d_rel, y_rel)]).get(2895, 0.0))
  assert max(abs(v) for v in speeds) < CROSS_VY_MS

  def y_of(t):
    return measure(min(t, 23 * RADAR_DT))[1]

  def v_of(t):
    i = min(len(speeds) - 1, int(math.floor(t / RADAR_DT + 1e-9)))
    return speeds[i]

  rows = _run(y_of, v_lat_of=v_of, model_prob=0.90, v_lead=0.0, seconds=3.0, gap0=70.0)
  assert not any(classified for _, _, _, classified, _ in rows)
  plain = _run(y_of, v_lat_of=lambda t: 0.0, model_prob=0.90, v_lead=0.0, seconds=3.0, gap0=70.0)
  for got, base in zip(rows, plain):
    assert abs(got[2] - base[2]) < 0.08


def test_vision_lead_inside_55m_is_not_kept_as_a_crosser():
  """High model probability at the glint is a stopped lead, even if y is sweeping."""
  rows = _run(_e1_y, v_lat_of=lambda t: _vlat_at(t, _e1_y), model_prob=0.85, gap0=40.0, seconds=3.0)
  assert not any(classified for _, _, _, classified, _ in rows)
  mid = [a for t, _, a, _, _ in rows if 2.0 <= t <= 2.8]
  assert _median(mid) < -1.0


def test_imminent_overlap_returns_to_the_normal_brake_without_a_step():
  rows_far = _run(_e1_y, v_lat_of=lambda t: _vlat_at(t, _e1_y), seconds=3.2)
  assert any(classified for _, _, _, classified, _ in rows_far)
  gentle = _median([a for t, _, a, _, _ in rows_far if 2.4 <= t <= 3.0])
  # Same crosser, but the gap is already inside the stop-now distance.
  close = _run(_e1_y, v_lat_of=lambda t: _vlat_at(t, _e1_y), seconds=3.2, gap0=12.0)
  late = [a for t, _, a, _, _ in close if 2.6 <= t <= 3.1]
  assert _median(late) < gentle - 0.50
  prev = close[0][2]
  for _, _, a, _, _ in close[1:]:
    assert a - prev >= -2.5 * DT - 0.02
    prev = a


def test_along_traffic_and_a_pulling_away_cutin_are_not_crossers():
  sweeping = _run(_e1_y, v_lat_of=lambda t: _vlat_at(t, _e1_y), v_lead=V_EGO + 3.0, seconds=2.0)
  assert not any(classified for _, _, _, classified, _ in sweeping)
  assert min(a for _, _, a, _, _ in sweeping) >= -0.05


def test_x2_caps_room_shortening_for_a_stopped_lead_only():
  v, gap = 13.2, 118.0
  slack = gap - 6.0
  room = kinematic_room_m(slack, v, T_FOLLOW, 0.0)
  assert room >= X2_ROOM_FRAC * slack - 1e-6
  a = -(v * v) / (2.0 * room)
  assert -1.25 <= a <= -0.90
  moving = kinematic_room_m(slack, v, T_FOLLOW, 20.0)
  assert moving < X2_ROOM_FRAC * slack
  # Close stopped car: the floor does not take the brake away.
  close = kinematic_required_accel(24.0, 13.0, T_FOLLOW, 0.0)
  assert close <= -1.5


def test_track_radar_state_carries_ground_lateral_speed():
  tr = Track(2842, 0.0, KalmanParams(0.05))
  tr.update(40.0, 0.4, -V_EGO, 0.0, True, KalmanParams(0.05))
  tr.vLat = 1.4
  assert tr.get_RadarState(0.3)["vLat"] == pytest.approx(1.4)


def test_radard_fit_publishes_a_leftward_sweep_and_not_a_fixed_point():
  scenario = RadarScenario(v_ego=V_EGO)
  t = 0.0
  for i in range(10):
    t = i * RADAR_DT
    y = 1.6 * t
    scenario.step(t, vision_d_rel=60.0, radar_points=[(2842, 60.0, y, -V_EGO)], vision_prob=0.3)
  sweep = scenario.radar.tracks[2842].vLat
  assert sweep > CROSS_VY_MS

  still = RadarScenario(v_ego=V_EGO)
  for i in range(10):
    t = i * RADAR_DT
    still.step(t, vision_d_rel=60.0, radar_points=[(2895, 60.0, 1.2, -V_EGO)], vision_prob=0.9)
  assert abs(still.radar.tracks[2895].vLat) < 0.35
