"""Cone-line detector: synthetic radar, hysteresis, and the rejects."""

import math
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.cone_line import (
  CLEARANCE_M,
  CONFIRM_S,
  DROP_S,
  LAT_MIN_M,
  WOULD_STEER_CAP_M,
  ConeLineDetector,
  device_y_right,
)

V = 12.0
DT = 0.125  # 8 Hz
# Straight path. Device y is +right (model position.y). radar yRel is +left,
# so a left-side return has positive yRel and negative path lat.
STRAIGHT_X = [0.0, 30.0, 70.0]
STRAIGHT_Y = [0.0, 0.0, 0.0]


def _pts(xs, y_rel, v=V, measured=True, v_rel=None):
  rel = -v if v_rel is None else v_rel
  return [
    SimpleNamespace(dRel=x - 1.52, yRel=y_rel, vRel=rel, measured=measured)
    for x in xs
  ]


def _cones(y_rel=1.6):
  # Device x along the cone stretch. yRel > 0 is to the left.
  return _pts([10, 16, 23, 31, 39, 48, 57], y_rel)


def _run(det, points, seconds, path_x=STRAIGHT_X, path_y=STRAIGHT_Y, v=V, edges=None, dt=DT):
  n = max(1, int(round(seconds / dt)))
  sample = None
  for _ in range(n):
    sample = det.update(points, v, path_x, path_y, edges, dt)
  return sample


def test_left_side_cone_line_confirms_with_hysteresis():
  det = ConeLineDetector()
  early = _run(det, _cones(), CONFIRM_S * 0.5)
  assert early.active is False
  assert early.confidence < 0.5
  sample = _run(det, _cones(), CONFIRM_S * 0.6)
  assert sample.active is True
  assert sample.side == 1
  assert sample.count >= 4
  assert sample.span_m >= 10.0
  assert sample.confidence >= 0.5
  assert sample.barrier is False
  assert sample.parked is False
  # yRel +1.6 is 1.6 m left. Path lat is +right, so the line is at -1.6.
  # Already past the 1 m clearance, so no shift.
  assert sample.lat_mid == pytest.approx(-1.6, abs=0.02)
  assert sample.would_limit == 0.0
  assert sample.would_steer == 0.0
  assert device_y_right(1.6) == pytest.approx(-1.6)
  # A short dropout does not clear the line.
  held = _run(det, [], DROP_S * 0.5)
  assert held.active is True
  cleared = _run(det, [], DROP_S)
  assert cleared.active is False


def test_close_line_reports_the_would_limit_shift():
  det = ConeLineDetector()
  # 0.7 m left of the path (yRel +left). Keep ~1.0 m → shift the path right by 0.3 m.
  sample = _run(det, _cones(0.7), CONFIRM_S + 1.0)
  assert sample.active is True
  assert sample.side == 1
  assert sample.lat_near < 0.0
  assert sample.would_limit == pytest.approx(CLEARANCE_M - 0.7, abs=0.02)
  assert sample.would_steer == pytest.approx(CLEARANCE_M - 0.7, abs=0.02)


def test_single_mailbox_never_becomes_a_line():
  det = ConeLineDetector()
  sample = _run(det, _pts([18.0], 1.5), 3.0)
  assert sample.active is False
  assert sample.count == 0
  assert sample.parked is False
  assert sample.barrier is False
  assert sample.would_limit == 0.0


def test_no_objects():
  det = ConeLineDetector()
  sample = _run(det, [], 2.0)
  assert sample.active is False
  assert sample.side == 0
  assert sample.count == 0


def test_guardrail_is_flagged_not_a_cone_line():
  det = ConeLineDetector()
  # Continuous returns every 1 m. Distinguishable from drums spaced several meters apart.
  # A single scan must keep them: the match gate is wider than 1 m, and a new
  # return is not allowed to steal the post the previous return just created.
  xs = [8.0 + i for i in range(22)]
  det.update(_pts(xs, 1.7), V, STRAIGHT_X, STRAIGHT_Y, None, DT)
  assert len(det._posts) >= 18
  sample = _run(det, _pts(xs, 1.7), CONFIRM_S + 1.0)
  assert sample.active is False
  assert sample.barrier is True
  assert sample.parked is False
  assert sample.side == 1
  assert sample.lat_mid < 0.0
  assert sample.would_steer == 0.0


def test_parked_car_wide_cluster_is_rejected():
  det = ConeLineDetector()
  # Four returns across the width of a car, only a few meters long.
  points = []
  for i, lat in enumerate((-1.0, -1.6, -2.2, -2.5)):
    points.extend(_pts([20.0 + i * 0.8], -lat))
  sample = _run(det, points, 3.0)
  assert sample.active is False
  assert sample.parked is True
  assert sample.barrier is False


def test_moving_traffic_and_in_path_lead_are_not_a_line():
  det = ConeLineDetector()
  moving = _pts([10, 16, 23, 31, 39, 48, 57], 1.6, v_rel=-V + 6.0)
  sample = _run(det, moving, CONFIRM_S + 0.5)
  assert sample.active is False
  lead = _pts([12, 18, 25, 33, 42], 0.2)  # inside the lane, under the roadside band
  sample = _run(ConeLineDetector(), lead, CONFIRM_S + 0.5)
  assert sample.active is False
  assert LAT_MIN_M == 0.30


def test_curve_uses_path_relative_lateral_not_ego_yrel():
  """Ego yRel 0.4 m can still be 1.6 m off the path once the path has bent."""
  det = ConeLineDetector()
  xs = [10, 16, 23, 31, 39, 48, 57]
  # Path y +1.2 is 1.2 m to the right. yRel +0.4 is 0.4 m left of the car
  # (device y -0.4), so the return sits 1.6 m left of the path.
  path_x = [0.0, 20.0, 40.0, 70.0]
  path_y = [0.0, 1.2, 1.2, 1.2]
  sample = _run(det, _pts(xs, 0.4), CONFIRM_S + DT, path_x, path_y)
  assert sample.active is True
  assert sample.side == 1
  assert sample.lat_mid == pytest.approx(-1.6, abs=0.05)
  # The same ego offset ON the path (path y = -0.4) is inside the lane band.
  on_path = _run(ConeLineDetector(), _pts(xs, 0.4), CONFIRM_S + DT,
                 [0.0, 70.0], [-0.4, -0.4])
  assert on_path.active is False


def test_objects_past_the_road_edge_are_dropped():
  det = ConeLineDetector()
  edges = [([0.0, 80.0], [2.2, 2.2]), ([0.0, 80.0], [-1.2, -1.2])]
  # yRel +1.9 is device y -1.9, past the left road edge (edge y is +right).
  outside = _run(det, _pts([10, 16, 23, 31, 39, 48, 57], 1.9), CONFIRM_S + DT, edges=edges)
  assert outside.active is False
  inside = _run(ConeLineDetector(), _cones(1.5), CONFIRM_S + DT, edges=edges)
  assert inside.active is True
  assert inside.side == 1


def test_right_side_yrel_is_negative():
  """The mirror of the left-side case. yRel < 0 is the driver's right."""
  sample = _run(ConeLineDetector(), _cones(-1.6), CONFIRM_S + DT)
  assert sample.active is True
  assert sample.side == -1
  assert sample.lat_mid == pytest.approx(1.6, abs=0.02)


def test_posts_closer_than_half_a_meter_still_count():
  """The old 0.5 m inner band dropped the taper. 0.40 m is a cone; 0.20 m is a lead."""
  close = _pts([10, 18, 26, 34, 42, 50], 0.40)
  sample = _run(ConeLineDetector(), close, CONFIRM_S + DT)
  assert sample.active is True
  assert sample.side == 1
  assert sample.count >= 6
  assert sample.lat_mid == pytest.approx(-0.40, abs=0.05)
  lane = _run(ConeLineDetector(), _pts([10, 18, 26, 34, 42, 50], 0.20), CONFIRM_S + DT)
  assert lane.active is False


def test_would_steer_is_capped_and_points_away_from_left_posts():
  # 0.55 m left. Opening 1.0 m wants 0.45 m; the log caps the proposal at 0.4.
  sample = _run(ConeLineDetector(), _cones(0.55), CONFIRM_S + 1.0)
  assert sample.active is True
  assert sample.side == 1
  assert sample.would_limit == pytest.approx(CLEARANCE_M - 0.55, abs=0.03)
  assert sample.would_steer == pytest.approx(WOULD_STEER_CAP_M, abs=0.02)
  assert sample.would_steer > 0.0  # +right, away from the left posts


def test_curved_ground_line_confirms_when_a_straight_lat_std_would_not():
  """Quadratic in the device frame. Raw lateral std is past 0.5 m, so the
  old straight-line gate would have thrown the row out. The ground-frame
  fit keeps it, on the left.
  """
  xs = [10, 18, 26, 34, 42, 50, 58]
  # y_right = -0.40 - 0.0035*(x-34)^2. Ends are ~2.4 m left, the middle
  # is 0.40 m left, and the raw std is past the 0.50 m residual cap.
  ys = [-0.40 - 0.0035 * (x - 34.0) ** 2 for x in xs]
  assert _std_abs(ys) > 0.50
  assert max(abs(y) for y in ys) < 2.50
  points = _pts_yx(xs, [-y for y in ys])  # yRel = -y_right, so these are left posts
  sample = _run(ConeLineDetector(), points, CONFIRM_S + DT)
  assert sample.active is True
  assert sample.side == 1
  assert sample.lat_near < 0.0


def test_oct7_left_channelizers_hold_through_the_taper():
  """Route 00000172--2a94c25d70, 14:21:15–14:21:34 CT.

  Orange/white posts on the left of the lane, tapering from about 2.3 m
  out to 0.36 m (inside the old 0.5 m discard). Once the car is in the
  zone the radar only returns a couple of posts per scan. Side must be
  left (+1). The line stays up for ~2 s after the hits stop.
  """
  v = 18.0
  dt = DT
  # Every post stays inside the 6–70 m scan. The last one is 0.36 m off
  # the path, inside the old 0.5 m discard.
  xs = [12.0 + 8.0 * i for i in range(8)]
  ys = [-2.30 + (2.30 - 0.36) * (i / 7.0) for i in range(8)]
  assert ys[0] == pytest.approx(-2.30, abs=0.01)
  assert ys[-1] == pytest.approx(-0.36, abs=0.01)
  assert abs(ys[-1]) < 0.50
  assert max(xs) <= 70.0
  det = ConeLineDetector()

  def points(which):
    return [
      SimpleNamespace(dRel=xs[i] - 1.52, yRel=-ys[i], vRel=-v, measured=True)
      for i in which
    ]

  def step(which):
    return det.update(points(which), v, STRAIGHT_X, STRAIGHT_Y, None, dt, 0.0)

  sample = None
  for _ in range(int(round(1.3 / dt))):
    sample = step(range(8))
  assert sample is not None and sample.active is True
  assert sample.side == 1
  assert sample.lat_mid < 0.0
  assert len(det._posts) <= 10
  assert any(abs(p.y) < 0.50 for p in det._posts)

  # Two posts a scan for 3 s, always including the post inside 0.5 m.
  for n in range(int(round(3.0 / dt))):
    sample = step((n % 6, 7))
    assert sample.side == 1
    assert sample.active is True
  assert len(det._posts) <= 12

  for _ in range(int(round(1.5 / dt))):
    sample = step(())
  assert sample.active is True
  assert sample.side == 1

  for _ in range(int(round(0.8 / dt))):
    sample = step(())
  assert sample.active is False


def test_moving_posts_associate_by_position_not_by_duplicating():
  """On the road each scan steps back by about v*dt. Compensation has to
  match the same posts, including through a small yaw, instead of spawning
  a new post every frame.
  """
  v = 16.0
  yaw = -0.04  # left turn, +right frame
  dt = DT
  xs0 = [14.0, 22.0, 30.0, 38.0, 46.0, 54.0]
  y_rel = 1.6
  det = ConeLineDetector()
  sample = _run(det, _pts(xs0, y_rel, v=v), CONFIRM_S + dt, v=v, dt=dt)
  assert sample.active is True
  assert sample.side == 1
  n0 = len(det._posts)
  shift = 0.0
  psi = 0.0
  for _ in range(10):
    shift += v * dt
    psi += yaw * dt
    c, s = math.cos(psi), math.sin(psi)
    pts = []
    for x0 in xs0:
      # Fixed world posts, ego moved forward `shift` and yawed `psi`.
      dx, dy = x0 - shift, -1.6
      x_v = c * dx + s * dy
      y_v = -s * dx + c * dy
      if 6.0 <= x_v <= 70.0:
        pts.append(SimpleNamespace(dRel=x_v - 1.52, yRel=-y_v, vRel=-v, measured=True))
    sample = det.update(pts, v, STRAIGHT_X, STRAIGHT_Y, None, dt, yaw)
    assert len(det._posts) <= n0 + 2
  assert sample.active is True
  assert sample.side == 1


def test_town_and_intersection_near_misses_stay_inactive():
  """14:29 town street and 14:44 intersection must not become a cone line.

  The town case is two brief clutter bursts. The intersection is a real-looking
  line for 0.6 s (under the 1.0 s confirm) and then poles on both sides.
  """
  det = ConeLineDetector()
  clutter = _pts([14.0, 18.0], 1.1) + _pts([22.0], -2.4) + _pts([16.0, 17.0, 18.5], 2.2)
  sample = _run(det, clutter, 0.25, v=5.0)
  assert sample.active is False
  sample = _run(det, [], 2.5, v=5.0)
  parked = []
  for i, lat in enumerate((1.2, 1.9, 2.6)):
    parked.extend(_pts([18.0 + i * 0.7], lat))
  sample = _run(det, parked, 0.4, v=4.0)
  assert sample.active is False
  sample = _run(det, [], 2.0, v=6.0)
  assert sample.active is False

  det = ConeLineDetector()
  saw = False
  sample = _run(det, _cones(1.5), 0.60, v=7.5)
  saw = saw or sample.active
  assert sample.active is False
  assert sample.confidence <= 0.49
  mixed = _pts([12, 20, 28], 1.4) + _pts([16, 24, 33], -1.7)
  sample = _run(det, mixed, 1.2, v=7.5)
  saw = saw or sample.active
  sample = _run(det, [], 1.0, v=7.5)
  assert sample.active is False
  assert saw is False


def test_schema_documents_the_right_positive_frame():
  from pathlib import Path
  text = Path(__file__).resolve().parents[4].joinpath("cereal/custom.capnp").read_text()
  cone = text.split("struct ConeLineNAP", 1)[1].split("struct PathObstacleNAP", 1)[0]
  assert "+right" in cone
  assert "yRel is +left" in cone
  assert "+1 left" in cone
  assert "wouldSteer" in cone
  assert "Not applied" in cone
  obstacle = text.split("struct PathObstacleNAP", 1)[1]
  assert "+right" in obstacle.split("enum ObjectClass", 1)[0]


def test_a_fat_scan_stays_cheap():
  import time
  det = ConeLineDetector()
  points = []
  for i in range(200):
    points.append(SimpleNamespace(
      dRel=8.0 + (i % 50), yRel=((i % 9) - 4) * 0.4, vRel=-V, measured=True,
    ))
  t0 = time.monotonic()
  sample = None
  for _ in range(30):
    sample = det.update(points, V, STRAIGHT_X, STRAIGHT_Y, None, DT, 0.0)
  assert time.monotonic() - t0 < 0.25
  assert sample is not None
  assert sample.active is False


def _std_abs(vals):
  n = len(vals)
  mean = sum(vals) / n
  return (sum((v - mean) ** 2 for v in vals) / n) ** 0.5


def _pts_yx(xs, y_rels, v=V):
  return [
    SimpleNamespace(dRel=x - 1.52, yRel=y_rel, vRel=-v, measured=True)
    for x, y_rel in zip(xs, y_rels)
  ]
