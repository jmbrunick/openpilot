"""Cone-line detector: synthetic radar, hysteresis, and the rejects."""

from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.cone_line import (
  CLEARANCE_M,
  CONFIRM_S,
  DROP_S,
  LAT_MIN_M,
  ConeLineDetector,
)

V = 12.0
DT = 0.125  # 8 Hz
# Straight path. Device y +left, so a right-side return has negative lat.
STRAIGHT_X = [0.0, 30.0, 70.0]
STRAIGHT_Y = [0.0, 0.0, 0.0]


def _pts(xs, y_rel, v=V, measured=True, v_rel=None):
  rel = -v if v_rel is None else v_rel
  return [
    SimpleNamespace(dRel=x - 1.52, yRel=y_rel, vRel=rel, measured=measured)
    for x in xs
  ]


def _cones(y_rel=1.6):
  # Device x along the cone stretch. yRel > 0 is to the right.
  return _pts([10, 16, 23, 31, 39, 48, 57], y_rel)


def _run(det, points, seconds, path_x=STRAIGHT_X, path_y=STRAIGHT_Y, v=V, edges=None, dt=DT):
  n = max(1, int(round(seconds / dt)))
  sample = None
  for _ in range(n):
    sample = det.update(points, v, path_x, path_y, edges, dt)
  return sample


def test_right_side_cone_line_confirms_with_hysteresis():
  det = ConeLineDetector()
  early = _run(det, _cones(), CONFIRM_S * 0.5)
  assert early.active is False
  assert early.confidence < 0.5
  sample = _run(det, _cones(), CONFIRM_S * 0.6)
  assert sample.active is True
  assert sample.side == -1
  assert sample.count >= 4
  assert sample.span_m >= 10.0
  assert sample.confidence >= 0.5
  assert sample.barrier is False
  assert sample.parked is False
  # Line is ~1.6 m right of a straight path: already past the 1 m clearance.
  assert sample.lat_mid == pytest.approx(-1.6, abs=0.02)
  assert sample.would_limit == 0.0
  # A short dropout does not clear the line.
  held = _run(det, [], DROP_S * 0.5)
  assert held.active is True
  cleared = _run(det, [], DROP_S)
  assert cleared.active is False


def test_close_line_reports_the_would_limit_shift():
  det = ConeLineDetector()
  # 0.7 m right of the path. Keep ~1.0 m → shift the path left by 0.3 m.
  sample = _run(det, _cones(0.7), CONFIRM_S + DT)
  assert sample.active is True
  assert sample.side == -1
  assert sample.lat_near < 0.0
  assert sample.would_limit == pytest.approx(CLEARANCE_M - 0.7, abs=0.02)


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
  xs = [8.0 + i for i in range(22)]
  sample = _run(det, _pts(xs, 1.7), CONFIRM_S + 1.0)
  assert sample.active is False
  assert sample.barrier is True
  assert sample.parked is False
  assert sample.side == -1
  assert sample.would_limit == 0.0 or sample.lat_mid < 0.0


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
  lead = _pts([12, 18, 25, 33, 42], 0.2)  # inside the lane, under the 0.5 m band
  sample = _run(ConeLineDetector(), lead, CONFIRM_S + 0.5)
  assert sample.active is False
  assert LAT_MIN_M == 0.50


def test_curve_uses_path_relative_lateral_not_ego_yrel():
  """Ego yRel 0.4 m can still be 1.5 m off the path once the path has bent."""
  det = ConeLineDetector()
  xs = [10, 16, 23, 31, 39, 48, 57]
  # Path is already +1.2 m left. Radar yRel +0.4 (device y -0.4) sits ~1.6 m right of it.
  path_x = [0.0, 20.0, 40.0, 70.0]
  path_y = [0.0, 1.2, 1.2, 1.2]
  sample = _run(det, _pts(xs, 0.4), CONFIRM_S + DT, path_x, path_y)
  assert sample.active is True
  assert sample.side == -1
  assert sample.lat_mid == pytest.approx(-1.6, abs=0.05)
  # The same ego offset ON the path (path y = -0.4) is inside the lane band.
  on_path = _run(ConeLineDetector(), _pts(xs, 0.4), CONFIRM_S + DT,
                 [0.0, 70.0], [-0.4, -0.4])
  assert on_path.active is False


def test_objects_past_the_road_edge_are_dropped():
  det = ConeLineDetector()
  edges = [([0.0, 80.0], [2.2, 2.2]), ([0.0, 80.0], [-1.2, -1.2])]
  # Device y -1.9 is in the 2.5 m band of a straight path, but past the right edge.
  outside = _run(det, _pts([10, 16, 23, 31, 39, 48, 57], 1.9), CONFIRM_S + DT, edges=edges)
  assert outside.active is False
  inside = _run(ConeLineDetector(), _cones(1.5), CONFIRM_S + DT, edges=edges)
  assert inside.active is True
  assert inside.side == -1
