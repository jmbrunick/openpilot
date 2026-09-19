"""Travel-path / oncoming gates. No cereal — fusion stays in test_radard."""
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.radar_path_gate import (
  ONCOMING_VLEAD_MS,
  PATH_HALF_WIDTH_M,
  PATH_INCUMBENT_HALF_WIDTH_M,
  RADAR_TO_CAMERA_M,
  ROUNDABOUT_EXIT_PATH_X,
  ROUNDABOUT_EXIT_PATH_Y,
  ROUNDABOUT_EXIT_X_M,
  CURVE_OUTSIDE_PATH_X,
  CURVE_OUTSIDE_PATH_Y,
  CURVE_OUTSIDE_TWO_LANES_M,
  CURVE_OUTSIDE_X_M,
  path_lateral_m,
  path_y_at_x,
  radar_follow_ok,
  track_is_in_path,
  track_is_oncoming,
  vision_lead_follow_ok,
)

ROUNDABOUT_ENTRANT_DREL = ROUNDABOUT_EXIT_X_M - RADAR_TO_CAMERA_M


def radar(d_rel=40.0, y_rel=0.0, v_rel=0.0, v_lead=None):
  kw = dict(dRel=d_rel, yRel=y_rel, vRel=v_rel)
  if v_lead is not None:
    kw["vLead"] = v_lead
  return SimpleNamespace(**kw)


def test_straight_path_uses_yrel():
  # No model path: device y = −yRel. Left sign (yRel +3.2, roadside) is off path.
  assert track_is_in_path(radar(y_rel=0.4))
  assert not track_is_in_path(radar(y_rel=3.2))
  assert not track_is_in_path(radar(y_rel=-3.2))
  assert abs(path_lateral_m(radar(y_rel=3.2))) == 3.2


def test_model_path_rejects_left_sign_on_straight():
  path_x = (0.0, 20.0, 40.0, 80.0)
  path_y = (0.0, 0.0, 0.0, 0.0)
  sign = radar(d_rel=40.0, y_rel=3.2, v_rel=-20.0)  # stationary, left of path
  assert not radar_follow_ok(sign, v_ego=20.0, path_x=path_x, path_y=path_y)


def test_model_path_keeps_lead_that_follows_curve():
  # Path goes left 3 m by 40 m. On-path radar has device y = +3 → yRel = −3.
  path_x = (0.0, 20.0, 40.0, 80.0)
  path_y = (0.0, 1.5, 3.0, 4.0)
  on_path = radar(d_rel=40.0 - 1.52, y_rel=-3.0, v_rel=0.0)
  ego_center = radar(d_rel=40.0 - 1.52, y_rel=0.0, v_rel=0.0)
  assert abs(path_y_at_x(path_x, path_y, 40.0) - 3.0) < 1e-6
  assert track_is_in_path(on_path, path_x, path_y)
  assert not track_is_in_path(ego_center, path_x, path_y)


def test_oncoming_is_closing_from_ahead_not_stopped():
  v_ego = 20.0
  stopped = radar(d_rel=40.0, y_rel=0.0, v_rel=-v_ego)  # vLead = 0
  same_dir = radar(d_rel=40.0, y_rel=0.0, v_rel=-2.0)   # vLead = 18
  oncoming = radar(d_rel=40.0, y_rel=-1.5, v_rel=-(v_ego + 15.0))  # vLead = −15
  assert not track_is_oncoming(stopped, v_ego)
  assert not track_is_oncoming(same_dir, v_ego)
  assert track_is_oncoming(oncoming, v_ego)
  assert radar_follow_ok(stopped, v_ego)
  assert radar_follow_ok(same_dir, v_ego)
  assert not radar_follow_ok(oncoming, v_ego)


def test_oncoming_not_applied_at_crawl():
  oncoming = radar(d_rel=20.0, y_rel=0.0, v_rel=-8.0, v_lead=-7.0)
  assert not track_is_oncoming(oncoming, v_ego=1.0)
  assert radar_follow_ok(oncoming, v_ego=1.0)


def test_incumbent_half_width_is_wider_than_acquire():
  edge = radar(d_rel=40.0, y_rel=PATH_HALF_WIDTH_M + 0.2)
  assert not radar_follow_ok(edge, v_ego=20.0, max_lat=PATH_HALF_WIDTH_M)
  assert radar_follow_ok(edge, v_ego=20.0, max_lat=PATH_INCUMBENT_HALF_WIDTH_M)


def test_roundabout_exit_rejects_entering_cross_traffic():
  """Right-exit path; entering vehicle is not oncoming and not on the path."""
  path_x, path_y = ROUNDABOUT_EXIT_PATH_X, ROUNDABOUT_EXIT_PATH_Y
  assert abs(path_y_at_x(path_x, path_y, ROUNDABOUT_EXIT_X_M) - (-4.0)) < 1e-6

  # Looks ahead (yRel≈0), circulating ~same speed — the 20:54 max-regen lock.
  entrant = radar(d_rel=ROUNDABOUT_ENTRANT_DREL, y_rel=0.0, v_rel=-2.0)
  assert not track_is_oncoming(entrant, v_ego=12.0)
  assert not track_is_in_path(entrant, path_x, path_y)
  assert not radar_follow_ok(entrant, v_ego=12.0, path_x=path_x, path_y=path_y)
  assert abs(path_lateral_m(entrant, path_x, path_y)) >= 3.5

  # Lead already on the exit (device y = −4 → yRel = +4) stays valid.
  on_exit = radar(d_rel=ROUNDABOUT_ENTRANT_DREL, y_rel=4.0, v_rel=-1.0)
  assert track_is_in_path(on_exit, path_x, path_y)
  assert radar_follow_ok(on_exit, v_ego=12.0, path_x=path_x, path_y=path_y)


def test_curve_outside_sign_is_not_an_oncoming_lead():
  """~20:55 CT: static sign ~two lanes outside a left curve.

  Stationary (vLead≈0) is not oncoming. Bosch may also report a bogus
  closing vRel — still reject. A real lead on the curve stays valid.
  """
  path_x, path_y = CURVE_OUTSIDE_PATH_X, CURVE_OUTSIDE_PATH_Y
  path_at = path_y_at_x(path_x, path_y, CURVE_OUTSIDE_X_M)
  assert abs(path_at - 4.0) < 1e-6
  d_rel = CURVE_OUTSIDE_X_M - RADAR_TO_CAMERA_M
  # device y = path − two lanes (outside / right of a left turn)
  device_y = path_at - CURVE_OUTSIDE_TWO_LANES_M
  y_rel = -device_y
  v_ego = 16.0

  stationary = radar(d_rel=d_rel, y_rel=y_rel, v_rel=-v_ego)
  bogus_oncoming = radar(d_rel=d_rel, y_rel=y_rel, v_rel=-(v_ego + 18.0))
  assert not track_is_oncoming(stationary, v_ego)
  assert track_is_oncoming(bogus_oncoming, v_ego)
  assert abs(abs(path_lateral_m(stationary, path_x, path_y)) - CURVE_OUTSIDE_TWO_LANES_M) < 1e-6
  assert not track_is_in_path(stationary, path_x, path_y)
  assert not radar_follow_ok(stationary, v_ego, path_x, path_y)
  assert not radar_follow_ok(bogus_oncoming, v_ego, path_x, path_y)

  on_curve = radar(d_rel=d_rel, y_rel=-path_at, v_rel=-1.0)
  assert track_is_in_path(on_curve, path_x, path_y)
  assert radar_follow_ok(on_curve, v_ego, path_x, path_y)


def test_2102_perimeter_splits_near_edge_and_farther_over():
  """2.0 m perimeter: 2.23 near-edge fails; 3.2 farther-over stays off-path."""
  v_ego = 20.0
  near = radar(d_rel=40.0, y_rel=2.23, v_rel=-(v_ego + 18.8))
  far = radar(d_rel=40.0, y_rel=3.2, v_rel=-(v_ego + 20.0))
  assert not radar_follow_ok(near, v_ego, max_lat=PATH_INCUMBENT_HALF_WIDTH_M)
  assert not radar_follow_ok(far, v_ego, max_lat=PATH_INCUMBENT_HALF_WIDTH_M)
  assert abs(path_lateral_m(near)) > PATH_INCUMBENT_HALF_WIDTH_M
  assert abs(path_lateral_m(far)) > PATH_INCUMBENT_HALF_WIDTH_M


def test_straight_opposing_passers_are_not_follow_leads():
  """20:58: mid-lane opposing stays off the follow set (ignored, no brake)."""
  v_ego = 20.0
  path_x = (0.0, 40.0, 80.0)
  path_y = (0.0, 0.0, 0.0)
  for y_rel, d_rel in ((3.1, 80.0), (2.9, 50.0), (3.4, 28.0)):
    t = radar(d_rel=d_rel, y_rel=y_rel, v_rel=-(v_ego + 20.0))
    assert track_is_oncoming(t, v_ego)
    assert not track_is_in_path(t, path_x, path_y)
    assert not radar_follow_ok(t, v_ego, path_x, path_y)


def test_ep2059_near_edge_oncoming_fails_follow_ok():
  """yRel +2.23 is inside old 2.5 m; vLead oncoming + 2.0 m gate must drop it."""
  v_ego = 20.0
  near = radar(d_rel=40.0, y_rel=2.23, v_rel=-(v_ego + 18.8))
  mid = radar(d_rel=50.0, y_rel=3.2, v_rel=-(v_ego + 20.0))
  assert track_is_oncoming(near, v_ego)
  assert not radar_follow_ok(near, v_ego)
  assert not radar_follow_ok(near, v_ego, max_lat=PATH_INCUMBENT_HALF_WIDTH_M)
  assert not radar_follow_ok(mid, v_ego)


def test_vision_lead_rejects_oncoming_and_far_lateral():
  oncoming = SimpleNamespace(x=[40.0], y=[0.0], v=[ONCOMING_VLEAD_MS - 1.0])
  assert not vision_lead_follow_ok(oncoming, v_ego=20.0)
  far = SimpleNamespace(x=[40.0], y=[3.5], v=[20.0])
  assert not vision_lead_follow_ok(far, v_ego=20.0)
  ok = SimpleNamespace(x=[40.0], y=[0.3], v=[18.0])
  assert vision_lead_follow_ok(ok, v_ego=20.0)
