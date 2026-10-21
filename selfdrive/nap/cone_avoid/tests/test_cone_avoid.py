import json
import os
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.nap.cone_avoid.config import MODE_ACTIVE, MODE_LOG, ConeAvoidConfig
from openpilot.selfdrive.nap.cone_avoid.detector import (
  CAMERA, RADAR, ConeObs, camera_cone_candidates, camera_rows, fuse, obs_from_messages,
)
from openpilot.selfdrive.nap.cone_avoid.hook import ConeAvoid, offset_to_curvature
from openpilot.selfdrive.nap.cone_avoid.planner import ConeAvoidPlanner

DT = 0.01
V = 13.0  # ~29 mph
ROW_Y = -1.95  # left row: exactly 1.0 m gap to the car edge (no shift needed)


def _row(xs, y=ROW_Y, source=CAMERA):
  return [ConeObs(x, y, source, 0.9, 0.0, in_row=(source == RADAR)) for x in xs]


def _run(planner, obs_fn, secs, v=V):
  outs = []
  for i in range(int(secs / DT)):
    outs.append(planner.update(fuse(obs_fn(i * DT)), v, DT))
  return outs


def test_outlier_cone_in_row_drives_shift_then_returns():
  p = ConeAvoidPlanner()

  def obs(t):
    xs = [50 - V * t + 8 * k for k in range(-6, 8)]
    o = _row([x for x in xs if abs(x - (50 - V * t)) > 0.1])
    o.append(ConeObs(50 - V * t, ROW_Y + 0.6, CAMERA))  # one cone knocked 0.6 m inward
    return o

  outs = _run(p, obs, 9.0)
  peak = max(o.offset_m for o in outs)
  assert 0.55 <= peak <= 0.62, peak                  # ~0.6 m right, not smoothed away
  assert max(o.request_m for o in outs) == max(o.request_m for o in outs[:200])
  assert abs(outs[-1].offset_m) < 0.02                # rate-limited return to the row line
  # rate limit respected
  d = np.diff([o.offset_m for o in outs])
  assert np.max(np.abs(d)) <= 0.25 * DT + 1e-9
  assert not any(o.take_over for o in outs)


def test_lone_cone_in_lane_is_handled():
  for src in (CAMERA, RADAR):
    p = ConeAvoidPlanner()
    # one detection only, 0.3 m left of the path centre (in our lane)
    def obs(t, src=src):
      return [ConeObs(30 - V * t, -0.3, src, 0.9, 0.0, in_row=False)]
    outs = _run(p, obs, 2.0)
    if src == CAMERA:
      assert outs[0].request_m > 1.0 and outs[0].take_over       # over cap -> take over
      assert all(o.target_m == 0.0 for o in outs[:50])            # 0.3+1.0-0.95 < 0.5: alert, don't steer
    else:
      assert outs[0].low_conf and outs[0].n_cones == 1             # radar single: counted, low confidence
  # lone camera cone near the lane edge: shifts by itself (no 4-post minimum)
  p = ConeAvoidPlanner()
  outs = _run(p, lambda t: [ConeObs(30 - V * t, -1.5, CAMERA)], 1.5)
  assert abs(max(o.target_m for o in outs) - 0.45) < 1e-6
  assert not outs[0].take_over


def test_radar_ghost_logged_low_conf_not_steered():
  ca = ConeAvoid(cfg=ConeAvoidConfig())
  ca.mode = MODE_LOG
  cones = fuse([ConeObs(25.0, -1.4, RADAR, 0.4, 0.0, in_row=False)])
  assert cones[0].low_conf
  k = ca.step(0.001, cones=cones, v_ego=V, lat_active=True, blinker=False, lane_change=False,
              steering_pressed=False, torque_nm=0.0, dt=DT)
  assert k == 0.001
  assert ca.out.low_conf
  assert ca.planner.update(cones, V, DT).request_m > 0.4   # logged request
  assert ca.out.offset_m == 0.0                           # never acted on
  # camera confirmation of the same post upgrades it
  cones2 = fuse([ConeObs(25.0, -1.4, RADAR, 0.4, 0.0), ConeObs(25.3, -1.5, CAMERA)])
  assert len(cones2) == 1 and not cones2[0].low_conf


def test_corridor_midpoint_and_max_shift_param():
  p = ConeAvoidPlanner()
  out = _run(p, lambda t: [ConeObs(20, -2.2, CAMERA), ConeObs(20, 2.6, CAMERA)], 0.1)[-1]
  assert out.corridor and abs(out.target_m - 0.2) < 1e-6
  cfg = ConeAvoidConfig(max_shift_m=1.5)
  p = ConeAvoidPlanner(cfg)
  out = _run(p, lambda t: [ConeObs(20, -0.7, CAMERA)], 0.1)[-1]
  assert abs(out.target_m - 1.25) < 1e-6 and not out.take_over
  assert ConeAvoidConfig(max_shift_m=9).clamp().max_shift_m == 2.0


def test_hook_safety_gates():
  ca = ConeAvoid()
  ca.mode = MODE_ACTIVE
  cones = fuse([ConeObs(20, -1.5, CAMERA)])
  kw = dict(cones=cones, v_ego=V, blinker=False, lane_change=False, torque_nm=0.0, dt=DT)
  for _ in range(300):
    k = ca.step(0.0, lat_active=True, steering_pressed=False, **kw)
  assert 0 < k <= 0.02
  assert ca.step(0.0, lat_active=True, steering_pressed=True, **kw) == 0.0   # push cancels at once
  k1 = ca.step(0.0, lat_active=True, steering_pressed=False, **kw)
  assert k1 < k                                                              # fades back in
  assert ca.step(0.0, lat_active=False, steering_pressed=False, **kw) == 0.0
  assert ca.step(0.0, lat_active=True, steering_pressed=False, **{**kw, "blinker": True}) == 0.0
  assert ca.step(0.0, lat_active=True, steering_pressed=False, **{**kw, "lane_change": True}) == 0.0
  assert abs(offset_to_curvature(5.0, 1.0, 0.02)) == 0.02


def test_stale_cone_data_ignored():
  assert fuse([ConeObs(20, -1.0, CAMERA, age_s=0.31)]) == []
  assert len(fuse([ConeObs(20, -1.0, CAMERA, age_s=0.29)])) == 1


def test_camera_check_proposes_cones_alone():
  img = np.full((240, 320, 3), 90, np.uint8)
  img[150:170, 60:66] = (255, 120, 30)    # left post
  img[150:170, 250:256] = (255, 120, 30)  # right post
  obs = camera_cone_candidates(img, fx=300.0, cx=160.0, cy=120.0)
  assert len(obs) == 2 and obs[0].y < 0 < obs[1].y and all(o.source == CAMERA for o in obs)
  cones = fuse(obs + [ConeObs(o.x + 8, o.y, CAMERA) for o in obs])
  assert set(camera_rows(cones)) == {-1, 1}   # lines formed from the camera alone


def _msg(r):
  return SimpleNamespace(active=r["a"], count=r["n"], confidence=r["conf"], side=r["side"],
                         leftActive=r["L"], leftClear=r["Lc"], rightActive=r["R"], rightClear=r["Rc"],
                         latNear=r["near"], latMid=r["mid"], latFar=0.0)


def test_replay_route_1b8_seg11_12_log_only():
  """Oct 21 3:17-3:18 PM construction zone: Log-only would shift right and keep clearance."""
  path = os.path.join(os.path.dirname(__file__), "route_1b8_seg11_12_cones.json")
  rows = json.load(open(path))["rows"]
  ca = ConeAvoid()
  ca.mode = MODE_LOG
  log = []
  prev_t = rows[0]["t"]
  for r in rows:
    dt = max(r["t"] - prev_t, 0.0) or 0.1
    prev_t = r["t"]
    cones = fuse(obs_from_messages(_msg(r), None, 0.05, 1e9))
    k = ca.step(0.0, cones=cones, v_ego=r["v"], lat_active=bool(r["lat"]), blinker=r["bl"], lane_change=False,
                steering_pressed=r["pr"], torque_nm=r["tq"], dt=dt)
    assert k == 0.0                         # log-only never changes the command
    log.append((r["t"], ca.out, ca.planner.offset, r))
  # 15:17:36-15:17:39 (t 6-9 s): cone line active on the LEFT; plan shifts right
  win = [o for t, o, _, r in log if 6.0 <= t <= 9.0]
  assert max(o.offset_m for o in win) > 0.15
  assert all(o.offset_m >= 0.0 for t, o, _, r in log)   # never toward the posts (left)
  # 15:17:50-15:17:59 (t 20-29 s): posts left; plan right, clearance kept at the target
  win2 = [(o, r) for t, o, _, r in log if 20.0 <= t <= 29.0 and r["a"] and r["L"]]
  assert win2
  for o, _r in win2:
    assert o.clear_m >= 0.5
  assert sum(o.clear_m >= 0.99 for o, _ in win2) / len(win2) > 0.8
  # cap respected throughout
  assert max(o.offset_m for _, o, _, _ in log) <= 1.0 + 1e-9
