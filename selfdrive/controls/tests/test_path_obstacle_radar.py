"""radard fold-in for the path obstacle detector.

radarState bytes must not move when the scan is on, and a detector that
throws must not stop radarState. The camera helper must not open VisionIPC
until a candidate is active.
"""
from __future__ import annotations

import math
import sys
import time
import types
from types import SimpleNamespace

import numpy as np


def _install_stubs() -> None:
  """Let these tests import radard without a built msgq or params_pyx."""
  try:
    import openpilot.common.params_pyx  # noqa: F401
  except Exception:
    mod = types.ModuleType("openpilot.common.params_pyx")

    class Params:
      def __init__(self, *args, **kwargs):
        pass

      def get_bool(self, key, block=False):
        return True

      def get(self, *args, **kwargs):
        return None

      def put(self, *args, **kwargs):
        pass

      def put_bool(self, *args, **kwargs):
        pass

      def remove(self, *args, **kwargs):
        pass

    mod.Params = Params
    mod.ParamKeyFlag = SimpleNamespace(PERSISTENT=1)
    mod.ParamKeyType = SimpleNamespace(BOOL=1)
    mod.UnknownKeyName = type("UnknownKeyName", (Exception,), {})
    sys.modules["openpilot.common.params_pyx"] = mod

  try:
    from msgq.ipc_pyx import Context  # noqa: F401
    return
  except Exception:
    pass
  for key in list(sys.modules):
    if key == "msgq" or key.startswith("msgq."):
      del sys.modules[key]

  msgq = types.ModuleType("msgq")

  class _Sock:
    def connect(self, *args, **kwargs):
      pass

    def setTimeout(self, *args, **kwargs):
      pass

    def receive(self, non_blocking=False):
      return None

    def send(self, data):
      pass

  class Poller:
    def registerSocket(self, sock):
      pass

    def poll(self, timeout):
      return []

  class SocketEventHandle:
    def __init__(self, *args, **kwargs):
      self.enabled = False

  msgq.Context = type("Context", (), {})
  msgq.Poller = Poller
  msgq.SubSocket = _Sock
  msgq.PubSocket = _Sock
  msgq.SocketEventHandle = SocketEventHandle
  msgq.MultiplePublishersError = type("MultiplePublishersError", (Exception,), {})
  msgq.IpcError = type("IpcError", (Exception,), {})
  msgq.toggle_fake_events = lambda *a, **k: None
  msgq.set_fake_prefix = lambda *a, **k: None
  msgq.get_fake_prefix = lambda: b""
  msgq.delete_fake_prefix = lambda *a, **k: None
  msgq.wait_for_one_event = lambda *a, **k: None
  msgq.context = msgq.Context()

  def pub_sock(endpoint, segment_size=0):
    return _Sock()

  def sub_sock(endpoint, poller=None, addr="127.0.0.1", conflate=False, timeout=None, segment_size=0):
    sock = _Sock()
    if poller is not None:
      poller.registerSocket(sock)
    return sock

  def drain_sock_raw(sock, wait_for_one=False):
    return []

  def fake_event_handle(endpoint, identifier=None, override=True, enable=False):
    return SocketEventHandle()

  msgq.pub_sock = pub_sock
  msgq.sub_sock = sub_sock
  msgq.drain_sock_raw = drain_sock_raw
  msgq.fake_event_handle = fake_event_handle
  msgq.NO_TRAVERSAL_LIMIT = 2**64 - 1
  sys.modules["msgq"] = msgq
  sys.modules["msgq.ipc_pyx"] = msgq


_install_stubs()

from openpilot.selfdrive.controls.lib.path_obstacle import (  # noqa: E402
  BUDGET_S,
  MAX_POINTS,
  ObstacleStage,
  PathObstacleDetector,
  _Hit,
  hit_from_msg,
  hit_to_msg,
)
from openpilot.selfdrive.controls.radard import RadarD  # noqa: E402
from openpilot.selfdrive.controls.tests.test_radard import RadarScenario  # noqa: E402
from openpilot.selfdrive.pathobstacled.pathobstacled import Helper, _crop  # noqa: E402
import cereal.messaging as messaging  # noqa: E402


class _Capture:
  def __init__(self):
    self.sent = []

  def send(self, name, msg):
    self.sent.append((name, bytes(msg.to_bytes()), bool(msg.valid)))


def _radar_bytes(rd: RadarD):
  state = rd.radar_state
  raw = bytes(state.to_bytes())
  clear = getattr(state, "clear_write_flag", None)
  if clear is not None:
    clear()
  return bool(rd.radar_state_valid), raw


def _drive(enabled: bool):
  scenario = RadarScenario(v_ego=20.0)
  scenario.radar._obstacle._log_override = enabled
  real_stash = scenario.radar._stash_obstacle

  def wrapped(*args, **kwargs):
    before_stash = _radar_bytes(scenario.radar)
    real_stash(*args, **kwargs)
    after_stash = _radar_bytes(scenario.radar)
    assert before_stash == after_stash, "obstacle stash changed radarState"

  scenario.radar._stash_obstacle = wrapped
  pm = _Capture()
  frames = []
  samples = [
    (30.0, [(7, 30.0, 0.0, 0.0), (9, 18.0, -0.2, -20.0)]),
    (30.2, [(7, 30.1, 0.0, 0.0), (9, 17.4, -0.1, -20.0)]),
    (28.0, [(7, 28.0, 0.1, -0.4), (4, 22.0, 1.2, -19.5)]),
    (40.0, None),
    (36.0, [(11, 16.0, 0.2, -19.0)]),
  ]
  t = 1.0
  for _repeat in range(3):
    for vision, points in samples:
      scenario.step(t, vision_d_rel=vision, radar_points=points)
      before = _radar_bytes(scenario.radar)
      scenario.radar.run_obstacle(pm)
      after = _radar_bytes(scenario.radar)
      assert before == after, "run_obstacle changed radarState"
      frames.append(before)
      t += 0.05
  return frames, pm


def test_radar_state_bytes_match_with_detector_on_or_off():
  on, pm_on = _drive(True)
  off, _pm_off = _drive(False)
  assert on == off
  assert any(name == "pathObstacleNAP" and valid for name, _raw, valid in pm_on.sent)


def test_raising_detector_still_publishes_radar_state_and_breaker_trips():
  scenario = RadarScenario()
  scenario.radar._obstacle._log_override = True
  calls = {"n": 0}

  def boom(*args, **kwargs):
    calls["n"] += 1
    raise RuntimeError("obstacle scan failed")

  scenario.radar._obstacle.det.begin = boom
  pm = _Capture()
  for i in range(6):
    lead = scenario.step(1.0 + i * 0.1, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
    assert scenario.radar.radar_state is not None
    assert lead.status
    scenario.radar.publish(pm)
    scenario.radar.run_obstacle(pm)
  assert calls["n"] == 3
  assert scenario.radar._obstacle._disabled
  radar_sends = [item for item in pm.sent if item[0] == "radarState"]
  assert len(radar_sends) == 6


def test_past_deadline_is_over_budget_and_stage_skips():
  det = PathObstacleDetector()
  points = [SimpleNamespace(dRel=20.0, yRel=0.0, vRel=-20.0, measured=True, trackId=1, yvRel=0.0, rcs=5.0)]
  hit = det.begin(points, 20.0, [0.0, 50.0], [0.0, 0.0], 0.08, deadline=time.monotonic() - 1.0)
  assert hit.reject_reason == "over_budget"
  assert hit.active is False

  stage = ObstacleStage()
  stage._log_override = True
  calls = {"n": 0}
  real_begin = stage.det.begin

  def slow_begin(*args, **kwargs):
    calls["n"] += 1
    time.sleep(0.009)
    return stage.det._miss([], "over_budget", 1.0)

  stage.det.begin = slow_begin
  now = time.monotonic()
  hit = stage.step(points, 20.0, [0.0, 50.0], [0.0, 0.0], 0.08, [], None, [], now)
  assert hit is not None and hit.reject_reason == "over_budget"
  assert stage._skip >= 1
  assert stage._skip == max(1, math.ceil(0.009 / BUDGET_S)) or stage._skip >= 2
  skipped = stage._skip
  for i in range(skipped):
    stage.step(points, 20.0, [0.0, 50.0], [0.0, 0.0], 0.08, [], None, [], now + i + 1)
  assert calls["n"] == 1
  stage.det.begin = real_begin


def test_hit_round_trip():
  hit = _Hit(
    True, 7, (3, 11), 20.0, 1.25, -0.4, -2.0, 0.3, 1.2, 0.8,
    "animal", "entering", 4.0, 1.5, 2, 0.6, "none", 0.7, 0.4,
  )
  msg = messaging.new_message("pathObstacleNAP", valid=True)
  hit_to_msg(hit, msg.pathObstacleNAP)
  msg.pathObstacleNAP.laneProbMin = 0.42
  msg.pathObstacleNAP.pathYStd3s = 0.9
  back = hit_from_msg(msg)
  assert back.active == hit.active
  assert back.track_id == hit.track_id
  assert back.member_ids == hit.member_ids
  assert back.radar_class == hit.radar_class
  assert back.zone == hit.zone
  assert back.reject_reason == hit.reject_reason
  assert back.cluster_count == hit.cluster_count
  for got, want in (
    (back.x, hit.x), (back.y, hit.y), (back.lateral, hit.lateral),
    (back.v_rel, hit.v_rel), (back.v_lat, hit.v_lat), (back.along, hit.along),
    (back.radar_conf, hit.radar_conf), (back.time_to_reach, hit.time_to_reach),
    (back.time_to_enter, hit.time_to_enter), (back.span_m, hit.span_m),
    (back.lighting, hit.lighting), (back.lively, hit.lively),
  ):
    assert math.isclose(got, want, rel_tol=0.0, abs_tol=1e-5)
  assert bool(msg.valid)


def test_helper_without_trigger_does_not_open_visionipc():
  opened = []

  class VisionIpcClient:
    def __init__(self, *args, **kwargs):
      opened.append(args)
      raise AssertionError("VisionIPC opened")

  vipc = types.ModuleType("msgq.visionipc")
  vipc.VisionIpcClient = VisionIpcClient
  vipc.VisionStreamType = SimpleNamespace(VISION_STREAM_ROAD=1, VISION_STREAM_WIDE_ROAD=2)
  sys.modules["msgq.visionipc"] = vipc

  helper = Helper()
  assert helper.on_radar(None, time.monotonic()) is None
  idle = messaging.new_message("pathObstacleNAP", valid=True)
  idle.pathObstacleNAP.active = False
  idle.pathObstacleNAP.heartbeat = True
  assert helper.on_radar(idle, time.monotonic()) is None
  assert opened == []
  assert helper.cams.road is None and helper.cams.wide is None


def test_crop_is_detached_from_the_camera_buffer():
  plane = np.arange(400, dtype=np.uint8).reshape(20, 20)
  before = plane.copy()
  patch, road = _crop(plane, (2.2, 2.2, 12.8, 14.4))
  assert patch is not None and road is not None
  patch[:] = 3
  road[:] = 9
  assert np.array_equal(plane, before)


def test_scan_timing_on_a_full_frame():
  """Print p50/p99 for the scan radard runs. Hard budget is 4 ms."""
  det = PathObstacleDetector()
  points = [
    SimpleNamespace(
      dRel=8.0 + (i % 20) * 4.0,
      yRel=((i % 9) - 4) * 0.4,
      vRel=-20.0,
      measured=True,
      trackId=i + 1,
      yvRel=0.05 * ((i % 5) - 2),
      rcs=5.0,
    )
    for i in range(MAX_POINTS)
  ]
  path_x = [0.0, 10.0, 20.0, 40.0, 80.0]
  path_y = [0.0, 0.0, 0.0, 0.0, 0.0]
  # Warm the tracks so the timed calls include history, not first-seen setup only.
  for _ in range(4):
    det.begin(points, 20.0, path_x, path_y, 0.08)
  samples = []
  for _ in range(80):
    t0 = time.perf_counter()
    det.begin(points, 20.0, path_x, path_y, 0.08, deadline=time.monotonic() + BUDGET_S)
    samples.append((time.perf_counter() - t0) * 1000.0)
  samples.sort()
  p50 = samples[len(samples) // 2]
  p99 = samples[int(len(samples) * 0.99)]
  print(f"radard obstacle scan n={MAX_POINTS} p50 {p50:.3f} ms p99 {p99:.3f} ms max {samples[-1]:.3f} ms")
  assert p50 < BUDGET_S * 1000.0
