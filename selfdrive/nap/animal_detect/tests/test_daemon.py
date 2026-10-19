import math
import os

import numpy as np

from openpilot.selfdrive.nap.animal_detect import animaldetd
from openpilot.selfdrive.nap.animal_detect.animaldetd import Daemon, build_msg, cams_from_calib, detect
from openpilot.selfdrive.nap.animal_detect.config import CORES, ERROR_BACKOFF_S, TILE_H, TILE_W

CAL = (0.00086, -0.0738, -0.0217, 1.228, 0.0, 0.018, -0.026)


def frame(fid=1):
  return (np.full((1208, 1928), 60, np.uint8), np.full((604, 1928), 128, np.uint8), fid)


class StubRunner:
  """One deer box on the right road tile, at the hand-marked New Ulm spot."""
  def __init__(self, raise_on_call=False):
    self.raise_on_call = raise_on_call
    self.calls = []

  def __call__(self, tiles):
    if self.raise_on_call:
      raise RuntimeError("gpu fault")
    self.calls.append([t.shape for t in tiles])
    out = np.zeros((len(tiles), 20, 9), np.float32)
    out[1, 0, :5] = (440, 165, 100, 60, 0.9)   # tile px
    out[1, 0, 6] = 0.9                          # deer
    return out, 7.5


def test_little_cores_only():
  assert set(CORES) <= {0, 1, 2, 3}
  assert 4 not in CORES and 6 not in CORES


def test_detect_maps_to_full_frame_and_range():
  road, wide = cams_from_calib(CAL)
  r = StubRunner()
  dets, ms, _prep, n = detect({"road": frame()[:2], "wide": frame()[:2]}, {"road": road, "wide": wide}, r)
  assert n == 3 and ms == 7.5 and r.calls[0] == [(TILE_H, TILE_W, 3)] * 3
  assert len(dets) == 1
  d = dets[0]
  assert d.cls == "deer" and d.camera == "road"
  assert 1300 < 0.5 * (d.box[0] + d.box[2]) < 1460
  assert 35 < d.range_cam < 90 and d.bearing_deg > 5


def test_step_ok_and_camera_confirm():
  d = Daemon(runner_factory=StubRunner)
  for i in range(3):
    status, dets, info = d.step(float(i), {"road": frame(i), "wide": None}, None)
  assert status == "ok" and dets[0].confirmed and dets[0].frames == 3 and info["tiles"] == 2


def test_runner_error_never_raises_and_backs_off():
  d = Daemon(runner_factory=lambda: StubRunner(raise_on_call=True))
  status, dets, _ = d.step(100.0, {"road": frame(), "wide": frame()}, None)
  assert status == "model_error" and dets == [] and d.next_run == 100.0 + ERROR_BACKOFF_S
  d.step(d.next_run, {"road": frame(), "wide": frame()}, None)
  assert d.err_backoff == 4 * ERROR_BACKOFF_S


def test_missing_model_is_not_a_crash():
  def boom():
    raise FileNotFoundError("no pkl")
  d = Daemon(runner_factory=boom)
  status, dets, _ = d.step(0.0, {"road": frame()}, None)
  assert status == "no_model" and dets == [] and d.next_run == 5.0


def test_no_frame():
  d = Daemon(runner_factory=StubRunner)
  assert d.step(0.0, {"road": None, "wide": None}, None)[0] == "no_frame"


def test_bad_radar_message_ignored():
  d = Daemon(runner_factory=StubRunner)
  status, dets, _ = d.step(0.0, {"road": frame()}, object())
  assert status == "ok" and not dets[0].radar_matched


def test_model_drop_backoff():
  d = Daemon(runner_factory=StubRunner)
  d.on_model_drop(10.0)
  assert d.next_run >= 11.0


def _schema():
  try:
    from cereal import custom
    return custom
  except Exception:
    import capnp
    here = os.path.dirname(os.path.abspath(__file__))
    cer = os.path.normpath(os.path.join(here, "../../../../cereal"))
    return capnp.load(os.path.join(cer, "custom.capnp"), imports=[cer, os.path.join(cer, "include")])


class _FakeMessaging:
  def __init__(self, custom):
    self.custom = custom

  def new_message(self, name, valid=True):
    class _M:
      pass
    m = _M()
    setattr(m, name, self.custom.AnimalDetectNAP.new_message())
    return m


def test_build_msg_roundtrip():
  custom = _schema()
  d = Daemon(runner_factory=StubRunner)
  status, dets, info = d.step(0.0, {"road": frame(42)}, None)
  msg = build_msg(_FakeMessaging(custom), dets, status=status, **info)
  b = msg.animalDetectNAP
  assert b.status == "ok" and b.frameId == 42 and b.tilesRun == 2 and len(b.detections) == 1
  e = b.detections[0]
  assert str(e.cls) == "deer" and str(e.camera) == "road" and not e.radarMatched and math.isnan(e.vRel)
  b2 = build_msg(_FakeMessaging(custom), [], status="weird").animalDetectNAP
  assert b2.status == "model_error" and len(b2.detections) == 0


def test_daemon_publishes_only_its_own_message():
  src = open(animaldetd.__file__).read()
  assert 'PubMaster(["animalDetectNAP"])' in src
  for banned in ("carControl", "longitudinalPlan", "selfdriveState", "onroadEvents", "EventName", "AudibleAlert"):
    assert banned not in src
