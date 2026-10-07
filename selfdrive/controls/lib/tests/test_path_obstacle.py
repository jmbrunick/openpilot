"""Radar obstacle trigger, projection, fusion, and the animal/person chime."""
from __future__ import annotations

import time
import wave
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.controls.lib.path_obstacle import (
  LIVING_CLASSES,
  PARAM_OBSTACLE_CHIME,
  PARAM_OBSTACLE_LOG,
  BrakeIntent,
  ChimeGate,
  ConeHint,
  ObstacleSample,
  PathObstacleDetector,
  VisionScore,
  chime_banner,
  fuse_scores,
  fusion_weights,
  future_brake_intent,
  lively_from_history,
  patch_change_score,
  project_road_point,
  scale_living_vision,
  prune_thumbs,
  save_ppm,
  should_raise_chime,
)
from openpilot.selfdrive.ui.obstacle_chime import OBSTACLE_CHIME_ID, alert_sound_id

ROOT = Path(__file__).resolve().parents[4]
V_EGO = 12.0
PATH_X = [0.0, 100.0]
PATH_Y = [0.0, 0.0]
ANIMAL = VisionScore(conf=0.70, human=0.10, animal=0.82, obstacle=0.18)
HUMAN = VisionScore(conf=0.72, human=0.64, animal=0.10, obstacle=0.14)
DEBRIS = VisionScore(conf=0.80, human=0.08, animal=0.10, obstacle=0.70)
WEAK = VisionScore(conf=0.20, human=0.90, animal=0.05, obstacle=0.05)


def _pt(x, y_rel, v_rel, tid, yv_rel=0.0, rcs=5.0):
  return SimpleNamespace(
    dRel=x - 1.52, yRel=y_rel, vRel=v_rel, measured=True,
    trackId=tid, yvRel=yv_rel, rcs=rcs,
  )


def _run(det, points, n=4, dt=0.1, vision=None, chime_enabled=True, now0=100.0, **kw):
  sample = None
  for i in range(n):
    sample = det.update(
      points, V_EGO, PATH_X, PATH_Y, dt,
      vision=vision, chime_enabled=chime_enabled, now=now0 + i * dt, **kw,
    )
  return sample


def _agreed(cls, zone, tid):
  return ObstacleSample(
    active=True, track_id=tid, object_class=cls, zone=zone, agree=True,
    member_ids=(tid,), reject_reason="none",
  )


def test_projection_matches_identity_and_camera():
  ground = project_road_point(20, 0, 0)
  assert ground is not None
  assert abs(ground[0] - 672.0) < 1e-6
  assert abs(ground[1] - (380.0 + 1141.5 * 1.22 / 20.0)) < 1e-4
  top = project_road_point(20, 0, 1.0)
  assert top is not None
  assert abs(top[1] - (380.0 + 1141.5 * 0.22 / 20.0)) < 1e-4
  left = project_road_point(20, 2.0, 0)
  assert left is not None
  assert abs(left[0] - (672.0 - 1141.5 * 2.0 / 20.0)) < 1e-4

  from openpilot.common.transformations.camera import get_view_frame_from_road_frame
  view = get_view_frame_from_road_frame(0.0, 0.0, 0.0, 1.22).dot(np.array([20.0, 0.0, 0.0, 1.0]))
  u = 672.0 + 1141.5 * (view[0] / view[2])
  v = 380.0 + 1141.5 * (view[1] / view[2])
  assert abs(u - ground[0]) < 1e-4
  assert abs(v - ground[1]) < 1e-4


def test_fusion_weights_are_smooth_and_nan_vision_does_not_agree():
  good = fusion_weights(1.0)
  mid = fusion_weights(0.5)
  poor = fusion_weights(0.0)
  assert abs(good[0] - 0.50) < 1e-9 and abs(good[1] - 0.50) < 1e-9
  assert abs(mid[0] - 0.64) < 1e-9 and abs(mid[1] - 0.36) < 1e-9
  assert abs(poor[0] - 0.78) < 1e-9 and abs(poor[1] - 0.22) < 1e-9
  assert poor[0] > mid[0] > good[0]
  fused, _wr, _wv, agree = fuse_scores(0.80, None, 1.0)
  assert agree is False and abs(fused - 0.80) < 1e-9
  _f, _wr, _wv, agree = fuse_scores(0.80, float("nan"), 0.0)
  assert agree is False
  _f, _wr, _wv, agree = fuse_scores(0.60, 0.50, 1.0)
  assert agree is True
  assert fuse_scores(0.50, 0.50, 1.0)[3] is False
  assert fuse_scores(0.60, 0.40, 1.0)[3] is False


def test_stationary_in_path_deer_chimes_only_when_vision_says_animal():
  deer = [_pt(30.0, 0.0, -V_EGO, 3)]
  bare = _run(PathObstacleDetector(), deer, vision=None)
  assert bare.vision_evaluated is False
  assert bare.chimed is False
  assert bare.object_class == "unknown"

  seen = _run(PathObstacleDetector(), deer, vision=ANIMAL)
  assert seen.active and seen.zone == "inPath"
  assert seen.object_class == "animal"
  assert seen.lively_score < 0.2
  assert 0.25 < seen.vision_conf_animal < 0.55
  assert seen.agree and seen.vision_evaluated
  assert seen.chimed and seen.chime_reason == "chimed"
  assert seen.brake_gate is True
  assert future_brake_intent(seen) is not None

  debris = _run(PathObstacleDetector(), deer, vision=DEBRIS)
  assert debris.object_class == "obstacle"
  assert debris.chimed and debris.chime_reason == "chimed"
  assert debris.brake_gate is True


def test_human_entering_and_cyclist_chime_and_can_brake_later():
  walker = [_pt(25.0, -2.3, -V_EGO, 8, yv_rel=1.0)]
  person = _run(PathObstacleDetector(), walker, vision=HUMAN)
  assert person.object_class == "human"
  assert person.zone == "entering"
  assert person.agree and person.chimed
  assert person.brake_gate is True
  intent = future_brake_intent(person)
  assert isinstance(intent, BrakeIntent) and intent.allowed and intent.decel == 0.0
  assert chime_banner(person.object_class, person.zone) == "Object ahead"

  weak = _run(PathObstacleDetector(), walker, vision=WEAK)
  assert weak.object_class == "human"
  assert weak.chimed is False and weak.chime_reason == "no_agree"

  cyclist = [_pt(22.0, 0.0, 6.0 - V_EGO, 9)]
  rider = _run(PathObstacleDetector(), cyclist, vision=HUMAN)
  assert rider.object_class == "human" and rider.zone == "inPath" and rider.chimed
  lead = _run(PathObstacleDetector(), cyclist, vision=HUMAN, lead_ids=(9,))
  assert lead.reject_reason == "lead" and lead.chimed is False


def test_rejects_clutter_vehicles_and_a_lane_spanning_tree():
  det = PathObstacleDetector()
  assert _run(det, [_pt(30.0, 0.0, -V_EGO, 4)], lead_ids=(4,)).reject_reason == "lead"
  cones = ConeHint(active=True, side=1, lat_near=2.0, lat_mid=2.0, lat_far=2.0)
  assert _run(PathObstacleDetector(), [_pt(20.0, -2.0, -V_EGO, 5)], cone=cones).reject_reason == "cone_line"
  rail = ConeHint(barrier=True, side=-1, lat_near=-3.0, lat_mid=-3.0, lat_far=-3.0)
  assert _run(PathObstacleDetector(), [_pt(20.0, 3.0, -V_EGO, 6)], cone=rail).reject_reason == "barrier"
  parked = ConeHint(parked=True)
  assert _run(PathObstacleDetector(), [_pt(18.0, -3.2, -V_EGO, 7)], cone=parked).reject_reason == "parked"
  assert _run(PathObstacleDetector(), [_pt(40.0, 0.0, 18.0 - V_EGO, 11)]).reject_reason == "vehicle"
  assert _run(PathObstacleDetector(), [_pt(10.0, 0.0, -V_EGO, 12)]).reject_reason == "road_surface"
  assert _run(PathObstacleDetector(), [_pt(40.0, -15.0, -V_EGO, 13)]).reject_reason == "off_path"
  assert _run(PathObstacleDetector(), [_pt(8.0, -12.0, -V_EGO, 14)]).reject_reason == "fov"

  tree = [
    _pt(28.0, 1.2, -V_EGO, 21),
    _pt(28.0, 0.0, -V_EGO, 22),
    _pt(28.0, -1.2, -V_EGO, 23),
  ]
  fallen = _run(PathObstacleDetector(), tree, vision=ANIMAL)
  assert fallen.object_class == "obstacle"
  assert fallen.cluster_count == 3 and fallen.span_m >= 1.8
  assert fallen.agree and fallen.chimed and fallen.chime_reason == "chimed"
  assert fallen.brake_gate is True
  assert future_brake_intent(fallen) is not None

  curtain = [
    _pt(40.0, 5.0, -V_EGO, 31),
    _pt(40.0, 2.0, -V_EGO, 32),
    _pt(40.0, -1.0, -V_EGO, 33),
    _pt(40.0, -4.0, -V_EGO, 34),
  ]
  assert _run(PathObstacleDetector(), curtain).reject_reason == "overhead"


def test_still_mailbox_stays_quiet_and_a_moving_person_chimes():
  mailbox = VisionScore(conf=0.70, human=0.64, animal=0.10, obstacle=0.16)
  shoulder = [_pt(24.0, -4.5, -V_EGO, 41)]
  still = _run(PathObstacleDetector(), shoulder, n=16, vision=mailbox)
  assert still.zone == "roadside"
  assert still.lively_score < 0.15
  assert still.object_class != "human"
  assert still.chimed is False
  assert still.vision_conf_human > 0.15

  walking = [_pt(24.0, -4.5, 1.6 - V_EGO, 42)]
  person = _run(PathObstacleDetector(), walking, n=8, vision=mailbox)
  assert person.zone == "roadside" and person.object_class == "human"
  assert person.lively_score > 0.8
  assert person.agree and person.chimed and person.chime_reason == "chimed"
  assert person.brake_gate is False
  assert future_brake_intent(person) is None
  assert chime_banner("human", "roadside") == "Object near road"
  assert chime_banner(1, 3) == "Object near road"
  assert chime_banner("obstacle", "inPath") == "Object ahead"
  assert chime_banner("unknown", "entering") == "Object ahead"

  quiet = _run(PathObstacleDetector(), shoulder, n=8, vision=WEAK)
  assert quiet.chimed is False
  assert quiet.chime_reason == "inanimate"


def test_wander_and_patch_change_raise_lively_without_zeroing_a_freeze():
  frozen = scale_living_vision(ANIMAL, 0.0)
  assert frozen.animal > 0.30
  assert frozen.animal < ANIMAL.animal
  moving = scale_living_vision(ANIMAL, 1.0)
  assert abs(moving.animal - ANIMAL.animal) < 1e-9

  assert lively_from_history([(0.0, 20.0, 0.0, 0.0, 0.0), (1.5, 20.1, 0.05, 0.0, 0.0)]) < 0.05
  assert lively_from_history([(0.0, 20.0, 0.0, 1.6, 0.0)]) > 0.8
  assert lively_from_history([(0.0, 20.0, 0.0, 0.0, 1.2)]) > 0.8
  wandered = [(i * 0.1, 24.0, 4.0 + 0.12 * i, 0.0, 0.0) for i in range(16)]
  assert lively_from_history(wandered) > 0.4

  det = PathObstacleDetector()
  sample = None
  chimed = False
  for i in range(16):
    y_rel = -4.0 - 0.12 * i
    sample = det.update([_pt(24.0, y_rel, -V_EGO, 43)], V_EGO, PATH_X, PATH_Y, 0.1, vision=HUMAN, now=200.0 + i * 0.1)
    chimed = chimed or bool(sample.chimed)
  assert sample is not None and chimed
  assert sample.lively_score > 0.4 and sample.object_class == "human"

  still_patch = [[40, 40, 40], [40, 42, 40], [40, 40, 40]]
  moved_patch = [[40, 180, 40], [180, 40, 180], [40, 180, 40]]
  assert patch_change_score(still_patch, still_patch) == 0.0
  assert patch_change_score(still_patch, moved_patch) > 0.5
  boosted = _run(PathObstacleDetector(), [_pt(24.0, -4.5, -V_EGO, 44)], n=8, vision=HUMAN, vision_motion=1.0)
  assert boosted.lively_score > 0.8 and boosted.object_class == "human" and boosted.chimed


def test_chime_hold_cooldown_and_shared_classes():
  det = PathObstacleDetector()
  deer = [_pt(30.0, 0.0, -V_EGO, 3)]
  first = _run(det, deer, n=4, vision=ANIMAL, now0=100.0)
  assert first.chimed
  held = _run(det, deer, n=1, vision=ANIMAL, now0=100.4)
  assert held.chimed and held.chime_reason == "chimed"
  later = _run(det, deer, n=1, vision=ANIMAL, now0=100.8)
  assert later.chimed is False and later.chime_reason == "already_tracked"

  gate = ChimeGate()
  assert gate.consider(_agreed("animal", "inPath", 1), True, 10.0) == (True, "chimed")
  assert gate.consider(_agreed("animal", "inPath", 1), True, 10.2) == (True, "chimed")
  assert gate.consider(_agreed("animal", "inPath", 1), True, 10.5) == (False, "already_tracked")
  assert gate.consider(_agreed("human", "entering", 2), True, 12.0) == (False, "cooldown")
  assert gate.consider(_agreed("human", "roadside", 2), True, 19.0) == (True, "chimed")
  assert gate.consider(_agreed("animal", "inPath", 1), True, 40.0) == (False, "already_tracked")

  shared = ChimeGate()
  assert shared.consider(_agreed("human", "inPath", 5), True, 0.0)[0] is True
  assert shared.consider(_agreed("animal", "roadside", 6), True, 1.0) == (False, "cooldown")

  muted = ChimeGate()
  assert muted.consider(_agreed("human", "inPath", 7), False, 0.0) == (False, "disabled")
  assert muted.consider(_agreed("human", "inPath", 7), True, 0.0) == (True, "chimed")

  off = _run(PathObstacleDetector(), deer, vision=ANIMAL, chime_enabled=False)
  assert off.chime_reason == "disabled" and off.chimed is False
  assert should_raise_chime(True, "human") and should_raise_chime(True, "obstacle")
  assert should_raise_chime(True, "unknown")
  assert not should_raise_chime(False, "human")
  assert LIVING_CLASSES == frozenset({"animal", "human"})
  gate_still = ChimeGate()
  assert gate_still.consider(_agreed("obstacle", "roadside", 8), True, 0.0) == (False, "inanimate")
  assert gate_still.consider(_agreed("unknown", "inPath", 8), True, 0.0) == (True, "chimed")
  lively_post = ObstacleSample(
    active=True, track_id=9, object_class="obstacle", zone="roadside", agree=True,
    member_ids=(9,), reject_reason="none", lively_score=0.8,
  )
  assert ChimeGate().consider(lively_post, True, 0.0) == (True, "chimed")


def test_future_brake_is_any_agreed_object_in_or_entering():
  assert future_brake_intent(_agreed("animal", "inPath", 1)) is not None
  assert future_brake_intent(_agreed("obstacle", "inPath", 1)) is not None
  assert future_brake_intent(_agreed("unknown", "entering", 1)) is not None
  assert future_brake_intent(_agreed("human", "roadside", 1)) is None
  quiet = ObstacleSample(active=True, agree=False, zone="inPath", object_class="obstacle", member_ids=(1,))
  assert future_brake_intent(quiet) is None


def test_alert_sound_id_and_unique_wav():
  assert alert_sound_id(2, "obstacleChime/permanent") == OBSTACLE_CHIME_ID
  assert alert_sound_id(2, "") == 2
  assert OBSTACLE_CHIME_ID < 0
  path = ROOT / "selfdrive/assets/sounds/animal_chime.wav"
  with wave.open(str(path), "rb") as handle:
    assert handle.getnchannels() == 1
    assert handle.getsampwidth() == 2
    assert handle.getframerate() == 48000
    frames = handle.getnframes()
    duration = frames / 48000.0
    pcm = np.frombuffer(handle.readframes(frames), dtype=np.int16)
  assert 0.35 <= duration <= 0.70
  spec = np.abs(np.fft.rfft(pcm.astype(np.float64)))
  freqs = np.fft.rfftfreq(len(pcm), 1.0 / 48000.0)
  peak = float(freqs[int(spec.argmax())])
  assert 350.0 < peak < 650.0
  high = spec[(freqs > 1500.0) & (freqs < 1800.0)].sum()
  assert high < 0.05 * spec.sum()
  low = spec[(freqs > 360.0) & (freqs < 430.0)].sum()
  mid = spec[(freqs > 540.0) & (freqs < 640.0)].sum()
  assert low > 0.02 * spec.sum() and mid > 0.02 * spec.sum()


def test_thumbnail_prune_and_scan_cost():
  folder = ROOT / "selfdrive/controls/lib/tests/_thumb_tmp"
  folder.mkdir(exist_ok=True)
  try:
    for i in range(30):
      assert save_ppm(str(folder / f"{i:02d}.ppm"), [[i, 255 - i], [10, 20]])
    prune_thumbs(str(folder), max_files=24, max_bytes=2_000_000)
    assert len(list(folder.glob("*.ppm"))) <= 24
  finally:
    for path in folder.glob("*.ppm"):
      path.unlink()
    folder.rmdir()

  det = PathObstacleDetector()
  points = [
    _pt(8.0 + (i % 20) * 4.0, ((i % 9) - 4) * 0.4, -V_EGO, i + 1, yv_rel=0.05 * ((i % 5) - 2))
    for i in range(40)
  ]
  n = 50
  t0 = time.perf_counter()
  for i in range(n):
    det.update(points, V_EGO, PATH_X, PATH_Y, 0.05, now=float(i))
  mean_ms = (time.perf_counter() - t0) / n * 1000.0
  print(f"path obstacle scan mean {mean_ms:.3f} ms over {n} frames of 40 tracks")
  assert mean_ms < 5.0


def test_wiring_stays_off_the_control_core_and_off_longitudinal():
  def text(rel):
    return (ROOT / rel).read_text(encoding="utf-8")

  for rel in (
    "selfdrive/controls/controlsd.py",
    "selfdrive/controls/lib/longcontrol.py",
    "selfdrive/controls/lib/longitudinal_planner.py",
  ):
    src = text(rel)
    assert "pathObstacle" not in src
    assert "obstacleChime" not in src
    assert "future_brake_intent" not in src
  lib = text("selfdrive/controls/lib/path_obstacle.py")
  assert "actuators" not in lib
  assert PARAM_OBSTACLE_CHIME == "NAPObstacleChime"
  assert PARAM_OBSTACLE_LOG == "NAPObstacleLog"

  events = text("selfdrive/selfdrived/events.py")
  block = events.split("_obstacle_chime", 1)[1][:600]
  assert "ET.PERMANENT" in block
  assert "ET.WARNING" not in block
  assert "NO_ENTRY" not in block
  assert "SOFT_DISABLE" not in block
  assert "IMMEDIATE_DISABLE" not in block
  assert "Person ahead" not in events
  assert "Object ahead" in events
  assert "obstacle_chime_alert" in events

  selfd = text("selfdrive/selfdrived/selfdrived.py")
  assert "'pathObstacleNAP'" in selfd
  assert "should_raise_chime" in selfd
  assert "a_cmd=float(self.sm['carControl'].actuators.accel),\n" in selfd

  keys = text("common/params_keys.h")
  assert '{"NAPObstacleChime", {PERSISTENT, BOOL, "1"}}' in keys
  assert '{"NAPObstacleLog", {PERSISTENT, BOOL, "1"}}' in keys
  services = text("cereal/services.py")
  assert '"pathObstacleNAP": (True, 8., 2)' in services
  manner = text("selfdrive/ui/layouts/settings/driving_mannerisms.py")
  assert "Live object detection chime" in manner
  assert "livelyScore" in text("cereal/custom.capnp")
  assert "self._all_items.append(self._turn_in_buttons)" in manner

  proc = text("selfdrive/pathobstacled/pathobstacled.py")
  assert "CORES = [0, 1, 2, 3]" in proc
  assert "set_core_affinity(CORES)" in proc
  assert "os.nice(NICE)" in proc
  assert "NICE = 19" in proc
  assert "config_realtime_process(4" not in proc
  cfg = text("system/manager/process_config.py")
  assert 'PythonProcess("pathobstacled"' in cfg
  assert "optional=True" in cfg.split('PythonProcess("pathobstacled"', 1)[1][:240]
  optional = text("system/manager/optional_procs.py")
  assert "pathobstacled" in optional
  assert '"card"' not in optional or "card" not in optional.split("OPTIONAL_PROCESS_NAMES", 1)[1][:200]
  onroad = text("selfdrive/test/test_onroad.py")
  assert '"selfdrive.pathobstacled.pathobstacled": 6.0' in onroad
