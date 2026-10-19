import math

from openpilot.selfdrive.nap.animal_detect.config import CAMERA_CONFIRM_FRAMES, RADAR_TO_CAMERA
from openpilot.selfdrive.nap.animal_detect.fuse import Detection, RadarPoint, Tracker, fuse, match_radar


def det(bearing=5.0, rng=50.0, cls="deer", box=(100, 100, 150, 140), cam="road", conf=0.6):
  return Detection(cls, conf, cam, box, bearing, rng)


def radar_at(bearing_deg, range_cam, tid=7, v_rel=-15.0):
  d = range_cam - RADAR_TO_CAMERA
  return RadarPoint(tid, d, -range_cam * math.tan(math.radians(bearing_deg)), v_rel)


def test_radar_bearing_sign_left_positive_yrel():
  assert RadarPoint(1, 50.0, 5.0, 0.0).bearing_deg < 0      # +yRel is left
  assert RadarPoint(1, 50.0, -5.0, 0.0).bearing_deg > 0


def test_match_inside_gates():
  p = match_radar(det(5.0, 50.0), [radar_at(6.2, 60.0)])
  assert p is not None and p.track_id == 7


def test_no_match_outside_bearing():
  assert match_radar(det(5.0, 50.0), [radar_at(6.7, 50.0)]) is None


def test_no_match_outside_range():
  assert match_radar(det(5.0, 50.0), [radar_at(5.0, 63.0)]) is None
  assert match_radar(det(5.0, 50.0), [radar_at(5.0, 37.0)]) is None


def test_nan_camera_range_never_matches():
  assert match_radar(det(5.0, float("nan")), [radar_at(5.0, 50.0)]) is None


def test_best_of_two():
  p = match_radar(det(5.0, 50.0), [radar_at(6.0, 58.0, tid=1), radar_at(5.1, 51.0, tid=2)])
  assert p.track_id == 2


def test_radar_fills_range_and_vrel():
  tr = Tracker()
  d = fuse([det(5.0, 45.0)], [radar_at(5.2, 50.0, v_rel=-12.0)], tr)[0]
  assert d.radar_matched and d.confirmed and d.radar_track_id == 7
  assert abs(d.range_m - 50.0) < 1e-3 and d.v_rel == -12.0
  assert d.lateral_m > 0


def test_camera_only_confirms_after_three_frames():
  tr = Tracker()
  for i in range(1, CAMERA_CONFIRM_FRAMES + 1):
    d = fuse([det()], [], tr)[0]
    assert d.frames == i and d.confirmed == (i >= CAMERA_CONFIRM_FRAMES)
    assert math.isnan(d.v_rel) and d.range_m == d.range_cam


def test_one_miss_allowed_two_misses_reset():
  tr = Tracker()
  fuse([det()], [], tr)
  fuse([det()], [], tr)
  fuse([], [], tr)
  assert fuse([det()], [], tr)[0].frames == 3
  fuse([], [], tr)
  fuse([], [], tr)
  assert fuse([det()], [], tr)[0].frames == 1


def test_person_and_animal_tracks_apart():
  tr = Tracker()
  fuse([det(cls="deer")], [], tr)
  d = fuse([det(cls="person")], [], tr)[0]
  assert d.frames == 1


def test_cameras_tracked_apart():
  tr = Tracker()
  fuse([det(cam="road")], [], tr)
  assert fuse([det(cam="wide")], [], tr)[0].frames == 1
