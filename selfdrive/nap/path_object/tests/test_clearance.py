import math
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.nap.path_object import clearance as C
from openpilot.selfdrive.controls.lib.path_object_response import confirmed_from_msg

F = 2648.0
CAM_H = 1.22
HZN = 600.0


def _frame(d, bar_h0, bar_h1, w=1928, h=1208, seed=0):
  """Flat-grey road scene with one bright horizontal bar between two heights at range d."""
  rng = np.random.default_rng(seed)
  img = np.full((h, w), 60, np.float32) + rng.normal(0, 1.0, (h, w))
  ppm = F / d
  vg = HZN + F * CAM_H / d
  top, bot = int(vg - bar_h1 * ppm), int(vg - bar_h0 * ppm)
  img[max(0, top):max(0, bot), :] = 200
  return img.clip(0, 255).astype(np.uint8), vg, ppm


def _run(bar, ranges, daylight=True, tid=7):
  g = C.ClearanceGate()
  out = []
  for i, d in enumerate(ranges):
    img, vg, ppm = _frame(d, *bar, seed=i)
    prof = C.edge_profile(img, 964.0, vg, ppm, F / d)
    out.append(g.update(tid, d, prof, now=i * 0.2, daylight=daylight))
  return out


def test_error_grows_with_range():
  assert C.error_m(30) < C.error_m(60) < C.error_m(120)
  assert 0.8 < C.error_m(120) < 1.0


def test_bridge_is_ignored_and_stays_ignored_close():
  out = _run((4.5, 5.5), [110, 100, 90, 80, 70, 60, 50, 40, 30, 20])
  assert not out[0][1]                      # one frame is never enough
  assert all(ig for _, ig in out[3:])
  assert all(abs(b - 4.5) <= 0.5 for b, _ in out[3:])


def test_ground_object_is_never_ignored():
  out = _run((0.0, 1.2), [110, 100, 90, 80, 70, 60, 50, 40, 30, 20])
  assert not any(ig for _, ig in out)
  assert all(b <= 0.5 for b, _ in out)


def test_night_never_ignores():
  out = _run((4.5, 5.5), [110, 100, 90, 80, 70, 60], daylight=False)
  assert not any(ig for _, ig in out)


def test_marginal_height_near_cutoff_kept():
  # 2.25 m bottom: above 6 ft but inside the far-range error band, and under 3 m close in.
  out = _run((2.25, 3.0), [110, 100, 95, 90, 85, 35, 30])
  assert not any(ig for _, ig in out[:5])
  assert not out[-1][1]


def test_three_low_reads_clear_the_latch():
  g = C.ClearanceGate()
  t = 0.0
  for d in (100, 90, 80, 70):
    img, vg, ppm = _frame(d, 4.5, 5.5)
    g.update(1, d, C.edge_profile(img, 964.0, vg, ppm, F / d), t, True)
    t += 0.2
  assert g.get(1)[1]
  for k, d in enumerate((65, 60, 55)):
    img, vg, ppm = _frame(d, 0.0, 1.0)
    g.update(1, d, C.edge_profile(img, 964.0, vg, ppm, F / d), t, True)
    t += 0.2
    assert g.get(1)[1] == (k < 2)


def test_bad_inputs_are_safe():
  assert C.edge_profile(None, 1, 1, 10, 5) is None
  assert C.edge_profile(np.zeros((10, 10), np.uint8), 5, 5, 1.0, 3) is None
  assert C.daylight_from_gps(None) is False
  assert C.daylight_from_gps(SimpleNamespace(latitude=0.0, longitude=0.0, unixTimestampMillis=1)) is False
  g = C.ClearanceGate()
  assert g.update(3, float("nan"), None, 0.0, True)[1] is False
  assert math.isnan(g.get(99)[0])


def test_daylight_from_gps_noon_vs_midnight():
  noon = SimpleNamespace(latitude=45.0, longitude=-93.0, unixTimestampMillis=1792083600000)    # Oct 15 2026 12:00 CDT
  night = SimpleNamespace(latitude=45.0, longitude=-93.0, unixTimestampMillis=1792040400000)   # Oct 15 2026 00:00 CDT
  assert C.daylight_from_gps(noon) and not C.daylight_from_gps(night)


def test_confirmed_from_msg_drops_overhead():
  obs = SimpleNamespace(agree=True, visionEvaluated=True, rangeM=50.0, lateral=0.0, spanM=0.5, vRel=-15.0,
                        objectClass="obstacle", overheadIgnore=False)
  assert confirmed_from_msg(obs, 0.1) is not None
  obs.overheadIgnore = True
  assert confirmed_from_msg(obs, 0.1) is None
