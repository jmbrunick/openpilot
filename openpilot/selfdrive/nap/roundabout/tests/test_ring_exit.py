from openpilot.selfdrive.nap.roundabout.ring_exit import RingExitHold, leaving_ring, ring_center_radius
import math

# 30th St SW style ring, R ~22 m, centred at (45.0, -95.0)
LAT0, LON0, R = 45.0, -95.0, 22.0
RING = [(LAT0 + R / 6371000 * 180 / math.pi * math.cos(a), LON0 + R / 6371000 * 180 / math.pi * math.sin(a) / math.cos(math.radians(LAT0)))
        for a in [i * 2 * math.pi / 24 for i in range(25)]]


def _pt(north_m, east_m):
  return LAT0 + north_m / 6371000 * 180 / math.pi, LON0 + east_m / 6371000 * 180 / math.pi / math.cos(math.radians(LAT0))


def test_center_radius():
  center = ring_center_radius(RING)
  assert center is not None
  la, lo, r = center
  assert abs(r - R) < 0.5


def test_leaving_vs_entering_and_circulating():
  lat, lon = _pt(30.0, 0.0)  # 30 m north of centre
  assert leaving_ring(lat, lon, 0.0, RING)        # heading north, away
  assert not leaving_ring(lat, lon, 180.0, RING)  # heading in
  assert not leaving_ring(lat, lon, 270.0, RING)  # tangential
  lat, lon = _pt(20.0, 0.0)
  assert not leaving_ring(lat, lon, 0.0, RING)    # still inside the ring radius


def _run(h, seq, on=True, dt=0.1):
  out = []
  for a in seq:
    out.append(h.update(on, True, a, dt))
  return out


def test_hold_after_left_circulation_unwinds_oct14():
  h = RingExitHold()
  # entry right deflection (-35..-87), circulate left 67..148, unwind to 2
  seq = [-35] * 5 + [-87] * 10 + [-15] * 5 + [67] * 5 + [111] * 10 + [148] * 10 + [100] * 20 + [71] * 5 + [48] * 5 + [32] * 5 + [2] * 5
  out = _run(h, seq)
  first = out.index(True)
  assert seq[first] < 0.4 * 148
  assert all(out[first:])
  assert not any(out[:45])


def test_entry_right_deflection_never_holds():
  h = RingExitHold()
  assert not any(_run(h, [-87] * 20 + [0] * 20))


def test_reset_when_ring_clears():
  h = RingExitHold()
  _run(h, [120] * 20 + [0] * 10)
  assert h.holding
  assert not h.update(False, False, 0.0, 0.1)
  assert not h.holding
