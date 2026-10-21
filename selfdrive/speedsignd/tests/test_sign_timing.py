"""Speed-aware read timing, stale-crop guard, budget exemption, scan rate, strips."""
from openpilot.selfdrive.speedsignd import strip_geom, two_stage as T
from openpilot.selfdrive.speedsignd.speedsignd import drop_cpu_reasons


def _tracker_with(boxes_t):
  tr = T.SignTracker()
  for t, b in boxes_t:
    tr.update([(*b, 0.5)], t)
  return tr


def test_read_px_scales_with_speed_and_defaults():
  assert T.read_px_for_speed(None) == T.CAPTURE_MIN_H
  assert T.read_px_for_speed(10.0) == 60.0
  assert T.read_px_for_speed(26.0) == 40.0
  assert 40.0 < T.read_px_for_speed(20.0) < 60.0


def test_scan_hz_one_per_12m_clamped():
  assert T.scan_hz_for_speed(None) == T.PROPOSER_HZ
  assert T.scan_hz_for_speed(24.0) == 2.0
  assert T.scan_hz_for_speed(1.0) == T.SCAN_HZ_FLOOR  # v2.1 floor below 55 mph
  assert T.scan_hz_for_speed(40.0) == T.SCAN_HZ_MAX


def test_proposer_period_uses_speed_and_burst():
  tr = T.SignTracker()
  assert abs(tr.proposer_period_s(0.0, 12.0) - 0.5) < 1e-9  # v2.1 2 Hz floor
  assert abs(tr.proposer_period_s(0.0, 28.8) - 1.0 / 2.4) < 1e-9
  tr.update([(1500, 600, 20, 25, 0.5)], 0.0)  # 1-hit track -> burst
  assert abs(tr.proposer_period_s(0.1, 12.0) - 1.0 / T.PROPOSER_BURST_HZ) < 1e-9


def test_legacy_without_speed_still_needs_60px():
  tr = _tracker_with([(0.0, (1500, 600, 40, 45)), (0.5, (1520, 590, 44, 50))])
  assert tr.capture_candidate(0.5) is None


def test_fast_speed_reads_at_40px():
  tr = _tracker_with([(0.0, (1500, 600, 36, 42)), (0.5, (1520, 590, 40, 46))])
  assert tr.capture_candidate(0.5, v_ego=27.0) is not None


def test_predicted_exit_reads_before_sign_leaves_strip_top():
  # Growing and rising toward the strip top (y=525): next frame is above it.
  tr = _tracker_with([(0.0, (1500, 560, 26, 32)), (0.5, (1540, 535, 30, 38))])
  assert tr.capture_candidate(0.5) is None  # legacy: too small
  c = tr.capture_candidate(0.5, v_ego=15.0, next_t=1.0, strip=T.STRIP_RECT)
  assert c is not None
  assert c.crop_box() == (1540, 535, 30, 38)  # measured, never predicted


def test_lost_track_is_never_cropped_stale_and_retires():
  tr = _tracker_with([(0.0, (1500, 600, 50, 70)), (0.5, (1520, 590, 55, 75))])
  tr.update([], 0.8)  # not re-measured this frame
  assert tr.capture_candidate(0.8, v_ego=25.0) is None
  tr.update([], 1.3)
  assert tr.capture_candidate(1.3) is None
  assert tr.tracks[0].done


def test_budget_exempt_only_with_confirmed_live_track():
  tr = _tracker_with([(0.0, (1500, 600, 30, 35))])
  assert not tr.budget_exempt(0.0)          # one hit
  tr.update([(1505, 598, 32, 37, 0.5)], 0.5)
  assert tr.budget_exempt(0.5)
  assert not tr.budget_exempt(2.0)          # lost
  t0 = tr.tracks[0]
  t0.reads = T.MAX_READS_PER_TRACK
  assert not tr.budget_exempt(0.5)


def test_drop_cpu_reasons_keeps_model_lag():
  assert drop_cpu_reasons("cpu-load+model-lag+cpu-budget", True) == "model-lag"
  assert drop_cpu_reasons("cpu-load+disk-stall", True) == "disk-stall"
  assert drop_cpu_reasons("cpu-load", False) == "cpu-load"


def test_strips_fallback_and_calibrated():
  assert strip_geom.strips(1928, 1208) == (T.STRIP_RECT, None)
  assert strip_geom.strips(1344, 760, rpy=[0, 0, 0]) == (T.STRIP_RECT, None)
  main, upper = strip_geom.strips(1928, 1208, rpy=[0, 0, 0])
  assert main == T.STRIP_RECT and upper is None  # no road edge
  main, upper = strip_geom.strips(1928, 1208, rpy=[0, 0, 0], edge=([0, 20, 40, 80], [4.0] * 4, 0.3))
  assert main == T.STRIP_RECT
  assert upper is not None
  x, y, w, h = upper
  assert (w, h) == (strip_geom.UPPER_W, strip_geom.UPPER_H) and y == main[1] - h and x >= main[0]
  assert abs(w / h - main[2] / main[3]) < 0.05  # same 2.75:1 proposer scale
  assert y <= 360


def test_strips_ignore_unconfident_or_left_edge():
  _, upper = strip_geom.strips(1928, 1208, rpy=[0, 0, 0], edge=([0, 40, 80], [4.0] * 3, 2.0))
  assert upper is None
  _, upper = strip_geom.strips(1928, 1208, rpy=[0, 0, 0], edge=([0, 40, 80], [-3.0] * 3, 0.3))
  assert upper is None


def test_strip_horizon_shift_is_clamped():
  main, _ = strip_geom.strips(1928, 1208, rpy=[0, 0.5, 0])
  assert abs(main[1] - T.STRIP_RECT[1]) <= strip_geom.HORIZON_MAX_DY
  assert main[1] % 2 == T.STRIP_RECT[1] % 2


def test_bank_and_union_cover_upper():
  main, upper = T.STRIP_RECT, (1200, 350, 482, 175)
  u = T.union_rect(main, upper)
  assert u[1] == 350 and u[1] + u[3] == main[1] + main[3]
  bx, by, bw, bh = T.frame_bank_rect(1928, 1208, u)
  assert by <= 350 and bx <= 964


# --- v2.1 ---------------------------------------------------------------

def test_v21_scan_floor_2hz_below_55mph():
  from openpilot.selfdrive.speedsignd import two_stage as ts
  assert ts.scan_hz_for_speed(8.0) == 2.0      # 18 mph
  assert ts.scan_hz_for_speed(12.0) == 2.0     # 27 mph (was 1.0)
  assert ts.scan_hz_for_speed(24.0) == 2.0
  assert ts.scan_hz_for_speed(40.0) == ts.SCAN_HZ_MAX


def test_v21_reader_next_after():
  from openpilot.selfdrive.speedsignd import two_stage as ts
  kw = dict(end=10.0, infer_s=0.8, steady=13.2, duty_t=11.0)
  assert ts.reader_next_after(exempt=True, empty=False, **kw) == 10.8
  assert ts.reader_next_after(exempt=True, empty=True, **kw) == 10.0
  assert ts.reader_next_after(exempt=False, empty=True, **kw) == 11.0
  assert ts.reader_next_after(exempt=False, empty=False, **kw) == 13.2


def _confirmed(ts, h=40.0):
  tr = ts.SignTracker()
  tr.update([(1500, 600, h * 0.8, h, 0.6)], 0.0)
  tr.update([(1510, 598, h * 0.8, h, 0.6)], 0.25)
  return tr


def test_v21_exempt_reads_capped_at_3():
  from openpilot.selfdrive.speedsignd import two_stage as ts
  tr = _confirmed(ts)
  t = tr.tracks[0]
  for i in range(3):
    assert tr.budget_exempt(0.25)
    t.reads = i + 1
  assert not tr.budget_exempt(0.25)


def test_v21_exempt_allows_read_while_resting():
  from openpilot.selfdrive.speedsignd import two_stage as ts
  tr = _confirmed(ts, h=70.0)
  c = ts.select_read_target(tr, [(1520, 596, 56, 72, 0.6)], 0.5, allow_read=False, allow_exempt=True)
  assert c is not None
  tr2 = _confirmed(ts, h=70.0)
  assert ts.select_read_target(tr2, [(1520, 596, 56, 72, 0.6)], 0.5, allow_read=False) is None


def test_v21_small_low_conf_read_waits_for_growth():
  from openpilot.selfdrive.speedsignd import two_stage as ts
  tr = _confirmed(ts, h=48.0)
  t = tr.tracks[0]
  tr.mark_read(t, got_mph=False, neg=False, read_h=48.0, top_conf=0.6)
  assert t.reread_h == 48.0 * ts.REREAD_GROW and not t.done
  tr.update([(1515, 597, 40, 50, 0.6)], 0.5)
  assert tr.capture_candidate(0.5) is None          # not grown enough
  tr.update([(1520, 596, 50, 62, 0.6)], 0.75)
  assert tr.capture_candidate(0.75) is t            # grown -> re-read
  tr.mark_read(t, got_mph=True, neg=False, read_h=62.0, top_conf=0.95)
  assert t.reread_h == 0.0


def test_v21_proposer_duty_cap_25pct():
  from openpilot.selfdrive.speedsignd import two_stage as ts
  d = ts.ProposerDuty()
  n = 0
  t = 0.0
  while t < 10.0:
    if d.fits(t, 1):
      d.add(t, 1)
      n += 1
    t += 0.2   # a 5 Hz burst asks for 50 passes
  assert n * ts.PROPOSER_PASS_S_DEFAULT <= 2.5 + 1e-6
  assert n == 20
  assert d.next_fit(t, 1) >= t
