import json
import os


def _lines(path):
  with open(path) as f:
    return f.read().splitlines()

import numpy as np

from openpilot.selfdrive.speedsignd import two_stage as ts
from openpilot.selfdrive.speedsignd.nv12 import Nv12DetectCrop


def test_decode_maps_strip_to_frame():
  raw = np.zeros((1, 5, 4), np.float32)
  raw[0, :, 0] = [176, 64, 20, 20, 0.9]   # strip center
  raw[0, :, 1] = [10, 10, 5, 5, 0.05]     # below threshold
  out = ts.decode_proposals(raw, ts.STRIP_RECT)
  assert len(out) == 1
  x, y, w, h, s = out[0]
  sx = 964 / 352
  assert abs((x + w / 2) - (964 + 176 * sx)) < 1.0
  assert abs((y + h / 2) - (525 + 64 * 350 / 128)) < 1.0
  assert s == np.float32(0.9)


def test_tracker_needs_two_hits_and_captures_large():
  tr = ts.SignTracker()
  tr.update([(1100, 740, 20, 30, 0.5)], 0.0)
  assert tr.capture_candidate(0.0) is None  # seen once
  tr.update([(1130, 735, 24, 34, 0.5)], 0.5)
  assert tr.capture_candidate(0.5) is None  # small, mid-frame
  tr.update([(1400, 700, 50, 70, 0.6)], 1.0)
  c = tr.capture_candidate(1.0)
  assert c is not None and c.hits == 3


def test_neg_track_suppressed():
  tr = ts.SignTracker()
  for i, t in enumerate((0.0, 0.5)):
    tr.update([(1500 + 40 * i, 700, 60, 70, 0.5)], t)
  c = tr.capture_candidate(0.5)
  assert c is not None
  tr.mark_read(c, got_mph=False, neg=ts.neg_read({"peak_name": "neg_other", "peak_conf": 0.8}))
  assert c.neg and c.done
  tr.update([(1580, 700, 64, 74, 0.5)], 1.0)
  assert tr.capture_candidate(1.0) is None


def test_capture_rect_clamped():
  x, y, w, h = ts.capture_rect((1900, 1190, 20, 20))
  assert w == h == ts.CAPTURE_MIN_SIDE
  assert x + w <= 1928 and y + h <= 1208 and x % 2 == 0 and y % 2 == 0
  _, _, w2, _ = ts.capture_rect((1000, 500, 300, 300))
  assert w2 == ts.CAPTURE_MAX_SIDE


def test_skip_when_stopped():
  assert not ts.proposals_allowed(0.5)
  assert ts.proposals_allowed(5.0)
  assert ts.proposals_allowed(None)


def test_strip_rgb_shape():
  crop = Nv12DetectCrop(y=np.full((350, 964), 100, np.uint8), uv=np.full((175, 964), 128, np.uint8),
                        frame_w=1928, frame_h=1208, crop=ts.STRIP_RECT)
  rgb = ts.strip_rgb_from_crop(crop)
  assert rgb.shape == (128, 352, 3) and rgb.dtype == np.uint8


def test_read_target_is_the_box_after_this_frames_proposals():
  """Extrapolation from the previous hits is not the crop."""
  tr = ts.SignTracker()
  tr.update([(1000, 700, 40, 70, 0.5)], 0.0)
  tr.update([(1300, 680, 50, 80, 0.5)], 0.5)
  stale = tr.tracks[0].predicted(1.0)
  assert stale[0] > 1500  # 1300 + (1300 - 1000)
  cand = ts.select_read_target(tr, [(1250, 670, 55, 90, 0.6)], 1.0, allow_read=True)
  assert cand is not None and cand.hits >= 2
  assert abs(cand.crop_box()[0] - 1250) < 1.0
  assert cand.crop_box()[0] < stale[0] - 100
  x, y, w, h = ts.capture_rect(cand.crop_box())
  assert abs((x + w / 2) - (1250 + 55 / 2)) <= 2




def test_bank_covers_a_capture_of_every_strip_corner():
  bx, by, bw, bh = ts.frame_bank_rect()
  boxes = (
    (964, 525, 40, 60),
    (1800, 800, 80, 100),
    (964 + 964 - 40, 525 + 350 - 50, 30, 40),
    (1400, 700, 20, 80),
  )
  for box in boxes:
    x, y, w, h = ts.capture_rect(box)
    assert x >= bx - 0 and y >= by and x + w <= bx + bw and y + h <= by + bh, (box, (x, y, w, h), (bx, by, bw, bh))


def test_subcrop_keeps_frame_coordinates():
  from openpilot.selfdrive.speedsignd.nv12 import subcrop_nv12
  bank = ts.frame_bank_rect()
  bx, by, bw, bh = bank
  parent = Nv12DetectCrop(
    y=np.arange(bh * bw, dtype=np.uint8).reshape(bh, bw),
    uv=np.full((bh // 2, bw), 128, np.uint8),
    frame_w=1928, frame_h=1208, crop=bank,
  )
  rect = ts.capture_rect((1400, 700, 40, 80))
  sub = subcrop_nv12(parent, rect)
  assert sub is not None
  assert sub.crop == rect
  assert sub.y.shape == (rect[3], rect[2])
  assert sub.uv is not None and sub.uv.shape[0] == rect[3] // 2


def test_burst_while_one_hit_then_back_to_2hz():
  tr = ts.SignTracker()
  assert abs(tr.proposer_period_s(0.0) - 1.0 / ts.PROPOSER_HZ) < 1e-9
  tr.update([(1000, 700, 20, 30, 0.5)], 0.0)
  assert tr.wants_fast_proposer(0.0)
  assert abs(tr.proposer_period_s(0.0) - 1.0 / ts.PROPOSER_BURST_HZ) < 1e-9
  tr.update([(1040, 690, 24, 36, 0.5)], 0.2)
  assert not tr.wants_fast_proposer(0.2)
  assert abs(tr.proposer_period_s(0.2) - 1.0 / ts.PROPOSER_HZ) < 1e-9


def test_high_conf_single_proposal_can_be_read():
  tr = ts.SignTracker()
  tr.update([(1500, 700, 60, 70, 0.90)], 0.0)
  c = tr.capture_candidate(0.0)
  assert c is not None and c.hits == 1
  low = ts.SignTracker()
  low.update([(1500, 700, 60, 70, 0.50)], 0.0)
  assert low.capture_candidate(0.0) is None


def test_mph_read_allows_one_more_then_closes():
  tr = ts.SignTracker()
  tr.update([(1500, 700, 60, 70, 0.5)], 0.0)
  tr.update([(1540, 690, 64, 74, 0.5)], 0.4)
  c = tr.capture_candidate(0.4)
  assert c is not None
  tr.mark_read(c, got_mph=True, neg=False)
  assert not c.done and c.reads == 1
  again = tr.capture_candidate(0.4)
  assert again is c
  tr.mark_read(c, got_mph=True, neg=False)
  assert c.done
  assert tr.capture_candidate(0.4) is None


def test_read_budget_gate_still_updates_the_tracker():
  tr = ts.SignTracker()
  assert ts.select_read_target(tr, [(1500, 700, 60, 70, 0.5)], 0.0, allow_read=False) is None
  assert tr.tracks and tr.tracks[0].hits == 1
  cand = ts.select_read_target(tr, [(1540, 690, 64, 80, 0.5)], 0.4, allow_read=True)
  assert cand is not None and cand.hits == 2


def test_feedback_yes_no(tmp_path):
  d = str(tmp_path)
  assert ts.write_pending({"mph": 45, "conf": 0.8, "track_id": 3, "lat": 1.0, "lon": 2.0,
                           "gps_ok": True, "t": 1.0, "class": "sl_45"}, b"\xff\xd8jpg", d)
  rec = ts.record_answer(45, True, d, now=1000.0)
  assert rec["answer"] == "yes" and rec["track_id"] == 3 and rec["jpg"].endswith("_yes_45.jpg")
  assert os.path.isfile(os.path.join(d, rec["jpg"]))
  assert len(_lines(os.path.join(d, ts.CONFIRMED_JSONL))) == 1
  rec2 = ts.record_answer(30, False, d, now=1001.0)  # plate mph != pending -> stale, no jpg
  assert rec2["stale_pending"] and "jpg" not in rec2
  lines = [json.loads(x) for x in _lines(os.path.join(d, ts.FEEDBACK_JSONL))]
  assert [x["answer"] for x in lines] == ["yes", "no"]
  assert len(_lines(os.path.join(d, ts.CONFIRMED_JSONL))) == 1
