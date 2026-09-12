"""Two-frame debounce before JSONL; HUD lights on the first in-threshold hit."""
from __future__ import annotations

from openpilot.selfdrive.speedsignd.debounce import SignDebounce
from openpilot.selfdrive.speedsignd.detect import SpeedSign
from openpilot.selfdrive.speedsignd.jsonl import JsonlLogger
from openpilot.selfdrive.speedsignd.speedsignd import SPEEDSIGND_HZ, next_detect_mono, process_frame
from openpilot.selfdrive.speedsignd.weights_manifest import DEBOUNCE_HITS, DEBOUNCE_WINDOW_S


def test_single_frame_is_not_enough():
  d = SignDebounce()
  signs = [SpeedSign(mph=55, conf=0.9, bbox=(1, 1, 10, 10))]
  assert d.update(signs, 0.0) == []


def test_two_agreeing_frames_confirm():
  d = SignDebounce()
  s = SpeedSign(mph=60, conf=0.8, bbox=(2, 2, 12, 12))
  assert d.update([s], 0.0) == []
  # 1 Hz spacing (and a skipped cycle at ~2 s) must still confirm.
  out = d.update([s], 1.0)
  assert len(out) == 1 and out[0].mph == 60


def test_two_hits_after_skip_still_confirm():
  d = SignDebounce()
  s = SpeedSign(mph=55, conf=0.85, bbox=(0, 0, 8, 8))
  assert d.update([s], 0.0) == []
  out = d.update([s], 2.0)
  assert len(out) == 1 and out[0].mph == 55


def test_disagreement_does_not_confirm():
  d = SignDebounce()
  d.update([SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 1, 1))], 0.0)
  assert d.update([SpeedSign(mph=60, conf=0.9, bbox=(0, 0, 1, 1))], 1.0) == []


def test_stale_hit_expires():
  d = SignDebounce()
  s = SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 1, 1))
  d.update([s], 0.0)
  assert d.update([s], DEBOUNCE_WINDOW_S + 0.01) == []


def test_low_conf_ignored():
  d = SignDebounce()
  s = SpeedSign(mph=55, conf=0.2, bbox=(0, 0, 1, 1))
  d.update([s], 0.0)
  assert d.update([s], 1.0) == []


def test_process_frame_debounce_blocks_jsonl(tmp_path):
  class _Det:
    onnx = None

    def detect(self, y, min_conf=None, rgb=None, nv12=None):
      return [SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 8, 8))]

  log = JsonlLogger(str(tmp_path / "out.jsonl"))
  debounce = SignDebounce()
  y = __import__("numpy").zeros((32, 32), __import__("numpy").uint8)
  signs, written = process_frame(y, 45.0, -95.0, 0.0, True, _Det(), log, now=1.0, debounce=debounce)
  # First in-threshold hit lights HUD; JSONL still waits for the second frame.
  assert signs and signs[0].mph == 55
  assert written == []
  signs, written = process_frame(y, 45.0, -95.0, 0.0, True, _Det(), log, now=2.0, debounce=debounce)
  assert signs and signs[0].mph == 55
  assert written and written[0]["mph"] == 55


def test_update_split_hud_on_first_hit():
  d = SignDebounce()
  s = SpeedSign(mph=55, conf=0.72, bbox=(0, 0, 8, 8))
  split = d.update_split([s], 0.0)
  assert split.hud and split.hud[0].mph == 55
  assert split.confirmed == []
  split = d.update_split([s], 1.0)
  assert split.hud and split.confirmed
  assert split.confirmed[0].mph == 55


def test_overrun_two_hit_window_can_miss_jsonl():
  """1.5 s tinygrad infer + payback spaces JSONL hits by 3 s (inside 4 s).

  A 2.1 s infer spaces them by 4.2 s — JSONL never confirms. HUD must still
  light on the first hit so a highway R2-1 is not blank after WAIT.
  """
  period, budget = 1.0 / SPEEDSIGND_HZ, 0.100
  infer_s = 2.1
  gap = next_detect_mono(infer_s, infer_s, period, budget)
  assert gap > DEBOUNCE_WINDOW_S
  d = SignDebounce()
  s = SpeedSign(mph=60, conf=0.8, bbox=(0, 0, 8, 8))
  first = d.update_split([s], 0.0)
  assert first.hud and first.hud[0].mph == 60
  assert first.confirmed == []
  second = d.update_split([s], gap)
  assert second.hud and second.hud[0].mph == 60
  assert second.confirmed == []


def test_debounce_constants_match_1hz():
  assert SPEEDSIGND_HZ == 1.0
  assert DEBOUNCE_HITS == 2
  assert DEBOUNCE_WINDOW_S == 4.0
  # Two 1 Hz hits (or 1 Hz + one skip) must fit in the JSONL window.
  assert DEBOUNCE_WINDOW_S >= 2.0 / SPEEDSIGND_HZ
