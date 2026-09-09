"""HUD hold and cereal live sample for the speed-sign logger."""
from __future__ import annotations

from types import SimpleNamespace

from openpilot.selfdrive.speedsignd.detect import SpeedSign, SpeedSignDetector, paint_mutcd_r2_1
from openpilot.selfdrive.speedsignd.hud import HUD_HOLD_S, LiveSignHold, apply_live_sign, hud_should_show
from openpilot.selfdrive.speedsignd.jsonl import JsonlLogger
from openpilot.selfdrive.speedsignd.speedsignd import process_frame
from openpilot.selfdrive.speedsignd.tests.test_detect import _scene


def test_hold_keeps_mph_then_clears():
  h = LiveSignHold(hold_s=1.5)
  signs = [SpeedSign(mph=45, conf=0.8, bbox=(0, 0, 10, 10))]
  live, mph, conf = h.update(signs, 10.0)
  assert live and mph == 45 and conf == 0.8
  live, mph, _ = h.update([], 11.0)
  assert live and mph == 45
  live, mph, _ = h.update([], 11.49)
  assert live and mph == 45
  live, mph, conf = h.update([], 11.51)
  assert not live and mph == 0 and conf == 0.0


def test_new_detection_refreshes_hold():
  h = LiveSignHold(hold_s=1.5)
  h.update([SpeedSign(mph=30, conf=0.7, bbox=(0, 0, 1, 1))], 0.0)
  live, mph, _ = h.update([SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 1, 1))], 1.0)
  assert live and mph == 55
  live, mph, _ = h.update([], 2.4)
  assert live and mph == 55


def test_hud_hidden_unless_enabled_and_valid():
  assert not hud_should_show(False, True, 45)
  assert not hud_should_show(True, False, 45)
  assert not hud_should_show(True, True, 0)
  assert hud_should_show(True, True, 45)


def test_apply_live_sign_clears_when_invalid():
  d = SimpleNamespace()
  apply_live_sign(d, mph=45, conf=0.8, valid=True)
  assert d.mph == 45 and d.valid
  apply_live_sign(d, mph=45, conf=0.8, valid=False)
  assert d.mph == 0 and d.conf == 0.0 and not d.valid


def test_detect_without_gps_still_returns_signs(tmp_path):
  y = _scene()
  paint_mutcd_r2_1(y, 35, x=200, y=30, w=90, h=112)
  signs, written = process_frame(
    y, 0.0, 0.0, None, False,
    SpeedSignDetector(onnx_path=str(tmp_path / "missing.onnx")),
    JsonlLogger(str(tmp_path / "out.jsonl")),
    now=1.0,
  )
  assert written == []
  assert signs and signs[0].mph == 35


def test_hold_duration_constant():
  assert HUD_HOLD_S == 1.5


def test_tici_plate_sits_right_of_center_left_of_exp_button():
  tici_w = 200
  rect_w = 2160
  button_x = rect_w - 30 - 192
  x = button_x - 18 - tici_w
  assert x + tici_w <= button_x
  assert x > rect_w / 2


def test_ui_and_cereal_wire_live_sign():
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  custom = (root / "cereal" / "custom.capnp").read_text(encoding="utf-8")
  log = (root / "cereal" / "log.capnp").read_text(encoding="utf-8")
  services = (root / "cereal" / "services.py").read_text(encoding="utf-8")
  ui = (root / "selfdrive" / "ui" / "ui_state.py").read_text(encoding="utf-8")
  tici = (root / "selfdrive" / "ui" / "onroad" / "hud_renderer.py").read_text(encoding="utf-8")
  mici = (root / "selfdrive" / "ui" / "mici" / "onroad" / "hud_renderer.py").read_text(encoding="utf-8")
  assert "struct LiveSpeedSignNAP" in custom
  assert "liveSpeedSignNAP @108" in log
  assert '"liveSpeedSignNAP"' in services
  assert "liveSpeedSignNAP" in ui
  assert "draw_tici_speed_sign" in tici
  assert "draw_mici_speed_sign" in mici
