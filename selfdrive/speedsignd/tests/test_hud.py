"""HUD hold and cereal live sample for the speed-sign logger."""
from __future__ import annotations

from types import SimpleNamespace

import pytest

from openpilot.selfdrive.speedsignd.detect import SpeedSign, SpeedSignDetector, paint_mutcd_r2_1
from openpilot.selfdrive.speedsignd.hud import (
  HUD_HOLD_S,
  HUD_MISSING_WEIGHTS_TEXT,
  LiveSignHold,
  plate_digit_size,
  apply_live_sign,
  hud_should_show,
  hud_should_show_missing_weights,
  live_sign_from_event,
  live_sign_view,
)
from openpilot.selfdrive.speedsignd.jsonl import JsonlLogger
from openpilot.selfdrive.speedsignd.speedsignd import live_sign_publish_fields, process_frame
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
  assert d.mph == 45 and d.valid and not d.weightsMissing
  apply_live_sign(d, mph=45, conf=0.8, valid=False)
  assert d.mph == 0 and d.conf == 0.0 and not d.valid and not d.weightsMissing
  apply_live_sign(d, mph=45, conf=0.8, valid=False, weights_missing=True)
  assert d.mph == 0 and not d.valid and d.weightsMissing


def test_hud_missing_weights_plate_when_logger_on():
  assert HUD_MISSING_WEIGHTS_TEXT == "NO WT"
  assert plate_digit_size("NO WT", 120) < plate_digit_size("55", 120)
  assert hud_should_show_missing_weights(True, True)
  assert not hud_should_show_missing_weights(False, True)
  assert not hud_should_show_missing_weights(True, False)

  missing = live_sign_view(enabled=True, msg_valid=True, mph=0, valid=False, weights_missing=True)
  assert missing.show and missing.show_missing_weights
  assert missing.plate_text == "NO WT"
  assert not missing.show_mph

  # Never a false mph while weights are missing, even if numpy produced one.
  fake = live_sign_view(enabled=True, msg_valid=True, mph=55, valid=True, weights_missing=True)
  assert fake.show_missing_weights and fake.plate_text == "NO WT"
  assert not fake.show_mph

  off = live_sign_view(enabled=False, msg_valid=True, mph=0, valid=False, weights_missing=True)
  assert not off.show

  blank = live_sign_view(enabled=True, msg_valid=False, mph=0, valid=False, weights_missing=False)
  assert not blank.show and blank.plate_text == ""

  mph = live_sign_view(enabled=True, msg_valid=True, mph=55, valid=True, weights_missing=False)
  assert mph.show_mph and mph.mph == 55 and mph.plate_text == "55"

  ev = live_sign_from_event(True, SimpleNamespace(mph=0, valid=False, weightsMissing=True), True)
  assert ev.show_missing_weights and ev.plate_text == "NO WT"
  ev_blank = live_sign_from_event(True, SimpleNamespace(mph=0, valid=False, weightsMissing=False), False)
  assert not ev_blank.show
  ev_mph = live_sign_from_event(True, SimpleNamespace(mph=60, valid=True, weightsMissing=False), True)
  assert ev_mph.show_mph and ev_mph.plate_text == "60"


def test_publish_fields_flag_missing_weights_without_mph():
  h = LiveSignHold(hold_s=1.5)
  signs = [SpeedSign(mph=45, conf=0.8, bbox=(0, 0, 10, 10))]
  msg_valid, mph, conf, missing = live_sign_publish_fields(h, signs, 10.0, True)
  assert msg_valid and missing and mph == 0 and conf == 0.0
  # Hold was not primed by the numpy hit.
  live, held, _ = h.update([], 10.1)
  assert not live and held == 0

  msg_valid, mph, conf, missing = live_sign_publish_fields(h, signs, 11.0, False)
  assert msg_valid and not missing and mph == 45
  live, held, _ = h.update([], 11.5)
  assert live and held == 45


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


def test_cereal_live_sign_has_weights_missing_field():
  capnp = pytest.importorskip("capnp")
  from pathlib import Path
  capnp.remove_import_hook()
  custom_path = Path(__file__).resolve().parents[3] / "cereal" / "custom.capnp"
  custom = capnp.load(str(custom_path))
  msg = custom.LiveSpeedSignNAP.new_message()
  assert msg.mph == 0
  assert not msg.weightsMissing
  apply_live_sign(msg, mph=0, conf=0.0, valid=False, weights_missing=True)
  assert msg.weightsMissing and msg.mph == 0 and not msg.valid


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
  assert "weightsMissing @3" in custom
  assert "liveSpeedSignNAP @108" in log
  assert '"liveSpeedSignNAP"' in services
  assert "liveSpeedSignNAP" in ui
  assert "draw_tici_speed_sign" in tici
  assert "draw_mici_speed_sign" in mici
  onroad = (root / "selfdrive" / "ui" / "onroad" / "speed_sign_hud.py").read_text(encoding="utf-8")
  assert "NO WT" in onroad
  assert "live_sign_from_event" in onroad
