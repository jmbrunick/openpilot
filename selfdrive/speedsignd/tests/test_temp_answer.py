"""Yes / No / Temp on the speed-sign prompt. Temp is logged as "temp" and never saved."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from openpilot.selfdrive.mapd import temp_limit as TL
from openpilot.selfdrive.mapd.speed_overrides import PARAM as CONFIRMED_PARAM
from openpilot.selfdrive.speedsignd import confirm as C
from openpilot.selfdrive.speedsignd import mcu_prompt as M
from openpilot.selfdrive.speedsignd.hud import (
  HUD_CONFIRM_TEMP,
  LiveSignView,
  TICI_CONFIRM_BTN_H,
  TICI_CONFIRM_GAP,
  TICI_SIGN_H,
  TICI_SIGN_W,
  answer_button_rects,
  hit_answer_button,
  point_in_confirm_buttons,
  tici_sign_origin,
)

ROOT = Path(__file__).resolve().parents[3]


class Mem:
  def __init__(self):
    self.store = {}

  def get(self, key, *a, **k):
    return self.store.get(key)

  def put(self, key, value, *a, **k):
    self.store[key] = value

  def remove(self, key):
    self.store.pop(key, None)


def _setup(monkeypatch, tmp_path, seq=4, mph=45, way=111):
  monkeypatch.setenv("NAP_SPEED_SIGN_PROMPT_LOCK", str(tmp_path / "prompt.lock"))
  monkeypatch.setenv("NAP_SPEED_SIGN_FEEDBACK", str(tmp_path / "feedback"))
  monkeypatch.setenv("NAP_MAP_SPEED_OVERRIDES", str(tmp_path / "map_speed_overrides.json"))
  p = Mem()
  # request_temp / write_answer / request_override build Params() when none is given.
  import openpilot.common.params as params_mod
  monkeypatch.setattr(params_mod, "Params", lambda *a, **k: p, raising=False)
  fb = tmp_path / "feedback"
  fb.mkdir()
  (fb / "pending.json").write_text(json.dumps({"mph": mph, "seq": seq, "lat": 45.0, "lon": -93.0,
                                               "bearing": 90.0, "has_jpg": True}))
  (fb / "pending.jpg").write_bytes(b"\xff\xd8jpg")
  M.sync_hud_prompt(LiveSignView(show_mph=True, mph=mph, proposal_mode=True, proposed_mph=mph, proposed_seq=seq,
                                 proposed_way_id=way, proposed_heading=90.0), p)
  return p, fb


@pytest.fixture(autouse=True)
def _fake_params_module(monkeypatch):
  import sys
  import types
  if "openpilot.common.params" not in sys.modules:
    try:
      import openpilot.common.params  # noqa: F401
    except Exception:
      mod = types.ModuleType("openpilot.common.params")
      mod.Params = None
      monkeypatch.setitem(sys.modules, "openpilot.common.params", mod)


def test_temp_logs_temp_and_saves_nothing(monkeypatch, tmp_path):
  p, fb = _setup(monkeypatch, tmp_path)
  assert M.answer_prompt(4, "temp", params=p)
  rows = [json.loads(x) for x in (fb / "feedback.jsonl").read_text().splitlines()]
  assert [r["answer"] for r in rows] == ["temp"] and rows[0]["seq"] == 4
  assert "_temp_45" in rows[0]["jpg"]
  assert not (fb / "confirmed.jsonl").exists()              # never a permanent confirm
  assert CONFIRMED_PARAM not in p.store                      # never a map override
  assert not (tmp_path / "map_speed_overrides.json").exists()
  temp = TL.parse_temp(p.store[TL.PARAM])
  assert temp["mph"] == 45 and temp["read_seq"] == 4 and temp["way_id"] == 111 and temp["lat"] == 45.0
  answer_row = p.store[C.ANSWER_PARAM]           # JSON param: stored as the dict (hygiene fix)
  assert answer_row == {"seq": 4, "mph": 45, "yes": False, "answer": "temp"}
  rec = p.store[M.PROMPT_PARAM]
  assert rec["answered"] and rec["answer"] == "temp" and rec["yes"] is False
  assert M.read_note(p) == "Temp 45 this drive"
  # Once per id, from either screen.
  assert M.answer_prompt(4, True, params=p) is False


def test_yes_still_confirms_and_writes_no_temp(monkeypatch, tmp_path):
  p, fb = _setup(monkeypatch, tmp_path)
  assert M.answer_prompt(4, True, params=p)
  assert (fb / "confirmed.jsonl").exists()
  assert CONFIRMED_PARAM in p.store and TL.PARAM not in p.store


def test_temp_without_read_time_road_still_applies(monkeypatch, tmp_path):
  p, fb = _setup(monkeypatch, tmp_path, way=0)
  assert M.answer_prompt(4, "temp", params=p)
  temp = TL.parse_temp(p.store[TL.PARAM])
  assert temp["mph"] == 45 and temp["way_id"] == 0     # mapd anchors it on the current road
  assert CONFIRMED_PARAM not in p.store


def test_plate_state_temp_closes_without_confirming():
  ps = C.PlateState()
  pr = ps.propose(45, 0.9, way_id=1, heading=90.0, read_id=("read", 1, 0.0), now=0.0)
  assert ps.answer({"seq": pr.seq, "mph": 45, "yes": False, "answer": "temp"})
  assert ps.proposal is None and ps.plate_mph == 0
  assert ps.confirmed_mph == 0 and ps.temp_mph == 45
  # Old answers (no "answer" key) are unchanged.
  pr = ps.propose(55, 0.9, read_id=("read", 2, 1.0), now=1.0)
  assert ps.answer(json.dumps({"seq": pr.seq, "mph": 55, "yes": True})) and ps.confirmed_mph == 55


def test_timeout_unchanged():
  assert C.PROMPT_TIMEOUT_S == 20.0


def test_answer_kind():
  assert [C.answer_kind(v) for v in (True, False, "temp", "TEMP", "yes", "no")] == \
    ["yes", "no", "temp", "temp", "yes", "no"]
  with pytest.raises(ValueError):
    C.answer_kind("engage")


def test_three_buttons_tici_and_mici():
  x, y = tici_sign_origin()
  yes, no, temp = answer_button_rects(x, y, TICI_SIGN_W, TICI_SIGN_H, prompt_h=32)
  assert temp[1] == no[1] + TICI_CONFIRM_BTN_H + TICI_CONFIRM_GAP and temp[3] == TICI_CONFIRM_BTN_H
  assert temp[1] + temp[3] <= 1080                       # on screen (comma 3X 2160x1080)
  assert hit_answer_button(temp[0] + 5, temp[1] + 5, yes, no, temp) == "temp"
  assert hit_answer_button(yes[0] + 5, yes[1] + 5, yes, no, temp) == "yes"
  assert hit_answer_button(no[0] + 5, no[1] + 5, yes, no, temp) == "no"
  assert hit_answer_button(0, 0, yes, no, temp) is None
  v = LiveSignView(show_mph=True, mph=45, proposal_mode=True, proposed_mph=45, proposed_seq=3)
  assert point_in_confirm_buttons(temp[0] + 5, temp[1] + 5, v, yes, no, temp)
  myes, mno, mtemp = answer_button_rects(8, 6, 108, 128, beside=True)
  assert myes[1] < mno[1] < mtemp[1] and mtemp[1] + mtemp[3] <= 6 + 128 + 0.01
  assert HUD_CONFIRM_TEMP == "Temp"


def test_comma_hud_draws_and_answers_temp():
  hud = (ROOT / "selfdrive/ui/onroad/speed_sign_hud.py").read_text()
  assert "hit_answer_button(mouse_pos.x, mouse_pos.y, self._yes, self._no, self._temp)" in hud
  assert "HUD_CONFIRM_TEMP" in hud and "_TEMP_BG" in hud
