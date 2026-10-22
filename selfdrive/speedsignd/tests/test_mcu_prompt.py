"""MCU and comma share one speed-sign Yes/No. Display and map only."""
from __future__ import annotations

import json
import threading
from pathlib import Path

from openpilot.selfdrive.speedsignd import confirm as C
from openpilot.selfdrive.speedsignd import mcu_prompt as M
from openpilot.selfdrive.speedsignd.hud import LiveSignView
from openpilot.selfdrive.mapd.speed_overrides import PARAM, SpeedOverrides, request_override


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


def _view(seq=4, mph=45, way=111, heading=90.0, plate=35, **kw):
  return LiveSignView(
    show_mph=True, mph=plate, proposal_mode=True,
    proposed_mph=mph, proposed_seq=seq, proposed_way_id=way, proposed_heading=heading,
    **kw,
  )


def _bind(monkeypatch, tmp_path):
  monkeypatch.setenv("NAP_SPEED_SIGN_PROMPT_LOCK", str(tmp_path / "prompt.lock"))
  monkeypatch.setenv("NAP_SPEED_SIGN_FEEDBACK", str(tmp_path / "feedback"))
  return Mem()


def test_publish_shows_read_speed_and_read_time_way(monkeypatch, tmp_path):
  p = _bind(monkeypatch, tmp_path)
  assert M.sync_hud_prompt(_view(), p) is False
  pending = M.read_pending(p)
  assert pending == {"id": 4, "mph": 45, "way_id": 111, "heading": 90.0}
  # A second frame with the same prompt does not need another write to stay open.
  assert M.sync_hud_prompt(_view(), p) is False
  assert M.read_pending(p)["mph"] == 45


def test_missing_weights_is_not_a_prompt(monkeypatch, tmp_path):
  p = _bind(monkeypatch, tmp_path)
  M.sync_hud_prompt(_view(), p)
  hidden = LiveSignView(show_missing_weights=True, proposal_mode=True, proposed_mph=45, proposed_seq=4)
  assert M.sync_hud_prompt(hidden, p) is False
  assert M.read_pending(p) is None


def test_yes_uses_read_time_way_not_the_car_now(monkeypatch, tmp_path):
  """Sign was read on way 111 heading 90. The tap happens after the car is on way 222."""
  from openpilot.selfdrive.speedsignd.two_stage import record_answer as real_record

  p = _bind(monkeypatch, tmp_path)
  M.sync_hud_prompt(_view(seq=1, way=111, heading=90.0), p)
  order = []
  plate = C.PlateState()
  plate.propose(45, 0.91, way_id=111, heading=90.0)

  def record(mph, yes):
    order.append("record_answer")
    (tmp_path / "feedback").mkdir(exist_ok=True)
    (tmp_path / "feedback" / "pending.json").write_text(json.dumps({
      "mph": 45, "lat": 45.0, "lon": -93.0, "bearing": 90.0,
    }))
    return real_record(mph, yes)

  def write(seq, mph, yes, params=None):
    order.append("write_answer")
    assert plate.answer({"seq": seq, "mph": mph, "yes": yes})
    return True

  captured = {}

  def override(mph, rec, seq, params=None, way_id=None, heading=None):
    order.append("request_override")
    captured.update(mph=mph, way_id=way_id, heading=heading, rec=rec)
    fake = Mem()
    assert request_override(mph, rec, seq, params=fake, way_id=way_id, heading=heading)
    captured["param"] = fake.store[PARAM]
    return True

  monkeypatch.setattr("openpilot.selfdrive.speedsignd.two_stage.record_answer", record)
  monkeypatch.setattr("openpilot.selfdrive.speedsignd.confirm.write_answer", write)
  monkeypatch.setattr("openpilot.selfdrive.mapd.speed_overrides.request_override", override)

  # Client (or a moved car) must not be able to substitute the current way.
  assert M.answer_prompt(1, True, params=p, snapshot={"id": 1, "mph": 45, "way_id": 222, "heading": 0.0})
  assert order == ["record_answer", "write_answer", "request_override"]
  assert captured["way_id"] == 111 and captured["heading"] == 90.0
  assert captured["param"]["way_id"] == 111 and captured["param"]["heading"] == 90.0
  assert plate.plate_mph == 45
  stored = SpeedOverrides(str(tmp_path / "map.json"))
  got = stored.consume(captured["param"], type("M", (), {"way_id": 222, "road_name": "now"})(), 0.0, 1.0)
  assert got["way_id"] == 111 and got["heading"] == 90.0
  assert stored.find(222, 0.0) is None
  assert M.read_pending(p) is None
  assert M.sync_hud_prompt(_view(seq=1), p) is True  # comma HUD hides; no second answer
  assert M.answer_prompt(1, True, params=p) is False
  assert order == ["record_answer", "write_answer", "request_override"]


def test_no_leaves_the_plate_and_skips_the_map(monkeypatch, tmp_path):
  p = _bind(monkeypatch, tmp_path)
  plate = C.PlateState()
  pr = plate.propose(45, 0.8, way_id=111, heading=90.0)
  M.sync_hud_prompt(_view(seq=pr.seq, mph=pr.mph, way=pr.way_id, heading=pr.heading), p)
  calls = []

  def record(mph, yes):
    calls.append(("record_answer", mph, yes))
    return {"lat": 1.0, "lon": 2.0, "bearing": 90.0}

  def write(seq, mph, yes, params=None):
    calls.append(("write_answer", seq, mph, yes))
    plate.answer({"seq": seq, "mph": mph, "yes": yes})
    return True

  def override(*a, **k):
    calls.append("request_override")
    return True

  monkeypatch.setattr("openpilot.selfdrive.speedsignd.two_stage.record_answer", record)
  monkeypatch.setattr("openpilot.selfdrive.speedsignd.confirm.write_answer", write)
  monkeypatch.setattr("openpilot.selfdrive.mapd.speed_overrides.request_override", override)
  assert M.answer_prompt(pr.seq, False, params=p)
  assert plate.plate_mph == 0 and plate.proposal is None
  assert "request_override" not in calls
  assert [c[0] for c in calls] == ["record_answer", "write_answer"]


def test_timeout_clears_the_prompt_and_leaves_the_plate(monkeypatch, tmp_path):
  p = _bind(monkeypatch, tmp_path)
  plate = C.PlateState()
  pr = plate.propose(45, 0.8, way_id=111, heading=90.0)
  M.sync_hud_prompt(_view(seq=pr.seq), p)
  called = []
  monkeypatch.setattr(M, "on_confirm_accuracy", lambda *a, **k: called.append(a))
  # Prompt went away (timeout / read dropped) with no tap.
  assert M.sync_hud_prompt(LiveSignView(show_mph=True, mph=0), p) is False
  assert M.read_pending(p) is None
  assert called == []
  assert M.answer_prompt(pr.seq, True, params=p) is False
  assert plate.plate_mph == 0 and plate.proposal is not None
  assert called == []


def test_each_id_answered_once_across_threads(monkeypatch, tmp_path):
  p = _bind(monkeypatch, tmp_path)
  M.sync_hud_prompt(_view(), p)
  got = []
  monkeypatch.setattr(M, "on_confirm_accuracy", lambda *a, **k: got.append(a))
  barrier = threading.Barrier(2)
  results = []

  def go(yes):
    barrier.wait()
    results.append(M.answer_prompt(4, yes, params=p))

  threads = [threading.Thread(target=go, args=(yes,)) for yes in (True, False)]
  for t in threads:
    t.start()
  for t in threads:
    t.join()
  assert results.count(True) == 1
  assert len(got) == 1
  assert M.answer_prompt(4, True, params=p, snapshot={"id": 4, "mph": 45, "way_id": 999, "heading": 1.0}) is False
  assert len(got) == 1


def test_snapshot_before_publish_still_uses_that_read(monkeypatch, tmp_path):
  p = _bind(monkeypatch, tmp_path)
  seen = {}

  def confirm(mph, yes, proposal=None):
    seen.update(mph=mph, yes=yes, proposal=proposal)

  monkeypatch.setattr(M, "on_confirm_accuracy", confirm)
  snap = {"id": 7, "mph": 55, "way_id": 333, "heading": 180.0}
  assert M.answer_prompt(7, True, params=p, snapshot=snap)
  assert seen["proposal"]["way_id"] == 333 and seen["proposal"]["heading"] == 180.0
  assert M.answer_prompt(7, False, params=p, snapshot=dict(snap, way_id=1)) is False


def test_store_failure_still_confirms_the_comma_snapshot(monkeypatch, tmp_path):
  """A missing prompt param must not drop the comma tap. The read-time way is the snapshot."""
  p = _bind(monkeypatch, tmp_path)
  seen = []

  def confirm(mph, yes, proposal=None):
    seen.append((mph, yes, dict(proposal or {})))

  def boom(key, value, *a, **k):
    raise RuntimeError("param unavailable")

  monkeypatch.setattr(M, "on_confirm_accuracy", confirm)
  p.put = boom
  snap = {"id": 4, "mph": 45, "way_id": 111, "heading": 90.0}
  assert M.answer_prompt(4, True, params=p, snapshot=snap)
  assert seen == [(45, True, {"seq": 4, "way_id": 111, "heading": 90.0})]
  assert M.answer_prompt(4, True, params=p) is False


def test_hook_surface_has_no_carstate_or_log_field():
  hud = (ROOT / "selfdrive/ui/onroad/speed_sign_hud.py").read_text(encoding="utf-8")
  impl = (ROOT / "selfdrive/speedsignd/mcu_prompt.py").read_text(encoding="utf-8")
  api = (ROOT / "selfdrive/nap_dash/speed_sign_api.py").read_text(encoding="utf-8")
  keys = (ROOT / "common/nap_params_keys.h").read_text(encoding="utf-8")
  log = (ROOT / "cereal/log.capnp").read_text(encoding="utf-8")
  assert "sync_hud_prompt" in hud and "answer_prompt" in hud
  assert "Params(" not in hud
  assert "threading.Thread(target=_safe_confirm" in hud
  for text in (impl, api, hud):
    assert "carState" not in text
    assert "SubMaster" not in text
    assert "panda" not in text.lower()
    assert "controlsd" not in text
  assert "NAPSpeedSignPrompt" in keys
  assert "NAPSpeedSignPrompt" not in log
  body = impl.split("def on_confirm_accuracy", 1)[1].split("\ndef ", 1)[0]
  assert body.index("record_answer") < body.index("write_answer") < body.index("request_override")
  assert "if yes:" in body
