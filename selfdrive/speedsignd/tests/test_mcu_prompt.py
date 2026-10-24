"""MCU and comma share one speed-sign Yes/No. Display and map only."""
from __future__ import annotations

import json
import threading
from pathlib import Path

from openpilot.selfdrive.speedsignd import confirm as C
from openpilot.selfdrive.speedsignd import mcu_prompt as M
from openpilot.selfdrive.speedsignd.hud import LiveSignView, hud_confirm_visible
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

  def record(mph, yes, seq=None):
    order.append("record_answer")
    (tmp_path / "feedback").mkdir(exist_ok=True)
    (tmp_path / "feedback" / "pending.json").write_text(json.dumps({
      "mph": 45, "seq": 1, "lat": 45.0, "lon": -93.0, "bearing": 90.0,
    }))
    return real_record(mph, yes, seq=seq)

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
  assert plate.plate_mph == 0 and plate.confirmed_mph == 45   # read cleared; mph went to mapd
  stored = SpeedOverrides(str(tmp_path / "map.json"))
  got = stored.consume(captured["param"], type("M", (), {"way_id": 222, "road_name": "now"})(), 0.0, 1.0)
  assert got["way_id"] == 111 and got["heading"] == 90.0
  assert stored.find(222, 0.0) is None
  assert M.read_pending(p) is None
  assert M.sync_hud_prompt(_view(seq=1), p) is True  # comma HUD hides; no second answer
  assert M.answer_prompt(1, True, params=p) is False
  assert order == ["record_answer", "write_answer", "request_override"]


def test_no_clears_the_read_and_skips_the_map(monkeypatch, tmp_path):
  p = _bind(monkeypatch, tmp_path)
  plate = C.PlateState()
  pr = plate.propose(45, 0.8, way_id=111, heading=90.0)
  M.sync_hud_prompt(_view(seq=pr.seq, mph=pr.mph, way=pr.way_id, heading=pr.heading), p)
  calls = []

  def record(mph, yes, seq=None):
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
  assert plate.plate_mph == 0 and plate.confirmed_mph == 0 and plate.proposal is None
  assert "request_override" not in calls
  assert [c[0] for c in calls] == ["record_answer", "write_answer"]


def test_timeout_clears_the_prompt_and_skips_the_map(monkeypatch, tmp_path):
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
  assert plate.plate_mph == 45 and plate.confirmed_mph == 0 and plate.proposal is not None
  assert plate.expire(pr.t + C.PROMPT_TIMEOUT_S + 0.01)   # +0.01: (t + 20) - t can round below 20              # speedsignd timeout clears the read
  assert plate.plate_mph == 0 and plate.proposal is None
  assert not plate.answer({"seq": pr.seq, "mph": 45, "yes": True})
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


def test_real_proposal_seq_is_not_replaced(monkeypatch, tmp_path):
  p = _bind(monkeypatch, tmp_path)
  assert M.sync_hud_prompt(_view(seq=4), p) is False
  assert M.read_pending(p)["id"] == 4


def test_string_json_put_still_publishes(monkeypatch, tmp_path):
  """A JSON param written as a dict raises TypeError when the key type is STRING."""
  p = _bind(monkeypatch, tmp_path)

  def put(key, value, *a, **k):
    if isinstance(value, dict):
      raise TypeError("type mismatch")
    p.store[key] = value

  p.put = put
  assert M.sync_hud_prompt(_view(seq=8, mph=30), p) is False
  assert M.read_pending(p) == {"id": 8, "mph": 30, "way_id": 111, "heading": 90.0}


def test_missing_param_key_falls_back_to_a_file(monkeypatch, tmp_path):
  """params_pyx built before NAPSpeedSignPrompt raises. The comma used to swallow that."""
  base = tmp_path / "params"
  base.mkdir()

  class Old:
    def __init__(self):
      self.store = {}

    def get_param_path(self, key=""):
      return str(base)

    def get(self, key, *a, **k):
      raise RuntimeError(f"UnknownKeyName {key}")

    def put(self, key, value, *a, **k):
      raise RuntimeError(f"UnknownKeyName {key}")

    def remove(self, key):
      raise RuntimeError(f"UnknownKeyName {key}")

  monkeypatch.setenv("NAP_SPEED_SIGN_PROMPT_LOCK", str(tmp_path / "prompt.lock"))
  p = Old()
  assert M.sync_hud_prompt(_view(seq=6, mph=25), p) is False
  assert M.read_pending(p)["mph"] == 25 and M.read_pending(p)["id"] == 6
  assert (base / "NAPSpeedSignPrompt").is_file()


def test_hook_surface_has_no_carstate_or_log_field():
  hud = (ROOT / "selfdrive/ui/onroad/speed_sign_hud.py").read_text(encoding="utf-8")
  impl = (ROOT / "selfdrive/speedsignd/mcu_prompt.py").read_text(encoding="utf-8")
  api = (ROOT / "selfdrive/nap_dash/speed_sign_api.py").read_text(encoding="utf-8")
  keys = (ROOT / "common/nap_params_keys.h").read_text(encoding="utf-8")
  log = (ROOT / "cereal/log.capnp").read_text(encoding="utf-8")
  assert "sync_hud_prompt" in hud and "answer_prompt" in hud
  assert "Params(" not in hud
  update = hud.split("def _update_state", 1)[1].split("\n  def ", 1)[0]
  assert "cloudlog.exception(\"speed_sign_hud: MCU prompt sync failed\")" in update
  assert "\n      pass\n" not in update
  render = hud.split("def _render", 1)[1]
  assert "draw_tici_speed_sign" in render and "self._view" in render
  widget = (ROOT / "system/ui/widgets/__init__.py").read_text(encoding="utf-8")
  widget_render = widget.split("def render", 1)[1].split("\n  def ", 1)[0]
  assert widget_render.index("_update_state") < widget_render.index("_render")
  onroad_hud = (ROOT / "selfdrive/ui/onroad/hud_renderer.py").read_text(encoding="utf-8")
  road = (ROOT / "selfdrive/ui/onroad/augmented_road_view.py").read_text(encoding="utf-8")
  assert "_speed_sign_hud.render" in onroad_hud
  assert "_hud_renderer.render" in road
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


# --- one prompt = one read: stale pairings -------------------------------------

def _capture(monkeypatch):
  """Real on_confirm_accuracy, recording what reaches mapd."""
  sent = []
  monkeypatch.setattr("openpilot.selfdrive.speedsignd.confirm.write_answer", lambda *a, **k: True)
  monkeypatch.setattr("openpilot.selfdrive.mapd.speed_overrides.request_override",
                      lambda mph, rec, seq, params=None, way_id=None, heading=None:
                      sent.append({"mph": mph, "way_id": way_id, "heading": heading,
                                   "lat": (rec or {}).get("lat")}) or True)
  return sent


def test_newer_same_mph_read_replaces_prompt_and_location():
  """Justin's 60->30 case: a 30 read outside town, then a real 30 in town."""
  plate = C.PlateState()
  out = plate.propose(30, 0.9, way_id=900, heading=10.0, lat=1.0, lon=1.0, read_id=("read", 1, 1.0))
  town = plate.propose(30, 0.9, way_id=111, heading=90.0, lat=2.0, lon=2.0, read_id=("read", 2, 9.0))
  assert town.seq != out.seq and town.way_id == 111 and town.lat == 2.0
  assert not plate.answer({"seq": out.seq, "mph": 30, "yes": True})     # old id rejected
  assert plate.answer({"seq": town.seq, "mph": 30, "yes": True})
  # The answered read does not ask again; an open read keeps its prompt.
  assert plate.propose(30, 0.95, way_id=555, heading=0.0, read_id=("read", 2, 9.0)) is None
  nxt = plate.propose(40, 0.9, way_id=7, read_id=("read", 3, 12.0))
  assert plate.propose(40, 0.9, way_id=8, read_id=("read", 3, 12.0)) is nxt and nxt.way_id == 7


def test_new_read_before_answer_rejects_old_id(monkeypatch, tmp_path):
  p = _bind(monkeypatch, tmp_path)
  sent = _capture(monkeypatch)
  M.sync_hud_prompt(_view(seq=1, mph=30, way=900, heading=10.0), p)
  M.sync_hud_prompt(_view(seq=2, mph=30, way=111, heading=90.0), p)    # newer read shown
  # MCU still holding id 1, comma snapshot of id 1: both refused.
  assert M.answer_prompt(1, True, params=p) is False
  assert M.answer_prompt(1, True, params=p, snapshot={"id": 1, "mph": 30, "way_id": 900, "heading": 10.0}) is False
  assert sent == []
  assert M.answer_prompt(2, True, params=p)
  assert sent == [{"mph": 30, "way_id": 111, "heading": 90.0, "lat": None}]


def test_late_yes_after_timeout_is_refused(monkeypatch, tmp_path):
  p = _bind(monkeypatch, tmp_path)
  sent = _capture(monkeypatch)
  M.sync_hud_prompt(_view(seq=3, mph=30, way=111), p)
  M.sync_hud_prompt(LiveSignView(show_mph=True, mph=30), p)            # timed out, hidden
  assert M.answer_prompt(3, True, params=p) is False
  assert M.answer_prompt(3, True, params=p, snapshot={"id": 3, "mph": 30, "way_id": 111, "heading": 90.0}) is False
  assert sent == []


def test_yes_from_mcu_and_comma_applies_once(monkeypatch, tmp_path):
  p = _bind(monkeypatch, tmp_path)
  sent = _capture(monkeypatch)
  M.sync_hud_prompt(_view(seq=5, mph=30, way=111, heading=90.0), p)
  assert M.answer_prompt(5, True, params=p)                             # MCU
  assert M.answer_prompt(5, True, params=p, snapshot={"id": 5, "mph": 30, "way_id": 111, "heading": 90.0}) is False
  assert len(sent) == 1 and sent[0]["way_id"] == 111


def test_pending_location_of_another_read_is_not_used(monkeypatch, tmp_path):
  """pending.json from a different read with the same mph must not lend its lat/lon."""
  p = _bind(monkeypatch, tmp_path)
  sent = _capture(monkeypatch)
  (tmp_path / "feedback").mkdir(exist_ok=True)
  (tmp_path / "feedback" / "pending.json").write_text(json.dumps({"mph": 30, "seq": 1, "lat": 45.0, "lon": -93.0}))
  M.sync_hud_prompt(_view(seq=2, mph=30, way=111, heading=90.0), p)
  assert M.answer_prompt(2, True, params=p)
  assert sent[0]["lat"] is None and sent[0]["way_id"] == 111


def test_comma_gate_hides_prompt_and_plate_on_answer():
  from openpilot.selfdrive.speedsignd.hud import AnswerGate
  g = AnswerGate()
  view = _view(seq=7, mph=45, plate=45)
  assert hud_confirm_visible(g.filter(view, 0.0))
  g.answer_seq(7)
  g.answer(45, True, 1.0)
  out = g.filter(view, 1.0)
  assert out.proposed_mph == 0 and out.mph == 0 and not out.show_mph
  assert not hud_confirm_visible(out)


def test_yes_without_read_time_road_never_writes_the_map(monkeypatch, tmp_path):
  p = _bind(monkeypatch, tmp_path)
  M.sync_hud_prompt(_view(seq=5, mph=55, way=0, heading=0.0), p)
  calls = []
  monkeypatch.setattr("openpilot.selfdrive.speedsignd.two_stage.record_answer",
                      lambda mph, yes, seq=None: {"lat": 1.0, "lon": 2.0, "bearing": 90.0})
  monkeypatch.setattr("openpilot.selfdrive.speedsignd.confirm.write_answer", lambda *a, **k: True)
  monkeypatch.setattr("openpilot.selfdrive.mapd.speed_overrides.request_override",
                      lambda *a, **k: calls.append(a) or True)
  assert M.answer_prompt(5, True, params=p)
  assert calls == []                       # no guessed road, nothing to mapd
  assert PARAM not in p.store
  assert M.read_pending(p) is None
  assert M.read_note(p) == M.NO_ROAD_NOTE  # comma HUD and MCU both read this
  from openpilot.selfdrive.nap_dash.speed_sign_api import read_sign
  assert read_sign(p) == {"pending": False, "note": M.NO_ROAD_NOTE}
  import time
  assert M.read_note(p, now=time.monotonic() + M.NOTE_S + 1) is None


def test_yes_with_road_has_no_note(monkeypatch, tmp_path):
  p = _bind(monkeypatch, tmp_path)
  M.sync_hud_prompt(_view(seq=6), p)
  monkeypatch.setattr("openpilot.selfdrive.speedsignd.two_stage.record_answer", lambda *a, **k: None)
  monkeypatch.setattr("openpilot.selfdrive.speedsignd.confirm.write_answer", lambda *a, **k: True)
  monkeypatch.setattr("openpilot.selfdrive.mapd.speed_overrides.request_override", lambda *a, **k: True)
  assert M.answer_prompt(6, True, params=p)
  assert M.read_note(p) is None
