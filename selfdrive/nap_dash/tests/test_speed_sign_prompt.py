"""MCU page shows the speed-sign prompt and answers it once, with no second dialog."""
from __future__ import annotations

import json
from http.client import HTTPConnection
from pathlib import Path
from threading import Thread

import pytest

from openpilot.selfdrive.nap_dash.speed_sign_api import SignError, handle_sign, read_sign
from openpilot.selfdrive.speedsignd.hud import LiveSignView
from openpilot.selfdrive.speedsignd.mcu_prompt import sync_hud_prompt


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


def _open(monkeypatch, tmp_path, seq=9, mph=35, way=111, heading=90.0):
  monkeypatch.setenv("NAP_SPEED_SIGN_PROMPT_LOCK", str(tmp_path / "prompt.lock"))
  p = Mem()
  sync_hud_prompt(LiveSignView(
    show_mph=True, mph=25, proposal_mode=True,
    proposed_mph=mph, proposed_seq=seq, proposed_way_id=way, proposed_heading=heading,
  ), p)
  return p


def test_page_shows_read_speed_and_one_prompt(monkeypatch, tmp_path):
  p = _open(monkeypatch, tmp_path)
  snap = read_sign(p)
  assert snap["pending"] is True and snap["mph"] == 35 and snap["id"] == 9
  assert snap["prompt"] == "35 mph accurate?"
  assert snap["yes"] == "Yes" and snap["no"] == "No"
  assert "way_id" not in snap
  html = (ROOT / "selfdrive/nap_dash/dashboard.html").read_text(encoding="utf-8")
  assert 'id="ssign"' in html and 'id="ssign-mph"' in html
  assert "setInterval(ssTick, 500)" in html
  poll = html.split("function ssPoll()", 1)[1].split("function ", 1)[0]
  assert "/api/speed_sign?t=" in poll
  show = html.split("function showPanel", 1)[1].split("function ", 1)[0]
  assert "ssPoll" not in show and "ssRender" not in show
  assert html.find('id="content"') < html.find('id="ssign"')
  render = html.split("function ssRender", 1)[1].split("function ", 1)[0]
  assert "ssId = data.id" in render
  chunk = html.split("function ssAnswer", 1)[1].split("function ", 1)[0]
  assert "/api/speed_sign" in chunk
  assert "confirm" not in chunk
  assert "window.confirm" not in html


def test_yes_and_no_ignore_client_way_and_need_no_extra_confirm(monkeypatch, tmp_path):
  seen = []

  def confirm(mph, yes, proposal=None):
    seen.append((mph, yes, dict(proposal or {})))

  monkeypatch.setattr("openpilot.selfdrive.speedsignd.mcu_prompt.on_confirm_accuracy", confirm)
  p = _open(monkeypatch, tmp_path, way=111, heading=90.0)
  snap = handle_sign({"answer": "yes", "id": 9, "way_id": 222, "heading": 0, "confirm": True}, p)
  assert snap["pending"] is False
  assert seen == [(35, True, {"seq": 9, "way_id": 111, "heading": 90.0})]
  with pytest.raises(SignError, match="not waiting"):
    handle_sign({"answer": "no", "id": 9}, p)
  assert len(seen) == 1

  p2 = _open(monkeypatch, tmp_path, seq=10, mph=50)
  assert handle_sign({"answer": "no", "id": 10}, p2)["pending"] is False
  assert seen[-1][1] is False and seen[-1][2]["way_id"] == 111


def test_mcu_accepts_current_read_id_and_refuses_stale(monkeypatch, tmp_path):
  """A newer read replaces the prompt. The MCU must answer that id, not the previous one."""
  monkeypatch.setattr(
    "openpilot.selfdrive.speedsignd.mcu_prompt.on_confirm_accuracy",
    lambda *a, **k: None,
  )
  p = _open(monkeypatch, tmp_path, seq=1, mph=30, way=900, heading=10.0)
  sync_hud_prompt(LiveSignView(
    show_mph=True, mph=30, proposal_mode=True,
    proposed_mph=30, proposed_seq=2, proposed_way_id=111, proposed_heading=90.0,
  ), p)
  snap = read_sign(p)
  assert snap["pending"] is True and snap["id"] == 2 and snap["mph"] == 30
  with pytest.raises(SignError, match="not waiting"):
    handle_sign({"answer": "yes", "id": 1}, p)
  assert read_sign(p)["pending"] is True and read_sign(p)["id"] == 2
  done = handle_sign({"answer": "yes", "id": snap["id"]}, p)
  assert done["pending"] is False
  with pytest.raises(SignError, match="not waiting"):
    handle_sign({"answer": "no", "id": 2}, p)


def test_bad_answer_and_stale_id(monkeypatch, tmp_path):
  monkeypatch.setattr(
    "openpilot.selfdrive.speedsignd.mcu_prompt.on_confirm_accuracy",
    lambda *a, **k: (_ for _ in ()).throw(AssertionError("should not confirm")),
  )
  p = _open(monkeypatch, tmp_path)
  for payload in ({"answer": "engage", "id": 9}, {"answer": "yes"}, {"id": 9}, {}):
    with pytest.raises(SignError):
      handle_sign(payload, p)
  assert read_sign(p)["pending"] is True


def test_http_round_trip_closes_prompt_and_engage_stays_blocked(monkeypatch, tmp_path):
  from openpilot.selfdrive.nap_dash import server as srv

  monkeypatch.setattr(
    "openpilot.selfdrive.speedsignd.mcu_prompt.on_confirm_accuracy",
    lambda *a, **k: None,
  )
  p = _open(monkeypatch, tmp_path)
  monkeypatch.setattr(srv, "_params", lambda: p)
  httpd = srv.ThreadingHTTPServer(("127.0.0.1", 0), srv.Handler)
  Thread(target=httpd.serve_forever, daemon=True).start()
  host, port = httpd.server_address

  def call(method, path, body=None):
    conn = HTTPConnection(host, port, timeout=3)
    raw = json.dumps(body).encode() if body is not None else None
    conn.request(method, path, raw, headers={"Content-Type": "application/json"})
    resp = conn.getresponse()
    headers = {k.lower(): v for k, v in resp.getheaders()}
    out = (resp.status, json.loads(resp.read().decode()), headers)
    conn.close()
    return out

  try:
    status, data, headers = call("GET", "/api/speed_sign?t=1")
    assert status == 200 and data["pending"] is True and data["mph"] == 35
    assert "no-store" in headers.get("cache-control", "")
    assert headers.get("pragma") == "no-cache"
    status, data, _headers = call("POST", "/api/speed_sign", {"answer": "yes", "id": data["id"], "way_id": 999})
    assert status == 200 and data["pending"] is False
    assert call("POST", "/api/speed_sign", {"answer": "yes", "id": 9})[0] == 400
    assert call("POST", "/api/set", {"action": "engage"})[0] == 400
    assert call("GET", "/api/speed_sign")[1]["pending"] is False
  finally:
    httpd.shutdown()
    httpd.server_close()


def test_sources_do_not_subscribe_or_engage():
  server = (ROOT / "selfdrive/nap_dash/server.py").read_text(encoding="utf-8")
  api = (ROOT / "selfdrive/nap_dash/speed_sign_api.py").read_text(encoding="utf-8")
  assert "/api/speed_sign" in server
  assert "answer_prompt" in api
  for text in (server, api):
    assert "SubMaster" not in text
    assert "carState" not in text
    assert "panda" not in text.lower()
    assert "controlsd" not in text
  assert "engaging openpilot from the web is not allowed" in server
