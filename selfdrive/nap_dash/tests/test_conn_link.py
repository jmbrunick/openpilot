"""MCU connection dot: clock-only /api/ping and the header link indicator."""
from __future__ import annotations

import ast
import json
import shutil
import subprocess
import time
from http.client import HTTPConnection
from pathlib import Path
from threading import Thread

from openpilot.selfdrive.nap_dash.server import Handler, ThreadingHTTPServer, ping_payload
from openpilot.selfdrive.nap_dash.tests.test_settings import FakeParams

ROOT = Path(__file__).resolve().parents[3]
HTML = ROOT / "selfdrive" / "nap_dash" / "dashboard.html"
SERVER = ROOT / "selfdrive" / "nap_dash" / "server.py"


def test_ping_payload_is_clock_only():
  now = 1_700_000_000.123
  body = ping_payload(now)
  assert body == {"ok": True, "ts": 1_700_000_000_123}
  src = SERVER.read_text(encoding="utf-8")
  tree = ast.parse(src)
  fn = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "ping_payload")
  body = fn.body[1:] if ast.get_docstring(fn) else fn.body
  names = {node.id for node in ast.walk(ast.Module(body=body, type_ignores=[])) if isinstance(node, ast.Name)}
  for banned in ("Params", "SubMaster", "carState"):
    assert banned not in names
  dumped = ast.dump(ast.Module(body=body, type_ignores=[]))
  assert "messaging" not in dumped
  assert "cereal" not in dumped
  assert "time" in names


def test_ping_http_is_small_and_unstored(monkeypatch):
  from openpilot.selfdrive.nap_dash import server as srv

  params = FakeParams()
  monkeypatch.setattr(srv, "PARAMS", params)
  monkeypatch.setattr(srv, "_params", lambda: params)
  httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
  thread = Thread(target=httpd.serve_forever, daemon=True)
  thread.start()
  try:
    host, port = httpd.server_address
    conn = HTTPConnection(host, port, timeout=3)
    before = int(time.time() * 1000)
    conn.request("GET", "/api/ping?t=1")
    resp = conn.getresponse()
    raw = resp.read()
    after = int(time.time() * 1000)
    assert resp.status == 200
    assert "no-store" in resp.getheader("Cache-Control", "")
    assert resp.getheader("Pragma") == "no-cache"
    assert resp.getheader("Expires") == "0"
    assert len(raw) < 80
    body = json.loads(raw.decode("utf-8"))
    assert body["ok"] is True
    assert before - 1000 <= body["ts"] <= after + 1000
    conn.close()

    conn = HTTPConnection(host, port, timeout=3)
    conn.request("GET", "/api/ping")
    again = conn.getresponse()
    second = json.loads(again.read().decode("utf-8"))
    assert again.status == 200
    assert second["ts"] >= body["ts"]
    conn.close()
  finally:
    httpd.shutdown()
    httpd.server_close()


def _node_bin():
  found = shutil.which("node")
  if found:
    return found
  for candidate in ("/usr/bin/node", "/usr/local/bin/node", "/exec-daemon/node"):
    if Path(candidate).is_file():
      return candidate
  return None


def test_indicator_is_inline_and_outside_bookmark_slot():
  html = HTML.read_text(encoding="utf-8")
  header = html.split('<div class="top">', 1)[1].split('<div id="sub"', 1)[0]
  nav = header.index('id="nav"')
  conn = header.index('id="conncell"')
  drive = header.index('id="drivecell"')
  assert nav < conn < drive
  assert "position:" not in header
  assert 'id="bookmark"' not in header

  render = html.split("function renderNav()", 1)[1].split("function renderSub()", 1)[0]
  assert 'bm.id = "bookmark"' in render
  assert "nav.appendChild(bm)" in render
  assert "conncell" not in render
  assert "connlabel" not in render

  css = html.split("#drivecell", 1)[1].split("#main", 1)[0]
  assert "position:" not in css
  assert "width: 120px" in css
  assert "width: 152px" in css
  assert "width: 12px" in css
  assert "height: 12px" in css
  assert "table-layout: fixed" in html


def test_dashboard_link_copy_and_poll():
  html = HTML.read_text(encoding="utf-8")
  for text in (
    "Connected",
    "Reconnecting…",
    "No connection",
    "comma not reachable",
    "can't reach phone",
    "comma not updating",
    "Phone OK - comma not reachable",
    "Can't reach phone (check MCU Wi-Fi / hotspot)",
    "No connection to openpilot",
    "/api/ping",
    "LINK_POLL_MS = 2000",
    "LINK_TIMEOUT_MS = 3000",
    "LINK_STALE_MS = 6000",
    "actoff",
    "linknav",
    "reloadPageState",
  ):
    assert text in html, text
  assert "=>" not in html
  assert "let " not in html
  assert "const " not in html
  assert "fetch(" not in html
  assert "SubMaster" not in html
  assert "carState" not in html


def test_link_state_machine_matches_the_three_failures():
  node = _node_bin()
  if node is None:
    return
  html = HTML.read_text(encoding="utf-8")
  src = html.split("/* LINK_STATE_START */", 1)[1].split("/* LINK_STATE_END */", 1)[0]
  script = src + r"""
  function assert(cond, msg) { if (!cond) { throw new Error(msg); } }
  assert(classifyPing(200, 0) === "ok", "fresh");
  assert(classifyPing(200, 5999) === "ok", "just under 6s");
  assert(classifyPing(200, 6000) === "stale", "stale at 6s");
  assert(classifyPing(200, null) === "stale", "missing ts");
  assert(classifyPing(502, null) === "comma", "502");
  assert(classifyPing(503, null) === "comma", "503");
  assert(classifyPing(504, null) === "comma", "504");
  assert(classifyPing(0, null) === "phone", "network");
  assert(linkView(true, 99999) === "ok", "good wins");
  assert(linkView(false, 1000) === "retry", "one miss");
  assert(linkView(false, 6000) === "down", "six seconds");
  assert(linkLabel("ok") === "Connected", "green");
  assert(linkLabel("retry") === "Reconnecting…", "amber");
  assert(linkLabel("down") === "No connection", "red");
  assert(linkReason("comma") === "comma not reachable", "comma reason");
  assert(linkReason("phone") === "can't reach phone", "phone reason");
  assert(linkReason("stale") === "comma not updating", "stale reason");
  assert(linkDetail("ok", "", "3:04:05 PM") === "last update 3:04:05 PM", "connected detail");
  assert(linkDetail("down", "comma", "3:04:05 PM").indexOf("Phone OK - comma not reachable") === 0, "502 detail");
  assert(linkDetail("retry", "phone", "").indexOf("Can't reach phone (check MCU Wi-Fi / hotspot)") === 0, "phone detail");
  assert(linkDetail("down", "stale", "3:04:05 PM").indexOf("comma not updating") === 0, "stale detail");
  var clock = reconcileClock(5000, 1000, null);
  assert(clock.age === 0 && clock.offset === 4000, "first sample");
  var aged = reconcileClock(12000, 1000, clock.offset);
  assert(aged.age === 7000 && aged.offset === 4000, "cached body ages");
  var live = reconcileClock(7000, 3000, clock.offset);
  assert(live.age === 0, "live body stays fresh");
  assert(pingServerMs({ts: 1700000000123}) === 1700000000123, "ms ts");
  assert(pingServerMs({ts: 1700000000.5}) === 1700000000500, "seconds ts");
  assert(pingServerMs({}) === null, "no ts");
  assert(formatWhen(new Date(2020, 0, 15, 15, 4, 5).getTime()) === "3:04:05 PM", "clock text");
  """
  subprocess.check_call([node, "-e", script])
