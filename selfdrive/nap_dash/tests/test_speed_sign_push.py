"""The comma pushes the speed-sign prompt to the MCU page (SSE, long-poll fallback)."""
from __future__ import annotations

import json
import socket
import time
from http.client import HTTPConnection
from pathlib import Path
from threading import Thread

import pytest

from openpilot.selfdrive.nap_dash import server as srv

HTML = (Path(srv.__file__).resolve().parent / "dashboard.html").read_text(encoding="utf-8")


class Box:
  def __init__(self, state):
    self.state = state

  def read(self):
    return dict(self.state)


@pytest.fixture
def watch(monkeypatch):
  box = Box({"pending": False})
  w = srv.SignWatch(read=box.read, params=lambda: object())
  w.WATCH_S = 0.01
  monkeypatch.setattr(srv, "SIGN_WATCH", w)
  monkeypatch.setattr(srv, "SSE_HEARTBEAT_S", 0.3)
  monkeypatch.setattr(srv, "LONG_POLL_S", 1.0)
  return w, box


@pytest.fixture
def httpd():
  server = srv.ThreadingHTTPServer(("127.0.0.1", 0), srv.Handler)
  Thread(target=server.serve_forever, daemon=True).start()
  try:
    yield server.server_address
  finally:
    server.shutdown()
    server.server_close()


def _events(sock_file, want, timeout=3.0):
  """Read SSE frames until `want` sign events (or timeout)."""
  out, hb, buf = [], 0, []
  end = time.monotonic() + timeout
  while len(out) < want and time.monotonic() < end:
    line = sock_file.readline()
    if not line:
      break
    line = line.decode().rstrip("\n")
    if line == "":
      for b in buf:
        if b.startswith("data: "):
          out.append(json.loads(b[6:]))
        elif b.startswith(": hb"):
          hb += 1
      buf = []
    else:
      buf.append(line)
  return out, hb


def _open_stream(addr):
  s = socket.create_connection(addr, timeout=3)
  s.sendall(b"GET /api/speed_sign/stream?t=1 HTTP/1.1\r\nHost: x\r\n\r\n")
  f = s.makefile("rb")
  status = f.readline()
  headers = {}
  while True:
    line = f.readline().decode().strip()
    if not line:
      break
    k, v = line.split(":", 1)
    headers[k.strip().lower()] = v.strip()
  return s, f, status, headers


def test_check_bumps_version_only_on_change(watch):
  w, box = watch
  assert w.check(force=True) and w.version == 1
  assert not w.check(force=True) and w.version == 1
  box.state = {"pending": True, "id": 3, "mph": 30}
  assert w.check(force=True) and w.version == 2
  assert w.payload() == {"pending": True, "id": 3, "mph": 30, "v": 2}


def test_stream_sends_state_on_connect_then_each_change(watch, httpd):
  w, box = watch
  s, f, status, headers = _open_stream(httpd)
  try:
    assert b" 200 " in status
    assert headers["content-type"] == "text/event-stream"
    assert "no-store" in headers["cache-control"]
    first, _ = _events(f, 1)
    assert first[0]["pending"] is False
    box.state = {"pending": True, "id": 7, "mph": 45, "prompt": "45 mph accurate?"}
    t0 = time.monotonic()
    got, _ = _events(f, 1)
    assert got[0]["pending"] is True and got[0]["mph"] == 45
    assert time.monotonic() - t0 < 0.5      # pushed, not the 5 s backstop poll
    box.state = {"pending": False}
    got, _ = _events(f, 1)
    assert got[0]["pending"] is False
  finally:
    s.close()


def test_stream_heartbeat_and_other_requests_not_blocked(watch, httpd):
  s, f, _, _ = _open_stream(httpd)
  try:
    _events(f, 1)
    conn = HTTPConnection(*httpd, timeout=2)
    conn.request("GET", "/api/ping")
    assert conn.getresponse().status == 200
    conn.close()
    _, hb = _events(f, 1, timeout=1.0)
    assert hb >= 1
  finally:
    s.close()


def test_stream_cap_newest_wins(watch):
  w, _ = watch
  tokens = [w.open_stream() for _ in range(w.MAX_STREAMS + 2)]
  assert not w.stream_alive(tokens[0]) and not w.stream_alive(tokens[1])
  assert all(w.stream_alive(t) for t in tokens[2:])
  for t in tokens:
    w.close_stream(t)
  assert w._streams == []


def test_long_poll_returns_on_change(watch, httpd):
  w, box = watch
  w.start()
  v = w.version

  def flip():
    time.sleep(0.2)
    box.state = {"pending": True, "id": 2, "mph": 25}
  Thread(target=flip, daemon=True).start()
  conn = HTTPConnection(*httpd, timeout=3)
  t0 = time.monotonic()
  conn.request("GET", f"/api/speed_sign?wait={v}&t=1")
  resp = conn.getresponse()
  data = json.loads(resp.read())
  assert resp.status == 200 and data["pending"] is True and data["v"] > v
  assert time.monotonic() - t0 < 0.9
  conn.close()


def test_long_poll_times_out_with_current_state(watch, httpd):
  w, _ = watch
  w.start()
  conn = HTTPConnection(*httpd, timeout=3)
  conn.request("GET", f"/api/speed_sign?wait={w.version}")
  data = json.loads(conn.getresponse().read())
  assert data["pending"] is False and data["v"] == w.version
  conn.close()


def test_long_poll_stale_version_returns_now(watch, httpd):
  w, _ = watch
  w.start()
  conn = HTTPConnection(*httpd, timeout=3)
  t0 = time.monotonic()
  conn.request("GET", "/api/speed_sign?wait=-1")
  json.loads(conn.getresponse().read())
  assert time.monotonic() - t0 < 0.5
  conn.close()


def test_param_file_change_is_seen(tmp_path):
  class P:
    def get_param_path(self, key=""):
      return str(tmp_path)
  box = Box({"pending": False})
  w = srv.SignWatch(read=box.read, params=lambda: P())
  (tmp_path / "NAPSpeedSignPrompt").write_text("{}")
  w.check(force=True)
  box.state = {"pending": True, "id": 1, "mph": 30}
  (tmp_path / "NAPSpeedSignPrompt").write_text('{"id": 1, "mph": 30}')
  assert w.check() is True


def test_page_uses_push_with_fallbacks():
  assert 'new EventSource("/api/speed_sign/stream?t="' in HTML
  assert 'addEventListener("sign"' in HTML
  assert '"/api/speed_sign?wait="' in HTML
  assert "if (window.EventSource) ssStreamOpen(); else ssLongPoll();" in HTML
  assert "var SS_PUSH_BACKSTOP_MS = 5000;" in HTML
  pushed = HTML.split("function ssPushed", 1)[1].split("function ", 1)[0]
  assert '!document.getElementById("ssign")' in pushed and "location.reload(true)" in pushed
  assert "ssRender(data)" in pushed
  assert "fetch(" not in HTML and "=>" not in HTML
