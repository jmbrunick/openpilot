"""Process wiring + HTTP smoke for the in-tree Dash. No live cereal required."""
from __future__ import annotations

import json
from http.client import HTTPConnection
from pathlib import Path
from threading import Thread

from openpilot.selfdrive.nap_dash.server import Handler, ThreadingHTTPServer
from openpilot.selfdrive.nap_dash.settings import read_settings, write_setting
from openpilot.selfdrive.nap_dash.tests.test_settings import FakeParams, PARAM_FOLLOW_DISTANCE

ROOT = Path(__file__).resolve().parents[3]


def test_process_config_registers_always_on_python_process():
  cfg = (ROOT / "system" / "manager" / "process_config.py").read_text(encoding="utf-8")
  assert 'PythonProcess("nap_dash"' in cfg
  assert "selfdrive.nap_dash.server" in cfg
  assert "always_run" in cfg
  assert "restart_if_crash=True" in cfg.split("nap_dash", 1)[1][:400]


def test_docs_cover_hotspot_and_mannerisms():
  docs = (ROOT / "docs-nap" / "nap-dash.md").read_text(encoding="utf-8")
  readme = (ROOT / "docs-nap" / "README.md").read_text(encoding="utf-8")
  assert "7070" in docs
  assert "comma hotspot" in docs.lower() or "hotspot" in docs.lower()
  assert "Driving Mannerisms" in docs
  assert "do not merge" in docs.lower()
  assert "nap-release" in docs
  assert "nap-dash.md" in readme
  assert "panda" in docs.lower()
  assert "NAPMapSpeedAccel" in docs
  assert "apply_hypermile_toggle" not in docs
  assert "NAPHypermile" not in docs
  assert "http://<device-ip>:7070" in docs or "http://<device>:7070" in docs


def test_onroad_cpu_budget_lists_nap_dash():
  onroad = (ROOT / "selfdrive" / "test" / "test_onroad.py").read_text(encoding="utf-8")
  assert '"selfdrive.nap_dash.server"' in onroad


def test_http_set_round_trip(monkeypatch):
  from openpilot.selfdrive.nap_dash import server as srv

  params = FakeParams(ints={PARAM_FOLLOW_DISTANCE: 4})
  monkeypatch.setattr(srv, "PARAMS", params)
  monkeypatch.setattr(srv, "locked_settings", lambda: read_settings(params))
  monkeypatch.setattr(srv, "locked_write", lambda name, value: write_setting(params, name, value))

  httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
  thread = Thread(target=httpd.serve_forever, daemon=True)
  thread.start()
  try:
    host, port = httpd.server_address
    conn = HTTPConnection(host, port, timeout=3)
    conn.request("GET", "/")
    home = conn.getresponse()
    body = home.read().decode("utf-8")
    assert home.status == 200
    assert "Driving Mannerisms" in body
    assert "speed_trim" not in body
    assert 'id="hypermile"' not in body
    conn.close()

    conn = HTTPConnection(host, port, timeout=3)
    payload = json.dumps({"name": "follow_distance", "value": 6}).encode()
    conn.request("POST", "/api/set", body=payload, headers={"Content-Type": "application/json"})
    resp = conn.getresponse()
    data = json.loads(resp.read().decode("utf-8"))
    assert resp.status == 200
    assert data["settings"]["follow_distance"] == 6
    assert params.ints[PARAM_FOLLOW_DISTANCE] == 6
    conn.close()

    conn = HTTPConnection(host, port, timeout=3)
    conn.request("POST", "/api/set", body=json.dumps({"name": "speed_trim", "value": 5}).encode(),
                 headers={"Content-Type": "application/json"})
    bad = conn.getresponse()
    err = json.loads(bad.read().decode("utf-8"))
    assert bad.status == 400
    assert "rejected" in err["error"] or "unknown" in err["error"]
    conn.close()

    conn = HTTPConnection(host, port, timeout=3)
    conn.request("POST", "/api/nav/update", body=json.dumps({"route_state": "active"}).encode(),
                 headers={"Content-Type": "application/json"})
    nav = conn.getresponse()
    assert nav.status == 404
    conn.close()
  finally:
    httpd.shutdown()
    httpd.server_close()


def test_server_source_has_no_nav_write_or_settings_file():
  src = (ROOT / "selfdrive" / "nap_dash" / "server.py").read_text(encoding="utf-8")
  assert "NAP_SETTINGS_FILE" not in src
  assert "update_nap_settings_file" not in src
  assert "/api/nav" not in src
  assert "PHONE_HTML" not in src
  assert "def update_navigation" not in src
