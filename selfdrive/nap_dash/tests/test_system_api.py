"""nap_dash /api/software — UpdaterTargetBranch + SIGUSR1. No SSH. No engage."""
from __future__ import annotations

import json
from http.client import HTTPConnection
from threading import Thread

import pytest

from openpilot.selfdrive.nap_dash.server import Handler, ThreadingHTTPServer
from openpilot.selfdrive.nap_dash.system_api import (
  SoftwareError,
  handle_software,
  read_software,
  sanitize_branch,
)


class FakeParams:
  def __init__(self, values=None, bools=None):
    self.values = dict(values or {})
    self.bools = dict(bools or {})
    self.writes: list[tuple[str, object]] = []

  def get(self, key, return_default=False):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.bools.get(key, False))

  def put(self, key, value, block=False):
    self.writes.append((key, value))
    self.values[key] = value


def test_sanitize_branch_matches_hub_rules():
  assert sanitize_branch("nap-release") == "nap-release"
  assert sanitize_branch("cursor/nap-dash-dev-e946") == "cursor/nap-dash-dev-e946"
  with pytest.raises(SoftwareError):
    sanitize_branch("../etc/passwd")
  with pytest.raises(SoftwareError):
    sanitize_branch("engage")
  with pytest.raises(SoftwareError):
    sanitize_branch("branch;reboot")


def test_set_branch_writes_target_and_pings_updated():
  params = FakeParams(values={"GitBranch": "nap-dev", "UpdaterTargetBranch": "nap-dev"})
  pings = []
  snap = handle_software(
    {"action": "set_branch", "branch": "nap-release"},
    params,
    pinger=lambda sig: pings.append(sig),
  )
  assert snap["target_branch"] == "nap-release"
  assert ("UpdaterTargetBranch", "nap-release") in params.writes
  assert pings == ["USR1"]


def test_fetch_pings_without_writing_branch():
  params = FakeParams(values={"UpdaterTargetBranch": "nap-dev"})
  pings = []
  handle_software({"action": "fetch"}, params, pinger=lambda sig: pings.append(sig))
  assert params.writes == []
  assert pings == ["USR1"]


def test_rejects_engage_and_uninstall():
  params = FakeParams()
  for payload in ({"action": "engage"}, {"action": "uninstall"}, {"action": "set_branch", "branch": "engage"}):
    with pytest.raises(SoftwareError):
      handle_software(payload, params, pinger=lambda *_a: None)


def test_read_software_lists_available_branches():
  params = FakeParams(
    values={
      "GitBranch": "nap-dev",
      "GitCommit": "abc123",
      "UpdaterTargetBranch": "nap-release",
      "UpdaterAvailableBranches": "nap-release,nap-dev,cursor/nap-dash-dev-e946",
      "UpdaterState": "idle",
    },
    bools={"UpdaterFetchAvailable": True},
  )
  snap = read_software(params)
  assert snap["git_branch"] == "nap-dev"
  assert snap["available_branches"] == ["nap-release", "nap-dev", "cursor/nap-dash-dev-e946"]
  assert snap["fetch_available"] is True


def test_http_software_round_trip(monkeypatch):
  from openpilot.selfdrive.nap_dash import server as srv

  params = FakeParams(values={"GitBranch": "nap-dev", "UpdaterTargetBranch": "nap-dev"})
  monkeypatch.setattr(srv, "PARAMS", params)
  monkeypatch.setattr(srv, "_params", lambda: params)

  httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
  thread = Thread(target=httpd.serve_forever, daemon=True)
  thread.start()
  try:
    host, port = httpd.server_address
    conn = HTTPConnection(host, port, timeout=3)
    conn.request("GET", "/api/software")
    got = json.loads(conn.getresponse().read().decode())
    assert got["git_branch"] == "nap-dev"
    conn.close()

    conn = HTTPConnection(host, port, timeout=3)
    conn.request(
      "POST",
      "/api/software",
      body=json.dumps({"action": "set_branch", "branch": "nap-release"}).encode(),
      headers={"Content-Type": "application/json"},
    )
    resp = conn.getresponse()
    data = json.loads(resp.read().decode())
    assert resp.status == 200
    assert data["target_branch"] == "nap-release"
    conn.close()

    conn = HTTPConnection(host, port, timeout=3)
    conn.request(
      "POST",
      "/api/software",
      body=json.dumps({"action": "engage"}).encode(),
      headers={"Content-Type": "application/json"},
    )
    bad = conn.getresponse()
    err = json.loads(bad.read().decode())
    assert bad.status == 400
    assert "not allowed" in err["error"]
    conn.close()
  finally:
    httpd.shutdown()
    httpd.server_close()
