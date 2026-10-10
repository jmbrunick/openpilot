"""nap_dash /api/software — UpdaterTargetBranch + SIGUSR1. No SSH. No engage."""
from __future__ import annotations

import datetime
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
  time_ago,
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

  def put_bool(self, key, value):
    self.writes.append((key, bool(value)))
    self.bools[key] = bool(value)


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


def test_set_offline_writes_disable_updates():
  params = FakeParams()
  snap = handle_software(
    {"action": "set_offline", "offline": True},
    params,
    pinger=lambda *_a: None,
  )
  assert ("DisableUpdates", True) in params.writes
  assert snap["disable_updates"] is True
  assert snap["offline"] is True
  snap = handle_software(
    {"action": "set_offline", "offline": False},
    params,
    pinger=lambda *_a: None,
  )
  assert snap["offline"] is False


def test_download_sends_sighup_like_comma_software_panel():
  params = FakeParams()
  pings = []
  handle_software({"action": "download"}, params, pinger=lambda sig: pings.append(sig), running=True)
  assert pings == ["HUP"]


def test_check_and_download_blocked_when_updates_paused_or_onroad():
  for bools, words in (({"DisableUpdates": True}, "paused"), ({"IsOnroad": True}, "car is off")):
    params = FakeParams(bools=bools)
    pings: list[str] = []
    for action in ("fetch", "download"):
      with pytest.raises(SoftwareError, match=words):
        handle_software({"action": action}, params, pinger=pings.append, running=True)
    assert pings == []
    snap = read_software(params, running=True)
    assert words in snap["blocked_reason"]
    assert snap["can_check"] is False and snap["can_download"] is False


def test_status_mirrors_updater_params():
  base = {"UpdaterTargetBranch": "nap-dev"}
  snap = read_software(FakeParams(values=dict(base, UpdaterState="checking...")), running=True)
  assert snap["busy"] is True and snap["status_text"] == "checking…"
  assert snap["can_check"] is False

  snap = read_software(FakeParams(values=dict(base, UpdaterState="downloading...")), running=True)
  assert snap["status_text"] == "downloading…"

  recent = datetime.datetime.now(datetime.UTC).replace(tzinfo=None) - datetime.timedelta(minutes=5)
  snap = read_software(FakeParams(values=dict(base, UpdaterState="idle", LastUpdateTime=recent)), running=True)
  assert snap["status_text"] == "up to date, last checked 5 minutes ago"
  assert snap["last_update_time"].endswith("Z")
  assert snap["can_check"] is True and snap["next_action"] == "check"

  snap = read_software(FakeParams(values=dict(base), bools={"UpdaterFetchAvailable": True}), running=True)
  assert snap["status_text"].startswith("update available")
  assert snap["next_action"] == "download" and snap["can_download"] is True

  snap = read_software(
    FakeParams(values=dict(base, UpdaterNewDescription="0.10.1 / nap-dev / abc1234"), bools={"UpdateAvailable": True}),
    running=True,
  )
  assert snap["status_text"].startswith("update ready, reboot to install (0.10.1")

  snap = read_software(
    FakeParams(values=dict(base, UpdateFailedCount=2, LastUpdateException="command failed: git fetch")),
    running=True,
  )
  assert snap["update_failed_count"] == 2
  assert snap["status_text"].startswith("failed to check for update")
  assert snap["last_update_exception"] == "command failed: git fetch"


def test_status_flags_stopped_updater_and_time_ago_buckets():
  snap = read_software(FakeParams(), running=False)
  assert snap["updater_running"] is False
  assert "not running" in snap["blocked_reason"]
  assert snap["status_text"] == "up to date, last checked never"
  now = datetime.datetime(2026, 10, 8, 12, 0, 0)
  assert time_ago(None, now) == "never"
  assert time_ago(now - datetime.timedelta(seconds=10), now) == "now"
  assert time_ago(now - datetime.timedelta(hours=1), now) == "1 hour ago"
  assert time_ago(now - datetime.timedelta(days=3), now) == "3 days ago"


def test_install_and_reboot_mirrors_native_install():
  ready = {"UpdaterNewDescription": "0.10.1 / nap-dev / abc1234 / 2026-10-08"}
  params = FakeParams(values=dict(ready), bools={"UpdateAvailable": True})
  snap = read_software(params, running=True)
  assert snap["can_install"] is True
  assert snap["install_label"] == "Install & reboot with 0.10.1 / nap-dev"

  # confirm tap is required
  with pytest.raises(SoftwareError, match="confirmation"):
    handle_software({"action": "install"}, params, pinger=lambda *_a: None, running=True)
  assert ("DoReboot", True) not in params.writes

  snap = handle_software({"action": "install", "confirm": True}, params, pinger=lambda *_a: None, running=True)
  assert ("DoReboot", True) in params.writes
  assert snap["rebooting"] is True


def test_install_label_keeps_slash_branch_names():
  params = FakeParams(
    values={"UpdaterNewDescription": "0.10.1 / cursor/mcu-web-ui-c588 / abc1234 / 2026-10-08"},
    bools={"UpdateAvailable": True},
  )
  assert read_software(params, running=True)["install_label"] == "Install & reboot with 0.10.1 / cursor/mcu-web-ui-c588"


def test_install_blocked_onroad_engaged_or_without_update():
  ready = {"UpdaterNewDescription": "0.10.1 / nap-dev / abc1234 / 2026-10-08"}
  cases = (
    ({"UpdateAvailable": True, "IsOnroad": True}, "car is on"),
    ({"UpdateAvailable": True, "IsOnroad": True, "IsEngaged": True}, "engaged"),
    ({}, "No downloaded update"),
  )
  for bools, words in cases:
    params = FakeParams(values=dict(ready), bools=bools)
    with pytest.raises(SoftwareError, match=words):
      handle_software({"action": "install", "confirm": True}, params, pinger=lambda *_a: None, running=True)
    assert ("DoReboot", True) not in params.writes
    assert read_software(params, running=True)["can_install"] is False


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
    bools={"UpdaterFetchAvailable": True, "DisableUpdates": True},
  )
  snap = read_software(params)
  assert snap["git_branch"] == "nap-dev"
  assert snap["available_branches"] == ["nap-release", "nap-dev", "cursor/nap-dash-dev-e946"]
  assert snap["fetch_available"] is True
  assert snap["offline"] is True


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
