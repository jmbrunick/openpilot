import hashlib
import os
import sqlite3

import zstandard

from openpilot.selfdrive.nap.maps_update import maps_updated as U


def _overlay_zst(tmp_path, rows=1):
  db = tmp_path / "src.sqlite"
  con = sqlite3.connect(db)
  con.execute("CREATE TABLE overhead(id INTEGER PRIMARY KEY, lat REAL, lon REAL, kind TEXT, under_way_id INTEGER, heading_deg REAL, source TEXT)")
  for i in range(rows):
    con.execute("INSERT INTO overhead VALUES(?,?,?,?,?,?,?)", (i, 44.0, -95.0, "bridge", 1, 0.0, "osm"))
  con.commit()
  con.close()
  zst = tmp_path / "overhead_us.sqlite.zst"
  zst.write_bytes(zstandard.ZstdCompressor().compress(db.read_bytes()))
  return zst, hashlib.sha256(zst.read_bytes()).hexdigest()


def test_due_version_daily_retry():
  assert U.due({}, "abc", 0.0)
  assert not U.due({"version": "abc", "checked_at": 100.0}, "abc", 200.0)
  assert U.due({"version": "abc", "checked_at": 0.0}, "abc", U.DAILY_S + 1)
  assert not U.due({"version": "abc", "checked_at": 0.0}, "abc", U.DAILY_S + 1, daily=False)
  assert not U.due({"version": "old", "retry_after": 500.0}, "abc", 100.0)
  assert not U.due({}, "", 0.0)


def test_overlay_allowed_metered():
  assert U.overlay_allowed({"bytes": 5_000_000}, metered=True)
  assert not U.overlay_allowed({"bytes": 50_000_000}, metered=True)
  assert U.overlay_allowed({"bytes": 50_000_000}, metered=False)
  assert not U.overlay_allowed({}, metered=True)


def test_install_overlay_atomic_and_verified(tmp_path):
  base = tmp_path / "osm"
  base.mkdir()
  zst, sha = _overlay_zst(tmp_path)
  live = base / "overhead_us.sqlite"
  live.write_bytes(b"old")
  assert not U.install_overlay("overhead", {"url": f"file://{zst}", "sha256": "0" * 64}, str(base))
  assert live.read_bytes() == b"old"                      # bad sha: untouched
  assert U.install_overlay("overhead", {"url": f"file://{zst}", "sha256": sha}, str(base))
  assert sqlite3.connect(live).execute("SELECT COUNT(*) FROM overhead").fetchone()[0] == 1
  assert sorted(os.listdir(base)) == ["overhead_us.sqlite"]   # no partials left


def test_run_once_metered_skips_big_pack_but_takes_small_overlay(tmp_path):
  base = tmp_path / "osm"
  base.mkdir()
  zst, sha = _overlay_zst(tmp_path)
  idx = {"overlays": {"overhead": {"revision": "1", "url": f"file://{zst}", "sha256": sha, "bytes": zst.stat().st_size}}}
  calls = []
  st = U.run_once(idx, {}, True, str(base), refresh_big=lambda: calls.append(1))
  assert calls == [] and st["big_checked"] is False and st["overlays"]["overhead"] == "1"
  st2 = U.run_once(idx, st, False, str(base), refresh_big=lambda: calls.append(1))
  assert calls == [1] and st2["big_checked"] is True
  big = dict(idx)
  big["overlays"] = {"overhead": dict(idx["overlays"]["overhead"], revision="2", bytes=10 ** 9)}
  assert U.run_once(big, st2, True, str(base))["overlays"]["overhead"] == "1"   # too big for cellular


def test_state_roundtrip(tmp_path):
  U.save_state(str(tmp_path), {"version": "x"})
  assert U.load_state(str(tmp_path)) == {"version": "x"}
  assert U.load_state(str(tmp_path / "none")) == {}
