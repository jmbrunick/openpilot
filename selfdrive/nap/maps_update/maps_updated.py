"""NAP map auto-update (offroad daemon). Self-contained; the one hook is its
PythonProcess line in system/manager/process_config.py (only_offroad).

When: after a software version change (git commit differs from the last
check) and, optionally, once a day (NAP_MAPS_DAILY, default on).
What: reads the published maps-index.json (same URL Download US Maps uses).
  * Big speed-limit pack: fetch_maps.refresh_maps() (sha-checked, atomic) but
    only on an unmetered link. Hotspot / cellular is metered: skipped.
  * Overlays (index["overlays"], e.g. "overhead"): small versioned sqlite.zst
    files. Allowed on a metered link when bytes <= OVERLAY_CELL_MAX_BYTES.
    Download to a .partial, verify sha256 of the zst, decompress, open it and
    check the expected table, then os.replace() onto the live file. mapd picks
    the new file up by mtime. A failure leaves the old file untouched.
State lives in a JSON sidecar next to the maps (no new params).
Nothing here publishes anything; it only reads the public index.
"""
from __future__ import annotations

import hashlib
import json
import os
import sqlite3
import time
import urllib.request

CHECK_STATE = "nap_maps_update.json"
OVERLAY_CELL_MAX_BYTES = 20 * 1024 * 1024
DAILY_S = 24 * 3600
LOOP_S = 60.0
RETRY_S = 30 * 60
OVERLAY_FILES = {"overhead": ("overhead_us.sqlite", "overhead")}   # name -> (file, required table)
USER_AGENT = "NotAutopilot-maps-update/1.0 (OSM ODbL; https://github.com/jmbrunick/openpilot)"


def osm_dir() -> str:
  from openpilot.selfdrive.mapd.db_paths import osm_dir as _d
  return _d()


def overlay_path(name: str, base: str | None = None) -> str:
  return os.path.join(base or osm_dir(), OVERLAY_FILES[name][0])


def load_state(base: str) -> dict:
  try:
    with open(os.path.join(base, CHECK_STATE)) as f:
      d = json.load(f)
    return d if isinstance(d, dict) else {}
  except (OSError, ValueError):
    return {}


def save_state(base: str, state: dict) -> None:
  os.makedirs(base, exist_ok=True)
  p = os.path.join(base, CHECK_STATE)
  tmp = p + ".tmp"
  with open(tmp, "w") as f:
    json.dump(state, f)
  os.replace(tmp, p)


def due(state: dict, version: str, now: float, daily: bool = True) -> bool:
  if not version:
    return False
  if now < float(state.get("retry_after", 0.0)):
    return False
  if state.get("version") != version:
    return True
  return daily and now - float(state.get("checked_at", 0.0)) >= DAILY_S


def overlay_allowed(entry: dict, metered: bool) -> bool:
  try:
    size = int(entry.get("bytes", 0))
  except (TypeError, ValueError):
    return False
  return (not metered) or (0 < size <= OVERLAY_CELL_MAX_BYTES)


def _sha256(path: str) -> str:
  h = hashlib.sha256()
  with open(path, "rb") as f:
    for chunk in iter(lambda: f.read(1 << 20), b""):
      h.update(chunk)
  return h.hexdigest()


def _fetch(url: str, dest: str) -> None:
  if url.startswith("file://"):
    with open(url[len("file://"):], "rb") as src, open(dest, "wb") as out:
      out.write(src.read())
    return
  req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
  with urllib.request.urlopen(req, timeout=60) as resp, open(dest, "wb") as out:
    while True:
      b = resp.read(1 << 20)
      if not b:
        break
      out.write(b)


def install_overlay(name: str, entry: dict, base: str) -> bool:
  """Download, verify, decompress, check, then atomically replace. True when installed."""
  file_name, table = OVERLAY_FILES[name]
  live = os.path.join(base, file_name)
  zst = live + ".zst.partial"
  raw = live + ".partial"
  try:
    _fetch(str(entry["url"]), zst)
    want = str(entry.get("sha256", "")).lower()
    if not want or _sha256(zst) != want:
      raise ValueError("sha256 mismatch")
    if str(entry["url"]).endswith(".zst"):
      import zstandard
      with open(zst, "rb") as src, open(raw, "wb") as out:
        zstandard.ZstdDecompressor().copy_stream(src, out)
    else:
      os.replace(zst, raw)
    con = sqlite3.connect(raw)
    try:
      con.execute(f"SELECT 1 FROM {table} LIMIT 1").fetchall()
    finally:
      con.close()
    os.replace(raw, live)
    return True
  except Exception:
    return False
  finally:
    for p in (zst, raw):
      try:
        os.remove(p)
      except OSError:
        pass


def run_once(index: dict, state: dict, metered: bool, base: str, refresh_big=None) -> dict:
  """One check. Returns the new state (caller saves it)."""
  out = dict(state)
  overlays = out.setdefault("overlays", {})
  for name, entry in (index.get("overlays") or {}).items():
    if name not in OVERLAY_FILES or not isinstance(entry, dict):
      continue
    rev = str(entry.get("revision", ""))
    have = overlays.get(name, "")
    if rev and (rev != have or not os.path.isfile(overlay_path(name, base))):
      if overlay_allowed(entry, metered) and install_overlay(name, entry, base):
        overlays[name] = rev
  if not metered and refresh_big is not None:
    try:
      refresh_big()
      out["big_checked"] = True
    except Exception:
      out["big_checked"] = False
  else:
    out["big_checked"] = False      # retried on the next unmetered check
  return out


def _version() -> str:
  try:
    from openpilot.common.git import get_commit
    return get_commit() or ""
  except Exception:
    return ""


def main() -> None:
  import cereal.messaging as messaging
  from openpilot.common.swaglog import cloudlog
  sm = messaging.SubMaster(["deviceState"])
  daily = os.environ.get("NAP_MAPS_DAILY", "1") != "0"
  version = _version()
  while True:
    sm.update(1000)
    now = time.time()  # noqa: TID251
    base = osm_dir()
    state = load_state(base)
    if sm.seen["deviceState"] and due(state, version, now, daily):
      metered = bool(sm["deviceState"].networkMetered)
      try:
        from openpilot.selfdrive.mapd.fetch_maps import refresh_maps
        from openpilot.selfdrive.mapd.maps_manifest import MAPS_INDEX_URL
        req = urllib.request.Request(os.environ.get("NAP_MAPS_INDEX_URL") or MAPS_INDEX_URL, headers={"User-Agent": USER_AGENT})
        with urllib.request.urlopen(req, timeout=30) as resp:
          index = json.loads(resp.read())
        new = run_once(index, state, metered, base, refresh_big=refresh_maps)
        new["checked_at"] = now
        if new.get("big_checked"):
          new["version"], new["retry_after"] = version, 0.0
        else:   # metered or failed: keep the old version so the big pack is retried
          new["version"], new["retry_after"] = state.get("version", ""), now + RETRY_S
        save_state(base, new)
        cloudlog.info("nap maps update: metered=%s overlays=%s big=%s", metered, new.get("overlays"), new.get("big_checked"))
      except Exception:
        cloudlog.exception("nap maps update failed")
        state["retry_after"] = now + RETRY_S
        save_state(base, state)
    time.sleep(LOOP_S)


if __name__ == "__main__":
  main()
