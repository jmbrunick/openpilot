"""Roundabout map status: message validity, ring-data check, Map Speed Limit summary.

Kept apart from roundabout_map.py (pack rows / ring fit) so mapd, the settings pages and
tests share one small module. No cereal change.
"""
from __future__ import annotations

import os
import sqlite3

from openpilot.selfdrive.mapd.roundabout_map import ROLE_RING

RING_MISSING_TEXT = "Missing - run Refresh maps"


def map_msg_valid(*, gps_ok: bool, db_loaded: bool, matched: bool, ring_hint: bool) -> bool:
  """liveMapDataNAP.valid. A ring hint keeps the message valid without a speed match.

  A ring often has no maxspeed, so no speed-limit match. Without this the message
  is invalid there and readers that gate on valid never see the ring hint. Every
  speed-limit reader still gates on speedLimitValid, so a ring-only message
  publishes no limit. Non-ring frames are unchanged.
  """
  return bool(gps_ok and db_loaded and (matched or ring_hint))


def pack_ring_count(path: str | None) -> int | None:
  """Ring rows in a map pack. None if there is no readable pack; 0 if it has no ring data.

  Read-only and never raises: a v3 pack (no rb_ways table) and a v4 pack whose
  Refresh never stored a ring both report 0.
  """
  if not path or not os.path.isfile(path):
    return None
  try:
    con = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    try:
      row = con.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='rb_ways'").fetchone()
      if row is None:
        return 0
      return int(con.execute("SELECT COUNT(*) FROM rb_ways WHERE role = ?", (ROLE_RING,)).fetchone()[0])
    finally:
      con.close()
  except (sqlite3.Error, OSError, ValueError):
    return None


def ring_data_summary(count: int | None) -> str:
  if count is None:
    return "Not installed"
  if count <= 0:
    return RING_MISSING_TEXT
  return f"{count:,} rings"


def installed_ring_summary(path: str | None = None) -> str:
  """Roundabout ring rows in the installed pack. Says to refresh when there are none."""
  from openpilot.selfdrive.mapd.db_paths import default_db_path
  return ring_data_summary(pack_ring_count(path or default_db_path()))


class RingDataWatch:
  """One log line per pack state: the pack has (or lacks) ring rows.

  A pack without ring rows never produces a roundabout hint, so the slow-down
  silently never fires. check() returns a message the first time a pack file
  (path, mtime, size) is seen, and None until it changes.
  """

  def __init__(self) -> None:
    self._key: tuple | None = None

  def check(self, path: str | None) -> tuple[str, str] | None:
    """(level, message) once per pack state, else None."""
    try:
      st = os.stat(path) if path else None
    except OSError:
      st = None
    key = (path, st.st_mtime_ns, st.st_size) if st is not None else (path, None, None)
    if key == self._key:
      return None
    self._key = key
    count = pack_ring_count(path)
    if count is None:
      return None
    if count <= 0:
      return ("warning", f"mapd: {path} has no roundabout ring rows; the roundabout slow-down cannot fire"
                         + " (Settings \u2192 NAP \u2192 Map Speed Limit \u2192 Refresh maps)")
    return ("info", f"mapd: {path} has {count} roundabout ring rows")
