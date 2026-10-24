#!/usr/bin/env python3
"""List legacy (unversioned, not applied) sign overrides next to their OSM limit. Read-only.

  python3 -m openpilot.selfdrive.mapd.list_legacy_overrides [overrides.json] [osm.sqlite]
"""
from __future__ import annotations

import json
import sqlite3
import sys
import time

from openpilot.selfdrive.mapd.db_paths import default_db_path
from openpilot.selfdrive.mapd.speed_overrides import is_current, is_legacy, overrides_path

MS_TO_MPH = 2.23694


def osm_limit(con, way_id: int):
  if con is None:
    return None
  try:
    row = con.execute("SELECT name, highway, maxspeed_ms FROM ways WHERE way_id=?", (int(way_id),)).fetchone()
  except sqlite3.Error:
    return None
  if row is None:
    return None
  return row[0] or "", row[1] or "", float(row[2] or 0.0) * MS_TO_MPH


def rows(path: str, db: str) -> list[dict]:
  with open(path) as f:
    data = json.load(f)
  try:
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
  except sqlite3.Error:
    con = None
  out = []
  for e in data.get("overrides", []):
    if not isinstance(e, dict) or not (is_legacy(e) or is_current(e)):
      continue
    o = osm_limit(con, e["way_id"])
    out.append({"legacy": is_legacy(e), "way_id": int(e["way_id"]), "heading": e.get("heading"), "mph": int(e["mph"]),
                "osm_mph": None if o is None else round(o[2]), "osm_name": "" if o is None else o[0],
                "road": e.get("road", ""), "t": e.get("t"), "lat": e.get("lat"), "lon": e.get("lon")})
  if con is not None:
    con.close()
  return out


def main(argv=None) -> int:
  argv = sys.argv[1:] if argv is None else argv
  path = argv[0] if argv else overrides_path()
  db = argv[1] if len(argv) > 1 else default_db_path()
  print(f"{'status':8} {'way':>11} {'hdg':>5} {'sign':>4} {'osm':>4}  saved             name")
  for r in rows(path, db):
    t = time.strftime("%Y-%m-%d %H:%M", time.localtime(r["t"])) if r["t"] else "?"
    osm = "?" if r["osm_mph"] is None else str(r["osm_mph"])
    status = "LEGACY" if r["legacy"] else "active"
    name = r["osm_name"] or r["road"]
    print(f"{status:8} {r['way_id']:>11} {r['heading'] or 0:>5.0f} {r['mph']:>4} {osm:>4}  {t:17} {name}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
