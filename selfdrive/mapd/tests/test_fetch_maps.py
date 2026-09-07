import os
from pathlib import Path

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.fetch_maps import fetch_and_install, installed_db_summary
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB


def test_fetch_installs_sqlite(tmp_path):
  src = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(src)
  OsmSpeedLimitDB.insert_way(
    con, 1, "Main", "primary", 35 * CV.MPH_TO_MS,
    [(37.0, -122.0), (37.001, -122.0)],
  )
  con.execute("INSERT OR REPLACE INTO meta(key, value) VALUES ('way_count', '1')")
  con.commit()
  con.close()

  dest = str(tmp_path / "installed" / "speed_limits.sqlite")
  fetch_and_install(dest=dest, url=Path(src).as_uri(), sha256="")

  db = OsmSpeedLimitDB(dest)
  assert db.open()
  m = db.lookup(37.0004, -122.0, bearing_deg=0.0)
  assert m is not None and m.way_id == 1
  db.close()
  summary = installed_db_summary(dest)
  assert "Installed" in summary
  assert os.path.getsize(dest) > 1024
