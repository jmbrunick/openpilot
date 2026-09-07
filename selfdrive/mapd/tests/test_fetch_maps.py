import http.server
import os
import threading
from functools import partial

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.fetch_maps import fetch_and_install, installed_db_summary
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB


def _serve(directory: str, port: int) -> http.server.HTTPServer:
  handler = partial(http.server.SimpleHTTPRequestHandler, directory=directory)
  httpd = http.server.HTTPServer(("127.0.0.1", port), handler)
  threading.Thread(target=httpd.serve_forever, daemon=True).start()
  return httpd


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

  httpd = _serve(str(tmp_path), 0)
  port = httpd.server_address[1]
  dest = str(tmp_path / "installed" / "speed_limits.sqlite")
  try:
    fetch_and_install(dest=dest, url=f"http://127.0.0.1:{port}/speed_limits.sqlite", sha256="")
  finally:
    httpd.shutdown()

  db = OsmSpeedLimitDB(dest)
  assert db.open()
  m = db.lookup(37.0004, -122.0, bearing_deg=0.0)
  assert m is not None and m.way_id == 1
  db.close()
  summary = installed_db_summary(dest)
  assert "Installed" in summary
  assert os.path.getsize(dest) > 1024
