from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB, _pack_coords, _unpack_coords, simplify_coords
from openpilot.selfdrive.mapd.overpass import ways_from_overpass
from openpilot.selfdrive.mapd.speed_limit import parse_maxspeed


def test_parse_maxspeed_units():
  assert parse_maxspeed(None) is None
  assert parse_maxspeed("") is None
  assert parse_maxspeed("signals") is None
  assert parse_maxspeed("US:urban") is None
  mph = parse_maxspeed("65 mph")
  assert mph is not None and abs(mph - 65 * CV.MPH_TO_MS) < 1e-6
  kph = parse_maxspeed("100")
  assert kph is not None and abs(kph - 100 * CV.KPH_TO_MS) < 1e-6
  assert parse_maxspeed("50 km/h") == parse_maxspeed("50")


def test_rtree_lookup_and_heading(tmp_path):
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  # Eastbound 45 mph way through (37.0, -122.0)
  OsmSpeedLimitDB.insert_way(
    con, 1, "Test Rd", "primary", 45 * CV.MPH_TO_MS,
    [(37.0, -122.001), (37.0, -121.999)],
  )
  # Parallel 25 mph way 200m north — should not win
  OsmSpeedLimitDB.insert_way(
    con, 2, "Side St", "residential", 25 * CV.MPH_TO_MS,
    [(37.002, -122.001), (37.002, -121.999)],
  )
  con.commit()
  con.close()

  db = OsmSpeedLimitDB(path)
  assert db.open()
  m = db.lookup(37.0, -122.0, bearing_deg=90.0)
  assert m is not None
  assert m.way_id == 1
  assert abs(m.speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.1
  assert m.road_name == "Test Rd"

  miss = db.lookup(38.0, -122.0, bearing_deg=90.0)
  assert miss is None
  db.close()


def test_overpass_json_import(tmp_path):
  payload = {
    "elements": [
      {
        "type": "way",
        "id": 42,
        "tags": {"highway": "residential", "name": "Oak", "maxspeed": "25 mph"},
        "geometry": [
          {"lat": 37.5, "lon": -122.4},
          {"lat": 37.501, "lon": -122.4},
        ],
      },
      {
        "type": "way",
        "id": 43,
        "tags": {"highway": "service", "maxspeed": "signals"},
        "geometry": [{"lat": 37.5, "lon": -122.4}, {"lat": 37.5, "lon": -122.401}],
      },
    ]
  }
  ways = ways_from_overpass(payload)
  assert len(ways) == 1
  assert ways[0]["way_id"] == 42
  out = str(tmp_path / "out.sqlite")
  con = OsmSpeedLimitDB.create(out)
  for w in ways:
    OsmSpeedLimitDB.insert_way(con, w["way_id"], w["name"], w["highway"], w["maxspeed_ms"], w["coords"])
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(out)
  assert db.open()
  m = db.lookup(37.5005, -122.4, bearing_deg=0.0)
  assert m is not None and m.way_id == 42


def test_simplify_collinear_and_f64_unpack():
  # 1 km eastbound straight line at 1 m spacing should collapse to endpoints.
  coords = [(37.0, -122.0 + i * 1e-5) for i in range(100)]
  simple = simplify_coords(coords, tol_m=5.0)
  assert len(simple) == 2
  assert simple[0] == coords[0] and simple[-1] == coords[-1]

  packed64 = _pack_coords([(37.5, -122.4), (37.6, -122.4)])
  # Force a legacy float64 blob and ensure unpack still works.
  import struct
  n = 2
  blob = struct.pack("<I", n) + struct.pack("<dd", 37.5, -122.4) + struct.pack("<dd", 37.6, -122.4)
  pts = _unpack_coords(blob)
  assert abs(pts[0][0] - 37.5) < 1e-9
  assert len(_unpack_coords(packed64)) == 2

