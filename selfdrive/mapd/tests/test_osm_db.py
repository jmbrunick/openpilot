from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import OSM_SIGN_LEAD_S, osm_sign_lead_m
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB, _offset_point, _pack_coords, _unpack_coords, simplify_coords
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


def test_lookahead_finds_upcoming_lower_limit(tmp_path):
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  # Eastbound 45 mph, then 25 mph after lon=-122.0 (~180 m at this latitude).
  OsmSpeedLimitDB.insert_way(
    con, 1, "Main", "primary", 45 * CV.MPH_TO_MS,
    [(37.0, -122.004), (37.0, -122.000)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 2, "Main", "primary", 25 * CV.MPH_TO_MS,
    [(37.0, -122.000), (37.0, -121.996)],
  )
  con.commit()
  con.close()

  db = OsmSpeedLimitDB(path)
  assert db.open()
  m = db.lookup(37.0, -122.002, bearing_deg=90.0)
  assert m is not None
  assert m.way_id == 1
  assert abs(m.speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.2
  assert abs(m.next_speed_limit_ms - 25 * CV.MPH_TO_MS) < 0.2
  assert 80.0 <= m.next_distance_m <= 280.0
  db.close()


def test_lookahead_reports_upcoming_higher_but_does_not_hide_current(tmp_path):
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1, "Main", "primary", 25 * CV.MPH_TO_MS,
    [(37.0, -122.004), (37.0, -122.000)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 2, "Main", "primary", 45 * CV.MPH_TO_MS,
    [(37.0, -122.000), (37.0, -121.996)],
  )
  con.commit()
  con.close()

  db = OsmSpeedLimitDB(path)
  assert db.open()
  m = db.lookup(37.0, -122.002, bearing_deg=90.0)
  assert m is not None
  assert abs(m.speed_limit_ms - 25 * CV.MPH_TO_MS) < 0.2
  assert abs(m.next_speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.2
  db.close()


def test_lookahead_does_not_skip_short_intermediate_limit(tmp_path):
  """Geodesic 40 m probes skip a ~15 m 50 between 60 and 30; along-way must not.

  US 12 near Benson: 60→50 (short)→30. Justin saw no 60→50 anticipatory; 50→30 worked.
  """
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  # ~178 m of 60, ~15 m of 50, then 30. Probes at 160 m (60) and 200 m (30).
  OsmSpeedLimitDB.insert_way(
    con, 1, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [(37.0, -122.004), (37.0, -122.000)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 2, "US 12", "trunk", 50 * CV.MPH_TO_MS,
    [(37.0, -122.000), (37.0, -121.99983)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 3, "US 12", "trunk", 30 * CV.MPH_TO_MS,
    [(37.0, -121.99983), (37.0, -121.996)],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  m = db.lookup(37.0, -122.002, bearing_deg=90.0)
  assert m is not None
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert abs(m.next_speed_limit_ms - 50 * CV.MPH_TO_MS) < 0.3, m.next_speed_limit_ms * CV.MS_TO_MPH
  assert m.next_distance_m < 250.0
  db.close()


def test_benson_us12_60_to_50_is_next_not_30(tmp_path):
  """Way 1557241351 (60) then 1227205099 (50, ~760 m) westbound toward Benson MN."""
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1557241351, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [
      (45.3079280, -95.5767060),
      (45.3084190, -95.5784110),
      (45.3087680, -95.5796230),
      (45.3088579, -95.5799336),
    ],
  )
  OsmSpeedLimitDB.insert_way(
    con, 1227205099, "US 12", "trunk", 50 * CV.MPH_TO_MS,
    [
      (45.3088579, -95.5799336),
      (45.3093400, -95.5816000),
      (45.3103910, -95.5852260),
      (45.3114806, -95.5888992),
    ],
  )
  OsmSpeedLimitDB.insert_way(
    con, 99, "US 12", "trunk", 30 * CV.MPH_TO_MS,
    [(45.3114806, -95.5888992), (45.3122, -95.5930)],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  m = db.lookup(45.3079280, -95.5767060, bearing_deg=290.0)
  assert m is not None
  assert m.way_id == 1557241351
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert abs(m.next_speed_limit_ms - 50 * CV.MPH_TO_MS) < 0.3, m.next_speed_limit_ms * CV.MS_TO_MPH
  assert 180.0 <= m.next_distance_m <= 320.0
  db.close()


def test_osm_sign_lead_is_speed_times_1_5s_not_fixed_meters():
  """Justin: 1.5 s delay. 50 mph → ~33.5 m, 60 mph → ~40 m. Not a constant offset."""
  assert abs(OSM_SIGN_LEAD_S - 1.5) < 1e-9
  m50 = osm_sign_lead_m(50 * CV.MPH_TO_MS)
  m60 = osm_sign_lead_m(60 * CV.MPH_TO_MS)
  assert abs(m50 - 50 * CV.MPH_TO_MS * 1.5) < 1e-9
  assert abs(m60 - 60 * CV.MPH_TO_MS * 1.5) < 1e-9
  assert abs(m50 - 33.5) < 0.1
  assert abs(m60 - 40.2) < 0.1
  assert m60 > m50 + 5.0
  assert osm_sign_lead_m(0.0) == 0.0
  assert osm_sign_lead_m(-5.0) == 0.0


def test_sign_lead_advances_limit_and_next_distance(tmp_path):
  """GNSS lag offset: posted match and remaining-to-next are at v*1.5 s."""
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1, "Main", "primary", 45 * CV.MPH_TO_MS,
    [(37.0, -122.004), (37.0, -122.000)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 2, "Main", "primary", 25 * CV.MPH_TO_MS,
    [(37.0, -122.000), (37.0, -121.996)],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  v60 = 60 * CV.MPH_TO_MS
  lead_m = osm_sign_lead_m(v60)
  assert abs(lead_m - v60 * OSM_SIGN_LEAD_S) < 1e-9

  gps_lat, gps_lon = 37.0, -122.002
  cold = db.lookup(gps_lat, gps_lon, bearing_deg=90.0)
  hot = db.lookup(gps_lat, gps_lon, bearing_deg=90.0, v_ego_ms=v60)
  assert cold is not None and hot is not None
  assert abs(cold.speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.2
  assert abs(hot.speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.2
  assert abs(hot.next_speed_limit_ms - 25 * CV.MPH_TO_MS) < 0.2
  assert abs(cold.next_speed_limit_ms - 25 * CV.MPH_TO_MS) < 0.2
  # Remaining is from the lag-corrected point; +110 m decrease margin is separate.
  assert hot.next_distance_m < cold.next_distance_m
  assert abs((cold.next_distance_m - hot.next_distance_m) - lead_m) < 12.0

  # Close enough that 1.5 s at 60 mph is already on the 25: LIMIT updates (down).
  qlat, qlon = _offset_point(37.0, -122.000, 270.0, 20.0)
  at_sign = db.lookup(qlat, qlon, bearing_deg=90.0)
  early = db.lookup(qlat, qlon, bearing_deg=90.0, v_ego_ms=v60)
  assert at_sign is not None and abs(at_sign.speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.2
  assert early is not None and abs(early.speed_limit_ms - 25 * CV.MPH_TO_MS) < 0.2
  db.close()


def test_sign_lead_raises_posted_when_offset_already_in_higher_zone(tmp_path):
  """Same 1.5 s offset both ways: 20 m before 25→45, lag-corrected posted is 45."""
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1, "Main", "primary", 25 * CV.MPH_TO_MS,
    [(37.0, -122.004), (37.0, -122.000)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 2, "Main", "primary", 45 * CV.MPH_TO_MS,
    [(37.0, -122.000), (37.0, -121.996)],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  v60 = 60 * CV.MPH_TO_MS
  qlat, qlon = _offset_point(37.0, -122.000, 270.0, 20.0)
  at_gps = db.lookup(qlat, qlon, bearing_deg=90.0)
  lagged = db.lookup(qlat, qlon, bearing_deg=90.0, v_ego_ms=v60)
  assert at_gps is not None and abs(at_gps.speed_limit_ms - 25 * CV.MPH_TO_MS) < 0.2
  assert lagged is not None and abs(lagged.speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.2
  db.close()


def test_higher_limit_far_ahead_stays_next_not_posted(tmp_path):
  """~180 m of 25 then 45: 1.5 s offset (~40 m) is still on 25. Not lookahead raise."""
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1, "Main", "primary", 25 * CV.MPH_TO_MS,
    [(37.0, -122.004), (37.0, -122.000)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 2, "Main", "primary", 45 * CV.MPH_TO_MS,
    [(37.0, -122.000), (37.0, -121.996)],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  v60 = 60 * CV.MPH_TO_MS
  m = db.lookup(37.0, -122.002, bearing_deg=90.0, v_ego_ms=v60)
  assert m is not None
  assert abs(m.speed_limit_ms - 25 * CV.MPH_TO_MS) < 0.2
  assert abs(m.next_speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.2
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


def test_delete_ways_intersecting_bbox(tmp_path):
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1, "Near", "primary", 35 * CV.MPH_TO_MS,
    [(37.0, -122.001), (37.0, -121.999)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 2, "Far", "primary", 25 * CV.MPH_TO_MS,
    [(40.7, -74.001), (40.7, -73.999)],
  )
  n = OsmSpeedLimitDB.delete_ways_intersecting_bbox(con, 36.9, -122.1, 37.1, -121.9)
  assert n == 1
  OsmSpeedLimitDB.recount_ways(con)
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  assert db.lookup(37.0, -122.0, bearing_deg=90.0) is None
  far = db.lookup(40.7, -74.0, bearing_deg=90.0)
  assert far is not None and far.way_id == 2
  db.close()
  assert OsmSpeedLimitDB.way_count(path) == 1


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

