from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import HEADING_ALIGN_DEG, MIN_ZONE_LENGTH_M, OSM_SIGN_LEAD_S, osm_sign_lead_m
from openpilot.selfdrive.mapd.osm_db import (
  OsmSpeedLimitDB, _continues_route, _heading_aligned, _offset_point, _pack_coords,
  _unpack_coords, _wrap_heading_delta, simplify_coords,
)
from openpilot.selfdrive.mapd.overpass import ways_from_overpass
from openpilot.selfdrive.mapd.speed_limit import parse_maxspeed


def test_continues_route_rejects_cross_street_fills():
  assert _continues_route(90.0, 90.0, "US 12", "trunk", "US 12", "trunk")
  assert _continues_route(90.0, 80.0, "US 12", "trunk", "US 12", "primary")
  assert not _continues_route(90.0, 0.0, "US 12", "trunk", "Oak", "residential")
  assert not _continues_route(90.0, 90.0, "US 12", "trunk", "Oak", "residential")
  # Same name may change class in town.
  assert _continues_route(90.0, 90.0, "US 12", "trunk", "US 12", "residential")
  # Reverse-digitized along-route town ways (Δ ≈ 180°) must stay on-route.
  # Name change is normal (US 12 → Atlantic / Main Avenue); class continuity
  # + bidirectional heading is enough.
  assert _heading_aligned(110.0, 291.0)
  assert _heading_aligned(124.0, 304.0)
  assert not _heading_aligned(110.0, 20.0)
  assert _continues_route(110.0, 291.0, "US 12", "trunk", "Atlantic Avenue", "primary")
  assert _continues_route(124.0, 304.0, "US 12", "trunk", "Main Avenue", "primary")
  # Ep3 Kerkhoven: travel SE ~117.5°, Atlantic 30 digitized NW ~310°.
  assert _heading_aligned(117.5, 310.3)
  assert _wrap_heading_delta(117.5, 310.3) > HEADING_ALIGN_DEG  # old unidirectional gate
  assert _continues_route(117.5, 310.3, "", "trunk", "Atlantic Avenue", "trunk")
  # Ep4 Pennock: travel SE ~112°, Pacific Ave 45 digitized NW ~290°.
  assert _heading_aligned(111.8, 289.9)
  assert _wrap_heading_delta(111.8, 289.9) > HEADING_ALIGN_DEG
  assert _continues_route(111.8, 289.9, "", "trunk", "Pacific Avenue Southwest", "trunk")
  # Reverse-digitized residential fill is still a fill.
  assert not _continues_route(110.0, 290.0, "US 12", "trunk", "Oak", "residential")
  # ~90° cross street still fails heading (min(Δ, 180−Δ) = 90).
  assert not _heading_aligned(112.0, 22.0)
  assert not _continues_route(112.0, 22.0, "", "trunk", "County Road", "unclassified")


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


def test_min_zone_length_is_about_250_feet():
  assert abs(MIN_ZONE_LENGTH_M - 76.0) < 1e-9
  assert 75.0 <= MIN_ZONE_LENGTH_M <= 77.0


def test_lookahead_does_not_skip_short_intermediate_limit(tmp_path):
  """Along-way still publishes a real short-but-legal 60→50.

  The 50 must last longer than MIN_ZONE_LENGTH_M (~250 ft). US 12 east of
  Benson is ~760 m of tagged 50 — this fixture is a compact ~200 m 50.
  A stub under ~250 ft is ignored (see test_next_limit_ignores_stub_zone).
  """
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  # ~178 m of 60, ~200 m of 50, then 30.
  start50 = (37.0, -122.000)
  end50 = _offset_point(start50[0], start50[1], 90.0, 200.0)
  end30 = _offset_point(end50[0], end50[1], 90.0, 250.0)
  OsmSpeedLimitDB.insert_way(
    con, 1, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [(37.0, -122.004), start50],
  )
  OsmSpeedLimitDB.insert_way(
    con, 2, "US 12", "trunk", 50 * CV.MPH_TO_MS,
    [start50, end50],
  )
  OsmSpeedLimitDB.insert_way(
    con, 3, "US 12", "trunk", 30 * CV.MPH_TO_MS,
    [end50, end30],
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
  # 1.5 s offset into the short 50 must not snap posted to 50.
  v60 = 60 * CV.MPH_TO_MS
  near50 = db.lookup(45.3087680, -95.5796230, bearing_deg=290.0, v_ego_ms=v60)
  assert near50 is not None
  assert abs(near50.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert abs(near50.next_speed_limit_ms - 50 * CV.MPH_TO_MS) < 0.3
  db.close()


def test_next_limit_ignores_cross_street_when_highway_curves(tmp_path):
  """Geodesic heading ray leaves a curved 60 and must not pick a 30 fill."""
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [(37.0, -122.004), (37.0, -122.000), (37.0005, -121.996)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 2, "Oak", "residential", 30 * CV.MPH_TO_MS,
    [(36.999, -121.998), (37.001, -121.998)],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  m = db.lookup(37.0, -122.002, bearing_deg=90.0)
  assert m is not None
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert m.next_speed_limit_ms == 0.0
  db.close()


def test_next_limit_ignores_cross_streets_with_gps_heading_error(tmp_path):
  """Long tagged 60: remaining > 600 m, so along-way used to return None and
  geodesic picked a 30 fill as soon as GPS heading was a few degrees off."""
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [(37.0, -122.010), (37.0, -121.990)],
  )
  for i, lon in enumerate((-122.005, -122.003, -122.001, -121.999)):
    OsmSpeedLimitDB.insert_way(
      con, 10 + i, "Cross", "residential", 30 * CV.MPH_TO_MS,
      [(36.997, lon), (37.003, lon)],
    )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  for hdg in (90.0, 85.0, 70.0):
    m = db.lookup(37.0, -122.006, bearing_deg=hdg)
    assert m is not None, hdg
    assert m.way_id == 1, hdg
    assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3, hdg
    assert m.next_speed_limit_ms == 0.0, (hdg, m.next_speed_limit_ms * CV.MS_TO_MPH)
  db.close()


def test_on_route_60_to_50_wins_over_cross_street_fill(tmp_path):
  """US 12 60→50 at an intersection with a 30 mph residential fill."""
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [(37.0, -122.004), (37.0, -122.000)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 2, "US 12", "trunk", 50 * CV.MPH_TO_MS,
    [(37.0, -122.000), (37.0, -121.996)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 3, "Oak", "residential", 30 * CV.MPH_TO_MS,
    [(36.998, -122.000), (37.002, -122.000)],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  m = db.lookup(37.0, -122.002, bearing_deg=90.0)
  assert m is not None
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert abs(m.next_speed_limit_ms - 50 * CV.MPH_TO_MS) < 0.3, m.next_speed_limit_ms * CV.MS_TO_MPH
  db.close()


def test_geodesic_gap_finds_on_route_50_not_cross_street(tmp_path):
  """OSM gap after the 60 way: heading ray may fill in, but not a 30 cross street."""
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [(37.0, -122.004), (37.0, -122.000)],
  )
  # ~80 m gap (along-way 12/25/40 m steps miss; geodesic 40 m probes hit).
  OsmSpeedLimitDB.insert_way(
    con, 2, "US 12", "trunk", 50 * CV.MPH_TO_MS,
    [(37.0, -121.9991), (37.0, -121.996)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 3, "Oak", "residential", 30 * CV.MPH_TO_MS,
    [(36.998, -121.99955), (37.002, -121.99955)],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  m = db.lookup(37.0, -122.002, bearing_deg=90.0)
  assert m is not None
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert abs(m.next_speed_limit_ms - 50 * CV.MPH_TO_MS) < 0.3, m.next_speed_limit_ms * CV.MS_TO_MPH
  db.close()


def test_benson_us12_long_60_ignores_side_street_30_fill(tmp_path):
  """Westbound US 12 60 near Benson: a statutory 30 fill beside the highway is
  not nextSpeedLimit. MAX must not walk down toward that 30."""
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1557241351, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [
      (45.3079280, -95.5767060),
      (45.3084190, -95.5784110),
      (45.3087680, -95.5796230),
      (45.3088579, -95.5799336),
      (45.3100, -95.5840),
      (45.3120, -95.5900),
    ],
  )
  # North-south residential ~200 m along heading 290 from the start.
  OsmSpeedLimitDB.insert_way(
    con, 200, "Minnesota Ave", "residential", 30 * CV.MPH_TO_MS,
    [(45.3075, -95.5784110), (45.3095, -95.5784110)],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  m = db.lookup(45.3079280, -95.5767060, bearing_deg=290.0)
  assert m is not None
  assert m.way_id == 1557241351
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert m.next_speed_limit_ms == 0.0
  db.close()


def test_next_limit_ignores_stub_zone_shorter_than_min_length(tmp_path):
  """~10 m of 30 between two 60s is bleed, not a posted drop.

  Same name/class so #53 route continuity would accept it; min-zone must reject.
  """
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [(37.0, -122.004), (37.0, -122.000)],
  )
  # ~10 m stub (well under MIN_ZONE_LENGTH_M / ~250 ft).
  OsmSpeedLimitDB.insert_way(
    con, 2, "US 12", "trunk", 30 * CV.MPH_TO_MS,
    [(37.0, -122.000), (37.0, -121.99989)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 3, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [(37.0, -121.99989), (37.0, -121.996)],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  m = db.lookup(37.0, -122.002, bearing_deg=90.0)
  assert m is not None
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert m.next_speed_limit_ms == 0.0, m.next_speed_limit_ms * CV.MS_TO_MPH
  db.close()


def test_next_limit_ignores_stub_under_250_ft(tmp_path):
  """~60 m of 30 (~197 ft) is still under MIN_ZONE_LENGTH_M (~250 ft).

  #82/#83's 15 m / ~50 ft gate still flashed these. Same name/class so
  #53 would accept it; the raised min-zone must reject.
  """
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  start30 = (37.0, -122.000)
  end30 = _offset_point(start30[0], start30[1], 90.0, 60.0)
  end60 = _offset_point(end30[0], end30[1], 90.0, 250.0)
  OsmSpeedLimitDB.insert_way(
    con, 1, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [(37.0, -122.004), start30],
  )
  OsmSpeedLimitDB.insert_way(
    con, 2, "US 12", "trunk", 30 * CV.MPH_TO_MS,
    [start30, end30],
  )
  OsmSpeedLimitDB.insert_way(
    con, 3, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [end30, end60],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  m = db.lookup(37.0, -122.002, bearing_deg=90.0)
  assert m is not None
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert m.next_speed_limit_ms == 0.0, m.next_speed_limit_ms * CV.MS_TO_MPH
  db.close()


def test_next_limit_keeps_real_drop_longer_than_min_length(tmp_path):
  """A 30 that continues ~200 m after a 60 is a real on-route drop."""
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [(37.0, -122.004), (37.0, -122.000)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 2, "US 12", "trunk", 30 * CV.MPH_TO_MS,
    [(37.0, -122.000), (37.0, -121.997)],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  m = db.lookup(37.0, -122.002, bearing_deg=90.0)
  assert m is not None
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert abs(m.next_speed_limit_ms - 30 * CV.MPH_TO_MS) < 0.3, m.next_speed_limit_ms * CV.MS_TO_MPH
  db.close()


def _reverse(coords: list[tuple[float, float]]) -> list[tuple[float, float]]:
  return list(reversed(coords))


def test_town_entry_short_first_way_long_contiguous_run(tmp_path):
  """Ep1 Benson / Atlantic: first 30 way is ~76 m (the old first-way gate),
  but the contiguous same-limit chain is hundreds of meters.

  Ways are reverse-digitized (Δheading ≈ 180°) and rename US 12 → Atlantic.
  Early look-ahead must publish next=30 before GPS is on the town way.
  """
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  travel = 90.0
  us12_start = (37.0, -122.012)
  junc = _offset_point(us12_start[0], us12_start[1], travel, 550.0)
  a1_end = _offset_point(junc[0], junc[1], travel, 76.1)
  a2_end = _offset_point(a1_end[0], a1_end[1], travel, 200.0)
  a3_end = _offset_point(a2_end[0], a2_end[1], travel, 200.0)
  OsmSpeedLimitDB.insert_way(
    con, 18267060, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [us12_start, junc],
  )
  OsmSpeedLimitDB.insert_way(
    con, 1227205098, "Atlantic Avenue", "primary", 30 * CV.MPH_TO_MS,
    _reverse([junc, a1_end]),
  )
  OsmSpeedLimitDB.insert_way(
    con, 18267487, "Atlantic Avenue", "primary", 30 * CV.MPH_TO_MS,
    _reverse([a1_end, a2_end]),
  )
  OsmSpeedLimitDB.insert_way(
    con, 1227205097, "Atlantic Avenue", "primary", 30 * CV.MPH_TO_MS,
    _reverse([a2_end, a3_end]),
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  # ~400 m before the junction — inside Early 600 m, still posted 60.
  qlat, qlon = _offset_point(junc[0], junc[1], travel + 180.0, 400.0)
  m = db.lookup(qlat, qlon, bearing_deg=travel)
  assert m is not None
  assert m.way_id == 18267060
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert abs(m.next_speed_limit_ms - 30 * CV.MPH_TO_MS) < 0.3, m.next_speed_limit_ms * CV.MS_TO_MPH
  assert 300.0 <= m.next_distance_m <= 500.0
  db.close()


def test_town_entry_reverse_digitized_long_way(tmp_path):
  """Ep2 Main Ave: the town 30 is long (~400 m here; on-car 1149 m) but
  reverse-digitized. Short 60 stubs sit in front. next must still publish 30.
  """
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  travel = 90.0
  us12_start = (37.0, -122.012)
  junc = _offset_point(us12_start[0], us12_start[1], travel, 550.0)
  main60_end = _offset_point(junc[0], junc[1], travel, 250.0)
  stub60_end = _offset_point(main60_end[0], main60_end[1], travel, 23.0)
  town30_end = _offset_point(stub60_end[0], stub60_end[1], travel, 400.0)
  OsmSpeedLimitDB.insert_way(
    con, 18267048, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [us12_start, junc],
  )
  OsmSpeedLimitDB.insert_way(
    con, 1227205096, "Main Avenue", "primary", 60 * CV.MPH_TO_MS,
    _reverse([junc, main60_end]),
  )
  OsmSpeedLimitDB.insert_way(
    con, 1557241354, "Main Avenue", "primary", 60 * CV.MPH_TO_MS,
    _reverse([main60_end, stub60_end]),
  )
  OsmSpeedLimitDB.insert_way(
    con, 18267827, "Main Avenue", "primary", 30 * CV.MPH_TO_MS,
    _reverse([stub60_end, town30_end]),
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  # On the highway ~150 m before the name change: 150+250+23 ≈ 423 m to the 30.
  qlat, qlon = _offset_point(junc[0], junc[1], travel + 180.0, 150.0)
  m = db.lookup(qlat, qlon, bearing_deg=travel)
  assert m is not None
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert abs(m.next_speed_limit_ms - 30 * CV.MPH_TO_MS) < 0.3, m.next_speed_limit_ms * CV.MS_TO_MPH
  assert m.next_distance_m > 200.0
  # On the Main Ave 60 lead-in (the on-car name change): still posted 60, next=30.
  on_main = _offset_point(junc[0], junc[1], travel, 100.0)
  m2 = db.lookup(on_main[0], on_main[1], bearing_deg=travel)
  assert m2 is not None
  assert abs(m2.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert abs(m2.next_speed_limit_ms - 30 * CV.MPH_TO_MS) < 0.3, m2.next_speed_limit_ms * CV.MS_TO_MPH
  db.close()


def test_town_entry_kerkhoven_long_reverse_digitized(tmp_path):
  """Ep3 ~19:24 Kerkhoven: even longer town than Murdock, still reverse-digitized.

  Same drive as REPORT.md ep1/ep2 (`1c95345a3286a5db|000000df--467073c363`).
  US 12 60 way 18267051 then Atlantic Avenue 30: 47 m + 68 m stubs and an
  876 m town way (chain ~1.1 km). Digitized NW (~300°) while travel is SE
  (~118°). Short-zone-alone cannot explain a dead next — the long way is
  far past 76 m. The old unidirectional heading gate can.

  Real corridor coords (OSM), not a corridor hardcode in matcher code.
  """
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  # Approach 60, first ~1.4 km from the town end (digitized NW).
  OsmSpeedLimitDB.insert_way(
    con, 18267051, "", "trunk", 60 * CV.MPH_TO_MS,
    [
      (45.1934513, -95.3224471),
      (45.1935615, -95.3227475),
      (45.1936458, -95.3230027),
      (45.1937388, -95.3232855),
      (45.1938181, -95.3235502),
      (45.1938968, -95.3238425),
      (45.1941100, -95.3246503),
      (45.1942096, -95.3249748),
      (45.1943107, -95.3252671),
      (45.1944248, -95.3255547),
      (45.1955660, -95.3283440),
      (45.1957940, -95.3289060),
      (45.1997270, -95.3385890),
    ],
  )
  OsmSpeedLimitDB.insert_way(
    con, 1312882771, "Atlantic Avenue", "trunk", 30 * CV.MPH_TO_MS,
    [(45.1932411, -95.3219240), (45.1934513, -95.3224471)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 1312882772, "Atlantic Avenue", "trunk", 30 * CV.MPH_TO_MS,
    [(45.1929386, -95.3211712), (45.1930521, -95.3214537), (45.1932411, -95.3219240)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 18267484, "Atlantic Avenue", "trunk", 30 * CV.MPH_TO_MS,
    [
      (45.1889261, -95.3115623),
      (45.1891320, -95.3119070),
      (45.1892753, -95.3122187),
      (45.1893454, -95.3123712),
      (45.1893958, -95.3124952),
      (45.1894422, -95.3126084),
      (45.1898958, -95.3137152),
      (45.1904075, -95.3149739),
      (45.1904499, -95.3150765),
      (45.1909150, -95.3162020),
      (45.1914230, -95.3174460),
      (45.1919300, -95.3186970),
      (45.1924364, -95.3199345),
      (45.1929386, -95.3211712),
    ],
  )
  OsmSpeedLimitDB.insert_way(
    con, 1227205094, "Atlantic Avenue", "trunk", 30 * CV.MPH_TO_MS,
    [(45.1881551, -95.3101903), (45.1883478, -95.3105791), (45.1889261, -95.3115623)],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  travel = 117.5
  # ~400 m before the 30, still on US 12 60 (inside Early 600 m).
  qlat, qlon = _offset_point(45.1934513, -95.3224471, 297.5, 400.0)
  m = db.lookup(qlat, qlon, bearing_deg=travel)
  assert m is not None
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert abs(m.next_speed_limit_ms - 30 * CV.MPH_TO_MS) < 0.3, m.next_speed_limit_ms * CV.MS_TO_MPH
  assert 250.0 <= m.next_distance_m <= 550.0
  # Closer in: still posted 60, next still the long town 30.
  q2lat, q2lon = _offset_point(45.1934513, -95.3224471, 297.5, 150.0)
  m2 = db.lookup(q2lat, q2lon, bearing_deg=travel)
  assert m2 is not None
  assert abs(m2.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert abs(m2.next_speed_limit_ms - 30 * CV.MPH_TO_MS) < 0.3, m2.next_speed_limit_ms * CV.MS_TO_MPH
  db.close()


def test_town_entry_pennock_60_to_45_reverse_digitized(tmp_path):
  """Ep4 ~19:32 Pennock: shorter town, 60→45, still reverse-digitized.

  Same drive. US 12 60 way 18120141 then Pacific Avenue SW/SE 45
  (719 m + 1010 m). Digitized NW (~290°) while travel is SE (~112°).
  Do not lower MIN_ZONE — this 45 run is far past 76 m. next must publish 45
  so Early can bleed before the sign.
  """
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 18120141, "", "trunk", 60 * CV.MPH_TO_MS,
    [
      (45.1497750, -95.1859457),
      (45.1499532, -95.1865781),
      (45.1501123, -95.1871137),
      (45.1506249, -95.1888123),
      (45.1508476, -95.1895753),
      (45.1510528, -95.1903475),
      (45.1550563, -95.2060945),
    ],
  )
  OsmSpeedLimitDB.insert_way(
    con, 18120656, "Pacific Avenue Southwest", "trunk", 45 * CV.MPH_TO_MS,
    [
      (45.1475731, -95.1773305),
      (45.1479335, -95.1787404),
      (45.1482841, -95.1801123),
      (45.1497750, -95.1859457),
    ],
  )
  OsmSpeedLimitDB.insert_way(
    con, 18120434, "Pacific Avenue Southeast", "trunk", 45 * CV.MPH_TO_MS,
    [
      (45.1445357, -95.1651890),
      (45.1459258, -95.1708021),
      (45.1465717, -95.1733621),
      (45.1468998, -95.1746620),
      (45.1472401, -95.1760108),
      (45.1475731, -95.1773305),
    ],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  travel = 111.8
  qlat, qlon = _offset_point(45.1497750, -95.1859457, 291.8, 400.0)
  m = db.lookup(qlat, qlon, bearing_deg=travel)
  assert m is not None
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert abs(m.next_speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.3, m.next_speed_limit_ms * CV.MS_TO_MPH
  assert 250.0 <= m.next_distance_m <= 550.0
  q2lat, q2lon = _offset_point(45.1497750, -95.1859457, 291.8, 150.0)
  m2 = db.lookup(q2lat, q2lon, bearing_deg=travel)
  assert m2 is not None
  assert abs(m2.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert abs(m2.next_speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.3, m2.next_speed_limit_ms * CV.MS_TO_MPH
  db.close()


def test_angled_cross_street_45_blip_still_ignored(tmp_path):
  """Do not reintroduce aggressive look-ahead for angled side-road bleed.

  SE highway vs a ~40° 45 mph fill that only lasts ~60 m along heading
  (returns to 60). MIN_ZONE stays ~250 ft — this blip must not arm next.
  """
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  travel = 112.0
  start = (37.0, -122.010)
  mid = _offset_point(start[0], start[1], travel, 400.0)
  after = _offset_point(mid[0], mid[1], travel, 12.0)
  end = _offset_point(start[0], start[1], travel, 900.0)
  OsmSpeedLimitDB.insert_way(
    con, 1, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [start, mid],
  )
  OsmSpeedLimitDB.insert_way(
    con, 3, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [after, end],
  )
  # ~40° off travel: long enough to be geometrically tempting, short along
  # the highway heading so persist / returns-to-prior must reject.
  fill_a = _offset_point(mid[0], mid[1], travel - 140.0, 80.0)
  fill_b = _offset_point(mid[0], mid[1], travel + 40.0, 80.0)
  OsmSpeedLimitDB.insert_way(
    con, 2, "County Road", "unclassified", 45 * CV.MPH_TO_MS,
    [fill_a, mid, fill_b],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  qlat, qlon = _offset_point(mid[0], mid[1], travel + 180.0, 200.0)
  m = db.lookup(qlat, qlon, bearing_deg=travel)
  assert m is not None
  assert m.way_id == 1
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert m.next_speed_limit_ms == 0.0, m.next_speed_limit_ms * CV.MS_TO_MPH
  # GPS snapped onto the fill, still heading along US 12.
  on_fill = _offset_point(mid[0], mid[1], travel + 40.0, 6.0)
  m2 = db.lookup(on_fill[0], on_fill[1], bearing_deg=travel)
  assert m2 is not None
  assert abs(m2.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert m2.next_speed_limit_ms == 0.0, m2.next_speed_limit_ms * CV.MS_TO_MPH
  db.close()


def test_town_entry_short_first_piece_under_min_zone_still_publishes(tmp_path):
  """First town 30 piece is well under ~250 ft; the same-limit run is not."""
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  travel = 90.0
  us12_start = (37.0, -122.010)
  junc = _offset_point(us12_start[0], us12_start[1], travel, 400.0)
  first_end = _offset_point(junc[0], junc[1], travel, 40.0)
  rest_end = _offset_point(first_end[0], first_end[1], travel, 250.0)
  OsmSpeedLimitDB.insert_way(
    con, 1, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [us12_start, junc],
  )
  OsmSpeedLimitDB.insert_way(
    con, 2, "Atlantic Avenue", "primary", 30 * CV.MPH_TO_MS,
    _reverse([junc, first_end]),
  )
  OsmSpeedLimitDB.insert_way(
    con, 3, "Atlantic Avenue", "primary", 30 * CV.MPH_TO_MS,
    _reverse([first_end, rest_end]),
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  qlat, qlon = _offset_point(junc[0], junc[1], travel + 180.0, 200.0)
  m = db.lookup(qlat, qlon, bearing_deg=travel)
  assert m is not None
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert abs(m.next_speed_limit_ms - 30 * CV.MPH_TO_MS) < 0.3, m.next_speed_limit_ms * CV.MS_TO_MPH
  db.close()


def test_side_road_bleed_blip_still_ignored_with_reverse_town_ways(tmp_path):
  """Standing rule: N–S residential / unclassified fill must not arm next.

  Same highway as the town-entry fixtures (eastbound 60) plus a long N–S 30
  fill. Reverse-digitized heading tolerance must not open this hole.
  """
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  travel = 90.0
  start = (37.0, -122.010)
  mid = _offset_point(start[0], start[1], travel, 400.0)
  end = _offset_point(start[0], start[1], travel, 900.0)
  OsmSpeedLimitDB.insert_way(
    con, 1, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [start, mid, end],
  )
  cross_n = _offset_point(mid[0], mid[1], 0.0, 200.0)
  cross_s = _offset_point(mid[0], mid[1], 180.0, 200.0)
  OsmSpeedLimitDB.insert_way(
    con, 2, "Oak", "residential", 30 * CV.MPH_TO_MS,
    [cross_s, mid, cross_n],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  qlat, qlon = _offset_point(mid[0], mid[1], travel + 180.0, 200.0)
  m = db.lookup(qlat, qlon, bearing_deg=travel)
  assert m is not None
  assert m.way_id == 1
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert m.next_speed_limit_ms == 0.0, m.next_speed_limit_ms * CV.MS_TO_MPH
  db.close()


def test_current_match_ignores_cross_street_bleed(tmp_path):
  """GPS on a side street at the intersection must not snap posted to 30."""
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1, "Main", "primary", 60 * CV.MPH_TO_MS,
    [(37.0, -122.004), (37.0, -121.996)],
  )
  OsmSpeedLimitDB.insert_way(
    con, 2, "Oak", "residential", 30 * CV.MPH_TO_MS,
    [(36.997, -122.000), (37.003, -122.000)],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  # ~5 m north of the highway, on Oak, heading east along Main.
  qlat, qlon = _offset_point(37.0, -122.000, 0.0, 5.0)
  m = db.lookup(qlat, qlon, bearing_deg=90.0)
  assert m is not None
  assert m.way_id == 1
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert m.next_speed_limit_ms == 0.0
  db.close()


def test_se_heading_ew_fill_does_not_drop_posted_or_next(tmp_path):
  """SE highway vs E-W grid: Δheading ~30° is inside HEADING_ALIGN_DEG.

  #53's bearing gate still lets that fill score as a current/next candidate.
  The min-zone gate must keep posted on the highway (US 12 SE Benson→DeGraff
  pattern; geometry is generic, not a corridor hardcode).
  """
  path = str(tmp_path / "speed_limits.sqlite")
  con = OsmSpeedLimitDB.create(path)
  start = (37.0, -122.004)
  mid = _offset_point(start[0], start[1], 120.0, 400.0)
  end = _offset_point(start[0], start[1], 120.0, 800.0)
  OsmSpeedLimitDB.insert_way(
    con, 1, "US 12", "trunk", 60 * CV.MPH_TO_MS,
    [start, mid, end],
  )
  cross_w = _offset_point(mid[0], mid[1], 270.0, 200.0)
  cross_e = _offset_point(mid[0], mid[1], 90.0, 200.0)
  OsmSpeedLimitDB.insert_way(
    con, 2, "Township", "unclassified", 30 * CV.MPH_TO_MS,
    [cross_w, mid, cross_e],
  )
  con.commit()
  con.close()
  db = OsmSpeedLimitDB(path)
  assert db.open()
  # On the fill, a few meters west of the crossing, still heading SE on US 12.
  qlat, qlon = _offset_point(mid[0], mid[1], 270.0, 6.0)
  m = db.lookup(qlat, qlon, bearing_deg=120.0)
  assert m is not None
  assert m.way_id == 1, (m.way_id, m.road_name, m.speed_limit_ms * CV.MS_TO_MPH)
  assert abs(m.speed_limit_ms - 60 * CV.MPH_TO_MS) < 0.3
  assert m.next_speed_limit_ms == 0.0, m.next_speed_limit_ms * CV.MS_TO_MPH
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
  # Remaining is from GPS minus v*1.5 s; +110 m decrease margin is separate.
  assert hot.next_distance_m < cold.next_distance_m
  assert abs((cold.next_distance_m - hot.next_distance_m) - lead_m) < 12.0

  # Offset already in the 25: do not snap posted to 25. Keep 45 and next=25
  # so kin+110 m can ease MAX (the 60→50 snap).
  qlat, qlon = _offset_point(37.0, -122.000, 270.0, 20.0)
  at_sign = db.lookup(qlat, qlon, bearing_deg=90.0)
  early = db.lookup(qlat, qlon, bearing_deg=90.0, v_ego_ms=v60)
  assert at_sign is not None and abs(at_sign.speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.2
  assert early is not None and abs(early.speed_limit_ms - 45 * CV.MPH_TO_MS) < 0.2
  assert abs(early.next_speed_limit_ms - 25 * CV.MPH_TO_MS) < 0.2
  assert early.next_distance_m < at_sign.next_distance_m
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
    OsmSpeedLimitDB.insert_way(
      con, w["way_id"], w["name"], w["highway"], w["maxspeed_ms"], w["coords"],
      junction=w.get("junction") or "",
    )
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

