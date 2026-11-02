"""Minnesota statutory maxspeed fills (NAP pack only, never uploaded to OSM)."""
from __future__ import annotations

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.mn_statutory import (
  ALLEY_MPH,
  BENSON_LAT,
  BENSON_LON,
  EXPRESSWAY_MPH,
  FILL_SOURCE,
  MOTORWAY_RURAL_MPH,
  RURAL_OTHER_MPH,
  URBAN_DISTRICT_MPH,
  PlaceIndex,
  bbox_intersects_minnesota,
  benson_fill_bbox,
  in_minnesota,
  is_urban_place_tags,
  is_urban_way,
  places_from_overpass,
  statutory_maxspeed_mph,
)
from openpilot.selfdrive.mapd.overpass import (
  bbox_from_center,
  overpass_query,
  resolve_way_speed,
  ways_from_overpass,
)


def _ms(mph: float) -> float:
  return mph * CV.MPH_TO_MS


def test_statutory_defaults_by_class():
  assert statutory_maxspeed_mph("alley", {}, urban=False) == ALLEY_MPH
  assert statutory_maxspeed_mph("living_street", {}, urban=True) == ALLEY_MPH
  assert statutory_maxspeed_mph("residential", {}, urban=False) == URBAN_DISTRICT_MPH
  assert statutory_maxspeed_mph("unclassified", {}, urban=True) == URBAN_DISTRICT_MPH
  assert statutory_maxspeed_mph("unclassified", {}, urban=False) == RURAL_OTHER_MPH
  assert statutory_maxspeed_mph("tertiary", {}, urban=False) == RURAL_OTHER_MPH
  assert statutory_maxspeed_mph("secondary", {}, urban=True) == URBAN_DISTRICT_MPH
  assert statutory_maxspeed_mph("primary", {}, urban=False) == RURAL_OTHER_MPH
  assert statutory_maxspeed_mph("trunk", {}, urban=False) == RURAL_OTHER_MPH
  assert statutory_maxspeed_mph("motorway", {}, urban=False) == MOTORWAY_RURAL_MPH
  assert statutory_maxspeed_mph("motorway_link", {}, urban=True) == MOTORWAY_RURAL_MPH
  assert statutory_maxspeed_mph(
    "primary", {"expressway": "yes"}, urban=True,
  ) == EXPRESSWAY_MPH
  assert statutory_maxspeed_mph("trunk", {"motorroad": "yes"}, urban=False) == EXPRESSWAY_MPH


def test_skip_service_track_path_construction():
  for hw in ("service", "track", "path", "footway", "cycleway", "pedestrian",
             "construction", "proposed", "abandoned"):
    assert statutory_maxspeed_mph(hw, {}, urban=True) is None


def test_tagged_maxspeed_is_authoritative():
  tags = {"highway": "residential", "maxspeed": "25 mph"}
  coords = [(BENSON_LAT, BENSON_LON), (BENSON_LAT, BENSON_LON + 0.001)]
  ms, source = resolve_way_speed(tags, coords, fill_unmarked=True)
  assert source == "osm"
  assert ms is not None and abs(ms - _ms(25)) < 1e-6
  # Non-numeric OSM tag still fills.
  tags2 = {"highway": "residential", "maxspeed": "signals"}
  ms2, source2 = resolve_way_speed(tags2, coords, fill_unmarked=True)
  assert source2 == FILL_SOURCE
  assert ms2 is not None and abs(ms2 - _ms(30)) < 1e-6


def test_no_fill_without_flag():
  tags = {"highway": "residential", "name": "Oak"}
  coords = [(BENSON_LAT, BENSON_LON), (BENSON_LAT, BENSON_LON + 0.001)]
  ms, source = resolve_way_speed(tags, coords, fill_unmarked=False)
  assert ms is None and source == ""


def test_mn_city_admin_boundary_is_urban_place():
  assert is_urban_place_tags({
    "name": "Benson", "boundary": "administrative", "admin_level": "8", "border_type": "city",
  })
  assert is_urban_place_tags({"place": "town", "name": "Kerkhoven"})
  assert not is_urban_place_tags({
    "name": "Benson Township", "boundary": "administrative", "admin_level": "8", "border_type": "township",
  })
  assert is_urban_place_tags({
    "name": "Saint Cloud", "boundary": "administrative", "admin_level": "8",
  })


def test_place_polygon_marks_named_primary_urban():
  places = PlaceIndex()
  # Closed square around Benson ~0.02 deg (~2 km).
  ring = [
    (BENSON_LAT - 0.02, BENSON_LON - 0.02),
    (BENSON_LAT - 0.02, BENSON_LON + 0.02),
    (BENSON_LAT + 0.02, BENSON_LON + 0.02),
    (BENSON_LAT + 0.02, BENSON_LON - 0.02),
    (BENSON_LAT - 0.02, BENSON_LON - 0.02),
  ]
  places.add_rings([ring], "Benson")
  in_town = [(BENSON_LAT, BENSON_LON - 0.001), (BENSON_LAT, BENSON_LON + 0.001)]
  rural = [(BENSON_LAT + 0.5, BENSON_LON), (BENSON_LAT + 0.5, BENSON_LON + 0.001)]
  tags = {"highway": "primary", "name": "US 12"}
  assert is_urban_way(tags, in_town, places)
  assert not is_urban_way(tags, rural, places)
  urban_ms, src = resolve_way_speed(tags, in_town, fill_unmarked=True, places=places)
  rural_ms, src_r = resolve_way_speed(tags, rural, fill_unmarked=True, places=places)
  assert src == FILL_SOURCE and src_r == FILL_SOURCE
  assert urban_ms is not None and abs(urban_ms - _ms(30)) < 1e-6
  assert rural_ms is not None and abs(rural_ms - _ms(55)) < 1e-6


def test_sidewalk_is_urban():
  tags = {"highway": "tertiary", "sidewalk": "both"}
  coords = [(BENSON_LAT + 0.4, BENSON_LON), (BENSON_LAT + 0.4, BENSON_LON + 0.001)]
  assert is_urban_way(tags, coords, PlaceIndex())
  mph = statutory_maxspeed_mph("tertiary", tags, urban=True)
  assert mph == URBAN_DISTRICT_MPH


def test_places_from_overpass_relation_and_way():
  payload = {
    "elements": [
      {
        "type": "way",
        "id": 1,
        "tags": {"place": "town", "name": "Benson"},
        "geometry": [
          {"lat": 45.30, "lon": -95.62},
          {"lat": 45.30, "lon": -95.58},
          {"lat": 45.33, "lon": -95.58},
          {"lat": 45.33, "lon": -95.62},
        ],
      },
      {
        "type": "relation",
        "id": 3,
        "tags": {"name": "Benson", "boundary": "administrative", "admin_level": "8", "border_type": "city"},
        "members": [
          {
            "type": "way",
            "role": "outer",
            "geometry": [
              {"lat": 45.30, "lon": -95.62},
              {"lat": 45.30, "lon": -95.58},
              {"lat": 45.33, "lon": -95.58},
              {"lat": 45.33, "lon": -95.62},
            ],
          }
        ],
      },
      {
        "type": "relation",
        "id": 4,
        "tags": {"name": "Benson Township", "boundary": "administrative", "admin_level": "8", "border_type": "township"},
        "members": [
          {
            "type": "way",
            "role": "outer",
            "geometry": [
              {"lat": 45.0, "lon": -96.0},
              {"lat": 45.0, "lon": -95.9},
              {"lat": 45.1, "lon": -95.9},
              {"lat": 45.1, "lon": -96.0},
            ],
          }
        ],
      },
    ]
  }
  idx = places_from_overpass(payload)
  assert len(idx) == 2
  assert idx.contains(45.315, -95.60)
  assert not idx.contains(45.05, -95.95)
  assert not idx.contains(46.0, -95.6)


def test_ways_from_overpass_fills_and_skips():
  payload = {
    "elements": [
      {
        "type": "way",
        "id": 10,
        "tags": {"highway": "residential", "name": "Oak"},
        "geometry": [
          {"lat": BENSON_LAT, "lon": BENSON_LON - 0.001},
          {"lat": BENSON_LAT, "lon": BENSON_LON + 0.001},
        ],
      },
      {
        "type": "way",
        "id": 11,
        "tags": {"highway": "service"},
        "geometry": [
          {"lat": BENSON_LAT, "lon": BENSON_LON - 0.001},
          {"lat": BENSON_LAT, "lon": BENSON_LON + 0.001},
        ],
      },
      {
        "type": "way",
        "id": 12,
        "tags": {"highway": "primary", "maxspeed": "60 mph", "name": "US 12"},
        "geometry": [
          {"lat": BENSON_LAT + 0.01, "lon": BENSON_LON - 0.001},
          {"lat": BENSON_LAT + 0.01, "lon": BENSON_LON + 0.001},
        ],
      },
      {
        "type": "way",
        "id": 13,
        "tags": {"place": "city", "name": "Benson"},
        "geometry": [
          {"lat": BENSON_LAT - 0.05, "lon": BENSON_LON - 0.05},
          {"lat": BENSON_LAT - 0.05, "lon": BENSON_LON + 0.05},
          {"lat": BENSON_LAT + 0.05, "lon": BENSON_LON + 0.05},
          {"lat": BENSON_LAT + 0.05, "lon": BENSON_LON - 0.05},
        ],
      },
    ]
  }
  skipped = ways_from_overpass(payload)
  assert [w["way_id"] for w in skipped] == [12]

  filled = ways_from_overpass(payload, fill_unmarked=True)
  ids = {w["way_id"]: w for w in filled}
  assert 10 in ids and 12 in ids
  assert 11 not in ids
  assert 13 not in ids  # place polygon is not a highway
  assert ids[10]["source"] == FILL_SOURCE
  assert abs(ids[10]["maxspeed_ms"] - _ms(30)) < 0.2
  assert ids[12]["source"] == "osm"
  assert abs(ids[12]["maxspeed_ms"] - _ms(60)) < 0.2


def test_overpass_query_unmarked_fetches_all_fillable_highways():
  tagged = overpass_query(44.0, -96.0, 46.0, -94.0, include_unmarked=False)
  assert 'way["highway"]["maxspeed"]' in tagged
  unmarked = overpass_query(44.0, -96.0, 46.0, -94.0, include_unmarked=True)
  assert '["maxspeed"]' not in unmarked
  assert "motorway" in unmarked
  assert "admin_level" in unmarked
  assert "out geom" in unmarked


def test_benson_bbox_is_clipped_to_minnesota():
  assert in_minnesota(BENSON_LAT, BENSON_LON)
  south, west, north, east = benson_fill_bbox()
  assert west >= -97.239 - 1e-9
  assert in_minnesota((south + north) / 2, (west + east) / 2)
  box = bbox_from_center(BENSON_LAT, BENSON_LON, 160.934)
  assert bbox_intersects_minnesota(box)
  sf = bbox_from_center(37.7749, -122.4194, 160.934)
  assert not bbox_intersects_minnesota(sf)
