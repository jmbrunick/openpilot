from types import SimpleNamespace

from openpilot.selfdrive.mapd.speed_overrides import MPH_TO_MS, PENDING_MAX_S, SpeedOverrides, parse_confirmed



def _read_text(p):
  with open(p) as f:
    return f.read()


def _read_json(p):
  import json
  return json.loads(_read_text(p))

def m(way):
  return SimpleNamespace(way_id=way, road_name="CR 31")


def test_parse():
  assert parse_confirmed(None) is None
  assert parse_confirmed({"mph": 33, "seq": 1}) is None
  assert parse_confirmed('{"mph": 55, "seq": 2, "lat": 45.1}') == {"mph": 55, "seq": 2.0, "lat": 45.1}


def test_yes_keyed_by_way_and_direction_and_persists(tmp_path):
  p = str(tmp_path / "o.json")
  o = SpeedOverrides(p)
  e = o.consume({"mph": 55, "seq": 1.0, "way_id": 111, "heading": 90.0}, m(111), 90.0, 0.0)
  assert e["way_id"] == 111 and e["mph"] == 55
  assert abs(o.limit_ms(111, 100.0) - 55 * MPH_TO_MS) < 1e-6
  assert o.limit_ms(111, 270.0) is None   # whole way, read direction only (Oct 25: direction-aware)
  assert abs(o.limit_ms(111, 130.0) - 55 * MPH_TO_MS) < 1e-6   # within 45 deg of the read heading
  assert o.limit_ms(111, 140.0) is None
  assert abs(o.limit_ms(111, None) - 55 * MPH_TO_MS) < 1e-6    # no bearing: old behavior
  assert o.limit_ms(222, 90.0) is None
  assert o.consume({"mph": 55, "seq": 1.0, "way_id": 222, "heading": 90.0}, m(222), 90.0, 1.0) is None  # same seq once
  o2 = SpeedOverrides(p)                        # future drive
  assert abs(o2.limit_ms(111, 80.0) - 55 * MPH_TO_MS) < 1e-6
  o2.consume({"mph": 45, "seq": 2.0, "way_id": 111, "heading": 95.0}, m(111), 95.0, 2.0)   # newer YES replaces
  assert len(o2.entries) == 1 and o2.find(111, 90.0)["mph"] == 45


def test_yes_without_read_time_way_stores_nothing_even_when_matched_later(tmp_path):
  o = SpeedOverrides(str(tmp_path / "o.json"))
  assert o.consume({"mph": 30, "seq": 1.0}, None, 10.0, 0.0) is None
  assert o.consume(None, m(5), 10.0, 3.0) is None                 # no tap-time keying
  assert o.find(5, 10.0) is None and PENDING_MAX_S > 0


def test_request_override_writes_param():
  from openpilot.selfdrive.mapd.speed_overrides import PARAM, request_override

  class P:
    d: dict = {}

    def put(self, k, v):
      self.d[k] = v
  p = P()
  assert request_override(35, {"lat": 1.0, "bearing": 90.0}, 3.0, params=p)
  assert p.d[PARAM] == {"mph": 35, "seq": 3.0, "lat": 1.0, "bearing": 90.0}
  assert not request_override(33, None, 4.0, params=P())


def test_v23_yes_after_car_moved_uses_read_time_way(tmp_path):
  """Sign read on way 111 heading 90; the car turned onto way 222 (heading 0) before the tap."""
  from openpilot.selfdrive.mapd.speed_overrides import request_override

  class P:
    def __init__(self):
      self.d = {}

    def put(self, k, v):
      self.d[k] = v

  params = P()
  assert request_override(45, {"lat": 45.0, "lon": -93.0, "bearing": 90.0}, 7.0, params=params,
                          way_id=111, heading=90.0)
  payload = list(params.d.values())[0]
  o = SpeedOverrides(str(tmp_path / "o.json"))
  e = o.consume(payload, m(222), 0.0, 1.0)          # tap-time match is way 222
  assert e is not None and e["way_id"] == 111 and e["heading"] == 90.0 and e["mph"] == 45
  assert o.find(222, 0.0) is None                    # never the tap-time way
  assert abs(o.limit_ms(111, 80.0) - 45 * MPH_TO_MS) < 1e-6
  assert o.limit_ms(111, 270.0) is None                          # read direction only
  assert o.limit_ms(111, 160.0) is None              # 70 deg off the read heading (Oct 25)
  assert o.limit_ms(111, 130.0) is not None


def test_v23_yes_without_read_time_way_stores_nothing(tmp_path):
  o = SpeedOverrides(str(tmp_path / "o.json"))
  assert o.consume({"mph": 45, "seq": 3.0, "way_id": 0, "heading": 90.0}, m(222), 0.0, 1.0) is None
  assert o.consume(None, m(222), 0.0, 2.0) is None
  assert o.entries == []


def test_yes_equal_to_current_limit_rewrites_nothing(tmp_path):
  o = SpeedOverrides(str(tmp_path / "o.json"))
  osm45 = SimpleNamespace(way_id=111, road_name="CR 31", speed_limit_ms=45 * MPH_TO_MS)
  # Same as OSM on the matched way: nothing stored.
  assert o.consume({"mph": 45, "seq": 1.0, "way_id": 111, "heading": 90.0}, osm45, 90.0, 0.0) is None
  assert o.entries == []
  # Different: stored.
  assert o.consume({"mph": 35, "seq": 2.0, "way_id": 111, "heading": 90.0}, osm45, 90.0, 1.0)["mph"] == 35
  t = o._mtime
  # Same as the stored override: no rewrite.
  assert o.consume({"mph": 35, "seq": 3.0, "way_id": 111, "heading": 95.0}, None, 90.0, 2.0) is None
  assert len(o.entries) == 1 and o._mtime == t
  # Back to the OSM value differs from the override: replaced.
  assert o.consume({"mph": 45, "seq": 4.0, "way_id": 111, "heading": 90.0}, osm45, 90.0, 3.0)["mph"] == 45


# --- both directions, from the sign onward ------------------------------------
# Way 7 runs west -> east along lat 45.0, ~0.0127 deg lon per km.
W7 = [(45.0, -93.0 + i * 0.00127) for i in range(11)]   # ~100 m apart, ~1 km long


def _at(m_east):
  return 45.0, -93.0 + m_east / 100.0 * 0.00127


def test_yes_covers_both_directions_from_sign_onward(tmp_path):
  o = SpeedOverrides(str(tmp_path / "o.json"))
  lat, lon = _at(400)
  e = o.consume({"mph": 30, "seq": 1.0, "way_id": 7, "heading": 90.0, "lat": lat, "lon": lon},
                None, 90.0, 0.0, way_coords=lambda w: W7 if w == 7 else [])
  assert e["both"] and e["dir"] == 1 and 390 < e["from_s"] < 410
  for m_east in (450, 900):                 # past the sign: read direction only (Oct 25)
    assert abs(o.limit_ms(7, 90.0, *_at(m_east), W7) - 30 * MPH_TO_MS) < 1e-6
    assert o.limit_ms(7, 270.0, *_at(m_east), W7) is None
  for hdg in (90.0, 270.0):                 # before the sign: OSM
    assert o.limit_ms(7, hdg, *_at(200), W7) is None
  assert o.limit_ms(7, 90.0) is None        # no position: no span match


def test_westbound_sign_covers_west_part(tmp_path):
  o = SpeedOverrides(str(tmp_path / "o.json"))
  lat, lon = _at(600)
  e = o.consume({"mph": 40, "seq": 1.0, "way_id": 7, "heading": 270.0, "lat": lat, "lon": lon},
                None, 270.0, 0.0, way_coords=lambda w: W7)
  assert e["dir"] == -1
  assert o.limit_ms(7, 270.0, *_at(100), W7) is not None
  assert o.limit_ms(7, 90.0, *_at(100), W7) is None    # eastbound: not the sign's direction (Oct 25)
  assert o.limit_ms(7, 270.0, *_at(800), W7) is None   # behind the sign


def test_span_replaces_heading_only_row(tmp_path):
  """A legacy eastbound-only 30 (Justin's E 30 / W 60 road) is replaced by a both-way span."""
  o = SpeedOverrides(str(tmp_path / "o.json"))
  row = {"v": 2, "way_id": 7, "heading": 90.0, "mph": 30}
  o.entries, o.rows = [row], [row]
  assert o.limit_ms(7, 90.0) and o.limit_ms(7, 270.0) is None
  lat, lon = _at(0)
  o.consume({"mph": 60, "seq": 2.0, "way_id": 7, "heading": 270.0, "lat": lat, "lon": lon},
            None, 270.0, 0.0, way_coords=lambda w: W7)
  assert len(o.entries) == 1 and o.entries[0]["both"]


def test_auto_confirm_payload_and_row_source(tmp_path):
  from openpilot.selfdrive.mapd.speed_overrides import request_override

  class P:
    def __init__(self):
      self.d = {}

    def put(self, k, v):
      self.d[k] = v
  p = P()
  assert request_override(50, {"lat": 45.3, "lon": -95.5, "bearing": 292.0}, 7.0, params=p,
                          way_id=111, heading=292.0, auto="tesla")
  payload = p.d["NAPSpeedSignConfirmed"]
  assert payload["auto"] == "tesla"
  assert parse_confirmed(payload)["auto"] == "tesla"
  assert not request_override(50, None, 8.0, params=P(), way_id=111, heading=292.0, auto="mpp")   # MPP never confirms
  assert "auto" not in parse_confirmed({"mph": 50, "seq": 1, "auto": "mpp"})
  o = SpeedOverrides(str(tmp_path / "o.json"))
  e = o.consume(payload, m(111), 292.0, 0.0)
  assert e["source"] == "sign_auto_tesla"
  e2 = o.consume({"mph": 45, "seq": 9.0, "way_id": 222, "heading": 90.0}, m(222), 90.0, 1.0)
  assert e2["source"] == "sign_yes"


def _legacy_file(p):
  import json
  rows = [{"way_id": 18267060, "heading": 112.0, "mph": 30, "lat": 45.0, "lon": -93.0, "road": "US 12", "t": 1.0,
           "source": "sign_yes"}]
  with open(p, "w") as f:
    json.dump({"version": 1, "overrides": rows}, f)
  return rows


def test_legacy_rows_kept_but_not_applied(tmp_path):
  p = str(tmp_path / "o.json")
  rows = _legacy_file(p)
  raw = _read_text(p)
  o = SpeedOverrides(p)
  assert o.limit_ms(18267060, 110.0) is None
  assert o.find_legacy(18267060, 110.0)["mph"] == 30
  assert _read_text(p) == raw                                  # load never rewrites
  e = o.consume({"mph": 45, "seq": 3.0, "way_id": 777, "heading": 90.0, "lat": 44.9, "lon": -93.1}, None, None, 0.0)
  assert e["v"] == 2 and e["lat"] == 44.9 and e["lon"] == -93.1
  data = _read_json(p)["overrides"]
  assert data[0] == rows[0] and data[1]["way_id"] == 777          # legacy row untouched
  # A new versioned YES on the legacy way applies; the legacy row still stays.
  o.consume({"mph": 60, "seq": 4.0, "way_id": 18267060, "heading": 112.0}, None, None, 1.0)
  assert abs(o.limit_ms(18267060, 112.0) - 60 * MPH_TO_MS) < 1e-6
  assert _read_json(p)["overrides"][0] == rows[0]


def test_clear_removes_current_way_only_and_keeps_legacy(tmp_path):
  p = str(tmp_path / "o.json")
  _legacy_file(p)
  o = SpeedOverrides(p)
  o.consume({"mph": 60, "seq": 1.0, "way_id": 18267060, "heading": 112.0}, None, None, 0.0)
  o.consume({"mph": 40, "seq": 2.0, "way_id": 99, "heading": 292.0}, None, None, 0.0)
  gone = o.clear(18267060, 115.0)
  assert [g["mph"] for g in gone] == [60]
  assert o.limit_ms(18267060, 112.0) is None and o.find(99, 290.0)["mph"] == 40
  data = _read_json(p)["overrides"]
  assert any("v" not in r for r in data) and len(data) == 2
  assert o.clear(18267060, None) == []


def test_new_yes_never_drops_legacy_row(tmp_path):
  p = str(tmp_path / "o.json")
  rows = _legacy_file(p)
  o = SpeedOverrides(p)
  lat, lon = _at(0)
  o.consume({"mph": 60, "seq": 2.0, "way_id": 18267060, "heading": 292.0, "lat": lat, "lon": lon},
            None, 292.0, 0.0, way_coords=lambda w: W7)
  assert _read_json(p)["overrides"][0] == rows[0]


def test_list_legacy_script(tmp_path):
  import sqlite3
  from openpilot.selfdrive.mapd.list_legacy_overrides import rows
  p = str(tmp_path / "o.json")
  _legacy_file(p)
  db = str(tmp_path / "m.sqlite")
  con = sqlite3.connect(db)
  con.execute("CREATE TABLE ways(way_id INTEGER, name TEXT, highway TEXT, maxspeed_ms REAL)")
  con.execute("INSERT INTO ways VALUES (18267060, 'US 12', 'trunk', ?)", (60 * MPH_TO_MS,))
  con.commit()
  con.close()
  r = rows(p, db)
  assert r == [dict(r[0], legacy=True, mph=30, osm_mph=60)]


def test_opposite_direction_skip_logged_once_per_stretch(tmp_path):
  o = SpeedOverrides(str(tmp_path / "o.json"))
  o.consume({"mph": 40, "seq": 1.0, "way_id": 7, "heading": 270.0, "lat": _at(600)[0], "lon": _at(600)[1]},
            None, 270.0, 0.0, way_coords=lambda w: W7)
  for x in (100, 150, 200):
    assert o.limit_ms(7, 90.0, *_at(x), W7) is None
  assert len(o.take_skipped()) == 1
  assert o.take_skipped() == []


def test_row_without_direction_still_applies_and_is_noted(tmp_path):
  o = SpeedOverrides(str(tmp_path / "o.json"))
  row = {"v": 2, "way_id": 7, "heading": None, "mph": 35, "both": True}
  o.entries, o.rows = [row], [row]
  assert o.limit_ms(7, 90.0) is not None and o.limit_ms(7, 270.0) is not None
  assert o.take_undirected() == [(7, 35)]
  assert o.take_undirected() == []
