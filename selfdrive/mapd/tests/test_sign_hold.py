"""Yes holds past its way section; town cap on live Tesla (Oct 26 drive 1d5)."""
import json
import os

from openpilot.selfdrive.mapd.tesla_limit import (MPH_TO_MS, SignHold, cap_town_tesla, road_key, select_current)

DATA = os.path.join(os.path.dirname(__file__), "data", "limit_1d5_stjoe.json")


def mph(ms):
  return round(ms / MPH_TO_MS)


def test_road_key_ignores_direction():
  assert road_key("West Minnesota Street") == road_key("East Minnesota Street")
  assert road_key("3rd Street South") != road_key("MN Highway 23")
  assert road_key("") == ""


def test_hold_beats_tesla_and_osm():
  h = SignHold()
  h.start(30, "W Main St", 0.0, 45.0, -94.0, sign_seq=5)
  hold = h.update(1.0, "W Main St", 45.0001, -94.0)
  assert mph(select_current(None, 55, 25 * MPH_TO_MS, 40 * MPH_TO_MS, hold_ms=hold, osm_estimated=False)[0]) == 30
  # a confirmed section on this way still wins
  assert mph(select_current(35 * MPH_TO_MS, 55, 25 * MPH_TO_MS, hold_ms=hold)[0]) == 35


def test_hold_ends_on_road_change():
  h = SignHold()
  h.start(30, "Main St", 0.0)
  assert h.update(1.0, "") is not None          # unnamed way: no decision
  assert h.update(2.0, "main st") is not None
  assert h.update(3.0, "Oak Ave") is None
  assert h.end_reason == ("road", 30)


def test_hold_ends_on_new_sign_value_not_same_value():
  h = SignHold()
  h.start(30, "Main St", 0.0, sign_seq=5)
  h.note_sign(5, 30)
  h.note_sign(6, 30)
  assert h.update(1.0, "Main St") is not None
  h.note_sign(7, 45)
  assert h.update(2.0, "Main St") is None
  assert h.end_reason == ("sign", 30)


def test_hold_ends_after_distance_and_time():
  h = SignHold()
  h.start(30, "Main St", 0.0, 45.0, -94.0)
  lat = 45.0
  ends = []
  for i in range(1, 25):            # ~100 m steps
    lat += 0.0009
    out = h.update(float(i), "Main St", lat, -94.0)
    if out is None:
      ends.append((i, h.end_reason))
      break
  assert ends and ends[0][1] == ("distance", 30) and 19 <= ends[0][0] <= 21, ends
  h.start(30, "Main St", 0.0)
  assert h.update(599.0, "Main St") is not None
  assert h.update(601.0, "Main St") is None


def test_town_cap():
  assert cap_town_tesla(55, 25 * MPH_TO_MS, False) == 0      # St Joseph
  assert cap_town_tesla(50, 35 * MPH_TO_MS, False) == 0      # Cold Spring
  assert cap_town_tesla(45, 35 * MPH_TO_MS, False) == 45     # within +10
  assert cap_town_tesla(25, 35 * MPH_TO_MS, False) == 25     # lower is fine
  assert cap_town_tesla(65, 40 * MPH_TO_MS, False) == 65     # not town
  assert cap_town_tesla(55, 30 * MPH_TO_MS, True) == 55      # OSM default: no cap
  # old call shape keeps old behaviour
  assert mph(select_current(None, 55, 25 * MPH_TO_MS)[0]) == 55


def _replay(yes_events):
  """Replay 1d5 mapd inputs. sections: way -> mph of the per-way confirmed
  section a Yes created (the way it was given on, plus 3rd St S way 307499710
  that the Cold Spring section already carried to)."""
  rows = json.load(open(DATA))["rows"]
  h = SignHold()
  sections = {}
  last_tesla, last_tesla_t = 0, -1e9
  out = []
  for r in rows:
    t = r["t"]
    if r["tv"] and r["tesla"] > 0:
      last_tesla, last_tesla_t = r["tesla"], t
    tesla = last_tesla if t - last_tesla_t <= 10.0 else 0   # TeslaPrimary dropout hold (approx.)
    h.note_sign(r["seq"], r["sign"])
    for wall, val, ways in yes_events:
      if r["wall"] == wall and not any(w in sections for w in ways):
        for w in ways:
          sections[w] = val
        h.start(val, r["road"], t, r["lat"], r["lon"], sign_seq=r["seq"])
    hold = h.update(t, r["road"], r["lat"], r["lon"])
    sign = sections.get(r["way"])
    cur, _ = select_current(sign * MPH_TO_MS if sign else None, tesla, r["osm"] * MPH_TO_MS,
                            hold_ms=hold, osm_estimated=False)
    out.append((r["wall"], mph(cur), r["osm"]))
  return out


def test_replay_1d5_st_joseph_holds_30():
  # Only the first Yes (08:04:29, way 319653107); later Yes taps not needed now.
  out = _replay([("08:04:29", 30, [319653107])])
  after = [m for w, m, _ in out if "08:04:29" <= w <= "08:05:20"]
  assert after and all(m == 30 for m in after), after
  # before the Yes, in town: Tesla 55 never published over tagged OSM 25-35
  before = [m for w, m, osm in out if "08:03:50" <= w < "08:04:29" and 0 < osm <= 35]
  assert before and max(before) <= 35, before


def test_replay_1d5_cold_spring_no_tesla_50_in_town():
  out = _replay([("07:50:36", 50, [363603405, 307499710])])
  town = [m for w, m, _ in out if "07:50:45" <= w <= "07:51:10"]
  assert town and all(m == 35 for m in town), town
  assert {w: m for w, m, _ in out}["07:50:38"] == 50
