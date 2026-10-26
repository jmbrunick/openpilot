"""Posted segments under 100 yd with a <= 15 mph jump are written out (Justin, Oct 26 2026)."""
import pytest

from openpilot.selfdrive.mapd.short_segment import SHORT_SEG_M, ShortSegmentHold, filter_next, limit_profile
from openpilot.selfdrive.mapd.speed_overrides import MPH_TO_MS, SpeedOverrides
from openpilot.selfdrive.mapd.tests.test_speed_override_sections import mk_db, pt, way_at

EB = 90.0


def mph(ms):
  return round(ms / MPH_TO_MS)


def road(tmp_path, secs):
  """secs: [(x0, x1, mph)] -> one way per section, alternating digitizing direction."""
  return mk_db(tmp_path, [(x0, x1, m, i % 2 == 1) for i, (x0, x1, m) in enumerate(secs)])


def drive(db, x0, x1, entries=(), step=5.0, src=None):
  """Published mph at each x driving east with the OSM / section value as the raw current limit."""
  h, out = ShortSegmentHold(), []
  o = list(entries)
  x = x0
  while x <= x1:
    w = way_at(db, x)
    lat, lon = pt(x)
    prof, walked = limit_profile(o, db, w["way_id"], w["coords"], lat, lon, EB)
    raw = prof[0][1] if prof else w["maxspeed_ms"]
    out.append((x, mph(raw), mph(h.update(raw, src, x, prof, walked))))
    x += step
  return out


def published(trace, x):
  return min(trace, key=lambda r: abs(r[0] - x))[2]


def test_60_m_sliver_is_written_out_both_ends(tmp_path):
  db = road(tmp_path, [(0, 500, 45), (500, 560, 35), (560, 1500, 45)])
  tr = drive(db, 300, 900)
  assert all(p == 45 for _x, _r, p in tr)
  assert any(r == 35 for _x, r, _p in tr)        # the raw value really dipped


def test_sliver_into_a_new_long_limit_holds_then_takes_it(tmp_path):
  # map changes at A (500) to 50, the 55 sign the map meant stands at B (560)
  db = road(tmp_path, [(0, 500, 45), (500, 560, 50), (560, 1500, 55)])
  tr = drive(db, 300, 900)
  assert published(tr, 530) == 45 and published(tr, 600) == 55


def test_100_yard_segment_applies(tmp_path):
  db = road(tmp_path, [(0, 500, 45), (500, 500 + SHORT_SEG_M + 15, 35), (500 + SHORT_SEG_M + 15, 1500, 45)])
  tr = drive(db, 300, 900)
  assert published(tr, 510) == 35


@pytest.mark.parametrize("low", [25, 29])
def test_big_jump_applies_normally(tmp_path, low):
  db = road(tmp_path, [(0, 500, 45), (500, 560, low), (560, 1500, 45)])
  tr = drive(db, 300, 900)
  assert published(tr, 530) == low


def test_exactly_15_mph_is_written_out(tmp_path):
  db = road(tmp_path, [(0, 500, 45), (500, 560, 30), (560, 1500, 45)])
  assert published(drive(db, 300, 900), 530) == 45


def test_sign_section_counts_as_geometry(tmp_path):
  # map 45 / 55 at A=500; a confirmed 45 section from the sign at B=560 eastbound: 55 lasts 60 m
  db = road(tmp_path, [(0, 500, 45), (500, 1500, 55)])
  o = SpeedOverrides(str(tmp_path / "o.json"))
  w = way_at(db, 560)
  lat, lon = pt(560)
  import json
  assert o.consume(json.dumps({"mph": 45, "seq": 1.0, "way_id": w["way_id"], "heading": EB, "lat": lat, "lon": lon}),
                   None, EB, 1.0, way_coords=db.way_coords, db=db) is not None
  tr = drive(db, 300, 900, entries=o.entries, src="sign")
  assert all(p == 45 for _x, _r, p in tr)


def test_lookahead_skips_the_sliver(tmp_path):
  db = road(tmp_path, [(0, 500, 45), (500, 560, 35), (560, 1500, 45)])
  w = way_at(db, 300)
  lat, lon = pt(300)
  prof, walked = limit_profile([], db, w["way_id"], w["coords"], lat, lon, EB)
  assert filter_next(45 * MPH_TO_MS, (35 * MPH_TO_MS, 190.0), prof, walked) is None


def test_lookahead_jumps_to_the_change_after_the_sliver(tmp_path):
  db = road(tmp_path, [(0, 500, 45), (500, 560, 35), (560, 1500, 30)])
  w = way_at(db, 300)
  lat, lon = pt(300)
  prof, walked = limit_profile([], db, w["way_id"], w["coords"], lat, lon, EB)
  ms, dist = filter_next(45 * MPH_TO_MS, (35 * MPH_TO_MS, 190.0), prof, walked)
  assert mph(ms) == 30 and abs(dist - 250.0) <= 6.0   # lead (-10 m) kept


def test_lookahead_big_jump_or_long_segment_untouched(tmp_path):
  db = road(tmp_path, [(0, 500, 45), (500, 560, 25), (560, 1500, 45)])
  w = way_at(db, 300)
  lat, lon = pt(300)
  prof, walked = limit_profile([], db, w["way_id"], w["coords"], lat, lon, EB)
  nxt = (25 * MPH_TO_MS, 190.0)
  assert filter_next(45 * MPH_TO_MS, nxt, prof, walked) == nxt


def test_lookahead_without_matching_geometry_is_untouched():
  nxt = (35 * MPH_TO_MS, 120.0)
  assert filter_next(45 * MPH_TO_MS, nxt, [], 0.0) == nxt
  prof = [(0.0, 45 * MPH_TO_MS), (300.0, 35 * MPH_TO_MS), (330.0, 45 * MPH_TO_MS)]
  assert filter_next(45 * MPH_TO_MS, nxt, prof, 600.0) == nxt   # different distance: not this change


# --- live Tesla (no geometry) ---

def run(h, seq, src="tesla"):
  return [mph(h.update(m * MPH_TO_MS, src, x)) for x, m in seq]


def test_tesla_short_rise_never_published():
  h = ShortSegmentHold()
  seq = [(x, 45) for x in range(0, 100, 10)] + [(x, 55) for x in range(100, 160, 10)] + [(x, 45) for x in range(160, 300, 10)]
  assert set(run(h, seq)) == {45}


def test_tesla_long_rise_published_after_100_yd():
  h = ShortSegmentHold()
  seq = [(0, 45)] + [(x, 55) for x in range(100, 400, 10)]
  out = run(h, seq)
  first = seq[out.index(55)][0]
  assert 100 + SHORT_SEG_M <= first <= 100 + SHORT_SEG_M + 10


def test_tesla_drop_is_immediate_and_a_sliver_drop_ends_at_once():
  h = ShortSegmentHold()
  seq = [(0, 45), (100, 35), (110, 35), (150, 45), (160, 45)]
  assert run(h, seq) == [45, 35, 35, 45, 45]      # never delayed down; restored without the rise hold


def test_tesla_rise_after_a_long_drop_is_held():
  h = ShortSegmentHold()
  seq = [(0, 45), (100, 35), (300, 45), (350, 45), (400, 45)]
  assert run(h, seq) == [45, 35, 35, 35, 45]


def test_tesla_big_rise_is_immediate():
  h = ShortSegmentHold()
  assert run(h, [(0, 30), (10, 55)]) == [30, 55]
