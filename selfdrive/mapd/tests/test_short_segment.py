"""Posted segments under 100 yd with a <= 15 mph jump are written out (Justin, Oct 26 2026)."""
import json

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


def test_60_m_big_sliver_is_written_out_both_ends(tmp_path):
  db = road(tmp_path, [(0, 500, 30), (500, 560, 60), (560, 1500, 30)])   # DeGraff / Clarkfield shape
  tr = drive(db, 300, 900)
  assert all(p == 30 for _x, _r, p in tr)
  assert any(r == 60 for _x, r, _p in tr)        # the raw value really jumped


def test_big_drop_sliver_with_geometry_is_written_out(tmp_path):
  db = road(tmp_path, [(0, 500, 55), (500, 560, 35), (560, 1500, 55)])
  assert all(p == 55 for _x, _r, p in drive(db, 300, 900))


def test_big_sliver_into_a_new_long_limit_holds_then_takes_it(tmp_path):
  # map 30, a 60 for 60 m, then the road's real 50
  db = road(tmp_path, [(0, 500, 30), (500, 560, 60), (560, 1500, 50)])
  tr = drive(db, 300, 900)
  assert published(tr, 530) == 30 and published(tr, 600) == 50


def test_100_yard_segment_applies(tmp_path):
  db = road(tmp_path, [(0, 500, 30), (500, 500 + SHORT_SEG_M + 15, 60), (500 + SHORT_SEG_M + 15, 1500, 30)])
  assert published(drive(db, 300, 900), 510) == 60


@pytest.mark.parametrize("other", [35, 45, 30])   # 10, 0.. 15 mph from 45: small jumps apply normally
def test_small_jump_applies_normally(tmp_path, other):
  db = road(tmp_path, [(0, 500, 45), (500, 560, other), (560, 1500, 45)])
  assert published(drive(db, 300, 900), 530) == other


def test_16_mph_sliver_is_written_out(tmp_path):
  db = road(tmp_path, [(0, 500, 45), (500, 560, 29), (560, 1500, 45)])
  assert published(drive(db, 300, 900), 530) == 45


def test_sign_section_counts_as_geometry(tmp_path):
  # map 30 / 55 at A=500; a confirmed 30 section from the sign at B=560 eastbound: 55 lasts 60 m
  db = road(tmp_path, [(0, 500, 30), (500, 1500, 55)])
  o = SpeedOverrides(str(tmp_path / "o.json"))
  w = way_at(db, 560)
  lat, lon = pt(560)
  assert o.consume(json.dumps({"mph": 30, "seq": 1.0, "way_id": w["way_id"], "heading": EB, "lat": lat, "lon": lon}),
                   None, EB, 1.0, way_coords=db.way_coords, db=db) is not None
  tr = drive(db, 300, 900, entries=o.entries, src="sign")
  assert all(p == 30 for _x, _r, p in tr)


def _prof(db, x):
  w = way_at(db, x)
  lat, lon = pt(x)
  return limit_profile([], db, w["way_id"], w["coords"], lat, lon, EB)


def test_lookahead_skips_the_sliver(tmp_path):
  db = road(tmp_path, [(0, 500, 30), (500, 560, 60), (560, 1500, 30)])
  prof, walked = _prof(db, 300)
  assert filter_next(30 * MPH_TO_MS, (60 * MPH_TO_MS, 190.0), prof, walked) is None


def test_lookahead_jumps_to_the_change_after_the_sliver(tmp_path):
  db = road(tmp_path, [(0, 500, 30), (500, 560, 60), (560, 1500, 40)])
  prof, walked = _prof(db, 300)
  ms, dist = filter_next(30 * MPH_TO_MS, (60 * MPH_TO_MS, 190.0), prof, walked)
  assert mph(ms) == 40 and abs(dist - 250.0) <= 6.0   # lead (-10 m) kept


@pytest.mark.parametrize("secs", [[(0, 500, 45), (500, 560, 35), (560, 1500, 45)],     # small jump
                                  [(0, 500, 30), (500, 700, 60), (700, 1500, 30)]])    # long segment
def test_lookahead_small_jump_or_long_segment_untouched(tmp_path, secs):
  db = road(tmp_path, secs)
  prof, walked = _prof(db, 300)
  nxt = (secs[1][2] * MPH_TO_MS, 190.0)
  assert filter_next(secs[0][2] * MPH_TO_MS, nxt, prof, walked) == nxt


def test_lookahead_without_matching_geometry_is_untouched():
  nxt = (60 * MPH_TO_MS, 120.0)
  assert filter_next(30 * MPH_TO_MS, nxt, [], 0.0) == nxt
  prof = [(0.0, 30 * MPH_TO_MS), (300.0, 60 * MPH_TO_MS), (330.0, 30 * MPH_TO_MS)]
  assert filter_next(30 * MPH_TO_MS, nxt, prof, 600.0) == nxt   # different distance: not this change


# --- live Tesla (no geometry) ---

def run(h, seq, src="tesla"):
  return [mph(h.update(m * MPH_TO_MS, src, x)) for x, m in seq]


def test_tesla_short_big_rise_never_published():
  h = ShortSegmentHold()
  seq = [(x, 30) for x in range(0, 100, 10)] + [(x, 60) for x in range(100, 160, 10)] + [(x, 30) for x in range(160, 300, 10)]
  assert set(run(h, seq)) == {30}


def test_tesla_long_big_rise_published_after_100_yd():
  h = ShortSegmentHold()
  seq = [(0, 30)] + [(x, 60) for x in range(100, 400, 10)]
  out = run(h, seq)
  first = seq[out.index(60)][0]
  assert 100 + SHORT_SEG_M <= first <= 100 + SHORT_SEG_M + 10


def test_tesla_big_drop_is_immediate_and_a_sliver_drop_ends_at_once():
  h = ShortSegmentHold()
  seq = [(0, 55), (100, 30), (110, 30), (150, 55), (160, 55)]
  assert run(h, seq) == [55, 30, 30, 55, 55]      # never delayed down; restored without the rise hold


def test_tesla_big_rise_after_a_long_drop_is_held():
  h = ShortSegmentHold()
  seq = [(0, 55), (100, 30), (300, 55), (350, 55), (400, 55)]
  assert run(h, seq) == [55, 30, 30, 30, 55]


def test_tesla_small_changes_are_immediate():
  h = ShortSegmentHold()
  assert run(h, [(0, 45), (10, 55), (20, 40)]) == [45, 55, 40]
