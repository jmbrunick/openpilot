"""Tesla-learned lookahead sections (NAPTeslaLimitPrimary, Oct 23 2026)."""
import json
import os
from types import SimpleNamespace

from openpilot.selfdrive.mapd import tesla_learned as tl
from openpilot.selfdrive.mapd.review_flags import FLAGS_NAME, ReviewFlagger, SegmentClock
from openpilot.selfdrive.mapd.speed_overrides import MPH_TO_MS, SpeedOverrides, along_way, effective_next
from openpilot.selfdrive.mapd.tesla_limit import MapdTeslaHook, TeslaLimitFusion
from openpilot.selfdrive.mapd.tests.test_speed_override_sections import mk_db, pt, short_ways, way_at, yes

# OSM: 55 the whole way. Posted (what Tesla shows): 45 from 1500 to 2500 m.
ROAD = [(0, 4000, 55)]


def tesla_at(x):
  return 45 if 1500 <= x < 2500 else 55


def drive(lt, db, wall, tesla=tesla_at, x0=0, x1=4000, step=12.5, osm=55):
  """One eastbound pass at 25 m/s (mapd 2 Hz = 12.5 m per tick). Returns promoted rows."""
  out, x = [], x0
  while x < x1:
    w = way_at(db, x)
    s = along_way(*pt(x), w["coords"])[0]
    r = lt.observe(w["way_id"], s, tesla(x), osm, 25.0, wall + (x - x0) / 25.0)
    if r is not None:
      out.append(r)
    x += step
  r = lt.flush()
  return out + ([r] if r is not None else [])


def learned_mph(lt, db, x):
  w = way_at(db, x)
  ms = lt.limit_ms(w["way_id"], along_way(*pt(x), w["coords"])[0])
  return None if ms is None else round(ms / MPH_TO_MS)


def nxt(db, x, o=None, lt=None, osm_next=None):
  w = way_at(db, x)
  lat, lon = pt(x)
  entries = o.entries if o is not None else []
  sign = o.limit_ms(w["way_id"], 90.0, lat, lon, w["coords"]) if o is not None else None
  cur = sign if sign is not None else w["maxspeed_ms"]
  r = effective_next(entries, db, w["way_id"], w["coords"], lat, lon, 90.0, cur, sign is not None, osm_next,
                     learned=lt.limit_ms if lt is not None and lt.has_active() else None)
  return None if r is None else (round(r[0] / MPH_TO_MS), r[1])


def test_promotion_needs_two_separate_passes(tmp_path):
  db = mk_db(tmp_path, short_ways(ROAD))
  lt = tl.TeslaLearned(str(tmp_path / "l.json"), drive="d1")
  assert drive(lt, db, 1000.0) == []
  assert lt.rows and not lt.has_active()
  # same drive, 10 min later: not a separate pass
  assert drive(lt, db, 1600.0) == []
  assert not lt.has_active()
  assert learned_mph(lt, db, 2000) is None
  # another drive: active
  lt.drive = "d2"
  promoted = drive(lt, db, 5000.0)
  assert promoted and all(r[tl.MPH] == 45 and r[tl.OSM] == 55 for r in promoted)
  assert learned_mph(lt, db, 2000) == 45
  assert learned_mph(lt, db, 1000) is None and learned_mph(lt, db, 3000) is None
  # bins agree with OSM are never stored
  assert all(r[tl.MPH] != r[tl.OSM] for r in lt.rows.values())


def test_same_drive_after_30_min_counts(tmp_path):
  db = mk_db(tmp_path, short_ways(ROAD))
  lt = tl.TeslaLearned(str(tmp_path / "l.json"), drive="d1")
  drive(lt, db, 1000.0)
  assert drive(lt, db, 1000.0 + tl.PASS_GAP_S + 300) != []
  assert learned_mph(lt, db, 2000) == 45


def test_replace_then_retire(tmp_path):
  db = mk_db(tmp_path, short_ways(ROAD))
  lt = tl.TeslaLearned(str(tmp_path / "l.json"), drive="d1")
  drive(lt, db, 0.0)
  lt.drive = "d2"
  drive(lt, db, 5000.0)
  assert learned_mph(lt, db, 2000) == 45
  # a different stable Tesla value replaces the row (1 pass, inactive)
  lt.drive = "d3"
  drive(lt, db, 9000.0, tesla=lambda x: 40 if 1500 <= x < 2500 else 55)
  assert learned_mph(lt, db, 2000) is None
  key = next(k for k, r in lt.rows.items() if r[tl.MPH] == 40)
  assert lt.rows[key][tl.N] == 1 and not lt.rows[key][tl.ACTIVE]
  # Tesla agrees with OSM on a later pass: retired
  lt.drive = "d4"
  drive(lt, db, 13000.0, tesla=lambda x: 55)
  assert lt.rows == {}


def test_held_or_unknown_tesla_never_learned(tmp_path):
  db = mk_db(tmp_path, short_ways(ROAD))
  lt = tl.TeslaLearned(str(tmp_path / "l.json"), drive="d1")
  drive(lt, db, 0.0, tesla=lambda x: 0)
  assert lt.rows == {}


def test_slow_samples_ignored(tmp_path):
  db = mk_db(tmp_path, short_ways(ROAD))
  lt = tl.TeslaLearned(str(tmp_path / "l.json"), drive="d1")
  w = way_at(db, 2000)
  s = along_way(*pt(2000), w["coords"])[0]
  for i in range(20):
    lt.observe(w["way_id"], s, 45, 55, 1.0, float(i))
  lt.flush()
  assert lt.rows == {}


def test_cap_prunes_oldest_and_file_small(tmp_path, monkeypatch):
  monkeypatch.setattr(tl, "MAX_ROWS", 100)
  monkeypatch.setattr(tl, "PRUNE_TO", 90)
  lt = tl.TeslaLearned(str(tmp_path / "l.json"), drive="d1")
  for i in range(150):
    lt.commit_section((10_000 + i, 3), 45, 55, 1000.0 + i)
  assert len(lt.rows) <= 100
  assert (10_000 + 149, 3) in lt.rows and (10_000, 3) not in lt.rows   # oldest last-seen pruned first
  assert lt.save(0.0, force=True)
  assert os.path.getsize(lt.path) < 100 * 80


def test_full_cap_under_1mb(tmp_path):
  lt = tl.TeslaLearned(str(tmp_path / "l.json"), drive="1761234567-12345")
  for i in range(tl.MAX_ROWS):
    lt.commit_section((1_234_567_890 + i, 123), 45, 55, 1761234567.5 + i)
  lt.save(0.0, force=True)
  assert os.path.getsize(lt.path) < 1_000_000


def test_persistence_roundtrip_and_save_period(tmp_path):
  db = mk_db(tmp_path, short_ways(ROAD))
  path = str(tmp_path / "l.json")
  lt = tl.TeslaLearned(path, drive="d1")
  drive(lt, db, 0.0)
  assert lt.save(100.0)
  lt.commit_section((1, 1), 30, 40, 50.0)
  assert not lt.save(120.0)              # inside SAVE_PERIOD_S
  assert lt.save(100.0 + tl.SAVE_PERIOD_S)
  assert not os.path.exists(path + ".tmp")
  d = json.load(open(path))
  assert d["v"] == tl.VERSION
  lt2 = tl.TeslaLearned(path, drive="d2")
  lt2.load()
  assert lt2.rows == dict(lt.rows)
  drive(lt2, db, 9000.0)
  assert learned_mph(lt2, db, 2000) == 45


def test_corrupt_file_ignored(tmp_path):
  path = tmp_path / "l.json"
  path.write_text("{not json")
  lt = tl.TeslaLearned(str(path))
  lt.load()
  assert lt.rows == {}


def _learned_45(tmp_path, db):
  lt = tl.TeslaLearned(str(tmp_path / "l.json"), drive="d1")
  drive(lt, db, 0.0)
  lt.drive = "d2"
  drive(lt, db, 5000.0)
  return lt


def test_lookahead_uses_learned_over_osm(tmp_path):
  db = mk_db(tmp_path, short_ways(ROAD))
  lt = _learned_45(tmp_path, db)
  # OSM has no change ahead; the learned 45 section starts ~500 m ahead
  assert nxt(db, 1000) is None
  m, dist = nxt(db, 1000, lt=lt)
  assert m == 45 and abs(dist - 500) <= 100
  # inside the learned section: the end of it (back to OSM 55) is next
  m, dist = nxt(db, 2000, lt=lt)
  assert m == 55 and abs(dist - 500) <= 100


def test_confirmed_sign_beats_learned_in_lookahead(tmp_path):
  db = mk_db(tmp_path, short_ways(ROAD))
  lt = _learned_45(tmp_path, db)
  o = SpeedOverrides(str(tmp_path / "o.json"))
  yes(o, db, 1400, 35, 90.0, 1)              # confirmed 35 sign covers the stretch
  m, dist = nxt(db, 1000, o=o, lt=lt)
  assert m == 35 and abs(dist - 400) <= 20
  # inside the sign section the learned 45 never shows up as a change ahead
  r = nxt(db, 1700, o=o, lt=lt)
  assert r is None or r[0] != 45


def test_inactive_rows_not_used(tmp_path):
  db = mk_db(tmp_path, short_ways(ROAD))
  lt = tl.TeslaLearned(str(tmp_path / "l.json"), drive="d1")
  drive(lt, db, 0.0)
  assert nxt(db, 1000, lt=lt if lt.has_active() else None) is None


class _Msg:
  def drain_sock(self, sock):
    return []


def test_hook_learns_live_tesla_and_writes_one_review_note(tmp_path):
  db = mk_db(tmp_path, short_ways(ROAD))
  fusion = TeslaLimitFusion()
  flagger = ReviewFlagger(out_dir=str(tmp_path / "nap"), seg_clock=SegmentClock(root=str(tmp_path / "logs")))
  hook = MapdTeslaHook(_Msg(), flagger=flagger, reader=SimpleNamespace(fusion=fusion, update=lambda evs: None), sock=object())
  hook.primary_on = True

  def run(drive_id, wall0, mono0):
    hook.learned.drive = drive_id
    x, t = 0.0, mono0
    while x < 4000:
      fusion.add_base(t, tesla_at(x), 99)
      w = way_at(db, x)
      lat, lon = pt(x)
      hook.learn_tick(t, wall0 + t, w["way_id"], along_way(lat, lon, w["coords"])[0], 55, 25.0, lat, lon, 90.0, "US 12")
      x += 12.5
      t += 0.5
    hook.learn_tick(t, wall0 + t, None, None, 0, 0.0, None, None, None, "")

  hook.learned = tl.TeslaLearned(str(tmp_path / "l.json"))
  run("d1", 1_000_000.0, 0.0)
  run("d2", 1_100_000.0, 1000.0)
  assert hook.learned.has_active()
  lines = [json.loads(s) for s in open(tmp_path / "nap" / FLAGS_NAME)]
  assert lines and all(f["kind"] == "disagree" and f["reason"] == "tesla_osm_disagree_tesla_45_osm_55" for f in lines)
  n = len(lines)
  assert n <= 4    # one per active spot (200 m dedupe on a 1 km section)
  # learned file written (promotion saves at once)
  assert os.path.exists(tmp_path / "l.json")
  # retire + re-promote within 7 days at the same spot: deduped, no new note
  for k in list(hook.learned.rows):
    hook.learned.rows[k][tl.ACTIVE] = 0
    hook.learned.rows[k][tl.N] = 1
  run("d3", 1_200_000.0, 2000.0)
  assert len(open(tmp_path / "nap" / FLAGS_NAME).readlines()) == n


def test_hook_primary_off_learns_nothing(tmp_path):
  hook = MapdTeslaHook(_Msg(), flagger=ReviewFlagger(out_dir=str(tmp_path)),
                       reader=SimpleNamespace(fusion=TeslaLimitFusion(), update=lambda evs: None), sock=object())
  hook.primary_on = False
  assert hook.learned_store() is None
  assert hook.learn_tick(0.0, 0.0, 1, 10.0, 55, 25.0, 0, 0, 0, "") is None
  assert hook.primary_mph(0.0) == 0
