import json
import os
from types import SimpleNamespace

from openpilot.selfdrive.mapd.review_flags import (
  DEDUPE_S, FLAGS_NAME, READ_AFTER_S, SINGLE_NAME, ReviewFlagger, SegmentClock,
)

MPH = 0.44704


class Clock:
  def __init__(self):
    self.route, self.seg, self.start_mono = "000001bb--32fe2f4345", 11, 1000.0

  def update(self, mono):
    pass

  def at(self, mono):
    return {"route": self.route, "segment": self.seg, "seg_offset_s": round(mono - self.start_mono, 1)}


def drive(fl, t0, t1, tesla, osm, *, speed_mph=45.0, brg=lambda t: 112.0, wall0=1_792_702_000.0,
          lat0=45.3, lon0=-95.6):
  """1 Hz drive heading east at speed_mph; tesla/osm are functions of t."""
  out = []
  t = t0
  while t <= t1:
    lon = lon0 + (t - t0) * speed_mph * MPH / 78_000.0
    f = fl.tick(t, wall0 + t, lat0, lon, brg(t), speed_mph * MPH, "US 12", tesla(t), osm(t), tesla(t))
    if f is not None:
      out.append(f)
    t += 1.0
  return out


def step(at, a, b):
  return lambda t: a if t < at else b


def test_both_maps_same_value_writes_flag_with_route_segment_and_time(tmp_path):
  fl = ReviewFlagger(out_dir=str(tmp_path), seg_clock=Clock())
  out = drive(fl, 1000, 1100, step(1030, 50, 60), step(1033, 50, 60))
  assert len(out) == 1
  f = out[0]
  assert f["kind"] == "both" and f["reason"] == "both_maps_50_to_60" and f["mph"] == 60
  assert f["route"] == "000001bb--32fe2f4345" and f["segment"] == 11 and f["seg_offset_s"] == 30.0
  assert f["t_mono"] == 1030.0 and f["frame_mono_ns"] == 1030 * 10**9
  assert f["tesla"]["old"] == 50 and f["osm"]["new"] == 60
  lines = (tmp_path / FLAGS_NAME).read_text().splitlines()
  assert json.loads(lines[0])["reason"] == "both_maps_50_to_60"
  assert not (tmp_path / SINGLE_NAME).exists()
  # decided only once the read window closed
  assert out[0]["wall"] == 1_792_702_000.0 + 1030


def test_matching_read_suppresses(tmp_path):
  fl = ReviewFlagger(out_dir=str(tmp_path), seg_clock=Clock())
  fl.add_read(1040.0, 60)    # read 10 s after the change
  assert drive(fl, 1000, 1100, step(1030, 50, 60), step(1033, 50, 60)) == []


def test_non_matching_read_does_not_suppress(tmp_path):
  fl = ReviewFlagger(out_dir=str(tmp_path), seg_clock=Clock())
  fl.add_read(1040.0, 55)
  out = drive(fl, 1000, 1100, step(1030, 50, 60), step(1033, 50, 60))
  assert len(out) == 1 and out[0]["reads_nearby"] == [55]


def test_turn_and_slow_are_dropped(tmp_path):
  fl = ReviewFlagger(out_dir=str(tmp_path), seg_clock=Clock())
  assert drive(fl, 1000, 1100, step(1030, 50, 60), step(1031, 50, 60), brg=lambda t: 112.0 if t < 1030 else 200.0) == []
  fl = ReviewFlagger(out_dir=str(tmp_path), seg_clock=Clock())
  assert drive(fl, 1000, 1100, step(1030, 30, 50), step(1031, 30, 50), speed_mph=15.0) == []


def test_single_map_goes_to_separate_log(tmp_path):
  fl = ReviewFlagger(out_dir=str(tmp_path), seg_clock=Clock())
  out = drive(fl, 1000, 1100, step(1030, 30, 55), lambda t: 30)
  assert len(out) == 1 and out[0]["kind"] == "single" and out[0]["reason"] == "tesla_only_30_to_55"
  assert (tmp_path / SINGLE_NAME).exists() and not (tmp_path / FLAGS_NAME).exists()
  fl = ReviewFlagger(out_dir=str(tmp_path / "b"), seg_clock=Clock())
  out = drive(fl, 1000, 1100, step(1030, 50, 60), step(1033, 50, 55))
  assert out[0]["kind"] == "single" and out[0]["reason"] == "maps_disagree_tesla_60_osm_55"


def test_unknown_is_not_a_change(tmp_path):
  fl = ReviewFlagger(out_dir=str(tmp_path), seg_clock=Clock())
  assert drive(fl, 1000, 1100, step(1030, 50, 0), step(1030, 50, 0)) == []


def test_seven_day_dedupe_across_restarts(tmp_path):
  fl = ReviewFlagger(out_dir=str(tmp_path), seg_clock=Clock())
  assert len(drive(fl, 1000, 1100, step(1030, 50, 60), step(1033, 50, 60))) == 1
  fl2 = ReviewFlagger(out_dir=str(tmp_path), seg_clock=Clock())     # next drive, same spot
  assert drive(fl2, 1000, 1100, step(1030, 50, 60), step(1033, 50, 60), wall0=1_792_702_000.0 + 3600) == []
  fl3 = ReviewFlagger(out_dir=str(tmp_path), seg_clock=Clock())     # 8 days later: logged again
  assert len(drive(fl3, 1000, 1100, step(1030, 50, 60), step(1033, 50, 60),
                   wall0=1_792_702_000.0 + DEDUPE_S + 86400)) == 1
  fl4 = ReviewFlagger(out_dir=str(tmp_path), seg_clock=Clock())     # same spot, other direction
  assert len(drive(fl4, 1000, 1100, step(1030, 50, 60), step(1033, 50, 60), brg=lambda t: 292.0,
                   wall0=1_792_702_000.0 + 7200)) == 1


def test_changes_within_30s_merge(tmp_path):
  fl = ReviewFlagger(out_dir=str(tmp_path), seg_clock=Clock())
  out = drive(fl, 1000, 1000 + 40 + READ_AFTER_S + 10, step(1030, 50, 60), step(1050, 50, 60))
  assert len(out) == 1 and out[0]["kind"] == "both"


def test_segment_clock(tmp_path):
  os.makedirs(tmp_path / "000001bb--32fe2f4345--4")
  c = SegmentClock(str(tmp_path))
  c.update(100.0)
  assert c.at(103.0) == {"route": "000001bb--32fe2f4345", "segment": 4, "seg_offset_s": 3.0}
  os.makedirs(tmp_path / "000001bb--32fe2f4345--5")
  c.update(160.0)
  assert c.at(161.5)["segment"] == 5 and c.at(161.5)["seg_offset_s"] == 1.5
  assert SegmentClock(str(tmp_path / "missing")).at(1.0)["route"] is None


def test_mapd_hook_reads_and_flags(tmp_path):
  from openpilot.selfdrive.mapd.tesla_limit import MapdTeslaHook, TeslaLimitFusion

  class Reader:
    def __init__(self):
      self.fusion = TeslaLimitFusion()

    def update(self, evs):
      for t, mph in evs:
        self.fusion.add_base(t, mph, 99)

  q = []
  messaging = SimpleNamespace(drain_sock=lambda s: [q.pop(0)] if q else [])
  fl = ReviewFlagger(out_dir=str(tmp_path), seg_clock=Clock())
  h = MapdTeslaHook(messaging, flagger=fl, reader=Reader(), sock=object())
  h.note_sign(SimpleNamespace(proposedSeq=0, proposedMph=0, signSeq=0, signMph=0), 0.0)
  h.note_sign(SimpleNamespace(proposedSeq=1, proposedMph=60, signSeq=0, signMph=0), 1.0)
  assert list(fl.reads) == [(1.0, 60)]
  flags = []
  for i in range(200):
    t = 1000 + i * 0.5
    q.append((t, 50 if t < 1030 else 60))
    h.update_can()
    f = h.flag_tick(t, 1.7e9 + t, 45.3, -95.6 + i * 1e-4, 112.0, 20.0, "US 12", 50 if t < 1032 else 60, True)
    if f:
      flags.append(f)
  assert [f["reason"] for f in flags] == ["both_maps_50_to_60"]
  assert h.state(1099.0).base_mph == 60
