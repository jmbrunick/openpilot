#!/usr/bin/env python3
"""Box-side offline replay of the Tesla map limit feature on uploaded rlogs.

Runs the real modules (selfdrive/mapd/tesla_limit.py, review_flags.py,
selfdrive/speedsignd/auto_confirm.py) over logged CAN + liveMapDataNAP +
liveSpeedSignNAP + GPS, at mapd's 2 Hz (one tick per logged liveMapDataNAP):

  * the new liveMapDataNAP fields per tick -> <out>/fields.csv
  * auto-confirms (each logged prompt kept open up to 20 s as if unanswered)
  * review flags / single-map notes (written under <out>/flags/)

Limits: osmRawMph comes from the logged speedLimit when its source is not a
sign override (0 under an override); on device mapd has the raw value always.

Usage: tools/nap/replay_tesla_limit.py <route_dir> [<route_dir> ...] --out DIR
  route_dir holds <seg>.rlog.zst (comma API layout) or <route>--<seg>/rlog.zst
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import glob
import json
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for p in (ROOT, os.path.join(ROOT, "opendbc_repo")):
  if p not in sys.path:
    sys.path.insert(0, p)

from openpilot.selfdrive.mapd.review_flags import ReviewFlagger
from openpilot.selfdrive.mapd.tesla_limit import TeslaCanReader, BASE_STALE_S
from openpilot.selfdrive.speedsignd.auto_confirm import match_source
from openpilot.selfdrive.speedsignd.confirm import PROMPT_TIMEOUT_S, Proposal

MS_TO_MPH = 2.23694
WANT = {"can", "liveMapDataNAP", "liveSpeedSignNAP", "gpsLocationExternal", "gpsLocation", "carState"}


class FixedSegClock:
  def __init__(self) -> None:
    self.route = None
    self.seg = None
    self.start_mono = None

  def update(self, mono: float) -> None:
    pass

  def at(self, mono: float) -> dict:
    off = mono - self.start_mono if self.start_mono is not None else None
    return {"route": self.route, "segment": self.seg, "seg_offset_s": round(off, 1) if off is not None else None}


class MdView:
  pass


def segments(route_dir: str) -> list[tuple[str, int, str]]:
  route = os.path.basename(os.path.normpath(route_dir))
  out = []
  for p in glob.glob(os.path.join(route_dir, "*.rlog.zst")):
    out.append((route, int(os.path.basename(p).split(".")[0]), p))
  for p in glob.glob(os.path.join(route_dir, "*--*", "rlog.zst")):
    m = re.match(r"(.+)--(\d+)$", os.path.basename(os.path.dirname(p)))
    if m:
      out.append((m.group(1), int(m.group(2)), p))
  return sorted(out, key=lambda x: x[1])


def read_events(path: str, log):
  import zstandard
  with open(path, "rb") as f:
    raw = zstandard.ZstdDecompressor().stream_reader(f).read()
  evs = [e for e in log.Event.read_multiple_bytes(raw) if e.which() in WANT]
  evs.sort(key=lambda e: e.logMonoTime)
  return evs


def main(argv=None) -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("routes", nargs="+")
  ap.add_argument("--out", required=True)
  a = ap.parse_args(argv)
  import capnp
  capnp.remove_import_hook()
  log = capnp.load(os.path.join(ROOT, "cereal", "log.capnp"))

  os.makedirs(a.out, exist_ok=True)
  fields_f = open(os.path.join(a.out, "fields.csv"), "w", newline="")
  fw = csv.writer(fields_f)
  fw.writerow(["wall", "route", "seg", "lat", "lon", "speedLimitMph", "source", "teslaBaseMph", "teslaMppMph", "teslaConf",
               "teslaValid", "osmRawMph", "autoConfirmSeq", "reviewFlag", "reviewReason"])
  summary = {"auto_confirms": [], "prompts": [], "flags": [], "single": [], "can_parse_s": 0.0, "log_s": 0.0}
  flag_dir = os.path.join(a.out, "flags")
  if os.path.isdir(flag_dir):
    for f in os.listdir(flag_dir):
      os.remove(os.path.join(flag_dir, f))

  for route_dir in a.routes:
    reader = TeslaCanReader()
    clock = FixedSegClock()
    flagger = ReviewFlagger(out_dir=flag_dir, seg_clock=clock)
    gps = None
    v_ego = 0.0
    prop_seq = sign_seq = None
    prop: Proposal | None = None
    prop_info = None
    auto_seq = 0
    flag_t = -1e9
    wall_off = None
    for route, seg, path in segments(route_dir):
      evs = read_events(path, log)
      if not evs:
        continue
      clock.route, clock.seg, clock.start_mono = route, seg, evs[0].logMonoTime * 1e-9
      can_batch = []
      for e in evs:
        w = e.which()
        now = e.logMonoTime * 1e-9
        if w == "can":
          can_batch.append(e)
          continue
        if can_batch:
          t0 = time.perf_counter()
          reader.update(can_batch)
          summary["can_parse_s"] += time.perf_counter() - t0
          can_batch = []
        if w in ("gpsLocationExternal", "gpsLocation"):
          g = getattr(e, w)
          if g.hasFix:
            gps = (g.latitude, g.longitude, g.bearingDeg, g.speed)
            wall_off = g.unixTimestampMillis * 1e-3 - now
        elif w == "carState":
          v_ego = e.carState.vEgo
        elif w == "liveSpeedSignNAP":
          s = e.liveSpeedSignNAP
          if int(s.proposedSeq) != prop_seq:
            if prop_seq is not None and int(s.proposedMph) > 0:
              flagger.add_read(now, int(s.proposedMph))
              prop = Proposal(int(s.proposedMph), float(s.proposedConf), int(s.proposedSeq), int(s.proposedWayId),
                              float(s.proposedHeading), float(s.proposedLat), float(s.proposedLon), None, now)
              prop_info = {"t": now, "mph": prop.mph, "conf": round(prop.conf, 3), "way": prop.way_id,
                           "lat": prop.lat, "lon": prop.lon, "route": route, "seg": seg, "auto": None,
                           "wall": round(now + wall_off, 1) if wall_off is not None else None,
                           "when": fmt(now + wall_off) if wall_off is not None else None}
              summary["prompts"].append(prop_info)
            prop_seq = int(s.proposedSeq)
          if int(s.signSeq) != sign_seq:
            if sign_seq is not None and int(s.signMph) > 0:
              flagger.add_read(now, int(s.signMph))
            sign_seq = int(s.signSeq)
        elif w == "liveMapDataNAP":
          m = e.liveMapDataNAP
          st = reader.fusion.state(now)
          src = str(m.source)
          osm_raw = int(round(m.speedLimit * MS_TO_MPH)) if (m.speedLimitValid and "sign" not in src and m.speedLimit > 0) else 0
          md = MdView()
          md.teslaBaseMph, md.teslaValid, md.osmRawMph = st.base_mph, st.valid, osm_raw
          # auto-confirm check (speedsignd side) while the prompt is open
          if prop is not None:
            if now - prop.t >= PROMPT_TIMEOUT_S:
              prop = None
            elif gps is not None:
              how = match_source(prop, md, gps[0], gps[1], gps[2])
              if how is not None:
                auto_seq += 1
                prop_info["auto"] = how
                prop_info["auto_after_s"] = round(now - prop.t, 1)
                prop_info["tesla"], prop_info["osm"] = st.base_mph, osm_raw
                summary["auto_confirms"].append(prop_info)
                prop = None
          flag = None
          if now - flag_t >= 1.0:
            flag_t = now
            fz = reader.fusion
            base_raw = fz.cand if (fz.cand and fz.last_base_t is not None and now - fz.last_base_t <= BASE_STALE_S) else 0
            wall = now + wall_off if wall_off is not None else 0.0
            if gps is not None:
              flag = flagger.tick(now, wall, gps[0], gps[1], gps[2], v_ego or gps[3], str(m.roadName),
                                  int(base_raw or 0), osm_raw, st.mpp_mph, gps_ok=True)
          if flag is not None:
            summary["flags" if flag["kind"] == "both" else "single"].append(flag)
          wall = now + wall_off if wall_off is not None else 0.0
          fw.writerow([round(wall, 1), route, seg, round(m.latitude, 6), round(m.longitude, 6),
                       round(m.speedLimit * MS_TO_MPH), src, st.base_mph, st.mpp_mph, st.conf, int(st.valid), osm_raw,
                       auto_seq, int(flag is not None), flag["reason"] if flag else ""])
      if can_batch:
        reader.update(can_batch)
  fields_f.close()
  with open(os.path.join(a.out, "summary.json"), "w") as f:
    json.dump(summary, f, indent=1, default=str)
  return 0


def fmt(t: float) -> str:
  return dt.datetime.fromtimestamp(t).strftime("%m-%d %H:%M:%S")


if __name__ == "__main__":
  sys.exit(main())
