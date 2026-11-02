#!/usr/bin/env python3
"""Box-side: cut road-camera frames / crops around each sign review flag.

NOT for the device. Input is review_flags.jsonl (or review_single.jsonl) copied
off the comma, plus routes that were already uploaded. Output is a review
folder per flag for labeling into the speed-sign training set:

  <out>/<idx>_<route>--<seg>_<mph>/
      meta.json            the flag line + the resolved camera time
      frame_000.jpg ...    full road frames, --before .. --after seconds
      crop_000.jpg ...     right-shoulder crop (US signs) of the same frames
  <out>/index.csv          one row per flag (for the labeling sheet)

Routes are looked up in --routes-dir with either layout:
  <routes-dir>/<route>--<seg>/{fcamera.hevc,qcamera.ts,rlog.zst,qlog.zst,...}   (device layout)
  <routes-dir>/<route>/<seg>.{fcamera.hevc,qcamera.ts,rlog.zst,qlog.zst}
With --download, missing segment files are fetched from the comma API (files
that are ALREADY uploaded; this never asks the device to upload anything).
The JWT is read from --jwt-file and never printed.

Camera time: a flag stores t_mono (CLOCK_MONOTONIC s at the map change) and a
coarse seg_offset_s. When the segment's rlog/qlog is present the offset is
refined to t_mono - first logMonoTime of that segment.

Usage:
  tools/nap/review_flag_frames.py review_flags.jsonl --routes-dir routes --out review \
      [--dongle 1c95345a3286a5db --download --jwt-file ~/comma_jwt.txt] [--camera fcamera|qcamera]
"""
from __future__ import annotations

import argparse
import bz2
import csv
import json
import os
import shutil
import subprocess
import sys
import urllib.request

SEG_S = 60.0
CAM_FPS = {"fcamera": 20.0, "qcamera": 20.0}
API = "https://api.commadotai.com/v1/route/{dongle}|{route}/files"


def load_flags(path: str) -> list[dict]:
  out = []
  with open(path) as f:
    for line in f:
      line = line.strip()
      if not line:
        continue
      try:
        out.append(json.loads(line))
      except ValueError:
        continue
  return out


def seg_file(routes_dir: str, route: str, seg: int, name: str) -> str | None:
  for p in (os.path.join(routes_dir, f"{route}--{seg}", name), os.path.join(routes_dir, route, f"{seg}.{name}")):
    if os.path.isfile(p):
      return p
  return None


class Downloader:
  def __init__(self, dongle: str, jwt_file: str, routes_dir: str) -> None:
    self.dongle, self.routes_dir = dongle, routes_dir
    with open(os.path.expanduser(jwt_file)) as f:
      self._hdr = {"Authorization": "JWT " + f.read().strip()}
    self._files: dict[str, dict] = {}

  def _list(self, route: str) -> dict:
    if route not in self._files:
      req = urllib.request.Request(API.format(dongle=self.dongle, route=route), headers=self._hdr)
      with urllib.request.urlopen(req, timeout=60) as r:
        self._files[route] = json.load(r)
    return self._files[route]

  def fetch(self, route: str, seg: int, name: str) -> str | None:
    kind = {"fcamera.hevc": "cameras", "qcamera.ts": "qcameras", "rlog.zst": "logs", "qlog.zst": "qlogs"}[name]
    urls = [u for u in self._list(route).get(kind, []) if f"/{route}/{seg}/" in u]
    if not urls:
      return None
    dst = os.path.join(self.routes_dir, route, f"{seg}.{name}")
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    tmp = dst + ".part"
    with urllib.request.urlopen(urls[0], timeout=300) as r, open(tmp, "wb") as f:
      shutil.copyfileobj(r, f)
    os.replace(tmp, dst)
    return dst


def first_mono_s(log_path: str) -> float | None:
  """First logMonoTime (s) of an rlog/qlog, via the cereal schema if available."""
  try:
    import capnp
    capnp.remove_import_hook()
    here = os.path.dirname(os.path.abspath(__file__))
    log = capnp.load(os.path.join(here, "..", "..", "cereal", "log.capnp"))
    with open(log_path, "rb") as f:
      raw = f.read()
    if log_path.endswith(".zst"):
      import zstandard
      raw = zstandard.ZstdDecompressor().stream_reader(raw).read(64 * 1024 * 1024)
    elif log_path.endswith(".bz2"):
      raw = bz2.decompress(raw)
    for ev in log.Event.read_multiple_bytes(raw):
      if ev.which() == "initData":   # initData carries the ROUTE start time, not the segment's
        continue
      return ev.logMonoTime * 1e-9
  except Exception as exc:  # schema / decode problems fall back to the coarse offset
    print(f"  (log time refine skipped: {exc})", file=sys.stderr)
  return None


def resolve(routes_dir: str, flag: dict, dl: Downloader | None, camera: str) -> tuple[int, float, str] | None:
  """(segment, offset s, camera path) of the flag's change moment."""
  route, seg = flag.get("route"), flag.get("segment")
  if not route or seg is None:
    return None
  seg = int(seg)
  off = flag.get("seg_offset_s")
  for name in ("rlog.zst", "qlog.zst", "rlog.bz2", "qlog.bz2", "rlog", "qlog"):
    p = seg_file(routes_dir, route, seg, name)
    if p is None and dl is not None and name in ("qlog.zst",):
      p = dl.fetch(route, seg, name)
    if p is not None and flag.get("t_mono") is not None:
      t0 = first_mono_s(p)
      if t0 is not None:
        off = float(flag["t_mono"]) - t0
      break
  if off is None:
    return None
  off = float(off)
  # The change may fall in a neighbour segment if the coarse offset was off.
  while off < 0 and seg > 0:
    seg, off = seg - 1, off + SEG_S
  while off >= SEG_S:
    seg, off = seg + 1, off - SEG_S
  name = "fcamera.hevc" if camera == "fcamera" else "qcamera.ts"
  cam = seg_file(routes_dir, route, seg, name)
  if cam is None and dl is not None:
    cam = dl.fetch(route, seg, name)
  if cam is None:
    return None
  return seg, off, cam


def cut(cam: str, start: float, dur: float, fps: float, out_dir: str, prefix: str, start_idx: int,
        crop: bool) -> int:
  """Extract frames; returns how many were written."""
  start = max(0.0, start)
  if dur <= 0:
    return 0
  vf = f"fps={fps},format=yuvj420p"   # camera streams are limited-range; mjpeg wants full-range
  pattern = os.path.join(out_dir, f"{prefix}_%03d.jpg")
  base = ["ffmpeg", "-loglevel", "error", "-y"]
  if cam.endswith(".hevc"):
    # raw hevc has no index: give the input rate and seek on the OUTPUT side (decode-and-drop);
    # an input-side -ss on raw hevc silently yields no frames.
    seek = ["-r", str(CAM_FPS["fcamera"]), "-i", cam, "-ss", f"{start:.2f}"]
  else:
    seek = ["-ss", f"{start:.2f}", "-i", cam]
  cmd = base + seek + ["-t", f"{dur:.2f}", "-vf", vf if not crop else
                vf + ",crop=iw*0.45:ih*0.55:iw*0.55:ih*0.2", "-start_number", str(start_idx), "-q:v", "3", pattern]
  subprocess.run(cmd, check=False)
  return len([f for f in os.listdir(out_dir) if f.startswith(prefix + "_")]) - start_idx


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("flags")
  ap.add_argument("--routes-dir", required=True)
  ap.add_argument("--out", required=True)
  ap.add_argument("--before", type=float, default=8.0, help="seconds before the map change (sign is usually ahead)")
  ap.add_argument("--after", type=float, default=6.0)
  ap.add_argument("--fps", type=float, default=2.0)
  ap.add_argument("--camera", choices=("fcamera", "qcamera"), default="fcamera")
  ap.add_argument("--no-crop", action="store_true")
  ap.add_argument("--download", action="store_true", help="fetch already-uploaded files from the comma API")
  ap.add_argument("--dongle", default=None)
  ap.add_argument("--jwt-file", default=None)
  a = ap.parse_args(argv)

  dl = None
  if a.download:
    if not (a.dongle and a.jwt_file):
      ap.error("--download needs --dongle and --jwt-file")
    dl = Downloader(a.dongle, a.jwt_file, a.routes_dir)
  os.makedirs(a.out, exist_ok=True)
  rows = []
  for i, flag in enumerate(load_flags(a.flags)):
    res = resolve(a.routes_dir, flag, dl, a.camera)
    tag = f"{i:03d}_{flag.get('route')}--{flag.get('segment')}_{flag.get('mph')}"
    if res is None:
      print(f"{tag}: no camera / segment found, skipped")
      rows.append({"idx": i, "dir": "", "status": "missing", **{k: flag.get(k) for k in ("route", "segment", "mph", "reason", "lat", "lon")}})
      continue
    seg, off, cam = res
    d = os.path.join(a.out, tag)
    os.makedirs(d, exist_ok=True)
    start, end = off - a.before, off + a.after
    n = 0
    # window may straddle a segment boundary
    if start < 0 and seg > 0:
      name = "fcamera.hevc" if a.camera == "fcamera" else "qcamera.ts"
      prev = seg_file(a.routes_dir, flag["route"], seg - 1, name)
      if prev is None and dl is not None:
        prev = dl.fetch(flag["route"], seg - 1, name)
      if prev is not None:
        n += cut(prev, SEG_S + start, -start, a.fps, d, "frame", 0, False)
        if not a.no_crop:
          cut(prev, SEG_S + start, -start, a.fps, d, "crop", 0, True)
    k = n
    n += cut(cam, start, end - max(start, 0.0), a.fps, d, "frame", k, False)
    if not a.no_crop:
      cut(cam, start, end - max(start, 0.0), a.fps, d, "crop", k, True)
    meta = dict(flag, camera=os.path.basename(cam), camera_segment=seg, camera_offset_s=round(off, 2),
                window_s=[-a.before, a.after], fps=a.fps)
    with open(os.path.join(d, "meta.json"), "w") as f:
      json.dump(meta, f, indent=1)
    print(f"{tag}: {n} frames from seg {seg} @ {off:.1f}s -> {d}")
    rows.append({"idx": i, "dir": tag, "status": "ok", "frames": n, "camera_segment": seg, "camera_offset_s": round(off, 2),
                 **{k: flag.get(k) for k in ("route", "segment", "mph", "reason", "lat", "lon")}})
  keys = ["idx", "dir", "status", "frames", "route", "segment", "camera_segment", "camera_offset_s", "mph", "reason", "lat", "lon",
          "sign_present", "sign_mph", "notes"]
  with open(os.path.join(a.out, "index.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
    w.writeheader()
    for r in rows:
      w.writerow(r)
  return 0


if __name__ == "__main__":
  sys.exit(main())
