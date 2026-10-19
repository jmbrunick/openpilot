#!/usr/bin/env python3
"""NAP camera-first animal/person detector. LOG ONLY.

Publishes animalDetectNAP at up to 5 Hz: YOLOX-nano boxes on two full-
resolution road-camera tiles around the horizon plus a reduced wide-camera
tile, matched to radar (liveTracks) within ±1.5 deg / ±25 % range, with a
camera-only track confirmed after 3 frames. Nothing reads it for alerts or
braking; pathobstacled only copies the matching class into its log.

Little cores 0-3, nice 19. Manager runs it only while NAPAnimalDetect is on,
as an optional process, and every error is caught here, so a crash or a
missing model can never disengage.
"""
from __future__ import annotations

import math
import os
import time

from openpilot.selfdrive.nap.animal_detect.config import (
  CORES, DROP_BACKOFF_MAX_S, DROP_BACKOFF_S, ERROR_BACKOFF_MAX_S, ERROR_BACKOFF_S, NICE, PARAM_ENABLE, RATE_HZ,
)
from openpilot.selfdrive.nap.animal_detect.fuse import Detection, RadarPoint, Tracker, fuse
from openpilot.selfdrive.nap.animal_detect.postprocess import decode
from openpilot.selfdrive.nap.animal_detect.roi import (
  CamModel, bearing_deg, bgr_from_nv12, ground_range, pad_tile, road_tiles, tile_to_source, wide_tile,
)

ROAD_F, WIDE_F = 2648.0, 567.0     # comma 3X AR0231/OX03C10, full 1928x1208
CAL_REFRESH_S = 10.0
PARAM_REFRESH_S = 1.0
STATUS = ("ok", "no_model", "no_frame", "model_error", "backoff")
_DEFAULT_CALIB = (0.0, 0.0, 0.0, 1.22, 0.0, 0.0, 0.0)


def calibration_from_params(params) -> tuple[float, ...]:
  """rpy, height and wide euler from CalibrationParams. Defaults on any problem."""
  try:
    raw = params.get("CalibrationParams")
    if not raw:
      return _DEFAULT_CALIB
    from cereal import log
    with log.Event.from_bytes(raw) as msg:
      cal = msg.liveCalibration
      rpy = [float(v) for v in cal.rpyCalib][:3]
      wide = [float(v) for v in cal.wideFromDeviceEuler][:3]
      h = [float(v) for v in cal.height]
    if len(rpy) < 3 or not all(math.isfinite(v) for v in rpy):
      return _DEFAULT_CALIB
    wide = wide if len(wide) == 3 and all(math.isfinite(v) for v in wide) else [0.0, 0.0, 0.0]
    height = h[0] if h and math.isfinite(h[0]) and h[0] > 0.2 else 1.22
    return (rpy[0], rpy[1], rpy[2], height, wide[0], wide[1], wide[2])
  except Exception:
    return _DEFAULT_CALIB


def cams_from_calib(calib, road_wh=(1928, 1208), wide_wh=(1928, 1208)) -> tuple[CamModel, CamModel]:
  roll, pitch, yaw, h, wr, wp, wy = calib
  road = CamModel(road_wh[0], road_wh[1], ROAD_F * road_wh[0] / 1928.0, roll, pitch, yaw, h)
  # Wide is a fisheye; a pinhole at its centre focal is close enough for bearing and near range.
  wide = CamModel(wide_wh[0], wide_wh[1], WIDE_F * wide_wh[0] / 1928.0, roll + wr, pitch + wp, yaw + wy, h)
  return road, wide


def radar_points(msg) -> list[RadarPoint]:
  out = []
  if msg is None:
    return out
  try:
    for p in msg.points:
      out.append(RadarPoint(int(p.trackId), float(p.dRel), float(p.yRel), float(p.vRel)))
  except Exception:
    return []
  return out


def detect(planes: dict, cams: dict, runner) -> tuple[list[Detection], float, float, int]:
  """planes: {"road": (y, uv), "wide": (y, uv) or None}. Returns (detections, model ms, prep ms, tiles)."""
  t0 = time.perf_counter()
  tiles, imgs = [], []
  if planes.get("road") is not None:
    for t in road_tiles(cams["road"]):
      tiles.append(t)
      imgs.append(pad_tile(bgr_from_nv12(*planes["road"], t)))
  if planes.get("wide") is not None:
    t = wide_tile(cams["wide"])
    tiles.append(t)
    imgs.append(pad_tile(bgr_from_nv12(*planes["wide"], t)))
  prep_ms = (time.perf_counter() - t0) * 1e3
  if not imgs:
    return [], 0.0, prep_ms, 0
  out, model_ms = runner(imgs)
  dets: list[Detection] = []
  for tile, found in zip(tiles, decode(out), strict=False):
    cam = cams[tile.camera]
    for cls, conf, box in found:
      sb = tile_to_source(tile, box)
      u = 0.5 * (sb[0] + sb[2])
      dets.append(Detection(cls, conf, tile.camera, sb, bearing_deg(cam, u), ground_range(cam, sb[3])))
  return dets, model_ms, prep_ms, len(imgs)


def _f(v) -> float:
  try:
    v = float(v)
  except (TypeError, ValueError):
    return float("nan")
  return v if math.isfinite(v) else float("nan")


def build_msg(messaging, dets, *, status, frame_id=0, wide_frame_id=0, model_ms=0.0, prep_ms=0.0, tiles=0, horizon=float("nan")):
  msg = messaging.new_message("animalDetectNAP", valid=True)
  b = msg.animalDetectNAP
  b.status = status if status in STATUS else "model_error"
  b.frameId = int(frame_id) & 0xFFFFFFFF
  b.wideFrameId = int(wide_frame_id) & 0xFFFFFFFF
  b.modelMs = _f(model_ms)
  b.prepMs = _f(prep_ms)
  b.tilesRun = int(tiles) & 0xFF
  b.horizonV = _f(horizon)
  lst = b.init("detections", len(dets))
  for i, d in enumerate(dets):
    e = lst[i]
    e.cls = d.cls
    e.conf = _f(d.conf)
    e.camera = d.camera
    e.x0, e.y0, e.x1, e.y1 = (_f(v) for v in d.box)
    e.bearingDeg = _f(d.bearing_deg)
    e.rangeCam = _f(d.range_cam)
    e.trackId = int(d.track_id) & 0xFFFFFFFF
    e.frames = min(int(d.frames), 0xFFFF)
    e.confirmed = bool(d.confirmed)
    e.radarMatched = bool(d.radar_matched)
    e.radarTrackId = int(d.radar_track_id) & 0xFFFFFFFFFFFFFFFF
    e.range = _f(d.range_m)
    e.vRel = _f(d.v_rel)
    e.lateral = _f(d.lateral_m)
  return msg


class _Cams:
  """Latest NV12 road and wide frames over VisionIPC, with a connect backoff."""

  def __init__(self):
    self.clients = {}
    self.next_try = {"road": 0.0, "wide": 0.0}

  def latest(self, stream: str, now: float):
    from msgq.visionipc import VisionIpcClient, VisionStreamType
    import numpy as np
    c = self.clients.get(stream)
    if c is None:
      if now < self.next_try[stream]:
        return None
      self.next_try[stream] = now + 2.0
      kind = VisionStreamType.VISION_STREAM_WIDE_ROAD if stream == "wide" else VisionStreamType.VISION_STREAM_ROAD
      c = VisionIpcClient("camerad", kind, True)
      if not c.connect(False):
        return None
      self.clients[stream] = c
    buf, last = c.recv(timeout_ms=20), None
    while buf is not None:          # drain to the newest frame
      last = buf
      buf = c.recv(timeout_ms=0)
    if last is None:
      return None
    w, h = int(last.width), int(last.height)
    stride = int(last.stride) or w
    uv_off = int(getattr(last, "uv_offset", 0) or stride * h)
    data = np.frombuffer(last.data, dtype=np.uint8)
    if len(data) < uv_off + stride * (h // 2):
      return None
    y = data[:stride * h].reshape(h, stride)[:, :w].copy()
    uv = data[uv_off:uv_off + stride * (h // 2)].reshape(h // 2, stride)[:, :w].copy()
    return y, uv, int(last.frame_id)


class Daemon:
  def __init__(self, runner_factory=None):
    self.tracker = Tracker()
    self.runner = None
    self.runner_factory = runner_factory
    self.runner_error = ""
    self.calib = _DEFAULT_CALIB
    self.calib_t = -1e9
    self.next_run = 0.0
    self.err_backoff = ERROR_BACKOFF_S
    self.drop_backoff = DROP_BACKOFF_S

  def ensure_runner(self) -> bool:
    if self.runner is not None:
      return True
    try:
      if self.runner_factory is None:
        from openpilot.selfdrive.nap.animal_detect.runner import TinygradRunner
        self.runner = TinygradRunner()
      else:
        self.runner = self.runner_factory()
      return True
    except Exception as e:
      self.runner_error = type(e).__name__
      return False

  def on_model_drop(self, now: float) -> None:
    self.next_run = max(self.next_run, now + self.drop_backoff)
    self.drop_backoff = min(DROP_BACKOFF_MAX_S, self.drop_backoff * 2.0)

  def step(self, now: float, frames: dict, radar_msg, params=None):
    """One detection cycle. Never raises. Returns (status, dets, info dict)."""
    if params is not None and now - self.calib_t > CAL_REFRESH_S:
      self.calib = calibration_from_params(params)
      self.calib_t = now
    if not self.ensure_runner():
      self.next_run = now + 5.0
      return "no_model", [], {}
    road = frames.get("road")
    if road is None:
      self.next_run = now + 0.5
      return "no_frame", [], {}
    try:
      wide = frames.get("wide")
      rc, wc = cams_from_calib(self.calib, road_wh=(road[0].shape[1], road[0].shape[0]),
                               wide_wh=(wide[0].shape[1], wide[0].shape[0]) if wide is not None else (1928, 1208))
      planes = {"road": (road[0], road[1]), "wide": (wide[0], wide[1]) if wide is not None else None}
      dets, model_ms, prep_ms, tiles = detect(planes, {"road": rc, "wide": wc}, self.runner)
      dets = fuse(dets, radar_points(radar_msg), self.tracker)
      self.err_backoff = ERROR_BACKOFF_S
      self.drop_backoff = DROP_BACKOFF_S
      self.next_run = now + 1.0 / RATE_HZ
      from openpilot.selfdrive.nap.animal_detect.roi import vanishing_point
      info = dict(frame_id=road[2], wide_frame_id=wide[2] if wide is not None else 0, model_ms=model_ms,
                  prep_ms=prep_ms, tiles=tiles, horizon=vanishing_point(rc)[1])
      return "ok", dets, info
    except Exception:
      self.tracker.reset()
      self.next_run = now + self.err_backoff
      self.err_backoff = min(ERROR_BACKOFF_MAX_S, self.err_backoff * 2.0)
      return "model_error", [], {}


def _run() -> None:
  import cereal.messaging as messaging
  from openpilot.common.params import Params

  params = Params()
  pm = messaging.PubMaster(["animalDetectNAP"])
  dmd_sock = messaging.sub_sock("drivingModelData", conflate=True, timeout=100)
  tracks_sock = messaging.sub_sock("liveTracks", conflate=True)
  cams = _Cams()
  d = Daemon()
  enabled, param_t, last_status_t = True, -1e9, -1e9
  while True:
    dmd = messaging.recv_one(dmd_sock)   # paced by modeld: run right after it, in the GPU gap
    now = time.monotonic()
    if now - param_t > PARAM_REFRESH_S:
      try:
        enabled = bool(params.get_bool(PARAM_ENABLE))
      except Exception:
        enabled = False
      param_t = now
    if not enabled:
      d.tracker.reset()
      time.sleep(1.0)
      continue
    if dmd is not None:
      try:
        if float(dmd.drivingModelData.frameDropPerc) > 0.0:
          d.on_model_drop(now)
      except Exception:
        pass
    if now < d.next_run:
      continue
    try:
      frames = {"road": cams.latest("road", now), "wide": cams.latest("wide", now)}
      frames = {k: (v[0], v[1], v[2]) if v is not None else None for k, v in frames.items()}
    except Exception:
      frames = {"road": None, "wide": None}
    status, dets, info = d.step(now, frames, messaging.recv_one_or_none(tracks_sock), params)
    if status != "ok" and now - last_status_t < 5.0:
      continue
    last_status_t = now
    try:
      pm.send("animalDetectNAP", build_msg(messaging, dets, status=status, **info))
    except Exception:
      pass


def main() -> None:
  try:
    from openpilot.common.realtime import drop_realtime, set_core_affinity
    drop_realtime()
    set_core_affinity(list(CORES))
  except Exception:
    pass
  try:
    os.nice(NICE)
  except OSError:
    pass
  while True:
    try:
      _run()
    except Exception:
      try:
        from openpilot.common.swaglog import cloudlog
        cloudlog.exception("animaldetd: loop error, restarting in 5 s")
      except Exception:
        pass
      time.sleep(5.0)


if __name__ == "__main__":
  main()
