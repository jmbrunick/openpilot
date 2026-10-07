#!/usr/bin/env python3
"""Camera helper for the path-obstacle chime. Little cores, nice 19.

radard owns the radar scan and publishes pathObstacleNAP. This process
blocks on that socket (no poll loop) and opens the camera only while a
candidate is active, at most 5 times a second. A crash here must not
disengage: manager starts this process as optional.
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import replace

from openpilot.selfdrive.controls.lib.path_obstacle import (
  PARAM_OBSTACLE_CHIME,
  PARAM_OBSTACLE_LOG,
  PathObstacleDetector,
  hit_from_msg,
  lighting_score,
  patch_change_score,
  patch_signature,
  project_road_point,
  projection_box,
  prune_thumbs,
  save_ppm,
  score_row_patches,
)

# comma 3X road camera (OS04C10). Wide focal is the stock fisheye guess.
ROAD_W, ROAD_H, ROAD_F = 1344, 760, 1141.5
WIDE_F = 425.25
AR_W, AR_H, AR_F, AR_WIDE_F = 1928, 1208, 2648.0, 567.0
VISION_HZ = 5.0
VISION_BUDGET_S = 0.008
RECV_TIMEOUT_MS = 20
THUMB_DIR = "/data/media/0/realdata/path_obstacle_thumbs"
THUMB_GAP_S = 2.0
NICE = 19
CORES = [0, 1, 2, 3]
# Block in the kernel until radard publishes, or this long for a param refresh.
RADAR_TIMEOUT_MS = 1500
# CalibrationParams is written by calibrationd every few blocks, not every frame.
CAL_REFRESH_S = 10.0
# roll, pitch, yaw, height, wide-from-device euler. Height 1.22 m is the stock fallback.
_DEFAULT_CALIB = (0.0, 0.0, 0.0, 1.22, 0.0, 0.0, 0.0)


def _bool_param(params, key: str, default: bool) -> bool:
  try:
    return bool(params.get_bool(key))
  except Exception:
    return default


def _cam_size(sensor) -> tuple[int, int, float, float]:
  name = str(sensor or "").lower()
  if "ar0231" in name or "ox03" in name:
    return AR_W, AR_H, AR_F, AR_WIDE_F
  return ROAD_W, ROAD_H, ROAD_F, WIDE_F


def _own(plane):
  """Copy a crop out of camerad's live VisionIPC buffer before reading it."""
  if plane is None:
    return None
  try:
    return plane.copy()
  except Exception:
    return plane


def _lighting_from(body, cam, gps) -> float:
  """Lane and path std come from radard. Exposure and sun are local."""
  try:
    lane = float(body.laneProbMin)
  except Exception:
    lane = 1.0
  if not math.isfinite(lane):
    lane = 1.0
  try:
    std = float(body.pathYStd3s)
  except Exception:
    std = float("nan")
  path_std = std if math.isfinite(std) else None
  sun = False
  try:
    from openpilot.selfdrive.controls.lib.lat_low_visibility import sun_ahead_from_fix
    if gps is not None:
      got = sun_ahead_from_fix(
        latitude=gps.latitude,
        longitude=gps.longitude,
        unix_timestamp_millis=gps.unixTimestampMillis,
        bearing_deg=gps.bearingDeg,
        horizontal_accuracy_m=gps.horizontalAccuracy,
      )
      sun = bool(got)
  except Exception:
    sun = False
  integ = gain = grey = None
  if cam is not None:
    try:
      integ = float(cam.integLines)
    except Exception:
      integ = None
    try:
      gain = float(cam.gain)
    except Exception:
      gain = None
    try:
      grey = float(cam.measuredGreyFraction)
    except Exception:
      grey = None
  return lighting_score(
    lane_left=lane,
    lane_right=lane,
    path_y_std=path_std,
    integ_lines=integ,
    gain=gain,
    grey=grey,
    sun_ahead=sun,
    headlights=False,
  )


def _finite_list(values) -> list[float]:
  try:
    seq = list(values)
  except TypeError:
    return []
  out = []
  for value in seq:
    try:
      num = float(value)
    except (TypeError, ValueError):
      continue
    if math.isfinite(num):
      out.append(num)
  return out


def _calibration_from_params(params):
  """rpy, height, and wide euler from CalibrationParams. No liveCalibration socket."""
  try:
    raw = params.get("CalibrationParams")
  except Exception:
    raw = None
  if not raw:
    return _DEFAULT_CALIB
  try:
    from cereal import log
    with log.Event.from_bytes(raw) as msg:
      cal = msg.liveCalibration
      rpy = _finite_list(cal.rpyCalib)
      wide = _finite_list(cal.wideFromDeviceEuler)
      heights = _finite_list(cal.height)
  except Exception:
    return _DEFAULT_CALIB
  while len(rpy) < 3:
    rpy.append(0.0)
  while len(wide) < 3:
    wide.append(0.0)
  height = heights[0] if heights and heights[0] > 0.2 else 1.22
  return (rpy[0], rpy[1], rpy[2], height, wide[0], wide[1], wide[2])


def _crop(plane, box):
  u0, v0, u1, v1 = box
  height, width = plane.shape[:2]
  x0 = max(0, int(u0))
  x1 = min(width, int(math_ceil(u1)))
  y0 = max(0, int(v0))
  y1 = min(height, int(math_ceil(v1)))
  if x1 - x0 < 4 or y1 - y0 < 4:
    return None, None
  patch = plane[y0:y1, x0:x1]
  road_h = max(4, (y1 - y0) // 3)
  ry0 = min(height - 1, y1)
  ry1 = min(height, ry0 + road_h)
  road = plane[ry0:ry1, x0:x1] if ry1 - ry0 >= 2 else patch[-road_h:, :]
  return _own(patch), _own(road)


def math_ceil(value: float) -> int:
  iv = int(value)
  return iv if iv == value or value < 0 else iv + 1


class _Cameras:
  def __init__(self):
    self.road = None
    self.wide = None
    self._vipc = None
    self.last_thumb = 0.0
    self.patch_sig: dict[int, tuple] = {}

  def _client(self, stream):
    if self._vipc is None:
      from msgq.visionipc import VisionIpcClient, VisionStreamType
      self._vipc = (VisionIpcClient, VisionStreamType)
    VisionIpcClient, VisionStreamType = self._vipc
    kind = VisionStreamType.VISION_STREAM_WIDE_ROAD if stream == "wide" else VisionStreamType.VISION_STREAM_ROAD
    client = VisionIpcClient("camerad", kind, True)
    client.connect(False)
    return client

  def grab(self, wide: bool):
    try:
      from openpilot.selfdrive.speedsignd.nv12 import y_plane_from_nv12
    except Exception:
      return None
    try:
      if wide:
        if self.wide is None:
          self.wide = self._client("wide")
        buf = self.wide.recv(timeout_ms=RECV_TIMEOUT_MS)
      else:
        if self.road is None:
          self.road = self._client("road")
        buf = self.road.recv(timeout_ms=RECV_TIMEOUT_MS)
    except Exception:
      return None
    if buf is None:
      return None
    try:
      return y_plane_from_nv12(buf)
    except Exception:
      return None


def _project(hit, roll, pitch, yaw, height, focal, width, cam_h):
  cx, cy = width / 2.0, cam_h / 2.0
  ground = project_road_point(
    hit.x, hit.y, 0.0, roll=roll, pitch=pitch, yaw=yaw,
    height=height, focal=focal, cx=cx, cy=cy,
  )
  top = project_road_point(
    hit.x, hit.y, 1.0, roll=roll, pitch=pitch, yaw=yaw,
    height=height, focal=focal, cx=cx, cy=cy,
  )
  return ground, top


def _vision_for(hit, cam, calib, cams: _Cameras):
  """Return (score or None, microseconds, patch or None)."""
  started = time.monotonic()

  def elapsed():
    return (time.monotonic() - started) * 1e6

  roll, pitch, yaw, height, wr, wp, wy = calib if calib is not None else _DEFAULT_CALIB
  try:
    sensor = "" if cam is None else cam.sensor
  except Exception:
    sensor = ""
  width, cam_h, road_f, wide_f = _cam_size(sensor)
  use_wide = hit.x < 20.0 or abs(hit.lateral) > 2.0
  ground, top = _project(hit, roll, pitch, yaw, height, road_f, width, cam_h)
  if ground is not None and (ground[0] < width * 0.08 or ground[0] > width * 0.92):
    use_wide = True
  focal = road_f
  if use_wide:
    focal = wide_f
    ground, top = _project(hit, roll + wr, pitch + wp, yaw + wy, height, focal, width, cam_h)
  if ground is None or top is None:
    return None, elapsed(), None
  plane = cams.grab(use_wide)
  if plane is None:
    return None, elapsed(), None
  box = projection_box(ground[0], top[1], ground[1], focal, hit.x)
  patch, road = _crop(plane, box)
  if patch is None:
    return None, elapsed(), None
  try:
    score = score_row_patches(patch, road)
  except Exception:
    return None, elapsed(), None
  return score, elapsed(), patch


def _maybe_thumb(patch, now: float, last: float) -> float:
  if now - last < THUMB_GAP_S:
    return last
  try:
    os.makedirs(THUMB_DIR, exist_ok=True)
  except OSError:
    return last
  path = os.path.join(THUMB_DIR, f"{int(now * 1000)}.ppm")
  if save_ppm(path, patch):
    prune_thumbs(THUMB_DIR)
    return now
  return last


def _finite_or_nan(value) -> float:
  try:
    num = float(value)
  except (TypeError, ValueError):
    return float("nan")
  if not math.isfinite(num):
    return float("nan")
  return num


def _publish_vision(pm, messaging, sample, vision_us: float) -> None:
  msg = messaging.new_message("pathObstacleVisionNAP", valid=True)
  body = msg.pathObstacleVisionNAP
  body.trackId = int(sample.track_id) & 0xFFFFFFFFFFFFFFFF
  body.visionEvaluated = bool(sample.vision_evaluated)
  body.visionConf = _finite_or_nan(sample.vision_conf)
  body.visionConfHuman = _finite_or_nan(sample.vision_conf_human)
  body.visionConfAnimal = _finite_or_nan(sample.vision_conf_animal)
  body.visionConfObstacle = _finite_or_nan(sample.vision_conf_obstacle)
  body.lightingScore = float(sample.lighting_score)
  body.wRadar = float(sample.w_radar)
  body.wVision = float(sample.w_vision)
  body.fusedScore = float(sample.fused_score)
  body.agree = bool(sample.agree)
  obj = sample.object_class if sample.object_class in ("unknown", "human", "animal", "obstacle") else "unknown"
  zone = sample.zone if sample.zone in ("none", "inPath", "entering", "roadside") else "none"
  body.objectClass = obj
  body.zone = zone
  body.brakeGate = bool(sample.brake_gate)
  body.chimed = bool(sample.chimed)
  body.chimeReason = str(sample.chime_reason)
  body.livelyScore = float(sample.lively_score)
  body.visionUs = float(vision_us)
  pm.send("pathObstacleVisionNAP", msg)


class Helper:
  """Blocks on radard. VisionIPC stays closed until a candidate is active."""

  def __init__(self, det: PathObstacleDetector | None = None):
    self.det = det or PathObstacleDetector()
    self.cams = _Cameras()
    self.log_on = True
    self.chime_on = True
    self.param_t = 0.0
    self.calib_t = 0.0
    self.next_vision = 0.0
    self.cam = None
    self.gps = None
    self.calib = _DEFAULT_CALIB

  def refresh_params(self, params, now: float) -> None:
    if now - self.param_t > 1.0:
      prev_log = self.log_on
      self.log_on = _bool_param(params, PARAM_OBSTACLE_LOG, True)
      self.chime_on = _bool_param(params, PARAM_OBSTACLE_CHIME, True)
      self.param_t = now
      if self.log_on and not prev_log:
        self.calib_t = 0.0
    if now - self.calib_t > CAL_REFRESH_S:
      self.calib = _calibration_from_params(params)
      self.calib_t = now

  def absorb(self, cam, gps) -> None:
    if cam is not None:
      self.cam = cam
    if gps is not None:
      self.gps = gps

  def on_radar(self, msg, now: float):
    """Fusion and chime for one active radar hit. None if there is nothing to do.

    The camera is grabbed only when the 5 Hz budget allows it.
    """
    if not self.log_on or msg is None:
      if not self.log_on:
        self.det.reset()
      return None
    try:
      body = msg.pathObstacleNAP
    except Exception:
      return None
    if not bool(getattr(body, "active", False)):
      return None
    hit = replace(hit_from_msg(body), lighting=_lighting_from(body, self.cam, self.gps))
    vision = None
    vision_us = 0.0
    motion = 0.0
    patch = None
    if now >= self.next_vision:
      vision, vision_us, patch = _vision_for(hit, self.cam, self.calib, self.cams)
      gap = 1.0 / VISION_HZ
      if vision_us > VISION_BUDGET_S * 1e6:
        gap = max(gap, 0.40)
      self.next_vision = time.monotonic() + gap
      if patch is not None and hit.track_id:
        sig = patch_signature(patch)
        prev = self.cams.patch_sig.get(int(hit.track_id))
        if prev is not None:
          motion = patch_change_score(prev, sig)
        self.cams.patch_sig[int(hit.track_id)] = sig
        if len(self.cams.patch_sig) > 32:
          self.cams.patch_sig.pop(next(iter(self.cams.patch_sig)))
      if vision is not None and patch is not None and vision.conf >= 0.30:
        self.cams.last_thumb = _maybe_thumb(patch, now, self.cams.last_thumb)
    sample = self.det.commit(hit, vision, chime_enabled=self.chime_on, now=now, vision_motion=motion)
    return sample, vision_us


def _latest_body(messaging, sock):
  last = None
  while True:
    msg = messaging.recv_one_or_none(sock)
    if msg is None:
      return last
    try:
      last = getattr(msg, msg.which())
    except Exception:
      last = msg


def _run() -> None:
  import cereal.messaging as messaging
  from openpilot.common.params import Params

  params = Params()
  from openpilot.common.gps import get_gps_location_service
  # No carState, modelV2, radarState, liveTracks, deviceState, or
  # liveCalibration. Those services are at or near the msgq reader cap.
  # Sun uses whichever GPS this device publishes. Calibration is the param.
  gps_service = get_gps_location_service(params)
  sock = messaging.sub_sock("pathObstacleNAP", timeout=RADAR_TIMEOUT_MS)
  cam_sock = messaging.sub_sock("roadCameraState", conflate=True)
  gps_sock = messaging.sub_sock(gps_service, conflate=True)
  pm = messaging.PubMaster(["pathObstacleVisionNAP"])
  helper = Helper()

  while True:
    msg = messaging.recv_one(sock)
    now = time.monotonic()
    helper.refresh_params(params, now)
    if msg is None or not helper.log_on:
      if not helper.log_on:
        helper.det.reset()
      continue
    try:
      active = bool(msg.pathObstacleNAP.active)
    except Exception:
      continue
    if not active:
      continue
    try:
      helper.absorb(
        _latest_body(messaging, cam_sock),
        _latest_body(messaging, gps_sock),
      )
      result = helper.on_radar(msg, now)
      if result is None:
        continue
      sample, vision_us = result
      _publish_vision(pm, messaging, sample, vision_us)
    except Exception:
      continue


def main() -> None:
  from openpilot.common.realtime import drop_realtime, set_core_affinity
  drop_realtime()
  set_core_affinity(CORES)
  try:
    os.nice(NICE)
  except OSError:
    pass
  _run()


if __name__ == "__main__":
  main()
