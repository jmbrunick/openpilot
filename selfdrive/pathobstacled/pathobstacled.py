#!/usr/bin/env python3
"""Log-only path obstacle detector. Little cores, nice 19.

Radar scans every liveTracks frame. The camera is read only after a
candidate has already persisted, and at most 5 times a second. A crash
here must not disengage: manager starts this process as optional.
"""
from __future__ import annotations

import os
import time

from openpilot.selfdrive.controls.lib.path_obstacle import (
  PARAM_OBSTACLE_CHIME,
  PARAM_OBSTACLE_LOG,
  ConeHint,
  PathObstacleDetector,
  lighting_score,
  patch_change_score,
  patch_signature,
  project_road_point,
  projection_box,
  prune_thumbs,
  save_ppm,
  score_row_patches,
  vehicle_exclusion_points,
)
from openpilot.selfdrive.controls.lib.radar_path_gate import model_path_xy

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


def _cone_hint(msg) -> ConeHint | None:
  try:
    return ConeHint(
      active=bool(msg.active),
      side=int(msg.side),
      lat_near=float(msg.latNear),
      lat_mid=float(msg.latMid),
      lat_far=float(msg.latFar),
      barrier=bool(msg.barrier),
      parked=bool(msg.parked),
    )
  except Exception:
    return None


def _tracked_vehicles(model, radar_state):
  """lead0 and lead1, any other model vehicle, and both radard leads."""
  try:
    leads_v3 = model.leadsV3
  except Exception:
    leads_v3 = ()
  try:
    leads_v2 = model.leads
  except Exception:
    leads_v2 = ()
  radar = []
  for name in ("leadOne", "leadTwo"):
    try:
      radar.append(getattr(radar_state, name))
    except Exception:
      continue
  try:
    return vehicle_exclusion_points(leads_v3, leads_v2, radar)
  except Exception:
    return [], []


def _path_std(model) -> float | None:
  try:
    ts = [float(v) for v in model.position.t]
    stds = [float(v) for v in model.position.yStd]
  except Exception:
    return None
  if not ts or not stds:
    return None
  n = min(len(ts), len(stds))
  idx = min(range(n), key=lambda i: abs(ts[i] - 3.0))
  return stds[idx]


def _lane(model, index: int) -> float:
  try:
    return float(model.laneLineProbs[index])
  except Exception:
    return 1.0


def _headlights(cs) -> bool:
  for name in ("headlights", "lowBeamOn", "highBeamOn"):
    try:
      value = getattr(cs, name)
    except Exception:
      continue
    if isinstance(value, bool) and value:
      return True
  return False


def _lighting(sm) -> float:
  model = sm["modelV2"]
  cam = sm["roadCameraState"]
  sun = False
  try:
    from openpilot.selfdrive.controls.lib.lat_low_visibility import sun_ahead_from_fix
    gps = sm["gpsLocationExternal"]
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
    lane_left=_lane(model, 1),
    lane_right=_lane(model, 2),
    path_y_std=_path_std(model),
    integ_lines=integ,
    gain=gain,
    grey=grey,
    sun_ahead=sun,
    headlights=_headlights(sm["carState"]),
  )


def _calib(sm) -> tuple[float, float, float, float]:
  try:
    cal = sm["liveCalibration"]
    rpy = [float(v) for v in cal.rpyCalib]
    height = float(cal.height) if float(cal.height) > 0.2 else 1.22
  except Exception:
    return 0.0, 0.0, 0.0, 1.22
  while len(rpy) < 3:
    rpy.append(0.0)
  return rpy[0], rpy[1], rpy[2], height


def _wide_euler(sm) -> tuple[float, float, float]:
  try:
    rpy = [float(v) for v in sm["liveCalibration"].wideFromDeviceEuler]
  except Exception:
    return 0.0, 0.0, 0.0
  while len(rpy) < 3:
    rpy.append(0.0)
  return rpy[0], rpy[1], rpy[2]


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
  return patch, road


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


def _vision_for(hit, sm, cams: _Cameras):
  """Return (score or None, microseconds, patch or None)."""
  started = time.monotonic()

  def elapsed():
    return (time.monotonic() - started) * 1e6

  roll, pitch, yaw, height = _calib(sm)
  try:
    sensor = sm["roadCameraState"].sensor
  except Exception:
    sensor = ""
  width, cam_h, road_f, wide_f = _cam_size(sensor)
  use_wide = hit.x < 20.0 or abs(hit.lateral) > 2.0
  ground, top = _project(hit, roll, pitch, yaw, height, road_f, width, cam_h)
  if ground is not None and (ground[0] < width * 0.08 or ground[0] > width * 0.92):
    use_wide = True
  focal = road_f
  if use_wide:
    wr, wp, wy = _wide_euler(sm)
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


def _fill(msg, sample, scan_us: float, vision_us: float) -> None:
  body = msg.pathObstacleNAP
  body.active = bool(sample.active)
  body.trackId = int(sample.track_id)
  body.range = float(sample.range_m)
  body.lateral = float(sample.lateral)
  body.vRel = float(sample.v_rel)
  body.vLat = float(sample.v_lat)
  body.radarConf = float(sample.radar_conf)
  body.visionConf = float(sample.vision_conf)
  body.visionEvaluated = bool(sample.vision_evaluated)
  body.lightingScore = float(sample.lighting_score)
  body.wRadar = float(sample.w_radar)
  body.wVision = float(sample.w_vision)
  body.fusedScore = float(sample.fused_score)
  body.agree = bool(sample.agree)
  body.rejectReason = str(sample.reject_reason)
  body.inPath = bool(sample.in_path)
  body.timeToReach = float(sample.time_to_reach)
  body.objectClass = sample.object_class
  body.visionConfHuman = float(sample.vision_conf_human)
  body.visionConfAnimal = float(sample.vision_conf_animal)
  body.visionConfObstacle = float(sample.vision_conf_obstacle)
  body.clusterCount = int(sample.cluster_count)
  body.spanM = float(sample.span_m)
  body.timeToEnter = float(sample.time_to_enter)
  body.entering = bool(sample.entering)
  body.zone = sample.zone if sample.zone in ("none", "inPath", "entering", "roadside") else "none"
  body.brakeGate = bool(sample.brake_gate)
  body.chimed = bool(sample.chimed)
  body.chimeReason = str(sample.chime_reason)
  body.livelyScore = float(sample.lively_score)
  body.scanUs = float(scan_us)
  body.visionUs = float(vision_us)


def _publish_disabled(pm, messaging) -> None:
  from openpilot.selfdrive.controls.lib.path_obstacle import ObstacleSample
  msg = messaging.new_message("pathObstacleNAP")
  _fill(msg, ObstacleSample(reject_reason="disabled", chime_reason="disabled"), 0.0, 0.0)
  pm.send("pathObstacleNAP", msg)


def _run() -> None:
  import cereal.messaging as messaging
  from openpilot.common.params import Params
  from openpilot.common.realtime import Ratekeeper

  params = Params()
  sm = messaging.SubMaster([
    "modelV2", "carState", "liveTracks", "liveCalibration", "coneLineNAP",
    "radarState", "roadCameraState", "wideRoadCameraState", "deviceState",
    "gpsLocationExternal",
  ], poll="liveTracks")
  pm = messaging.PubMaster(["pathObstacleNAP"])
  det = PathObstacleDetector()
  cams = _Cameras()
  rk = Ratekeeper(8, print_delay_threshold=None)
  log_on = True
  chime_on = True
  param_t = 0.0
  last_mono = 0.0
  next_vision = 0.0
  last_disabled = 0.0

  while True:
    sm.update(100)
    now = time.monotonic()
    if now - param_t > 1.0:
      log_on = _bool_param(params, PARAM_OBSTACLE_LOG, True)
      chime_on = _bool_param(params, PARAM_OBSTACLE_CHIME, True)
      param_t = now
    if not log_on:
      det.reset()
      if now - last_disabled > 1.0:
        _publish_disabled(pm, messaging)
        last_disabled = now
      rk.keep_time()
      continue
    if not sm.updated["liveTracks"]:
      rk.keep_time()
      continue
    mono = float(sm.logMonoTime["liveTracks"]) * 1e-9
    dt = 0.1 if last_mono <= 0.0 else min(0.5, max(0.0, mono - last_mono))
    last_mono = mono
    try:
      v_ego = float(sm["carState"].vEgo)
    except Exception:
      v_ego = 0.0
    path_x, path_y = model_path_xy(sm["modelV2"])
    lead_ids, vehicle_pts = _tracked_vehicles(sm["modelV2"], sm["radarState"])
    t0 = time.monotonic()
    hit = det.begin(
      sm["liveTracks"], v_ego, path_x, path_y, dt,
      lead_ids=lead_ids,
      cone=_cone_hint(sm["coneLineNAP"]),
      model_leads=vehicle_pts,
      lighting=_lighting(sm),
    )
    scan_us = (time.monotonic() - t0) * 1e6
    vision = None
    vision_us = 0.0
    motion = 0.0
    if hit.active and now >= next_vision:
      vision, vision_us, patch = _vision_for(hit, sm, cams)
      gap = 1.0 / VISION_HZ
      if vision_us > VISION_BUDGET_S * 1e6:
        gap = max(gap, 0.40)
      next_vision = time.monotonic() + gap
      if patch is not None and hit.track_id:
        sig = patch_signature(patch)
        prev = cams.patch_sig.get(int(hit.track_id))
        if prev is not None:
          motion = patch_change_score(prev, sig)
        cams.patch_sig[int(hit.track_id)] = sig
        if len(cams.patch_sig) > 32:
          cams.patch_sig.pop(next(iter(cams.patch_sig)))
      if vision is not None and patch is not None and vision.conf >= 0.30:
        cams.last_thumb = _maybe_thumb(patch, now, cams.last_thumb)
    sample = det.commit(hit, vision, chime_enabled=chime_on, now=now, vision_motion=motion)
    msg = messaging.new_message("pathObstacleNAP")
    _fill(msg, sample, scan_us, vision_us)
    pm.send("pathObstacleNAP", msg)
    rk.keep_time()


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
