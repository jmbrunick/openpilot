#!/usr/bin/env python3
"""On-drive MUTCD speed-sign logger (log-only).

Reads the ROAD camera + GNSS, appends JSONL under /data, and publishes
liveSpeedSignNAP for the on-road HUD. Does not write sqlite, does not change
vCruise / HUD MAX, and does not talk to osm.org.

Stock modelV2 has no speedSign head — this is a separate process, default off.

Safety: Logger On starts the process + HUD only. Heavy YOLO never runs while
openpilot or cruise is engaged (or engagement is unknown). Disengaged / parked
testing still uses 1 Hz, skip-on-overrun, SCHED_OTHER + nice 19. 4 Hz YOLO
on a 3X starved modeld (~35% drops) and can TAKE CONTROL / process-timeout.
"""
from __future__ import annotations

import math
import os
import time
from typing import Any

from openpilot.selfdrive.speedsignd.debounce import SignDebounce
from openpilot.selfdrive.speedsignd.detect import SpeedSignDetector
from openpilot.selfdrive.speedsignd.hud import LiveSignHold, apply_live_sign
from openpilot.selfdrive.speedsignd.jsonl import JsonlLogger, make_record
from openpilot.selfdrive.speedsignd.nv12 import rgb_from_nv12, y_plane_from_nv12
from openpilot.selfdrive.speedsignd.paths import PARAM_KEY, default_log_path, default_onnx_path

# Safe default. 4 Hz YOLOv8s tinygrad on ROAD frames saturates a 3X CPU core
# and Ratekeeper catch-up never sleeps. Override: env NAP_SPEED_SIGN_HZ.
SPEEDSIGND_HZ = 1.0
HZ_ENV = "NAP_SPEED_SIGN_HZ"
HZ_MIN = 0.2
HZ_MAX = 4.0
# If an ONNX infer exceeds this (or the loop period), skip frames until free.
INFER_BUDGET_MS = 100.0
INFER_LOG_PERIOD_S = 15.0
# Clearly below modeld (SCHED_FIFO 55). Do not raise modeld.
SPEEDSIGND_NICE = 19
VISION_TIMEOUT_MS = 200
SERVICE_NAME = "liveSpeedSignNAP"
# Retry ONNX after Settings → Install weights without requiring a reboot.
ONNX_RETRY_S = 15.0


def should_run_speed_sign_log(started: bool, params: Any, _cp: Any = None) -> bool:
  """Manager gate. Default-off: unset/false param never starts the process."""
  try:
    enabled = params.get_bool(PARAM_KEY)
  except Exception:
    enabled = False
  return bool(started) and bool(enabled)


def driving_controls_engaged(
  *,
  selfdrive_enabled: bool | None,
  cruise_enabled: bool | None,
) -> bool:
  """True when OP/cruise is controlling, or we cannot tell (fail-safe).

  None = no live sample. Unknown is treated as engaged so YOLO cannot start
  until cereal says the stack is idle.
  """
  if selfdrive_enabled is None and cruise_enabled is None:
    return True
  return bool(selfdrive_enabled) or bool(cruise_enabled)


def should_run_onnx_detect(engaged: bool) -> bool:
  """Heavy ONNX only when the driving stack is not controlling the car."""
  return not bool(engaged)


def _optional_sm_bool(sm: Any, service: str, *attrs: str) -> bool | None:
  """None when this service has never arrived, is dead/invalid, or unreadable."""
  try:
    if int(sm.recv_frame.get(service, -1)) <= 0:
      return None
  except Exception:
    return None
  for flag_name in ("alive", "valid"):
    try:
      flags = getattr(sm, flag_name, None)
      if flags is not None and service in flags and not flags[service]:
        return None
    except Exception:
      return None
  try:
    obj: Any = sm[service]
    for name in attrs:
      obj = getattr(obj, name)
    return bool(obj)
  except Exception:
    return None


def engaged_from_sm(sm: Any) -> bool:
  """selfdriveState.enabled or carState.cruiseState.enabled; unknown → engaged."""
  ss = _optional_sm_bool(sm, "selfdriveState", "enabled")
  cs = _optional_sm_bool(sm, "carState", "cruiseState", "enabled")
  try:
    seen_ss = int(sm.recv_frame.get("selfdriveState", -1)) > 0
  except Exception:
    seen_ss = False
  # A previously live selfdriveState that is now dead/invalid is not "idle".
  if seen_ss and ss is None:
    return True
  return driving_controls_engaged(selfdrive_enabled=ss, cruise_enabled=cs)


def parse_detect_hz(raw: str | None, default: float = SPEEDSIGND_HZ) -> float:
  """Clamp env override. Default 1 Hz; refuse junk / out-of-range."""
  if raw is None or str(raw).strip() == "":
    return float(default)
  try:
    hz = float(raw)
  except (TypeError, ValueError):
    return float(default)
  if not math.isfinite(hz):
    return float(default)
  return min(HZ_MAX, max(HZ_MIN, hz))


def infer_overran(infer_s: float, period_s: float, budget_s: float) -> bool:
  """True when this infer used more than the period or the CPU budget."""
  return infer_s > period_s or infer_s > budget_s


def next_detect_mono(infer_end: float, infer_s: float, period_s: float, budget_s: float) -> float:
  """Earliest monotonic time another ONNX infer may start.

  Cheap infer: Ratekeeper spaces the next loop (return infer_end).
  Overrun: skip until free — wait max(period, infer) after the infer ends so
  Ratekeeper cannot pile catch-up work.
  """
  if infer_overran(infer_s, period_s, budget_s):
    return infer_end + max(period_s, infer_s)
  return infer_end


def reset_ratekeeper_if_behind(rk, now: float) -> bool:
  """Drop Ratekeeper catch-up so an overrun does not burst more infers."""
  if rk.remaining < 0:
    rk._next_frame_time = now + rk._interval
    return True
  return False


def yield_to_modeld() -> None:
  """SCHED_OTHER + nice 19. Lowers speedsignd only; modeld stays FIFO."""
  from openpilot.common.realtime import drop_realtime
  drop_realtime()
  try:
    os.nice(SPEEDSIGND_NICE)
  except OSError:
    pass


def process_frame(
  y,
  lat: float,
  lon: float,
  bearing: float | None,
  gps_ok: bool,
  detector: SpeedSignDetector,
  logger: JsonlLogger,
  now: float,
  rgb=None,
  debounce: SignDebounce | None = None,
) -> tuple[list, list[dict]]:
  """Detect on every ROAD frame. JSONL only with a GNSS fix.

  `debounce` is applied before HUD/JSONL so a single noisy frame does not
  count. Tests omit it and see raw detections.
  """
  signs = [] if (y is None and rgb is None) else detector.detect(y, rgb=rgb)
  if debounce is not None:
    signs = debounce.update(signs, now)
  written: list[dict] = []
  if gps_ok:
    for sign in signs:
      rec = make_record(now, lat, lon, bearing, sign.mph, sign.conf)
      if logger.write(rec):
        written.append(rec)
  return signs, written


def process_observations(
  y,
  lat: float,
  lon: float,
  bearing: float | None,
  gps_ok: bool,
  detector: SpeedSignDetector,
  logger: JsonlLogger,
  now: float,
) -> list[dict]:
  """Detect + append. No GPS fix → no row (lat/lon would be junk)."""
  _signs, written = process_frame(y, lat, lon, bearing, gps_ok, detector, logger, now)
  return written


def detect_if_allowed(
  y,
  lat: float,
  lon: float,
  bearing: float | None,
  gps_ok: bool,
  detector: SpeedSignDetector,
  logger: JsonlLogger,
  now: float,
  *,
  engaged: bool,
  rgb=None,
  debounce: SignDebounce | None = None,
) -> tuple[list, list[dict]]:
  """ONNX/JSONL only when not engaged. HUD keep-alive is the caller's job."""
  if not should_run_onnx_detect(engaged):
    return [], []
  return process_frame(
    y, lat, lon, bearing, gps_ok, detector, logger, now,
    rgb=rgb, debounce=debounce,
  )


def _connect_road_camera():
  from msgq.visionipc import VisionIpcClient, VisionStreamType
  client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
  return client


def live_sign_publish_fields(
  hold: LiveSignHold,
  signs,
  now_mono: float,
  weights_missing: bool,
  detect_paused: bool = False,
) -> tuple[bool, int, float, bool, bool]:
  """msg.valid, mph, conf, weights_missing, detect_paused for liveSpeedSignNAP.

  When ONNX is missing, keep msg.valid so the HUD can show NO WT — never a
  numpy-fallback mph. When engaged, show WAIT and do not update hold.
  """
  if weights_missing:
    return True, 0, 0.0, True, False
  if detect_paused:
    return True, 0, 0.0, False, True
  live, mph, conf = hold.update(signs, now_mono)
  return live, mph if live else 0, conf if live else 0.0, False, False


def _publish_live(
  pm, hold: LiveSignHold, signs, now_mono: float, messaging,
  weights_missing: bool, detect_paused: bool = False,
) -> None:
  msg_valid, mph, conf, missing, paused = live_sign_publish_fields(
    hold, signs, now_mono, weights_missing, detect_paused,
  )
  msg = messaging.new_message(SERVICE_NAME)
  msg.valid = msg_valid
  apply_live_sign(
    getattr(msg, SERVICE_NAME),
    mph=mph, conf=conf, valid=bool(mph), weights_missing=missing, detect_paused=paused,
  )
  pm.send(SERVICE_NAME, msg)


def _log_infer_timing(cloudlog, infer_ms: list[float], skip_count: int, hz: float) -> None:
  n = len(infer_ms)
  mean_ms = sum(infer_ms) / n if n else 0.0
  max_ms = max(infer_ms) if n else 0.0
  cloudlog.info(
    "speedsignd timing hz=%.2f infer_ms mean=%.1f max=%.1f n=%d skip=%d",
    hz, mean_ms, max_ms, n, skip_count,
  )


def main():
  from cereal import messaging
  from openpilot.common.realtime import Ratekeeper
  from openpilot.common.swaglog import cloudlog
  from openpilot.selfdrive.mapd.gps_fix import gps_sample_from_sm

  yield_to_modeld()

  hz = parse_detect_hz(os.environ.get(HZ_ENV))
  period_s = 1.0 / hz
  budget_s = INFER_BUDGET_MS / 1000.0

  log_path = default_log_path()
  onnx_path = default_onnx_path()
  detector = SpeedSignDetector(onnx_path=onnx_path)
  logger = JsonlLogger(log_path)
  hold = LiveSignHold()
  debounce = SignDebounce()
  backend = "yolo-onnx" if detector.onnx is not None else "numpy-mutcd"
  cloudlog.info(
    "speedsignd starting log=%s backend=%s onnx=%s hz=%.2f budget_ms=%.0f nice=%d no_onnx_while_engaged=1",
    log_path, backend, onnx_path, hz, INFER_BUDGET_MS, SPEEDSIGND_NICE,
  )
  if detector.onnx is None:
    cloudlog.warning(
      "speedsignd: no ONNX at %s — numpy fallback will not see real roadside signs. "
      "HUD will show NO WT. Settings → NAP → Install weights, or: "
      "python -m scripts.nap.install_speed_sign_weights",
      onnx_path,
    )

  sm = messaging.SubMaster(["gpsLocationExternal", "gpsLocation", "selfdriveState", "carState"])
  pm = messaging.PubMaster([SERVICE_NAME])
  rk = Ratekeeper(hz, print_delay_threshold=None)
  client = None
  last_connect = 0.0
  last_onnx_try = time.monotonic()
  next_detect = 0.0
  infer_ms: list[float] = []
  skip_count = 0
  last_timing_log = time.monotonic()
  last_paused: bool | None = None

  while True:
    sm.update(0)
    now_mono = time.monotonic()
    engaged = engaged_from_sm(sm)
    allow_detect = should_run_onnx_detect(engaged)
    paused = not allow_detect
    if last_paused != paused:
      cloudlog.info("speedsignd detect %s (engaged=%s)", "paused" if paused else "running", engaged)
      last_paused = paused
    if now_mono - last_timing_log >= INFER_LOG_PERIOD_S:
      _log_infer_timing(cloudlog, infer_ms, skip_count, hz)
      infer_ms = []
      skip_count = 0
      last_timing_log = now_mono
    # Do not load 43 MB ONNX while engaged — wait until the stack is idle.
    if allow_detect and detector.onnx is None and now_mono - last_onnx_try >= ONNX_RETRY_S:
      last_onnx_try = now_mono
      if detector.try_reload():
        cloudlog.info("speedsignd: ONNX loaded after retry backend=yolo-onnx onnx=%s", onnx_path)
    signs: list = []
    if client is None or not client.is_connected():
      if now_mono - last_connect >= 0.5:
        last_connect = now_mono
        try:
          if client is None:
            client = _connect_road_camera()
          client.connect(False)
        except Exception:
          client = None
    else:
      buf = client.recv(timeout_ms=VISION_TIMEOUT_MS)
      y = y_plane_from_nv12(buf) if buf is not None else None
      rgb = rgb_from_nv12(buf) if buf is not None else None
      lat, lon, bearing, gps_ok = gps_sample_from_sm(sm, now=now_mono)
      # On-road: do not run numpy-mutcd when weights are missing (no fake mph / JSONL).
      have_frame = detector.onnx is not None and (y is not None or rgb is not None)
      if have_frame and allow_detect and now_mono >= next_detect:
        t0 = time.monotonic()
        signs, _written = detect_if_allowed(
          y, lat, lon, bearing, gps_ok, detector, logger, time.time(),
          engaged=engaged, rgb=rgb, debounce=debounce,
        )
        infer_s = time.monotonic() - t0
        infer_ms.append(infer_s * 1000.0)
        next_detect = next_detect_mono(t0 + infer_s, infer_s, period_s, budget_s)
      elif have_frame and allow_detect:
        skip_count += 1
    _publish_live(
      pm, hold, signs, now_mono, messaging,
      detector.weights_missing(), detect_paused=not allow_detect,
    )
    rk.keep_time()
    reset_ratekeeper_if_behind(rk, time.monotonic())


if __name__ == "__main__":
  main()
