#!/usr/bin/env python3
"""On-drive MUTCD speed-sign logger (log-only).

Reads the ROAD camera + GNSS, appends JSONL under /data, and publishes
liveSpeedSignNAP for the on-road HUD. Does not write sqlite, does not change
vCruise / HUD MAX, and does not talk to osm.org.

Stock modelV2 has no speedSign head — this is a separate process, default off.
"""
from __future__ import annotations

import time
from typing import Any

from openpilot.selfdrive.speedsignd.debounce import SignDebounce
from openpilot.selfdrive.speedsignd.detect import SpeedSignDetector
from openpilot.selfdrive.speedsignd.hud import LiveSignHold, apply_live_sign
from openpilot.selfdrive.speedsignd.jsonl import JsonlLogger, make_record
from openpilot.selfdrive.speedsignd.nv12 import rgb_from_nv12, y_plane_from_nv12
from openpilot.selfdrive.speedsignd.paths import PARAM_KEY, default_log_path, default_onnx_path

SPEEDSIGND_HZ = 4.0
VISION_TIMEOUT_MS = 200
SERVICE_NAME = "liveSpeedSignNAP"


def should_run_speed_sign_log(started: bool, params: Any, _cp: Any = None) -> bool:
  """Manager gate. Default-off: unset/false param never starts the process."""
  try:
    enabled = params.get_bool(PARAM_KEY)
  except Exception:
    enabled = False
  return bool(started) and bool(enabled)


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


def _connect_road_camera():
  from msgq.visionipc import VisionIpcClient, VisionStreamType
  client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
  return client


def _publish_live(pm, hold: LiveSignHold, signs, now_mono: float, messaging) -> None:
  live, mph, conf = hold.update(signs, now_mono)
  msg = messaging.new_message(SERVICE_NAME)
  msg.valid = live
  apply_live_sign(getattr(msg, SERVICE_NAME), mph=mph, conf=conf, valid=live)
  pm.send(SERVICE_NAME, msg)


def main():
  from cereal import messaging
  from openpilot.common.realtime import Ratekeeper
  from openpilot.common.swaglog import cloudlog
  from openpilot.selfdrive.mapd.gps_fix import gps_sample_from_sm

  log_path = default_log_path()
  onnx_path = default_onnx_path()
  detector = SpeedSignDetector(onnx_path=onnx_path)
  logger = JsonlLogger(log_path)
  hold = LiveSignHold()
  debounce = SignDebounce()
  backend = "yolo-onnx" if detector.onnx is not None else "numpy-mutcd"
  cloudlog.info("speedsignd starting log=%s backend=%s onnx=%s", log_path, backend, onnx_path)
  if detector.onnx is None:
    cloudlog.warning(
      "speedsignd: no ONNX at %s — numpy fallback will not see real roadside signs. "
      "On the 3X: python -m scripts.nap.install_speed_sign_weights",
      onnx_path,
    )

  sm = messaging.SubMaster(["gpsLocationExternal", "gpsLocation"])
  pm = messaging.PubMaster([SERVICE_NAME])
  rk = Ratekeeper(SPEEDSIGND_HZ, print_delay_threshold=None)
  client = None
  last_connect = 0.0

  while True:
    sm.update(0)
    now_mono = time.monotonic()
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
      if y is not None or rgb is not None:
        signs, _written = process_frame(
          y, lat, lon, bearing, gps_ok, detector, logger, time.time(),
          rgb=rgb, debounce=debounce,
        )
    _publish_live(pm, hold, signs, now_mono, messaging)
    rk.keep_time()


if __name__ == "__main__":
  main()
