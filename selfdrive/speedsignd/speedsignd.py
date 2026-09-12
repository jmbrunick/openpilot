#!/usr/bin/env python3
"""On-drive MUTCD speed-sign logger (log-only).

Reads the ROAD camera + GNSS, appends JSONL under /data, and publishes
liveSpeedSignNAP for the on-road HUD. Does not write sqlite, does not change
vCruise / HUD MAX, and does not talk to osm.org.

Stock modelV2 has no speedSign head — this is a separate process, default off.

Safety: Logger On starts the process + HUD. Heavy YOLO never runs while
openpilot is actively controlling actuators (selfdriveState.active, or
state in enabled / softDisabling / overriding). Manual driving — including
moving, stock CC, no assist — still runs 1 Hz detect (skip-on-overrun,
nice 19, little cores 0-3, 1 ONNX thread). HUD lights on the first
in-threshold YOLO hit; JSONL still needs two agreeing frames. SubMaster
is polled at 20 Hz so 100 Hz selfdriveState alive/valid does not
false-trigger WAIT. Unknown cereal after a short startup allows throttled
detect (not WAIT). Not gated on park / Force Offroad. Detect copies a
ROAD crop and letterboxes after downsample — never a full-frame RGB
convert. 4 Hz YOLO on a 3X starved modeld and can TAKE CONTROL /
process-timeout.
"""
from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Any

from openpilot.selfdrive.speedsignd.debounce import SignDebounce
from openpilot.selfdrive.speedsignd.hud import LiveSignHold, apply_live_sign
from openpilot.selfdrive.speedsignd.jsonl import JsonlLogger, make_record
from openpilot.selfdrive.speedsignd.detect import (
  CAP_MS_DEFAULT,
  CAP_MS_ENV,
  THREADS_DEFAULT,
  THREADS_ENV,
  SpeedSignDetector,
  limit_infer_threads,
  parse_infer_cap_ms,
  parse_infer_threads,
)
from openpilot.selfdrive.speedsignd.nv12 import copy_nv12_detect_crop
from openpilot.selfdrive.speedsignd.paths import PARAM_KEY, default_log_path, default_onnx_path

# Safe default. 4 Hz YOLOv8s tinygrad on ROAD frames saturates a 3X CPU core
# and Ratekeeper catch-up never sleeps. Override: env NAP_SPEED_SIGN_HZ.
SPEEDSIGND_HZ = 1.0
HZ_ENV = "NAP_SPEED_SIGN_HZ"
HZ_MIN = 0.2
HZ_MAX = 4.0
# Poll selfdriveState faster than detect. 100 Hz service + 1 Hz update makes
# SubMaster.alive flap (timeout is 10/freq = 100 ms) and stuck the WAIT plate.
SM_HZ = 20.0
# After this, unread cereal allows throttled detect instead of forever-WAIT.
UNKNOWN_GRACE_S = 2.0
# If an ONNX infer exceeds this (or the detect period), skip frames until free.
INFER_BUDGET_MS = 100.0
INFER_LOG_PERIOD_S = 15.0
# Clearly below modeld (SCHED_FIFO 55 on core 7). Do not raise modeld.
# Shared little cluster — same mask as loggerd / athenad. Isolcpus 4-7 stay
# with camerad / modeld.
SPEEDSIGND_NICE = 19
SPEEDSIGND_CORES = (0, 1, 2, 3)
VISION_TIMEOUT_MS = 200
SERVICE_NAME = "liveSpeedSignNAP"
# Retry ONNX after Settings → Install weights without requiring a reboot.
ONNX_RETRY_S = 15.0

# Matches selfdrive.selfdrived.state.ACTIVE_STATES (actuators on).
CONTROLLING_STATES = frozenset({"enabled", "softDisabling", "overriding"})
STATE_BY_RAW = {
  0: "disabled",
  1: "preEnabled",
  2: "enabled",
  3: "softDisabling",
  4: "overriding",
}


def should_run_speed_sign_log(started: bool, params: Any, _cp: Any = None) -> bool:
  """Manager gate. Default-off: unset/false param never starts the process."""
  try:
    enabled = params.get_bool(PARAM_KEY)
  except Exception:
    enabled = False
  return bool(started) and bool(enabled)


@dataclass(frozen=True)
class EngagementSample:
  """Last read of selfdriveState for the ONNX / WAIT gate."""
  known: bool
  controlling: bool
  allow_detect: bool
  detect_paused: bool  # HUD WAIT — only when OP is commanding actuators
  enabled: bool | None = None
  active: bool | None = None
  state: str | None = None
  alive: bool | None = None
  valid: bool | None = None
  recv_frame: int = 0
  unknown_after_grace: bool = False


def selfdrive_state_name(state: Any) -> str | None:
  """Normalize cereal OpenpilotState to a short name."""
  if state is None:
    return None
  if isinstance(state, str):
    return state.split(".")[-1]
  if isinstance(state, int):
    return STATE_BY_RAW.get(state)
  raw = getattr(state, "raw", None)
  if raw is not None:
    try:
      mapped = STATE_BY_RAW.get(int(raw))
      if mapped is not None:
        return mapped
    except (TypeError, ValueError):
      pass
  try:
    return str(state).split(".")[-1]
  except Exception:
    return None


def op_controlling(*, active: bool | None = None, state: Any = None,
                   enabled: bool | None = None) -> bool:
  """True only when openpilot is commanding actuators.

  Prefer `active`. `state` in enabled / softDisabling / overriding also
  counts. `disabled` and `preEnabled` do not. Unknown fields are not
  controlling — the caller applies a short startup grace instead of
  treating cereal silence as forever-WAIT.
  """
  name = selfdrive_state_name(state)
  if name == "disabled" or name == "preEnabled":
    return False
  if name in CONTROLLING_STATES:
    return True
  if active is True:
    return True
  if active is False:
    return False
  if enabled is False:
    return False
  if enabled is True:
    return True
  return False


def should_run_onnx_detect(controlling: bool) -> bool:
  """Heavy ONNX when OP is not commanding actuators (manual driving, moving OK)."""
  return not bool(controlling)


def _sm_flag(sm: Any, flag_name: str, service: str) -> bool | None:
  try:
    flags = getattr(sm, flag_name, None)
    if flags is None or service not in flags:
      return None
    return bool(flags[service])
  except Exception:
    return None


def _sm_seen(sm: Any, service: str) -> bool:
  """True after a real selfdriveState payload. Do not use recv_frame <= 0.

  SubMaster starts recv_frame at 0 and writes frame 0 on the first receive.
  The old `recv_frame <= 0` check treated that first (and only) packet as
  unseen, then a 1 Hz loop left alive stale → permanent WAIT.
  """
  try:
    seen = getattr(sm, "seen", None)
    if seen is not None and service in seen:
      return bool(seen[service])
  except Exception:
    pass
  try:
    return int(sm.recv_frame.get(service, -1)) > 0
  except Exception:
    return False


def engagement_from_sm(
  sm: Any,
  *,
  now: float | None = None,
  started_at: float = 0.0,
  grace_s: float = UNKNOWN_GRACE_S,
) -> EngagementSample:
  """Read engagement without treating alive/valid flaps as 'OP is driving'.

  Last parsed active/state/enabled wins even if SubMaster marks the socket
  dead — 1 Hz poll of a 100 Hz service makes alive timeout (100 ms) fire
  every tick. Unknown cereal: no YOLO for `grace_s`, then throttled detect
  (not WAIT). WAIT is only detect_paused when we know OP is controlling.
  """
  service = "selfdriveState"
  try:
    recv_frame = int(sm.recv_frame.get(service, 0) or 0)
  except Exception:
    recv_frame = 0
  alive = _sm_flag(sm, "alive", service)
  valid = _sm_flag(sm, "valid", service)
  seen = _sm_seen(sm, service)

  active = enabled = None
  state: str | None = None
  known = False
  if seen:
    try:
      obj: Any = sm[service]
      active = bool(obj.active)
      enabled = bool(obj.enabled)
      state = selfdrive_state_name(getattr(obj, "state", None))
      known = True
    except Exception:
      known = False

  controlling = op_controlling(active=active, state=state, enabled=enabled) if known else False
  if known:
    allow_detect = not controlling
    detect_paused = controlling
    unknown_after = False
  else:
    # now is None (unit tests): treat as post-grace so unknown allows detect.
    in_grace = False if now is None else (float(now) - float(started_at)) < float(grace_s)
    allow_detect = not in_grace
    detect_paused = False
    unknown_after = not in_grace

  return EngagementSample(
    known=known,
    controlling=controlling,
    allow_detect=allow_detect,
    detect_paused=detect_paused,
    enabled=enabled,
    active=active,
    state=state,
    alive=alive,
    valid=valid,
    recv_frame=recv_frame,
    unknown_after_grace=unknown_after,
  )


def engagement_log_fields(sample: EngagementSample) -> str:
  return (
    f"controlling={sample.controlling} enabled={sample.enabled} "
    + f"active={sample.active} state={sample.state} "
    + f"alive={sample.alive} valid={sample.valid} known={sample.known}"
  )


def engaged_from_sm(sm: Any, **kwargs) -> bool:
  """Back-compat: True when OP is commanding actuators (not cereal-unknown)."""
  return engagement_from_sm(sm, **kwargs).controlling


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


def next_detect_mono(infer_end: float, infer_s: float, period_s: float, budget_s: float,
                     cap_s: float = 0.0) -> float:
  """Earliest monotonic time another ONNX infer may start.

  Cheap infer: Ratekeeper spaces the next loop (return infer_end).
  Overrun: skip until free — wait max(period, infer) after the infer ends so
  Ratekeeper cannot pile catch-up work. Over the optional infer cap: one extra
  period so a 1.8 s tinygrad session cannot immediately start another.
  """
  wait = 0.0
  if infer_overran(infer_s, period_s, budget_s):
    wait = max(period_s, infer_s)
  if cap_s > 0.0 and infer_s > cap_s:
    wait = max(wait, infer_s) + period_s
  return infer_end + wait if wait else infer_end


def reset_ratekeeper_if_behind(rk, now: float) -> bool:
  """Drop Ratekeeper catch-up so an overrun does not burst more infers."""
  if rk.remaining < 0:
    rk._next_frame_time = now + rk._interval
    return True
  return False


def yield_to_modeld() -> None:
  """SCHED_OTHER + nice 19 + little cores. Lowers speedsignd only; modeld stays FIFO."""
  from openpilot.common.realtime import drop_realtime, set_core_affinity
  drop_realtime()
  try:
    os.nice(SPEEDSIGND_NICE)
  except OSError:
    pass
  try:
    set_core_affinity(list(SPEEDSIGND_CORES))
  except Exception:
    pass
  limit_infer_threads()


def should_reset_detect_after_wait(last_allow: bool | None, allow_detect: bool) -> bool:
  """First manual tick after WAIT: do not sit out a leftover next_detect."""
  return bool(allow_detect) and last_allow is False


def drain_vision_latest(client) -> None:
  """Non-blocking ROAD recv so WAIT idle does not wedge the VisionIPC socket."""
  if client is None:
    return
  try:
    client.recv(timeout_ms=0)
  except Exception:
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
  nv12=None,
) -> tuple[list, list[dict]]:
  """Detect on every ROAD frame. JSONL only with a GNSS fix.

  With `debounce`: HUD uses the first in-threshold hit; JSONL still needs
  two agreeing frames. Tests omit debounce and see raw detections.
  """
  signs = [] if (y is None and rgb is None and nv12 is None) else detector.detect(y, rgb=rgb, nv12=nv12)
  hud_signs = signs
  jsonl_signs = signs
  if debounce is not None:
    split = debounce.update_split(signs, now)
    hud_signs = split.hud
    jsonl_signs = split.confirmed
  written: list[dict] = []
  if gps_ok:
    for sign in jsonl_signs:
      rec = make_record(now, lat, lon, bearing, sign.mph, sign.conf)
      if logger.write(rec):
        written.append(rec)
  return hud_signs, written


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


@dataclass
class InferOutcome:
  signs: list
  written: list[dict]
  infer_s: float
  error: str | None = None
  diag: dict | None = None


def format_infer_diag(diag: dict | None, *, allow_detect: bool) -> str:
  """One-line on-car fields: backend, frame, letterbox, crop, sha, peak."""
  d = diag or {}
  crop = d.get("crop") or (0, 0, 0, 0)
  if isinstance(crop, (list, tuple)) and len(crop) == 4:
    crop_s = f"{int(crop[0])},{int(crop[1])} {int(crop[2])}x{int(crop[3])}"
  else:
    crop_s = str(crop)
  out = d.get("out_shape", ())
  if isinstance(out, (list, tuple)):
    out_s = "x".join(str(int(v)) for v in out) if out else "-"
  else:
    out_s = str(out)
  sl_conf = float(d.get("sl_peak_conf", 0.0) or 0.0)
  sl_name = d.get("sl_peak_name", "") or ""
  n_over = int(d.get("n_over", 0) or 0)
  # Always log sl_peak — Justin's R2-1 miss is n_over=0 with a weak speedLimit*.
  # When n_over>0 the global peak is often stop; sl_peak is the HUD-relevant head.
  sl_s = f" sl_peak={sl_conf:.2f}/{sl_name}"
  top3 = d.get("top3") or ()
  if top3:
    top_s = " top=" + ",".join(f"{n}:{c:.2f}" for n, c in list(top3)[:3])
  else:
    top_s = ""
  posted = d.get("posted") or ()
  if posted:
    posted_s = " cls=" + ",".join(f"{int(mph)}:{float(c):.2f}" for mph, c in posted)
  else:
    posted_s = ""
  refine = d.get("refine") or ()
  if refine:
    parts = []
    for row in refine:
      if not row or len(row) < 4:
        continue
      class_mph, hud_mph, ref_mph, ref_conf = row[0], row[1], row[2], row[3]
      if ref_mph is None:
        parts.append(f"-({int(class_mph)})")
      elif int(ref_mph) == int(hud_mph) == int(class_mph):
        parts.append(f"{int(ref_mph)}:{float(ref_conf):.2f}")
      else:
        parts.append(f"{int(hud_mph)}:{float(ref_conf):.2f}(class={int(class_mph)})")
    refine_s = " refine=" + ",".join(parts) if parts else ""
  else:
    refine_s = ""
  return (
    f"backend={d.get('backend', '?')} "
    + f"frame={int(d.get('frame_w', 0) or 0)}x{int(d.get('frame_h', 0) or 0)} "
    + f"letterbox={int(d.get('letterbox', 0) or 0)} crop={crop_s} "
    + f"onnx={d.get('weights_path', '')} sha={d.get('weights_sha', '')} "
    + f"allow={int(bool(allow_detect))} out={out_s} "
    + f"peak={float(d.get('peak_conf', 0.0) or 0.0):.2f}/{d.get('peak_name', '')} "
    + f"n_over={n_over}"
    + sl_s
    + f" n_over_sl={int(d.get('n_over_sl', 0) or 0)}"
    + top_s
    + posted_s
    + refine_s
    + " "
    + f"luma={float(d.get('luma_mean', 0.0) or 0.0):.0f}/{float(d.get('luma_std', 0.0) or 0.0):.0f} "
    + f"chroma={int(d.get('chroma', 0) or 0)} "
    + f"prep={float(d.get('prep_ms', 0.0) or 0.0):.0f} "
    + f"sess={float(d.get('sess_ms', 0.0) or 0.0):.0f}"
  )


def detect_skip_reason(
  *,
  connected: bool,
  onnx: bool,
  busy: bool,
  holdoff: bool,
  buf_empty: bool | None = None,
  parse_fail: bool = False,
) -> str:
  """Why YOLO did not start. Distinguishes infer-never-ran from raw=[]."""
  if not connected:
    return "vision-disconnected"
  if not onnx:
    return "no-onnx"
  if parse_fail:
    return "nv12-parse"
  if buf_empty:
    return "road-recv-empty"
  if busy:
    return "infer-busy"
  if holdoff:
    return "holdoff"
  return "ok"


class InferSlot:
  """At most one ONNX infer. The 20 Hz loop never joins on engage.

  Re-engage used to block speedsignd for 300–1500 ms on an in-flight YOLO
  (and leave a core hot) → Communication Issue Between Processes. pause()
  drops the result immediately and refuses new work. The leftover infer may
  still finish at nice 19; we do not start another and we do not wait.

  resume() only clears the start-gate. A generation counter keeps the
  abandoned infer from publishing after WAIT clears (that used to apply
  skip-on-overrun to a stale leftover and delay the first real manual detect).
  """

  def __init__(self):
    self._lock = threading.Lock()
    self._cancel = threading.Event()
    self._busy = False
    self._outcome: InferOutcome | None = None
    self._gen = 0

  @property
  def busy(self) -> bool:
    with self._lock:
      return self._busy

  def pause(self) -> bool:
    """Cancel in-flight work and drop results. True if an infer was busy."""
    self._cancel.set()
    with self._lock:
      was_busy = self._busy
      self._outcome = None
      self._gen += 1
      return was_busy

  def resume(self) -> None:
    self._cancel.clear()

  def take(self) -> InferOutcome | None:
    with self._lock:
      out = self._outcome
      self._outcome = None
      return out

  def start(self, fn) -> bool:
    """Run fn() in a daemon thread. fn returns (signs, written) or + diag."""
    with self._lock:
      if self._busy or self._cancel.is_set():
        return False
      self._busy = True
      self._outcome = None
      gen = self._gen

    def run():
      t0 = time.monotonic()
      signs: list = []
      written: list[dict] = []
      diag = None
      error = None
      try:
        if self._cancel.is_set():
          return
        result = fn()
        if isinstance(result, tuple) and len(result) >= 3:
          signs, written, diag = result[0], result[1], result[2]
        else:
          signs, written = result
      except Exception as e:
        error = f"{type(e).__name__}: {e}"
        signs, written = [], []
      finally:
        infer_s = time.monotonic() - t0
        with self._lock:
          if not self._cancel.is_set() and gen == self._gen:
            self._outcome = InferOutcome(
              list(signs), list(written), infer_s, error=error, diag=diag,
            )
          self._busy = False

    threading.Thread(target=run, name="speedsignd-onnx", daemon=True).start()
    return True


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
  controlling: bool,
  rgb=None,
  debounce: SignDebounce | None = None,
  nv12=None,
) -> tuple[list, list[dict]]:
  """ONNX/JSONL only when OP is not commanding actuators. HUD is the caller's job."""
  if not should_run_onnx_detect(controlling):
    return [], []
  return process_frame(
    y, lat, lon, bearing, gps_ok, detector, logger, now,
    rgb=rgb, debounce=debounce, nv12=nv12,
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
  numpy-fallback mph. When OP is controlling, show WAIT and do not update
  hold. Unknown cereal must not set detect_paused (no false WAIT).
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
  threads = parse_infer_threads(os.environ.get(THREADS_ENV), THREADS_DEFAULT)
  cap_ms = parse_infer_cap_ms(os.environ.get(CAP_MS_ENV), CAP_MS_DEFAULT)
  cap_s = cap_ms / 1000.0 if cap_ms > 0 else 0.0

  log_path = default_log_path()
  onnx_path = default_onnx_path()
  detector = SpeedSignDetector(onnx_path=onnx_path)
  logger = JsonlLogger(log_path)
  hold = LiveSignHold()
  debounce = SignDebounce()
  backend = detector.backend_name()
  weights_sha = detector.weights_sha_short()
  cloudlog.info(
    "speedsignd starting log=%s backend=%s onnx=%s sha=%s sm_hz=%.1f detect_hz=%.2f budget_ms=%.0f cap_ms=%.0f threads=%d cores=%s nice=%d no_onnx_while_controlling=1 crop_rgb=1",
    log_path, backend, onnx_path, weights_sha, SM_HZ, hz, INFER_BUDGET_MS, cap_ms, threads,
    ",".join(str(c) for c in SPEEDSIGND_CORES), SPEEDSIGND_NICE,
  )
  if detector.onnx is None:
    cloudlog.warning(
      "speedsignd: no ONNX at %s — numpy fallback will not see real roadside signs. "
      + "HUD will show NO WT. Settings → NAP → Install weights, or: "
      + "python -m scripts.nap.install_speed_sign_weights",
      onnx_path,
    )

  sm = messaging.SubMaster(
    ["gpsLocationExternal", "gpsLocation", "selfdriveState"],
    frequency=SM_HZ,
  )
  pm = messaging.PubMaster([SERVICE_NAME])
  rk = Ratekeeper(SM_HZ, print_delay_threshold=None)
  client = None
  last_connect = 0.0
  last_onnx_try = time.monotonic()
  next_detect = 0.0
  infer_ms: list[float] = []
  skip_count = 0
  last_timing_log = time.monotonic()
  started_at = time.monotonic()
  last_allow: bool | None = None
  last_paused: bool | None = None
  last_unknown_warn = 0.0
  last_empty_frame_log = 0.0
  last_empty_raw_log = 0.0
  last_infer_done = 0.0
  last_gate_log = 0.0
  last_skip_reason = "init"
  in_holdoff = False
  slot = InferSlot()

  while True:
    sm.update(0)
    now_mono = time.monotonic()
    sample = engagement_from_sm(sm, now=now_mono, started_at=started_at)
    if not sample.allow_detect:
      abandoned = slot.pause()
      if abandoned:
        cloudlog.info("speedsignd abandon in-flight ONNX (%s)", engagement_log_fields(sample))
    else:
      if should_reset_detect_after_wait(last_allow, sample.allow_detect):
        next_detect = 0.0
        in_holdoff = False
      slot.resume()
    if last_allow != sample.allow_detect or last_paused != sample.detect_paused:
      cloudlog.info(
        "speedsignd detect %s (%s)",
        "paused" if not sample.allow_detect else "running",
        engagement_log_fields(sample),
      )
      last_allow = sample.allow_detect
      last_paused = sample.detect_paused
    if sample.unknown_after_grace and now_mono - last_unknown_warn >= INFER_LOG_PERIOD_S:
      cloudlog.warning(
        "speedsignd selfdriveState unread after %.1fs; allowing throttled detect (not WAIT) %s",
        now_mono - started_at, engagement_log_fields(sample),
      )
      last_unknown_warn = now_mono
    if now_mono - last_timing_log >= INFER_LOG_PERIOD_S:
      _log_infer_timing(cloudlog, infer_ms, skip_count, hz)
      infer_ms = []
      skip_count = 0
      last_timing_log = now_mono
    # Do not load 43 MB ONNX while OP is controlling.
    if sample.allow_detect and detector.onnx is None and now_mono - last_onnx_try >= ONNX_RETRY_S:
      last_onnx_try = now_mono
      if detector.try_reload():
        cloudlog.info(
          "speedsignd: ONNX loaded after retry backend=%s onnx=%s sha=%s",
          detector.backend_name(), onnx_path, detector.weights_sha_short(),
        )
    signs: list = []
    if sample.allow_detect:
      outcome = slot.take()
      if outcome is not None:
        signs = outcome.signs
        infer_ms.append(outcome.infer_s * 1000.0)
        next_detect = max(
          next_detect,
          next_detect_mono(now_mono, outcome.infer_s, period_s, budget_s, cap_s),
        )
        raw = list(getattr(debounce, "last_raw", []))
        diag = outcome.diag if outcome.diag is not None else detector.diag_dict()
        err = f" err={outcome.error}" if outcome.error else ""
        if outcome.error and not diag.get("error"):
          diag = dict(diag)
          diag["error"] = outcome.error
        cloudlog.info(
          "speedsignd infer %.0fms %s raw=%s hud=%s jsonl=%s%s",
          outcome.infer_s * 1000.0,
          format_infer_diag(diag, allow_detect=sample.allow_detect),
          [(int(s.mph), round(float(s.conf), 2)) for s in raw],
          [(int(s.mph), round(float(s.conf), 2)) for s in signs],
          [w.get("mph") for w in outcome.written],
          err,
        )
        last_infer_done = now_mono
        last_skip_reason = "ok"
        if not raw and now_mono - last_empty_raw_log >= INFER_LOG_PERIOD_S:
          cloudlog.info(
            "speedsignd empty-raw %s",
            format_infer_diag(diag, allow_detect=sample.allow_detect),
          )
          last_empty_raw_log = now_mono
    if sample.allow_detect and now_mono - last_gate_log >= INFER_LOG_PERIOD_S:
      if last_infer_done == 0.0 or now_mono - last_infer_done >= INFER_LOG_PERIOD_S:
        connected = client is not None and bool(getattr(client, "is_connected", lambda: False)())
        cloudlog.info(
          "speedsignd waiting-infer reason=%s connected=%s busy=%s holdoff=%s onnx=%s allow=1",
          last_skip_reason, connected, slot.busy, in_holdoff, detector.onnx is not None,
        )
      last_gate_log = now_mono
    if client is None or not client.is_connected():
      last_skip_reason = detect_skip_reason(
        connected=False, onnx=detector.onnx is not None, busy=slot.busy, holdoff=in_holdoff,
      )
      if now_mono - last_connect >= 0.5:
        last_connect = now_mono
        try:
          if client is None:
            client = _connect_road_camera()
          client.connect(False)
        except Exception:
          client = None
    elif not sample.allow_detect:
      # Do not idle the ROAD client for the whole engage — first manual recv
      # after WAIT used to timeout / return nothing while modeld stayed healthy.
      drain_vision_latest(client)
    elif sample.allow_detect and now_mono >= next_detect and not slot.busy:
      in_holdoff = False
      buf = client.recv(timeout_ms=VISION_TIMEOUT_MS)
      # Engage can happen during the vision wait — re-read before ONNX.
      sm.update(0)
      sample = engagement_from_sm(sm, now=time.monotonic(), started_at=started_at)
      if not sample.allow_detect:
        slot.pause()
      else:
        nv12 = copy_nv12_detect_crop(buf) if buf is not None else None
        if buf is None:
          last_skip_reason = detect_skip_reason(
            connected=True, onnx=detector.onnx is not None, busy=False, holdoff=False,
            buf_empty=True,
          )
          now_empty = time.monotonic()
          if now_empty - last_empty_frame_log >= INFER_LOG_PERIOD_S:
            cloudlog.info("speedsignd ROAD recv empty (timeout_ms=%d)", VISION_TIMEOUT_MS)
            last_empty_frame_log = now_empty
        elif nv12 is None:
          last_skip_reason = detect_skip_reason(
            connected=True, onnx=detector.onnx is not None, busy=False, holdoff=False,
            parse_fail=True,
          )
          now_empty = time.monotonic()
          if now_empty - last_empty_frame_log >= INFER_LOG_PERIOD_S:
            stride = getattr(buf, "stride", 0)
            uv_off = getattr(buf, "uv_offset", 0)
            cloudlog.info(
              "speedsignd NV12 parse failed w=%s h=%s stride=%s uv_offset=%s",
              getattr(buf, "width", None), getattr(buf, "height", None), stride, uv_off,
            )
            last_empty_frame_log = now_empty
        lat, lon, bearing, gps_ok = gps_sample_from_sm(sm, now=time.monotonic())
        # On-road: do not run numpy-mutcd when weights are missing (no fake mph / JSONL).
        have_frame = detector.onnx is not None and nv12 is not None
        if have_frame:
          def _run(nv12=nv12, lat=lat, lon=lon, bearing=bearing, gps_ok=gps_ok):
            signs_w = detect_if_allowed(
              nv12.y, lat, lon, bearing, gps_ok, detector, logger, time.monotonic(),
              controlling=False, rgb=None, debounce=debounce, nv12=nv12,
            )
            return signs_w[0], signs_w[1], detector.diag_dict()
          if slot.start(_run):
            next_detect = time.monotonic() + period_s
            last_skip_reason = "ok"
          else:
            last_skip_reason = detect_skip_reason(
              connected=True, onnx=True, busy=True, holdoff=False,
            )
        elif detector.onnx is None:
          last_skip_reason = detect_skip_reason(
            connected=True, onnx=False, busy=slot.busy, holdoff=False,
          )
    elif sample.allow_detect and (slot.busy or (next_detect > 0 and now_mono < next_detect)):
      last_skip_reason = detect_skip_reason(
        connected=True, onnx=detector.onnx is not None, busy=slot.busy, holdoff=True,
      )
      if not in_holdoff:
        skip_count += 1
        in_holdoff = True
      if slot.busy:
        try:
          os.sched_yield()
        except Exception:
          pass
    _publish_live(
      pm, hold, signs, now_mono, messaging,
      detector.weights_missing(), detect_paused=sample.detect_paused,
    )
    rk.keep_time()
    reset_ratekeeper_if_behind(rk, time.monotonic())


if __name__ == "__main__":
  main()
