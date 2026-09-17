#!/usr/bin/env python3
"""Always-on NAP companion Dash for Justin's tree.

Adapted from Philip's NAP-Dash server_v21 UI shell (live cluster, dashcam
viewer/export, phone-friendly page). Settings writes go only through
`settings.py` → Params / SL-FAI helpers. Hypermile is nap-dev-only.

Not included (do not port):
- cruise trim / speed-offset injector
- City Turns / Tap LC / Corner Assist / Lane Centering
- Navigation write / phone nav remote
- Cloudflare / ngrok / Tailscale Funnel
- Hypermile / NAPHypermile* (not on nap-release)
"""
from __future__ import annotations

import datetime
import json
import os
import subprocess
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

try:
  from openpilot.common.swaglog import cloudlog
except Exception:  # pragma: no cover - unit tests without zmq
  class _PrintLog:
    def info(self, msg): print(msg)
    def warning(self, msg): print(msg)
  cloudlog = _PrintLog()

from openpilot.selfdrive.nap_dash.settings import (
  SettingError,
  read_settings,
  setting_catalog,
  write_setting,
)

HOST = os.environ.get("NAP_DASH_HOST", "0.0.0.0")
PORT = int(os.environ.get("NAP_DASH_PORT", "7070"))
REALDATA = os.environ.get("NAP_DASH_REALDATA", "/data/media/0/realdata")
SHM_DIR = os.environ.get("NAP_DASH_SHM", "/dev/shm")
DASHBOARD_PATH = Path(__file__).resolve().parent / "dashboard.html"

# 10 Hz live loop. modelV2 is for the cluster path; can is Tesla BMS.
TELEMETRY_SVCS = (
  "carState", "selfdriveState", "controlsState",
  "radarState", "deviceState", "modelV2", "can",
)
MPH_PER_MPS = 2.2369362921

STATE: dict = {
  "ts": 0,
  "car": {},
  "drive": {},
  "plan": {},
  "lead1": {},
  "settings": {},
  "health": {},
  "engagement": {},
  "bms": {
    "bricks": [0] * 96, "temps_dict": {}, "pack_v": 0, "pack_i": 0,
    "ui_soc": 0, "rated_range": 0, "nom_full": 0.0, "nom_rem": 0.0,
    "buffer": 0.0, "max_discharge": 0, "max_regen": 0,
  },
}
LOCK = threading.Lock()
STOP = threading.Event()
PARAMS = None
PM_LOCK = threading.Lock()


def _params():
  global PARAMS
  if PARAMS is None:
    from openpilot.common.params import Params
    PARAMS = Params()
  return PARAMS


def num(v, default=0.0):
  try:
    if hasattr(v, "raw"):
      v = v.raw
    x = float(v)
    return x if x == x and abs(x) != float("inf") else default
  except Exception:
    return default


def safe_int(v, default=0):
  try:
    return int(v.raw if hasattr(v, "raw") else v)
  except Exception:
    return default


def safe_attr(obj, attr, default=0):
  if obj is None:
    return default
  try:
    return getattr(obj, attr, default)
  except Exception:
    return default


def lead_dict(lead):
  out = {}
  if lead is None:
    return out
  for key in ("status", "dRel", "yRel", "vRel", "vLead", "aLeadK", "aLeadTau", "modelProb", "radar", "fcw"):
    try:
      value = getattr(lead, key)
      out[key] = value if isinstance(value, bool) else num(value)
    except Exception:
      pass
  return out


def locked_settings():
  with PM_LOCK:
    return read_settings(_params())


def locked_write(name, value):
  with PM_LOCK:
    return write_setting(_params(), name, value)


def state_snapshot():
  with LOCK:
    keep = (
      "ui_soc", "rated_range", "pack_v", "pack_i", "nom_full", "nom_rem",
      "buffer", "display_soc", "usable_full", "usable_rem", "max_discharge",
      "max_regen", "bricks", "temps_dict", "min_v", "max_v", "min_t", "max_t",
      "expected_rem", "ideal_rem", "charge_complete",
    )
    bms = {key: STATE["bms"].get(key, 0) for key in keep}
    return {
      "ts": STATE["ts"],
      "car": json.loads(json.dumps(STATE["car"])),
      "drive": json.loads(json.dumps(STATE["drive"])),
      "plan": json.loads(json.dumps(STATE["plan"])),
      "lead1": json.loads(json.dumps(STATE["lead1"])),
      "settings": json.loads(json.dumps(STATE["settings"])),
      "health": json.loads(json.dumps(STATE["health"])),
      "engagement": json.loads(json.dumps(STATE["engagement"])),
      "bms": json.loads(json.dumps(bms)),
    }


def _path_data(mdl, radar, v_ego):
  path = {"ego": [], "lanes": [], "edges": [], "leads": []}
  if mdl is not None:
    try:
      xs = getattr(mdl.position, "x", [])
      ys = getattr(mdl.position, "y", [])
      for i in range(0, min(len(xs), len(ys), 30), 2):
        path["ego"].append([float(xs[i]), float(ys[i])])
    except Exception:
      pass
    try:
      if hasattr(mdl, "laneLines") and hasattr(mdl, "laneLineProbs"):
        for idx, line in enumerate(mdl.laneLines):
          prob = float(mdl.laneLineProbs[idx])
          if prob > 0.3:
            pts = []
            l_xs = getattr(line, "x", [])
            l_ys = getattr(line, "y", [])
            for i in range(0, min(len(l_xs), len(l_ys), 30), 3):
              pts.append([float(l_xs[i]), float(l_ys[i])])
            path["lanes"].append({"pts": pts, "prob": prob, "idx": idx})
    except Exception:
      pass
    try:
      if hasattr(mdl, "roadEdges") and hasattr(mdl, "roadEdgeStds"):
        for idx, edge in enumerate(mdl.roadEdges):
          std = float(mdl.roadEdgeStds[idx])
          pts = []
          r_xs = getattr(edge, "x", [])
          r_ys = getattr(edge, "y", [])
          for i in range(0, min(len(r_xs), len(r_ys), 30), 3):
            pts.append([float(r_xs[i]), float(r_ys[i])])
          path["edges"].append({"pts": pts, "std": std})
    except Exception:
      pass
    try:
      if hasattr(mdl, "leadsV3"):
        for lead in mdl.leadsV3:
          prob = float(getattr(lead, "prob", 0))
          if prob > 0.1:
            l_xs = getattr(lead, "x", [0])
            l_ys = getattr(lead, "y", [0])
            l_vs = getattr(lead, "v", [0])
            path["leads"].append({
              "x": float(l_xs[0]), "y": float(l_ys[0]),
              "v": float(l_vs[0]), "prob": prob,
            })
    except Exception:
      pass
  if not path["leads"] and radar is not None:
    for name in ("leadOne", "leadTwo"):
      ld = getattr(radar, name, None)
      if ld and getattr(ld, "status", False):
        v_lead = float(getattr(ld, "vLead", getattr(ld, "vRel", 0) + v_ego))
        path["leads"].append({
          "x": float(getattr(ld, "dRel", 0)),
          "y": float(getattr(ld, "yRel", 0)),
          "v": v_lead,
          "prob": 1.0,
        })
  return path


def _decode_bms(sm, svcs):
  if "can" not in svcs or not sm.updated.get("can", False):
    return
  STATE["bms"].setdefault("temps_dict", {})
  for msg in sm["can"]:
    addr = msg.address
    data = msg.dat
    if addr in (0x132, 0x102) and len(data) >= 4:
      STATE["bms"]["pack_v"] = (data[0] | (data[1] << 8)) * 0.01
      raw_i = data[2] | (data[3] << 8)
      if raw_i >= 32768:
        raw_i -= 65536
      STATE["bms"]["pack_i"] = raw_i * 0.1
    elif addr == 0x302 and len(data) >= 3:
      STATE["bms"]["ui_soc"] = ((data[1] >> 2) | ((data[2] & 0x0F) << 6)) * 0.1
    elif addr == 0x338 and len(data) >= 2:
      STATE["bms"]["rated_range"] = data[0] | (data[1] << 8)
    elif addr == 0x6F2 and len(data) >= 8:
      mux = data[0]
      if mux <= 31 and data[1:8] != b"\xff" * 7:
        bits = int.from_bytes(bytes(data[1:8]), "little")
        vals = [(bits >> (14 * k)) & 0x3FFF for k in range(4)]
        if mux < 24:
          idx = mux * 4
          for k, raw in enumerate(vals):
            if raw in (0, 0x3FFF):
              continue
            STATE["bms"]["bricks"][idx + k] = raw * 0.305175
        else:
          base = (mux - 24) * 4
          for k, raw in enumerate(vals):
            if raw == 0x3FFF:
              continue
            if raw & 0x2000:
              raw -= 0x4000
            temp = raw * 0.0122
            if -50 < temp < 120:
              STATE["bms"]["temps_dict"][f"{base + k}"] = temp
    elif addr == 0x382 and len(data) >= 8:
      bits = int.from_bytes(bytes(data[:8]), "little")
      nom_full = ((bits >> 0) & 0x7FF) * 0.1
      nom_rem = ((bits >> 11) & 0x7FF) * 0.1
      expected_rem = ((bits >> 22) & 0x7FF) * 0.1
      ideal_rem = ((bits >> 33) & 0x7FF) * 0.1
      charge_complete = ((bits >> 44) & 0x7FF) * 0.1
      buffer = ((bits >> 55) & 0x1FF) * 0.1
      plausible = (
        20.0 <= nom_full <= 120.0
        and 0.0 <= nom_rem <= nom_full + 2.0
        and 0.0 <= expected_rem <= nom_full + 10.0
        and 0.0 <= ideal_rem <= nom_full + 10.0
        and 0.0 <= buffer < nom_full
        and buffer <= 20.0
      )
      if plausible:
        usable_full = max(0.0, nom_full - buffer)
        usable_rem = max(0.0, min(usable_full, nom_rem - buffer))
        display_soc = usable_rem / usable_full * 100.0 if usable_full > 0.0 else 0.0
        STATE["bms"]["nom_full"] = nom_full
        STATE["bms"]["nom_rem"] = nom_rem
        STATE["bms"]["expected_rem"] = expected_rem
        STATE["bms"]["ideal_rem"] = ideal_rem
        STATE["bms"]["charge_complete"] = charge_complete
        STATE["bms"]["buffer"] = buffer
        STATE["bms"]["usable_full"] = usable_full
        STATE["bms"]["usable_rem"] = usable_rem
        STATE["bms"]["display_soc"] = max(0.0, min(100.0, display_soc))
    elif addr == 0x252 and len(data) >= 4:
      STATE["bms"]["max_regen"] = (data[0] | (data[1] << 8)) * 0.01
      STATE["bms"]["max_discharge"] = (data[2] | (data[3] << 8)) * 0.01

  valid_bricks = [v for v in STATE["bms"]["bricks"] if v > 2000]
  if valid_bricks:
    STATE["bms"]["min_v"] = min(valid_bricks)
    STATE["bms"]["max_v"] = max(valid_bricks)
  valid_temps = [t for t in STATE["bms"]["temps_dict"].values() if -40 < t < 120]
  if valid_temps:
    STATE["bms"]["min_t"] = min(valid_temps)
    STATE["bms"]["max_t"] = max(valid_temps)


def telemetry():
  import cereal.messaging as messaging

  svcs = list(TELEMETRY_SVCS)
  sm = None
  while svcs and not STOP.is_set():
    try:
      sm = messaging.SubMaster(svcs)
      break
    except Exception as exc:
      bad = str(exc.args[0]) if exc.args else str(exc)
      removed = False
      for name in list(svcs):
        if name in bad:
          svcs.remove(name)
          removed = True
      if not removed:
        cloudlog.warning(f"nap_dash telemetry unavailable: {exc}")
        return

  tick = 0
  engage = {"manual": 0.0, "both": 0.0}
  last = time.monotonic()
  while not STOP.is_set():
    try:
      sm.update(100)
      now = time.monotonic()
      dt = now - last
      last = now
      if dt > 1.0 or dt < 0:
        dt = 0

      cs = sm["carState"] if "carState" in svcs else None
      sd = sm["selfdriveState"] if "selfdriveState" in svcs else None
      ctl = sm["controlsState"] if "controlsState" in svcs else None
      radar = sm["radarState"] if "radarState" in svcs else None
      ds = sm["deviceState"] if "deviceState" in svcs else None
      mdl = sm["modelV2"] if "modelV2" in svcs else None

      if tick % 10 == 0:
        current = locked_settings()
        with LOCK:
          STATE["settings"] = current

      enabled = safe_attr(sd, "enabled", safe_attr(ctl, "enabled", False))
      active = safe_attr(sd, "active", safe_attr(ctl, "active", False))
      v_cruise = num(safe_attr(cs, "vCruise", 0))
      v_ego = num(safe_attr(cs, "vEgo", 0))
      dist_mi = (v_ego * dt) * 0.000621371
      gear_str = str(safe_attr(cs, "gearShifter", "")).lower()
      in_drive = ("drive" in gear_str) or (v_ego > 0.5)
      if active:
        engage["both"] += dist_mi
      elif in_drive:
        engage["manual"] += dist_mi

      try:
        uptime = float(open("/proc/uptime", encoding="utf-8").read().split()[0])
      except Exception:
        uptime = 0.0
      try:
        st = os.statvfs("/data")
        storage_total = st.f_frsize * st.f_blocks
        storage_free = st.f_frsize * st.f_bavail
        storage_used = max(0, storage_total - storage_free)
        storage_pct = (storage_used / storage_total * 100.0) if storage_total else 0.0
      except Exception:
        storage_total = storage_free = storage_used = 0
        storage_pct = 0.0

      with LOCK:
        STATE.update({
          "ts": datetime.datetime.now(datetime.UTC).timestamp(),
          "health": {
            "temp": max(safe_attr(ds, "cpuTempC", [0]) or [0]),
            "battery": num(safe_attr(ds, "batteryPercent", 0)),
            "uptime": uptime,
            "storageUsed": storage_used,
            "storageTotal": storage_total,
            "storagePct": storage_pct,
          },
          "car": {
            "vEgo": v_ego,
            "aEgo": num(safe_attr(cs, "aEgo", 0)),
            "steer": num(safe_attr(cs, "steeringAngleDeg", 0)),
            "vCruise": v_cruise,
            "brakePressed": bool(safe_attr(cs, "brakePressed", False)),
            "gasPressed": bool(safe_attr(cs, "gasPressed", False)),
            "leftBlinker": bool(safe_attr(cs, "leftBlinker", False)),
            "rightBlinker": bool(safe_attr(cs, "rightBlinker", False)),
          },
          "drive": {"active": bool(active), "enabled": bool(enabled)},
          "plan": _path_data(mdl, radar, v_ego),
          "lead1": lead_dict(safe_attr(radar, "leadOne", None)),
          "engagement": {"manual": engage["manual"], "both": engage["both"]},
        })
        try:
          _decode_bms(sm, svcs)
        except Exception:
          pass
      tick += 1
    except Exception:
      traceback.print_exc()
    time.sleep(0.05)


def get_routes():
  routes_dict: dict[str, list[int]] = {}
  route_times: dict[tuple[str, int], float] = {}
  if not os.path.exists(REALDATA):
    return []
  try:
    for name in os.listdir(REALDATA):
      path = os.path.join(REALDATA, name)
      if os.path.isdir(path) and "--" in name:
        route_name, _, seg = name.rpartition("--")
        if seg.isdigit():
          routes_dict.setdefault(route_name, []).append(int(seg))
          route_times[(route_name, int(seg))] = os.path.getmtime(path)
  except Exception:
    pass
  routes = []
  for route, segs in routes_dict.items():
    routes.append({
      "name": route,
      "segs": sorted(segs),
      "times": {str(seg): route_times.get((route, seg), 0) for seg in segs},
    })
  routes.sort(key=lambda x: x["name"], reverse=True)
  return routes


def _safe_segment(route_seg: str) -> str | None:
  if ".." in route_seg or "/" in route_seg or "\\" in route_seg:
    return None
  return route_seg


def find_log_path(seg_dir: str) -> str | None:
  for name in ("rlog.zst", "rlog.bz2", "qlog.zst", "qlog.bz2"):
    path = os.path.join(seg_dir, name)
    if os.path.exists(path):
      return path
  return None


def parse_telemetry_timeline(log_path: str):
  from openpilot.tools.lib.logreader import LogReader

  lr = LogReader(log_path)
  timeline, engagement = [], []
  v_ego = steer = lead_d = 0.0
  gas = brake = left_b = right_b = engaged = False
  t0 = None
  for msg in lr:
    try:
      which = msg.which()
      if which not in ("carState", "radarState", "selfdriveState", "controlsState"):
        continue
      t = msg.logMonoTime / 1e9
      if t0 is None:
        t0 = t
      rel_t = t - t0
      if rel_t < 0 or rel_t > 61.0:
        continue
      if which == "carState":
        cs = msg.carState
        v_ego = getattr(cs, "vEgo", 0)
        steer = getattr(cs, "steeringAngleDeg", 0)
        gas = getattr(cs, "gasPressed", False)
        brake = getattr(cs, "brakePressed", False)
        left_b = getattr(cs, "leftBlinker", False)
        right_b = getattr(cs, "rightBlinker", False)
      elif which == "radarState":
        lead = getattr(msg.radarState, "leadOne", None)
        lead_d = getattr(lead, "dRel", 0) if lead and getattr(lead, "status", False) else 0.0
      elif which == "selfdriveState":
        engaged = bool(getattr(msg.selfdriveState, "active", False))
      elif which == "controlsState":
        engaged = bool(getattr(msg.controlsState, "active", engaged))
      idx = int(rel_t * 10)
      while len(timeline) <= idx:
        timeline.append([0.0, 0.0, 0, 0, 0.0, 0, 0, 0])
        engagement.append(False)
      timeline[idx] = [
        float(v_ego) * MPH_PER_MPS, float(steer),
        1 if gas else 0, 1 if brake else 0, float(lead_d or 0),
        1 if left_b else 0, 1 if right_b else 0, 1 if engaged else 0,
      ]
      engagement[idx] = engaged
    except Exception:
      continue
  return timeline, engagement, t0


def get_mp4_path(route_seg: str, cam_type: str = "qcamera") -> str | None:
  if _safe_segment(route_seg) is None:
    return None
  if cam_type not in ("qcamera", "fcamera", "dcamera"):
    return None
  base_path = os.path.join(REALDATA, route_seg, cam_type)
  cam_file = base_path + ".hevc"
  if not os.path.exists(cam_file):
    cam_file = base_path + ".ts"
  if not os.path.exists(cam_file):
    return None
  os.makedirs(SHM_DIR, exist_ok=True)
  tmp_path = os.path.join(SHM_DIR, f"vid_{route_seg}_{cam_type}.mp4")
  if (
    not os.path.exists(tmp_path)
    or os.path.getsize(tmp_path) < 1024
    or os.path.getmtime(tmp_path) < os.path.getmtime(cam_file)
  ):
    temporary = tmp_path + ".part"
    try:
      proc = subprocess.run(
        ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error", "-i", cam_file,
         "-c", "copy", "-movflags", "+faststart", "-f", "mp4", temporary],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=120, check=False,
      )
      if proc.returncode == 0 and os.path.exists(temporary) and os.path.getsize(temporary) >= 1024:
        os.replace(temporary, tmp_path)
    except Exception:
      pass
    finally:
      try:
        if os.path.exists(temporary):
          os.unlink(temporary)
      except Exception:
        pass
  return tmp_path if os.path.exists(tmp_path) and os.path.getsize(tmp_path) >= 1024 else None


def serve_file_with_range(handler, path, content_type="video/mp4", attachment=None):
  try:
    file_size = os.path.getsize(path)
  except OSError:
    handler.send_error(404)
    return
  if file_size <= 0:
    handler.send_error(404)
    return
  range_header = handler.headers.get("Range")
  start, end, status = 0, file_size - 1, 200
  if range_header:
    try:
      unit, _, rng = range_header.partition("=")
      if unit.strip().lower() != "bytes":
        raise ValueError()
      first, _, last = rng.partition("-")
      if first.strip():
        start = int(first)
        end = int(last) if last.strip() else file_size - 1
      else:
        suffix = int(last)
        if suffix <= 0:
          raise ValueError()
        start = max(0, file_size - suffix)
        end = file_size - 1
      if start < 0 or start >= file_size or end < start:
        raise ValueError()
      end = min(end, file_size - 1)
      status = 206
    except Exception:
      handler.send_response(416)
      handler.send_header("Content-Range", f"bytes */{file_size}")
      handler.send_header("Accept-Ranges", "bytes")
      handler.end_headers()
      return
  length = end - start + 1
  handler.send_response(status)
  handler.send_header("Content-Type", content_type)
  handler.send_header("Accept-Ranges", "bytes")
  handler.send_header("Content-Length", str(length))
  handler.send_header("Cache-Control", "no-cache")
  if status == 206:
    handler.send_header("Content-Range", f"bytes {start}-{end}/{file_size}")
  if attachment:
    safe = os.path.basename(attachment).replace('"', "")
    handler.send_header("Content-Disposition", f'attachment; filename="{safe}"')
  handler.end_headers()
  if getattr(handler, "command", "GET") == "HEAD":
    return
  try:
    with open(path, "rb") as handle:
      handle.seek(start)
      remaining = length
      while remaining > 0:
        chunk = handle.read(min(262144, remaining))
        if not chunk:
          break
        handler.wfile.write(chunk)
        remaining -= len(chunk)
  except (BrokenPipeError, ConnectionResetError):
    pass


def dashboard_html() -> bytes:
  return DASHBOARD_PATH.read_bytes()


class Handler(BaseHTTPRequestHandler):
  def log_message(self, *_args):
    pass

  def send_json(self, obj, status=200):
    body = json.dumps(obj, separators=(",", ":")).encode()
    self.send_response(status)
    self.send_header("Content-Type", "application/json")
    self.send_header("Cache-Control", "no-cache, no-store")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    self.wfile.write(body)

  def _send_html(self):
    body = dashboard_html()
    self.send_response(200)
    self.send_header("Content-Type", "text/html; charset=utf-8")
    self.send_header("Cache-Control", "no-cache, no-store")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    self.wfile.write(body)

  def do_HEAD(self):
    path = urlparse(self.path).path
    if path.startswith("/stream/"):
      route_seg = unquote(path.split("/stream/", 1)[1])
      cam = parse_qs(urlparse(self.path).query).get("cam", ["qcamera"])[0]
      vid = get_mp4_path(route_seg, cam)
      if vid:
        return serve_file_with_range(self, vid, "video/mp4")
      return self.send_error(404, "Video not found")
    self.send_error(404)

  def do_GET(self):
    parsed = urlparse(self.path)
    path = parsed.path
    if path in ("/", "/index.html", "/phone", "/phone/"):
      return self._send_html()
    if path == "/api/state":
      return self.send_json(state_snapshot())
    if path == "/api/settings":
      return self.send_json({"settings": locked_settings(), "catalog": setting_catalog()})
    if path == "/api/routes":
      return self.send_json(get_routes())
    if path.startswith("/api/log/"):
      route_seg = unquote(path.split("/api/log/", 1)[1])
      if _safe_segment(route_seg) is None:
        return self.send_json({"error": "invalid segment"}, 400)
      seg_dir = os.path.join(REALDATA, route_seg)
      log_path = find_log_path(seg_dir)
      if not log_path:
        return self.send_json({"error": "not found"}, 404)
      try:
        timeline, _eng, t0 = parse_telemetry_timeline(log_path)
        return self.send_json({"data": timeline, "start": t0})
      except Exception as exc:
        return self.send_json({"error": str(exc)}, 500)
    if path.startswith("/stream/"):
      route_seg = unquote(path.split("/stream/", 1)[1])
      cam = parse_qs(parsed.query).get("cam", ["qcamera"])[0]
      vid = get_mp4_path(route_seg, cam)
      if vid:
        return serve_file_with_range(self, vid, "video/mp4")
      return self.send_error(404, "Video not found")
    if path.startswith("/export/"):
      route_seg = unquote(path.split("/export/", 1)[1])
      cam = parse_qs(parsed.query).get("cam", ["qcamera"])[0]
      return self.handle_export(route_seg, cam)
    return self.send_json({"error": "not found"}, 404)

  def handle_export(self, route_seg, cam_type):
    if cam_type not in ("qcamera", "fcamera", "dcamera"):
      return self.send_json({"error": "invalid camera"}, 400)
    if _safe_segment(route_seg) is None:
      return self.send_json({"error": "invalid segment"}, 400)
    src_path = get_mp4_path(route_seg, cam_type)
    if not src_path:
      return self.send_json({"error": "no video"}, 404)
    log_path = find_log_path(os.path.join(REALDATA, route_seg))
    if not log_path:
      return self.send_json({"error": "no telemetry log"}, 404)
    os.makedirs(SHM_DIR, exist_ok=True)
    out_path = os.path.join(SHM_DIR, f"exp_nap_{route_seg}_{cam_type}.mp4")
    work_path = out_path + ".part"
    ass_path = os.path.join(SHM_DIR, f"hud_{route_seg}_{cam_type}.ass")
    try:
      newest_input = max(os.path.getmtime(src_path), os.path.getmtime(log_path))
      if not os.path.exists(out_path) or os.path.getsize(out_path) < 1024 or os.path.getmtime(out_path) < newest_input:
        timeline, engagement, _t0 = parse_telemetry_timeline(log_path)
        if not timeline:
          return self.send_json({"error": "no telemetry"}, 422)

        def ass_time(sec):
          sec = max(0.0, float(sec))
          whole = int(sec)
          cs = int(round((sec - whole) * 100))
          if cs >= 100:
            whole += 1
            cs = 0
          return f"{whole // 3600}:{(whole % 3600) // 60:02d}:{whole % 60:02d}.{cs:02d}"

        def ass_escape(text):
          return str(text).replace("\\", "\\\\").replace("{", "\\{").replace("}", "\\}")

        ass_format = ",".join((
          "Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour",
          " OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX",
          " ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment",
          " MarginL, MarginR, MarginV, Encoding",
        ))
        styles = f"""[Script Info]
ScriptType: v4.00+
PlayResX: 1928
PlayResY: 1208
ScaledBorderAndShadow: yes

[V4+ Styles]
{ass_format}
Style: State,Arial,34,&H00FFFFFF,&H00000000,&H99000000,&H99000000,1,0,0,0,100,100,0,0,1,2,2,2,30,30,30,1
Style: Speed,Arial,72,&H00FFFFFF,&H00000000,&H99000000,&H99000000,1,0,0,0,100,100,0,0,1,3,3,2,30,30,70,1
Style: Lead,Arial,36,&H00FFB656,&H00000000,&H99000000,&H99000000,1,0,0,0,100,100,0,0,1,2,2,8,30,30,50,1
Style: Steer,Arial,30,&H00FFFFFF,&H00000000,&H99000000,&H99000000,1,0,0,0,100,100,0,0,1,2,2,1,30,30,55,1
Style: Brake,Arial,34,&H003B3BFF,&H00000000,&H99000000,&H99000000,1,0,0,0,100,100,0,0,1,2,2,3,30,30,60,1
Style: Gas,Arial,34,&H0034C759,&H00000000,&H99000000,&H99000000,1,0,0,0,100,100,0,0,1,2,2,3,30,30,120,1
Style: Arrow,Arial,44,&H0034C759,&H00000000,&H99000000,&H99000000,1,0,0,0,100,100,0,0,1,2,2,5,30,30,55,1

[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
"""
        lines = []
        for i, frame in enumerate(timeline[:600]):
          start, end = ass_time(i / 10), ass_time((i + 1) / 10)
          state = "ENGAGED" if (i < len(engagement) and engagement[i]) else "STANDBY"
          lines.append(f"Dialogue: 0,{start},{end},State,,0,0,0,,{ass_escape(state)}")
          lines.append(f"Dialogue: 1,{start},{end},Speed,,0,0,0,,{ass_escape(f'{frame[0]:.0f} MPH')}")
          lines.append(f"Dialogue: 1,{start},{end},Lead,,0,0,0,,{ass_escape(f'LEAD {frame[4]:.1f} M' if frame[4] > 0 else 'LEAD --')}")
          lines.append(f"Dialogue: 1,{start},{end},Steer,,0,0,0,,{ass_escape(f'STR {frame[1]:+.1f}°')}")
          if frame[3]:
            lines.append(f"Dialogue: 2,{start},{end},Brake,,0,0,0,,BRAKE")
          if frame[2]:
            lines.append(f"Dialogue: 2,{start},{end},Gas,,0,0,0,,GAS")
          if frame[5]:
            lines.append(f"Dialogue: 2,{start},{end},Arrow,,0,0,0,,◀")
          if frame[6]:
            lines.append(f"Dialogue: 2,{start},{end},Arrow,,0,0,0,,▶")
        Path(ass_path).write_text(styles + "\n".join(lines) + "\n", encoding="utf-8")
        ass_name = os.path.basename(ass_path)
        cmd = [
          "ffmpeg", "-y", "-hide_banner", "-loglevel", "error", "-i", src_path,
          "-vf", f"ass=filename={ass_name}", "-map", "0:v:0", "-map", "0:a?",
          "-c:v", "libx264", "-preset", "ultrafast", "-crf", "24", "-pix_fmt", "yuv420p",
          "-c:a", "aac", "-b:a", "128k", "-threads", "2", "-movflags", "+faststart",
          "-f", "mp4", work_path,
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600, cwd=SHM_DIR, check=False)
        if proc.returncode != 0 or not os.path.exists(work_path) or os.path.getsize(work_path) < 1024:
          return self.send_json({"error": "ffmpeg export failed", "detail": (proc.stderr or "")[-1200:]}, 500)
        os.replace(work_path, out_path)
      serve_file_with_range(self, out_path, "video/mp4", f"NAP_Clip_{route_seg}_{cam_type}.mp4")
    except subprocess.TimeoutExpired:
      self.send_json({"error": "export timed out"}, 504)
    except Exception as exc:
      self.send_json({"error": f"export failed: {exc}"}, 500)
    finally:
      for leftover in (ass_path, work_path):
        try:
          if os.path.exists(leftover):
            os.unlink(leftover)
        except Exception:
          pass

  def do_POST(self):
    path = urlparse(self.path).path
    try:
      n = int(self.headers.get("Content-Length", "0"))
      if n < 0 or n > 16 * 1024:
        return self.send_json({"error": "payload too large"}, 413)
      payload = json.loads(self.rfile.read(n).decode() or "{}")
      if path != "/api/set":
        return self.send_json({"error": "not found"}, 404)
      settings = locked_write(str(payload.get("name") or payload.get("param") or ""), payload.get("value"))
      with LOCK:
        STATE["settings"] = settings
      snap = state_snapshot()
      snap["settings"] = settings
      return self.send_json(snap)
    except SettingError as exc:
      return self.send_json({"error": str(exc)}, 400)
    except Exception as exc:
      return self.send_json({"error": str(exc)}, 400)


class DashHTTPServer(ThreadingHTTPServer):
  allow_reuse_address = True
  daemon_threads = True


def bind_http_server(host=HOST, port=PORT):
  """Bind :7070. Returns None on failure so the process can stay alive."""
  try:
    return DashHTTPServer((host, port), Handler)
  except OSError as exc:
    cloudlog.warning(f"nap_dash failed to bind {host}:{port}: {exc}")
    return None


def idle_until_stop(stop_event=None, sleeper=time.sleep):
  """Keep the managed process alive so optional Dash cannot block engage."""
  stop_event = STOP if stop_event is None else stop_event
  while not stop_event.is_set():
    sleeper(1.0)


def main():
  cloudlog.info(f"nap_dash listening on {HOST}:{PORT} (comma hotspot / LAN only)")
  try:
    STATE["settings"] = locked_settings()
  except Exception:
    traceback.print_exc()
  threading.Thread(target=telemetry, name="nap-dash-telemetry", daemon=True).start()
  server = bind_http_server(HOST, PORT)
  if server is None:
    idle_until_stop()
    return
  try:
    server.serve_forever()
  except KeyboardInterrupt:
    pass
  except Exception:
    traceback.print_exc()
    idle_until_stop()
  finally:
    STOP.set()
    try:
      server.server_close()
    except Exception:
      pass


if __name__ == "__main__":
  main()
