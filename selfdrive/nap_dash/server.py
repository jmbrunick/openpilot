#!/usr/bin/env python3
"""Comma web UI. Params and software only.

HTTP on port 80 when the process can bind it, otherwise 7070.
This process does not open cereal sockets and does not run a live
telemetry loop. It cannot engage openpilot.

A crash, bind failure, or preimport error must not block engage
(the manager process is optional).
"""
from __future__ import annotations

import json
import os
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

try:
  from openpilot.common.swaglog import cloudlog
except Exception:  # pragma: no cover - unit tests without zmq
  class _PrintLog:
    def info(self, msg):
      print(msg)

    def warning(self, msg):
      print(msg)

    def exception(self, msg):
      print(msg)
  cloudlog = _PrintLog()

from openpilot.selfdrive.nap_dash.force_offroad_api import AckError, handle_ack, read_ack
from openpilot.selfdrive.nap_dash.review_api import read_flags
from openpilot.selfdrive.nap_dash.speed_sign_api import SignError, handle_sign, read_sign
from openpilot.selfdrive.nap_dash.talk_api import TALK, parse_active
from openpilot.selfdrive.nap_dash.settings import SettingError
from openpilot.selfdrive.nap_dash.system_api import (
  SoftwareError,
  handle_software,
  read_software,
)
from openpilot.selfdrive.nap_dash.ui_api import (
  get_manifest,
  network_status,
  public_manifest,
  read_settings,
  write_setting,
)

HOST = os.environ.get("NAP_DASH_HOST", "0.0.0.0")
DASHBOARD_PATH = Path(__file__).resolve().parent / "dashboard.html"

PARAMS = None
PM_LOCK = threading.Lock()
STOP = threading.Event()


def listen_ports():
  """Port 80 is the MCU URL. 7070 is the fallback if 80 cannot be bound."""
  raw = os.environ.get("NAP_DASH_PORT", "").strip()
  if raw:
    return [int(raw)]
  return [80, 7070]


def _params():
  global PARAMS
  if PARAMS is None:
    from openpilot.common.params import Params
    PARAMS = Params()
  return PARAMS


def locked_settings():
  with PM_LOCK:
    return read_settings(_params())


def locked_write(name, value, confirm=False):
  with PM_LOCK:
    return write_setting(_params(), name, value, confirm=confirm)


def bookmark_status(params):
  """{"recording": bool, "left": seconds}. feedbackd owns NAPBookmarkRecording."""
  try:
    left = params.get("NAPBookmarkRecording")
  except Exception:
    left = None
  if left is None:
    return {"recording": False, "left": 0}
  return {"recording": True, "left": int(left)}


def bookmark_tap(params):
  """Same as the comma sidebar Bookmark tap: feedbackd bookmarks + starts/stops the voice note.
  Only onroad (feedbackd/micd/loggerd do not run parked)."""
  if not params.get_bool("IsOnroad"):
    # Nothing records parked; a queued request would fire at the next drive start.
    out = bookmark_status(params)
    out["ok"] = False
    out["reason"] = "offroad"
    return out
  params.put("NAPBookmarkRequest", int(time.monotonic()))
  out = bookmark_status(params)
  out["ok"] = True
  return out


def state_snapshot():
  """Minimal JSON so an old page does not 500. No live car data."""
  return {
    "lite": True,
    "ts": 0,
    "car": {},
    "drive": {},
    "plan": {},
    "lead1": {},
    "settings": {},
    "health": {},
    "engagement": {},
    "bms": {},
  }


def ping_payload(now=None):
  """Clock-only liveness for the MCU link dot.

  No Params read and no cereal. A fresh ``ts`` (unix milliseconds) is the
  data timestamp the page uses to notice a cached or stuck response.
  """
  if now is None:
    now = time.time()  # noqa: TID251  unix ms the MCU shows as last update
  return {"ok": True, "ts": int(round(float(now) * 1000.0))}


class SignWatch:
  """Pushes NAPSpeedSignPrompt changes to the MCU page.

  The comma UI process writes the prompt (mcu_prompt.sync_hud_prompt), so
  nap_dash cannot be called directly. One lazy thread stats the param file
  every WATCH_S and re-reads it when the file changes (or every RECHECK_S,
  so a note that times out still clears). Each real change bumps ``version``
  and wakes every SSE stream and long-poll waiting on it.
  """
  WATCH_S = 0.05
  RECHECK_S = 1.0
  MAX_STREAMS = 4
  MAX_WAITERS = 8

  def __init__(self, read=None, params=None, clock=time.monotonic):
    self._read = read
    self._params = params
    self._clock = clock
    self.cond = threading.Condition()
    self.version = 0
    self.state = None
    self._sig = None
    self._last_read = -1e9
    self._thread = None
    self._streams = []
    self._waiters = 0

  def _params_obj(self):
    return self._params() if callable(self._params) else _params()

  def _read_state(self):
    if self._read is not None:
      return self._read()
    return read_sign(self._params_obj())

  def _signature(self):
    try:
      base = self._params_obj().get_param_path()
      st = os.stat(os.path.join(base, "NAPSpeedSignPrompt"))
      return (st.st_mtime_ns, st.st_size, st.st_ino)
    except Exception:
      return None

  def check(self, force=False):
    """Re-read when the file changed, RECHECK_S passed, or forced. True when the state changed."""
    sig = self._signature()
    now = self._clock()
    if not force and sig is not None and sig == self._sig and now - self._last_read < self.RECHECK_S:
      return False
    try:
      state = self._read_state()
    except Exception:
      return False
    self._sig = sig
    self._last_read = now
    with self.cond:
      if state == self.state:
        return False
      self.state = state
      self.version += 1
      self.cond.notify_all()
    return True

  def snapshot(self):
    with self.cond:
      return self.version, self.state

  def payload(self):
    version, state = self.snapshot()
    out = dict(state or {"pending": False})
    out["v"] = version
    return out

  def _loop(self):
    while not STOP.is_set():
      try:
        self.check()
      except Exception:
        pass
      time.sleep(self.WATCH_S)

  def start(self):
    with self.cond:
      if self._thread is not None:
        return
      self._thread = threading.Thread(target=self._loop, name="nap_dash_sign_watch", daemon=True)
      self._thread.start()
    self.check(force=True)

  def wait_change(self, since, timeout):
    """Block until version != since or timeout. Returns the current version."""
    deadline = self._clock() + max(0.0, timeout)
    with self.cond:
      while self.version == since and not STOP.is_set():
        left = deadline - self._clock()
        if left <= 0:
          break
        self.cond.wait(min(left, 1.0))
      return self.version

  def open_stream(self):
    """Register one SSE stream. The newest MAX_STREAMS win: a reloaded MCU page
    whose old socket is not closed yet must never lock the new one out."""
    token = object()
    with self.cond:
      self._streams.append(token)
      while len(self._streams) > self.MAX_STREAMS:
        self._streams.pop(0)
      self.cond.notify_all()
    return token

  def stream_alive(self, token):
    with self.cond:
      return token in self._streams

  def close_stream(self, token):
    with self.cond:
      if token in self._streams:
        self._streams.remove(token)

  def enter_wait(self):
    with self.cond:
      if self._waiters >= self.MAX_WAITERS:
        return False
      self._waiters += 1
      return True

  def leave_wait(self):
    with self.cond:
      self._waiters = max(0, self._waiters - 1)


SIGN_WATCH = SignWatch()
SSE_HEARTBEAT_S = 15.0
SSE_MAX_S = 600.0       # the page's EventSource reconnects; bounds a dead socket
LONG_POLL_S = 25.0


def sse_event(payload, event="sign") -> bytes:
  return f"event: {event}\ndata: {json.dumps(payload, separators=(',', ':'))}\n\n".encode()


def dashboard_html() -> bytes:
  try:
    return DASHBOARD_PATH.read_bytes()
  except OSError:
    return b"<h1>openpilot</h1><p>dashboard.html missing</p>"


class Handler(BaseHTTPRequestHandler):
  protocol_version = "HTTP/1.1"

  def log_message(self, fmt, *args):
    # /api/ping is polled every couple of seconds. Skip the access line so
    # the comma is not writing a log record on each MCU poll.
    if args and ("/api/ping" in str(args[0]) or "/api/speed_sign?wait" in str(args[0])):
      return
    cloudlog.info("nap_dash " + (fmt % args))

  def send_json(self, obj, code=200, cache_control=None):
    body = json.dumps(obj).encode()
    self.send_response(code)
    self.send_header("Content-Type", "application/json")
    self.send_header("Cache-Control", cache_control or "no-cache, no-store")
    if cache_control and "no-store" in cache_control:
      self.send_header("Pragma", "no-cache")
      self.send_header("Expires", "0")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    self.wfile.write(body)

  def _send_html(self):
    body = dashboard_html()
    self.send_response(200)
    self.send_header("Content-Type", "text/html; charset=utf-8")
    self.send_header("Cache-Control", "no-cache, no-store")
    # Tesla MCU WebKit ignores Cache-Control on a stable URL. Pragma/Expires
    # match /api/ping. A cached dashboard from before the speed-sign overlay
    # would never poll /api/speed_sign.
    self.send_header("Pragma", "no-cache")
    self.send_header("Expires", "0")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    self.wfile.write(body)

  def _sign_stream(self, watch=None, heartbeat_s=None, max_s=None):
    """Server-Sent Events: current state on connect, then one event per change."""
    watch = SIGN_WATCH if watch is None else watch
    heartbeat_s = SSE_HEARTBEAT_S if heartbeat_s is None else heartbeat_s
    max_s = SSE_MAX_S if max_s is None else max_s
    watch.start()
    self.close_connection = True
    self.send_response(200)
    self.send_header("Content-Type", "text/event-stream")
    self.send_header("Cache-Control", "no-cache, no-store, max-age=0")
    self.send_header("Pragma", "no-cache")
    self.send_header("Expires", "0")
    self.send_header("Connection", "close")
    self.end_headers()
    token = watch.open_stream()
    started = time.monotonic()
    try:
      # Padding: some WebKit builds buffer the first 2 KB before firing events.
      self.wfile.write(b":" + b" " * 2048 + b"\nretry: 2000\n\n")
      version, _ = watch.snapshot()
      self.wfile.write(sse_event(watch.payload()))
      self.wfile.flush()
      last_write = time.monotonic()
      while not STOP.is_set() and watch.stream_alive(token):
        now = time.monotonic()
        if now - started >= max_s:
          break
        wait = min(heartbeat_s - (now - last_write), max_s - (now - started))
        new_version = watch.wait_change(version, max(0.05, wait))
        if not watch.stream_alive(token):
          break
        if new_version != version:
          version = new_version
          self.wfile.write(sse_event(watch.payload()))
        elif time.monotonic() - last_write >= heartbeat_s:
          self.wfile.write(b": hb\n\n")
        else:
          continue
        self.wfile.flush()
        last_write = time.monotonic()
    except (BrokenPipeError, ConnectionResetError, OSError):
      pass
    finally:
      watch.close_stream(token)

  def _sign_long_poll(self, since, watch=None, timeout=None):
    """Fallback without EventSource: hold until the state version moves past ``since``."""
    watch = SIGN_WATCH if watch is None else watch
    watch.start()
    if watch.enter_wait():
      try:
        if watch.snapshot()[0] == since:
          watch.wait_change(since, LONG_POLL_S if timeout is None else timeout)
      finally:
        watch.leave_wait()
    return self.send_json(watch.payload(), cache_control="no-store, max-age=0")

  def do_GET(self):
    path = urlparse(self.path).path
    if path == "/api/speed_sign/stream":
      return self._sign_stream()
    if path == "/api/speed_sign" and "wait" in parse_qs(urlparse(self.path).query):
      try:
        since = int(parse_qs(urlparse(self.path).query)["wait"][0])
      except (TypeError, ValueError):
        since = -1
      return self._sign_long_poll(since)
    if path in ("/", "/index.html", "/phone", "/phone/"):
      return self._send_html()
    if path == "/api/ping":
      return self.send_json(ping_payload(), cache_control="no-store, max-age=0")
    if path == "/api/manifest":
      try:
        return self.send_json(public_manifest())
      except Exception as exc:
        return self.send_json({"error": str(exc)}, 500)
    if path == "/api/state":
      return self.send_json(state_snapshot())
    if path == "/api/settings":
      try:
        snap = locked_settings()
        snap["catalog"] = public_manifest()["controls"]
        return self.send_json(snap)
      except Exception as exc:
        return self.send_json({"error": str(exc)}, 500)
    if path == "/api/network":
      try:
        return self.send_json(network_status())
      except Exception as exc:
        return self.send_json({"error": str(exc)}, 500)
    if path == "/api/force_offroad":
      try:
        return self.send_json(read_ack(_params()), cache_control="no-store, max-age=0")
      except Exception as exc:
        return self.send_json({"error": str(exc)}, 500, cache_control="no-store, max-age=0")
    if path == "/api/speed_sign":
      try:
        return self.send_json(read_sign(_params()), cache_control="no-store, max-age=0")
      except Exception as exc:
        return self.send_json({"error": str(exc)}, 500, cache_control="no-store, max-age=0")
    if path == "/api/review_flags":
      try:
        kind = (parse_qs(urlparse(self.path).query).get("kind") or ["both"])[0]
        return self.send_json(read_flags(kind=kind), cache_control="no-store, max-age=0")
      except Exception as exc:
        return self.send_json({"error": str(exc)}, 500)
    if path == "/api/bookmark":
      try:
        return self.send_json(bookmark_status(_params()))
      except Exception as exc:
        return self.send_json({"error": str(exc)}, 500)
    if path == "/api/talk/poll":
      return self.send_json(TALK.poll(parse_active(urlparse(self.path).query)))
    if path == "/api/talk/state":
      return self.send_json(TALK.state())
    if path == "/api/software":
      try:
        return self.send_json(read_software(_params()))
      except Exception as exc:
        return self.send_json({"error": str(exc)}, 500)
    return self.send_json({"error": "not found"}, 404)

  def do_POST(self):
    path = urlparse(self.path).path
    try:
      n = int(self.headers.get("Content-Length", "0"))
      if n < 0 or n > 16 * 1024:
        return self.send_json({"error": "payload too large"}, 413)
      payload = json.loads(self.rfile.read(n).decode() or "{}")
      if path == "/api/software":
        return self.send_json(handle_software(payload, _params()))
      if path == "/api/force_offroad":
        return self.send_json(handle_ack(payload, _params()), cache_control="no-store, max-age=0")
      if path == "/api/speed_sign":
        out = handle_sign(payload, _params())
        SIGN_WATCH.check(force=True)   # a tap here clears every other open MCU page now
        return self.send_json(out, cache_control="no-store, max-age=0")
      if path == "/api/bookmark":
        return self.send_json(bookmark_tap(_params()))
      if path == "/api/talk":
        return self.send_json(TALK.tap())
      if path != "/api/set":
        return self.send_json({"error": "not found"}, 404)
      action = str(payload.get("action") or "").strip().lower()
      if action in {"engage", "disengage"}:
        return self.send_json({"error": "engaging openpilot from the web is not allowed"}, 400)
      name = str(payload.get("param") or payload.get("name") or "")
      snap = locked_write(name, payload.get("value"), bool(payload.get("confirm")))
      # Lets the page raise the steering Yes/No right after the tap, not on the next poll.
      snap["force_offroad_ack"] = read_ack(_params())
      return self.send_json(snap)
    except (AckError, SignError) as exc:
      return self.send_json({"error": str(exc)}, 400)
    except SoftwareError as exc:
      return self.send_json({"error": str(exc)}, 400)
    except SettingError as exc:
      return self.send_json({"error": str(exc)}, 400)
    except Exception as exc:
      return self.send_json({"error": str(exc)}, 400)


class DashHTTPServer(ThreadingHTTPServer):
  allow_reuse_address = True
  daemon_threads = True


def bind_http_server(host=HOST, port=None):
  """Bind port 80, then 7070. Returns None if every port fails.

  An explicit port (tests, NAP_DASH_PORT) tries only that port.
  """
  ports = [int(port)] if port is not None else listen_ports()
  for one in ports:
    try:
      server = DashHTTPServer((host, one), Handler)
      cloudlog.info(f"nap_dash listening on {host}:{one} (Params + software, no cereal)")
      return server
    except OSError as exc:
      cloudlog.warning(f"nap_dash failed to bind {host}:{one}: {exc}")
  return None


def idle_until_stop(stop_event=None, sleeper=time.sleep):
  """Keep the managed process alive so optional Dash cannot block engage."""
  stop_event = STOP if stop_event is None else stop_event
  while not stop_event.is_set():
    sleeper(1.0)


def main():
  try:
    hot = get_manifest().get("hotspot") or {}
    cloudlog.info("nap_dash url " + str(hot.get("url") or "http://100.99.9.1/"))
  except Exception:
    cloudlog.warning("nap_dash manifest unavailable at start")
  server = bind_http_server(HOST)
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
