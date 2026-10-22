"""Pending speed-sign Yes/No, shared by the comma HUD and nap_dash.

The HUD publishes the prompt it is showing (read speed, plus the way and
heading captured when the sign was read). Either screen answers through
answer_prompt(), once per prompt id. That calls on_confirm_accuracy, which
is the comma Yes/No path: two_stage.record_answer, and on Yes
mapd.speed_overrides.request_override (NAPSpeedSignConfirmed).

No cereal subscriber and no log fields. Nothing here brakes, steers, or
engages.
"""
from __future__ import annotations

import json
import os
import tempfile
import threading
from contextlib import contextmanager

from openpilot.selfdrive.speedsignd.hud import hud_confirm_visible

try:
  from openpilot.common.swaglog import cloudlog
except Exception:  # pragma: no cover - unit tests without zmq
  class cloudlog:
    @staticmethod
    def info(msg, *a):
      pass

    @staticmethod
    def warning(msg, *a):
      pass

    @staticmethod
    def exception(msg, *a):
      pass


PROMPT_PARAM = "NAPSpeedSignPrompt"

_PARAMS = None
_THREAD = threading.Lock()


def _params(params):
  if params is not None:
    return params
  global _PARAMS
  if _PARAMS is None:
    from openpilot.common.params import Params
    _PARAMS = Params()
  return _PARAMS


def _lock_path(params) -> str:
  env = os.environ.get("NAP_SPEED_SIGN_PROMPT_LOCK")
  if env:
    return env
  getter = getattr(params, "get_param_path", None)
  if getter is not None:
    try:
      base = getter()
      if base:
        return os.path.join(base, ".nap_speed_sign_prompt.lock")
    except Exception:
      pass
  return os.path.join(tempfile.gettempdir(), "nap_speed_sign_prompt.lock")


@contextmanager
def _locked(params):
  """One writer across the UI process and nap_dash."""
  path = _lock_path(params)
  os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
  with _THREAD:
    fh = open(path, "a+")
    try:
      import fcntl
      fcntl.flock(fh, fcntl.LOCK_EX)
      yield
    finally:
      try:
        import fcntl
        fcntl.flock(fh, fcntl.LOCK_UN)
      except Exception:
        pass
      fh.close()


def _load(raw):
  if raw is None or raw == b"" or raw == "":
    return None
  if isinstance(raw, dict):
    return raw
  try:
    if isinstance(raw, (bytes, bytearray)):
      raw = raw.decode()
    parsed = json.loads(raw) if isinstance(raw, str) else None
  except Exception:
    return None
  return parsed if isinstance(parsed, dict) else None


def _get(params) -> dict | None:
  try:
    return _load(params.get(PROMPT_PARAM))
  except Exception:
    return None


def _put(params, obj: dict) -> None:
  params.put(PROMPT_PARAM, obj)


def _clear(params, cleared_id: int = -1) -> None:
  """Close the open prompt. The id is kept so a late tap on it is refused."""
  _put(params, {"cleared": True, "id": int(cleared_id)})


def _open(cur) -> bool:
  if not isinstance(cur, dict) or cur.get("answered") or cur.get("cleared"):
    return False
  try:
    return int(cur.get("mph") or 0) > 0 and int(cur.get("id")) >= 0
  except (TypeError, ValueError):
    return False


def _fields(d: dict) -> dict:
  heading = d.get("heading")
  way = d.get("way_id")
  return {
    "id": int(d["id"]),
    "mph": int(d["mph"]),
    "way_id": 0 if way is None else int(way),
    "heading": 0.0 if heading is None else float(heading),
  }


def _same(cur: dict, showing: dict) -> bool:
  a, b = _fields(cur), showing
  return a["id"] == b["id"] and a["mph"] == b["mph"] and a["way_id"] == b["way_id"] and abs(a["heading"] - b["heading"]) < 1e-3


def prompt_from_view(view) -> dict | None:
  """The prompt the HUD is drawing, or None. Way/heading are the read-time values on the view."""
  if view is None or not getattr(view, "proposal_mode", False):
    return None
  if not hud_confirm_visible(view):
    return None
  try:
    mph = int(view.proposed_mph)
    seq = int(view.proposed_seq)
  except (TypeError, ValueError):
    return None
  if mph <= 0 or seq < 0:
    return None
  return _fields({
    "id": seq,
    "mph": mph,
    "way_id": getattr(view, "proposed_way_id", 0),
    "heading": getattr(view, "proposed_heading", 0.0),
  })


def read_pending(params=None) -> dict | None:
  """Open prompt {id, mph, way_id, heading}, or None once it has been answered or cleared."""
  params = _params(params)
  with _locked(params):
    cur = _get(params)
    if not _open(cur):
      return None
    return _fields(cur)


def sync_hud_prompt(view, params=None) -> bool:
  """Publish what the HUD is showing. True when this id was already answered elsewhere.

  A hidden prompt clears an unanswered record (No-equivalent timeout: the map
  override is not touched). An answered record stays so the same id cannot be answered again.
  """
  params = _params(params)
  showing = prompt_from_view(view)
  with _locked(params):
    cur = _get(params)
    if showing is None:
      if _open(cur):
        _clear(params, int(cur["id"]))
      return False
    if isinstance(cur, dict) and cur.get("answered") and int(cur.get("id", -1)) == showing["id"]:
      return True
    if _open(cur) and _same(cur, showing):
      return False
    _put(params, dict(showing, answered=False))
    return False


def on_confirm_accuracy(mph: int, yes: bool, proposal: dict | None = None) -> None:
  """Yes/No feedback: append to /data/media/0/nap/speedsign_feedback/feedback.jsonl.

  Keeps the peak-size crop JPEG speedsignd saved for the plate (pending.jpg)
  and, on Yes, also appends confirmed.jsonl (future OSM confirmed readings;
  nothing is uploaded to osm.org). The row and crop are mirrored into the
  current realdata segment for athena harvest.

  Yes only: also sets NAPSpeedSignConfirmed; mapd stores it as a local
  override for the matched OSM way + heading, so the regular speed-limit sign
  and Map Speed Limit (MAX = limit + offset) use it now and on future drives
  (selfdrive/mapd/speed_overrides.py). No is log-only.

  way_id and heading come from the proposal captured when the sign was read.
  """
  import time
  from openpilot.selfdrive.speedsignd.two_stage import record_answer
  prop = proposal or {}
  rec = record_answer(int(mph), bool(yes), seq=int(prop["seq"]) if prop.get("seq") else None)
  if rec is not None and rec.get("stale_pending"):
    rec = dict(rec, lat=None, lon=None, bearing=None)   # never another read's location
  applied = False
  if prop.get("seq"):
    # The plate already shows the read; Yes only adds the map override.
    from openpilot.selfdrive.speedsignd.confirm import write_answer
    if not write_answer(int(prop["seq"]), int(mph), bool(yes)):
      cloudlog.warning("speed_sign_hud: answer param failed")
  if yes:
    from openpilot.selfdrive.mapd.speed_overrides import request_override
    applied = request_override(int(mph), rec, time.monotonic(),
                               way_id=prop.get("way_id"), heading=prop.get("heading"))
  cloudlog.info("speed_sign_hud feedback mph=%s yes=%s saved=%s jpg=%s lat=%s lon=%s realdata=%s applied=%s",
                int(mph), bool(yes), rec is not None, (rec or {}).get("jpg"),
                (rec or {}).get("lat"), (rec or {}).get("lon"), (rec or {}).get("realdata"), applied)


def answer_prompt(prompt_id: int, yes: bool, params=None, snapshot: dict | None = None) -> bool:
  """Answer prompt `prompt_id` at most once. True when this call owned it.

  An open record wins over `snapshot` (the comma's in-memory copy). snapshot
  is only used when nothing is open, so a tap that lands before the first
  publish still answers, and a second tap does not. The stored way and heading
  are the ones published with the prompt. Callers cannot substitute another way.
  """
  try:
    prompt_id = int(prompt_id)
  except (TypeError, ValueError):
    return False
  owned = None
  try:
    params = _params(params)
    with _locked(params):
      cur = _get(params)
      if _open(cur) and int(cur["id"]) == prompt_id:
        owned = _fields(cur)
      elif _open(cur):
        owned = None
      elif snapshot is not None:
        try:
          snap = _fields(snapshot)
        except (TypeError, ValueError, KeyError):
          snap = None
        # Any record with this id or a newer one means the snapshot is stale:
        # answered, timed out, or replaced by a newer read.
        try:
          seen = int(cur.get("id", -1)) if isinstance(cur, dict) else -1
        except (TypeError, ValueError):
          seen = -1
        if snap is not None and snap["id"] == prompt_id and snap["mph"] > 0 and prompt_id > seen:
          owned = snap
      if owned is None:
        return False
      _put(params, dict(owned, answered=True, yes=bool(yes)))
  except Exception:
    # The comma tap carries the read-time snapshot. A broken prompt record
    # must not drop that Yes/No; the MCU has no snapshot and cannot invent one.
    cloudlog.exception("mcu_prompt: prompt store failed")
    if owned is None and snapshot is not None:
      try:
        owned = _fields(snapshot)
      except (TypeError, ValueError, KeyError):
        return False
      if owned["id"] != prompt_id or owned["mph"] <= 0:
        return False
    elif owned is None:
      return False
  try:
    on_confirm_accuracy(int(owned["mph"]), bool(yes), {
      "seq": int(owned["id"]),
      "way_id": int(owned["way_id"]),
      "heading": float(owned["heading"]),
    })
  except Exception:
    cloudlog.exception("mcu_prompt: confirm failed")
  return True
