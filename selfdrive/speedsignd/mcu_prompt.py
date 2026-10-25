"""Pending speed-sign Yes/No/Temp, shared by the comma HUD and nap_dash.

The HUD publishes the prompt it is showing (read speed, plus the way and
heading captured when the sign was read). Either screen answers through
answer_prompt(), once per prompt id. That calls on_confirm_accuracy, which
is the comma Yes/No path: two_stage.record_answer, and on Yes
mapd.speed_overrides.request_override (NAPSpeedSignConfirmed). Temp logs the
answer as "temp" and hands the mph to mapd.temp_limit.request_temp
(NAPSpeedSignTemp): a this-drive-only limit that is never saved.

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

# Yes on a prompt with no road matched at read time writes nothing to the map
# (never a guessed road); both screens show this briefly instead.
NO_ROAD_NOTE = "Not saved to map"
NOTE_S = 3.0
TEMP_NOTE = "Temp {mph} this drive"

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
_logged_put_failure = False


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
    loaded = _load(params.get(PROMPT_PARAM))
  except Exception:
    return _read_fallback(params)
  if loaded is not None:
    return loaded
  # Empty param is authoritative. The fallback file is only for a put that
  # could not use Params; a later successful put deletes it.
  return None


def _fallback_path(params):
  getter = getattr(params, "get_param_path", None)
  if getter is None:
    return None
  try:
    base = getter()
  except Exception:
    return None
  if not base:
    return None
  return os.path.join(base, PROMPT_PARAM)


def _write_fallback(params, obj: dict) -> bool:
  """Used when the compiled params binary does not know NAPSpeedSignPrompt yet."""
  path = _fallback_path(params)
  if not path:
    return False
  os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
  tmp = path + ".tmp"
  with open(tmp, "w", encoding="utf-8") as fh:
    fh.write(json.dumps(obj))
  os.replace(tmp, path)
  return True


def _read_fallback(params):
  path = _fallback_path(params)
  if not path or not os.path.isfile(path):
    return None
  try:
    with open(path, encoding="utf-8") as fh:
      return _load(fh.read())
  except Exception:
    return None


def _erase_fallback(params) -> None:
  path = _fallback_path(params)
  if not path:
    return
  try:
    os.remove(path)
  except OSError:
    pass


def _put(params, obj: dict) -> None:
  """JSON keys accept a dict. A binary that still has the key as STRING accepts the dumped text.

  UnknownKeyName (params built before this key) is written beside the params
  dir so nap_dash can still read it. A successful Params put removes that file.
  """
  global _logged_put_failure
  try:
    params.put(PROMPT_PARAM, obj)
  except TypeError:
    try:
      params.put(PROMPT_PARAM, json.dumps(obj))
    except Exception:
      if not _logged_put_failure:
        cloudlog.exception("mcu_prompt: NAPSpeedSignPrompt put failed")
        _logged_put_failure = True
      if not _write_fallback(params, obj):
        raise
      return
  except Exception:
    if not _logged_put_failure:
      cloudlog.exception("mcu_prompt: NAPSpeedSignPrompt put failed")
      _logged_put_failure = True
    if not _write_fallback(params, obj):
      raise
    return
  _erase_fallback(params)


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


def read_note(params=None, now: float | None = None) -> str | None:
  """Brief note left by the last answer (e.g. NO_ROAD_NOTE), or None once it is old."""
  import time
  try:
    params = _params(params)
    with _locked(params):
      cur = _get(params)
    if not isinstance(cur, dict) or not cur.get("note"):
      return None
    now = time.monotonic() if now is None else float(now)
    if 0.0 <= now - float(cur.get("note_t") or 0.0) <= NOTE_S:
      return str(cur["note"])
  except Exception:
    pass
  return None


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


def on_confirm_accuracy(mph: int, yes, proposal: dict | None = None) -> None:
  """Yes/No feedback: append to /data/media/0/nap/speedsign_feedback/feedback.jsonl.

  Keeps the peak-size crop JPEG speedsignd saved for the plate (pending.jpg)
  and, on Yes, also appends confirmed.jsonl (future OSM confirmed readings;
  nothing is uploaded to osm.org). The row and crop are mirrored into the
  current realdata segment for athena harvest.

  Yes only: also sets NAPSpeedSignConfirmed; mapd stores it as a local
  override for the matched OSM way + heading, so the regular speed-limit sign
  and Map Speed Limit (MAX = limit + offset) use it now and on future drives
  (selfdrive/mapd/speed_overrides.py). No is log-only.

  Temp (yes="temp"): the row is logged with answer "temp" (never
  confirmed.jsonl), and NAPSpeedSignTemp hands the mph to mapd for this road,
  this drive only (mapd/temp_limit.py). Nothing is saved to the map.

  way_id and heading come from the proposal captured when the sign was read.
  """
  import time
  from openpilot.selfdrive.speedsignd.confirm import answer_kind
  from openpilot.selfdrive.speedsignd.two_stage import record_answer
  kind = answer_kind(yes)
  yes = kind == "yes"
  prop = proposal or {}
  seq = int(prop["seq"]) if prop.get("seq") else None
  rec = (record_answer(int(mph), False, seq=seq, answer="temp") if kind == "temp"
         else record_answer(int(mph), yes, seq=seq))
  if rec is not None and rec.get("stale_pending"):
    rec = dict(rec, lat=None, lon=None, bearing=None,   # never another read's location
               sign_lat=None, sign_lon=None, sign_range_m=None)
  applied = False
  if prop.get("seq"):
    # The plate already shows the read; Yes only adds the map override.
    from openpilot.selfdrive.speedsignd.confirm import write_answer
    if not write_answer(int(prop["seq"]), int(mph), "temp" if kind == "temp" else yes):
      cloudlog.warning("speed_sign_hud: answer param failed")
  if kind == "temp":
    # This drive only, never saved. No read-time road: mapd anchors it on the road it is on now.
    from openpilot.selfdrive.mapd.temp_limit import request_temp
    applied = request_temp(int(mph), rec, read_seq=prop.get("seq"),
                           way_id=prop.get("way_id"), heading=prop.get("heading"))
  elif yes and int(prop.get("way_id") or 0) <= 0:
    # No road matched when the sign was read: never guess one. Nothing goes to mapd.
    cloudlog.info("speed_sign_hud: yes without a read-time road; not saved to map")
  elif yes:
    from openpilot.selfdrive.mapd.speed_overrides import request_override
    applied = request_override(int(mph), rec, time.monotonic(),
                               way_id=prop.get("way_id"), heading=prop.get("heading"))
  cloudlog.info("speed_sign_hud feedback mph=%s answer=%s saved=%s jpg=%s lat=%s lon=%s realdata=%s applied=%s",
                int(mph), kind, rec is not None, (rec or {}).get("jpg"),
                (rec or {}).get("lat"), (rec or {}).get("lon"), (rec or {}).get("realdata"), applied)


def answer_prompt(prompt_id: int, yes, params=None, snapshot: dict | None = None) -> bool:
  """Answer prompt `prompt_id` at most once. True when this call owned it.

  `yes` is True (Yes), False (No) or "temp" (Temp: this road, this drive only).

  An open record wins over `snapshot` (the comma's in-memory copy). snapshot
  is only used when nothing is open, so a tap that lands before the first
  publish still answers, and a second tap does not. The stored way and heading
  are the ones published with the prompt. Callers cannot substitute another way.
  """
  from openpilot.selfdrive.speedsignd.confirm import answer_kind
  try:
    prompt_id = int(prompt_id)
    kind = answer_kind(yes)
  except (TypeError, ValueError):
    return False
  yes = kind == "yes"
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
      rec = dict(owned, answered=True, yes=bool(yes), answer=kind)
      if yes and owned["way_id"] <= 0:
        import time
        rec.update(note=NO_ROAD_NOTE, note_t=time.monotonic())
      elif kind == "temp":
        import time
        rec.update(note=TEMP_NOTE.format(mph=int(owned["mph"])), note_t=time.monotonic())
      _put(params, rec)
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
    on_confirm_accuracy(int(owned["mph"]), "temp" if kind == "temp" else yes, {
      "seq": int(owned["id"]),
      "way_id": int(owned["way_id"]),
      "heading": float(owned["heading"]),
    })
  except Exception:
    cloudlog.exception("mcu_prompt: confirm failed")
  return True
