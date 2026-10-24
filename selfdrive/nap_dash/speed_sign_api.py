"""MCU mirror of the on-road speed-sign Yes/No/Temp.

Params only. Yes, No and Temp all go through speedsignd.mcu_prompt.answer_prompt,
which calls on_confirm_accuracy (record_answer, on Yes request_override, on
Temp mapd.temp_limit.request_temp: this road, this drive, never saved).
"temp" also reports mapd's running Temp ({"mph"}) or why it just ended.
The way and heading are the ones stored when the sign was read. This module
does not open cereal sockets and does not engage.
"""
from __future__ import annotations

from openpilot.selfdrive.mapd.temp_limit import read_state as read_temp_state
from openpilot.selfdrive.speedsignd.hud import HUD_CONFIRM_NO, HUD_CONFIRM_PROMPT, HUD_CONFIRM_TEMP, HUD_CONFIRM_YES
from openpilot.selfdrive.speedsignd.mcu_prompt import answer_prompt, read_note, read_pending

ANSWERS = {"yes": True, "no": False, "temp": "temp"}


class SignError(ValueError):
  pass


def _with_temp(out: dict, params) -> dict:
  try:
    st = read_temp_state(params)
  except Exception:
    st = None
  if st:
    out["temp_limit"] = st
  return out


def read_sign(params) -> dict:
  pending = read_pending(params)
  if not pending:
    note = read_note(params)
    return _with_temp({"pending": False, "note": note} if note else {"pending": False}, params)
  mph = int(pending["mph"])
  return _with_temp({
    "pending": True,
    "id": int(pending["id"]),
    "mph": mph,
    "prompt": f"{mph} mph {HUD_CONFIRM_PROMPT}",
    "yes": HUD_CONFIRM_YES,
    "no": HUD_CONFIRM_NO,
    "temp": HUD_CONFIRM_TEMP,
  }, params)


def handle_sign(payload: dict, params) -> dict:
  """One answer. No second confirmation. Client way/heading are ignored."""
  answer = str(payload.get("answer") or "").strip().lower()
  if answer not in ANSWERS:
    raise SignError("answer must be yes, no or temp")
  try:
    prompt_id = int(payload["id"])
  except (KeyError, TypeError, ValueError):
    raise SignError("prompt id required") from None
  if not answer_prompt(prompt_id, ANSWERS[answer], params=params):
    raise SignError("Nothing to confirm: that speed-sign prompt is not waiting.")
  return read_sign(params)
