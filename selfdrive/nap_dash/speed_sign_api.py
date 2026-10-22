"""MCU mirror of the on-road speed-sign Yes/No.

Params only. Yes and No both go through speedsignd.mcu_prompt.answer_prompt,
which calls on_confirm_accuracy (record_answer, and on Yes request_override).
The way and heading are the ones stored when the sign was read. This module
does not open cereal sockets and does not engage.
"""
from __future__ import annotations

from openpilot.selfdrive.speedsignd.hud import HUD_CONFIRM_NO, HUD_CONFIRM_PROMPT, HUD_CONFIRM_YES
from openpilot.selfdrive.speedsignd.mcu_prompt import answer_prompt, read_pending


class SignError(ValueError):
  pass


def read_sign(params) -> dict:
  pending = read_pending(params)
  if not pending:
    return {"pending": False}
  mph = int(pending["mph"])
  return {
    "pending": True,
    "id": int(pending["id"]),
    "mph": mph,
    "prompt": f"{mph} mph {HUD_CONFIRM_PROMPT}",
    "yes": HUD_CONFIRM_YES,
    "no": HUD_CONFIRM_NO,
  }


def handle_sign(payload: dict, params) -> dict:
  """One answer. No second confirmation. Client way/heading are ignored."""
  answer = str(payload.get("answer") or "").strip().lower()
  if answer not in ("yes", "no"):
    raise SignError("answer must be yes or no")
  try:
    prompt_id = int(payload["id"])
  except (KeyError, TypeError, ValueError):
    raise SignError("prompt id required") from None
  if not answer_prompt(prompt_id, answer == "yes", params=params):
    raise SignError("Nothing to confirm: that speed-sign prompt is not waiting.")
  return read_sign(params)
