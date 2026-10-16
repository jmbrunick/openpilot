"""Talk to CoS: a one-shot, in-memory flag the Phone Hub app polls.

The MCU button POSTs /api/talk. The phone polls GET /api/talk/poll about once
a second, consumes the flag, and starts voice on the phone. Nothing here
touches Params, cereal, CAN, or any car control; it's only a flag in memory.
"""
from __future__ import annotations

import threading
import time

STALE_S = 30.0  # an unconsumed tap older than this is dropped
PHONE_SEEN_S = 5.0


class TalkFlag:
  def __init__(self, clock=time.monotonic):
    self._clock = clock
    self._lock = threading.Lock()
    self._pending_at: float | None = None
    self._active = False
    self._phone_at: float | None = None

  def tap(self) -> dict:
    with self._lock:
      self._pending_at = self._clock()
      return self._state_locked()

  def poll(self, active: bool | None = None) -> dict:
    """Phone side: consume the pending tap (once) and report voice state."""
    with self._lock:
      now = self._clock()
      self._phone_at = now
      if active is not None:
        self._active = active
      talk = self._pending_at is not None and now - self._pending_at <= STALE_S
      age = None if self._pending_at is None else round(now - self._pending_at, 2)
      self._pending_at = None
      return {"talk": talk, "age": age}

  def state(self) -> dict:
    with self._lock:
      return self._state_locked()

  def _state_locked(self) -> dict:
    now = self._clock()
    pending = self._pending_at is not None and now - self._pending_at <= STALE_S
    phone = self._phone_at is not None and now - self._phone_at <= PHONE_SEEN_S
    return {"pending": pending, "active": self._active and phone, "phone": phone}


TALK = TalkFlag()


def parse_active(query: str) -> bool | None:
  from urllib.parse import parse_qs
  v = (parse_qs(query).get("active") or [None])[0]
  if v is None:
    return None
  return v.strip().lower() in ("1", "true", "yes", "on")
