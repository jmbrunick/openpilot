"""Gap lock: a stalk bump steps the locked meters by 1 m.

Third layer on the Pre-AP up/down stalk tip:

  * No lead: the tip steps MAX by 1 mph (unchanged).
  * Lead, no gap lock: the tip steps City/Highway Follow Distance 1–7 (unchanged).
  * Gap lock engaged: the tip steps the locked gap. Up = 1 m closer,
    down = 1 m farther, clamped to GAP_LOCK_MIN_M..GAP_LOCK_MAX_M.
    MAX and Follow Distance are left alone.

A full press (2nd detent, ±5 mph) still clears the lock and steps MAX,
as before.

card owns tip-vs-full (it sees the raw detent). The planner owns the
latch. They talk over two small Params, the same way the Follow
Distance stalk already does:

  * NAPGapLockActive (planner → card). Written only when the lock turns
    on or off. card reads it once at the start of each stalk gesture.
  * NAPGapLockBump (card → planner). seq * 10 + code, one write per
    gesture. The planner acts on a new seq.

Fail-safe: if the planner sees an up/down press while locked and no
verdict arrives within GAP_LOCK_BUMP_TIMEOUT_S, it clears the lock
(today's behavior).
"""

from __future__ import annotations

from openpilot.selfdrive.controls.lib.gap_lock import GAP_LOCK_MAX_M, GAP_LOCK_MIN_M

GAP_LOCK_BUMP_M = 1.0
GAP_LOCK_BUMP_TIMEOUT_S = 3.0

PARAM_GAP_LOCK_ACTIVE = "NAPGapLockActive"
PARAM_GAP_LOCK_BUMP = "NAPGapLockBump"

BUMP_NONE = 0
BUMP_CLOSER = 1   # stalk up tip
BUMP_FARTHER = 2  # stalk down tip
BUMP_FULL = 3     # 2nd detent while locked: clear, MAX ±5 as today

_SEQ_MOD = 10000

# Raw Pre-AP CruiseButtons idle (hypermile.CRUISE_STALK_IDLE).
_DETENT_IDLE = 0


def encode_bump(seq: int, code: int) -> int:
  return (int(seq) % _SEQ_MOD) * 10 + (int(code) % 10)


def decode_bump(raw) -> tuple[int, int]:
  """(seq, code). Missing or junk reads as (0, BUMP_NONE)."""
  if isinstance(raw, bool) or not isinstance(raw, (int, str, bytes)):
    return 0, BUMP_NONE
  try:
    value = int(raw)
  except (TypeError, ValueError):
    return 0, BUMP_NONE
  if value < 0:
    return 0, BUMP_NONE
  code = value % 10
  if code not in (BUMP_CLOSER, BUMP_FARTHER, BUMP_FULL):
    code = BUMP_NONE
  return value // 10, code


def bump_code(closer) -> int:
  if closer is True:
    return BUMP_CLOSER
  if closer is False:
    return BUMP_FARTHER
  return BUMP_NONE


def bumped_gap_m(gap_m, code: int):
  """New locked meters for a tip. None when there is no lock or no tip."""
  try:
    gap = float(gap_m)
  except (TypeError, ValueError):
    return None
  if gap != gap or gap <= 0.0:
    return None
  if code == BUMP_CLOSER:
    gap -= GAP_LOCK_BUMP_M
  elif code == BUMP_FARTHER:
    gap += GAP_LOCK_BUMP_M
  else:
    return None
  return float(min(max(gap, GAP_LOCK_MIN_M), GAP_LOCK_MAX_M))


def _read_bool(params, key) -> bool:
  try:
    return params.get_bool(key) is True
  except Exception:
    return False


class GapLockBumpCard:
  """card side. Decide once per gesture whether a tip belongs to gap lock."""

  def __init__(self) -> None:
    self.locked = False
    self.seq = 0
    self._prev_detent = _DETENT_IDLE

  def refresh(self, params, detent, button_press: bool) -> bool:
    """Read NAPGapLockActive on the first non-idle frame of a gesture.

    Raw detent when card has it. Otherwise any up/down press edge.
    """
    start = False
    if detent is not None:
      try:
        d = int(detent)
      except (TypeError, ValueError):
        d = _DETENT_IDLE
      start = d != _DETENT_IDLE and self._prev_detent == _DETENT_IDLE
      self._prev_detent = d
    elif button_press:
      start = True
    if start:
      self.locked = _read_bool(params, PARAM_GAP_LOCK_ACTIVE)
    return self.locked

  def report(self, params, code: int) -> bool:
    if code not in (BUMP_CLOSER, BUMP_FARTHER, BUMP_FULL):
      return False
    self.seq = (int(self.seq) % (_SEQ_MOD - 1)) + 1
    try:
      params.put(PARAM_GAP_LOCK_BUMP, encode_bump(self.seq, code))
    except Exception:
      return False
    return True


class GapLockBumpRouter:
  """Planner side. Turns presses + card verdicts into (clear, bump code)."""

  def __init__(self) -> None:
    self._seen_seq: int | None = None
    self._pending_s: float | None = None
    self._active_written: bool | None = None

  def read_verdict(self, params) -> int:
    try:
      raw = params.get(PARAM_GAP_LOCK_BUMP)
    except Exception:
      return BUMP_NONE
    seq, code = decode_bump(raw)
    if self._seen_seq is None:
      # First read after start: anything already there is stale.
      self._seen_seq = seq
      return BUMP_NONE
    if seq == self._seen_seq:
      return BUMP_NONE
    self._seen_seq = seq
    return code

  def update(self, dt, *, locked: bool, stalk_press: bool, verdict: int) -> tuple[bool, int]:
    """(stalk_exit for GapLockLatch, bump code to apply after the latch)."""
    frame_dt = 0.05 if dt is None or float(dt) <= 0.0 else float(dt)
    if not locked:
      self._pending_s = None
      return bool(stalk_press), BUMP_NONE
    if verdict == BUMP_FULL:
      self._pending_s = None
      return True, BUMP_NONE
    if verdict in (BUMP_CLOSER, BUMP_FARTHER):
      self._pending_s = None
      return False, verdict
    if self._pending_s is not None:
      self._pending_s += frame_dt
      if self._pending_s >= GAP_LOCK_BUMP_TIMEOUT_S:
        self._pending_s = None
        return True, BUMP_NONE
    elif stalk_press:
      self._pending_s = 0.0
    return False, BUMP_NONE

  def publish_active(self, params, locked: bool) -> None:
    """NAPGapLockActive on change only (and once at start)."""
    value = bool(locked)
    if self._active_written is value:
      return
    try:
      # Params.put_bool is non-blocking by default.
      params.put_bool(PARAM_GAP_LOCK_ACTIVE, value)
    except Exception:
      return
    self._active_written = value


def planner_gap_lock_step(router: GapLockBumpRouter, latch, params, dt, *,
                          stalk_press: bool, update_latch) -> float | None:
  """The planner hook. Runs the latch, then applies a tip. Returns new meters on a bump.

  update_latch(stalk_exit) runs GapLockLatch.update with the planner's
  other inputs unchanged.
  """
  verdict = router.read_verdict(params)
  stalk_exit, code = router.update(
    dt, locked=latch.gap_m is not None, stalk_press=stalk_press, verdict=verdict)
  update_latch(stalk_exit)
  new_m = None
  if code != BUMP_NONE and latch.gap_m is not None:
    new_m = latch.nudge(code == BUMP_CLOSER)
  router.publish_active(params, latch.gap_m is not None)
  return new_m


def card_gap_lock_stalk(bump: GapLockBumpCard, params, *, gap_locked: bool,
                        saw_hold_before: bool, saw_hold_now: bool,
                        commit: bool, closer) -> bool:
  """card hook after FollowStalkGesture.update. True when the tip went to gap lock.

  A True return means card must not also persist Follow Distance.
  """
  if not gap_locked:
    return False
  if saw_hold_now and not saw_hold_before:
    bump.report(params, BUMP_FULL)
    return False
  if commit and closer is not None:
    bump.report(params, bump_code(closer))
    return True
  return False
