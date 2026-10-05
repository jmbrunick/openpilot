"""Pre-AP gap lock: a 2 s engage-stalk hold latches radar meters.

The latch is a setpoint for the existing unified lead law
(`gap_set_override_m`). It is not a second controller. Mannerisms
`t_follow` still sizes intrusion, MPC, and FCW.

The hold arms whether longitudinal is already on, paused (pedal,
One-Pedal, brake, standstill wait), or not yet taken. A completed hold
resumes long so those meters become the setpoint. A tip, cancel, brake,
One-Pedal pause, or toggle off still clears a lock that is already on.

Stock DI still sees the physical pull (0x45 is relayed). After 0.5 s of
a qualified hold, the openpilot overlay repeats CANCEL on the existing
10 Hz spoof slot until the lever returns to idle. Panda does not treat
that in-session TX as a lateral drop: `tesla_preap_tx.h` calls
`pcm_cruise_check(false)` for lever == CANCEL only when
`!controls_allowed`.
"""

from __future__ import annotations

import time

from opendbc.car.tesla.values import CruiseButtons

from openpilot.selfdrive.controls.lib.lead_approach import (
  LEAD_CLOSE_HOLD_S,
  lead_follow_slack_m,
)

GAP_LOCK_HOLD_S = 2.0
GAP_LOCK_CANCEL_AFTER_S = 0.5
GAP_LOCK_MIN_M = 8.0
GAP_LOCK_MAX_M = 80.0
GAP_LOCK_MEDIAN_S = 0.4
GAP_LOCK_HUD_S = 1.5

HUD_NONE = 0
HUD_UNAVAILABLE = 2
HUD_LOST = 3

_PARAM_KEY = "NAPGapLock"
_param_cache = {"t": 0.0, "v": False}


def gap_lock_param_enabled(params) -> bool:
  """True only for a real bool True. MagicMock and a missing key stay off."""
  try:
    return params.get_bool(_PARAM_KEY) is True
  except Exception:
    return False


def gap_lock_enabled(engagement=None) -> bool:
  """Gesture enable. Tests set `_nap_gap_lock_enabled_override` on the FSM."""
  if engagement is not None:
    override = getattr(engagement, "_nap_gap_lock_enabled_override", None)
    if isinstance(override, bool):
      return override
  now = time.monotonic()
  if now - float(_param_cache["t"]) < 0.5:
    return bool(_param_cache["v"])
  val = False
  try:
    from openpilot.common.params import Params
    val = Params().get_bool(_PARAM_KEY) is True
  except Exception:
    val = False
  _param_cache["t"] = now
  _param_cache["v"] = val
  return val


def gap_lock_arm_seq(engagement) -> int:
  gesture = getattr(engagement, "_nap_gap_lock", None)
  if gesture is None:
    return 0
  return int(gesture.seq) & 0xFF


def gap_lock_cancel_hold(engagement) -> bool:
  gesture = getattr(engagement, "_nap_gap_lock", None)
  return bool(gesture is not None and gesture.cancel_hold)


def gap_lock_holding(engagement) -> bool:
  gesture = getattr(engagement, "_nap_gap_lock", None)
  return bool(gesture is not None and gesture.holding)


def _positive_meters(meters):
  try:
    value = float(meters)
  except (TypeError, ValueError):
    return None
  if value != value or value <= 0.0:
    return None
  return value


def gap_lock_engaged_text(meters) -> str | None:
  """Onroad alert line while a lock is latched. None when it is clear."""
  value = _positive_meters(meters)
  if value is None:
    return None
  return f"Gap lock engaged {int(round(value))} m"


def gap_lock_hud_chip(meters) -> str | None:
  """C3X MAX-column chip. None when gap lock is clear."""
  value = _positive_meters(meters)
  if value is None:
    return None
  return f"LOCK {int(round(value))}m"


def gap_lock_cancel_hold_tx(already_sent_stw: bool, hold: bool, frame: int) -> bool:
  """Repeat CANCEL on an empty 10 Hz 0x45 slot. Do not stack a second frame."""
  if hold is not True or already_sent_stw:
    return False
  try:
    return int(frame) % 10 == 0
  except (TypeError, ValueError):
    return False


def slack_for_guard(d_rel, v_lead, t_follow, gap_lock_m):
  """Actuator-guard slack. A lock uses locked meters; 0 stays on the time gap."""
  try:
    lock = float(gap_lock_m or 0.0)
  except (TypeError, ValueError):
    lock = 0.0
  if lock > 0.0 and d_rel is not None:
    return float(d_rel) - lock
  return lead_follow_slack_m(d_rel, v_lead, t_follow)


def stalk_follow_exit(cs) -> bool:
  """Up/down stalk edge (tip or 2nd detent). Both are `accelCruise` / `decelCruise`."""
  try:
    from cereal import car
    accel = int(car.CarState.ButtonEvent.Type.accelCruise)
    decel = int(car.CarState.ButtonEvent.Type.decelCruise)
    events = cs.buttonEvents
    n = len(events)
  except Exception:
    return False
  if n > 16:
    n = 16
  for i in range(n):
    try:
      be = events[i]
      if be.pressed is True and int(be.type) in (accel, decel):
        return True
    except Exception:
      continue
  return False


def _u8(value) -> int:
  if isinstance(value, bool) or value is None:
    return 0
  try:
    return int(value) & 0xFF
  except (TypeError, ValueError):
    return 0


def _track_id(value):
  if isinstance(value, bool) or value is None:
    return None
  try:
    return int(value)
  except (TypeError, ValueError):
    return None


def _median(values: list[float]) -> float:
  ordered = sorted(values)
  n = len(ordered)
  mid = n // 2
  if n % 2:
    return float(ordered[mid])
  return 0.5 * (float(ordered[mid - 1]) + float(ordered[mid]))


class GapLockGesture:
  """2.0 s MAIN hold. Long may be on, paused, or not yet taken."""

  def __init__(self) -> None:
    self.seq = 0
    self.cancel_hold = False
    self._holding = False
    self._t0 = 0.0
    self._fired = False

  @property
  def holding(self) -> bool:
    return self._holding

  def update(self, *, enabled: bool, use_pedal: bool, can_valid: bool,
             cruise_buttons, prev, now_ms: float, cruise_on: bool,
             qualify_press: bool, brake: bool = False) -> None:
    main = int(cruise_buttons) == int(CruiseButtons.MAIN)
    rising = main and int(prev) != int(CruiseButtons.MAIN)
    braked = brake is True
    if not (enabled and use_pedal and can_valid):
      self._reset_hold()
      return
    if rising and qualify_press and cruise_on and not braked:
      self._holding = True
      self._t0 = float(now_ms)
      self._fired = False
      self.cancel_hold = False
      return
    if not self._holding:
      self.cancel_hold = False
      return
    # Brake and cancel abort. A long pause during the hold does not:
    # the completed hold takes longitudinal itself.
    if (not main) or (not cruise_on) or braked:
      self._reset_hold()
      return
    elapsed = (float(now_ms) - self._t0) / 1000.0
    self.cancel_hold = elapsed >= GAP_LOCK_CANCEL_AFTER_S
    if elapsed >= GAP_LOCK_HOLD_S and not self._fired:
      self.seq = (int(self.seq) % 255) + 1
      self._fired = True

  def _reset_hold(self) -> None:
    self._holding = False
    self._fired = False
    self.cancel_hold = False


class GapLockLatch:
  """Planner-side meters + radar track. Seq 0 is unset; the first value is stored, not armed."""

  def __init__(self) -> None:
    self.track_id: int | None = None
    self.gap_m: float | None = None
    self.preview_m: float | None = None
    self.preview_track: int | None = None
    self.hud = HUD_NONE
    self._seen_seq: int | None = None
    self._samples: list[tuple[float, int, float]] = []
    self._t = 0.0
    self._absent = 0.0
    self._hud_until = 0.0

  def update(self, dt, *, enabled: bool, long_on: bool, seq, status: bool,
             radar: bool, track_id, d_rel, stalk_exit: bool, holding: bool = False) -> None:
    frame_dt = 0.05 if dt is None or float(dt) <= 0.0 else float(dt)
    self._t += frame_dt
    self._decay_hud()
    self._push_sample(status, radar, track_id, d_rel)

    seq_i = _u8(seq)
    if self._seen_seq is None:
      self._seen_seq = seq_i
      seq_changed = False
    else:
      seq_changed = seq_i != self._seen_seq

    if (not enabled) or (not long_on) or stalk_exit:
      self._clear(toast=False)
      if seq_changed:
        self._seen_seq = seq_i
      # While the stalk is still held, keep the running median as the
      # setpoint even if long is paused. The car is already on that
      # distance when the hold completes and long is taken.
      self._set_preview(holding and enabled and not stalk_exit, track_id)
      return

    if seq_changed:
      self._seen_seq = seq_i
      self._try_arm(status, radar, track_id)
    self._follow_track(frame_dt, status, radar, track_id)
    self._set_preview(holding and enabled and not stalk_exit, track_id)

  def _push_sample(self, status, radar, track_id, d_rel) -> None:
    cutoff = self._t - GAP_LOCK_MEDIAN_S
    self._samples = [s for s in self._samples if s[0] >= cutoff]
    if status is not True or radar is not True or d_rel is None:
      return
    tid = _track_id(track_id)
    if tid is None or tid < 0:
      return
    try:
      dist = float(d_rel)
    except (TypeError, ValueError):
      return
    if dist != dist:  # NaN
      return
    self._samples.append((self._t, tid, dist))

  def _set_preview(self, active: bool, track_id) -> None:
    self.preview_m = None
    self.preview_track = None
    if not active or self.gap_m is not None:
      return
    tid = _track_id(track_id)
    if tid is None or tid < 0:
      return
    vals = [d for _t, i, d in self._samples if i == tid]
    if not vals:
      return
    med = _median(vals)
    if med < GAP_LOCK_MIN_M or med > GAP_LOCK_MAX_M:
      return
    self.preview_m = float(med)
    self.preview_track = tid

  def _try_arm(self, status, radar, track_id) -> None:
    tid = _track_id(track_id)
    if status is not True or radar is not True or tid is None or tid < 0:
      self._pulse(HUD_UNAVAILABLE)
      return
    vals = [d for _t, i, d in self._samples if i == tid]
    if not vals:
      self._pulse(HUD_UNAVAILABLE)
      return
    med = _median(vals)
    if med < GAP_LOCK_MIN_M or med > GAP_LOCK_MAX_M:
      self._pulse(HUD_UNAVAILABLE)
      return
    self.track_id = tid
    self.gap_m = float(med)
    self._absent = 0.0
    self.hud = HUD_NONE
    self._hud_until = 0.0

  def _follow_track(self, dt: float, status, radar, track_id) -> None:
    if self.gap_m is None or self.track_id is None:
      self._absent = 0.0
      return
    tid = _track_id(track_id)
    if status is True and radar is True and tid == self.track_id:
      self._absent = 0.0
      return
    if status is True and (radar is not True or tid != self.track_id):
      self._clear(toast=True)
      return
    if status is True:
      return
    self._absent += dt
    if self._absent > LEAD_CLOSE_HOLD_S:
      self._clear(toast=True)

  def _clear(self, *, toast: bool) -> None:
    had = self.gap_m is not None
    self.track_id = None
    self.gap_m = None
    self.preview_m = None
    self.preview_track = None
    self._absent = 0.0
    if toast and had:
      self._pulse(HUD_LOST)
    elif not toast:
      self.hud = HUD_NONE
      self._hud_until = 0.0

  def _pulse(self, code: int) -> None:
    self.hud = int(code)
    self._hud_until = self._t + GAP_LOCK_HUD_S

  def _decay_hud(self) -> None:
    if self.hud and self._t >= self._hud_until:
      self.hud = HUD_NONE
