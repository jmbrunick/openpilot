"""Triple-tap detector for the hidden NAP settings popup.

Gesture
-------
A **sliding window** of 3 taps within ``WINDOW_S`` (1.0 s): the newest tap
counts if at least two earlier taps also fall in ``[now - 1.0, now]``.
Re-taps on NAP while already on the NAP panel still count — TICI records
them in ``SettingsLayout._handle_mouse_release``, not ``set_current_panel``.

Dismiss (TICI side popup / mici overlay)
----------------------------------------
Primary: tap outside the popup card. Also: the card's close control, or
tap the NAP sidebar / nap button again (TICI). Escape dismisses on TICI.
"""
from __future__ import annotations


WINDOW_S = 1.0
TAP_COUNT = 3
# mici: open the normal NAP page this long after the last tap if the burst
# never reached TAP_COUNT. Keeps single-tap navigation usable while still
# allowing a quick triple-tap. TICI has no delay (sidebar stays visible).
MICI_NAP_OPEN_DELAY_S = 0.40


class TripleTapDetector:
  """Count taps in a sliding time window. No GUI dependency."""

  def __init__(self, window_s: float = WINDOW_S, tap_count: int = TAP_COUNT):
    if window_s <= 0:
      raise ValueError("window_s must be > 0")
    if tap_count < 1:
      raise ValueError("tap_count must be >= 1")
    self.window_s = window_s
    self.tap_count = tap_count
    self._taps: list[float] = []

  def tap(self, now: float) -> bool:
    """Record a tap at ``now`` (monotonic seconds).

    Returns True if this tap completes ``tap_count`` taps inside ``window_s``.
    The buffer is cleared after a successful burst so the next sequence starts
    fresh.
    """
    cutoff = now - self.window_s
    self._taps = [t for t in self._taps if t >= cutoff]
    self._taps.append(now)
    if len(self._taps) >= self.tap_count:
      self.reset()
      return True
    return False

  def reset(self) -> None:
    self._taps.clear()

  @property
  def pending_count(self) -> int:
    return len(self._taps)
