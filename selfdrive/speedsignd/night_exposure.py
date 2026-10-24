"""Night short-exposure request (NAPSpeedSignShortExposure, default Off).

While the tracker has a live sign candidate, write a CLOCK_MONOTONIC deadline
to /dev/shm. camerad (road cam only) then trades exposure time for analog gain
at the same EV for a bounded run. camerad only shortens when the AE time is
above its cap, so this does nothing in daylight.
"""
import os
import time

PARAM = "NAPSpeedSignShortExposure"
REQUEST_PATH = "/dev/shm/nap_speedsign_exp"
WINDOW_S = 1.5        # each request; camerad rejects > 3 s
PARAM_POLL_S = 5.0
WRITE_MIN_S = 0.5


class NightExposure:
  def __init__(self, params, path: str = REQUEST_PATH):
    self.params = params
    self.path = path
    self.enabled = False
    self._next_poll = 0.0
    self._last_write = -1e9

  def update(self, has_track: bool, now: float | None = None) -> bool:
    now = time.monotonic() if now is None else now
    if now >= self._next_poll:
      try:
        self.enabled = bool(self.params.get_bool(PARAM))
      except Exception:
        self.enabled = False
      self._next_poll = now + PARAM_POLL_S
    if not (self.enabled and has_track) or now - self._last_write < WRITE_MIN_S:
      return False
    try:
      tmp = self.path + ".tmp"
      with open(tmp, "w") as f:
        f.write(str(int((now + WINDOW_S) * 1e9)))
      os.replace(tmp, self.path)
    except OSError:
      return False
    self._last_write = now
    return True
