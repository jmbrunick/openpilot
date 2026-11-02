"""Per-step monotonic timing for hardwared's publish loop."""

import time

# One iteration past this is logged with each step's duration so a stall
# names the blocking call. Publishing and the loop rate are unchanged.
SLOW_HW_LOOP_S = 1.0


class HardwareLoopTimer:
  def __init__(self, clock=None):
    self._clock = time.monotonic if clock is None else clock
    self._t0 = self._clock()
    self._prev = self._t0
    self.steps: dict[str, float] = {}

  def mark(self, name: str) -> None:
    now = self._clock()
    self.steps[name] = now - self._prev
    self._prev = now

  @property
  def total_s(self) -> float:
    return self._clock() - self._t0


def slow_loop_fields(timer: HardwareLoopTimer) -> dict[str, float] | None:
  """Step durations in milliseconds when the iteration exceeded 1 s.

  None when the loop was fast enough that nothing should be logged.
  """
  total = timer.total_s
  if total <= SLOW_HW_LOOP_S:
    return None
  fields = {f"{name}_ms": round(dt * 1000.0, 1) for name, dt in timer.steps.items()}
  fields["total_ms"] = round(total * 1000.0, 1)
  return fields
