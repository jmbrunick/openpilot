"""Temporal confirm so a single noisy frame does not light the SIGN HUD."""
from __future__ import annotations

from dataclasses import dataclass, field

from openpilot.selfdrive.speedsignd.detect_types import SpeedSign
from openpilot.selfdrive.speedsignd.weights_manifest import DEBOUNCE_HITS, DEBOUNCE_WINDOW_S, YOLO_MIN_CONF


@dataclass
class SignDebounce:
  """Require `need` detections of the same mph inside `window_s`."""

  need: int = DEBOUNCE_HITS
  window_s: float = DEBOUNCE_WINDOW_S
  min_conf: float = YOLO_MIN_CONF
  _hits: list[tuple[float, int, float, tuple[int, int, int, int]]] = field(default_factory=list)

  def update(self, signs: list[SpeedSign], now: float) -> list[SpeedSign]:
    self._hits = [h for h in self._hits if now - h[0] <= self.window_s]
    best = None
    for s in signs:
      if s.conf < self.min_conf or s.mph <= 0:
        continue
      if best is None or s.conf > best.conf:
        best = s
    if best is None:
      return []
    self._hits.append((now, int(best.mph), float(best.conf), best.bbox))
    rows = [h for h in self._hits if h[1] == best.mph]
    if len(rows) < self.need:
      return []
    top = max(rows, key=lambda r: r[2])
    return [SpeedSign(mph=int(best.mph), conf=float(top[2]), bbox=top[3])]
