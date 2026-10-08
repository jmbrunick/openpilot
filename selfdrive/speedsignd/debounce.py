"""Temporal confirm so a single noisy frame does not write JSONL.

HUD lights on the first *accepted* in-threshold hit (65/70 need refine).
At 1 Hz with skip-on-overrun, two agreeing frames often cannot land while
a roadside R2-1 is still in view — especially after WAIT, when the first
useful infer may already be a second late. JSONL keeps the two-hit confirm.
Empty-road YOLO max conf is ~0, so a single >= min_conf hit is already a
real plate, not asphalt.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

from openpilot.selfdrive.speedsignd.detect_types import SpeedSign
from openpilot.selfdrive.speedsignd.hud import accepted_hud_sign
from openpilot.selfdrive.speedsignd.weights_manifest import DEBOUNCE_HITS, DEBOUNCE_WINDOW_S, YOLO_MIN_CONF


@dataclass(frozen=True)
class DebounceResult:
  """HUD may accept a first hit; JSONL / confirmed needs `need` hits."""
  hud: list[SpeedSign]
  confirmed: list[SpeedSign]


@dataclass
class SignDebounce:
  """Track hits. HUD = first accepted mph; JSONL = `need` of the same mph."""

  need: int = DEBOUNCE_HITS
  window_s: float = DEBOUNCE_WINDOW_S
  min_conf: float = YOLO_MIN_CONF
  last_raw: list[SpeedSign] = field(default_factory=list)
  _hits: list[tuple[float, int, float, tuple[int, int, int, int]]] = field(default_factory=list)

  def update(self, signs: list[SpeedSign], now: float) -> list[SpeedSign]:
    """Back-compat: two-hit / JSONL confirm only."""
    return self.update_split(signs, now).confirmed

  def update_split(self, signs: list[SpeedSign], now: float) -> DebounceResult:
    # last_raw keeps detector mph (including unconfirmed 65) for infer logs.
    self.last_raw = [s for s in signs if s.conf >= self.min_conf and s.mph > 0]
    self._hits = [h for h in self._hits if now - h[0] <= self.window_s]
    accepted: list[SpeedSign] = []
    for s in self.last_raw:
      a = accepted_hud_sign(s)
      if a is not None:
        accepted.append(a)
    best = None
    for s in accepted:
      if best is None or s.conf > best.conf:
        best = s
    if best is None:
      return DebounceResult(hud=[], confirmed=[])
    self._hits.append((now, int(best.mph), float(best.conf), best.bbox))
    rows = [h for h in self._hits if h[1] == best.mph]
    hud = [best]
    if len(rows) < self.need:
      return DebounceResult(hud=hud, confirmed=[])
    top = max(rows, key=lambda r: r[2])
    confirmed = [replace(best, conf=float(top[2]), bbox=top[3])]
    return DebounceResult(hud=confirmed, confirmed=confirmed)
