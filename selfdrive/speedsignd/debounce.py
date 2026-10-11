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
from openpilot.selfdrive.speedsignd.hud import AGREE_READS, frame_reads
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
    hud: list[SpeedSign] = []
    for s in self.last_raw:
      pending, pair = frame_reads(s)
      if not pending and pair is None:
        continue
      hud.append(s)
      if pair is not None:
        self._hits.append((now, int(pair[0]), float(pair[1]), s.bbox))
        self._hits.append((now, int(pair[0]), float(pair[1]), s.bbox))
      for mph, conf in pending:
        self._hits.append((now, int(mph), float(conf), s.bbox))
    if not hud:
      return DebounceResult(hud=[], confirmed=[])
    counts: dict[int, list] = {}
    for hit in self._hits:
      counts.setdefault(int(hit[1]), []).append(hit)
    ready = [mph for mph, rows in counts.items() if len(rows) >= max(self.need, AGREE_READS)]
    if not ready:
      return DebounceResult(hud=hud, confirmed=[])
    best_mph = max(ready, key=lambda mph: max(row[2] for row in counts[mph]))
    top = max(counts[best_mph], key=lambda row: row[2])
    carrier = hud[-1]
    confirmed = [replace(carrier, mph=int(best_mph), conf=float(top[2]), bbox=top[3])]
    return DebounceResult(hud=hud, confirmed=confirmed)
