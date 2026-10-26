"""LIMIT-plate long-press popup (PR #391: speed limit source selection).

Hold the on-screen LIMIT plate for 2 s:
  * Tesla's and OSM's limits side by side. Tapping either one sets it right away
    as the limit for this road segment (same as Save with that value).
  * up / down arrows change the value in 5 mph steps; Save writes it.
  * Clear removes the saved limit where the car is now (NAPSpeedSignClearOverride).
  * Source: TESLA / OSM flips the global source toggle (NAPSpeedLimitPreferOsm).
Saved values go to mapd as NAPSpeedLimitManual and become a confirmed override row
from the car's spot on, both directions, never behind (#387); they stick until cleared.
Touches outside the panel close it. The model is pyray-free so it can be unit tested.
"""
from __future__ import annotations

import threading
import time
from collections.abc import Callable

from openpilot.selfdrive.mapd.limit_source import PARAM_PREFER_OSM, request_manual, step_mph

HOLD_S = 2.0
AUTO_CLOSE_S = 20.0
CLEAR_PARAM = "NAPSpeedSignClearOverride"

Rect = tuple[float, float, float, float]


def _inside(x: float, y: float, r: Rect) -> bool:
  return r[2] > 0 and r[3] > 0 and r[0] <= x <= r[0] + r[2] and r[1] <= y <= r[1] + r[3]


def popup_rects(sx: float, sy: float, sw: float, sh: float) -> dict[str, Rect]:
  """Panel + buttons for a screen rect (tici 2160x1080 class; scales with width)."""
  pw, ph = min(900.0, sw - 80.0), 560.0
  px, py = sx + (sw - pw) / 2.0, sy + max(40.0, (sh - ph) / 2.0)
  pad, gap = 30.0, 24.0
  col = (pw - 2 * pad - gap) / 2.0
  r: dict[str, Rect] = {"panel": (px, py, pw, ph)}
  y1 = py + 70.0
  r["tesla"] = (px + pad, y1, col, 130.0)
  r["osm"] = (px + pad + col + gap, y1, col, 130.0)
  y2 = y1 + 130.0 + gap
  bw = 130.0
  r["down"] = (px + pad, y2, bw, 120.0)
  r["value"] = (px + pad + bw + gap, y2, pw - 2 * pad - 2 * bw - 2 * gap, 120.0)
  r["up"] = (px + pw - pad - bw, y2, bw, 120.0)
  y3 = y2 + 120.0 + gap
  qw = (pw - 2 * pad - 3 * gap) / 4.0
  for i, k in enumerate(("save", "clear", "source", "close")):
    r[k] = (px + pad + i * (qw + gap), y3, qw, 110.0)
  return r


class LimitPopupModel:
  """Long-press timing, popup value and actions. writer(kind, value) does the param I/O."""

  def __init__(self, writer: Callable[[str, object], None] | None = None, clock: Callable[[], float] = time.monotonic):
    self._clock = clock
    self._writer = writer or _thread_writer
    self.press_t: float | None = None
    self.is_open = False
    self.opened_t = 0.0
    self.value = 0
    self.tesla_mph = 0
    self.osm_mph = 0
    self.prefer_osm = False
    self.note = ""

  # --- long press on the LIMIT plate ---
  def press_sign(self) -> None:
    if not self.is_open:
      self.press_t = self._clock()

  def release(self) -> None:
    self.press_t = None

  def tick(self, pressed: bool, current_mph: int, tesla_mph: int, osm_mph: int, prefer_osm: bool) -> bool:
    """Call every frame. Returns True on the frame the popup opens."""
    now = self._clock()
    self.tesla_mph, self.osm_mph, self.prefer_osm = int(tesla_mph), int(osm_mph), bool(prefer_osm)
    if self.is_open and now - self.opened_t > AUTO_CLOSE_S:
      self.close()
    if not pressed:
      self.press_t = None
    if not self.is_open and self.press_t is not None and now - self.press_t >= HOLD_S:
      self.press_t = None
      self.is_open = True
      self.opened_t = now
      v = int(round(int(current_mph) / 5.0)) * 5
      self.value = max(5, min(85, v)) if v > 0 else 25
      self.note = ""
      return True
    return False

  def close(self) -> None:
    self.is_open = False
    self.press_t = None

  # --- popup taps ---
  def tap(self, name: str | None) -> str | None:
    """Apply a tap on a popup control (None = outside the panel). Returns the action taken."""
    if not self.is_open:
      return None
    self.opened_t = self._clock()
    if name is None or name == "close":
      self.close()
      return "close"
    if name in ("up", "down"):
      self.value = step_mph(self.value, name == "up")
      return name
    if name in ("tesla", "osm"):
      mph = self.tesla_mph if name == "tesla" else self.osm_mph
      if mph <= 0:
        self.note = f"No {name.upper()} limit here"
        return None
      mph = max(5, min(85, int(round(mph / 5.0)) * 5))
      self.value = mph
      self._writer("manual", mph)
      self.close()
      return name
    if name == "save":
      self._writer("manual", int(self.value))
      self.close()
      return "save"
    if name == "clear":
      self._writer("clear", True)
      self.close()
      return "clear"
    if name == "source":
      self.prefer_osm = not self.prefer_osm
      self._writer("source", self.prefer_osm)
      return "source"
    return None


def _write(kind: str, value) -> None:
  from openpilot.common.params import Params
  params = Params()
  if kind == "manual":
    request_manual(int(value), time.monotonic(), params)
  elif kind == "clear":
    params.put_bool(CLEAR_PARAM, True)
  elif kind == "source":
    params.put_bool(PARAM_PREFER_OSM, bool(value))


def _thread_writer(kind: str, value) -> None:
  # Params I/O off the UI thread so a slow write never stalls a frame.
  def run():
    try:
      _write(kind, value)
    except Exception:
      pass
  threading.Thread(target=run, daemon=True, name="limit-popup").start()


def hit_name(x: float, y: float, rects: dict[str, Rect]) -> str | None:
  for k in ("tesla", "osm", "down", "up", "save", "clear", "source", "close"):
    if _inside(x, y, rects[k]):
      return k
  return "panel" if _inside(x, y, rects["panel"]) else None
