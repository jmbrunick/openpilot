"""LIMIT-plate marking for a speed-sign Temp (mapd/temp_limit.py).

While liveMapDataNAP.tempActive, the LIMIT plate is tinted orange with a TEMP
band (work-zone colors). When a Temp ends, a short line under the plate says
why for END_NOTE_S. Display only.
"""
from __future__ import annotations

import time

import pyray as rl
from openpilot.selfdrive.mapd.temp_limit import END_NOTE_S, end_text
from openpilot.system.ui.lib.text_measure import measure_text_cached

TEMP_TAG = "TEMP"
TEMP_FILL = rl.Color(255, 226, 190, 255)
TEMP_ORANGE = rl.Color(232, 112, 16, 255)
TEMP_INK = rl.BLACK
NOTE_INK = rl.Color(255, 190, 120, 255)


class TempEndNote:
  """'Temp ended: <why>' for END_NOTE_S after liveMapDataNAP.tempEndSeq moves."""

  def __init__(self, clock=time.monotonic) -> None:
    self._clock = clock
    self._seq: int | None = None
    self._text = ""
    self._t = -1e9

  def update(self, end_seq: int, reason: str) -> str | None:
    now = self._clock()
    end_seq = int(end_seq)
    if self._seq is None:
      self._seq = end_seq          # an end from before the UI started is not news
    elif end_seq != self._seq:
      self._seq = end_seq
      self._text = f"Temp ended: {end_text(reason)}"
      self._t = now
    if self._text and now - self._t <= END_NOTE_S:
      return self._text
    return None


def temp_from_md(md) -> tuple[bool, int]:
  try:
    return bool(md.tempActive), int(md.tempMph)
  except Exception:
    return False, 0


def draw_temp_plate(sign: rl.Rectangle, font, *, tag_size: int = 28) -> None:
  """Orange tint + TEMP band on a LIMIT plate (call before the label / number)."""
  rl.draw_rectangle_rounded(sign, 0.12, 8, TEMP_FILL)
  rl.draw_rectangle_rounded_lines_ex(sign, 0.12, 8, 4, TEMP_ORANGE)
  band_h = max(18.0, sign.height * 0.2)
  band = rl.Rectangle(sign.x + 6, sign.y + sign.height - band_h - 6, sign.width - 12, band_h)
  rl.draw_rectangle_rounded(band, 0.3, 6, TEMP_ORANGE)
  w = measure_text_cached(font, TEMP_TAG, tag_size).x
  rl.draw_text_ex(font, TEMP_TAG, rl.Vector2(band.x + (band.width - w) / 2, band.y + (band.height - tag_size) / 2),
                  tag_size, 0, TEMP_INK)


def draw_end_note(font, text: str, x: float, y: float, size: int = 26) -> None:
  rl.draw_text_ex(font, text, rl.Vector2(x, y), size, 0, NOTE_INK)
