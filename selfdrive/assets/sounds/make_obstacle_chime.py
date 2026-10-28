#!/usr/bin/env python3
"""Write animal_chime.wav: two rising high notes, played twice.

Stock engage is one decaying sine near 1661 Hz. Stock prompt is a
different tone. This is G6 (1568 Hz) then C7 (2093 Hz), each with a
short attack, a quick decay, and the 2nd and 3rd harmonics, then the
pair again. 48 kHz mono 16-bit, about 1.0 s, peak near full scale, so
the small speaker can actually say it.
"""
from __future__ import annotations

import math
import struct
import wave
from pathlib import Path

SAMPLE_RATE = 48000
PEAK = 0.98


def _note(freq: float, duration: float, decay: float) -> list[float]:
  n = int(duration * SAMPLE_RATE)
  attack = 0.006
  out = []
  for i in range(n):
    t = i / SAMPLE_RATE
    if t < attack:
      env = 0.5 * (1.0 - math.cos(math.pi * t / attack))
    else:
      env = math.exp(-(t - attack) / decay)
    sample = math.sin(2.0 * math.pi * freq * t)
    sample += 0.55 * math.sin(2.0 * math.pi * 2.0 * freq * t)
    sample += 0.28 * math.sin(2.0 * math.pi * 3.0 * freq * t)
    out.append(env * sample)
  return out


def _phrase() -> list[float]:
  gap = [0.0] * int(0.045 * SAMPLE_RATE)
  pause = [0.0] * int(0.07 * SAMPLE_RATE)
  # G6 then C7. Rising, so it is not the falling engage tone.
  return _note(1567.98, 0.18, 0.055) + gap + _note(2093.00, 0.22, 0.06) + pause


def build() -> list[float]:
  raw = _phrase() + _phrase()
  peak = max(abs(s) for s in raw) or 1.0
  return [PEAK * s / peak for s in raw]


def write(path: Path) -> None:
  samples = build()
  pcm = b"".join(struct.pack("<h", max(-32767, min(32767, int(s * 32767.0)))) for s in samples)
  with wave.open(str(path), "w") as handle:
    handle.setnchannels(1)
    handle.setsampwidth(2)
    handle.setframerate(SAMPLE_RATE)
    handle.writeframes(pcm)


if __name__ == "__main__":
  dest = Path(__file__).with_name("animal_chime.wav")
  write(dest)
  audio = build()
  print(dest, len(audio) / SAMPLE_RATE, max(abs(s) for s in audio))
