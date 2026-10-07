#!/usr/bin/env python3
"""Write animal_chime.wav: a two-note descending fourth.

Stock engage is one decaying sine near 1661 Hz. Stock disengage is one
tone near 1318 Hz. This is D5 (587 Hz) then G4 (392 Hz), with a short
gap, a raised-cosine attack, and a little second harmonic. 48 kHz mono
16-bit, about 0.46 s, so soundd can play it beside the other alerts.
"""
from __future__ import annotations

import math
import struct
import wave
from pathlib import Path

SAMPLE_RATE = 48000
FULL_SCALE = 0.62


def _tone(freq: float, duration: float) -> list[float]:
  n = int(duration * SAMPLE_RATE)
  attack = 0.012
  release = 0.028
  out = []
  for i in range(n):
    t = i / SAMPLE_RATE
    if t < attack:
      env = 0.5 * (1.0 - math.cos(math.pi * t / attack))
    elif t > duration - release:
      env = 0.5 * (1.0 - math.cos(math.pi * (duration - t) / release))
    else:
      env = 1.0
    sample = math.sin(2.0 * math.pi * freq * t)
    sample += 0.16 * math.sin(2.0 * math.pi * 2.0 * freq * t)
    out.append(FULL_SCALE * env * sample / 1.16)
  return out


def build() -> list[float]:
  gap = [0.0] * int(0.05 * SAMPLE_RATE)
  return _tone(587.33, 0.14) + gap + _tone(392.00, 0.27)


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
  print(dest, len(build()) / SAMPLE_RATE)
