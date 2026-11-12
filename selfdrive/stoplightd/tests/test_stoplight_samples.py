"""Lamp detector + tracker on real Scallywag frames (see samples/README.md)."""
import base64
import io
import json
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.stoplightd import tracker as tk
from openpilot.selfdrive.stoplightd.detect import GREEN, RED, Lamp, find_lamps

HERE = Path(__file__).resolve().parent
SAMPLES = HERE / "samples"
MANIFEST = json.loads((SAMPLES / "manifest.json").read_text())
PAIRS = [e for e in MANIFEST if e["label"] in ("red", "green")]
NEAR_PX = 14.0


def _planes(name):
  """JPEG -> BT.601 limited-range Y (full) and U/V (half), like camerad NV12."""
  from PIL import Image
  raw = base64.b64decode((SAMPLES / name).read_bytes())
  rgb = np.asarray(Image.open(io.BytesIO(raw)).convert("RGB")).astype(np.float32)
  r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
  y = 16 + 0.257 * r + 0.504 * g + 0.098 * b
  u = 128 - 0.148 * r - 0.291 * g + 0.439 * b
  v = 128 + 0.439 * r - 0.368 * g - 0.071 * b
  h, w = y.shape
  u = u.reshape(h // 2, 2, w // 2, 2).mean((1, 3))
  v = v.reshape(h // 2, 2, w // 2, 2).mean((1, 3))

  def q(a):
    return np.clip(np.rint(a), 0, 255).astype(np.uint8)
  return q(y), q(u), q(v)


def _lamps(entry):
  y, u, v = _planes(entry["file"])
  return [lp for lp in find_lamps(y[::2, ::2], u, v, roi_top=0.0, roi_bottom=1.0) if tk.lamp_ok(lp)]


def _near(lamps, color, xy):
  return [lp for lp in lamps if lp.color == color and abs(lp.x - xy[0]) <= NEAR_PX and abs(lp.y - xy[1]) <= NEAR_PX]


@pytest.mark.parametrize("entry", PAIRS, ids=lambda e: e["file"][:-8])
def test_real_lamp_color(entry):
  lamps = _lamps(entry)
  if entry["label"] == "red":
    assert _near(lamps, RED, entry["red_xy"]), lamps
    assert not _near(lamps, GREEN, entry["green_xy"]), lamps
  else:
    assert _near(lamps, GREEN, entry["green_xy"]), lamps
    assert not _near(lamps, RED, entry["red_xy"]), lamps


def _play(red_lamps, green_lamps, n_red=6, n_green=4, dt=0.25):
  tr = tk.StoplightTracker()
  outs = []
  t = 100.0
  for _ in range(n_red):
    outs.append(tr.update(t, red_lamps))
    t += dt
  for _ in range(n_green):
    outs.append(tr.update(t, green_lamps))
    t += dt
  return outs


@pytest.mark.parametrize("stop", sorted({e["stop"] for e in PAIRS}))
def test_real_pair_dings_once(stop):
  red = next(e for e in PAIRS if e["stop"] == stop and e["label"] == "red")
  green = next(e for e in PAIRS if e["stop"] == stop and e["label"] == "green")
  outs = _play(_lamps(red), _lamps(green))
  assert any(o.red for o in outs[:6]), "red flag never raised"
  assert sum(o.chime for o in outs) == 1
  assert outs[-1].green and not outs[-1].red


def test_billboard_flip_is_quiet():
  red = next(e for e in MANIFEST if e["label"] == "billboard_red")
  green = next(e for e in MANIFEST if e["label"] == "billboard_green")
  outs = _play(_lamps(red), _lamps(green))
  assert not any(o.chime for o in outs)


def _seq_lamps(rows):
  for t, ls in rows:
    yield t, [Lamp(RED if c == "r" else GREEN, *a) for c, *a in ls]


SEQ = json.loads((HERE / "sequences.json").read_text())


@pytest.mark.parametrize("seq", SEQ, ids=lambda s: f"{s['label']}_{s['route'][:8]}_{s['t_end']}")
def test_real_stop_sequences(seq):
  tr = tk.StoplightTracker()
  chimes = [t for t, lamps in _seq_lamps(seq["rows"]) if tr.update(t, lamps).chime]
  if seq["label"] == "pos":
    assert len(chimes) == 1
    # Before the car moved (t is relative to the move; the gate closes then).
    assert chimes[0] <= 0.2
  elif seq["label"] in ("neg", "red_no_green"):
    assert chimes == []
  else:
    assert len(chimes) <= 1


def test_sequence_totals():
  counts = {}
  for s in SEQ:
    counts[s["label"]] = counts.get(s["label"], 0) + 1
  assert counts.get("pos", 0) >= 20
  assert counts.get("neg", 0) + counts.get("red_no_green", 0) >= 20
