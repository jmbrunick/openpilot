import numpy as np

from openpilot.selfdrive.speedsignd.nv12 import Nv12DetectCrop
from openpilot.selfdrive.speedsignd.stack import StackConfig, phase_shift, stack_crop


def _bank(img, x0=100, y0=100):
  h, w = img.shape
  return Nv12DetectCrop(y=img, uv=np.full((h // 2, w), 128, np.uint8), frame_w=1928, frame_h=1208, crop=(x0, y0, w, h))


def _scene(seed=0):
  rng = np.random.default_rng(seed)
  img = rng.integers(0, 40, (200, 240)).astype(np.uint8)
  img[80:140, 100:150] = 230
  img[95:125, 115:135] = 20
  return img


def test_phase_shift_recovers_motion():
  a = _scene()
  b = np.roll(a, (6, -10), axis=(0, 1))
  dy, dx, peak = phase_shift(a[40:180, 60:200], b[40:180, 60:200])
  assert (dy, dx) == (6, -10) and peak > 0.1


def test_stack_aligns_and_averages_noise():
  rng = np.random.default_rng(1)
  clean = _scene()
  frames = []
  for i, sh in enumerate([(0, 0), (4, 6), (8, 12)]):
    n = np.clip(np.roll(clean, sh, axis=(0, 1)).astype(int) + rng.normal(0, 25, clean.shape), 0, 255).astype(np.uint8)
    frames.append(_bank(n))
  rect = (100 + 90, 100 + 70, 70, 80)
  out, n = stack_crop(frames, rect)
  assert n == 3
  ref = clean[70:150, 90:160].astype(float)
  single = frames[0].y[70:150, 90:160].astype(float)
  assert np.abs(out.y - ref).mean() < 0.75 * np.abs(single - ref).mean()


def test_single_bank_is_plain_crop():
  b = _bank(_scene())
  out, n = stack_crop([b], (190, 170, 70, 80))
  assert n == 1 and np.array_equal(out.y, b.y[70:150, 90:160])


def test_unlockable_frame_falls_back():
  b = _bank(_scene())
  junk = _bank(np.random.default_rng(5).integers(0, 255, (200, 240)).astype(np.uint8))
  out, n = stack_crop([b, junk], (190, 170, 70, 80))
  assert n == 1


def test_config_default_on_and_respects_off():
  class P:
    def __init__(self, v): self.v = v
    def get(self, k): return self.v
    def get_bool(self, k): return self.v == b"1"
  assert StackConfig().enabled(P(None), 0.0)
  assert not StackConfig().enabled(P(b"0"), 0.0)
