"""Multi-frame sign stacking (NAPSpeedSignStack, default Off).

Once the tracker has a live sign, speedsignd copies the same bank rect from
the next STACK_FRAMES-1 consecutive road frames. The read crop is cut from
each bank, frames 2..N are aligned to frame 1 by integer phase correlation on
Y (searched inside a small margin), then Y and UV are averaged and the reader
runs ONCE on the stack. Any failure falls back to the single-frame crop.

Oct 24 replay (111 labeled tracks, Oct 19 + Oct 23, v4 reader): single 85
right / 21 wrong / 5 missed; this stack 78 / 27 / 6; ECC-affine stack 68 /
28 / 15; sharpest-of-3 80 / 24 / 7. Averaging softens digit edges the
reader was trained on, so this ships Off.
"""
import numpy as np

from openpilot.selfdrive.speedsignd.nv12 import Nv12DetectCrop, subcrop_nv12

PARAM = "NAPSpeedSignStack"
STACK_FRAMES = 3
MAX_SHIFT = 24          # px search per frame (sign drift over 50-100 ms)
EXTRA_RECV_TIMEOUT_MS = 70
MIN_PEAK = 0.08         # normalized phase-corr peak; below = no lock, skip frame


def _expand(rect, m):
  x, y, w, h = (int(v) for v in rect)
  return (x - m, y - m, w + 2 * m, h + 2 * m)


def phase_shift(ref: np.ndarray, img: np.ndarray, max_shift: int = MAX_SHIFT) -> tuple[int, int, float] | None:
  """(dy, dx, peak) so that img[y+dy, x+dx] ~ ref[y, x]."""
  if ref.shape != img.shape or ref.ndim != 2 or min(ref.shape) < 8:
    return None
  a = ref.astype(np.float32); b = img.astype(np.float32)
  a -= a.mean(); b -= b.mean()
  win = np.outer(np.hanning(a.shape[0]), np.hanning(a.shape[1])).astype(np.float32)
  fa = np.fft.rfft2(a * win); fb = np.fft.rfft2(b * win)
  r = np.conj(fa) * fb
  r /= np.abs(r) + 1e-6
  c = np.fft.irfft2(r, s=a.shape)
  h, w = c.shape
  dy, dx = np.unravel_index(int(np.argmax(c)), c.shape)
  peak = float(c[dy, dx])
  dy = dy - h if dy > h // 2 else dy
  dx = dx - w if dx > w // 2 else dx
  if abs(dy) > max_shift or abs(dx) > max_shift:
    return None
  return int(dy), int(dx), peak


def stack_crop(banks: list, rect) -> tuple[Nv12DetectCrop | None, int]:
  """Stacked read crop for rect from frame banks (first = reference).

  Returns (crop, n_used). n_used == 1 means the plain single-frame crop.
  """
  if not banks or banks[0] is None:
    return None, 0
  base = subcrop_nv12(banks[0], rect)
  if base is None or len(banks) < 2:
    return base, 1 if base is not None else 0
  m = MAX_SHIFT
  ref_big = subcrop_nv12(banks[0], _expand(rect, m))
  if ref_big is None:
    return base, 1
  ys = [base.y.astype(np.float32)]
  uvs = [base.uv.astype(np.float32)] if base.uv is not None else None
  for bank in banks[1:]:
    if bank is None:
      continue
    big = subcrop_nv12(bank, _expand(rect, m))
    if big is None or big.y.shape != ref_big.y.shape:
      continue
    s = phase_shift(ref_big.y, big.y)
    if s is None or s[2] < MIN_PEAK:
      continue
    dy, dx = s[0] - s[0] % 2, s[1] - s[1] % 2  # keep NV12 UV aligned
    x, y, w, h = (int(v) for v in rect)
    c = subcrop_nv12(bank, (x + dx, y + dy, w, h))
    if c is None or c.y.shape != base.y.shape:
      continue
    ys.append(c.y.astype(np.float32))
    if uvs is not None:
      if c.uv is None or c.uv.shape != base.uv.shape:
        uvs = None
      else:
        uvs.append(c.uv.astype(np.float32))
  if len(ys) < 2:
    return base, 1
  y_out = np.clip(np.rint(np.mean(ys, axis=0)), 0, 255).astype(np.uint8)
  uv_out = base.uv
  if uvs is not None and len(uvs) == len(ys):
    uv_out = np.clip(np.rint(np.mean(uvs, axis=0)), 0, 255).astype(np.uint8)
  return Nv12DetectCrop(y=y_out, uv=uv_out, frame_w=base.frame_w, frame_h=base.frame_h, crop=base.crop), len(ys)


class StackConfig:
  """Cached param read (every PARAM_POLL_S). Missing key / error -> default Off."""
  PARAM_POLL_S = 5.0

  def __init__(self, default: bool = False):
    self.default = default
    self._val = default
    self._next = 0.0

  def enabled(self, params, now: float) -> bool:
    if now >= self._next:
      self._next = now + self.PARAM_POLL_S
      try:
        v = params.get(PARAM)
        self._val = self.default if v is None else bool(params.get_bool(PARAM))
      except Exception:
        self._val = self.default
    return self._val
