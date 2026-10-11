"""NV12 Y / RGB helpers for the ROAD camera."""
from __future__ import annotations

import numpy as np

from openpilot.selfdrive.speedsignd.nv12 import rgb_from_nv12, rgb_from_y, y_plane_from_nv12


def test_y_plane_from_nv12():
  class Buf:
    width = 4
    height = 2
    stride = 4
    data = bytes([1, 2, 3, 4, 5, 6, 7, 8]) + bytes(8)

  y = y_plane_from_nv12(Buf())
  assert y is not None
  assert y.shape == (2, 4)
  assert int(y[0, 0]) == 1


def test_rgb_from_y_replicates_luma():
  y = np.array([[10, 20], [30, 40]], np.uint8)
  rgb = rgb_from_y(y)
  assert rgb.shape == (2, 2, 3)
  assert int(rgb[0, 1, 2]) == 20


def test_rgb_from_nv12_gray_uv():
  # 2x2 gray: Y=128, U=V=128
  y = bytes([128, 128, 128, 128])
  uv = bytes([128, 128, 128, 128])

  class Buf:
    width = 2
    height = 2
    stride = 2
    data = y + uv

  rgb = rgb_from_nv12(Buf())
  assert rgb is not None
  assert rgb.shape == (2, 2, 3)
  assert 120 <= int(rgb[0, 0, 0]) <= 136
