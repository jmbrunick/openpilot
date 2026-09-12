"""NV12 Y / RGB helpers for the ROAD camera."""
from __future__ import annotations

import numpy as np

from openpilot.selfdrive.speedsignd.nv12 import (
  nv12_uv_offset,
  rgb_from_nv12,
  rgb_from_y,
  y_plane_from_nv12,
)


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
  copied = y_plane_from_nv12(Buf(), copy=True)
  assert copied is not None and copied.shape == (2, 4)
  copied[0, 0] = 99
  assert int(y_plane_from_nv12(Buf())[0, 0]) == 1


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


def test_nv12_uv_offset_prefers_buf_field():
  class Buf:
    uv_offset = 2048 * 1216

  assert nv12_uv_offset(Buf(), stride=2048, height=1208) == 2048 * 1216
  assert nv12_uv_offset(type("B", (), {})(), stride=2048, height=1208) == 2048 * 1208


def test_rgb_from_nv12_uses_uv_offset_not_visible_height():
  """3X Venus pad: UV starts after ALIGN(height,32) rows, not height.

  Zeros in the Y pad must not be read as U=V=0 (that turns gray into green).
  """
  width, height, stride = 4, 2, 4
  y = bytes([128] * (stride * height))
  pad = bytes([0] * 8)
  uv = bytes([128, 128, 128, 128])

  class Buf:
    pass

  Buf.width = width
  Buf.height = height
  Buf.stride = stride
  Buf.uv_offset = stride * height + len(pad)
  Buf.data = y + pad + uv

  rgb = rgb_from_nv12(Buf())
  assert rgb is not None
  assert 120 <= int(rgb[0, 0, 0]) <= 136
  assert 120 <= int(rgb[0, 0, 1]) <= 136
  assert 120 <= int(rgb[0, 0, 2]) <= 136
