import numpy as np
import pytest

from openpilot.selfdrive.nap.animal_detect.postprocess import decode


def _row(cx, cy, w, h, obj, cls):
  r = np.zeros(9, np.float32)
  r[:5] = (cx, cy, w, h, obj)
  r[5 + cls] = 0.9
  return r


def test_decode_nms_and_classes():
  out = np.zeros((2, 50, 9), np.float32)
  out[0, 0] = _row(100, 100, 40, 30, 0.9, 1)
  out[0, 1] = _row(102, 101, 40, 30, 0.8, 1)    # duplicate
  out[0, 2] = _row(400, 200, 20, 50, 0.7, 0)
  out[0, 3] = _row(500, 200, 20, 50, 0.1, 0)    # below threshold
  res = decode(out)
  assert [r[0] for r in res[0]] == ["deer", "person"] and res[1] == []
  x0, y0, x1, y1 = res[0][0][2]
  assert (x0, y0, x1, y1) == (80, 85, 120, 115)


def test_nan_rows_ignored():
  out = np.full((1, 3, 9), np.nan, np.float32)
  assert decode(out) == [[]]


def test_bad_shape_raises():
  with pytest.raises(ValueError):
    decode(np.zeros((1, 5, 7)))
