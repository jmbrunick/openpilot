"""Other-sign reader: optional weights, per-family rules, display-only."""
import pathlib
import re

import numpy as np

from openpilot.selfdrive.speedsignd import others as O
from openpilot.selfdrive.speedsignd.nv12 import Nv12DetectCrop  # noqa: F401  (shape reference)


class _Crop:
  def __init__(self, h=64, w=64):
    self.y = np.full((h, w), 120, np.uint8)
    self.uv = np.full((h // 2, w), 128, np.uint8)


class _Fake:
  def __init__(self, seq):
    self.ok = True
    self.seq = list(seq)

  def read(self, crop):
    return self.seq.pop(0) if self.seq else None


class _Md:
  class velocity:
    x = [20.0, 12.0, 4.0, 0.4]


def test_missing_weights_is_noop():
  r = O.OnnxFamilyReader("/nonexistent/other_signs.onnx")
  assert not r.ok and r.read(_Crop()) is None
  o = O.OtherSigns(r)
  o.set_enabled(True)
  assert not o.active
  st = o.on_read(_Crop(), 1, stop_plan=True)
  assert st.reader_missing and st.family == "" and not st.hud


def test_disabled_param_is_noop():
  o = O.OtherSigns(_Fake([O.FamilyRead("stop", 0.95)]))
  o.set_enabled(False)
  assert o.on_read(_Crop(), 1, stop_plan=True).family == ""


def test_stop_needs_two_reads_or_solo_with_model_stop_plan():
  o = O.OtherSigns(_Fake([O.FamilyRead("stop", 0.9)]))
  o.set_enabled(True)
  assert not o.on_read(_Crop(), 1, stop_plan=False).confirmed
  o = O.OtherSigns(_Fake([O.FamilyRead("stop", 0.9)]))
  o.set_enabled(True)
  st = o.on_read(_Crop(), 1, stop_plan=True)
  assert st.confirmed and st.hud and st.model_stop_plan
  o = O.OtherSigns(_Fake([O.FamilyRead("stop", 0.6), O.FamilyRead("stop", 0.6)]))
  o.set_enabled(True)
  o.on_read(_Crop(), 2, stop_plan=False)
  assert o.on_read(_Crop(), 2, stop_plan=False).confirmed


def test_yield_dne_need_two_agreeing_reads():
  for fam in ("yield", "do_not_enter"):
    assert O.decide(fam, 1, 0.99, True) == (False, False)
    assert O.decide(fam, 2, 0.6, False) == (True, True)


def test_warnings_hud_on_one_confident_read_log_on_two():
  for fam in ("school", "railroad", "curve"):
    assert O.decide(fam, 1, 0.75, False) == (True, False)
    assert O.decide(fam, 1, 0.6, False) == (False, False)
    assert O.decide(fam, 2, 0.6, False) == (True, True)


def test_disagreeing_reads_do_not_confirm_and_curve_detail():
  o = O.OtherSigns(_Fake([O.FamilyRead("curve", 0.8, "left"), O.FamilyRead("curve", 0.8, "right")]))
  o.set_enabled(True)
  o.on_read(_Crop(), 3, stop_plan=False)
  st = o.on_read(_Crop(), 3, stop_plan=False)
  assert st.family == "curve" and st.reads == 1 and not st.confirmed
  assert O.parse_class("curve_left", 0.8) == O.FamilyRead("curve", 0.8, "left")
  assert O.parse_class("none", 0.9) is None


def test_model_stop_plan_cross_check():
  assert O.model_stop_plan(_Md, 15.0)
  assert not O.model_stop_plan(_Md, 1.0)
  assert not O.model_stop_plan(object(), 15.0)


def test_rgb_square_shape():
  b = O._rgb_square(_Crop(80, 50), 64)
  assert b.shape == (1, 3, 64, 64) and 0.0 <= b.min() and b.max() <= 1.0


def test_others_never_touches_control():
  src = pathlib.Path(O.__file__).read_text()
  for bad in ("longitudinalPlan", "carControl", "PubMaster", "messaging", "accel", "controlsd"):
    assert bad not in re.sub(r'""".*?"""', "", src, flags=re.S), bad
  main = (pathlib.Path(O.__file__).parent / "speedsignd.py").read_text()
  pm = re.search(r"PubMaster\(\[([^\]]*)\]\)", main).group(1)
  assert pm.replace(" ", "") == 'SERVICE_NAME,"otherSignsNAP"'


def test_no_weights_shipped():
  d = pathlib.Path(O.__file__).parent
  assert not list(d.rglob("*.onnx"))
