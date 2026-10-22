"""Oct 22 route 000001bb: stuck 30 prompt, answer never reached speedsignd."""
from openpilot.selfdrive.speedsignd import confirm as C


class TypedParams:
  """Mimics common/params_pyx.pyx: a JSON-typed key only accepts dict/list."""
  def __init__(self):
    self.d = {}

  def put(self, k, v):
    if k == C.ANSWER_PARAM and not isinstance(v, (dict, list)):
      raise TypeError(f"Type mismatch while writing param {k}")
    self.d[k] = v

  def get(self, k):
    return self.d.get(k)


def test_write_answer_reaches_plate_with_json_typed_param():
  prm = TypedParams()
  p = C.PlateState()
  pr = p.propose(30, 0.8, way_id=1227205099, heading=294.0, now=0.0)
  assert C.write_answer(pr.seq, 30, False, params=prm)
  assert p.answer(prm.get(C.ANSWER_PARAM))
  assert p.proposal is None


def test_real_params_accepts_answer():
  try:
    from openpilot.common.params import Params
    prm = Params()
  except Exception:
    return
  assert C.write_answer(7, 50, True, params=prm)
  v = prm.get(C.ANSWER_PARAM)
  assert v == {"seq": 7, "mph": 50, "yes": True}
  prm.remove(C.ANSWER_PARAM)


def test_mcu_and_hud_answer_clear_speedsignd_prompt(tmp_path, monkeypatch):
  from openpilot.selfdrive.speedsignd import mcu_prompt as M
  prm = TypedParams()
  monkeypatch.setattr(C, "write_answer", lambda seq, mph, yes, params=None, _w=C.write_answer: _w(seq, mph, yes, params=prm))
  monkeypatch.setattr("openpilot.selfdrive.speedsignd.two_stage.record_answer", lambda mph, yes, seq=None: None)
  plate = C.PlateState()
  pr = plate.propose(30, 0.8, way_id=5, heading=90.0, now=0.0)
  M.on_confirm_accuracy(30, False, {"seq": pr.seq, "way_id": 5, "heading": 90.0})
  assert plate.answer(prm.get(C.ANSWER_PARAM)) and plate.proposal is None


def test_no_is_not_reraised_for_same_read():
  p = C.PlateState()
  pr = p.propose(30, 0.8, way_id=9, read_id=("hold", 1), now=0.0)
  assert p.answer({"seq": pr.seq, "mph": 30, "yes": False})
  assert p.propose(30, 0.8, way_id=9, read_id=("hold", 1), now=1.0) is None   # same read: not again
  assert p.propose(30, 0.8, way_id=9, read_id=("hold", 2), now=2.0) is not None  # new read may ask


def test_expired_read_not_reraised():
  p = C.PlateState()
  p.propose(30, 0.8, way_id=9, read_id=("hold", 1), now=0.0)
  assert p.expire(61.0)
  assert p.propose(30, 0.8, way_id=9, read_id=("hold", 1), now=62.0) is None


def test_expire_age_way_distance():
  p = C.PlateState()
  p.propose(30, 0.8, way_id=1, lat=45.0, lon=-93.0, now=0.0)
  assert not p.expire(30.0, 1, 45.0, -93.0)
  assert p.expire(61.0, 1, 45.0, -93.0) and p.expired_why == "age" and p.proposal is None
  p.propose(50, 0.8, way_id=1, lat=45.0, lon=-93.0, now=100.0)
  assert not p.expire(105.0, 2)                                   # way change too soon to tap
  assert p.expire(116.0, 2) and p.expired_why == "way"
  p.propose(45, 0.8, way_id=0, lat=45.0, lon=-93.0, now=200.0)
  assert not p.expire(210.0, 0, 45.005, -93.0)                    # ~560 m
  assert p.expire(220.0, 0, 45.01, -93.0) and p.expired_why == "dist"
  assert p.confirmed_mph == 0                                     # expiry never confirms
