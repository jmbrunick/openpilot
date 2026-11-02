from openpilot.selfdrive.speedsignd.confirm import ANSWER_PARAM, PlateState, write_answer


class P:
  """Mimics Params.put on a JSON key: only dict/list are accepted (a str raised TypeError)."""
  def __init__(self):
    self.d = {}

  def put(self, k, v):
    if not isinstance(v, (dict, list)):
      raise TypeError("JSON param needs dict/list")
    self.d[k] = v


def test_write_answer_puts_dict_and_plate_accepts_it():
  p = P()
  assert write_answer(3, 50, True, params=p)
  assert p.d[ANSWER_PARAM] == {"seq": 3, "mph": 50, "yes": True, "answer": "yes"}
  plate = PlateState()
  pr = plate.propose(50, 0.9, way_id=7, heading=90.0, lat=45.0, lon=-95.0, read_id=("r", 1))
  raw = dict(p.d[ANSWER_PARAM], seq=pr.seq)
  assert plate.answer(raw)


def test_temp_answer_goes_through_json_param():
  p = P()
  assert write_answer(4, 45, "temp", params=p)
  assert p.d[ANSWER_PARAM] == {"seq": 4, "mph": 45, "yes": False, "answer": "temp"}
