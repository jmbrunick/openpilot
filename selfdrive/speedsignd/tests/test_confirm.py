import json

from openpilot.selfdrive.speedsignd import confirm as C


def test_plate_changes_only_on_yes():
  p = C.PlateState()
  pr = p.propose(45, 0.8, way_id=111, heading=90.0)
  assert p.plate_mph == 0 and pr.mph == 45 and pr.way_id == 111
  assert p.answer(json.dumps({"seq": pr.seq, "mph": 45, "yes": False}))
  assert p.plate_mph == 0 and p.proposal is None                     # No keeps the old plate
  pr = p.propose(45, 0.8, way_id=111, heading=90.0)
  assert p.answer({"seq": pr.seq, "mph": 45, "yes": True})
  assert p.plate_mph == 45
  pr = p.propose(30, 0.7, way_id=222)
  assert p.plate_mph == 45                                            # a new read never changes it
  assert not p.answer({"seq": pr.seq + 5, "mph": 30, "yes": True})    # stale answer ignored
  assert p.plate_mph == 45 and p.proposal is not None
  assert p.propose(45, 0.9) is None                                   # already on the plate


def test_same_value_keeps_prompt_seq():
  p = C.PlateState()
  a = p.propose(50, 0.7)
  b = p.propose(50, 0.9)
  assert a.seq == b.seq
