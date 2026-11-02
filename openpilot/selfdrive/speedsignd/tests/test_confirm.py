import json

from openpilot.selfdrive.speedsignd import confirm as C


def test_plate_follows_reads_map_waits_for_yes():
  p = C.PlateState()
  pr = p.propose(45, 0.8, way_id=111, heading=90.0)
  assert p.plate_mph == 45 and pr.mph == 45 and pr.way_id == 111     # the plate shows the read at once
  assert p.answer(json.dumps({"seq": pr.seq, "mph": 45, "yes": False}))
  assert p.plate_mph == 0 and p.confirmed_mph == 0 and p.proposal is None    # No: map unchanged, read cleared
  pr = p.propose(45, 0.8, way_id=111, heading=90.0)
  assert p.plate_mph == 45
  assert p.answer({"seq": pr.seq, "mph": 45, "yes": True})
  assert p.plate_mph == 0 and p.confirmed_mph == 45                   # Yes: read cleared, mph goes to the map
  pr = p.propose(30, 0.7, way_id=222)
  assert p.plate_mph == 30 and p.confirmed_mph == 45                 # a new read replaces the plate
  assert not p.answer({"seq": pr.seq + 5, "mph": 30, "yes": True})    # stale answer ignored
  assert p.confirmed_mph == 45 and p.proposal is not None
  pr = p.propose(45, 0.9, way_id=333)                                 # same as confirmed: still prompts
  assert pr is not None and pr.mph == 45 and p.plate_mph == 45


def test_same_read_keeps_prompt_seq_new_read_does_not():
  p = C.PlateState()
  a = p.propose(50, 0.7, read_id=("read", 3, 1.0))
  b = p.propose(50, 0.9, read_id=("read", 3, 1.0))
  assert a.seq == b.seq
  c = p.propose(50, 0.9, way_id=7, read_id=("read", 4, 2.0))   # same mph, another sign
  assert c.seq != a.seq and c.way_id == 7


def test_timeout_clears_prompt_and_read_plate():
  p = C.PlateState()
  pr = p.propose(50, 0.8, way_id=9, heading=10.0, read_id=("read", 1, 1.0), now=100.0)
  assert not p.expire(100.0 + C.PROMPT_TIMEOUT_S - 0.1)
  assert p.plate_mph == 50 and p.proposal is not None
  assert p.expire(100.0 + C.PROMPT_TIMEOUT_S)
  assert p.plate_mph == 0 and p.proposal is None and p.confirmed_mph == 0
  assert not p.answer({"seq": pr.seq, "mph": 50, "yes": True})        # late Yes refused
  assert p.confirmed_mph == 0
  assert p.propose(50, 0.9, read_id=("read", 1, 1.0), now=200.0) is None   # same read not re-asked
  assert p.plate_mph == 0


def test_proposal_keeps_sign_location():
  from openpilot.selfdrive.speedsignd.confirm import PlateState as PS
  p = PS()
  pr = p.propose(60, 0.9, way_id=7, heading=180.0, lat=44.79, lon=-95.8, read_id=("r", 1), now=1.0,
                 sign={"lat": 44.7897, "lon": -95.8001, "range_m": 31.0})
  assert pr.sign_fields() == {"sign_lat": 44.7897, "sign_lon": -95.8001, "sign_range_m": 31.0}
  pr2 = PS().propose(60, 0.9, way_id=7, heading=180.0, lat=44.79, lon=-95.8, read_id=("r", 2), now=1.0)
  assert pr2.sign_fields() == {}
