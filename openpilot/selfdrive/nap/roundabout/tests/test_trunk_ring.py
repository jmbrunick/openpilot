from openpilot.selfdrive.nap.roundabout.trunk_ring import (
  GATE_BLOCK, GATE_OPEN, GATE_TOUCH, ring_touches_way, trunk_ring_gate, trunk_ring_ok,
)

RING = [(45.12090, -95.09420), (45.12100, -95.09400), (45.12090, -95.09380), (45.12080, -95.09400), (45.12090, -95.09420)]
ROAD_INTO_RING = [(45.12080, -95.10000), (45.12088, -95.09430)]
ROAD_BESIDE = [(45.12300, -95.10000), (45.12300, -95.09000)]  # ~230 m north


def test_gate_by_class():
  assert trunk_ring_gate("motorway") == GATE_BLOCK
  assert trunk_ring_gate("motorway_link") == GATE_BLOCK
  assert trunk_ring_gate("trunk") == GATE_TOUCH
  assert trunk_ring_gate("Trunk_Link") == GATE_TOUCH
  assert trunk_ring_gate("secondary") == GATE_OPEN
  assert trunk_ring_gate(None) == GATE_OPEN


def test_touch():
  assert ring_touches_way(RING, ROAD_INTO_RING)
  assert not ring_touches_way(RING, ROAD_BESIDE)
  assert not ring_touches_way(RING, None)


def test_trunk_ring_ok_us12_and_i74_cases():
  # US 12 ring (way 985352994) is itself trunk: accepted before it touches our way.
  assert trunk_ring_ok("trunk", RING, ROAD_BESIDE)
  # I-74 style residential loop beside the carriageway: rejected.
  assert not trunk_ring_ok("residential", RING, ROAD_BESIDE)
  assert not trunk_ring_ok("service", RING, ROAD_BESIDE)
  # A minor ring the trunk actually feeds into: accepted.
  assert trunk_ring_ok("unclassified", RING, ROAD_INTO_RING)
