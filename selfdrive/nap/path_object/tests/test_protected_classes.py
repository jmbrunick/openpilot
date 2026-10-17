"""Justin's top rule: people and animals are never suppressed, day or night, at any tagged spot."""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.path_object_response import brake_decel, confirmed_from_msg
from openpilot.selfdrive.nap.path_object import overhead_map as OM


def _obs(cls, day, **kw):
  base = dict(agree=True, visionEvaluated=True, rangeM=50.0, lateral=0.0, spanM=0.5, vRel=-20.0,
              objectClass=cls, overheadIgnore=True, bottomClearanceM=4.5, roadLights=False,
              roadLevelSeen=False, daylight=day, modelLead=False)
  base.update(kw)
  return SimpleNamespace(**base)


@pytest.mark.parametrize("kind", ["bridge", "signal", "confirmed", "gantry"])
@pytest.mark.parametrize("day", [True, False])
@pytest.mark.parametrize("cls", ["human", "animal"])
def test_people_animals_never_ignored(kind, day, cls):
  md = SimpleNamespace(overheadAheadM=50.0, overheadKind=kind)
  obs = _obs(cls, day)
  assert not OM.overhead_match(obs, md, 20.0)
  obj = confirmed_from_msg(obs, 0.1, md, 20.0)        # clearance overheadIgnore set too: still confirmed
  assert obj is not None and obj.object_class == cls
  assert brake_decel(obj, 20.0, -3.5) < 0.0             # still a would-brake -> wouldRegenAlert


@pytest.mark.parametrize("kind", ["bridge", "confirmed", "gantry"])
def test_night_road_level_object_inside_60m_cancels(kind):
  md = SimpleNamespace(overheadAheadM=45.0, overheadKind=kind)
  far = _obs("obstacle", False, rangeM=70.0, overheadIgnore=False)
  assert OM.overhead_match(far, SimpleNamespace(overheadAheadM=70.0, overheadKind=kind), 20.0)
  near_seen = _obs("obstacle", False, rangeM=45.0, overheadIgnore=False, roadLevelSeen=True)
  assert not OM.overhead_match(near_seen, md, 20.0)
  assert confirmed_from_msg(near_seen, 0.1, md, 20.0) is not None


def test_obstacle_clearance_ignore_still_applies():
  assert confirmed_from_msg(_obs("obstacle", True), 0.1) is None
