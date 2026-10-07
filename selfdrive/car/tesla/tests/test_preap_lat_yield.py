"""Pre-AP lateral yield, openpilot side: A cancels, B never cancels.

A steering yank / hands-on cancels OP only while OP is in full lateral
control (A). While lateral is yielded it is a driver maneuver and cancels
nothing (B); longitudinal stays engaged. "Yielded" is inferred from the
0x488 DAS_steeringControlType=1 history (opendbc preap/lat_yield.py,
mirrored by panda tesla_preap_latyield.h).

These tests run the real CarController.apply, the real card CarState path and
the real CarSpecificEvents. CarController.apply needs cereal, so the two
CarController obligations (stamp the frames it sends; refuse type 1 while
blocked) are pinned here and mutation-tested in
.github/ci/tesla_preap_lat_yield_mutations.py.
"""
from cereal import car, log
from opendbc.can import CANPacker, CANParser
from opendbc.car import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla.preap import lat_yield as ly
from opendbc.car.tesla.preap.lat_yield import LatYieldTracker

from openpilot.selfdrive.car.car_specific import CarSpecificEvents
from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import install_blinker_lat_pause
from openpilot.selfdrive.controls.lib.driver_lateral_handoff import cs_hands_on_level

EventName = log.OnroadEvent.EventName
STEER_ADDR = 0x488


class Clock:
  def __init__(self):
    self.t = 100.0

  def __call__(self):
    return self.t

  def advance(self, dt):
    self.t += dt


class Rig:
  """Real Pre-AP interface (CarState + CarController) with a fake yield clock."""

  def __init__(self):
    install_blinker_lat_pause()
    cp = interfaces["TESLA_MODEL_S_PREAP"].get_params(
      "TESLA_MODEL_S_PREAP", {bus: {} for bus in range(8)}, [],
      alpha_long=False, is_release=False, docs=False)
    self.cp = cp
    self.itf = interfaces["TESLA_MODEL_S_PREAP"](cp)
    self.packer = CANPacker("tesla_preap")
    self.parser = CANParser("tesla_preap", [("DAS_steeringControl", 50)], 0)
    self.clk = Clock()
    self.eng = self.itf.CS.engagement
    self.eng.lat_yield = LatYieldTracker(clock=self.clk)
    self.counter = 0
    self.events = CarSpecificEvents(cp)
    self.prev = None
    self.nanos = 0
    self.steer_types = []
    self.engage()

  def engage(self):
    self.itf.update(self._can(0))
    self.eng.cruiseEnabled = True
    self.eng.enableLongControl = True
    self.itf.CS.cruiseEnabled = True
    self.itf.CS.enableLongControl = True
    self.itf.CS.enableJustCC = False
    self.hands(0)  # CS_prev for the hands-on edge

  def _can(self, hands):
    self.counter += 1
    msgs = []
    for name, vals in (("DI_torque2", {"DI_gear": 4}),
                       ("EPAS_sysStatus", {"EPAS_handsOnLevel": hands, "EPAS_eacStatus": 1,
                                           "EPAS_internalSAS": 0,
                                           "EPAS_sysStatusCounter": self.counter % 16})):
      addr, dat, bus = self.packer.make_can_msg(name, 0, vals)
      msgs.append(CanData(addr, dat, bus))
    return [(self.nanos, msgs)]

  def drive(self, seconds, lat_active):
    """Run CarController.apply at 100 Hz; record 0x488 control types."""
    control = car.CarControl.new_message()
    control.enabled = True
    control.latActive = lat_active
    control.longActive = True
    control = control.as_reader()
    for _ in range(int(round(seconds / 0.01))):
      self.clk.advance(0.01)
      self.nanos += 10_000_000
      _, sends = self.itf.apply(control, now_nanos=self.nanos)
      for addr, dat, bus in sends:
        if addr == STEER_ADDR:
          self.parser.update([(self.nanos, [(addr, dat, bus)])])
          self.steer_types.append(int(self.parser.vl["DAS_steeringControl"]["DAS_steeringControlType"]))

  def hands(self, level):
    """One CarState frame with EPAS_handsOnLevel=level; returns (CS, events)."""
    ret = self.itf.update(self._can(level))
    ev = self.events.update(ret, self.prev or ret, car.CarControl.new_message())
    self.prev = ret
    return ret, ev


def test_carcontroller_stamps_the_type1_frames_it_sends():
  r = Rig()
  r.drive(0.3, lat_active=True)
  assert r.steer_types and set(r.steer_types) == {1}
  assert r.eng.lat_yield.full_control()  # the stamp is what makes this A
  r.clk.advance(ly.RECENT_S + 0.05)
  assert not r.eng.lat_yield.full_control()  # and it expires without more frames


def test_carcontroller_type0_frames_do_not_stamp():
  r = Rig()
  r.drive(0.3, lat_active=False)
  assert r.steer_types and set(r.steer_types) == {0}
  assert not r.eng.lat_yield.full_control()


def test_carcontroller_refuses_type1_while_blocked_and_resumes_after_release():
  r = Rig()
  r.drive(0.2, lat_active=True)
  r.eng.lat_yield.block()
  r.steer_types.clear()
  r.drive(0.2, lat_active=True)
  assert r.steer_types and set(r.steer_types) == {0}
  assert not r.eng.lat_yield.full_control()
  # hands released long enough: block clears, type 1 returns (and a re-arm
  # grace runs, since the stream resumed after a lapse)
  r.eng.lat_yield.update_block(False)
  r.clk.advance(ly.BLOCK_CLEAR_S + 0.01)
  r.eng.lat_yield.update_block(False)
  assert not r.eng.lat_yield.blocked
  r.steer_types.clear()
  r.drive(0.2, lat_active=True)
  assert 1 in r.steer_types


def test_A_yank_on_full_lateral_cancels_everything():
  r = Rig()
  r.drive(0.3, lat_active=True)
  ret, ev = r.hands(2)
  assert ret.steeringDisengage
  assert not r.eng.cruiseEnabled and not r.eng.enableLongControl
  assert cs_hands_on_level(ret) == 2
  assert ly.stash_full_control(ret.steeringTorqueEps)
  assert EventName.steerDisengage in ev.names
  assert EventName.pcmDisable in ev.names


def test_B_yank_on_yielded_lateral_cancels_nothing_and_keeps_long():
  r = Rig()
  r.drive(0.2, lat_active=True)
  r.drive(0.4, lat_active=False)  # driver handoff / blinker turn / ...
  ret, ev = r.hands(2)
  assert ret.steeringDisengage
  assert r.eng.cruiseEnabled and r.eng.enableLongControl
  assert cs_hands_on_level(ret) == 2  # hands level survives the stash
  assert not ly.stash_full_control(ret.steeringTorqueEps)
  assert EventName.steerDisengage not in ev.names
  assert EventName.pcmDisable not in ev.names
  assert r.eng.lat_yield.blocked
  # lateral stays refused until the hands come off, longitudinal untouched
  r.steer_types.clear()
  r.drive(0.2, lat_active=True)
  assert set(r.steer_types) == {0}
  assert r.eng.cruiseEnabled and r.eng.enableLongControl


def test_B_after_blend_grace_is_still_B_and_A_returns_after_grace():
  r = Rig()
  r.drive(0.2, lat_active=True)
  r.drive(0.4, lat_active=False)
  r.drive(0.5, lat_active=True)  # mid take-back blend: type 1 flows
  ret, ev = r.hands(2)
  assert r.eng.cruiseEnabled and EventName.steerDisengage not in ev.names
  r2 = Rig()
  r2.drive(0.2, lat_active=True)
  r2.drive(0.4, lat_active=False)
  r2.drive(ly.GRACE_S + 0.2, lat_active=True)  # blend done, OP steering again
  ret, ev = r2.hands(2)
  assert not r2.eng.cruiseEnabled
  assert EventName.steerDisengage in ev.names


def test_stash_decodes_hands_level_for_every_level_and_state():
  for hands in range(4):
    for full in (True, False):
      v = ly.encode_hands_stash(hands, full)
      assert ly.stash_full_control(v) is full
      assert int(round(v)) == hands


def test_card_handoff_forwards_steering_pressed_to_the_early_yield():
  from pathlib import Path

  from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import _handoff_for, update_card_lat_handoff
  from openpilot.selfdrive.controls.lib.driver_lateral_handoff import EARLY_YIELD_FRAMES

  # 1.4 Nm / 80 ms yields without steeringPressed, so a 3 Nm push still
  # yields if this kwarg is dropped. The forward itself is the contract.
  card = Path(__file__).resolve().parents[4] / "selfdrive/car/tesla/preap_blinker_lat_pause.py"
  assert "    steering_pressed=steering_pressed,\n    dt=dt,\n" in card.read_text()
  eng = Rig().eng
  for _ in range(EARLY_YIELD_FRAMES):
    update_card_lat_handoff(
      eng, engaged=True, lat_would_be_active=True, steering_torque=3.0, steering_rate_deg=0.0,
      hands_on_level=0, brake_applied=False, a_ego=0.0, v_ego=20.0,
      steering_pressed=True, param_on=True)
  assert _handoff_for(eng)._yielded
