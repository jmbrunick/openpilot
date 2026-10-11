"""Tip into a turn lane, then turn at the intersection mid-change.

Harness: card (CarSpecificEvents, Pre-AP) + the controlsd lane-change
pieces (TippedLaneChangeTorque, LaneChangeTurnHold, BlinkerLateralHold,
DriverLateralHandoff with soft-lat On) at 100 Hz, DesireHelper with a
modelV2-like lane model at 20 Hz. Lamps follow CC.leftBlinker /
rightBlinker (and a latched physical stalk).
"""

from pathlib import Path

import pytest

from cereal import car, log

from openpilot.common.constants import CV
from openpilot.selfdrive.car.car_specific import CarSpecificEvents
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import (
  BlinkerLateralHold,
  lat_active_with_blinker_pause,
)
from openpilot.selfdrive.controls.lib.desire_helper import (
  LANE_CHANGE_SPEED_MIN,
  DesireHelper,
  LaneChangeDirection,
  LaneChangeState,
)
from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  DriverLateralHandoff,
  lat_active_after_handoff,
)
from openpilot.selfdrive.controls.lib.lane_change_nudge import (
  EMERGENCY_RISE_WINDOW_S,
  EMERGENCY_TORQUE_NM,
  SOFT_YIELD_TRIGGER_NM,
  TippedLaneChangeTorque,
)
from openpilot.selfdrive.controls.lib.lane_change_turn import (
  HOLD_RESUME_SPEED,
  LANE_CHANGE_SPEED_MIN as TURN_SPEED_MIN,
  MODE_HOLD,
  MODE_TURN,
  TURN_ENTER_ANGLE_DEG,
  TURN_EXIT_ANGLE_DEG,
  TURN_EXIT_HOLD_S,
  TURN_HOLD_TIMEOUT_S,
  LaneChangeTurnHold,
  blinker_with_turn_hold,
  is_lane_change_turn,
)

EventName = log.OnroadEvent.EventName
ROOT = Path(__file__).resolve().parents[4]
DT = 0.01
MPH = CV.MPH_TO_MS
V_APPROACH = 25 * MPH


# Sep 28 11:13:18 CT, route 124 seg 14 (rlog carState, 20 Hz): the logged
# "emergency cancel" (-2.92 Nm). Right tip at 25.5 mph in a left curve;
# the car slowed under 20 mph while armed; the driver latched the stalk
# right at 11:13:20.29 and turned right hand over hand (wheel +5 -> -170
# deg, -2.2 to -2.9 Nm held ~3 s). The old build ended the change and
# dropped lateral at 11:13:21.24 when torque crossed its instantaneous
# 2.0 Nm line; openpilot (and long) stayed engaged (no steerDisengage in
# onroadEvents). It was a sustained turn, not a yank: under 0.55 -> over
# 2.2 Nm took ~0.45 s (the fast-rise window is 0.25 s).
# (torque Nm, steeringPressed, stalk, vEgo m/s, wheel angle deg)
TURN_RIGHT_1113 = [
  (-0.25, 0, 0, 11.37, +25.6),
  (-0.21, 0, 0, 11.38, +25.6),
  (-0.08, 0, 0, 11.39, +25.6),
  (+0.25, 0, 2, 11.40, +25.4),   # 11:13:18.488 tip right, 25.5 mph
  (+0.61, 0, 2, 11.41, +25.1),
  (+0.34, 0, 0, 11.42, +24.6),
  (-0.04, 0, 0, 11.43, +24.0),
  (-0.08, 0, 0, 11.44, +23.6),
  (-0.06, 0, 0, 11.44, +23.3),   # 11:13:18.738 preLaneChange (log)
  (-0.01, 0, 0, 11.45, +23.3),
  (+0.05, 0, 0, 11.46, +23.2),
  (+0.11, 0, 0, 11.47, +22.6),
  (+0.18, 0, 0, 11.47, +21.8),
  (+0.41, 0, 0, 11.48, +21.6),
  (+0.69, 0, 0, 11.48, +21.4),
  (+0.28, 0, 0, 11.48, +20.3),
  (-0.26, 0, 0, 11.48, +18.9),
  (+0.02, 0, 0, 11.49, +18.7),
  (+0.45, 0, 0, 11.50, +18.6),
  (+0.56, 0, 0, 11.40, +18.0),
  (+0.60, 0, 0, 11.29, +17.3),
  (+0.38, 0, 0, 11.25, +16.6),
  (+0.10, 0, 0, 11.23, +16.0),
  (+0.51, 0, 0, 11.18, +15.4),
  (+1.03, 0, 0, 11.14, +14.9),
  (+0.95, 1, 0, 11.08, +14.2),
  (+0.77, 1, 0, 11.02, +13.4),
  (+0.99, 0, 0, 10.95, +12.7),
  (+1.29, 0, 0, 10.87, +12.0),
  (+1.05, 1, 0, 10.79, +11.5),
  (+0.72, 1, 0, 10.71, +11.0),
  (+0.63, 1, 0, 10.60, +10.4),
  (+0.59, 1, 0, 10.49, +9.9),
  (+0.47, 0, 0, 10.38, +9.5),
  (+0.34, 0, 0, 10.27, +9.2),
  (+0.41, 0, 0, 10.15, +9.2),
  (+0.51, 0, 0, 10.03, +9.1),
  (+0.60, 0, 0, 9.96, +9.1),
  (+0.69, 0, 0, 9.89, +9.2),
  (+0.43, 0, 2, 9.80, +8.9),   # 11:13:20.288 stalk latched right
  (+0.12, 0, 2, 9.70, +8.6),
  (+0.06, 0, 2, 9.61, +8.5),
  (+0.07, 0, 2, 9.52, +8.4),
  (+0.02, 0, 2, 9.44, +8.4),
  (-0.04, 0, 2, 9.37, +8.4),
  (+0.06, 0, 2, 9.29, +8.4),
  (+0.18, 0, 2, 9.21, +8.4),
  (+0.01, 0, 2, 9.13, +8.1),
  (-0.22, 0, 2, 9.05, +7.8),
  (-0.60, 0, 2, 8.97, +7.4),
  (-1.00, 0, 2, 8.89, +7.0),
  (-0.92, 0, 2, 8.81, +6.8),
  (-0.76, 0, 2, 8.72, +6.6),
  (-1.04, 0, 2, 8.64, +6.3),
  (-1.40, 0, 2, 8.56, +6.0),
  (-1.49, 1, 2, 8.48, +5.8),
  (-1.53, 1, 2, 8.40, +5.6),
  (-1.85, 1, 2, 8.31, +5.2),
  (-2.21, 1, 2, 8.23, +4.8),   # 11:13:21.238 old build: lat off, LC off (2.0 Nm line)
  (-1.92, 1, 2, 8.15, +2.8),
  (-1.53, 1, 2, 8.07, +0.6),
  (-1.66, 1, 2, 7.99, -2.8),
  (-1.86, 1, 2, 7.92, -6.5),
  (-1.99, 1, 2, 7.84, -12.2),
  (-2.10, 1, 2, 7.75, -18.3),
  (-2.18, 1, 2, 7.67, -22.4),
  (-2.24, 1, 2, 7.58, -26.1),
  (-2.25, 1, 2, 7.50, -31.0),
  (-2.26, 1, 2, 7.42, -36.3),
  (-2.27, 1, 2, 7.34, -39.7),
  (-2.28, 1, 2, 7.26, -42.8),
  (-2.34, 1, 2, 7.18, -47.4),
  (-2.42, 1, 2, 7.10, -52.1),
  (-2.47, 1, 2, 7.02, -55.9),
  (-2.53, 1, 2, 6.93, -59.3),
  (-2.51, 1, 2, 6.84, -64.4),
  (-2.47, 1, 2, 6.75, -69.8),
  (-2.47, 1, 2, 6.67, -73.2),
  (-2.49, 1, 2, 6.59, -76.2),
  (-2.52, 1, 2, 6.51, -80.2),
  (-2.55, 1, 2, 6.44, -84.3),
  (-2.53, 1, 2, 6.37, -87.0),
  (-2.50, 1, 2, 6.31, -89.5),
  (-2.48, 1, 2, 6.24, -93.2),
  (-2.45, 1, 2, 6.17, -97.1),
  (-2.50, 1, 2, 6.10, -99.9),
  (-2.56, 1, 2, 6.03, -102.5),
  (-2.64, 1, 2, 5.98, -105.8),
  (-2.72, 1, 2, 5.93, -109.3),
  (-2.69, 1, 2, 5.88, -112.3),
  (-2.64, 1, 2, 5.83, -115.3),
  (-2.72, 1, 2, 5.78, -119.5),
  (-2.83, 1, 2, 5.72, -123.9),
  (-2.82, 1, 2, 5.68, -127.4),
  (-2.80, 1, 2, 5.64, -130.9),
  (-2.85, 1, 2, 5.60, -136.3),
  (-2.90, 1, 2, 5.57, -142.2),
  (-2.92, 1, 2, 5.54, -146.1),
  (-2.94, 1, 2, 5.51, -149.8),   # 11:13:23.238 -2.94 Nm peak, wheel -150 deg
  (-2.88, 1, 2, 5.49, -154.6),
  (-2.82, 1, 2, 5.47, -159.7),
  (-2.81, 1, 2, 5.46, -162.4),
  (-2.80, 1, 2, 5.45, -164.8),
  (-2.70, 1, 2, 5.43, -167.0),
  (-2.58, 1, 2, 5.40, -169.2),
  (-2.61, 1, 2, 5.38, -169.7),
  (-2.66, 1, 2, 5.35, -170.0),
  (-2.59, 1, 2, 5.36, -170.2),
  (-2.50, 1, 2, 5.38, -170.5),
  (-2.31, 1, 2, 5.37, -170.3),
  (-2.11, 1, 2, 5.35, -170.1),
  (-2.15, 1, 2, 5.34, -169.6),
  (-2.23, 1, 2, 5.32, -169.0),
  (-2.32, 1, 2, 5.31, -168.9),
]


class _Line:
  def __init__(self, y):
    self.y = [y, y, y]


class _Model:
  def __init__(self, prob=0.9):
    self.laneLines = [_Line(y) for y in (-5.55, -1.85, 1.85, 5.55)]
    self.laneLineProbs = [prob] * 4


def _cp():
  cp = car.CarParams.new_message()
  cp.carFingerprint = "TESLA_MODEL_S_PREAP"
  cp.brand = "tesla"
  cp.pcmCruise = True
  cp.openpilotLongitudinalControl = False
  return cp


class Sim:
  def __init__(self, v=V_APPROACH):
    self.cse = CarSpecificEvents(_cp())
    self.dh = DesireHelper()
    self.lct = TippedLaneChangeTorque()
    self.turn = LaneChangeTurnHold()
    self.hold = BlinkerLateralHold()
    self.handoff = DriverLateralHandoff(enabled=True)
    self.v = v
    self.prob = 0.9
    self.angle = 0.0
    self.lat_active = True
    self.cc_blinker = (False, False)
    self.lc_state = LaneChangeState.off
    self.lc_dir = 0
    self.prev = self._cs(0.0, 0, 0, 0)
    self.frame = 0
    self.t = 0.0
    self.disengage_t: list[float] = []
    self.lat_off_t: list[float] = []
    self.blinker_log: list[tuple[float, bool, bool]] = []

  def _cs(self, tq, pressed, stalk, hands):
    cs = car.CarState.new_message()
    cs.cruiseState.enabled = True
    cs.cruiseState.available = True
    cs.gearShifter = "drive"
    cs.vEgo = float(self.v)
    cs.steeringTorque = float(tq)
    cs.steeringPressed = bool(pressed)
    cs.steeringAngleDeg = float(self.angle)
    cs.turnSignalStalkState = int(stalk)
    cs.leftBlinker = bool(self.cc_blinker[0] or stalk == 1)
    cs.rightBlinker = bool(self.cc_blinker[1] or stalk == 2)
    cs.steeringTorqueEps = float(hands)
    cs.steeringDisengage = int(hands) >= 2
    return cs

  def step(self, tq=0.0, pressed=None, stalk=0, hands=None, angle=None):
    if angle is not None:
      rate = (angle - self.angle) / DT
      self.angle = angle
    else:
      rate = 0.0
    if pressed is None:
      pressed = abs(tq) > 1.0
    if hands is None:
      hands = 0 if abs(tq) < 0.5 else (1 if abs(tq) < 1.5 else 2)
    cs = self._cs(tq, pressed, stalk, hands)
    # controlsd (sees the last modelV2 lane-change state)
    tipped = self.lc_state != LaneChangeState.off
    self.lct.update(torque_nm=tq, hands_on_level=hands, direction=self.lc_dir,
                    tipped=tipped, dt=DT)
    turn_dir = self.turn.update(
      lc_state=self.lc_state, lc_direction=self.lc_dir, steering_angle_deg=self.angle,
      torque_nm=tq, lat_active=self.lat_active, v_ego=self.v, stalk_state=stalk,
      engaged=True, dt=DT)
    lat_would = lat_active_with_blinker_pause(
      active=True, steer_fault_temporary=False, steer_fault_permanent=False,
      standstill=False, steer_at_standstill=False,
      left_blinker=cs.leftBlinker, right_blinker=cs.rightBlinker,
      steering_pressed=pressed, steering_disengage=cs.steeringDisengage,
      engaged=True, hold=self.hold, alc_active=tipped, v_ego=self.v,
      stalk_state=stalk, soft_lat_on=True, driver_turn=self.turn.turning, dt=DT)
    out = self.handoff.update(
      engaged=True, lat_would_be_active=lat_would, steering_torque=tq,
      steering_rate_deg=rate, alc_active=tipped, blinker_paused=self.hold.turn_active,
      hands_on_level=hands, v_ego=self.v, emergency_yank=self.lct.release,
      lane_change_confirm=self.lct.confirm, dt=DT)
    self.lat_active = lat_active_after_handoff(lat_would, out.yielded)
    if not self.lat_active:
      self.lat_off_t.append(self.t)
    lc_blinker = DesireHelper.lane_change_keep_blinker(self.dh.lane_change_state,
                                                       self.dh.lane_change_direction)
    self.cc_blinker = blinker_with_turn_hold(lc_blinker, turn_dir)
    self.blinker_log.append((self.t,) + self.cc_blinker)
    # card
    cc = car.CarControl.new_message()
    cc.enabled = True
    cc.latActive = self.lat_active
    cc.leftBlinker, cc.rightBlinker = self.cc_blinker
    events = self.cse.update(cs, self.prev, cc)
    if EventName.steerDisengage in events.names:
      self.disengage_t.append(self.t)
    # modeld DesireHelper at 20 Hz
    if self.frame % 5 == 0:
      self.dh.update(cs, self.lat_active, 0.0, model=_Model(self.prob), engaged=True)
      self.lc_state = self.dh.lane_change_state
      d = self.dh.lane_change_direction
      self.lc_dir = 0 if self.lc_state == LaneChangeState.off else (
        1 if d == LaneChangeDirection.left else 2)
    self.prev = cs
    self.frame += 1
    self.t += DT

  def hold_for(self, seconds, **kw):
    for _ in range(int(round(seconds / DT))):
      self.step(**kw)

  def start_left_change(self):
    self.hold_for(0.2)
    self.hold_for(0.1, stalk=1)
    self.hold_for(0.3)
    assert self.dh.lane_change_state == LaneChangeState.preLaneChange
    self.hold_for(0.3, tq=1.1, pressed=True, hands=1)
    self.hold_for(0.3)
    assert self.dh.lane_change_state == LaneChangeState.laneChangeStarting
    assert self.dh.target_locked
    assert self.cc_blinker == (True, False)

  def hand_over_hand_left(self, peak=200.0, tq=2.0, seconds=2.0):
    """Turn the wheel left to ``peak`` deg with ``tq`` Nm, then hold."""
    n = int(round(seconds / DT))
    for k in range(n):
      self.step(tq=tq, angle=peak * (k + 1) / n)

  def unwind(self, seconds=2.0):
    start = self.angle
    n = int(round(seconds / DT))
    for k in range(n):
      self.step(tq=-0.3, angle=start * (1 - (k + 1) / n))


def _assert_turn_mode_entered(s):
  assert s.dh.lane_change_state == LaneChangeState.off
  assert not s.dh.target_locked
  assert s.turn.mode == MODE_TURN
  assert s.cc_blinker == (True, False)
  assert s.hold.turn_active


def test_thresholds_from_logs():
  assert TURN_ENTER_ANGLE_DEG == 45.0
  assert TURN_EXIT_ANGLE_DEG == 10.0
  assert TURN_EXIT_HOLD_S == 0.5
  assert TURN_HOLD_TIMEOUT_S == 25.0
  assert TURN_SPEED_MIN == LANE_CHANGE_SPEED_MIN
  # Largest logged lane change (24.5 deg) is not a turn; driver torque needed while lat is up.
  assert not is_lane_change_turn(direction=1, steering_angle_deg=24.5, torque_nm=2.0, lat_active=False)
  assert is_lane_change_turn(direction=1, steering_angle_deg=46.0, torque_nm=0.6, lat_active=True)
  assert not is_lane_change_turn(direction=1, steering_angle_deg=46.0, torque_nm=0.2, lat_active=True)
  assert is_lane_change_turn(direction=1, steering_angle_deg=46.0, torque_nm=0.0, lat_active=False)
  assert not is_lane_change_turn(direction=1, steering_angle_deg=-90.0, torque_nm=-2.0, lat_active=False)
  assert is_lane_change_turn(direction=2, steering_angle_deg=-90.0, torque_nm=-2.0, lat_active=False)
  assert not is_lane_change_turn(direction=0, steering_angle_deg=90.0, torque_nm=2.0, lat_active=False)


def test_turn_mid_change_pauses_lat_keeps_engaged_and_blinker():
  s = Sim()
  s.start_left_change()
  s.hand_over_hand_left()
  _assert_turn_mode_entered(s)
  assert s.disengage_t == []
  assert s.lat_off_t, "lateral pauses like a manual turn"
  s.hold_for(1.0, tq=1.2, angle=200.0)
  assert s.cc_blinker == (True, False)
  assert not s.lat_active
  s.unwind()
  # Wheel back near center for TURN_EXIT_HOLD_S: turn complete, blinker off.
  s.hold_for(TURN_EXIT_HOLD_S + 0.05, angle=0.0)
  assert s.cc_blinker == (False, False)
  assert s.turn.mode == 0
  assert s.disengage_t == []


def test_turn_then_straight_road_resumes_on_lane_center_not_old_target():
  s = Sim()
  s.start_left_change()
  s.hand_over_hand_left()
  s.unwind()
  s.hold_for(6.0, angle=0.0)
  assert s.lat_active, "lat comes back through the normal soft handoff"
  assert s.dh.lane_change_state == LaneChangeState.off
  assert not s.dh.target_locked
  assert s.dh.desire == log.Desire.none
  assert s.cc_blinker == (False, False)
  assert s.disengage_t == []


def test_sustained_hard_turn_is_never_an_emergency():
  s = Sim()
  s.start_left_change()
  # Slow build (no spike), then 3.2 Nm held through the whole turn.
  for k in range(40):
    s.step(tq=0.5 + 2.7 * (k + 1) / 40, angle=5.0)
  s.hand_over_hand_left(tq=3.2, seconds=3.0)
  s.hold_for(2.0, tq=3.2, hands=3, angle=200.0)
  assert s.disengage_t == []
  _assert_turn_mode_entered(s)
  assert not s.lat_active


def test_sudden_spike_mid_change_is_still_an_emergency():
  s = Sim()
  s.start_left_change()
  for k in range(30):
    s.step(tq=min(3.5, 3.5 * (k + 1) * DT / 0.05), hands=3, angle=5.0)
  assert s.disengage_t
  assert s.dh.lane_change_state == LaneChangeState.off


def test_confirm_bump_mid_change_keeps_the_target():
  s = Sim()
  s.start_left_change()
  s.hold_for(0.3, tq=1.5, angle=12.0)   # firm bump, lane-change sized angle
  s.hold_for(0.5, angle=6.0)
  assert s.dh.lane_change_state == LaneChangeState.laneChangeStarting
  assert s.dh.target_locked
  assert s.turn.mode == 0
  assert s.disengage_t == []
  assert s.cc_blinker == (True, False)


def test_curve_steered_by_openpilot_is_not_a_turn():
  s = Sim(v=30 * MPH)
  s.start_left_change()
  s.hold_for(1.0, tq=0.1, angle=50.0)   # lat active, no driver torque
  assert s.turn.mode == 0
  assert s.dh.lane_change_state == LaneChangeState.laneChangeStarting


def test_opposite_pull_still_cancels():
  s = Sim()
  s.start_left_change()
  s.hold_for(0.2, tq=-1.2, pressed=True, hands=1, angle=-3.0)
  assert s.dh.lane_change_state == LaneChangeState.off
  assert s.turn.mode == 0
  s.hold_for(0.3)
  assert s.cc_blinker == (False, False)


def test_latched_stalk_during_change_is_a_driver_turn():
  s = Sim()
  s.start_left_change()
  s.hold_for(1.2, stalk=1)
  assert s.dh.lane_change_state == LaneChangeState.off
  assert not s.dh.target_locked
  assert s.hold.turn_active
  # The physical latch keeps the lamp; turning stays engaged.
  s.hold_for(2.0, stalk=1, tq=2.0, angle=150.0)
  assert s.disengage_t == []
  assert s.prev.leftBlinker
  assert not s.lat_active


def test_lines_vanish_after_turn_mode_do_not_end_the_blinker_hold():
  s = Sim()
  s.start_left_change()
  s.hand_over_hand_left()
  _assert_turn_mode_entered(s)
  s.prob = 0.05
  s.hold_for(2.0, tq=1.2, angle=200.0)
  assert s.cc_blinker == (True, False)
  assert s.turn.mode == MODE_TURN
  assert s.disengage_t == []


def test_lines_vanish_before_the_turn_end_the_change_cleanly():
  s = Sim()
  s.start_left_change()
  s.prob = 0.05
  s.hold_for(2.0, angle=4.0)
  assert s.dh.lane_change_state == LaneChangeState.off
  assert not s.dh.target_locked
  assert s.disengage_t == []
  assert s.lat_active


def test_speed_under_20_mid_change_ends_it_and_holds_the_blinker_for_the_turn():
  s = Sim()
  s.start_left_change()
  s.v = 15 * MPH
  s.hold_for(0.3, angle=3.0)
  assert s.dh.lane_change_state == LaneChangeState.off
  assert not s.dh.target_locked
  assert s.turn.mode == MODE_HOLD
  assert s.cc_blinker == (True, False)
  assert s.disengage_t == []
  s.hand_over_hand_left()
  assert s.turn.mode == MODE_TURN
  s.unwind()
  s.hold_for(TURN_EXIT_HOLD_S + 0.05, angle=0.0)
  assert s.cc_blinker == (False, False)
  assert s.disengage_t == []


def test_speed_hold_ends_when_back_up_to_speed_without_a_turn():
  s = Sim()
  s.start_left_change()
  s.v = 15 * MPH
  s.hold_for(0.3)
  assert s.turn.mode == MODE_HOLD
  s.v = HOLD_RESUME_SPEED + 0.5
  s.hold_for(0.1)
  assert s.turn.mode == 0
  assert s.cc_blinker == (False, False)
  assert s.dh.lane_change_state == LaneChangeState.off


@pytest.mark.parametrize("how", ["timeout", "opposite_stalk"])
def test_blinker_hold_never_flashes_forever(how):
  s = Sim()
  s.start_left_change()
  s.hand_over_hand_left()
  assert s.turn.mode == MODE_TURN
  if how == "timeout":
    s.hold_for(TURN_HOLD_TIMEOUT_S - 2.5, tq=0.8, angle=120.0)
    assert s.cc_blinker == (True, False)
    s.hold_for(1.0, tq=0.8, angle=120.0)
  else:
    s.hold_for(0.1, stalk=2, angle=120.0)
    s.hold_for(0.1, angle=120.0)
  assert s.turn.mode == 0
  assert s.cc_blinker == (False, False)


def test_turn_hold_unit_exit_needs_the_hold_time():
  h = LaneChangeTurnHold()
  kw = dict(lc_state=LaneChangeState.laneChangeStarting, lc_direction=2, torque_nm=-1.0,
            lat_active=False, v_ego=V_APPROACH, stalk_state=0, engaged=True, dt=DT)
  assert h.update(steering_angle_deg=-90.0, **kw) == 2
  kw["lc_state"] = LaneChangeState.off
  for _ in range(int(TURN_EXIT_HOLD_S / DT) - 5):
    assert h.update(steering_angle_deg=-5.0, **kw) == 2
  assert h.update(steering_angle_deg=-20.0, **kw) == 2   # not settled: timer restarts
  for _ in range(int(TURN_EXIT_HOLD_S / DT) - 5):
    assert h.update(steering_angle_deg=0.0, **kw) == 2
  for _ in range(10):
    h.update(steering_angle_deg=0.0, **kw)
  assert h.direction == 0
  assert h.update(steering_angle_deg=-90.0, **kw) == 0   # lane change is off: no re-entry
  kw["engaged"] = False
  assert h.update(steering_angle_deg=-90.0, **kw) == 0
  assert blinker_with_turn_hold((False, False), 1) == (True, False)
  assert blinker_with_turn_hold((False, True), 1) == (False, True)


def test_lane_change_ending_with_hands_on_keeps_the_disengage_block():
  # ALC keep-alive ends (lamps dark 1 s) while the driver is still steering:
  # post-turn hand-on hold, so a hands-on edge does not USER_DISABLE.
  hold = BlinkerLateralHold()
  hold.update(True, False, False, engaged=True, dt=DT, stalk_state=1)
  hold.update(True, False, False, engaged=True, dt=DT, stalk_state=0)
  assert hold._alc_keep
  for _ in range(int(1.0 / DT) + 2):
    hold.update(False, False, True, engaged=True, dt=DT, steering_disengage=True)
  assert not hold._alc_keep
  assert hold.holding and hold.blocks_steer_disengage
  hold.update(False, False, False, engaged=True, dt=DT)
  assert not hold.holding and not hold.blocks_steer_disengage
  # Hands already off when the keep ends: nothing held.
  hold = BlinkerLateralHold()
  hold.update(True, False, False, engaged=True, dt=DT, stalk_state=1)
  hold.update(True, False, False, engaged=True, dt=DT, stalk_state=0)
  for _ in range(int(1.0 / DT) + 2):
    hold.update(False, False, False, engaged=True, dt=DT)
  assert not hold.holding and not hold.blocks_steer_disengage


def _replay_1113():
  s = Sim(v=TURN_RIGHT_1113[0][3])
  s.angle = TURN_RIGHT_1113[0][4]
  s.release_t = []
  s.ended_t = None
  for i, row in enumerate(TURN_RIGHT_1113):
    nxt = TURN_RIGHT_1113[i + 1] if i + 1 < len(TURN_RIGHT_1113) else row
    for k in range(5):   # 20 Hz -> 100 Hz, torque / angle linear
      f = k / 5
      s.v = row[3]
      s.step(tq=row[0] + (nxt[0] - row[0]) * f, pressed=bool(row[1]), stalk=row[2],
             angle=row[4] + (nxt[4] - row[4]) * f)
      if s.lct.release:
        s.release_t.append(s.t)
      if s.ended_t is None and s.t > 1.0 and s.dh.lane_change_state == LaneChangeState.off:
        s.ended_t = s.t
  return s


def test_sep28_1113_sustained_turn_with_latched_stalk_is_not_an_emergency():
  torques = [abs(r[0]) for r in TURN_RIGHT_1113]
  latch = next(i for i, r in enumerate(TURN_RIGHT_1113) if i > 5 and r[2] == 2) * 0.05
  over_old = next(i for i, t in enumerate(torques) if t > 2.0) * 0.05
  under = max(i for i, t in enumerate(torques) if t < SOFT_YIELD_TRIGGER_NM and i * 0.05 < over_old) * 0.05
  assert over_old - under > EMERGENCY_RISE_WINDOW_S   # not a fast rise
  assert max(torques) > EMERGENCY_TORQUE_NM            # but held over the hard line

  s = _replay_1113()
  assert s.release_t == [], "sustained same-direction turn must not be an emergency"
  assert s.disengage_t == [], "openpilot must stay engaged"
  # The change ends on the latched stalk (1.0 s), not on torque.
  assert s.ended_t is not None
  assert latch + 1.0 - 0.06 <= s.ended_t <= latch + 1.0 + 0.1
  assert s.dh.lane_change_state == LaneChangeState.off
  assert not s.dh.target_locked
  # Lateral paused through the turn like a manual blinker turn.
  assert s.lat_off_t and s.lat_off_t[0] >= s.ended_t - 0.06
  assert not s.lat_active
  assert s.prev.rightBlinker   # latched stalk keeps the lamp


def test_controlsd_wires_turn_hold_and_tipped_torque():
  src = (ROOT / "selfdrive/controls/controlsd.py").read_text()
  # Emergency or takeover yields lat; only the emergency disengages (card).
  assert "emergency_yank=bool(self._lane_change_torque.release)," in src
  assert "lane_change_confirm = self._lane_change_torque.confirm" in src
  # Turn mode pauses lat like a latched stalk and keeps the blinker.
  assert "driver_turn=self._lane_change_turn.turning," in src
  assert "lat_active=self._lat_active_prev," in src
  assert "self._lat_active_prev = bool(CC.latActive)" in src
  blinker = "\n".join((
    "CC.leftBlinker, CC.rightBlinker = blinker_with_turn_hold(",
    "      DesireHelper.lane_change_keep_blinker(",
    "        model_v2.meta.laneChangeState, model_v2.meta.laneChangeDirection),",
    "      turn_hold_dir)",
  ))
  assert blinker in src
