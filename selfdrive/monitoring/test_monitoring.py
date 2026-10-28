import numpy as np

from cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_DMON
from openpilot.selfdrive.monitoring.dm_toggles import (
  DEFAULT_FALSE_ALERT_IGNORE, DEFAULT_SIMULATE_LOOKING,
  PARAM_DM_FALSE_ALERT_IGNORE, PARAM_DM_SIMULATE_LOOKING,
)
from openpilot.selfdrive.monitoring.policy import (
  DriverMonitoring, DRIVER_MONITOR_SETTINGS, LOOK_SIM_MODE_PHONE, LOOK_SIM_MODE_GLANCE,
  LOOK_SIM_COUNTDOWN_MIN_S, LOOK_SIM_RANDOM_WINDOW_S, LOOK_SIM_FIRE_MAX_S,
  LOOK_SIM_HOLD_MIN_S, LOOK_SIM_HOLD_MAX_S,
  VISION_LOOKING_FILTER_X, VISION_RECOVERY_FACTOR_MAX, VISION_RECOVERY_FACTOR_MIN,
  DM_LOOKAWAY_GATE_MPH, DM_LOOKAWAY_GATE_MS, lookaway_alerts_paused,
  vision_looking_path, looking_recovery_time_s,
)

EventName = log.OnroadEvent.EventName
dm_settings = DRIVER_MONITOR_SETTINGS()

TEST_TIMESPAN = 120  # seconds
DISTRACTED_SECONDS_TO_ORANGE = dm_settings._VISION_POLICY_ALERT_2_TIMEOUT + 1
DISTRACTED_SECONDS_TO_RED = dm_settings._VISION_POLICY_ALERT_3_TIMEOUT + 1
INVISIBLE_SECONDS_TO_ORANGE = dm_settings._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT + 1
INVISIBLE_SECONDS_TO_RED = dm_settings._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT + 1

def make_msg(face_detected, distracted=False, model_uncertain=False,
             phone=False, pose=False):
  ds = log.DriverStateV2.new_message()
  # Large yaw trips uncalibrated pose (threshold ~0.40). Blink = eye.
  ds.leftDriverData.faceOrientation = [0., 1.2 if pose else 0., 0.]
  ds.leftDriverData.facePosition = [0., 0.]
  ds.leftDriverData.faceProb = 1. * face_detected
  ds.leftDriverData.leftEyeProb = 1.
  ds.leftDriverData.rightEyeProb = 1.
  ds.leftDriverData.leftBlinkProb = 1. * distracted
  ds.leftDriverData.rightBlinkProb = 1. * distracted
  ds.leftDriverData.faceOrientationStd = [1.*model_uncertain, 1.*model_uncertain, 1.*model_uncertain]
  ds.leftDriverData.facePositionStd = [1.*model_uncertain, 1.*model_uncertain]
  ds.leftDriverData.phoneProb = 1. * phone
  return ds


# driver state from neural net, 10Hz
msg_NO_FACE_DETECTED = make_msg(False)
msg_ATTENTIVE = make_msg(True)
msg_DISTRACTED = make_msg(True, distracted=True)
msg_ATTENTIVE_UNCERTAIN = make_msg(True, model_uncertain=True)
msg_DISTRACTED_UNCERTAIN = make_msg(True, distracted=True, model_uncertain=True)
msg_DISTRACTED_BUT_SOMEHOW_UNCERTAIN = make_msg(True, distracted=True, model_uncertain=dm_settings._HI_STD_THRESHOLD*1.5)
msg_PHONE_ONLY = make_msg(True, phone=True)
msg_POSE_ONLY = make_msg(True, pose=True)
msg_PHONE_AND_POSE = make_msg(True, phone=True, pose=True)
msg_PHONE_AND_EYE = make_msg(True, distracted=True, phone=True)

# driver interaction with car
car_interaction_DETECTED = True
car_interaction_NOT_DETECTED = False

# some common state vectors
always_no_face = [msg_NO_FACE_DETECTED] * int(TEST_TIMESPAN / DT_DMON)
always_attentive = [msg_ATTENTIVE] * int(TEST_TIMESPAN / DT_DMON)
always_distracted = [msg_DISTRACTED] * int(TEST_TIMESPAN / DT_DMON)
always_true = [True] * int(TEST_TIMESPAN / DT_DMON)
always_false = [False] * int(TEST_TIMESPAN / DT_DMON)
# Above the 2 mph look-away gate, below pose/wheelpos calib mins (13 / 11 m/s).
TEST_MOVING_MS = 5.0  # ~11 mph
TEST_CREEP_MS = DM_LOOKAWAY_GATE_MS * 0.5  # 1 mph
TEST_ABOVE_GATE_MS = DM_LOOKAWAY_GATE_MS + 0.05

class _NS:
  def __init__(self, **kw):
    self.__dict__.update(kw)


def _fake_sm(*, engaged=True, hands=1, steer_pressed=False, gas_pressed=False,
             driver_state=None, standstill=False, v_ego=20.0):
  from cereal import car
  return {
    'carState': _NS(
      vEgo=v_ego,
      gearShifter=car.CarState.GearShifter.drive,
      standstill=standstill,
      steeringPressed=steer_pressed,
      gasPressed=gas_pressed,
      steeringAngleDeg=0.0,
      handsOnLevel=hands,
      steeringTorqueEps=float(hands),
      steeringDisengage=False,
    ),
    'selfdriveState': _NS(enabled=engaged),
    'modelV2': _NS(meta=_NS(disengagePredictions=_NS(brakeDisengageProbs=[0.0]))),
    'liveCalibration': _NS(rpyCalib=[0., 0., 0.]),
    'driverStateV2': driver_state if driver_state is not None else msg_DISTRACTED,
  }


class TestMonitoring:
  def _run_seq(self, msgs, interaction, engaged, standstill, simulate_looking=False,
               false_alert_ignore=False, rng_seed=None, car_speed=None):
    DM = DriverMonitoring()
    DM.nap_dm_simulate_looking = bool(simulate_looking)
    DM.nap_dm_false_alert_ignore = bool(false_alert_ignore)
    if rng_seed is not None:
      DM._rng.seed(rng_seed)
      DM._redraw_look_sim_interval()
    if car_speed is None:
      speeds = [TEST_MOVING_MS] * len(msgs)
    elif isinstance(car_speed, (int, float)):
      speeds = [float(car_speed)] * len(msgs)
    else:
      speeds = car_speed
    alert_lvls = []
    for idx in range(len(msgs)):
      DM._update_states(msgs[idx], [0, 0, 0], speeds[idx], engaged[idx], standstill[idx])

      # evaluate events at 10Hz for tests
      DM._update_events(interaction[idx], engaged[idx], standstill[idx], 0)
      alert_lvls.append(DM.alert_level)
    assert len(alert_lvls) == len(msgs), f"got {len(alert_lvls)} for {len(msgs)} driverState input msgs"
    return alert_lvls, DM


  # engaged, driver is attentive all the time
  def test_fully_aware_driver(self):
    alert_lvls, d_status = self._run_seq(always_attentive, always_false, always_true, always_false)
    assert all(a == 0 for a in alert_lvls)
    assert d_status.active_policy == log.DriverMonitoringState.MonitoringPolicy.vision

  # engaged, driver is distracted and does nothing
  def test_fully_distracted_driver(self):
    alert_lvls, d_status = self._run_seq(always_distracted, always_false, always_true, always_false)
    s = d_status.settings
    assert alert_lvls[int(s._VISION_POLICY_ALERT_1_TIMEOUT / 2 / DT_DMON)] == 0
    assert alert_lvls[int((s._VISION_POLICY_ALERT_1_TIMEOUT + \
                    (s._VISION_POLICY_ALERT_2_TIMEOUT - s._VISION_POLICY_ALERT_1_TIMEOUT) / 2) / DT_DMON)] == 1
    assert alert_lvls[int((s._VISION_POLICY_ALERT_2_TIMEOUT + \
                    (s._VISION_POLICY_ALERT_3_TIMEOUT - s._VISION_POLICY_ALERT_2_TIMEOUT) / 2) / DT_DMON)] == 2
    assert alert_lvls[int((s._VISION_POLICY_ALERT_3_TIMEOUT + \
                    (TEST_TIMESPAN - 10 - s._VISION_POLICY_ALERT_3_TIMEOUT) / 2) / DT_DMON)] == 3
    assert isinstance(d_status.awareness, float)

  # engaged, no face detected the whole time, no action
  def test_fully_invisible_driver(self):
    alert_lvls, d_status = self._run_seq(always_no_face, always_false, always_true, always_false)
    s = d_status.settings
    assert alert_lvls[int(s._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT / 2 / DT_DMON)] == 0
    assert alert_lvls[int((s._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT + \
                    (s._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT - s._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT) / 2) / DT_DMON)] == 1
    assert alert_lvls[int((s._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT + \
                    (s._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT - s._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT) / 2) / DT_DMON)] == 2
    assert alert_lvls[int((s._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT + \
                    (TEST_TIMESPAN - 10 - s._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT) / 2) / DT_DMON)] == 3
    assert d_status.active_policy == log.DriverMonitoringState.MonitoringPolicy.wheeltouch

  # engaged, down to orange, driver pays attention, back to normal; then down to orange, driver touches wheel
  #  - should have short orange recovery time and no green afterwards; wheel touch only recovers when paying attention
  def test_normal_driver(self):
    ds_vector = [msg_DISTRACTED] * int(DISTRACTED_SECONDS_TO_ORANGE/DT_DMON) + \
                [msg_ATTENTIVE] * int(DISTRACTED_SECONDS_TO_ORANGE/DT_DMON) + \
                [msg_DISTRACTED] * int((DISTRACTED_SECONDS_TO_ORANGE+2)/DT_DMON) + \
                [msg_ATTENTIVE] * (int(TEST_TIMESPAN/DT_DMON)-int((DISTRACTED_SECONDS_TO_ORANGE*3+2)/DT_DMON))
    interaction_vector = [car_interaction_NOT_DETECTED] * int(DISTRACTED_SECONDS_TO_ORANGE*3/DT_DMON) + \
                         [car_interaction_DETECTED] * (int(TEST_TIMESPAN/DT_DMON)-int(DISTRACTED_SECONDS_TO_ORANGE*3/DT_DMON))
    alert_lvls, _ = self._run_seq(ds_vector, interaction_vector, always_true, always_false)
    assert alert_lvls[int(DISTRACTED_SECONDS_TO_ORANGE*0.5/DT_DMON)] == 0
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE-0.1)/DT_DMON)] == 2
    assert alert_lvls[int(DISTRACTED_SECONDS_TO_ORANGE*1.5/DT_DMON)] == 0
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE*3-0.1)/DT_DMON)] == 2
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE*3+0.1)/DT_DMON)] == 2
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE*3+2.5)/DT_DMON)] == 0

  # engaged, down to orange, driver dodges camera, then comes back still distracted, down to red, \
  #                          driver dodges, and then touches wheel to no avail, disengages and reengages
  #  - orange/red alert should remain after disappearance, and only disengaging clears red
  def test_biggest_comma_fan(self):
    _invisible_time = 2  # seconds
    ds_vector = always_distracted[:]
    interaction_vector = always_false[:]
    op_vector = always_true[:]
    ds_vector[int(DISTRACTED_SECONDS_TO_ORANGE/DT_DMON):int((DISTRACTED_SECONDS_TO_ORANGE+_invisible_time)/DT_DMON)] \
                                                        = [msg_NO_FACE_DETECTED] * int(_invisible_time/DT_DMON)
    ds_vector[int((DISTRACTED_SECONDS_TO_RED+_invisible_time)/DT_DMON):int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time)/DT_DMON)] \
                                                        = [msg_NO_FACE_DETECTED] * int(_invisible_time/DT_DMON)
    interaction_vector[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+0.5)/DT_DMON):int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+1.5)/DT_DMON)] \
                                                        = [True] * int(1/DT_DMON)
    op_vector[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+2.5)/DT_DMON):int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+3)/DT_DMON)] \
                                                        = [False] * int(0.5/DT_DMON)
    alert_lvls, _ = self._run_seq(ds_vector, interaction_vector, op_vector, always_false)
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_ORANGE+0.5*_invisible_time)/DT_DMON)] == 2
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_RED+1.5*_invisible_time)/DT_DMON)] == 3
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+1.5)/DT_DMON)] == 3
    assert alert_lvls[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+3.5)/DT_DMON)] == 0

  # engaged, invisible driver, down to orange, driver touches wheel; then down to orange again, driver appears
  #  - both actions should clear the alert, but momentary appearance should not
  def test_sometimes_transparent_commuter(self):
    _visible_time = np.random.choice([0.5, 10])
    ds_vector = always_no_face[:]*2
    interaction_vector = always_false[:]*2
    ds_vector[int((2*INVISIBLE_SECONDS_TO_ORANGE+1)/DT_DMON):int((2*INVISIBLE_SECONDS_TO_ORANGE+1+_visible_time)/DT_DMON)] = \
                                                                                             [msg_ATTENTIVE] * int(_visible_time/DT_DMON)
    interaction_vector[int((INVISIBLE_SECONDS_TO_ORANGE)/DT_DMON):int((INVISIBLE_SECONDS_TO_ORANGE+1)/DT_DMON)] = [True] * int(1/DT_DMON)
    alert_lvls, _ = self._run_seq(ds_vector, interaction_vector, 2*always_true, 2*always_false)
    assert alert_lvls[int(INVISIBLE_SECONDS_TO_ORANGE*0.5/DT_DMON)] == 0
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE-0.1)/DT_DMON)] == 2
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE+0.1)/DT_DMON)] == 0
    if _visible_time == 0.5:
      assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE*2+1-0.1)/DT_DMON)] == 2
      assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE*2+1+0.1+_visible_time)/DT_DMON)] == 1
    elif _visible_time == 10:
      assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE*2+1-0.1)/DT_DMON)] == 2
      assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE*2+1+0.1+_visible_time)/DT_DMON)] == 0

  # engaged, invisible driver, down to red, driver appears and then touches wheel, then disengages/reengages
  #  - only disengage will clear the alert
  def test_last_second_responder(self):
    _visible_time = 2  # seconds
    ds_vector = always_no_face[:]
    interaction_vector = always_false[:]
    op_vector = always_true[:]
    ds_vector[int(INVISIBLE_SECONDS_TO_RED/DT_DMON):int((INVISIBLE_SECONDS_TO_RED+_visible_time)/DT_DMON)] = [msg_ATTENTIVE] * int(_visible_time/DT_DMON)
    interaction_vector[int((INVISIBLE_SECONDS_TO_RED+_visible_time)/DT_DMON):int((INVISIBLE_SECONDS_TO_RED+_visible_time+1)/DT_DMON)] = [True] * int(1/DT_DMON)
    op_vector[int((INVISIBLE_SECONDS_TO_RED+_visible_time+1)/DT_DMON):int((INVISIBLE_SECONDS_TO_RED+_visible_time+0.5)/DT_DMON)] = [False] * int(0.5/DT_DMON)
    alert_lvls, _ = self._run_seq(ds_vector, interaction_vector, op_vector, always_false)
    assert alert_lvls[int(INVISIBLE_SECONDS_TO_ORANGE*0.5/DT_DMON)] == 0
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE-0.1)/DT_DMON)] == 2
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_RED-0.1)/DT_DMON)] == 3
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_RED+0.5*_visible_time)/DT_DMON)] == 3
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_RED+_visible_time+0.5)/DT_DMON)] == 3
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_RED+_visible_time+1+0.1)/DT_DMON)] == 0

  # disengaged, always distracted driver
  #  - dm should stay quiet when not engaged
  def test_pure_dashcam_user(self):
    alert_lvls, _ = self._run_seq(always_distracted, always_false, always_false, always_false)
    assert all(a == 0 for a in alert_lvls)

  # engaged, car stops at traffic light, down to orange, no action, then car starts moving
  #  - should only reach green when stopped, but continues counting down on launch
  def test_long_traffic_light_victim(self):
    _redlight_time = 60  # seconds
    standstill_vector = always_true[:]
    standstill_vector[int(_redlight_time/DT_DMON):] = [False] * int((TEST_TIMESPAN-_redlight_time)/DT_DMON)
    alert_lvls, d_status = self._run_seq(always_distracted, always_false, always_true, standstill_vector)
    s = d_status.settings
    assert alert_lvls[int((_redlight_time-0.1)/DT_DMON)] == 0
    _alert_1_to_2 = s._VISION_POLICY_ALERT_2_TIMEOUT - s._VISION_POLICY_ALERT_1_TIMEOUT
    assert alert_lvls[int((_redlight_time+0.5)/DT_DMON)] == 1
    assert alert_lvls[int((_redlight_time+_alert_1_to_2+0.5)/DT_DMON)] == 2

  # engaged, distracted while moving, then car stops after reaching orange
  #  - should reset timer to pre green at standstill
  def test_distracted_then_stops(self):
    _stop_time = DISTRACTED_SECONDS_TO_ORANGE + 1  # stop 1 second after reaching orange
    standstill_vector = always_false[:]
    standstill_vector[int(_stop_time/DT_DMON):] = [True] * int((TEST_TIMESPAN-_stop_time)/DT_DMON)
    alert_lvls, _ = self._run_seq(always_distracted, always_false, always_true, standstill_vector)
    # just before and briefly after stopping: orange alert; goes away quickly after stopped
    assert alert_lvls[int((_stop_time+0.1)/DT_DMON)] == 2
    assert alert_lvls[int((_stop_time+0.5)/DT_DMON)] == 0

  def test_lookaway_gate_helper(self):
    """2 mph is the gate: strictly below pauses; standstill still pauses at any vEgo."""
    assert lookaway_alerts_paused(False, 0.0)
    assert lookaway_alerts_paused(False, TEST_CREEP_MS)
    assert lookaway_alerts_paused(False, DM_LOOKAWAY_GATE_MS - 1e-6)
    assert not lookaway_alerts_paused(False, DM_LOOKAWAY_GATE_MS)
    assert not lookaway_alerts_paused(False, TEST_ABOVE_GATE_MS)
    assert not lookaway_alerts_paused(False, TEST_MOVING_MS)
    assert lookaway_alerts_paused(True, 20.0)

  def test_creeping_below_gate_never_reaches_green(self):
    """1 mph, CS.standstill false: no looking-away alert, any Sim Look / FAI combo."""
    for sim, fai in ((False, False), (True, False), (False, True)):
      alert_lvls, _ = self._run_seq(
        always_distracted, always_false, always_true, always_false,
        simulate_looking=sim, false_alert_ignore=fai, car_speed=TEST_CREEP_MS,
        rng_seed=1,
      )
      assert all(a == 0 for a in alert_lvls), (sim, fai)

  def test_creeping_no_face_never_reaches_green(self):
    """Eyes-off / no-face at a creep also stays quiet below the gate."""
    alert_lvls, _ = self._run_seq(
      always_no_face, always_false, always_true, always_false,
      simulate_looking=False, car_speed=TEST_CREEP_MS,
    )
    assert all(a == 0 for a in alert_lvls)

  def test_creeping_fai_pose_does_not_alert(self):
    """FAI On still lets pose drain when moving; below 2 mph it must not nag."""
    n = int(TEST_TIMESPAN / DT_DMON)
    alert_lvls, _ = self._run_seq(
      [msg_POSE_ONLY] * n, always_false, always_true, always_false,
      simulate_looking=False, false_alert_ignore=True,
      car_speed=TEST_CREEP_MS, rng_seed=3,
    )
    assert all(a == 0 for a in alert_lvls)

  def test_above_gate_stock_still_reaches_green(self):
    """Just above 2 mph: stock first prompt still fires (toggles Off)."""
    alert_lvls, d_status = self._run_seq(
      always_distracted, always_false, always_true, always_false,
      simulate_looking=False, car_speed=TEST_ABOVE_GATE_MS,
    )
    s = d_status.settings
    assert alert_lvls[int((s._VISION_POLICY_ALERT_1_TIMEOUT + 0.4) / DT_DMON)] == 1
    assert alert_lvls[int((s._VISION_POLICY_ALERT_2_TIMEOUT + 0.4) / DT_DMON)] == 2

  def test_above_gate_simulate_look_still_avoids_orange(self):
    """Above 2 mph, Sim Look On keeps prior 1–3 s full-wipe behavior."""
    alert_lvls, d_status = self._run_seq(
      always_distracted, always_false, always_true, always_false,
      simulate_looking=True, rng_seed=4, car_speed=TEST_ABOVE_GATE_MS,
    )
    assert all(a < 2 for a in alert_lvls)
    assert d_status.awareness > d_status.threshold_alert_2

  def test_creeping_then_launch_continues_countdown(self):
    """Like traffic-light standstill, but creeping 1 mph then rolling out."""
    _redlight_time = 60
    n = int(TEST_TIMESPAN / DT_DMON)
    i_go = int(_redlight_time / DT_DMON)
    speeds = [TEST_CREEP_MS] * i_go + [TEST_MOVING_MS] * (n - i_go)
    alert_lvls, d_status = self._run_seq(
      always_distracted, always_false, always_true, always_false,
      simulate_looking=False, car_speed=speeds,
    )
    s = d_status.settings
    assert alert_lvls[int((_redlight_time - 0.1) / DT_DMON)] == 0
    _alert_1_to_2 = s._VISION_POLICY_ALERT_2_TIMEOUT - s._VISION_POLICY_ALERT_1_TIMEOUT
    assert alert_lvls[int((_redlight_time + 0.5) / DT_DMON)] == 1
    assert alert_lvls[int((_redlight_time + _alert_1_to_2 + 0.5) / DT_DMON)] == 2

  def test_run_step_below_gate_no_lookaway_alert(self):
    """run_step uses vEgo: 1 mph + standstill false, toggles On or Off."""
    steps = int((dm_settings._VISION_POLICY_ALERT_2_TIMEOUT + 2.0) / DT_DMON)
    for sim, fai in ((False, False), (True, False), (False, True)):
      DM = DriverMonitoring()
      DM.nap_dm_simulate_looking = sim
      DM.nap_dm_false_alert_ignore = fai
      for _ in range(steps):
        DM.run_step(_fake_sm(hands=0, driver_state=msg_DISTRACTED,
                             v_ego=TEST_CREEP_MS, standstill=False))
      assert DM.alert_level == 0, (sim, fai)

  # engaged, model is somehow uncertain and driver is distracted
  #  - should fall back to wheel touch after uncertain alert
  def test_somehow_indecisive_model(self):
    ds_vector = [msg_DISTRACTED_BUT_SOMEHOW_UNCERTAIN] * int(TEST_TIMESPAN/DT_DMON)
    interaction_vector = always_false[:]
    alert_lvls, d_status = self._run_seq(ds_vector, interaction_vector, always_true, always_false)
    s = d_status.settings
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE-1+DT_DMON*s._HI_STD_FALLBACK_TIME-0.1)/DT_DMON)] == 1
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_ORANGE-1+DT_DMON*s._HI_STD_FALLBACK_TIME+0.1)/DT_DMON)] == 2
    assert alert_lvls[int((INVISIBLE_SECONDS_TO_RED-1+DT_DMON*s._HI_STD_FALLBACK_TIME+0.1)/DT_DMON)] == 3

  def test_stock_vision_timeouts_unchanged(self):
    s = DRIVER_MONITOR_SETTINGS()
    assert s._VISION_POLICY_ALERT_1_TIMEOUT == 3.
    assert s._VISION_POLICY_ALERT_2_TIMEOUT == 5.
    assert s._VISION_POLICY_ALERT_3_TIMEOUT == 11.
    assert PARAM_DM_SIMULATE_LOOKING == "NAPDmSimulateLooking"
    assert PARAM_DM_FALSE_ALERT_IGNORE == "NAPDmFalseAlertIgnore"
    assert DEFAULT_SIMULATE_LOOKING is True
    assert DEFAULT_FALSE_ALERT_IGNORE is False
    assert LOOK_SIM_MODE_PHONE == "phone"
    assert LOOK_SIM_MODE_GLANCE == "glance"
    assert LOOK_SIM_COUNTDOWN_MIN_S == 1.0
    assert LOOK_SIM_RANDOM_WINDOW_S == 2.0
    assert LOOK_SIM_FIRE_MAX_S == 3.0
    assert LOOK_SIM_HOLD_MIN_S == 0.5
    assert LOOK_SIM_HOLD_MAX_S == 12.0
    assert VISION_LOOKING_FILTER_X == 0.37
    assert s._TIMEOUT_RECOVERY_FACTOR_MAX == VISION_RECOVERY_FACTOR_MAX == 5.
    assert s._TIMEOUT_RECOVERY_FACTOR_MIN == VISION_RECOVERY_FACTOR_MIN == 1.25
    assert DM_LOOKAWAY_GATE_MPH == 2.0
    assert abs(DM_LOOKAWAY_GATE_MS - 2.0 * CV.MPH_TO_MS) < 1e-9

  def test_vision_looking_path_is_stock_glance_predicates(self):
    """Green-prompt clear path: face + low std + filter.x < 0.37."""
    assert vision_looking_path(True, True, 0.0)
    assert vision_looking_path(True, True, VISION_LOOKING_FILTER_X - 0.01)
    assert not vision_looking_path(True, True, VISION_LOOKING_FILTER_X)
    assert not vision_looking_path(False, True, 0.0)
    assert not vision_looking_path(True, False, 0.0)
    assert not vision_looking_path(True, True, 0.63)

  def test_looking_recovery_time_matches_policy_math(self):
    """Closed-form hold length matches discrete stock recovery to 1.0."""
    s = DRIVER_MONITOR_SETTINGS()
    t3 = s._VISION_POLICY_ALERT_3_TIMEOUT
    step = DT_DMON / t3
    for deficit_s in (1.0, 2.0, 3.0, 5.0):
      awareness = 1.0 - deficit_s / t3
      predicted = looking_recovery_time_s(awareness, t3)
      a = awareness
      frames = 0
      while a < 1.0 - 1e-12 and frames < 400:
        a = min(a + ((s._TIMEOUT_RECOVERY_FACTOR_MAX - s._TIMEOUT_RECOVERY_FACTOR_MIN) *
                     (1. - a) + s._TIMEOUT_RECOVERY_FACTOR_MIN) * step, 1.)
        frames += 1
      discrete = frames * DT_DMON
      assert abs(discrete - predicted) < 2 * DT_DMON + 1e-6
      assert predicted >= LOOK_SIM_HOLD_MIN_S or deficit_s < 1.0

  def _dm(self, *, simulate_looking=False, false_alert_ignore=False, fire_s=2.0):
    DM = DriverMonitoring()
    DM.nap_dm_simulate_looking = bool(simulate_looking)
    DM.nap_dm_false_alert_ignore = bool(false_alert_ignore)
    DM._look_sim_fire_s = fire_s
    return DM

  def _step(self, DM, msg, *, car_speed=TEST_MOVING_MS, standstill=False):
    DM._update_states(msg, [0, 0, 0], car_speed, True, standstill)
    DM._update_events(False, True, standstill, 0)

  def test_simulate_looking_holds_until_awareness_recovers(self):
    """Full-wipe hold is many frames on the looking-path; awareness to 1.0."""
    DM = self._dm(simulate_looking=True)
    hold_frames = 0
    start_a = None
    first_hold_a = None
    recovered = False
    for _ in range(int(10.0 / DT_DMON)):
      self._step(DM, msg_NO_FACE_DETECTED)
      if DM._look_sim_holding:
        hold_frames += 1
        assert DM._look_sim_mode == LOOK_SIM_MODE_GLANCE
        assert vision_looking_path(DM.face_detected, DM.pose.low_std,
                                   DM.driver_distraction_filter.x)
        if start_a is None:
          start_a = DM._look_sim_hold_start_awareness
          first_hold_a = DM.awareness
        if DM.awareness >= 1.0 - 1e-9 and hold_frames * DT_DMON >= LOOK_SIM_HOLD_MIN_S:
          recovered = True
          break
    assert recovered
    assert start_a is not None and start_a < 0.95
    assert first_hold_a is not None and first_hold_a < 1.0
    assert hold_frames >= int(LOOK_SIM_HOLD_MIN_S / DT_DMON)
    predicted = looking_recovery_time_s(start_a, DM.settings._VISION_POLICY_ALERT_3_TIMEOUT)
    assert hold_frames * DT_DMON + DT_DMON >= max(LOOK_SIM_HOLD_MIN_S, predicted) - 2 * DT_DMON
    assert DM.alert_level == 0

  def test_simulate_looking_fires_in_1_to_3s_window(self):
    """No hold until drain is past 1.0 s; fire time is uniform in (1.0 s, 3.0 s]."""
    DM = self._dm(simulate_looking=True, fire_s=0.4)  # would fire early without the 1 s gate
    drain_start = None
    hold_start = None
    for i in range(int(6.0 / DT_DMON)):
      was_holding = DM._look_sim_holding
      self._step(DM, msg_NO_FACE_DETECTED)
      if drain_start is None and DM.awareness < 1.0 and not DM._look_sim_holding:
        drain_start = i * DT_DMON
      if hold_start is None and DM._look_sim_holding and not was_holding:
        hold_start = i * DT_DMON
        break
    assert drain_start is not None and hold_start is not None
    assert (hold_start - drain_start) > LOOK_SIM_COUNTDOWN_MIN_S - 1e-6
    assert (hold_start - drain_start) <= LOOK_SIM_FIRE_MAX_S + 0.15

    elapsed = []
    for seed in range(16):
      DM = DriverMonitoring()
      DM.nap_dm_simulate_looking = True
      DM.nap_dm_false_alert_ignore = False
      DM._rng.seed(seed)
      DM._redraw_look_sim_interval()
      fire_s = DM._look_sim_fire_s
      assert LOOK_SIM_COUNTDOWN_MIN_S < fire_s <= LOOK_SIM_FIRE_MAX_S
      drain_start = None
      hold_start = None
      for i in range(int(8.0 / DT_DMON)):
        was_holding = DM._look_sim_holding
        self._step(DM, msg_NO_FACE_DETECTED)
        if drain_start is None and DM.awareness < 1.0 and not DM._look_sim_holding:
          drain_start = i * DT_DMON
        if hold_start is None and DM._look_sim_holding and not was_holding:
          hold_start = i * DT_DMON
          break
      assert drain_start is not None and hold_start is not None
      dt = hold_start - drain_start
      assert abs(dt - fire_s) < 0.15
      assert LOOK_SIM_COUNTDOWN_MIN_S < dt <= LOOK_SIM_FIRE_MAX_S + 0.15
      elapsed.append(dt)
    assert max(elapsed) - min(elapsed) > 0.5
    assert min(elapsed) > LOOK_SIM_COUNTDOWN_MIN_S - 1e-6
    assert max(elapsed) <= LOOK_SIM_FIRE_MAX_S + 0.15

  def test_simulate_looking_fire_time_redraws_in_window(self):
    DM = DriverMonitoring()
    DM.nap_dm_simulate_looking = True
    DM._rng.seed(11)
    seen = []
    for _ in range(40):
      DM._redraw_look_sim_interval()
      t = DM._look_sim_fire_s
      assert LOOK_SIM_COUNTDOWN_MIN_S < t <= LOOK_SIM_FIRE_MAX_S
      seen.append(round(t, 4))
    assert len(set(seen)) >= 8
    assert not all(abs(x - seen[0]) < 1e-6 for x in seen)
    assert any(x <= LOOK_SIM_COUNTDOWN_MIN_S + 0.5 for x in seen)
    assert any(x >= LOOK_SIM_FIRE_MAX_S - 0.5 for x in seen)

  def test_simulate_looking_engaged_never_reaches_orange(self):
    """Periodic full-wipe looking-path resets keep awareness above orange."""
    alert_lvls, d_status = self._run_seq(always_distracted, always_false, always_true,
                                         always_false, simulate_looking=True, rng_seed=4)
    assert all(a < 2 for a in alert_lvls)
    assert d_status.awareness > d_status.threshold_alert_2
    assert d_status.alert_level in (0, 1)

  def test_simulate_looking_no_face_also_resets(self):
    """maybe_distracted / no-face drain still gets a simulated glance."""
    alert_lvls, d_status = self._run_seq(always_no_face, always_false, always_true,
                                         always_false, simulate_looking=True, rng_seed=2)
    assert all(a < 2 for a in alert_lvls)
    assert d_status.awareness > 0.5

  def test_simulate_looking_clears_already_orange(self):
    """If he is already in orange, the next wipe pulse still resets."""
    DM = self._dm(simulate_looking=False)
    for _ in range(int((dm_settings._VISION_POLICY_ALERT_2_TIMEOUT + 0.4) / DT_DMON)):
      self._step(DM, msg_DISTRACTED)
    assert DM.alert_level == 2
    DM.set_nap_dm_toggles(simulate_looking=True)
    DM._look_sim_fire_s = 2.0
    DM._look_sim_countdown_s = 0.0
    cleared = False
    for _ in range(int(12.0 / DT_DMON)):
      self._step(DM, msg_DISTRACTED)
      if DM.awareness >= 1.0 - 1e-9 and DM.alert_level == 0:
        assert vision_looking_path(DM.face_detected, DM.pose.low_std,
                                   DM.driver_distraction_filter.x)
        assert DM._look_sim_hold_s + DT_DMON >= LOOK_SIM_HOLD_MIN_S
        cleared = True
        break
    assert cleared

  def test_simulate_looking_recovers_eye(self):
    """Eye blink drain is wiped on cadence; never reaches orange."""
    alert_lvls, d_status = self._run_seq(always_distracted, always_false, always_true,
                                         always_false, simulate_looking=True, rng_seed=4)
    assert all(a < 2 for a in alert_lvls)
    assert d_status.awareness > d_status.threshold_alert_2

  def test_simulate_looking_recovers_phone(self):
    """Phone-only drain is wiped on cadence when Simulate Look is On."""
    n = int(TEST_TIMESPAN / DT_DMON)
    alert_lvls, d_status = self._run_seq(
      [msg_PHONE_ONLY] * n, always_false, always_true, always_false,
      simulate_looking=True, false_alert_ignore=False, rng_seed=2,
    )
    assert all(a < 2 for a in alert_lvls)
    assert d_status.awareness > d_status.threshold_alert_2

  def test_simulate_looking_recovers_pose(self):
    """Pose drain is wiped on cadence when Simulate Look is On."""
    n = int(TEST_TIMESPAN / DT_DMON)
    alert_lvls, d_status = self._run_seq(
      [msg_POSE_ONLY] * n, always_false, always_true, always_false,
      simulate_looking=True, false_alert_ignore=False, rng_seed=3,
    )
    assert all(a < 2 for a in alert_lvls)
    assert d_status.awareness > d_status.threshold_alert_2

  def test_simulate_looking_off_is_stock(self):
    """Toggle Off: first prompt → orange → red on stock 3 / 5 / 11 s."""
    alert_lvls, d_status = self._run_seq(always_distracted, always_false, always_true,
                                         always_false, simulate_looking=False)
    s = d_status.settings
    assert alert_lvls[int(s._VISION_POLICY_ALERT_1_TIMEOUT / 2 / DT_DMON)] == 0
    assert alert_lvls[int((s._VISION_POLICY_ALERT_1_TIMEOUT + 0.4) / DT_DMON)] == 1
    assert alert_lvls[int((s._VISION_POLICY_ALERT_2_TIMEOUT + 0.4) / DT_DMON)] == 2
    assert alert_lvls[int((s._VISION_POLICY_ALERT_3_TIMEOUT + 0.4) / DT_DMON)] == 3

  def test_always_on_disengaged_does_not_simulate(self):
    """AlwaysOnDM when not engaged still counts; no simulated glances."""
    n = int(TEST_TIMESPAN / DT_DMON)
    DM = DriverMonitoring(always_on=True)
    DM.nap_dm_simulate_looking = True
    DM.nap_dm_false_alert_ignore = True
    alert_lvls = []
    for idx in range(n):
      DM._update_states(always_distracted[idx], [0, 0, 0], TEST_MOVING_MS, False, False)
      DM._update_events(False, False, False, 0)
      alert_lvls.append(DM.alert_level)
    s = DM.settings
    orange_i = int((s._VISION_POLICY_ALERT_2_TIMEOUT + 0.6) / DT_DMON)
    assert 1 in alert_lvls[:orange_i]
    assert alert_lvls[orange_i] == 2

  def test_run_step_simulate_looking_holds_above_orange(self):
    DM = DriverMonitoring()
    DM.nap_dm_simulate_looking = True
    DM.nap_dm_false_alert_ignore = False
    DM._rng.seed(5)
    DM._redraw_look_sim_interval()
    steps = int((dm_settings._VISION_POLICY_ALERT_2_TIMEOUT + 1.0) / DT_DMON)
    for _ in range(steps):
      DM.run_step(_fake_sm(hands=0, driver_state=msg_DISTRACTED))
    assert DM.alert_level < 2
    assert DM.awareness > DM.threshold_alert_2

  def test_run_step_param_off_is_stock(self):
    DM = DriverMonitoring()
    DM.nap_dm_simulate_looking = False
    DM.nap_dm_false_alert_ignore = False
    steps = int((dm_settings._VISION_POLICY_ALERT_1_TIMEOUT + 0.4) / DT_DMON)
    for _ in range(steps):
      DM.run_step(_fake_sm(hands=0, driver_state=msg_DISTRACTED))
    assert DM.alert_level == 1
    for _ in range(int((dm_settings._VISION_POLICY_ALERT_2_TIMEOUT -
                        dm_settings._VISION_POLICY_ALERT_1_TIMEOUT + 0.5) / DT_DMON)):
      DM.run_step(_fake_sm(hands=0, driver_state=msg_DISTRACTED))
    assert DM.alert_level == 2

  def test_false_alert_ignore_phone_only_recovers(self):
    """Phone-only + FAI On: awareness recovers on the 1–3 s cadence without a glance."""
    n = int(TEST_TIMESPAN / DT_DMON)
    alert_lvls, d_status = self._run_seq(
      [msg_PHONE_ONLY] * n, always_false, always_true, always_false,
      false_alert_ignore=True, rng_seed=4,
    )
    assert all(a < 2 for a in alert_lvls)
    assert d_status.awareness > d_status.threshold_alert_2
    assert d_status.alert_level in (0, 1)

  def test_false_alert_ignore_phone_soft_clear_not_pose_eye(self):
    """Hold clears the phone bit only; pose/eye types stay false on phone-only."""
    DM = self._dm(false_alert_ignore=True)
    recovered = False
    for _ in range(int(10.0 / DT_DMON)):
      self._step(DM, msg_PHONE_ONLY)
      if DM._look_sim_holding:
        assert DM._look_sim_mode == LOOK_SIM_MODE_PHONE
        assert DM.distracted_types['phone'] is False
        assert DM.distracted_types['pose'] is False
        assert DM.distracted_types['eye'] is False
        assert not DM.driver_distracted
        assert vision_looking_path(DM.face_detected, DM.pose.low_std,
                                   DM.driver_distraction_filter.x)
        if DM.awareness >= 1.0 - 1e-9 and DM._look_sim_hold_s + 1e-9 >= LOOK_SIM_HOLD_MIN_S:
          recovered = True
          break
    assert recovered
    assert DM.alert_level == 0

  def test_false_alert_ignore_pose_still_drains(self):
    """Pose alarming: FAI On (Sim Off) does not reset the timer."""
    n = int(TEST_TIMESPAN / DT_DMON)
    alert_lvls, d_status = self._run_seq(
      [msg_POSE_ONLY] * n, always_false, always_true, always_false,
      simulate_looking=False, false_alert_ignore=True, rng_seed=3,
    )
    s = d_status.settings
    assert alert_lvls[int((s._VISION_POLICY_ALERT_2_TIMEOUT + 0.4) / DT_DMON)] == 2
    assert alert_lvls[int((s._VISION_POLICY_ALERT_3_TIMEOUT + 0.4) / DT_DMON)] == 3
    assert d_status.distracted_types['pose']
    assert not d_status._look_sim_holding

  def test_false_alert_ignore_eye_still_drains(self):
    """Eye alarming: FAI On (Sim Off) does not reset; orange/red on stock timers."""
    n = int(TEST_TIMESPAN / DT_DMON)
    alert_lvls, d_status = self._run_seq(
      [msg_DISTRACTED] * n, always_false, always_true, always_false,
      simulate_looking=False, false_alert_ignore=True, rng_seed=3,
    )
    s = d_status.settings
    assert alert_lvls[int((s._VISION_POLICY_ALERT_2_TIMEOUT + 0.4) / DT_DMON)] == 2
    assert alert_lvls[int((s._VISION_POLICY_ALERT_3_TIMEOUT + 0.4) / DT_DMON)] == 3
    assert d_status.distracted_types['eye']

  def test_false_alert_ignore_phone_plus_eye_eye_wins(self):
    """Phone + eye: FAI On (Sim Off) does not start a phone hold; eye drains."""
    DM = self._dm(simulate_looking=False, false_alert_ignore=True, fire_s=1.2)
    for _ in range(int((dm_settings._VISION_POLICY_ALERT_2_TIMEOUT + 0.4) / DT_DMON)):
      self._step(DM, msg_PHONE_AND_EYE)
    assert DM.distracted_types['eye']
    assert DM.distracted_types['phone']
    assert not DM._look_sim_holding
    assert DM.alert_level == 2

  def test_false_alert_ignore_phone_plus_pose_pose_wins(self):
    """Phone + pose: FAI On (Sim Off) does not reset; pose keeps draining."""
    DM = self._dm(simulate_looking=False, false_alert_ignore=True, fire_s=1.2)
    for _ in range(int((dm_settings._VISION_POLICY_ALERT_2_TIMEOUT + 0.4) / DT_DMON)):
      self._step(DM, msg_PHONE_AND_POSE)
    assert DM.distracted_types['pose']
    assert DM.distracted_types['phone']
    assert not DM._look_sim_holding
    assert DM.alert_level == 2
    assert DM.awareness <= DM.threshold_alert_2

  def test_false_alert_ignore_phone_clears_after_pose_drops(self):
    """After pose clears, remaining phone-only can soft-clear on the same cadence."""
    DM = self._dm(false_alert_ignore=True, fire_s=1.2)
    for _ in range(int(3.5 / DT_DMON)):
      self._step(DM, msg_PHONE_AND_POSE)
    assert DM.distracted_types['pose']
    assert not DM._look_sim_holding
    drained = DM.awareness
    assert drained < 0.8
    recovered = False
    for _ in range(int(10.0 / DT_DMON)):
      self._step(DM, msg_PHONE_ONLY)
      if DM._look_sim_holding:
        assert DM._look_sim_mode == LOOK_SIM_MODE_PHONE
        assert DM.distracted_types['pose'] is False
        assert DM.distracted_types['phone'] is False
      if DM.awareness >= 1.0 - 1e-9 and DM.alert_level == 0:
        recovered = True
        break
    assert recovered
    assert DM.awareness > drained

  def test_false_alert_ignore_off_phone_is_stock_without_sim_look(self):
    """Both Off: phone-only drains on stock timers."""
    n = int(TEST_TIMESPAN / DT_DMON)
    alert_lvls, d_status = self._run_seq(
      [msg_PHONE_ONLY] * n, always_false, always_true, always_false,
      simulate_looking=False, false_alert_ignore=False, rng_seed=2,
    )
    s = d_status.settings
    assert alert_lvls[int((s._VISION_POLICY_ALERT_2_TIMEOUT + 0.4) / DT_DMON)] == 2
    assert alert_lvls[int((s._VISION_POLICY_ALERT_3_TIMEOUT + 0.4) / DT_DMON)] == 3
    assert d_status.distracted_types['phone']

  def test_false_alert_ignore_off_both_off_phone_is_stock(self):
    n = int(TEST_TIMESPAN / DT_DMON)
    alert_lvls, d_status = self._run_seq(
      [msg_PHONE_ONLY] * n, always_false, always_true, always_false,
    )
    s = d_status.settings
    assert alert_lvls[int((s._VISION_POLICY_ALERT_1_TIMEOUT + 0.4) / DT_DMON)] == 1
    assert alert_lvls[int((s._VISION_POLICY_ALERT_2_TIMEOUT + 0.4) / DT_DMON)] == 2

  def test_false_alert_ignore_aborts_hold_when_pose_starts(self):
    """Mid-hold pose must abort and resume draining (no awareness reset)."""
    DM = self._dm(false_alert_ignore=True, fire_s=1.2)
    holding = False
    for _ in range(int(6.0 / DT_DMON)):
      self._step(DM, msg_PHONE_ONLY)
      if DM._look_sim_holding:
        holding = True
        a_at_hold = DM.awareness
        break
    assert holding
    for _ in range(int(2.0 / DT_DMON)):
      self._step(DM, msg_PHONE_AND_POSE)
    assert not DM._look_sim_holding
    assert DM.distracted_types['pose']
    assert DM.awareness < a_at_hold + 1e-6
    for _ in range(int((dm_settings._VISION_POLICY_ALERT_2_TIMEOUT) / DT_DMON)):
      self._step(DM, msg_PHONE_AND_POSE)
    assert DM.alert_level >= 2

  def test_simulate_looking_full_wipe_clears_phone(self):
    """Simulate Look On owns the full wipe, including phone false nags."""
    DM = self._dm(simulate_looking=True, false_alert_ignore=False, fire_s=1.2)
    holding = False
    for _ in range(int(4.0 / DT_DMON)):
      self._step(DM, msg_PHONE_ONLY)
      if DM._look_sim_holding:
        holding = True
        assert DM._look_sim_mode == LOOK_SIM_MODE_GLANCE
        assert DM.distracted_types['phone'] is False
        assert not DM.driver_distracted
        assert vision_looking_path(DM.face_detected, DM.pose.low_std,
                                   DM.driver_distraction_filter.x)
        break
    assert holding

  def test_simulate_looking_keeps_hold_when_pose_starts(self):
    """Mid-hold pose must not abort Simulate Look the way FAI aborts."""
    DM = self._dm(simulate_looking=True, fire_s=1.2)
    holding = False
    for _ in range(int(6.0 / DT_DMON)):
      self._step(DM, msg_NO_FACE_DETECTED)
      if DM._look_sim_holding:
        holding = True
        a_at_hold = DM.awareness
        break
    assert holding
    self._step(DM, msg_POSE_ONLY)
    assert DM._look_sim_holding
    assert DM._look_sim_mode == LOOK_SIM_MODE_GLANCE
    assert DM.distracted_types['pose'] is False
    assert not DM.driver_distracted
    assert DM.awareness >= a_at_hold - 1e-6
    for _ in range(int((dm_settings._VISION_POLICY_ALERT_2_TIMEOUT + 1.0) / DT_DMON)):
      self._step(DM, msg_POSE_ONLY)
    assert DM.alert_level < 2
    assert DM.awareness > DM.threshold_alert_2

  def test_simulate_looking_hold_recovers_phone_pose_eye(self):
    """One cadence hold recovers awareness for phone, pose, and eye inputs."""
    for msg in (msg_PHONE_ONLY, msg_POSE_ONLY, msg_DISTRACTED, msg_PHONE_AND_POSE,
                msg_PHONE_AND_EYE):
      DM = self._dm(simulate_looking=True, fire_s=1.2)
      recovered = False
      for _ in range(int(10.0 / DT_DMON)):
        self._step(DM, msg)
        if DM._look_sim_holding:
          assert DM._look_sim_mode == LOOK_SIM_MODE_GLANCE
          assert DM.distracted_types['phone'] is False
          assert DM.distracted_types['pose'] is False
          assert DM.distracted_types['eye'] is False
          assert not DM.driver_distracted
        if DM.awareness >= 1.0 - 1e-9 and DM.alert_level == 0:
          recovered = True
          break
      assert recovered, f"did not recover for {msg}"

  def test_run_step_false_alert_ignore_phone_holds_above_orange(self):
    DM = DriverMonitoring()
    DM.nap_dm_simulate_looking = False
    DM.nap_dm_false_alert_ignore = True
    DM._rng.seed(5)
    DM._redraw_look_sim_interval()
    steps = int((dm_settings._VISION_POLICY_ALERT_2_TIMEOUT + 1.0) / DT_DMON)
    for _ in range(steps):
      DM.run_step(_fake_sm(hands=0, driver_state=msg_PHONE_ONLY))
    assert DM.alert_level < 2
    assert DM.awareness > DM.threshold_alert_2

  def test_stale_both_on_prefers_simulate_look_full_wipe(self):
    """Both-On (old install) resolves to Simulate Look; phone recovers via wipe."""
    DM = self._dm(simulate_looking=True, false_alert_ignore=True, fire_s=1.2)
    holding = False
    for _ in range(int(4.0 / DT_DMON)):
      self._step(DM, msg_PHONE_ONLY)
      if DM._look_sim_holding:
        holding = True
        assert DM._look_sim_mode == LOOK_SIM_MODE_GLANCE
        assert DM.distracted_types['phone'] is False
        break
    assert DM.nap_dm_simulate_looking is True
    assert DM.nap_dm_false_alert_ignore is False
    assert holding
    glance = False
    DM._end_look_sim_hold(redraw=True)
    DM._look_sim_fire_s = 1.2
    for _ in range(int(6.0 / DT_DMON)):
      self._step(DM, msg_NO_FACE_DETECTED)
      if DM._look_sim_holding:
        assert DM._look_sim_mode == LOOK_SIM_MODE_GLANCE
        glance = True
        break
    assert glance

  def test_turning_fai_on_aborts_glance_hold(self):
    DM = self._dm(simulate_looking=True, fire_s=1.2)
    holding = False
    for _ in range(int(6.0 / DT_DMON)):
      self._step(DM, msg_NO_FACE_DETECTED)
      if DM._look_sim_holding:
        holding = True
        assert DM._look_sim_mode == LOOK_SIM_MODE_GLANCE
        break
    assert holding
    DM.set_nap_dm_toggles(false_alert_ignore=True)
    assert DM.nap_dm_simulate_looking is False
    assert DM.nap_dm_false_alert_ignore is True
    assert not DM._look_sim_holding
    assert DM._look_sim_mode is None

  def test_turning_simulate_look_on_stops_phone_hold(self):
    DM = self._dm(false_alert_ignore=True, fire_s=1.2)
    holding = False
    for _ in range(int(6.0 / DT_DMON)):
      self._step(DM, msg_PHONE_ONLY)
      if DM._look_sim_holding:
        holding = True
        assert DM._look_sim_mode == LOOK_SIM_MODE_PHONE
        break
    assert holding
    DM.set_nap_dm_toggles(simulate_looking=True)
    assert DM.nap_dm_simulate_looking is True
    assert DM.nap_dm_false_alert_ignore is False
    assert not DM._look_sim_holding
    assert DM._look_sim_mode is None

  def test_set_both_true_resolves_to_simulate_look(self):
    DM = self._dm(simulate_looking=False, false_alert_ignore=True)
    DM.set_nap_dm_toggles(simulate_looking=True, false_alert_ignore=True)
    assert DM.nap_dm_simulate_looking is True
    assert DM.nap_dm_false_alert_ignore is False
