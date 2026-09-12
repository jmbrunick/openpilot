import numpy as np

from cereal import log
from openpilot.common.realtime import DT_DMON
from openpilot.selfdrive.monitoring.policy import (
  DriverMonitoring, DRIVER_MONITOR_SETTINGS, PARAM_DM_HANDS_ON_RESET,
  HANDS_ON_DM_RESET_LEVEL, VISION_ALERT_1_TIMEOUT_MIN, VISION_ALERT_1_TIMEOUT_MAX,
  in_first_prompt_band, cs_hands_on_level_for_dm,
)

EventName = log.OnroadEvent.EventName
dm_settings = DRIVER_MONITOR_SETTINGS()

TEST_TIMESPAN = 120  # seconds
DISTRACTED_SECONDS_TO_ORANGE = dm_settings._VISION_POLICY_ALERT_2_TIMEOUT + 1
DISTRACTED_SECONDS_TO_RED = dm_settings._VISION_POLICY_ALERT_3_TIMEOUT + 1
INVISIBLE_SECONDS_TO_ORANGE = dm_settings._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT + 1
INVISIBLE_SECONDS_TO_RED = dm_settings._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT + 1

def make_msg(face_detected, distracted=False, model_uncertain=False):
  ds = log.DriverStateV2.new_message()
  ds.leftDriverData.faceOrientation = [0., 0., 0.]
  ds.leftDriverData.facePosition = [0., 0.]
  ds.leftDriverData.faceProb = 1. * face_detected
  ds.leftDriverData.leftEyeProb = 1.
  ds.leftDriverData.rightEyeProb = 1.
  ds.leftDriverData.leftBlinkProb = 1. * distracted
  ds.leftDriverData.rightBlinkProb = 1. * distracted
  ds.leftDriverData.faceOrientationStd = [1.*model_uncertain, 1.*model_uncertain, 1.*model_uncertain]
  ds.leftDriverData.facePositionStd = [1.*model_uncertain, 1.*model_uncertain]
  # TODO: test both separately when e2e is used
  ds.leftDriverData.phoneProb = 0.
  return ds


# driver state from neural net, 10Hz
msg_NO_FACE_DETECTED = make_msg(False)
msg_ATTENTIVE = make_msg(True)
msg_DISTRACTED = make_msg(True, distracted=True)
msg_ATTENTIVE_UNCERTAIN = make_msg(True, model_uncertain=True)
msg_DISTRACTED_UNCERTAIN = make_msg(True, distracted=True, model_uncertain=True)
msg_DISTRACTED_BUT_SOMEHOW_UNCERTAIN = make_msg(True, distracted=True, model_uncertain=dm_settings._HI_STD_THRESHOLD*1.5)

# driver interaction with car
car_interaction_DETECTED = True
car_interaction_NOT_DETECTED = False

# some common state vectors
always_no_face = [msg_NO_FACE_DETECTED] * int(TEST_TIMESPAN / DT_DMON)
always_attentive = [msg_ATTENTIVE] * int(TEST_TIMESPAN / DT_DMON)
always_distracted = [msg_DISTRACTED] * int(TEST_TIMESPAN / DT_DMON)
always_true = [True] * int(TEST_TIMESPAN / DT_DMON)
always_false = [False] * int(TEST_TIMESPAN / DT_DMON)

class _NS:
  def __init__(self, **kw):
    self.__dict__.update(kw)


def _fake_sm(*, engaged=True, hands=1, steer_pressed=False, gas_pressed=False,
             driver_state=None, standstill=False):
  from cereal import car
  return {
    'carState': _NS(
      vEgo=20.0,
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
  def _run_seq(self, msgs, interaction, engaged, standstill, soft_presence=None,
               nap_dm=False, rng_seed=None):
    DM = DriverMonitoring()
    DM.nap_dm_hands_on_reset = bool(nap_dm)
    if rng_seed is not None:
      DM._rng.seed(rng_seed)
    DM._redraw_vision_alert_1()
    DM._apply_vision_alert_thresholds()
    alert_lvls = []
    for idx in range(len(msgs)):
      DM._update_states(msgs[idx], [0, 0, 0], 0, engaged[idx], standstill[idx])
      # cal_rpy and car_speed don't matter here

      # evaluate events at 10Hz for tests
      sp = False if soft_presence is None else bool(soft_presence[idx])
      DM._update_events(interaction[idx], engaged[idx], standstill[idx], 0, soft_presence=sp)
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

  def test_hands_on_reader_matches_soft_lat(self):
    """DM uses the same cereal / stash / disengage read as soft-lat."""
    from openpilot.selfdrive.controls.lib.driver_lateral_handoff import cs_hands_on_level
    cases = [
      _NS(handsOnLevel=1, steeringTorqueEps=0.0, steeringDisengage=False),
      _NS(handsOnLevel=0, steeringTorqueEps=1.0, steeringDisengage=False),
      _NS(handsOnLevel=0, steeringTorqueEps=0.0, steeringDisengage=True),
      _NS(handsOnLevel=0, steeringTorqueEps=0.0, steeringDisengage=False),
    ]
    for cs in cases:
      assert cs_hands_on_level_for_dm(cs) == cs_hands_on_level(cs)
    assert HANDS_ON_DM_RESET_LEVEL == 1

  def test_stock_vision_timeouts_unchanged(self):
    s = DRIVER_MONITOR_SETTINGS()
    assert s._VISION_POLICY_ALERT_1_TIMEOUT == 3.
    assert s._VISION_POLICY_ALERT_2_TIMEOUT == 5.
    assert s._VISION_POLICY_ALERT_3_TIMEOUT == 11.
    assert s._VISION_POLICY_ALERT_1_TIMEOUT_MIN == VISION_ALERT_1_TIMEOUT_MIN == 2.0
    assert s._VISION_POLICY_ALERT_1_TIMEOUT_MAX == VISION_ALERT_1_TIMEOUT_MAX == 4.5
    assert s._VISION_POLICY_ALERT_1_TIMEOUT_MAX < s._VISION_POLICY_ALERT_2_TIMEOUT
    assert PARAM_DM_HANDS_ON_RESET == "NAPDmHandsOnReset"
    assert HANDS_ON_DM_RESET_LEVEL == 1

  def test_first_prompt_band_helper(self):
    s = DRIVER_MONITOR_SETTINGS()
    t1 = 1. - s._VISION_POLICY_ALERT_1_TIMEOUT / s._VISION_POLICY_ALERT_3_TIMEOUT
    t2 = 1. - s._VISION_POLICY_ALERT_2_TIMEOUT / s._VISION_POLICY_ALERT_3_TIMEOUT
    step = DT_DMON / s._VISION_POLICY_ALERT_3_TIMEOUT
    assert not in_first_prompt_band(1.0, step, t1, t2)
    assert in_first_prompt_band(t1 + step, step, t1, t2)
    assert in_first_prompt_band(t1, step, t1, t2)
    assert in_first_prompt_band((t1 + t2) / 2, step, t1, t2)
    assert not in_first_prompt_band(t2, step, t1, t2)
    assert not in_first_prompt_band(0.0, step, t1, t2)

  def test_preap_hands_on_resets_near_alert_1(self):
    """Light hands-on at the first look-at-road band resets; does not keep barking."""
    n = int(TEST_TIMESPAN / DT_DMON)
    hands = [True] * n
    alert_lvls, d_status = self._run_seq(always_distracted, always_false, always_true,
                                         always_false, soft_presence=hands, nap_dm=True)
    s = d_status.settings
    assert alert_lvls[int(s._VISION_POLICY_ALERT_1_TIMEOUT_MIN / 2 / DT_DMON)] == 0
    assert alert_lvls[int((s._VISION_POLICY_ALERT_2_TIMEOUT + 0.5) / DT_DMON)] == 0
    assert all(a < 2 for a in alert_lvls)
    assert d_status.awareness > d_status.threshold_alert_2

  def test_ignored_dm_still_escalates_without_hands(self):
    """No rim contact: first prompt (random band), then orange, then red."""
    alert_lvls, d_status = self._run_seq(always_distracted, always_false, always_true,
                                         always_false, soft_presence=always_false, nap_dm=True)
    s = d_status.settings
    orange_i = int((s._VISION_POLICY_ALERT_2_TIMEOUT + 0.6) / DT_DMON)
    red_i = int((s._VISION_POLICY_ALERT_3_TIMEOUT + 0.6) / DT_DMON)
    assert 1 in alert_lvls[:orange_i]
    assert alert_lvls[orange_i] == 2
    assert alert_lvls[red_i] == 3

  def test_hands_on_does_not_clear_orange_or_red(self):
    """After orange / red, light hands-on is not enough — still escalate."""
    n = int(TEST_TIMESPAN / DT_DMON)
    orange_i = int((DISTRACTED_SECONDS_TO_ORANGE + 0.2) / DT_DMON)
    red_i = int((DISTRACTED_SECONDS_TO_RED + 0.2) / DT_DMON)
    hands = [False] * n
    for i in range(orange_i, min(orange_i + int(1 / DT_DMON), n)):
      hands[i] = True
    alert_lvls, _ = self._run_seq(always_distracted, always_false, always_true,
                                  always_false, soft_presence=hands, nap_dm=True)
    assert alert_lvls[orange_i] == 2

    hands_red = [False] * n
    for i in range(red_i, min(red_i + int(1 / DT_DMON), n)):
      hands_red[i] = True
    alert_lvls_red, _ = self._run_seq(always_distracted, always_false, always_true,
                                      always_false, soft_presence=hands_red, nap_dm=True)
    assert alert_lvls_red[red_i] == 3

  def test_always_on_disengaged_ignores_soft_presence(self):
    """AlwaysOnDM when not engaged still counts; hands-on must not wipe it."""
    n = int(TEST_TIMESPAN / DT_DMON)
    DM = DriverMonitoring(always_on=True)
    DM.nap_dm_hands_on_reset = True
    alert_lvls = []
    for idx in range(n):
      DM._update_states(always_distracted[idx], [0, 0, 0], 0, False, False)
      DM._update_events(False, False, False, 0, soft_presence=True)
      alert_lvls.append(DM.alert_level)
    s = DM.settings
    orange_i = int((s._VISION_POLICY_ALERT_2_TIMEOUT + 0.6) / DT_DMON)
    assert 1 in alert_lvls[:orange_i]
    assert alert_lvls[orange_i] == 2

  def test_first_prompt_timeout_redraws_in_band_not_fixed_3s(self):
    """Each full reset draws a new first-prompt time in [2.0, 4.5], not always 3 s."""
    DM = DriverMonitoring()
    DM.nap_dm_hands_on_reset = True
    DM._rng.seed(7)
    seen = []
    for _ in range(40):
      DM._reset_awareness()
      t = DM.vision_alert_1_timeout
      assert VISION_ALERT_1_TIMEOUT_MIN <= t <= VISION_ALERT_1_TIMEOUT_MAX
      expected_t1 = 1. - t / DM.settings._VISION_POLICY_ALERT_3_TIMEOUT
      assert abs(DM.threshold_alert_1 - expected_t1) < 1e-9
      seen.append(round(t, 4))
    assert len(set(seen)) >= 8
    assert not all(abs(x - 3.0) < 1e-6 for x in seen)

  def test_first_prompt_onset_varies_across_cycles(self):
    """First green alert must not land at the same time every episode."""
    onsets = []
    for seed in range(16):
      DM = DriverMonitoring()
      DM.nap_dm_hands_on_reset = True
      DM._rng.seed(seed)
      DM._redraw_vision_alert_1()
      DM._apply_vision_alert_thresholds()
      first = None
      for i in range(int(8.0 / DT_DMON)):
        DM._update_states(msg_DISTRACTED, [0, 0, 0], 0, True, False)
        DM._update_events(False, True, False, 0)
        if DM.alert_level == 1:
          first = i * DT_DMON
          break
      assert first is not None
      onsets.append(first)
    assert max(onsets) - min(onsets) > 0.5
    assert all(1.5 < t < dm_settings._VISION_POLICY_ALERT_2_TIMEOUT for t in onsets)

  def test_run_step_hands_on_resets_near_alert_1(self):
    """run_step: Pre-AP handsOnLevel >= 1 resets at the first vision prompt."""
    DM = DriverMonitoring()
    DM.nap_dm_hands_on_reset = True
    steps = int((dm_settings._VISION_POLICY_ALERT_2_TIMEOUT + 1.0) / DT_DMON)
    for _ in range(steps):
      DM.run_step(_fake_sm(hands=1, driver_state=msg_DISTRACTED))
    assert DM.alert_level < 2
    assert DM.awareness > DM.threshold_alert_2

  def test_run_step_no_hands_still_escalates(self):
    DM = DriverMonitoring()
    DM.nap_dm_hands_on_reset = True
    steps = int((dm_settings._VISION_POLICY_ALERT_2_TIMEOUT + 0.8) / DT_DMON)
    for _ in range(steps):
      DM.run_step(_fake_sm(hands=0, driver_state=msg_DISTRACTED))
    assert DM.alert_level == 2

  def test_run_step_param_off_is_stock(self):
    DM = DriverMonitoring()
    DM.nap_dm_hands_on_reset = False
    DM._redraw_vision_alert_1()
    DM._apply_vision_alert_thresholds()
    assert DM.vision_alert_1_timeout == dm_settings._VISION_POLICY_ALERT_1_TIMEOUT
    steps = int((dm_settings._VISION_POLICY_ALERT_1_TIMEOUT + 0.4) / DT_DMON)
    for _ in range(steps):
      DM.run_step(_fake_sm(hands=1, driver_state=msg_DISTRACTED))
    assert DM.alert_level == 1
    for _ in range(8):
      DM._reset_awareness()
      assert DM.vision_alert_1_timeout == dm_settings._VISION_POLICY_ALERT_1_TIMEOUT
