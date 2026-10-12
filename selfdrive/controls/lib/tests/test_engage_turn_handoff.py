"""Engage latch, turn-vs-fight, resting torque, and a single pull taking lateral back."""

from pathlib import Path
from types import SimpleNamespace

from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.values import CruiseButtons

from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import (
  ORPHAN_SESSION_S,
  install_blinker_lat_pause,
  reconcile_orphan_session,
)
from openpilot.selfdrive.controls.lib.desire_helper import _driver_torque_nm
from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  LAT_REENABLE_MIN_V_EGO,
  RELEASE_HOLD_S,
  DriverLateralHandoff,
)
from openpilot.selfdrive.controls.lib.lane_change_nudge import TippedLaneChangeTorque
from openpilot.selfdrive.selfdrived.preap_regen import orphan_pull_requests_enable

DT = 0.01
REST_NM = 0.25
# Above 10 mph. The inhibit is strict less-than.
CRUISE_V = LAT_REENABLE_MIN_V_EGO + 5.0
SLOW_V = LAT_REENABLE_MIN_V_EGO - 0.5
MODEL_K = 0.01
WHEEL_K = 0.0


def _step(h, *, torque=0.0, hands=0, dt=DT, v_ego=CRUISE_V, blinker=False,
          engaged=True, lat=True, pressed=False, angle=0.0, rate=0.0,
          model_k=MODEL_K, wheel_k=WHEEL_K):
  return h.update(
    engaged=engaged, lat_would_be_active=lat,
    steering_torque=torque, steering_rate_deg=rate,
    hands_on_level=hands, dt=dt, v_ego=v_ego,
    blinker_paused=blinker, steering_pressed=pressed,
    steering_angle_deg=angle,
    model_curvature=model_k, measured_curvature=wheel_k,
  )


def _run(h, seconds, **kw):
  n = max(1, int(round(seconds / DT)))
  out = None
  for _ in range(n):
    out = _step(h, **kw)
  return out


def _yield_once(h, torque=1.5):
  out = None
  for _ in range(12):
    out = _step(h, torque=torque, hands=1, pressed=True)
  assert out.yielded and out.authority == 0.0
  return out


def test_resting_offset_returns_lateral_after_a_held_turn():
  """Logged pattern: hands-off sits near +0.25 Nm. The free-wheel offset learns it."""
  h = DriverLateralHandoff()
  _run(h, 12.0, torque=REST_NM, hands=0, engaged=False, lat=False, angle=0.0, rate=-4095.5)
  assert 0.20 < h.rest_bias < 0.28
  assert 0.20 < h.rest_bias_free < 0.28

  turn = REST_NM + 2.0
  out = _run(h, 6.0, torque=turn, hands=1, pressed=True, blinker=True, engaged=True, lat=True)
  assert out.yielded

  out = None
  for _ in range(int((RELEASE_HOLD_S + 0.05) / DT)):
    out = _step(h, torque=REST_NM, hands=0, blinker=False, pressed=False)
    if out.blending:
      break
  assert out is not None and out.blending and not out.yielded


def test_blinker_and_low_speed_hold_the_yield():
  h = DriverLateralHandoff()
  push = 1.5
  _yield_once(h, push)
  out = _run(h, 1.0, torque=0.0, hands=0, blinker=True)
  assert out.yielded and not out.blending

  h2 = DriverLateralHandoff()
  _yield_once(h2, push)
  out = _run(h2, 1.0, torque=0.0, hands=0, v_ego=SLOW_V, blinker=False)
  assert out.yielded and not out.blending
  # Crossing 10 mph lands in yield; a quiet wheel then returns.
  out = _run(h2, RELEASE_HOLD_S + 0.05, torque=0.0, hands=0, v_ego=CRUISE_V, blinker=False)
  assert out.blending and not out.yielded


def test_light_hold_returns_without_curvature_agreement():
  h = DriverLateralHandoff()
  _yield_once(h, 1.5)
  early = _run(h, 2.0, torque=0.5, hands=0, model_k=MODEL_K, wheel_k=WHEEL_K)
  assert early.yielded and not early.blending
  out = _run(h, 0.6, torque=0.5, hands=0, model_k=MODEL_K, wheel_k=WHEEL_K)
  assert out.blending and not out.yielded


def test_single_pull_forces_the_return():
  h = DriverLateralHandoff()
  _yield_once(h, 1.5)
  _run(h, 0.5, torque=1.0, hands=1, model_k=MODEL_K)
  assert h._yielded
  h._soften = True
  h.driver_resume_request()
  assert not h._soften
  assert h._pull_pending and h._yielded and not h._blending
  out = _step(h, torque=1.0, hands=1, model_k=MODEL_K)
  assert out.blending and not out.yielded
  assert not h._pull_pending


def test_learned_offset_is_not_a_left_lane_nudge():
  nudge = TippedLaneChangeTorque()
  nudge.update(torque_nm=0.0, hands_on_level=0, direction=1, tipped=True, dt=DT)
  assert not nudge.confirm
  nudge.update(torque_nm=0.70, hands_on_level=1, direction=1, tipped=True, dt=DT)
  assert nudge.confirm

  resting = SimpleNamespace(steeringTorque=REST_NM, napRestTorqueNm=REST_NM, steeringPressed=False)
  assert abs(_driver_torque_nm(resting)) < 1e-9
  missing = SimpleNamespace(steeringTorque=0.40, steeringPressed=False)
  assert abs(_driver_torque_nm(missing) - 0.40) < 1e-9

  controls = Path("selfdrive/controls/controlsd.py").read_text()
  assert "self.lat_handoff.rest_bias" in controls
  assert "napStalkSeq" in controls
  sub = controls.split("messaging.SubMaster(", 1)[1].split(")", 1)[0]
  assert sub.count("'carState'") == 1
  assert "sub_sock" not in controls
  desire = Path("selfdrive/controls/lib/desire_helper.py").read_text()
  assert "napRestTorqueNm" in desire


def test_orphan_pull_is_pedal_single_and_no_pedal_only_with_long():
  assert orphan_pull_requests_enable(
    cruise_enabled=True, op_enabled=False, set_pressed=True, use_pedal=True, long_on=False)
  assert not orphan_pull_requests_enable(
    cruise_enabled=True, op_enabled=True, set_pressed=True, use_pedal=True, long_on=True)
  assert not orphan_pull_requests_enable(
    cruise_enabled=False, op_enabled=False, set_pressed=True, use_pedal=True, long_on=False)
  # No-pedal: the first pull is lateral-only. The completing pull has long on.
  assert not orphan_pull_requests_enable(
    cruise_enabled=True, op_enabled=False, set_pressed=True, use_pedal=False, long_on=False)
  assert orphan_pull_requests_enable(
    cruise_enabled=True, op_enabled=False, set_pressed=True, use_pedal=False, long_on=True)


def _buttons(eng, *, cruise_buttons=0, prev=0, t_ms=1000, use_pedal=True):
  return eng.process_buttons(
    cruise_buttons=cruise_buttons, prev_cruise_buttons=prev,
    curr_time_ms=t_ms, v_ego=15.0, speed_units="KPH",
    use_pedal=use_pedal, pedal_long_allowed=True,
    long_control_allowed=True, real_brake_pressed=False)


def test_orphan_session_resets_quietly_then_the_next_pull_engages():
  install_blinker_lat_pause()
  eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)
  eng.cruiseEnabled = True
  eng.enableLongControl = True
  eng.longCtrlEvent = "pccEnabled"
  cereal = SimpleNamespace(
    cruiseState=SimpleNamespace(enabled=True), enableLongControl=True)
  iface = SimpleNamespace(
    cruiseEnabled=True, enableLongControl=True, enableJustCC=False,
    pedal_speed_kph=10.0, preap_cc_cancel_needed=False, cruise_enabled_prev=True)

  timer = 0.0
  canceled = False
  frames = int(round((ORPHAN_SESSION_S - DT) / DT))
  for _ in range(frames):
    timer, canceled = reconcile_orphan_session(
      eng, op_enabled=False, orphan_s=timer, dt=DT,
      cereal_cs=cereal, interface_cs=iface)
    assert not canceled
  assert eng.cruiseEnabled
  assert timer + 1e-9 < ORPHAN_SESSION_S

  # Openpilot taking the session clears the timer and leaves cruise up.
  timer, canceled = reconcile_orphan_session(
    eng, op_enabled=True, orphan_s=timer, dt=DT, cereal_cs=cereal, interface_cs=iface)
  assert timer == 0.0 and not canceled and eng.cruiseEnabled

  timer = 0.0
  for _ in range(frames + 2):
    timer, canceled = reconcile_orphan_session(
      eng, op_enabled=False, orphan_s=timer, dt=DT,
      cereal_cs=cereal, interface_cs=iface)
    if canceled:
      break
  assert canceled
  assert not eng.cruiseEnabled
  assert not eng.enableLongControl
  assert eng.preap_cc_cancel_needed
  assert eng.longCtrlEvent != "pccDisabled"
  assert cereal.cruiseState.enabled is False
  assert iface.preap_cc_cancel_needed
  assert iface.cruise_enabled_prev is False

  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=5000)
  assert eng.cruiseEnabled and eng.enableLongControl


def test_in_session_pull_bumps_stalk_seq_and_a_fresh_engage_does_not():
  install_blinker_lat_pause()
  eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=1000)
  assert eng.cruiseEnabled
  assert int(getattr(eng, "_nap_stalk_seq", 0)) == 0

  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=0, t_ms=2000)
  assert int(eng._nap_stalk_seq) == 1
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=0, t_ms=3000)
  assert int(eng._nap_stalk_seq) == 2
