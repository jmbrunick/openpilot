from pathlib import Path

from cereal import log
from opendbc.car.tesla.preap.carstate import HANDS_ON_DISENGAGE_LEVEL
from opendbc.car.tesla.values import STEER_THRESHOLD
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import (
  BlinkerLateralHold,
  blinker_pauses_lateral,
  lat_active_with_blinker_pause,
)
from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  BLEND_TIME_S,
  DISTURBANCE_CURVATURE_ERR,
  EMERGENCY_DECEL_FRAMES,
  EMERGENCY_DECEL_MPS2,
  EMERGENCY_MIN_V_EGO,
  HANDS_OFF_CONFIRM_S,
  HANDS_ON_HOLD_LEVEL,
  PARAM_DRIVER_LAT_HANDOFF,
  PREAP_FINGERPRINT,
  QUIET_WAIT_S,
  RATE_INTENT_MIN_DEG_S,
  RATE_SNA_ABS_DEG_S,
  SMOOTHSTEP_MAX_SLOPE,
  SOFT_YIELD_DEBOUNCE_FRAMES,
  SOFT_YIELD_RELEASE_NM,
  SOFT_YIELD_TRIGGER_NM,
  STEER_RATE_QUIET_DEG_S,
  TESLA_MAX_ANGLE_RATE_DEG_PER_20MS,
  UI_LATERAL_RETURN_AUTHORITY,
  YIELD_AUTHORITY_TIME_S,
  YIELD_EMERGENCY_WINDOW_S,
  DriverLateralHandoff,
  apply_lat_authority,
  cs_hands_on_level,
  cs_real_brake_pressed,
  emergency_brake,
  emergency_cancel_active,
  handoff_enabled,
  hands_still_on,
  handoff_new_desired_curvature,
  hud_engaged_status,
  is_disturbance,
  pin_desired_curvature_to_measured,
  smoothstep,
  torque_rate_aligned,
)


State = log.SelfdriveState.OpenpilotState
DT = 0.01


def _new():
  return DriverLateralHandoff(enabled=True)


def _step(h, *, torque=0.0, rate=0.0, engaged=True, lat=True, alc=False,
          blinker_paused=False, hands_on=0, tracking_error=0.0,
          brake=False, a_ego=0.0, v_ego=15.0, dt=DT):
  return h.update(
    engaged=engaged,
    lat_would_be_active=lat,
    steering_torque=torque,
    steering_rate_deg=rate,
    alc_active=alc,
    blinker_paused=blinker_paused,
    hands_on_level=hands_on,
    tracking_error=tracking_error,
    brake_applied=brake,
    a_ego=a_ego,
    v_ego=v_ego,
    dt=dt,
  )


def _yield(h, torque=0.85, rate=25.0, hands_on=1):
  out = None
  for _ in range(SOFT_YIELD_DEBOUNCE_FRAMES):
    out = _step(h, torque=torque, rate=rate, hands_on=hands_on)
  assert out is not None
  assert out.yielded
  assert out.authority == 0.0
  return out


def _hard_brake_frames(h, *, frames=EMERGENCY_DECEL_FRAMES, a_ego=None,
                       brake=True, v_ego=15.0, torque=0.15, rate=0.0,
                       hands_on=1):
  if a_ego is None:
    a_ego = EMERGENCY_DECEL_MPS2
  out = None
  for _ in range(frames):
    out = _step(h, torque=torque, rate=rate, hands_on=hands_on,
                brake=brake, a_ego=a_ego, v_ego=v_ego)
  return out


def _quiet(h, seconds):
  return _step(h, torque=0.0, rate=0.0, hands_on=0, dt=seconds)


def _hands_off(h, *, torque=0.0, rate=0.0):
  """Confirm handsOnLevel==0 for HANDS_OFF_CONFIRM_S — starts the 1 s blend."""
  return _step(h, torque=torque, rate=rate, hands_on=0, dt=HANDS_OFF_CONFIRM_S)


def test_thresholds_are_derived_from_real_steering_pressed():
  assert STEER_THRESHOLD == 1
  assert HANDS_ON_DISENGAGE_LEVEL == 2
  assert SOFT_YIELD_TRIGGER_NM == 0.70 * float(STEER_THRESHOLD)
  assert SOFT_YIELD_RELEASE_NM == 0.40 * float(STEER_THRESHOLD)
  assert SOFT_YIELD_RELEASE_NM < SOFT_YIELD_TRIGGER_NM
  assert SOFT_YIELD_DEBOUNCE_FRAMES == 14
  assert PREAP_FINGERPRINT == "TESLA_MODEL_S_PREAP"
  assert QUIET_WAIT_S == 0.0
  assert HANDS_ON_HOLD_LEVEL == 1
  assert HANDS_OFF_CONFIRM_S == 0.08
  assert HANDS_OFF_CONFIRM_S < 0.25
  assert BLEND_TIME_S == 1.0
  assert UI_LATERAL_RETURN_AUTHORITY == 0.70
  assert YIELD_AUTHORITY_TIME_S == 0.0
  assert TESLA_MAX_ANGLE_RATE_DEG_PER_20MS == 5.0
  assert PARAM_DRIVER_LAT_HANDOFF == "NAPDriverLatHandoff"
  assert STEER_RATE_QUIET_DEG_S == 25.0
  assert RATE_INTENT_MIN_DEG_S == 10.0
  assert RATE_SNA_ABS_DEG_S == 400.0
  assert DISTURBANCE_CURVATURE_ERR == 0.0025
  assert EMERGENCY_DECEL_MPS2 == -3.5
  assert EMERGENCY_DECEL_FRAMES == 8
  assert EMERGENCY_MIN_V_EGO == 1.0
  assert YIELD_EMERGENCY_WINDOW_S == 2.0
  assert torque_rate_aligned(0.70, 10.0)
  assert not torque_rate_aligned(0.70, -10.0)
  assert not torque_rate_aligned(0.70, 4095.0)
  assert not emergency_brake(
    brake_applied=True, a_ego=-1.0, v_ego=15.0, decel_frames=20)
  assert emergency_brake(
    brake_applied=True, a_ego=-3.5, v_ego=15.0,
    decel_frames=EMERGENCY_DECEL_FRAMES)
  assert not emergency_cancel_active(
    yielded=False, blending=False, yield_age_s=None, hard_brake=True)


def test_param_defaults_on_and_toggle_off_disables():
  """Default On; Settings Off must be identity (stock lat)."""
  h = DriverLateralHandoff()
  assert h.enabled
  assert handoff_enabled(fingerprint=PREAP_FINGERPRINT, param_on=True)
  assert not handoff_enabled(fingerprint=PREAP_FINGERPRINT, param_on=False)
  assert not handoff_enabled(fingerprint="TESLA_MODEL_3", param_on=True)
  root = Path(__file__).resolve().parents[4]
  keys = (root / "common/params_keys.h").read_text()
  line = next(ln for ln in keys.splitlines() if '"NAPDriverLatHandoff"' in ln)
  assert "BOOL" in line and '"1"' in line
  cs = (root / "selfdrive/controls/controlsd.py").read_text()
  assert "PARAM_DRIVER_LAT_HANDOFF" in cs
  h_off = DriverLateralHandoff(enabled=False)
  for _ in range(SOFT_YIELD_DEBOUNCE_FRAMES + 5):
    out = _step(h_off, torque=1.5, rate=80.0)
  assert out.authority == 1.0
  assert not out.yielded
  assert not out.ui_paused


def test_light_input_yields_lateral_keeps_long_and_lat_active():
  h = _new()
  enabled = True
  long_override = False
  op_long = True
  lat_active = True  # blinker helper already ran; handoff does not clear this
  out = _yield(h)
  long_active = enabled and not long_override and op_long
  assert lat_active
  assert long_active
  assert enabled
  assert out.yielded
  assert out.authority == 0.0
  assert out.ui_paused
  assert not out.emergency_cancel


def test_below_threshold_road_noise_does_not_yield():
  h = _new()
  for _ in range(200):
    out = _step(h, torque=0.4, rate=8.0)
  assert not out.yielded
  assert out.authority == 1.0
  assert not out.ui_paused


def test_just_below_trigger_does_not_yield():
  h = _new()
  for _ in range(100):
    out = _step(h, torque=SOFT_YIELD_TRIGGER_NM - 0.01)
  assert not out.yielded
  assert out.authority == 1.0


def test_short_spike_is_debounced():
  h = _new()
  for _ in range(SOFT_YIELD_DEBOUNCE_FRAMES - 1):
    out = _step(h, torque=1.2, rate=30.0, hands_on=1)
  assert not out.yielded
  out = _step(h, torque=0.0)
  assert not out.yielded
  assert out.authority == 1.0


def test_gravel_spike_train_does_not_yield():
  """Rumble: brief 1.2 Nm hits with gaps must not accumulate to a yield."""
  h = _new()
  out = None
  for _ in range(40):
    for _ in range(5):
      out = _step(h, torque=1.2)
    for _ in range(5):
      out = _step(h, torque=0.15)
  assert not out.yielded
  assert out.authority == 1.0


def test_old_half_nm_sustained_does_not_yield():
  """0.5–0.65 Nm (old rumble / crosswind-ish) must not enter yield."""
  h = _new()
  for _ in range(int(1.5 / DT)):
    out = _step(h, torque=0.65)
  assert not out.yielded
  assert out.authority == 1.0


def test_gentle_070_for_140ms_yields():
  """0.70 Nm is still the torsion floor; intent also needs rate + hands."""
  h = _new()
  out = None
  for _ in range(SOFT_YIELD_DEBOUNCE_FRAMES - 1):
    out = _step(h, torque=0.70, rate=12.0, hands_on=1)
    assert not out.yielded
  out = _step(h, torque=0.70, rate=12.0, hands_on=1)
  assert out.yielded
  assert out.authority == 0.0


def test_sustained_driver_input_keeps_authority_at_zero():
  h = _new()
  _yield(h)
  for _ in range(int(2.0 / DT)):
    out = _step(h, torque=1.0, rate=40.0)
    assert out.authority == 0.0
    assert out.yielded
    assert not out.blending


def test_yielded_hands_on_stays_yielded_through_torsion_dip():
  """Mid-dodge: handsOnLevel>=1 keeps yield even if torsion drops below release.

  QUIET_WAIT_S=0 used to start the 1 s blend on that dip and pull toward
  the lane / pothole.
  """
  h = _new()
  _yield(h)
  out = None
  for _ in range(int(1.0 / DT)):
    out = _step(h, torque=0.15, rate=40.0, hands_on=1)
    assert out.yielded
    assert not out.blending
    assert out.authority == 0.0
  assert out is not None
  assert hands_still_on(1)
  assert not hands_still_on(0)


def test_hands_off_confirm_starts_blend_promptly():
  """Hands 0 for ~80 ms starts the blend — not a 0.25 s torque quiet."""
  h = _new()
  _yield(h)
  out = _step(h, torque=0.35, hands_on=0, dt=HANDS_OFF_CONFIRM_S - 0.01)
  assert out.yielded
  assert not out.blending
  out = _step(h, torque=0.35, hands_on=0, dt=0.01)
  assert out.blending
  assert not out.yielded
  assert out.authority == 0.0


def test_hands_back_on_mid_blend_reyields():
  h = _new()
  _yield(h)
  out = _hands_off(h)
  assert out.blending
  for _ in range(40):  # 0.4 s into the 1 s blend
    out = _quiet(h, DT)
  assert out.blending
  assert 0.0 < out.authority < 1.0
  out = _step(h, torque=0.2, hands_on=1)
  assert out.yielded
  assert not out.blending
  assert out.authority == 0.0
  assert out.ui_paused
  out = _hands_off(h, torque=0.2)
  assert out.blending
  assert not out.yielded
  assert out.authority == 0.0


def test_yielded_rate_above_old_gate_does_not_block_resume():
  """Caster / road rate after hands-off must not keep the yielded latch."""
  h = _new()
  _yield(h)
  out = _hands_off(h, torque=0.05, rate=STEER_RATE_QUIET_DEG_S + 55.0)
  assert not out.yielded
  assert out.blending
  assert out.authority == 0.0


def test_yield_then_hands_off_authority_blends_to_one():
  """Yield → hands off ~80 ms → 1 s smoothstep to 1.0."""
  h = _new()
  _yield(h)
  out = _hands_off(h, rate=40.0)
  assert out.blending
  assert not out.yielded
  assert out.authority == 0.0
  for _ in range(int(BLEND_TIME_S / DT)):
    out = _step(h, torque=0.0, rate=40.0, hands_on=0)
  assert out.authority == 1.0
  assert not out.blending
  assert not out.yielded
  assert not out.ui_paused


def test_blend_is_smoothstep_monotonic_one_second_bounded_slope():
  h = _new()
  _yield(h)
  _hands_off(h)
  prev = 0.0
  max_delta = 0.0
  authorities = []
  for i in range(int(BLEND_TIME_S / DT)):
    out = _quiet(h, DT)
    assert out.authority + 1e-9 >= prev
    max_delta = max(max_delta, out.authority - prev)
    authorities.append(out.authority)
    prev = out.authority
    t = (i + 1) * DT / BLEND_TIME_S
    assert abs(out.authority - smoothstep(min(t, 1.0))) < 1e-9
  assert authorities[-1] == 1.0
  assert not out.blending
  assert not out.yielded
  assert max_delta <= SMOOTHSTEP_MAX_SLOPE * DT + 1e-9


def test_renewed_input_during_blend_yields_and_retries_after_hands_off():
  h = _new()
  _yield(h)
  out = _hands_off(h)
  assert out.blending
  for _ in range(40):  # 0.4 s into the 1 s blend
    out = _quiet(h, DT)
  assert out.blending
  assert 0.0 < out.authority < 1.0
  out = _step(h, torque=1.0)
  assert out.yielded
  assert not out.blending
  assert out.authority == 0.0
  assert out.ui_paused
  # Hands off again: blend after the short confirm, not 0.25 s
  out = _hands_off(h)
  assert out.blending
  assert not out.yielded
  assert out.authority == 0.0


def test_mid_band_during_blend_does_not_reyield():
  """0.55 Nm during the return is OP/caster, not a new firm push."""
  h = _new()
  _yield(h)
  out = _hands_off(h)
  assert out.blending
  for _ in range(20):
    out = _step(h, torque=0.55)
    assert out.blending
    assert not out.yielded
  out = _step(h, torque=SOFT_YIELD_TRIGGER_NM)
  assert out.yielded
  assert not out.blending


def test_ui_paused_below_70_returns_at_70_without_chatter():
  h = _new()
  _yield(h)
  _hands_off(h)
  seen_unpaused = False
  for _ in range(int(BLEND_TIME_S / DT)):
    out = _quiet(h, DT)
    if out.authority < UI_LATERAL_RETURN_AUTHORITY:
      assert out.ui_paused
      assert not seen_unpaused
    else:
      assert not out.ui_paused
      seen_unpaused = True
      break
  assert seen_unpaused
  # Knock authority back down during the remaining blend. UI must go
  # paused immediately and stay there — no green flicker.
  out = _step(h, torque=1.0)
  assert out.ui_paused
  assert out.authority == 0.0
  status = hud_engaged_status(
    enabled=True, op_state=State.enabled, lat_active=True, lat_handoff_paused=True)
  assert status == "override"
  status = hud_engaged_status(
    enabled=True, op_state=State.enabled, lat_active=True, lat_handoff_paused=False)
  assert status == "engaged"


def test_ui_does_not_claim_lat_back_at_69_percent():
  assert hud_engaged_status(
    enabled=True, op_state=State.enabled, lat_active=True, lat_handoff_paused=True,
  ) == "override"
  # 69% is still paused (latched threshold 70%)
  h = _new()
  _yield(h)
  _hands_off(h)
  out = None
  while True:
    out = _quiet(h, DT)
    if out.authority >= 0.69:
      break
  # walk until just below 0.70
  while out.authority < UI_LATERAL_RETURN_AUTHORITY - 1e-9:
    assert out.ui_paused
    prev = out.authority
    out = _quiet(h, DT)
    if out.authority == prev:
      break
  if out.authority < UI_LATERAL_RETURN_AUTHORITY:
    assert out.ui_paused


def test_hud_gas_override_still_green_when_lat_active_and_not_handoff():
  assert hud_engaged_status(
    enabled=True, op_state=State.overriding, lat_active=True, lat_handoff_paused=False,
  ) == "engaged"
  assert hud_engaged_status(
    enabled=True, op_state=State.overriding, lat_active=True, lat_handoff_paused=True,
  ) == "override"
  assert hud_engaged_status(
    enabled=False, op_state=State.disabled, lat_active=False, lat_handoff_paused=False,
  ) == "disengaged"


def test_blinker_and_alc_paths_unchanged_and_do_not_arm_from_handoff():
  assert blinker_pauses_lateral(True, False)
  hold = BlinkerLateralHold()
  lat = lat_active_with_blinker_pause(
    active=True, steer_fault_temporary=False, steer_fault_permanent=False,
    standstill=False, steer_at_standstill=False,
    left_blinker=True, right_blinker=False, hold=hold, engaged=True,
  )
  assert not lat
  h = _new()
  out = _step(h, torque=0.8, lat=False, blinker_paused=True)
  assert out.authority == 1.0
  assert not out.yielded
  assert not out.blending

  h = _new()
  for _ in range(SOFT_YIELD_DEBOUNCE_FRAMES + 1):
    out = _step(h, torque=0.8, alc=True)
  assert not out.yielded
  assert out.authority == 1.0


def test_blinker_pause_gates_handoff_resume_blends_immediately():
  """Post-turn grab: blinker resume used to restore authority=1 onto model.

  Soft-yield stays gated during the pause. Rising edge starts the 1 s
  blend this frame (QUIET_WAIT_S == 0) — no yank, no extra quiet delay.
  Standstill-style lat-down without blinker_paused does not blend.
  """
  h = _new()
  for _ in range(int(1.5 / DT)):
    out = _step(h, torque=0.2, lat=False, blinker_paused=True)
    assert out.authority == 1.0
    assert not out.yielded
    assert not out.blending
  out = _step(h, torque=0.2, lat=True, blinker_paused=False)
  assert out.blending
  assert not out.yielded
  assert out.authority == 0.0
  assert out.ui_paused
  for _ in range(int(BLEND_TIME_S / DT)):
    out = _step(h, torque=0.0)
  assert out.authority == 1.0
  assert not out.blending
  assert not out.ui_paused

  h = _new()
  for _ in range(20):
    _step(h, torque=0.0, lat=False, blinker_paused=False)
  out = _step(h, torque=0.0, lat=True)
  assert not out.blending
  assert out.authority == 1.0


def test_engage_from_disabled_does_not_start_blinker_reentry_blend():
  h = _new()
  _step(h, engaged=False, lat=False)
  out = _step(h, engaged=True, lat=True)
  assert not out.blending
  assert out.authority == 1.0


def test_stalk_cancel_or_fault_resets_to_full_disengage_contract():
  h = _new()
  _yield(h)
  out = _step(h, torque=0.8, engaged=False)
  assert out.authority == 1.0
  assert not out.yielded
  assert not out.ui_paused
  # enabled session is gone; HUD is disengaged, not a lat-pause
  assert hud_engaged_status(
    enabled=False, op_state=State.disabled, lat_active=False, lat_handoff_paused=False,
  ) == "disengaged"


def test_apply_lat_authority_follows_driver_at_zero_and_nap_at_one():
  torque, angle, curv = apply_lat_authority(0.0, 0.4, 12.0, 3.0, 0.02, 0.001)
  assert torque == 0.0
  assert angle == 3.0
  assert curv == 0.001
  torque, angle, curv = apply_lat_authority(1.0, 0.4, 12.0, 3.0, 0.02, 0.001)
  assert torque == 0.4
  assert angle == 12.0
  assert curv == 0.02
  torque, angle, curv = apply_lat_authority(0.5, 0.4, 10.0, 0.0, 0.02, 0.0)
  assert abs(torque - 0.2) < 1e-9
  assert abs(angle - 5.0) < 1e-9


def _angle_from_curv(curv):
  """Linear stand-in for VM.get_steer_from_curvature (sign-preserving)."""
  return float(curv) * 800.0


def _clip_toward(prev, target, step=0.0002):
  """Stand-in for clip_curvature (jerk-limited slew from the pin)."""
  if target > prev:
    return min(prev + step, target)
  return max(prev - step, target)


def _step_actuators(h, *, torque, desired, model_curv, meas_curv, meas_angle,
                    v_ego=20.0, hands_on=0, rate=0.0, dt=DT):
  """Mirror controlsd: pin while yielded; clip + optional blend otherwise."""
  _ = v_ego
  out = _step(h, torque=torque, rate=rate, hands_on=hands_on, dt=dt)
  new_desired = handoff_new_desired_curvature(
    yielded=out.yielded, lat_active=True,
    model_curvature=model_curv, measured_curvature=meas_curv,
  )
  if pin_desired_curvature_to_measured(out.yielded, lat_active=True):
    desired = meas_curv
  else:
    desired = _clip_toward(desired, new_desired)
  lac_angle = _angle_from_curv(desired)
  if out.authority < 1.0:
    _t, angle, curv = apply_lat_authority(
      out.authority, 0.4, lac_angle, meas_angle, desired, meas_curv)
  else:
    angle, curv = lac_angle, desired
  return out, desired, angle, curv


def test_yield_pins_planner_resume_tracks_model_not_measured():
  """Justin: green + path-back, wheel firm/holding (not path-tracking).

  latActive stays true on purpose (no VM snap). The old path still fed
  the model into clip_curvature during yield, so desired ran to the lane
  while apply_lat_authority(0) commanded measured. After authority→1 the
  raw LaC angle was already at the model; VM/EPAS stayed near measured.

  Pin desired to the wheel while yielded. Blend/resume clip from that pin.
  At authority=1 the command equals LaC(desired), which has been slewing
  from measured toward the model — not stuck at yield-time measured.
  """
  h = _new()
  model_curv = 0.012
  meas_curv = 0.0
  meas_angle = _angle_from_curv(meas_curv)
  desired = model_curv  # was tracking the path before the push

  assert handoff_new_desired_curvature(
    yielded=False, lat_active=True,
    model_curvature=model_curv, measured_curvature=meas_curv,
  ) == model_curv
  assert handoff_new_desired_curvature(
    yielded=True, lat_active=True,
    model_curvature=model_curv, measured_curvature=meas_curv,
  ) == meas_curv
  assert pin_desired_curvature_to_measured(True)
  assert not pin_desired_curvature_to_measured(False)
  assert pin_desired_curvature_to_measured(False, lat_active=False)
  assert not pin_desired_curvature_to_measured(False, lat_active=True)

  for _ in range(SOFT_YIELD_DEBOUNCE_FRAMES):
    out, desired, angle, curv = _step_actuators(
      h, torque=1.0, rate=25.0, hands_on=1, desired=desired,
      model_curv=model_curv, meas_curv=meas_curv, meas_angle=meas_angle)
  assert out.yielded
  assert desired == meas_curv
  assert angle == meas_angle
  assert curv == meas_curv

  # Hands still on: pin stays even if torsion dipped.
  out, desired, angle, curv = _step_actuators(
    h, torque=0.1, desired=desired, model_curv=model_curv,
    meas_curv=meas_curv, meas_angle=meas_angle, hands_on=1)
  assert out.yielded
  assert desired == meas_curv
  assert angle == meas_angle

  # Hands off for the short confirm: blend starts, pin lifts.
  out, desired, angle, curv = _step_actuators(
    h, torque=0.0, desired=desired, model_curv=model_curv,
    meas_curv=meas_curv, meas_angle=meas_angle, dt=HANDS_OFF_CONFIRM_S)
  assert out.authority == 0.0
  assert out.blending
  assert not out.yielded
  assert not pin_desired_curvature_to_measured(out.yielded)
  assert handoff_new_desired_curvature(
    yielded=out.yielded, lat_active=True,
    model_curvature=model_curv, measured_curvature=meas_curv,
  ) == model_curv

  for _ in range(int(BLEND_TIME_S / DT)):
    out, desired, angle, curv = _step_actuators(
      h, torque=0.0, desired=desired, model_curv=model_curv,
      meas_curv=meas_curv, meas_angle=meas_angle)

  assert out.authority == 1.0
  assert not out.blending
  assert not out.yielded
  assert not out.ui_paused
  # identity on the planner/LaC target — not the yield-time measured hold
  assert abs(curv - desired) < 1e-12
  assert abs(angle - _angle_from_curv(desired)) < 1e-12
  assert abs(desired - meas_curv) > 1e-4
  assert abs(desired - model_curv) < abs(meas_curv - model_curv)
  assert abs(angle - meas_angle) > 1e-3
  # controlsd must pin + skip blend at 100%
  cs = (Path(__file__).resolve().parents[4] / "selfdrive/controls/controlsd.py").read_text()
  assert "pin_desired_curvature_to_measured" in cs
  assert "handoff_new_desired_curvature" in cs
  assert "authority < 1.0" in cs
  assert "CC.latActive" in cs
  assert "cs_hands_on_level" in cs
  assert "hands_on_level" in cs
  assert "cs_real_brake_pressed" in cs
  assert "tracking_error" in cs
  assert "emergency_cancel" in cs


def test_unyielded_lat_inactive_still_uses_measured_curvature():
  """Blinker / standstill path: latActive false → measured, snap-pin."""
  assert handoff_new_desired_curvature(
    yielded=False, lat_active=False,
    model_curvature=0.02, measured_curvature=0.001,
  ) == 0.001
  assert pin_desired_curvature_to_measured(False, lat_active=False)


def test_blinker_resume_pins_then_blends_from_wheel_not_model():
  """Lot turn: model points at grass; resume must not command that instantly."""
  h = _new()
  model_curv = 0.02  # "into the grass"
  meas_curv = 0.001
  meas_angle = _angle_from_curv(meas_curv)
  desired = 0.015  # lagged planner from the turn

  for _ in range(10):
    out = _step(h, torque=0.2, lat=False, blinker_paused=True)
    assert not out.yielded
    assert pin_desired_curvature_to_measured(out.yielded, lat_active=False)
    desired = meas_curv  # controlsd snap-pin while lat down

  out = _step(h, torque=0.0, lat=True, blinker_paused=False)
  assert out.blending
  assert out.authority == 0.0
  assert not pin_desired_curvature_to_measured(out.yielded, lat_active=True)
  new_desired = handoff_new_desired_curvature(
    yielded=out.yielded, lat_active=True,
    model_curvature=model_curv, measured_curvature=meas_curv,
  )
  assert new_desired == model_curv
  desired = _clip_toward(desired, new_desired)
  _t, angle, curv = apply_lat_authority(
    out.authority, 0.4, _angle_from_curv(desired), meas_angle, desired, meas_curv)
  # authority 0: command the wheel, not the grass-pointing model
  assert angle == meas_angle
  assert curv == meas_curv
  assert abs(desired - meas_curv) < abs(model_curv - meas_curv)
  cs = (Path(__file__).resolve().parents[4] / "selfdrive/controls/controlsd.py").read_text()
  assert "blinker_paused" in cs
  assert "blinker_lat_hold.holding" in cs


def test_disabled_for_non_preap_is_identity():
  h = DriverLateralHandoff(enabled=False)
  for _ in range(50):
    out = _step(h, torque=1.5)
  assert out.authority == 1.0
  assert not out.ui_paused


def test_blinker_resume_with_hands_on_yields_instead_of_blend():
  """Rising edge while still on the rim must not pull toward the model."""
  h = _new()
  for _ in range(10):
    _step(h, torque=0.2, lat=False, blinker_paused=True, hands_on=1)
  out = _step(h, torque=0.2, lat=True, blinker_paused=False, hands_on=1)
  assert out.yielded
  assert not out.blending
  assert out.authority == 0.0
  out = _hands_off(h, torque=0.2)
  assert out.blending
  assert not out.yielded


def test_cs_hands_on_level_reads_cereal_and_eps_stash():
  class _CS:
    pass

  cs = _CS()
  cs.handsOnLevel = 1
  cs.steeringTorqueEps = 0.0
  cs.steeringDisengage = False
  assert cs_hands_on_level(cs) == 1
  cs = _CS()
  cs.steeringTorqueEps = 1.0
  cs.steeringDisengage = False
  assert cs_hands_on_level(cs) == 1
  cs = _CS()
  cs.steeringDisengage = True
  assert cs_hands_on_level(cs) == 2
  cs = _CS()
  assert cs_hands_on_level(cs) == 0
  pause = (Path(__file__).resolve().parents[4] /
           "selfdrive/car/tesla/preap_blinker_lat_pause.py").read_text()
  assert "_publish_hands_on_level" in pause
  assert "steeringTorqueEps" in pause


def test_hysteresis_band_does_not_retrigger_from_texture_after_release():
  h = _new()
  _yield(h)
  _hands_off(h)
  for _ in range(int(BLEND_TIME_S / DT)):
    _quiet(h, DT)
  # back at full authority; 0.4 Nm is between release and trigger
  for _ in range(100):
    out = _step(h, torque=0.4)
  assert not out.yielded
  assert out.authority == 1.0


def test_intentional_sustained_torsion_rate_hands_yields():
  h = _new()
  out = _yield(h, torque=0.80, rate=18.0, hands_on=1)
  assert out.yielded
  assert out.authority == 0.0
  assert out.ui_paused
  assert not out.emergency_cancel


def test_gravel_like_spikes_do_not_yield_even_with_hands():
  """Rumble: brief 1.2 Nm hits, even with hands + rate, must not accumulate."""
  h = _new()
  out = None
  for _ in range(40):
    for _ in range(5):
      out = _step(h, torque=1.2, rate=40.0, hands_on=1)
    for _ in range(5):
      out = _step(h, torque=0.15, rate=4.0, hands_on=1)
  assert not out.yielded
  assert out.authority == 1.0


def test_sustained_torsion_without_rate_does_not_yield():
  """Crown / resting pressure: torque without a matching turn."""
  h = _new()
  for _ in range(int(1.0 / DT)):
    out = _step(h, torque=0.90, rate=0.0, hands_on=1)
  assert not out.yielded
  assert out.authority == 1.0


def test_sustained_torsion_without_hands_does_not_yield():
  h = _new()
  for _ in range(int(1.0 / DT)):
    out = _step(h, torque=0.90, rate=20.0, hands_on=0)
  assert not out.yielded
  assert out.authority == 1.0


def test_rate_opposite_torsion_does_not_yield():
  h = _new()
  for _ in range(int(1.0 / DT)):
    out = _step(h, torque=0.90, rate=-20.0, hands_on=1)
  assert not out.yielded
  assert out.authority == 1.0


def test_sna_rate_does_not_count_as_intent():
  h = _new()
  for _ in range(int(0.5 / DT)):
    out = _step(h, torque=1.0, rate=4095.0, hands_on=1)
  assert not out.yielded
  assert not torque_rate_aligned(1.0, 4095.0)


def test_wind_disturbance_high_effort_low_torsion_does_not_yield():
  """Controller fighting wind: high tracking error, no matching torsion."""
  h = _new()
  assert is_disturbance(
    torque_nm=0.2, rate_deg=4.0, tracking_error=DISTURBANCE_CURVATURE_ERR)
  for _ in range(int(1.5 / DT)):
    out = _step(h, torque=0.25, rate=6.0, hands_on=1,
                tracking_error=0.01)
  assert not out.yielded
  assert out.authority == 1.0


def test_matching_intent_is_not_blocked_by_tracking_error():
  """A real dodge leaves the path — high error with matching torsion is OK."""
  h = _new()
  assert not is_disturbance(
    torque_nm=0.80, rate_deg=20.0, tracking_error=0.02)
  out = _yield(h, torque=0.80, rate=20.0, hands_on=1)
  assert out.yielded
  out = _step(h, torque=0.80, rate=20.0, hands_on=1, tracking_error=0.02)
  assert out.yielded


def test_hands_on_hold_and_hands_off_blend_still_work_after_intent_yield():
  h = _new()
  _yield(h)
  for _ in range(int(0.6 / DT)):
    out = _step(h, torque=0.10, rate=30.0, hands_on=1)
    assert out.yielded
    assert not out.blending
    assert out.authority == 0.0
  out = _hands_off(h, torque=0.10, rate=30.0)
  assert out.blending
  assert not out.yielded
  for _ in range(int(BLEND_TIME_S / DT)):
    out = _step(h, torque=0.0, rate=0.0, hands_on=0)
  assert out.authority == 1.0
  assert not out.blending
  assert not out.ui_paused


def test_emergency_hard_brake_while_yielded_sets_cancel():
  h = _new()
  _yield(h)
  out = _hard_brake_frames(h)
  assert out.yielded
  assert out.emergency_cancel
  pause = (Path(__file__).resolve().parents[4] /
           "selfdrive/car/tesla/preap_blinker_lat_pause.py").read_text()
  assert "hard_cancel_session" in pause
  assert "update_card_lat_handoff" in pause
  assert "_publish_real_brake" in pause
  assert "emergency_cancel" in pause
  src = (Path(__file__).resolve().parents[4] /
         "selfdrive/controls/lib/driver_lateral_handoff.py").read_text()
  assert "_drop_longitudinal_keep_lateral" not in src
  assert "_nap_long_resume_pending" not in src


def test_emergency_hard_brake_in_yield_window_after_blend_sets_cancel():
  h = _new()
  _yield(h)
  _hands_off(h)
  for _ in range(int(BLEND_TIME_S / DT)):
    _quiet(h, DT)
  # Just after resume, still inside the 2 s yield-entry window.
  out = _hard_brake_frames(h, hands_on=0, torque=0.0)
  assert not out.yielded
  assert out.authority == 1.0
  assert out.emergency_cancel


def test_hard_brake_after_window_does_not_cancel():
  h = _new()
  _yield(h)
  _hands_off(h)
  for _ in range(int(BLEND_TIME_S / DT)):
    _quiet(h, DT)
  leftover = YIELD_EMERGENCY_WINDOW_S - BLEND_TIME_S - HANDS_OFF_CONFIRM_S
  for _ in range(int((leftover + 0.15) / DT)):
    out = _quiet(h, DT)
  assert not out.emergency_cancel
  out = _hard_brake_frames(h, hands_on=0, torque=0.0)
  assert not out.emergency_cancel


def test_light_brake_while_yielded_does_not_force_full_cancel():
  """Digital Applied + mild aEgo is the sticky-MAX silent pause, not this."""
  h = _new()
  _yield(h)
  out = None
  for _ in range(int(0.4 / DT)):
    out = _step(h, torque=0.15, hands_on=1, brake=True, a_ego=-1.2, v_ego=15.0)
  assert out.yielded
  assert not out.emergency_cancel


def test_light_brake_when_not_yielded_does_not_cancel():
  h = _new()
  out = None
  for _ in range(int(0.4 / DT)):
    out = _step(h, torque=0.2, hands_on=1, brake=True, a_ego=-1.0, v_ego=15.0)
  assert not out.yielded
  assert not out.emergency_cancel


def test_hard_decel_without_brake_does_not_cancel():
  """Pothole / KF spike: aEgo alone is not emergency."""
  h = _new()
  _yield(h)
  out = _hard_brake_frames(h, brake=False, a_ego=-6.0)
  assert out.yielded
  assert not out.emergency_cancel


def test_cs_real_brake_pressed_reads_brake_stash():
  class _CS:
    pass

  cs = _CS()
  cs.brake = 1.0
  cs.brakePressed = False
  assert cs_real_brake_pressed(cs)
  cs.brake = 0.0
  assert not cs_real_brake_pressed(cs)
  cs.realBrakePressed = True
  assert cs_real_brake_pressed(cs)
  cs = (Path(__file__).resolve().parents[4] / "selfdrive/controls/controlsd.py").read_text()
  assert "cs_real_brake_pressed" in cs
  assert "CC.cruiseControl.cancel" in cs
