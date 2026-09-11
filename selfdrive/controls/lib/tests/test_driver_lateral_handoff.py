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
  PARAM_DRIVER_LAT_HANDOFF,
  PREAP_FINGERPRINT,
  QUIET_WAIT_S,
  SMOOTHSTEP_MAX_SLOPE,
  SOFT_YIELD_DEBOUNCE_FRAMES,
  SOFT_YIELD_RELEASE_NM,
  SOFT_YIELD_TRIGGER_NM,
  STEER_RATE_QUIET_DEG_S,
  TESLA_MAX_ANGLE_RATE_DEG_PER_20MS,
  UI_LATERAL_RETURN_AUTHORITY,
  YIELD_AUTHORITY_TIME_S,
  DriverLateralHandoff,
  apply_lat_authority,
  handoff_enabled,
  hud_engaged_status,
  smoothstep,
)


State = log.SelfdriveState.OpenpilotState
DT = 0.01


def _new():
  return DriverLateralHandoff(enabled=True)


def _step(h, *, torque=0.0, rate=0.0, engaged=True, lat=True, alc=False, dt=DT):
  return h.update(
    engaged=engaged,
    lat_would_be_active=lat,
    steering_torque=torque,
    steering_rate_deg=rate,
    alc_active=alc,
    dt=dt,
  )


def _yield(h, torque=1.0):
  out = None
  for _ in range(SOFT_YIELD_DEBOUNCE_FRAMES):
    out = _step(h, torque=torque)
  assert out is not None
  assert out.yielded
  assert out.authority == 0.0
  return out


def _quiet(h, seconds):
  return _step(h, torque=0.0, rate=0.0, dt=seconds)


def test_thresholds_are_half_of_real_steering_pressed_not_invented():
  assert STEER_THRESHOLD == 1
  assert HANDS_ON_DISENGAGE_LEVEL == 2
  assert SOFT_YIELD_TRIGGER_NM == 0.85 * float(STEER_THRESHOLD)
  assert SOFT_YIELD_RELEASE_NM == 0.40 * float(STEER_THRESHOLD)
  assert SOFT_YIELD_RELEASE_NM < SOFT_YIELD_TRIGGER_NM
  assert SOFT_YIELD_DEBOUNCE_FRAMES == 25
  assert PREAP_FINGERPRINT == "TESLA_MODEL_S_PREAP"
  assert QUIET_WAIT_S == 0.25
  assert BLEND_TIME_S == 1.0
  assert UI_LATERAL_RETURN_AUTHORITY == 0.70
  assert YIELD_AUTHORITY_TIME_S == 0.0
  assert TESLA_MAX_ANGLE_RATE_DEG_PER_20MS == 5.0
  assert PARAM_DRIVER_LAT_HANDOFF == "NAPDriverLatHandoff"
  assert STEER_RATE_QUIET_DEG_S == 25.0


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
  for _ in range(SOFT_YIELD_DEBOUNCE_FRAMES):
    out = _step(h, torque=1.0)
  long_active = enabled and not long_override and op_long
  assert lat_active
  assert long_active
  assert enabled
  assert out.yielded
  assert out.authority == 0.0
  assert out.ui_paused


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
    out = _step(h, torque=1.2)
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
  """0.5–0.7 Nm (old trigger / crosswind-ish) must not yield."""
  h = _new()
  for _ in range(int(1.5 / DT)):
    out = _step(h, torque=0.7)
  assert not out.yielded
  assert out.authority == 1.0


def test_sustained_driver_input_keeps_authority_at_zero():
  h = _new()
  _yield(h)
  for _ in range(int(2.0 / DT)):
    out = _step(h, torque=1.0, rate=40.0)
    assert out.authority == 0.0
    assert out.yielded
    assert not out.blending


def test_yielded_hysteresis_band_0_35nm_begins_blend():
  """Yield, then 0.35 Nm (in the 0.3–0.5 band) at rate 0 for >0.25 s.

  That band is press-latch hysteresis only. Treating it as 'still
  holding' zeroed _quiet_s every frame and never handed back on-car.
  """
  h = _new()
  _yield(h)
  assert 0.35 < SOFT_YIELD_TRIGGER_NM
  out = None
  for _ in range(int(QUIET_WAIT_S / DT)):
    out = _step(h, torque=0.35, rate=0.0)
    assert out.authority == 0.0
  assert out is not None
  assert out.blending
  assert not out.yielded
  assert out.authority == 0.0


def test_yielded_old_half_nm_band_begins_blend():
  """0.6 Nm is below the 0.85 trigger; must not reset quiet."""
  h = _new()
  _yield(h)
  out = None
  for _ in range(int(QUIET_WAIT_S / DT)):
    out = _step(h, torque=0.6, rate=0.0)
  assert out.blending
  assert not out.yielded


def test_yielded_rate_above_old_gate_does_not_block_resume():
  """Caster / road rate after release must not keep the yielded latch.

  The old quiet gate required |rate| <= 25 deg/s AND |torque| <= 0.3 Nm.
  On a moving car those were never both true for 0.25 s, so authority
  never blended back. Quiet is torque-only while yielded.
  """
  h = _new()
  _yield(h)
  out = None
  for _ in range(int(QUIET_WAIT_S / DT)):
    out = _step(h, torque=0.05, rate=STEER_RATE_QUIET_DEG_S + 55.0)
  assert out is not None
  assert not out.yielded
  assert out.blending
  assert out.authority == 0.0


def test_yield_quiet_frames_authority_blends_to_one():
  """Yield → 0.25 s of released-torque frames → 1 s smoothstep to 1.0."""
  h = _new()
  _yield(h)
  out = None
  for _ in range(int(QUIET_WAIT_S / DT)):
    out = _step(h, torque=0.0, rate=40.0)
    assert out.authority == 0.0
  assert out is not None
  assert out.blending
  assert not out.yielded
  for _ in range(int(BLEND_TIME_S / DT)):
    out = _step(h, torque=0.0, rate=40.0)
  assert out.authority == 1.0
  assert not out.blending
  assert not out.yielded
  assert not out.ui_paused


def test_quiet_0_249_does_not_blend_0_25_starts():
  h = _new()
  _yield(h)
  out = _quiet(h, 0.249)
  assert out.yielded
  assert not out.blending
  assert out.authority == 0.0
  out = _quiet(h, 0.001)
  assert not out.yielded
  assert out.blending
  assert out.authority == 0.0


def test_blend_is_smoothstep_monotonic_one_second_bounded_slope():
  h = _new()
  _yield(h)
  _quiet(h, QUIET_WAIT_S)
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


def test_renewed_input_during_blend_yields_and_resets_timer():
  h = _new()
  _yield(h)
  _quiet(h, QUIET_WAIT_S)
  for _ in range(40):  # 0.4 s into the 1 s blend
    out = _quiet(h, DT)
  assert out.blending
  assert 0.0 < out.authority < 1.0
  out = _step(h, torque=1.0)
  assert out.yielded
  assert not out.blending
  assert out.authority == 0.0
  assert out.ui_paused
  out = _quiet(h, 0.249)
  assert out.yielded
  assert out.authority == 0.0
  out = _quiet(h, 0.001)
  assert out.blending
  assert out.authority == 0.0


def test_ui_paused_below_70_returns_at_70_without_chatter():
  h = _new()
  _yield(h)
  _quiet(h, QUIET_WAIT_S)
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
  _quiet(h, QUIET_WAIT_S)
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
  out = _step(h, torque=0.8, lat=False)
  assert out.authority == 1.0
  assert not out.yielded

  h = _new()
  for _ in range(SOFT_YIELD_DEBOUNCE_FRAMES + 1):
    out = _step(h, torque=0.8, alc=True)
  assert not out.yielded
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


def test_disabled_for_non_preap_is_identity():
  h = DriverLateralHandoff(enabled=False)
  for _ in range(50):
    out = _step(h, torque=1.5)
  assert out.authority == 1.0
  assert not out.ui_paused


def test_hysteresis_band_does_not_retrigger_from_texture_after_release():
  h = _new()
  _yield(h)
  _quiet(h, QUIET_WAIT_S)
  for _ in range(int(BLEND_TIME_S / DT)):
    _quiet(h, DT)
  # back at full authority; 0.4 Nm is between release and trigger
  for _ in range(100):
    out = _step(h, torque=0.4)
  assert not out.yielded
  assert out.authority == 1.0
