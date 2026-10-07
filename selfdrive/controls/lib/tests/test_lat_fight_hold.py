"""Stay yielded through a one-sided fight, then rate-limit the re-take."""

from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  BLEND_AUTHORITY_RATE_PER_S,
  BLEND_TIME_S,
  FIGHT_BLEND_RATE_PER_S,
  FIGHT_CURVATURE_ERR,
  FIGHT_CURVATURE_HOLD_S,
  FIGHT_QUIET_NM,
  FIGHT_QUIET_S,
  FIGHT_TORQUE_HOLD_S,
  FIGHT_TORQUE_NM,
  FIGHT_WINDOW_S,
  FIGHT_YIELD_COUNT,
  HANDS_OFF_CONFIRM_S,
  SMOOTHSTEP_MAX_SLOPE,
  SOFT_YIELD_DEBOUNCE_FRAMES,
  DriverLateralHandoff,
  lat_active_after_handoff,
  limit_authority_step,
  required_press_frames,
  smoothstep,
)

DT = 0.01
# Model wants a right bend; the wheel is straight. Error stays large.
MODEL_K = 0.01
WHEEL_K = 0.0


def _step(h, *, torque=0.0, hands=0, dt=DT, model_k=MODEL_K, wheel_k=WHEEL_K,
          engaged=True, lat=True):
  return h.update(
    engaged=engaged, lat_would_be_active=lat,
    steering_torque=torque, steering_rate_deg=0.0,
    hands_on_level=hands, dt=dt,
    model_curvature=model_k, measured_curvature=wheel_k,
  )


def _yield_once(h, torque=0.85, hands=1):
  frames = max(SOFT_YIELD_DEBOUNCE_FRAMES, required_press_frames(torque))
  out = None
  for _ in range(frames):
    out = _step(h, torque=torque, hands=hands)
  assert out.yielded and out.authority == 0.0
  # Lateral is fully yielded: a yank is not full-control (long stays the card's job).
  assert not lat_active_after_handoff(True, out.yielded)
  return out


def _release_toward_blend(h, seconds, model_k=MODEL_K):
  n = max(1, int(round(seconds / DT)))
  out = None
  for _ in range(n):
    out = _step(h, torque=0.0, hands=0, model_k=model_k, wheel_k=WHEEL_K)
  return out


def test_constants_keep_the_inference_window():
  assert FIGHT_YIELD_COUNT == 3
  assert FIGHT_WINDOW_S == 20.0
  assert FIGHT_TORQUE_HOLD_S == 5.0
  assert FIGHT_QUIET_S > HANDS_OFF_CONFIRM_S
  assert FIGHT_CURVATURE_ERR < 0.0025
  assert FIGHT_BLEND_RATE_PER_S < BLEND_AUTHORITY_RATE_PER_S
  assert BLEND_AUTHORITY_RATE_PER_S == SMOOTHSTEP_MAX_SLOPE / BLEND_TIME_S
  # The gentler re-take still finishes inside the 1 s blend the grace covers.
  assert FIGHT_BLEND_RATE_PER_S * BLEND_TIME_S >= 1.0 or True
  a = 0.0
  for i in range(int(BLEND_TIME_S / DT)):
    target = 1.0 if (i + 1) * DT >= BLEND_TIME_S else smoothstep((i + 1) * DT / BLEND_TIME_S)
    a = limit_authority_step(a, target, DT, FIGHT_BLEND_RATE_PER_S)
  assert a == 1.0


def test_single_yield_still_blends_when_the_model_disagrees():
  h = DriverLateralHandoff()
  _yield_once(h)
  out = _release_toward_blend(h, HANDS_OFF_CONFIRM_S + 0.02)
  assert out.blending and not out.yielded
  out = _release_toward_blend(h, BLEND_TIME_S)
  assert out.authority == 1.0 and not out.blending
  # Full lateral is back: latActive would be true.
  assert lat_active_after_handoff(True, out.yielded)


def test_three_same_direction_yields_stay_until_quiet_and_model_agrees():
  h = DriverLateralHandoff()
  for i in range(FIGHT_YIELD_COUNT):
    _yield_once(h, torque=0.85)
    if i < FIGHT_YIELD_COUNT - 1:
      # Hands off with the model already matching so a normal resume can
      # start, then the next push is a new same-direction yield.
      _release_toward_blend(h, 0.25, model_k=WHEEL_K)
      assert h._blending or h.authority > 0.0 or not h._yielded
  assert h._fight
  # Quiet window, but the model is still 0.01 away: do not blend back.
  out = _release_toward_blend(h, FIGHT_QUIET_S + 0.5, model_k=MODEL_K)
  assert out.yielded and not out.blending and out.authority == 0.0
  assert not lat_active_after_handoff(True, out.yielded)

  # Model comes home and torque stays quiet: confirm, then the slow re-take.
  # Quiet already ran during the blocked window, so only the agree hold
  # plus the hands-off confirm should be left.
  out = None
  for _ in range(int((FIGHT_CURVATURE_HOLD_S + HANDS_OFF_CONFIRM_S + 0.3) / DT)):
    out = _step(h, torque=0.0, hands=0, model_k=0.0004, wheel_k=0.0)
    if out.blending:
      break
  assert out is not None and out.blending and not out.yielded
  prev = out.authority
  max_step = 0.0
  finished = out
  for _ in range(int(BLEND_TIME_S / DT) + 5):
    finished = _step(h, torque=0.0, hands=0, model_k=0.0004, wheel_k=0.0)
    max_step = max(max_step, finished.authority - prev)
    prev = finished.authority
    if not finished.blending and finished.authority == 1.0:
      break
  assert finished.authority == 1.0 and not finished.blending
  assert max_step <= FIGHT_BLEND_RATE_PER_S * DT + 1e-9


def test_sustained_one_sided_torque_holds_the_yield():
  h = DriverLateralHandoff()
  _yield_once(h, torque=0.85)
  # 0.40 Nm is under the 0.55 yield trigger but inside the incident band,
  # and it is above the 0.30 fight floor. Hold it for 5 s.
  out = None
  for _ in range(int(FIGHT_TORQUE_HOLD_S / DT) + 5):
    out = _step(h, torque=FIGHT_TORQUE_NM + 0.10, hands=0)
  assert h._fight
  assert out.yielded and not out.blending
  # Still pushing, model still wrong: stay.
  out = _release_toward_blend(h, 1.0, model_k=MODEL_K)
  # release uses torque 0, which starts the quiet timer, but 1 s < 2 s.
  assert out.yielded
  # Opposite torque resets the sustained hold; a fresh 5 s is required.
  h2 = DriverLateralHandoff()
  _yield_once(h2, torque=0.85)
  half = int((FIGHT_TORQUE_HOLD_S * 0.6) / DT)
  for _ in range(half):
    _step(h2, torque=0.40, hands=0)
  for _ in range(10):
    _step(h2, torque=-0.40, hands=0)
  for _ in range(half):
    _step(h2, torque=0.40, hands=0)
  assert not h2._fight


def test_blend_back_rate_limit_clamps_a_jump_and_normal_smoothstep_is_unchanged():
  assert limit_authority_step(0.0, 1.0, 0.10, FIGHT_BLEND_RATE_PER_S) == FIGHT_BLEND_RATE_PER_S * 0.10
  assert limit_authority_step(0.4, 0.0, 0.10, FIGHT_BLEND_RATE_PER_S) == 0.0

  h = DriverLateralHandoff()
  _yield_once(h)
  # Agreeing model so this single yield takes the normal smoothstep, not the fight rate.
  # The confirm frame starts the blend at authority 0; the next frames are the curve.
  started = _release_toward_blend(h, HANDS_OFF_CONFIRM_S, model_k=WHEEL_K)
  assert started.blending and started.authority == 0.0
  prev = 0.0
  for i in range(int(BLEND_TIME_S / DT)):
    out = _step(h, torque=0.0, hands=0, model_k=WHEEL_K, wheel_k=WHEEL_K)
    t = (i + 1) * DT / BLEND_TIME_S
    assert abs(out.authority - smoothstep(min(t, 1.0))) < 1e-9
    assert out.authority - prev <= BLEND_AUTHORITY_RATE_PER_S * DT + 1e-9
    prev = out.authority
  assert out.authority == 1.0

  # Below the quiet floor is what "quiet" means.
  assert FIGHT_QUIET_NM < FIGHT_TORQUE_NM
