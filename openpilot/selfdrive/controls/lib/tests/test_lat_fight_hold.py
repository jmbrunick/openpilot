"""Push to take, release to give back. The tug-of-war latch is gone.

Two re-yields inside 15 s only slow the next taper. They never block a return.
"""

from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  AGREE_CURVATURE_ERR,
  AGREE_HOLD_S,
  BLEND_TIME_S,
  FULL_LATERAL_RESET_S,
  LIGHT_HOLD_S,
  RELEASE_HOLD_S,
  REYIELD_WINDOW_S,
  TAPER_RATE_MAX_PER_S,
  TAPER_RATE_MIN_PER_S,
  YIELD_FIRM_FRAMES,
  DriverLateralHandoff,
  lat_active_after_handoff,
  taper_rate_per_s,
)

DT = 0.01
MODEL_K = 0.01
WHEEL_K = 0.0


def _step(h, *, torque=0.0, hands=0, dt=DT, model_k=None, wheel_k=None,
          engaged=True, lat=True, angle=0.0):
  return h.update(
    engaged=engaged, lat_would_be_active=lat,
    steering_torque=torque, steering_rate_deg=-4095.5,
    hands_on_level=hands, dt=dt,
    steering_angle_deg=angle,
    model_curvature=model_k, measured_curvature=wheel_k,
  )


def _yield_once(h, torque=1.5, hands=0):
  out = None
  for _ in range(YIELD_FIRM_FRAMES):
    out = _step(h, torque=torque, hands=hands)
  assert out.yielded and out.authority == 0.0
  assert not lat_active_after_handoff(True, out.yielded)
  return out


def _release(h, seconds, model_k=None, wheel_k=None, torque=0.0):
  n = max(1, int(round(seconds / DT)))
  out = None
  for _ in range(n):
    out = _step(h, torque=torque, hands=0, model_k=model_k, wheel_k=wheel_k)
  return out


def test_release_rules_do_not_wait_on_hands():
  h = DriverLateralHandoff()
  _yield_once(h, torque=1.5)
  # R1: quiet torque, hands still on the rim.
  out = _release(h, RELEASE_HOLD_S + 0.02, torque=0.2)
  # hands default 0 in _release; pass hands via steps
  h = DriverLateralHandoff()
  _yield_once(h)
  out = None
  for _ in range(int(round((RELEASE_HOLD_S + 0.02) / DT))):
    out = _step(h, torque=0.2, hands=1)
  assert out.blending and not out.yielded
  assert lat_active_after_handoff(True, out.yielded)


def test_r2_returns_mid_curve_when_curvature_agrees():
  h = DriverLateralHandoff()
  _yield_once(h, torque=1.5)
  # Holding torque above R1, curvature still apart: stay yielded.
  out = _release(h, 0.40, model_k=0.002, wheel_k=0.0, torque=0.9)
  assert out.yielded and not out.blending
  # Agreement for 0.5 s, torque under 1.2 and not rising.
  out = _release(h, AGREE_HOLD_S + 0.02, model_k=0.0005, wheel_k=0.0, torque=0.9)
  assert out.blending and not out.yielded


def test_r3_returns_without_curvature_agreement():
  h = DriverLateralHandoff()
  _yield_once(h)
  out = _release(h, LIGHT_HOLD_S - 0.1, model_k=MODEL_K, wheel_k=WHEEL_K, torque=0.5)
  assert out.yielded and not out.blending
  out = _release(h, 0.2, model_k=MODEL_K, wheel_k=WHEEL_K, torque=0.5)
  assert out.blending and not out.yielded


def test_disagreement_above_the_light_band_stays_yielded():
  h = DriverLateralHandoff()
  _yield_once(h)
  out = _release(h, LIGHT_HOLD_S + 0.5, model_k=MODEL_K, wheel_k=WHEEL_K, torque=1.0)
  assert out.yielded and not out.blending


def test_two_reyields_slow_the_next_taper_and_full_lateral_clears_it():
  h = DriverLateralHandoff()
  _yield_once(h)
  _release(h, RELEASE_HOLD_S + 0.02)
  assert h._blending and not h._taper_soften
  # Deliberate push: no curvature, so a rise above the taper baseline re-yields.
  for _ in range(16):
    _step(h, torque=1.5)
  assert h._yielded
  _release(h, RELEASE_HOLD_S + 0.02)
  assert h._blending
  for _ in range(16):
    _step(h, torque=1.5)
  assert h._yielded and h._soften
  _release(h, RELEASE_HOLD_S + 0.02)
  assert h._blending and h._taper_soften
  rate = taper_rate_per_s(0.0, soften=h._taper_soften)
  assert rate == TAPER_RATE_MIN_PER_S / BLEND_TIME_S
  # Finish the slow taper, then 10 s of full lateral clears the softener.
  _release(h, BLEND_TIME_S + 0.05)
  assert h.authority == 1.0 and not h._blending
  _release(h, FULL_LATERAL_RESET_S)
  assert not h._soften
  assert h._reyields == []


def test_helping_torque_does_not_cancel_the_taper():
  h = DriverLateralHandoff()
  _yield_once(h)
  # Model wants more left than the wheel. Positive torque helps.
  _release(h, RELEASE_HOLD_S + 0.02, model_k=0.001, wheel_k=0.0)
  assert h._blending
  out = None
  for _ in range(80):
    out = _step(h, torque=1.5, model_k=0.001, wheel_k=0.0)
    if out.authority >= 1.0 and not out.blending:
      break
  assert out is not None and not out.yielded and out.authority == 1.0


def test_opposing_torque_reyields():
  h = DriverLateralHandoff()
  _yield_once(h)
  # R2 with torque 0.5 so the taper baseline sits under the oppose line
  # and a -1.0 Nm push is not also a 0.6 Nm rise.
  _release(h, 0.2, model_k=0.002, wheel_k=0.0, torque=0.5)
  assert h._yielded
  _release(h, AGREE_HOLD_S + 0.02, model_k=0.0004, wheel_k=0.0, torque=0.5)
  assert h._blending
  out = None
  for _ in range(16):
    out = _step(h, torque=-1.0, model_k=0.0004, wheel_k=0.0)
  assert out.yielded and not out.blending


def test_taper_rate_stays_inside_the_blend_window():
  assert taper_rate_per_s(None) == TAPER_RATE_MIN_PER_S / BLEND_TIME_S
  assert taper_rate_per_s(0.0) == TAPER_RATE_MAX_PER_S / BLEND_TIME_S
  # 0.002 / 0.001 = 2.0, the ceiling.
  assert taper_rate_per_s(0.001) == TAPER_RATE_MAX_PER_S / BLEND_TIME_S
  # 0.002 / 0.004 = 0.5, clamped to the floor.
  assert taper_rate_per_s(0.004) == TAPER_RATE_MIN_PER_S / BLEND_TIME_S
  assert AGREE_CURVATURE_ERR == 0.0008
  assert REYIELD_WINDOW_S == 15.0
