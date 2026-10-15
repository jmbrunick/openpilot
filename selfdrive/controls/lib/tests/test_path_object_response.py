"""Oct 14 path-object response: zones, avoid shift and log-only brake decel."""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib import path_object_response as por


def _obs(lat, cls="human", rng=40.0, span=0.5, agree=True, ev=True, vrel=-15.0):
  return SimpleNamespace(agree=agree, visionEvaluated=ev, rangeM=rng, lateral=lat, spanM=span,
                         objectClass=cls, vRel=vrel)


def _obj(*a, **k):
  return por.confirmed_from_msg(_obs(*a, **k), 0.1)


def test_needs_radar_and_vision_agreement_and_freshness():
  assert por.confirmed_from_msg(_obs(1.0, agree=False), 0.1) is None
  assert por.confirmed_from_msg(_obs(1.0, ev=False), 0.1) is None
  assert por.confirmed_from_msg(_obs(1.0), 1.0) is None
  assert por.confirmed_from_msg(_obs(1.0), 0.1) is not None


def test_zones():
  # car edge at 1.0 m; object half span 0.25
  assert por.zone(_obj(0.0)) == "brake"            # in the path
  assert por.zone(_obj(1.5)) == "brake"            # gap 0.25 m <= 1 ft
  assert por.zone(_obj(-1.6)) == "steer"           # gap 0.35 m
  assert por.zone(_obj(3.0)) == ""                 # human past ~3 ft beyond the 1.7 m half
  assert por.zone(_obj(2.8, cls="animal")) == "steer"
  assert por.zone(_obj(9.0, cls="animal")) == ""


def test_avoid_target_moves_away_and_stays_in_lane():
  right = por.avoid_target(_obj(1.6))
  assert -por.AVOID_CAP_M <= right < 0.0          # object right -> shift left
  assert por.avoid_target(_obj(-1.6)) > 0.0
  assert abs(por.avoid_target(_obj(1.6), lane_room=0.2)) == pytest.approx(0.2)
  assert por.avoid_target(_obj(0.5)) == 0.0        # brake zone: no steering


def test_brake_decel_is_strong_and_clipped_to_the_stack_floor():
  d = por.brake_decel(_obj(0.0, rng=30.0), 20.0, accel_min=-3.5)
  assert d == pytest.approx(-3.5)
  far = por.brake_decel(_obj(0.0, rng=85.0), 10.0, accel_min=-3.5)
  assert -3.5 <= far <= -1.75
  assert por.brake_decel(_obj(1.8), 20.0) == 0.0   # steer zone: never brakes


def test_avoid_shift_override_resume_and_release():
  av = por.AvoidShift()
  o = _obj(1.6)
  kw = dict(v_ego=13.0, gas_pressed=False, lat_active=True, dt=0.01)
  outs = [av.update(o, steering_pressed=False, **kw) for _ in range(300)]
  assert outs[-1] == pytest.approx(por.avoid_target(o), abs=1e-6)
  assert av.update(o, steering_pressed=True, **kw) == 0.0       # wheel overrides at once
  res = [av.update(o, steering_pressed=False, **kw) for _ in range(200)]
  assert all(abs(b) >= abs(a) - 1e-12 for a, b in zip(res, res[1:], strict=False))
  assert res[-1] == pytest.approx(outs[-1], abs=1e-6)
  gone = [av.update(None, steering_pressed=False, **kw) for _ in range(400)]
  assert gone[50] == pytest.approx(outs[-1], abs=1e-6)          # held ~1 s
  assert all(abs(b) <= abs(a) + 1e-12 for a, b in zip(gone, gone[1:], strict=False))
  assert gone[-1] == 0.0


def test_gas_and_low_speed_disable():
  av = por.AvoidShift()
  o = _obj(1.6)
  assert av.update(o, v_ego=13.0, gas_pressed=True, steering_pressed=False, lat_active=True, dt=0.01) == 0.0
  assert av.update(o, v_ego=5.0, gas_pressed=False, steering_pressed=False, lat_active=True, dt=0.01) == 0.0
