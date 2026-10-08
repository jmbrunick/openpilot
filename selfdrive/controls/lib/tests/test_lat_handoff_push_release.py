"""Replay-shaped traces for push-to-take, release-to-give-back.

Numbers follow the Oct 7 2026 route analysis (1c95345a3286a5db|00000172).
Logs are not in the tree, so each case is a synthetic trace with the
torque, curvature error, and wheel angle the report measured.
"""

from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  RELEASE_HOLD_S,
  DriverLateralHandoff,
  lat_active_after_handoff,
)
from openpilot.selfdrive.controls.lib.lane_change_nudge import steer_disengage_this_frame

DT = 0.01


def _play(segments, *, blinker=False, v_ego=20.0):
  """segments: (seconds, torque, curv_err, angle). Returns time of first blend."""
  h = DriverLateralHandoff()
  t = 0.0
  out = None
  for seconds, torque, err, angle in segments:
    n = int(round(seconds / DT))
    measured = 0.0
    model = measured + float(err)
    for _ in range(n):
      out = h.update(
        engaged=True, lat_would_be_active=True,
        steering_torque=torque, steering_rate_deg=-4095.5,
        hands_on_level=0, v_ego=v_ego, blinker_paused=blinker,
        steering_angle_deg=angle, dt=DT,
        model_curvature=model, measured_curvature=measured,
      )
      t += DT
      if out.blending:
        return t, out, h
  return None, out, h


def test_logged_yields_return_near_the_proposed_times():
  # 14:21:23 — push, then curvature agrees. R2 ~0.8 s from the push.
  t, out, _ = _play([(0.30, 1.0, 0.002, 0.0), (0.70, 0.8, 0.0005, 0.0)])
  assert t is not None and 0.70 <= t <= 0.95
  assert out.blending

  # 14:22:25 — long hold at 40 deg, then a light straight wheel. ~17.6 s.
  t, out, _ = _play([(17.3, 1.6, 0.002, 40.0), (1.0, 0.1, 0.002, 5.0)])
  assert t is not None and 17.4 <= t <= 18.0
  assert out.blending

  # 14:24:19 — mid-curve agreement. R2 ~3.9 s.
  t, out, _ = _play([(3.4, 1.5, 0.002, 8.0), (0.8, 0.9, 0.0005, 8.0)])
  assert t is not None and 3.7 <= t <= 4.1
  assert out.blending

  # 14:24:55 and 14:34:35 — torque already in the release band. ~0.30 s
  # (the report's 0.2 s is a 10 Hz estimate; R1 cannot be shorter than 0.30).
  for push in (0.10, 0.16):
    t, out, _ = _play([(push, 2.0, 0.0, 0.0), (0.6, 0.15, 0.0, 0.0)])
    assert t is not None
    quiet_for = t - push
    assert 0.25 <= quiet_for <= 0.40
    assert out.blending

  # 14:34:18 — firm-but-not-1.4 hold, then a release. R1 ~1.2 s.
  t, out, _ = _play([(0.90, 1.3, 0.001, 0.0), (0.6, 0.2, 0.001, 0.0)])
  assert t is not None and 1.0 <= t <= 1.4
  assert out.blending


def test_short_nudge_returns_promptly():
  t, out, _ = _play([(0.40, 1.2, 0.001, 4.0), (1.5, 0.0, 0.001, 4.0)])
  assert t is not None and 0.5 <= t <= 1.8
  assert out.blending and not out.yielded


def test_resting_hands_never_yield_or_block_return():
  h = DriverLateralHandoff()
  out = None
  for tq in (0.15, 0.25, 0.35, -0.20):
    for _ in range(300):
      out = h.update(
        engaged=True, lat_would_be_active=True, steering_torque=tq,
        steering_rate_deg=-4095.5, hands_on_level=1, v_ego=20.0,
        steering_angle_deg=2.0, dt=DT,
      )
      assert not out.yielded and out.authority == 1.0
  # A real push yields; resting torque afterwards does not hold the yield.
  for _ in range(10):
    out = h.update(
      engaged=True, lat_would_be_active=True, steering_torque=1.6,
      steering_rate_deg=-4095.5, hands_on_level=1, v_ego=20.0, dt=DT,
    )
  assert out.yielded
  n = int(round((RELEASE_HOLD_S + 0.02) / DT))
  for _ in range(n):
    out = h.update(
      engaged=True, lat_would_be_active=True, steering_torque=0.30,
      steering_rate_deg=-4095.5, hands_on_level=1, v_ego=20.0, dt=DT,
    )
  assert out.blending and not out.yielded


def test_helping_torque_never_cancels_the_taper():
  h = DriverLateralHandoff()
  for _ in range(10):
    h.update(engaged=True, lat_would_be_active=True, steering_torque=1.6,
             steering_rate_deg=-4095.5, v_ego=20.0, dt=DT,
             model_curvature=0.001, measured_curvature=0.0)
  out = None
  for _ in range(int(round((RELEASE_HOLD_S + 0.02) / DT))):
    out = h.update(engaged=True, lat_would_be_active=True, steering_torque=0.0,
                   steering_rate_deg=-4095.5, v_ego=20.0, dt=DT,
                   model_curvature=0.001, measured_curvature=0.0)
  assert out.blending
  for _ in range(80):
    out = h.update(engaged=True, lat_would_be_active=True, steering_torque=1.5,
                   steering_rate_deg=-4095.5, v_ego=20.0, dt=DT,
                   model_curvature=0.001, measured_curvature=0.0)
    if out.authority >= 1.0 and not out.blending:
      break
  assert not out.yielded and out.authority == 1.0
  assert lat_active_after_handoff(True, out.yielded)


def test_yank_at_full_lateral_disengages_and_yielded_does_not():
  h = DriverLateralHandoff()
  out = h.update(engaged=True, lat_would_be_active=True, steering_torque=3.0,
                 steering_rate_deg=-4095.5, hands_on_level=3, steering_pressed=True,
                 v_ego=20.0, dt=DT)
  assert not out.yielded and out.authority == 1.0
  assert lat_active_after_handoff(True, out.yielded)
  assert steer_disengage_this_frame(
    confirm=False, release=False, release_prev=False,
    disengage_edge=True, blocks=False, lat_full_control=True)

  for _ in range(10):
    out = h.update(engaged=True, lat_would_be_active=True, steering_torque=3.0,
                   steering_rate_deg=-4095.5, hands_on_level=3, steering_pressed=True,
                   v_ego=20.0, dt=DT)
  assert out.yielded
  assert not lat_active_after_handoff(True, out.yielded)
  assert not steer_disengage_this_frame(
    confirm=False, release=False, release_prev=False,
    disengage_edge=True, blocks=False, lat_full_control=False)

  for _ in range(40):
    out = h.update(engaged=True, lat_would_be_active=True, steering_torque=0.0,
                   steering_rate_deg=-4095.5, hands_on_level=1, v_ego=20.0, dt=DT)
  assert out.blending and lat_active_after_handoff(True, out.yielded)
  assert not steer_disengage_this_frame(
    confirm=False, release=False, release_prev=False,
    disengage_edge=True, blocks=False, lat_full_control=False)


def test_two_offsets_ignore_the_sna_rate():
  h = DriverLateralHandoff()
  for _ in range(800):
    h.update(engaged=True, lat_would_be_active=True, steering_torque=0.07,
             steering_rate_deg=-4095.5, hands_on_level=0, steering_angle_deg=0.0,
             v_ego=20.0, dt=DT)
  assert 0.05 < h.rest_bias_op < 0.07
  assert h.rest_bias == h.rest_bias_op
  for _ in range(12):
    h.update(engaged=True, lat_would_be_active=True, steering_torque=1.6,
             steering_rate_deg=-4095.5, hands_on_level=1, blinker_paused=True,
             steering_angle_deg=0.0, v_ego=20.0, dt=DT)
  for _ in range(800):
    h.update(engaged=True, lat_would_be_active=True, steering_torque=0.15,
             steering_rate_deg=-4095.5, hands_on_level=0, blinker_paused=True,
             steering_angle_deg=0.0, v_ego=20.0, dt=DT)
  assert h._yielded
  assert 0.12 < h.rest_bias_free < 0.15
  assert h.rest_bias == h.rest_bias_free
