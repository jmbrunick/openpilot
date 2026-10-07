"""Wheel-resume gate: no take-back while the wheel is still uncoiling.

Rapid back-and-forth turns (roundabout, switchback, parking lot): the driver
yields lateral, then lets the wheel slip through light / open hands. Hands
level 0 for 0.15 s used to start the 1 s blend mid-uncoil and OP grabbed the
wheel. After a big-angle handoff the blend now waits until the wheel is within
WHEEL_RESUME_STRAIGHT_DEG of straight (and stays there for a dwell).
"""
import math
from pathlib import Path

import pytest

from openpilot.selfdrive.controls.lib import driver_lateral_handoff as dlh
from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  BLEND_TIME_S,
  DT_CTRL,
  HANDS_OFF_CONFIRM_S,
  WHEEL_GATE_ARM_DEG,
  WHEEL_GATE_MAX_HOLD_S,
  WHEEL_RESUME_STRAIGHT_DEG,
  WHEEL_STRAIGHT_DWELL_S,
  DriverLateralHandoff,
  lat_active_after_handoff,
)

MPH = 0.44704


class Sim:
  def __init__(self, v_ego=20.0 * MPH, with_angle=True, blinker=False):
    self.h = DriverLateralHandoff()
    self.v = v_ego
    self.with_angle = with_angle
    self.blinker = blinker
    self.angle = 0.0
    self.out = None
    self.t = 0.0

  def step(self, *, tq=0.0, hands=0, angle=None, v=None, lat=True, pressed=False):
    if angle is not None:
      self.angle = angle
    if v is not None:
      self.v = v
    self.out = self.h.update(
      engaged=True, lat_would_be_active=lat, steering_torque=tq, steering_rate_deg=0.0,
      hands_on_level=hands, v_ego=self.v, blinker_paused=self.blinker,
      steering_pressed=pressed,
      steering_angle_deg=(self.angle if self.with_angle else None))
    self.t += DT_CTRL
    return self.out

  def push_to(self, angle, frames=20, tq=1.5):
    """Hands-on firm push that yields; wheel ends at `angle`."""
    for i in range(frames):
      self.step(tq=tq, hands=1, angle=angle * (i + 1) / frames)
    assert self.out.yielded
    return self.out

  def hands_off(self, seconds, angle=None):
    n = int(round(seconds / DT_CTRL))
    for _ in range(n):
      self.step(angle=angle)
    return self.out

  def uncoil(self, a0, a1, seconds):
    """Hands off; wheel slides linearly a0 -> a1. Returns first blend time or None."""
    n = int(round(seconds / DT_CTRL))
    for i in range(n):
      self.step(angle=a0 + (a1 - a0) * (i + 1) / n)
      if self.out.blending:
        return self.t
    return None


def test_constants_are_in_the_agreed_ranges():
  assert 10.0 <= WHEEL_RESUME_STRAIGHT_DEG <= 15.0
  assert WHEEL_GATE_ARM_DEG > WHEEL_RESUME_STRAIGHT_DEG
  assert WHEEL_GATE_MAX_HOLD_S > 2.0
  assert WHEEL_STRAIGHT_DWELL_S >= HANDS_OFF_CONFIRM_S - 1e-9


def test_without_angle_resume_is_unchanged():
  """No angle wired (None) -> hands-off confirm then blend, as before."""
  s = Sim(with_angle=False)
  s.push_to(200.0)
  s.hands_off(HANDS_OFF_CONFIRM_S + 0.03, angle=200.0)
  assert s.out.blending


def test_uncoil_from_large_angle_does_not_resume_until_straight():
  s = Sim()
  s.push_to(200.0)
  # hands off, wheel slides 200 -> 40 deg over 2 s: still yielded, EPS free
  assert s.uncoil(200.0, 40.0, 2.0) is None
  assert s.out.yielded and not lat_active_after_handoff(True, s.out.yielded)
  assert s.out.authority == 0.0
  # reaches ~straight and stays: blend starts after the dwell
  t_blend = s.uncoil(40.0, 5.0, 1.0)
  assert t_blend is not None
  assert s.out.blending and lat_active_after_handoff(True, s.out.yielded)


def test_blend_starts_within_dwell_of_entering_the_window():
  s = Sim()
  s.push_to(150.0)
  s.hands_off(0.5, angle=150.0)
  assert s.out.yielded
  s.step(angle=WHEEL_RESUME_STRAIGHT_DEG - 1.0)
  assert s.out.yielded
  frames = int(round(WHEEL_STRAIGHT_DWELL_S / DT_CTRL))
  for _ in range(frames + 1):
    s.step(angle=WHEEL_RESUME_STRAIGHT_DEG - 1.0)
  assert s.out.blending


def test_just_outside_the_window_still_holds():
  s = Sim()
  s.push_to(150.0)
  s.hands_off(2.0, angle=WHEEL_RESUME_STRAIGHT_DEG + 1.0)
  assert s.out.yielded and not s.out.blending


def test_rapid_swing_through_centre_is_not_straight():
  """Left lock -> right lock in ~0.25 s passes centre: no resume."""
  s = Sim()
  s.push_to(300.0)
  s.hands_off(HANDS_OFF_CONFIRM_S + 0.05, angle=300.0)
  assert s.out.yielded
  n = 25
  for i in range(n):
    s.step(angle=300.0 - 600.0 * (i + 1) / n)
    assert not s.out.blending
  assert s.out.yielded
  s.hands_off(1.0, angle=-300.0)
  assert s.out.yielded


def test_back_and_forth_turns_stay_yielded_throughout():
  s = Sim()
  s.push_to(180.0)
  for k in range(6):
    sign = -1.0 if k % 2 == 0 else 1.0
    a0 = -sign * 180.0
    # hand off, uncoil most of the way, then regrip for the next turn
    s.uncoil(a0, a0 * 0.2, 0.8)
    assert s.out.yielded and not s.out.blending, k
    for i in range(15):
      s.step(tq=1.2 * sign, hands=1, angle=a0 * 0.2 + (sign * 180.0 - a0 * 0.2) * (i + 1) / 15)
    assert s.out.yielded


def test_small_angle_nudge_resumes_as_before():
  """Highway lane nudge never reaches the arming angle: unchanged."""
  s = Sim(v_ego=30.0)
  s.push_to(WHEEL_GATE_ARM_DEG - 5.0)
  s.hands_off(HANDS_OFF_CONFIRM_S + 0.03, angle=WHEEL_GATE_ARM_DEG - 5.0)
  assert s.out.blending


def test_arming_is_the_peak_since_the_yield_not_the_start_angle():
  """Yield began near centre; wheel went to lock later -> still gated."""
  s = Sim()
  s.push_to(10.0, frames=12)
  for i in range(30):
    s.step(tq=1.2, hands=1, angle=10.0 + 4.0 * (i + 1))
  assert s.angle > WHEEL_GATE_ARM_DEG
  s.hands_off(1.0, angle=100.0)
  assert s.out.yielded and not s.out.blending


def test_hands_back_on_still_holds_and_hands_off_restarts():
  s = Sim()
  s.push_to(150.0)
  s.hands_off(0.5, angle=150.0)
  for _ in range(5):
    s.step(hands=1, angle=140.0)
  assert s.out.yielded
  s.hands_off(0.5, angle=140.0)
  assert s.out.yielded and not s.out.blending
  s.hands_off(0.5, angle=3.0)
  assert s.out.blending


def test_hands_on_during_the_dwell_restarts_the_dwell():
  s = Sim()
  s.push_to(150.0)
  s.hands_off(HANDS_OFF_CONFIRM_S + 0.10, angle=3.0)   # 0.10 s into the dwell
  assert s.out.yielded and not s.out.blending
  s.step(hands=1, angle=3.0)
  # confirm (0.15) + full dwell (0.15) again; a stale dwell would end at ~0.20
  s.hands_off(0.25, angle=3.0)
  assert s.out.yielded and not s.out.blending
  s.hands_off(0.10, angle=3.0)
  assert s.out.blending


def test_hands_on_restarts_the_safety_valve():
  s = Sim()
  s.push_to(150.0)
  s.hands_off(WHEEL_GATE_MAX_HOLD_S - 0.5, angle=90.0)
  s.step(hands=1, angle=90.0)
  s.hands_off(2.0, angle=90.0)
  assert s.out.yielded and not s.out.blending


def test_firm_push_in_window_cancels_blend_and_keeps_the_peak():
  s = Sim()
  s.push_to(150.0)
  s.hands_off(0.3, angle=3.0)
  assert s.out.blending
  for _ in range(5):
    s.step(tq=1.5, hands=1, angle=60.0)
  assert s.out.yielded and not s.out.blending
  # still above the window: gate remembers the big-angle history
  s.hands_off(1.0, angle=60.0)
  assert s.out.yielded and not s.out.blending
  s.hands_off(0.5, angle=2.0)
  assert s.out.blending


def test_completed_blend_clears_the_peak():
  s = Sim()
  s.push_to(150.0)
  s.hands_off(0.5, angle=3.0)
  assert s.out.blending
  s.hands_off(BLEND_TIME_S + 0.1, angle=3.0)
  assert s.out.authority == 1.0 and not s.out.blending
  # next small nudge is not gated by the old peak
  s.push_to(15.0)
  s.hands_off(HANDS_OFF_CONFIRM_S + 0.03, angle=15.0)
  assert s.out.blending


def test_safety_valve_lets_go_after_max_hold():
  s = Sim()
  s.push_to(150.0)
  s.hands_off(WHEEL_GATE_MAX_HOLD_S - 0.5, angle=90.0)
  assert s.out.yielded and not s.out.blending
  s.hands_off(1.0, angle=90.0)
  assert s.out.blending


def test_low_speed_lot_is_gated_and_crossing_10mph_at_lock_does_not_resume():
  """Lot: v < 10 mph inhibits re-enable; crossing 10 mph with the wheel near
  lock used to land in yield and resume after 0.15 s hands-off."""
  s = Sim(v_ego=4.0 * MPH)
  s.push_to(400.0, tq=1.5)
  s.hands_off(1.0, angle=400.0)
  assert s.out.yielded and not s.out.blending
  # accelerate through 10 mph with the wheel still at lock
  for i in range(150):
    s.step(angle=380.0, v=(4.0 + 0.1 * i) * MPH)
  assert s.v > 10.0 * MPH
  assert s.out.yielded and not s.out.blending
  # wheel unwinds: now it resumes
  s.hands_off(0.5, angle=4.0)
  assert s.out.blending


def test_low_speed_without_crossing_never_resumes_regardless_of_angle():
  s = Sim(v_ego=5.0 * MPH)
  s.push_to(300.0)
  s.hands_off(2.0, angle=0.0)
  assert s.out.yielded and not s.out.blending


def test_blinker_clear_path_is_gated_too():
  s = Sim(blinker=True)
  s.push_to(200.0)
  s.hands_off(0.5, angle=200.0)
  s.blinker = False
  s.hands_off(1.0, angle=200.0)
  assert s.out.yielded and not s.out.blending
  s.hands_off(0.5, angle=5.0)
  assert s.out.blending


def test_inhibit_cleared_landing_cannot_blend_in_one_big_step():
  """The inhibit-cleared landing accrues hands-off time itself; with a long
  frame (dt >= confirm) it used to start the blend on the spot."""
  s = Sim(blinker=True)
  s.push_to(200.0)
  s.hands_off(0.5, angle=200.0)
  s.blinker = False
  out = s.h.update(
    engaged=True, lat_would_be_active=True, steering_torque=0.0, steering_rate_deg=0.0,
    hands_on_level=0, v_ego=s.v, blinker_paused=False, dt=HANDS_OFF_CONFIRM_S + 0.05,
    steering_angle_deg=200.0)
  assert out.yielded and not out.blending


def test_standstill_blip_keeps_the_peak_when_inhibited():
  s = Sim(v_ego=3.0 * MPH)
  s.push_to(300.0)
  s.hands_off(0.3, angle=300.0)
  s.step(lat=False, angle=300.0)   # standstill / fault blip, latch reset
  s.v = 15.0 * MPH
  # wheel came back to 20 deg (below the arming angle, above the window):
  # only the remembered peak keeps the gate on
  s.hands_off(1.0, angle=20.0)
  assert s.out.yielded and not s.out.blending
  s.hands_off(0.5, angle=3.0)
  assert s.out.blending


def test_lane_change_and_disengage_reset_the_gate():
  s = Sim()
  s.push_to(200.0)
  out = s.h.update(engaged=True, lat_would_be_active=True, steering_torque=0.0,
                   steering_rate_deg=0.0, alc_active=True, v_ego=20 * MPH,
                   steering_angle_deg=200.0)
  assert out.authority == 1.0 and not out.yielded
  assert s.h._peak_angle_deg == 0.0
  s2 = Sim()
  s2.push_to(200.0)
  s2.h.update(engaged=False, lat_would_be_active=True, steering_torque=0.0,
              steering_rate_deg=0.0, v_ego=20 * MPH, steering_angle_deg=200.0)
  assert s2.h._peak_angle_deg == 0.0


def test_non_finite_or_missing_angle_never_blocks():
  for bad in (float("nan"), float("inf")):
    s = Sim()
    s.push_to(200.0)
    s.hands_off(HANDS_OFF_CONFIRM_S + 0.03, angle=bad)
    assert s.out.blending


def test_sign_of_the_angle_does_not_matter():
  for sign in (1.0, -1.0):
    s = Sim()
    s.push_to(sign * 200.0, tq=sign * 1.5)
    s.hands_off(1.0, angle=sign * 50.0)
    assert s.out.yielded
    s.hands_off(0.5, angle=sign * 4.0)
    assert s.out.blending


def test_latactive_false_for_the_whole_extended_yield():
  s = Sim()
  s.push_to(200.0)
  for i in range(300):
    s.step(angle=200.0 - 0.5 * i)
    assert not lat_active_after_handoff(True, s.out.yielded) or s.out.blending


def test_controlsd_passes_the_steering_angle_to_the_handoff():
  repo = Path(__file__).resolve().parents[4]
  controlsd = (repo / "selfdrive/controls/controlsd.py").read_text()
  assert (
    "      steering_angle_deg=float(CS.steeringAngleDeg),\n"
    "      steering_pressed=bool(CS.steeringPressed),\n"
    "      model_curvature=float(self._raw_model_curvature),\n"
    "      measured_curvature=float(self.curvature),\n"
  ) in controlsd


def test_gate_is_written_in_the_module_docstring_and_pins_the_contract():
  assert "WHEEL-RESUME GATE" in (dlh.__doc__ or "")
  assert math.isclose(WHEEL_RESUME_STRAIGHT_DEG, 12.0)
  assert BLEND_TIME_S == 1.0 and HANDS_OFF_CONFIRM_S == 0.15


@pytest.mark.parametrize("deg", [10.0, 15.0])
def test_boundary_values_of_the_range_work(deg, monkeypatch):
  monkeypatch.setattr(dlh, "WHEEL_RESUME_STRAIGHT_DEG", deg)
  s = Sim()
  s.push_to(150.0)
  s.hands_off(1.0, angle=deg + 0.5)
  assert s.out.yielded
  s.hands_off(0.5, angle=deg - 0.5)
  assert s.out.blending
