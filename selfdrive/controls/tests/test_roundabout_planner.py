"""Planner hook: RB funnel eases aTarget; a sharp corner / no flag does not.

Full LongitudinalPlanner loop needs cereal/scons. The planner calls
`apply_roundabout_plan` on the liveMapDataNAP hint — that is the aTarget
path under test here (Willmar: aTarget stayed +0.4 at 49 mph).
"""
from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import LOOKAHEAD_NORMAL
from openpilot.selfdrive.mapd.roundabout import (
  RB_V_MAX_MS,
  RoundaboutHint,
  apply_roundabout_plan,
)


def test_funnel_triggers_speed_ease():
  v_ego = 49.0 * CV.MPH_TO_MS
  v_cruise = 50.0 * CV.MPH_TO_MS
  hint = RoundaboutHint(
    approaching=True, distance_m=90.0, speed_limit_ms=20.0 * CV.MPH_TO_MS,
  )
  v_c, v_hud, a_tgt, rb_v = apply_roundabout_plan(
    v_ego, v_cruise, v_cruise, 0.4, hint, LOOKAHEAD_NORMAL,
  )
  assert rb_v is not None
  assert a_tgt < -0.15, a_tgt
  assert v_c < v_cruise
  assert v_hud < v_cruise
  assert rb_v <= RB_V_MAX_MS + 8.0


def test_sharp_corner_without_flag_does_not_ease():
  v_ego = 49.0 * CV.MPH_TO_MS
  v_cruise = 50.0 * CV.MPH_TO_MS
  v_c, v_hud, a_tgt, rb_v = apply_roundabout_plan(
    v_ego, v_cruise, v_cruise, 0.4, None, LOOKAHEAD_NORMAL,
  )
  assert rb_v is None
  assert a_tgt == 0.4
  assert v_c == v_cruise
  assert v_hud == v_cruise


def test_on_roundabout_holds_soft_target():
  v_ego = 49.0 * CV.MPH_TO_MS
  v_cruise = 50.0 * CV.MPH_TO_MS
  hint = RoundaboutHint(on_roundabout=True, speed_limit_ms=20.0 * CV.MPH_TO_MS)
  v_c, _v_hud, a_tgt, rb_v = apply_roundabout_plan(
    v_ego, v_cruise, v_cruise, 0.4, hint, LOOKAHEAD_NORMAL,
  )
  assert rb_v is not None
  assert abs(rb_v - 20.0 * CV.MPH_TO_MS) < 1e-6
  assert a_tgt < -0.15, a_tgt
  assert v_c <= rb_v
