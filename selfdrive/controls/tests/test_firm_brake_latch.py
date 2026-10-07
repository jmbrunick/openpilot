"""Firm-brake latch for the Pre-AP follow actuator guard (F1).

15a 11:26:49: a firm plan (-1.5) on a mid-gap close stepped -1.5 -> -0.22
-> -1.5 at the actuator when v_rel crossed 1.5 m/s, because the guard's
mild-floor clip comes back below that close. The latch keeps the clip off
while the plan is still firm. It may only make the command firmer: every
unlatched path (hold False, #222, EV settle -0.22, MAX, the -1.5 rail) is
the unchanged guard output: lead_approach.py is not touched by the latch.
"""
import itertools
from types import SimpleNamespace

import pytest
from cereal import car

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import firm_brake_latch as fbl
from openpilot.selfdrive.controls.lib.firm_brake_latch import (
  LEAD_FIRM_LATCH_ENTER_CLOSE_MS,
  LEAD_FIRM_LATCH_ENTER_PLAN_MS2,
  LEAD_FIRM_LATCH_HOLD_CLOSE_MS,
  LEAD_FIRM_LATCH_HOLD_PLAN_MS2,
  LEAD_FIRM_LATCH_MAX_S,
  FirmBrakeLatch,
  apply_firm_brake_latch,
  lead_midgap_firm_close_hit,
)
from openpilot.selfdrive.controls.lib.lead_approach import (
  LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2,
  guard_follow_actuator_regen,
)
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState

# Slack 18 m is mid-gap and above the near-gap band, below the large-slack
# early-kinematic brake (20 m): the geometry of the 11:26:49 cliff.
SLACK = 18.0
D_REL = 38.0
V_EGO = 20.0
V_CRUISE = 30.0
A_LEAD_FIRM = -0.6
LIMITS = (-1.5, 0.8)


def _guard(act, plan, v_rel, hold_pass=False, **kw):
  """The unchanged guard, then the latch seam exactly as LongControl wires it."""
  args = dict(v_rel=v_rel, d_rel=D_REL, slack=SLACK, v_ego=V_EGO,
              v_cruise=V_CRUISE, a_lead=A_LEAD_FIRM)
  args.update(kw)
  guarded = guard_follow_actuator_regen(act, plan, **args)
  return apply_firm_brake_latch(
    guarded, act, plan, hold_pass,
    v_rel=args["v_rel"], slack=args["slack"], a_lead=args["a_lead"],
  )


def _hit(v_rel, **kw):
  args = dict(a_lead=A_LEAD_FIRM)
  args.update(kw)
  return lead_midgap_firm_close_hit(v_rel, D_REL, SLACK, **args)


def test_constants_are_pinned():
  assert LEAD_FIRM_LATCH_ENTER_PLAN_MS2 == -0.80
  assert LEAD_FIRM_LATCH_HOLD_PLAN_MS2 == -0.50
  assert LEAD_FIRM_LATCH_ENTER_CLOSE_MS == 1.5
  assert LEAD_FIRM_LATCH_HOLD_CLOSE_MS == 0.80
  assert LEAD_FIRM_LATCH_MAX_S == 4.0


# --- the guard -----------------------------------------------------------------

def test_default_guard_still_clips_a_slow_close_to_mild():
  """No latch: the legacy cliff is untouched (this is what the latch fixes)."""
  assert _guard(-1.5, -1.5, 1.8) == pytest.approx(-1.5)
  assert _guard(-1.5, -1.5, 1.2) == pytest.approx(LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2)


def test_hold_pass_lets_the_firm_plan_reach_the_plant_below_the_close_gate():
  assert _guard(-1.5, -1.5, 1.2, hold_pass=True) == pytest.approx(-1.5)
  assert _guard(-1.0, -1.0, 1.0, hold_pass=True) == pytest.approx(-1.0)


def test_hold_pass_is_never_softer_than_the_unlatched_guard():
  acts = (-1.5, -1.2, -0.8, -0.5, -0.3, -0.22, -0.1, 0.0, 0.4)
  plans = (-1.5, -1.0, -0.8, -0.6, -0.5, -0.4, -0.22, 0.0, 0.3)
  v_rels = (-1.0, 0.0, 0.5, 1.0, 1.2, 1.49, 1.5, 1.8, 3.0, 6.5)
  slacks = (5.0, 10.0, 14.0, 16.0, 18.0, 22.0, 40.0, 60.0)
  a_leads = (None, -1.5, -0.6, -0.3, 0.0, 0.5)
  for act, plan, v_rel, slack, a_lead in itertools.product(acts, plans, v_rels, slacks, a_leads):
    kw = dict(v_rel=v_rel, d_rel=slack + 20.0, slack=slack, v_ego=V_EGO,
              v_cruise=V_CRUISE, a_lead=a_lead)
    base = guard_follow_actuator_regen(act, plan, **kw)
    held = apply_firm_brake_latch(
      base, act, plan, True, v_rel=v_rel, slack=slack, a_lead=a_lead)
    assert held <= base + 1e-12, (act, plan, kw)
    # No hold: the guard output is returned untouched.
    assert apply_firm_brake_latch(
      base, act, plan, False, v_rel=v_rel, slack=slack, a_lead=a_lead) == base
    # The latch never goes below the actuator the PID asked for.
    assert held >= min(act, base) - 1e-12


def test_hold_pass_changes_nothing_when_the_plan_is_not_firm():
  """EV settle -0.22 and coast stay bit-identical with the latch asserted."""
  for plan in (0.3, 0.0, -0.039, -0.22, -0.3):
    for act in (-1.5, -0.6, -0.22, 0.0):
      for v_rel in (0.2, 1.2, 1.8):
        kw = dict(v_rel=v_rel, d_rel=D_REL, slack=SLACK, v_ego=V_EGO,
                  v_cruise=V_CRUISE, a_lead=A_LEAD_FIRM)
        assert _guard(act, plan, v_rel, hold_pass=True) == \
          guard_follow_actuator_regen(act, plan, **kw), (plan, act, v_rel)


def test_emergency_and_rail_paths_ignore_the_latch():
  for kw in (dict(fcw=True), dict(crash_cnt=1), dict(d_rel=10.0)):
    a = _guard(-1.5, -1.5, 1.2, **kw)
    assert a == _guard(-1.5, -1.5, 1.2, hold_pass=True, **kw) == pytest.approx(-1.5)
  # The -1.5 rail is never exceeded by the latch.
  assert _guard(-1.5, -3.0, 1.2, hold_pass=True) >= -1.5


def test_early_kinematic_cap_keeps_ownership_of_its_frame():
  """Slack 22 m, firm lead, close 1.8: the guard's early brake (-0.87) is a
  deliberate softening. The latch must not override it with the raw -1.5."""
  kw = dict(d_rel=42.0, slack=22.0, v_ego=V_EGO, v_cruise=V_CRUISE, a_lead=-0.8)
  guarded = guard_follow_actuator_regen(-1.5, -1.5, v_rel=1.8, **kw)
  assert guarded > -1.2
  assert apply_firm_brake_latch(guarded, -1.5, -1.5, True, v_rel=1.8, slack=22.0, a_lead=-0.8) == guarded


def test_latch_seam_ignores_missing_inputs_and_soft_plans():
  assert apply_firm_brake_latch(None, -1.5, -1.5, True) is None
  assert apply_firm_brake_latch(-0.22, None, -1.5, True) == -0.22
  assert apply_firm_brake_latch(-0.22, -1.5, None, True) == -0.22
  assert apply_firm_brake_latch(-0.22, -1.5, -0.49, True) == -0.22
  assert apply_firm_brake_latch(-0.22, -1.5, -0.50, True) == -1.5
  assert apply_firm_brake_latch(-0.22, -1.5, -1.5, False) == -0.22


# --- the hit helper ------------------------------------------------------------

def test_hit_needs_a_firm_close_in_the_mid_gap_band():
  assert _hit(1.5)
  assert _hit(3.0)
  assert not _hit(1.49)
  assert not _hit(1.8, a_lead=-0.2)             # non-braking catch-up is a coast
  assert not _hit(1.8, a_lead=None)
  assert not _hit(1.8, should_stop=True)
  assert not _hit(7.0)                           # confirmed rapid passes raw anyway
  assert not lead_midgap_firm_close_hit(1.8, D_REL, 60.0, a_lead=A_LEAD_FIRM)  # out of band
  assert not lead_midgap_firm_close_hit(1.8, 10.0, SLACK, a_lead=A_LEAD_FIRM)  # near bumper
  assert not lead_midgap_firm_close_hit(None, D_REL, SLACK, a_lead=A_LEAD_FIRM)


def test_hit_accepts_the_222_residual_brake_without_a_firm_alead():
  """#222: close rising against ego's own accel is a real brake."""
  assert _hit(1.8, a_lead=0.0, prev_v_rel=1.2, a_ego=0.0, dt=0.5)
  assert not _hit(1.8, a_lead=0.0, prev_v_rel=1.8, a_ego=0.0, dt=0.5)


# --- the latch ------------------------------------------------------------------

def _enter(latch):
  assert latch.update(-1.5, 1.8, True, DT_CTRL) is True
  return latch


def test_latch_enters_on_a_firm_plan_and_a_firm_close():
  latch = FirmBrakeLatch()
  assert latch.update(-0.79, 1.8, True, DT_CTRL) is False
  assert latch.update(-1.5, 1.8, False, DT_CTRL) is False
  assert latch.update(-0.8, 1.8, True, DT_CTRL) is True


def test_latch_holds_through_the_close_easing_below_the_entry_gate():
  latch = _enter(FirmBrakeLatch())
  for v_rel in (1.45, 1.2, 1.0, 0.85, 0.8):
    assert latch.update(-1.5, v_rel, False, DT_CTRL) is True
  # A mild plan between enter (-0.8) and hold (-0.5) still holds.
  assert latch.update(-0.6, 1.0, False, DT_CTRL) is True


def test_latch_releases_when_the_plan_goes_soft():
  latch = _enter(FirmBrakeLatch())
  assert latch.update(-0.5, 1.2, False, DT_CTRL) is True
  assert latch.update(-0.49, 1.2, False, DT_CTRL) is False
  assert latch.update(-0.49, 1.2, False, DT_CTRL) is False
  # and re-entering needs a fresh firm plan AND hit
  assert latch.update(-1.5, 1.2, False, DT_CTRL) is False
  assert latch.update(-1.5, 1.8, True, DT_CTRL) is True


def test_latch_releases_when_the_close_collapses():
  latch = _enter(FirmBrakeLatch())
  assert latch.update(-1.5, 0.81, False, DT_CTRL) is True
  assert latch.update(-1.5, 0.79, False, DT_CTRL) is False


def test_latch_expires_after_four_seconds():
  latch = _enter(FirmBrakeLatch())
  steps = int(round(LEAD_FIRM_LATCH_MAX_S / DT_CTRL))
  for _ in range(steps):
    assert latch.update(-1.5, 1.2, False, DT_CTRL) is True
  assert latch.update(-1.5, 1.2, False, DT_CTRL) is False
  # the same frame does not silently re-arm without a hit
  assert latch.update(-1.5, 1.2, False, DT_CTRL) is False


def test_latch_resets_when_the_lead_or_plan_is_missing():
  latch = _enter(FirmBrakeLatch())
  assert latch.update(-1.5, None, False, DT_CTRL) is False
  assert latch.update(-1.5, 1.2, False, DT_CTRL) is False
  latch = _enter(FirmBrakeLatch())
  assert latch.update(None, 1.2, False, DT_CTRL) is False
  assert latch.update(-1.5, 1.2, False, DT_CTRL) is False


def test_latch_reset_clears_state():
  latch = _enter(FirmBrakeLatch())
  latch.reset()
  assert latch.on is False and latch.age == 0.0
  assert latch.update(-1.5, 1.2, False, DT_CTRL) is False


# --- LongControl seam ------------------------------------------------------------

def _params():
  p = car.CarParams.new_message()
  p.brand = "tesla"
  p.carFingerprint = "TESLA_MODEL_S_PREAP"
  p.openpilotLongitudinalControl = True
  p.pcmCruise = False
  from opendbc.car.tesla.preap.constants import PEDAL_LONG_K_BP, PEDAL_LONG_KI_V, PEDAL_LONG_KP_V
  p.longitudinalTuning.kpBP = PEDAL_LONG_K_BP
  p.longitudinalTuning.kpV = PEDAL_LONG_KP_V
  p.longitudinalTuning.kiBP = PEDAL_LONG_K_BP
  p.longitudinalTuning.kiV = PEDAL_LONG_KI_V
  p.longitudinalTuning.kf = 1.0
  p.vEgoStarting = 0.1
  return p


def _cs():
  s = car.CarState.new_message()
  s.vEgo = V_EGO
  s.brakePressed = False
  s.cruiseState.standstill = False
  return s


def _drive(lc, plan, v_rels, a_lead=A_LEAD_FIRM, frames_each=50):
  """One 100 Hz LongControl frame per sample; returns every actuator command."""
  out = []
  for v_rel in v_rels:
    for _ in range(frames_each):
      out.append(lc.update(
        True, _cs(), plan, False, LIMITS, lead_v_rel=v_rel, lead_d_rel=D_REL,
        lead_slack=SLACK, lead_fcw=False, lead_v_ego=V_EGO,
        lead_v_cruise=V_CRUISE, lead_a_lead=a_lead,
      ))
  return out


def _legacy_lc():
  lc = LongControl(_params())
  lc._firm_latch = SimpleNamespace(
    update=lambda *a, **k: False, reset=lambda: None, on=False)
  return lc


def test_longcontrol_removes_the_1p5_to_mild_to_1p5_cliff():
  """E1 shape: firm plan -1.5 held; v_rel 1.8 -> 1.2 -> 1.8."""
  v_rels = (1.8, 1.2, 1.0, 1.8)
  legacy = _drive(_legacy_lc(), -1.5, v_rels)
  latched = _drive(LongControl(_params()), -1.5, v_rels)
  # Legacy shows the cliff (this is the bug being fixed).
  assert max(legacy[60:140]) == pytest.approx(LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2, abs=0.05)
  assert max(b - a for a, b in zip(legacy, legacy[1:], strict=False)) > 0.8
  # Latched: command follows the plan, no up-step anywhere.
  assert max(latched) <= -1.4
  assert max(b - a for a, b in zip(latched, latched[1:], strict=False)) < 0.05


def test_longcontrol_latch_is_never_softer_than_legacy_and_expires():
  v_rels = (1.8, 1.2, 1.0, 0.6, 1.2)
  legacy = _drive(_legacy_lc(), -1.5, v_rels)
  latched = _drive(LongControl(_params()), -1.5, v_rels)
  assert all(n <= o + 1e-9 for n, o in zip(latched, legacy, strict=True))
  # After v_rel < 0.8 the latch is released and the legacy clip returns.
  assert latched[3 * 50 + 10] == pytest.approx(legacy[3 * 50 + 10])
  assert latched[-1] == pytest.approx(legacy[-1])


def test_longcontrol_mild_and_settle_plans_are_bit_identical_to_legacy():
  """EV mild settle -0.22 / coast never arm the latch."""
  for plan in (0.2, 0.0, -0.22, -0.45):
    v_rels = (1.8, 1.2, 1.0, 1.8, 0.5)
    assert _drive(LongControl(_params()), plan, v_rels, frames_each=20) == \
      _drive(_legacy_lc(), plan, v_rels, frames_each=20), plan


def test_longcontrol_drops_the_latch_with_the_lead_and_on_reset():
  lc = LongControl(_params())
  _drive(lc, -1.5, (1.8, 1.2), frames_each=10)
  assert lc._firm_latch.on
  _drive(lc, -1.5, (None,), frames_each=1)
  assert not lc._firm_latch.on
  _drive(lc, -1.5, (1.8, 1.2), frames_each=10)
  assert lc._firm_latch.on
  lc.reset()
  assert not lc._firm_latch.on


def test_longcontrol_latch_off_outside_pid_state():
  lc = LongControl(_params())
  _drive(lc, -1.5, (1.8, 1.2), frames_each=10)
  assert lc._firm_latch.on
  lc.update(False, _cs(), -1.5, False, LIMITS, lead_v_rel=1.2, lead_d_rel=D_REL,
            lead_slack=SLACK, lead_a_lead=A_LEAD_FIRM)
  assert lc.long_control_state == LongCtrlState.off
  assert not lc._firm_latch.on


def test_longcontrol_wires_the_latch_into_the_guard():
  import inspect
  src = inspect.getsource(LongControl)
  assert "apply_firm_brake_latch(" in src
  assert "self._firm_latch.update(" in src
  assert "DT_CTRL," in src
  assert "FirmBrakeLatch" in inspect.getsource(fbl)
