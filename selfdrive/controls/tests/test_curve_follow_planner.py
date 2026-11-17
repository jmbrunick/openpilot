"""Curve-follow wiring in the longitudinal planner (Pre-AP).

NAPCurveFollow: 0 off, 1 shadow (compute and log only), 2 active.
"""
import json

import pytest

from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
from openpilot.selfdrive.controls.lib.curve_preview import PREVIEW_FREE_A_MS2
from openpilot.selfdrive.controls.tests.test_curve_preview import _set_model_path
from openpilot.selfdrive.controls.tests.test_unified_lead_planner import _own_lead, _planner

DT = 0.05
BEND = (120.0, 60.0, 1.0 / 45.0)
STRAIGHT = (1e6, 1.0, 0.0)


def _drive(mode, v, bend, n, mpc_accel=0.4, lead=None, hook=None):
  planner, inputs, params = _planner(v, accel=mpc_accel)
  params.curve_follow = mode
  if lead is not None:
    _own_lead(inputs, *lead)
  s = 0.0
  rows = []
  for k in range(n):
    _set_model_path(inputs["modelV2"], v, s, bend)
    if hook is not None:
      hook(k, planner, inputs)
    planner.update(inputs)
    rows.append((float(planner.output_a_target), float(planner.curve_preview_a), float(planner.curve_follow_a)))
    s += v * DT
  return planner, rows


def _spy_errors(monkeypatch):
  lines = []
  monkeypatch.setattr(lp.cloudlog, "error", lambda msg, *a, **k: lines.append(msg))
  return lines


@pytest.mark.parametrize("lead", [None, (45.0, 14.0)])
def test_shadow_commands_are_bit_identical_to_off(lead):
  _, off = _drive(0, 20.0, BEND, 160, lead=lead)
  _, shadow = _drive(1, 20.0, BEND, 160, lead=lead)
  assert [r[0] for r in off] == [r[0] for r in shadow]
  assert [r[1] for r in off] == [r[1] for r in shadow]  # old preview untouched
  assert all(r[2] == PREVIEW_FREE_A_MS2 for r in off)  # off does not compute
  assert min(r[2] for r in shadow) < -0.2  # shadow does compute


def test_active_uses_the_curve_follow_ceiling():
  planner, rows = _drive(2, 20.0, BEND, 160)
  assert all(r[1] == r[2] for r in rows)
  assert min(r[1] for r in rows) < -0.2
  assert planner._cf_trusted


def test_active_can_only_lower_the_command():
  _, plain = _drive(0, 20.0, STRAIGHT, 120, mpc_accel=0.8)
  _, active = _drive(2, 20.0, BEND, 120, mpc_accel=0.8)
  assert all(a[0] <= p[0] + 1e-9 for a, p in zip(active, plain, strict=True))


@pytest.mark.parametrize("lead", [None, (30.0, 10.0)])
def test_active_on_a_straight_road_is_bit_identical_to_off(lead):
  """No bend, no change: the lead law (including braking for a lead) is untouched."""
  _, off = _drive(0, 20.0, STRAIGHT, 160, lead=lead, mpc_accel=-0.5)
  _, active = _drive(2, 20.0, STRAIGHT, 160, lead=lead, mpc_accel=-0.5)
  assert [r[0] for r in off] == [r[0] for r in active]


def test_lead_plus_bend_is_the_stricter_of_the_two():
  lead = (40.0, 20.0)
  _, lead_only = _drive(2, 20.0, STRAIGHT, 160, lead=lead)
  _, both = _drive(2, 20.0, BEND, 160, lead=lead)
  _, bend_only = _drive(2, 20.0, BEND, 160)
  assert all(b[0] <= l[0] + 1e-9 for b, l in zip(both, lead_only, strict=True))
  assert any(b[0] < l[0] - 0.1 for b, l in zip(both, lead_only, strict=True))
  # The curve-follow term itself does not depend on the lead.
  assert [r[2] for r in both] == [r[2] for r in bend_only]


def test_off_mode_emits_no_log_lines(monkeypatch):
  lines = _spy_errors(monkeypatch)
  _drive(0, 20.0, BEND, 100)
  assert not [m for m in lines if str(m).startswith("curvefollow ")]


@pytest.mark.parametrize("mode", [1, 2])
def test_log_lines_come_at_2hz_with_the_agreed_fields(monkeypatch, mode):
  lines = _spy_errors(monkeypatch)
  _drive(mode, 20.0, BEND, 100)
  cf = [m for m in lines if str(m).startswith("curvefollow ")]
  assert 9 <= len(cf) <= 11
  rec = json.loads(cf[-1][len("curvefollow "):])
  for key in ("cf", "cfr", "w", "li", "lt", "ld", "lv", "nf", "old", "at", "g", "pk", "tr", "mx"):
    assert key in rec
  assert any('"T"' in m for m in cf)  # full arrays on the relevant stretch


def test_a_logging_exception_cannot_touch_the_command(monkeypatch):
  def boom(*a, **k):
    raise RuntimeError("log sink down")

  _, ok = _drive(2, 20.0, BEND, 100)
  monkeypatch.setattr(lp, "format_log_line", boom)
  planner, bad = _drive(2, 20.0, BEND, 100)
  assert [r[0] for r in ok] == [r[0] for r in bad]
  assert not planner._cf_faulted


def test_a_fault_latches_and_the_old_preview_stays(monkeypatch):
  def boom(self, **kw):
    raise RuntimeError("bad")

  _, old = _drive(0, 20.0, BEND, 100)
  monkeypatch.setattr(lp.CurveFollow, "step", boom)
  planner, rows = _drive(2, 20.0, BEND, 100)
  assert planner._cf_faulted
  assert [r[0] for r in rows] == [r[0] for r in old]


def _ceiling_seen(monkeypatch, mode, break_path=False):
  seen = []
  real = lp.unified_follow_desired

  def spy(*a, **k):
    seen.append(k.get("v_ceiling"))
    return real(*a, **k)

  monkeypatch.setattr(lp, "unified_follow_desired", spy)
  planner, inputs, params = _planner(20.0)
  params.curve_follow = mode
  _own_lead(inputs, 60.0, 20.0)
  inputs["controlsState"].curvature = 1.0 / 30.0
  s = 0.0
  for _ in range(40):
    _set_model_path(inputs["modelV2"], 20.0, s, (1e6, 1.0, 0.0))
    if break_path:
      inputs["modelV2"].position.x = []
    planner.update(inputs)
    s += 1.0
  return planner, [c for c in seen if c is not None]


def test_reactive_cap_stays_unless_active_and_trusted(monkeypatch):
  for mode in (0, 1):
    _, seen = _ceiling_seen(monkeypatch, mode)
    assert seen and min(seen) < 15.0, mode  # present curvature caps the lead law's ceiling
  planner, seen = _ceiling_seen(monkeypatch, 2)
  assert planner._cf_trusted
  assert seen and min(seen) > 15.0  # curve-follow owns the curve; no reactive cap


def test_dead_camera_keeps_the_reactive_cap_in_active(monkeypatch):
  planner, seen = _ceiling_seen(monkeypatch, 2, break_path=True)
  assert not planner._cf_trusted
  assert seen and min(seen) < 15.0


def _long_on(inputs):
  cs = inputs["carState"]
  cs.enableLongControl = True
  cs.cruiseState.enabled = True


def _arm_gap_lock(planner, inputs, gap_m, track=1):
  _long_on(inputs)
  # The planner rereads NAPGapLock every frame from params (unknown keys
  # stay off on the test double). Keep the param true so the latch is not
  # cleared mid-drive, same as MAX being skipped while the lock is on.
  params = planner._params
  if not getattr(params, "_gap_lock_param", False):
    orig = params.get_bool
    def get_bool(key):
      if key == "NAPGapLock":
        return True
      return orig(key)
    params.get_bool = get_bool
    params._gap_lock_param = True
  planner._gap_lock_enabled = True
  planner._gap_lock.gap_m = float(gap_m)
  planner._gap_lock.track_id = int(track)


def _not_softer(both, only, slack=1e-6):
  assert all(b[0] <= o[0] + slack for b, o in zip(both, only, strict=True))


def test_lead_appearing_mid_curve_is_not_delayed_or_softened():
  """A closer, slower lead mid-curve takes the command immediately; the curve event must not delay or soften it."""
  appear = 80

  def hook(k, planner, inputs):
    if k >= appear:
      _own_lead(inputs, 25.0, 8.0, a_lead=-1.0)

  _, both = _drive(2, 20.0, BEND, 160, hook=hook)
  _, lead_only = _drive(2, 20.0, STRAIGHT, 160, hook=hook)
  _not_softer(both[appear:], lead_only[appear:])
  # Within ~1 s the command is as firm as lead-only (no event-slew delay).
  later = slice(appear + 8, appear + 24)
  assert min(r[0] for r in both[later]) <= min(r[0] for r in lead_only[later]) + 0.15
  assert min(r[0] for r in both[later]) < -0.5


def test_slower_lead_in_a_curve_is_not_softened():
  lead = (25.0, 8.0)
  _, lead_only = _drive(2, 20.0, STRAIGHT, 160, lead=lead)
  _, both = _drive(2, 20.0, BEND, 160, lead=lead)
  _not_softer(both, lead_only)
  # The lead is clearly the stricter term: the command matches lead-only,
  # the curve may only cap further, never lift it.
  assert min(r[0] for r in both) <= min(r[0] for r in lead_only) + 1e-6
  assert min(r[0] for r in lead_only) < -0.5


def test_lead_braking_in_a_curve_is_not_delayed_or_softened():
  brake = 60

  def hook(k, planner, inputs):
    if k < brake:
      _own_lead(inputs, 35.0, 20.0)
    else:
      _own_lead(inputs, 32.0, 12.0, a_lead=-1.5)

  _, both = _drive(2, 20.0, BEND, 160, hook=hook)
  _, lead_only = _drive(2, 20.0, STRAIGHT, 160, hook=hook)
  _not_softer(both[brake:], lead_only[brake:])
  later = slice(brake + 8, brake + 24)
  assert min(r[0] for r in both[later]) <= min(r[0] for r in lead_only[later]) + 0.15
  assert min(r[0] for r in both[later]) < min(r[0] for r in both[:brake]) - 0.2


def test_lead_pulling_away_on_exit_does_not_exceed_follow():
  """Exit re-accel stays within what lead follow allows."""
  pull = 200  # past the 120 m + 60 m bend at 20 m/s

  def hook(k, planner, inputs):
    if k < pull:
      _own_lead(inputs, 35.0, 20.0)
    else:
      _own_lead(inputs, 45.0, 28.0, a_lead=0.8)

  _, both = _drive(2, 20.0, BEND, 260, hook=hook)
  _, lead_only = _drive(2, 20.0, STRAIGHT, 260, hook=hook)
  _not_softer(both[pull:], lead_only[pull:])
  # May still be curve-capped, but never above the follow command.
  assert max(r[0] for r in both[pull:]) <= max(r[0] for r in lead_only[pull:]) + 1e-6


def test_gap_lock_through_a_curve_is_a_ceiling_only():
  """Gap Lock holding a gap on a lead wins; the curve term may only cap as a ceiling."""

  def hook(k, planner, inputs):
    _arm_gap_lock(planner, inputs, 30.0)
    _own_lead(inputs, 30.0, 20.0, track=1)

  planner, both = _drive(2, 20.0, BEND, 160, hook=hook)
  _, lock_only = _drive(2, 20.0, STRAIGHT, 160, hook=hook)
  assert planner._gap_lock.gap_m == 30.0
  _not_softer(both, lock_only)
  # The curve may still cap (stricter than a locked same-speed hold), but
  # the follow command is never lifted.
  assert any(b[0] < l[0] - 0.05 for b, l in zip(both, lock_only, strict=True)) or min(r[1] for r in both) < -0.05


def test_gap_lock_braking_in_a_curve_is_not_delayed_or_softened():
  """A locked lead that slows in the bend is followed immediately; the curve event must not fight the hold."""

  def hook(k, planner, inputs):
    _arm_gap_lock(planner, inputs, 30.0)
    if k < 60:
      _own_lead(inputs, 30.0, 20.0, track=1)
    else:
      _own_lead(inputs, 28.0, 10.0, a_lead=-1.5, track=1)

  planner, both = _drive(2, 20.0, BEND, 160, hook=hook)
  _, lock_only = _drive(2, 20.0, STRAIGHT, 160, hook=hook)
  assert planner._gap_lock.gap_m == 30.0
  _not_softer(both[60:], lock_only[60:])
  later = slice(68, 84)
  assert min(r[0] for r in both[later]) <= min(r[0] for r in lock_only[later]) + 0.15
  assert min(r[0] for r in both[later]) < -0.5


def test_gap_lock_exit_reaccel_stays_within_the_hold():
  pull = 200

  def hook(k, planner, inputs):
    _arm_gap_lock(planner, inputs, 30.0)
    if k < pull:
      _own_lead(inputs, 30.0, 20.0, track=1)
    else:
      _own_lead(inputs, 40.0, 28.0, a_lead=0.6, track=1)

  planner, both = _drive(2, 20.0, BEND, 260, hook=hook)
  _, lock_only = _drive(2, 20.0, STRAIGHT, 260, hook=hook)
  assert planner._gap_lock.gap_m == 30.0
  _not_softer(both[pull:], lock_only[pull:])
  assert max(r[0] for r in both[pull:]) <= max(r[0] for r in lock_only[pull:]) + 1e-6


def test_lead_and_gap_lock_do_not_receive_the_curve_as_a_map(monkeypatch):
  """Follow is computed without the curve in a_map / v_curve_cap; the curve is applied after as a ceiling."""
  seen = []
  real = lp.unified_follow_desired

  def spy(*a, **k):
    seen.append((k.get("a_map"), k.get("v_curve_cap"), k.get("gap_set_override_m")))
    return real(*a, **k)

  monkeypatch.setattr(lp, "unified_follow_desired", spy)

  def hook(k, planner, inputs):
    _arm_gap_lock(planner, inputs, 30.0)
    _own_lead(inputs, 30.0, 20.0, track=1)

  _drive(2, 20.0, BEND, 80, hook=hook)
  locked = [s for s in seen if s[2] is not None]
  assert locked
  assert all(s[2] == pytest.approx(30.0) for s in locked)
  assert all(s[0] is None for s in locked)
  assert all(s[1] is None for s in locked)

