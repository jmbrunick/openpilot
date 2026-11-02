"""F3: the far-gap kinematic gate widens while the PD term accelerates.

At a far gap with a slow close, `gate = smooth(v_close / 0.40)` flipped the
command between the PD accel (+0.2..+0.3) and a near-zero kinematic bound once
v_close crossed ~0.4 m/s: the accCmd pulses at 15a 11:27 and 159 08:03. The
widened gate keeps the PD accel through that edge. It only acts for slack past
15 m, an accelerating PD (a_pd > 0.4 m/s^2 to clear the base width) and a
kinematic bound that is not a real brake (>= -0.10), is never firmer than
before, and is bit-identical elsewhere.
"""
import math

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import unified_lead as ul
from openpilot.selfdrive.controls.lib.unified_lead import (
  KIN_GATE_MS,
  KIN_GATE_WIDEN_A_BOUND_LO,
  KIN_GATE_WIDEN_A_BOUND_RAMP,
  KIN_GATE_WIDEN_A_CAP,
  KIN_GATE_WIDEN_K,
  KIN_GATE_WIDEN_SLACK_LO_M,
  KIN_GATE_WIDEN_SLACK_RAMP_M,
  _kin_gate_widen,
  gap_set_m,
  unified_follow_desired,
)

TF = 1.5


def _baseline(monkeypatch):
  monkeypatch.setattr(ul, "_kin_gate_widen", lambda *a, **k: 0.0)


def _cmd(slack, v_close, vl=25.0, a_lead=0.0):
  gap = gap_set_m(vl, TF) + slack
  return unified_follow_desired(gap, vl + v_close, vl, a_lead, TF)


def test_constants_are_pinned():
  assert KIN_GATE_MS == 0.40
  assert KIN_GATE_WIDEN_SLACK_LO_M == 15.0
  assert KIN_GATE_WIDEN_SLACK_RAMP_M == 5.0
  assert KIN_GATE_WIDEN_A_BOUND_LO == -0.10
  assert KIN_GATE_WIDEN_A_BOUND_RAMP == 0.10
  assert KIN_GATE_WIDEN_A_CAP == 0.50
  assert KIN_GATE_WIDEN_K == 1.0


def test_widen_is_zero_unless_all_three_conditions_hold():
  assert _kin_gate_widen(30.0, 0.7, 0.0) > 0.0
  assert _kin_gate_widen(30.0, 0.0, 0.0) == 0.0     # PD not accelerating
  assert _kin_gate_widen(30.0, -0.5, 0.0) == 0.0
  assert _kin_gate_widen(30.0, 0.4, 0.0) == 0.0     # does not clear the base width
  assert _kin_gate_widen(15.0, 0.7, 0.0) == 0.0     # slack under the far-gap line
  assert _kin_gate_widen(5.0, 0.7, 0.0) == 0.0
  assert _kin_gate_widen(30.0, 0.7, -0.10) == 0.0   # a real bound keeps the old gate
  assert _kin_gate_widen(30.0, 0.7, -1.0) == 0.0


def test_widen_full_weight_and_cap():
  full = _kin_gate_widen(20.0, 0.9, 0.0)
  assert full == pytest.approx(KIN_GATE_WIDEN_A_CAP / KIN_GATE_WIDEN_K - KIN_GATE_MS)
  assert _kin_gate_widen(60.0, 5.0, 0.5) == pytest.approx(full)  # a_pd capped at 0.5
  assert _kin_gate_widen(25.0, 0.45, 0.0) == pytest.approx(0.45 - KIN_GATE_MS)


def test_widen_is_continuous_in_slack_bound_and_pd():
  sweeps = (
    (np.linspace(0.0, 40.0, 40001), lambda x: _kin_gate_widen(x, 0.7, 0.0)),
    (np.linspace(-0.3, 1.0, 13001), lambda x: _kin_gate_widen(30.0, 0.7, x)),
    (np.linspace(-0.5, 1.5, 20001), lambda x: _kin_gate_widen(30.0, x, 0.0)),
  )
  for xs, fn in sweeps:
    ys = np.array([fn(float(x)) for x in xs])
    assert np.abs(np.diff(ys)).max() < 0.01


def test_bit_identical_to_the_old_gate_inside_the_far_gap_line(monkeypatch):
  rng = np.random.default_rng(3)
  cases = []
  for _ in range(4000):
    vl = rng.uniform(0, 35)
    slack = rng.uniform(-8, KIN_GATE_WIDEN_SLACK_LO_M)
    cases.append((vl, vl + rng.uniform(-1.5, 2.5), rng.uniform(-1.5, 1.0), TF + rng.choice([-0.5, 0, 0.5]), slack))
  new = [unified_follow_desired(max(0.0, gap_set_m(vl, tf) + s), ve, vl, al, tf) for vl, ve, al, tf, s in cases]
  _baseline(monkeypatch)
  old = [unified_follow_desired(max(0.0, gap_set_m(vl, tf) + s), ve, vl, al, tf) for vl, ve, al, tf, s in cases]
  assert new == old


def test_never_firmer_and_unchanged_where_the_old_command_braked(monkeypatch):
  rng = np.random.default_rng(11)
  cases = []
  for _ in range(20000):
    vl = rng.uniform(0, 35)
    tf = float(rng.choice([0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9]))
    slack = float(rng.choice([rng.uniform(-5, 40), rng.uniform(15, 35)]))
    cases.append((max(0.0, gap_set_m(vl, tf) + slack), max(0.0, vl + rng.uniform(-1.5, 1.5)), vl,
                  rng.uniform(-1.5, 1.0), tf))
  new = [unified_follow_desired(*c) for c in cases]
  _baseline(monkeypatch)
  old = [unified_follow_desired(*c) for c in cases]
  changed = 0
  for n, o, c in zip(new, old, cases, strict=True):
    assert n >= o - 1e-12, c                      # never firmer
    if o < KIN_GATE_WIDEN_A_BOUND_LO - 1e-9:
      assert n == o, c                            # a real brake is untouched
    if n != o:
      changed += 1
      assert n - o < 0.45, c                      # bounded raise
  assert changed > 100


def test_far_gap_slow_close_keeps_the_pd_accel_through_the_old_gate_edge(monkeypatch):
  """A 0.4 m/s close at the old gate edge used to drop to ~0.

  The soft closing-speed law retires the large PD term past ~30 m, so this
  exemplar sits at 22 m, where the PD term still clears the base width.
  The widened gate keeps a small accel; the old gate does not.
  """
  new = _cmd(22.0, 0.4)
  _baseline(monkeypatch)
  old = _cmd(22.0, 0.4)
  assert old == pytest.approx(0.0, abs=0.01)
  assert new > 0.015
  # A faster close is back on the kinematic bound.
  assert _cmd(22.0, 0.8) == pytest.approx(0.0, abs=0.05)
  assert _cmd(22.0, 1.2) == pytest.approx(0.0, abs=0.1)


def test_close_sweep_has_a_smaller_step_than_the_old_gate(monkeypatch):
  """The pulse driver is the gate edge: the command steps less per 10 mm/s of closing speed."""
  xs = np.arange(-0.2, 1.6, 0.01)
  new = np.array([_cmd(22.0, float(x)) for x in xs])
  _baseline(monkeypatch)
  old = np.array([_cmd(22.0, float(x)) for x in xs])
  assert np.abs(np.diff(new)).max() < 0.90 * np.abs(np.diff(old)).max()
  assert np.all(new >= old - 1e-12)


def test_inside_the_follow_gap_and_real_closes_match_the_old_command(monkeypatch):
  for slack, v_close in ((-4.0, 1.0), (3.0, 1.5), (10.0, 2.0), (14.9, 0.6), (40.0, 4.0), (60.0, 6.0)):
    new = _cmd(slack, v_close, a_lead=-0.5)
    _baseline(monkeypatch)
    old = _cmd(slack, v_close, a_lead=-0.5)
    monkeypatch.undo()
    assert new == old, (slack, v_close)
    assert math.isfinite(new)
