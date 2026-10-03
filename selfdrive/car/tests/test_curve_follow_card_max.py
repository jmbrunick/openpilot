"""NAPCurveFollow and the HUD MAX (Pre-AP card).

Shadow (1) and off (0) leave CurveMaxHold exactly as it was. Active (2) never
touches MAX: no cap, no snapshot, no restore.
"""
import json

import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.car import card as card_mod
from openpilot.selfdrive.car.tests.test_preap_engage_max_after_pause import _harness, _run

MPH = CV.MPH_TO_KPH


def _curve(mode, seconds_in=2.0):
  h, cs, md, eng = _harness(70.0)
  h._curve_follow_mode = mode
  _run(h, cs, 2.0)
  lowest = h.v_cruise_helper.v_cruise_kph
  _run(h, cs, seconds_in, 40.0, 40.0, steer=lambda _x: 30.0)
  lowest = min(lowest, h.v_cruise_helper.v_cruise_kph)
  return h, cs, lowest


@pytest.mark.parametrize("mode", [0, 1])
def test_off_and_shadow_still_cap_max_in_a_bend(mode):
  h, cs, _ = _curve(mode)
  assert h.v_cruise_helper.v_cruise_kph < 60.0 * MPH
  _run(h, cs, 40.0, 40.0, 40.0, steer=lambda _x: 0.0)
  assert h.v_cruise_helper.v_cruise_kph == pytest.approx(70.0 * MPH, abs=0.3)


def test_active_never_touches_max():
  h, cs, _ = _curve(2, seconds_in=4.0)
  assert h.v_cruise_helper.v_cruise_kph == pytest.approx(70.0 * MPH, abs=0.3)
  assert h._curve_max.cap_kph is None
  _run(h, cs, 10.0, 40.0, 40.0, steer=lambda _x: 0.0)
  assert h.v_cruise_helper.v_cruise_kph == pytest.approx(70.0 * MPH, abs=0.3)


def test_active_max_is_bit_identical_to_a_straight_drive():
  a, cs_a, _ = _curve(2, seconds_in=4.0)
  b, cs_b, _md, _eng = _harness(70.0)
  b._curve_follow_mode = 2
  _run(b, cs_b, 2.0)
  _run(b, cs_b, 4.0, 40.0, 40.0, steer=lambda _x: 0.0)
  assert a.v_cruise_helper.v_cruise_kph == b.v_cruise_helper.v_cruise_kph


def test_flipping_to_active_mid_bend_gives_max_back():
  h, cs, _ = _curve(1, seconds_in=3.0)
  assert h.v_cruise_helper.v_cruise_kph < 60.0 * MPH
  h._curve_follow_mode = 2
  _run(h, cs, 10.0, 40.0, 40.0, steer=lambda _x: 30.0)
  assert h.v_cruise_helper.v_cruise_kph == pytest.approx(70.0 * MPH, abs=0.3)


def test_curvemax_log_line_is_2hz_and_only_while_the_cap_is_in_play(monkeypatch):
  lines = []
  monkeypatch.setattr(card_mod.cloudlog, "error", lambda msg, *a, **k: lines.append(msg))
  _curve(1, seconds_in=4.0)
  cm = [m for m in lines if str(m).startswith("curvemax ")]
  assert cm
  rec = json.loads(cm[-1][len("curvemax "):])
  for key in ("m", "act", "cap", "hud_in", "hud", "seed", "frz", "posted", "v"):
    assert key in rec
  assert rec["m"] == 1
  lines.clear()
  _curve(2, seconds_in=4.0)
  assert not [m for m in lines if str(m).startswith("curvemax ")]
