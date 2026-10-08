"""Guard: lateral handoff behavior and the lateral-yield inference must agree.

Neither panda nor the card is ever told "lateral is yielded". Both infer it
from the 0x488 DAS_steeringControlType=1 send history (opendbc
car/tesla/preap/lat_yield.py, safety/modes/tesla_preap_latyield.h) and assume
how openpilot hands lateral back and forth: type 1 flows only while OP
steers, a yield lasts at least HANDS_OFF_CONFIRM_S, and the take-back blend
lasts BLEND_TIME_S.

ANY change to lateral transition / handoff behavior (driver_lateral_handoff,
controlsd latActive, blinker / lane-change / roundabout lateral pause, a new
release or re-arm path, authority shaping) MUST update the inference. This
file fails when they diverge:

  1. constants   - handoff timing vs the inference's ASSUMED_* and windows
  2. pins        - handoff constants frozen; a failing pin tells the editor
                   to update the inference first, then re-pin
  3. markers     - INFERENCE CONTRACT comment present at every touch point
  4. behavior    - the real DriverLateralHandoff + lat_active_after_handoff
                   + 50 Hz stamp drives a LatYieldTracker; partial authority
                   must never read as full control, a settled yield and a
                   settled return must
  5. self-check  - the behavioral check does fail for a divergent handoff
"""
import re
from pathlib import Path

import pytest

from opendbc.car.tesla.preap import lat_yield as ly
from opendbc.car.tesla.preap.lat_yield import LatYieldTracker
from openpilot.selfdrive.controls.lib import driver_lateral_handoff as dlh
from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  DriverLateralHandoff,
  lat_active_after_handoff,
)

REPO = Path(__file__).resolve().parents[4]
HEADER = REPO / "opendbc_repo/opendbc/safety/modes/tesla_preap_latyield.h"
CARCONTROLLER = REPO / "opendbc_repo/opendbc/car/tesla/carcontroller.py"
LAT_YIELD_PY = REPO / "opendbc_repo/opendbc/car/tesla/preap/lat_yield.py"
HANDOFF_PY = REPO / "selfdrive/controls/lib/driver_lateral_handoff.py"

UPDATE_FIRST = " ".join((
  "The lateral handoff changed. Panda and the card infer 'lateral yielded' from the",
  "0x488 type-1 history (opendbc preap/lat_yield.py + safety/modes/tesla_preap_latyield.h).",
  "Update that inference (windows, ASSUMED_*, panda header, their tests) FIRST, then",
  "update this pin. Do not just edit the pin.",
))


def _header_us(name: str) -> int:
  m = re.search(rf"#define\s+{name}\s+(\d+)U?\b", HEADER.read_text())
  assert m, f"{name} missing from {HEADER}"
  return int(m.group(1))


# -- 1. constants ----------------------------------------------------------

def test_handoff_timing_matches_what_the_inference_assumes():
  assert dlh.BLEND_TIME_S == ly.ASSUMED_BLEND_TIME_S, UPDATE_FIRST
  assert dlh.HANDS_OFF_CONFIRM_S == ly.ASSUMED_HANDS_OFF_CONFIRM_S, UPDATE_FIRST
  # 0x488 goes out every second 100 Hz control frame
  assert 2 * dlh.DT_CTRL == pytest.approx(ly.ASSUMED_FRAME_PERIOD_S), UPDATE_FIRST
  assert dlh.YIELD_AUTHORITY_TIME_S == 0.0, UPDATE_FIRST
  assert dlh.QUIET_WAIT_S == 0.0, UPDATE_FIRST


def test_grace_covers_the_blend_and_gap_sits_below_the_shortest_yield():
  # A hand-grab anywhere in the take-back blend must still read as yielded.
  assert ly.GRACE_S >= dlh.BLEND_TIME_S - 1e-9, UPDATE_FIRST
  assert ly.PANDA_GRACE_S >= ly.GRACE_S + 0.1, UPDATE_FIRST
  assert ly.PANDA_ASSUMED_BLEND_S == dlh.BLEND_TIME_S, UPDATE_FIRST
  # The shortest yield is the hands-off confirm (+ one frame); the lapse
  # detector must be shorter, or the grace never starts after a short yield.
  assert ly.GAP_S < dlh.HANDS_OFF_CONFIRM_S + 2 * dlh.DT_CTRL - 0.02, UPDATE_FIRST
  assert ly.PANDA_GAP_S <= ly.GAP_S
  # Automatic give-back is RELEASE_HOLD_S, longer than the inference floor.
  assert dlh.RELEASE_HOLD_S + 1e-12 >= 0.30
  assert dlh.RELEASE_HOLD_S + 1e-12 >= dlh.HANDS_OFF_CONFIRM_S
  assert ly.GAP_S < dlh.RELEASE_HOLD_S


def test_panda_header_matches_python_windows():
  assert _header_us("PREAP_LAT_RECENT_US") / 1e6 == pytest.approx(ly.PANDA_RECENT_S)
  assert _header_us("PREAP_LAT_GAP_US") / 1e6 == pytest.approx(ly.PANDA_GAP_S)
  assert _header_us("PREAP_LAT_GRACE_US") / 1e6 == pytest.approx(ly.PANDA_GRACE_S)
  assert _header_us("PREAP_LAT_BLOCK_CLEAR_US") / 1e6 == pytest.approx(ly.PANDA_BLOCK_CLEAR_S)
  assert _header_us("PREAP_LAT_ASSUMED_BLEND_US") / 1e6 == pytest.approx(dlh.BLEND_TIME_S), UPDATE_FIRST


# -- 2. pins ---------------------------------------------------------------

def test_handoff_constants_are_pinned():
  pins = {
    "BLEND_TIME_S": 1.0,
    "HANDS_OFF_CONFIRM_S": 0.15,
    "QUIET_WAIT_S": 0.0,
    "YIELD_AUTHORITY_TIME_S": 0.0,
    "UI_LATERAL_RETURN_AUTHORITY": 0.70,
    "HANDS_ON_HOLD_LEVEL": 1,
    "SOFT_YIELD_DEBOUNCE_FRAMES": 9,
    "SOFT_YIELD_FAST_DEBOUNCE_FRAMES": 6,
    "SOFT_YIELD_RELEASE_FRAMES": 8,
    "EARLY_YIELD_NM": 2.0,
    "EARLY_YIELD_FRAMES": 8,
    "LAT_REENABLE_MIN_V_EGO_MPH": 10.0,
  }
  for name, want in pins.items():
    assert getattr(dlh, name) == want, f"dlh.{name} changed ({getattr(dlh, name)} != {want}). {UPDATE_FIRST}"


def test_inference_windows_are_pinned():
  pins = {"RECENT_S": 0.15, "GAP_S": 0.14, "GRACE_S": 1.0, "GRACE_DELAY_S": 0.05,
          "BLOCK_CLEAR_S": 0.20, "PANDA_RECENT_S": 0.1, "PANDA_GAP_S": 0.1,
          "PANDA_GRACE_S": 1.2, "PANDA_BLOCK_CLEAR_S": 0.15}
  for name, want in pins.items():
    assert getattr(ly, name) == want, " ".join((
      f"lat_yield.{name} changed ({getattr(ly, name)} != {want}). Re-check it against the",
      "handoff timing, tesla_preap_latyield.h and the panda/card ordering contract, then re-pin.",
    ))


# -- 3. markers ------------------------------------------------------------

@pytest.mark.parametrize("path", [HEADER, CARCONTROLLER, LAT_YIELD_PY, HANDOFF_PY])
def test_inference_contract_comment_present(path):
  text = path.read_text()
  if path == HANDOFF_PY:
    text = dlh.__doc__ or ""  # the module docstring, not just any comment
  assert "INFERENCE CONTRACT" in text, f"{path.name}: restore the INFERENCE CONTRACT note"
  assert "test_lat_yield_inference_guard" in path.read_text() or path == CARCONTROLLER, path.name


def test_lat_active_after_handoff_documents_the_contract():
  assert "INFERENCE CONTRACT" in (dlh.lat_active_after_handoff.__doc__ or "")


# -- 4. behavior -----------------------------------------------------------

DT = dlh.DT_CTRL  # 100 Hz control loop, 0x488 stamped on every other tick


class _Clock:
  t = 1000.0

  def __call__(self):
    return self.t


def simulate(script, handoff=None):
  """Drive handoff -> latActive -> 0x488 stamps -> LatYieldTracker.

  script: list of (seconds, torque_nm, hands_level, steering_pressed).
  Returns a list of per-tick records.
  """
  clk = _Clock()
  tracker = LatYieldTracker(clock=clk)
  h = handoff or DriverLateralHandoff(enabled=True)
  rec = []
  tick = 0
  for seconds, torque, hands, pressed in script:
    for _ in range(int(round(seconds / DT))):
      clk.t += DT
      out = h.update(engaged=True, lat_would_be_active=True, steering_torque=torque,
                     steering_rate_deg=0.0, hands_on_level=hands, v_ego=20.0,
                     steering_pressed=pressed)
      lat_active = lat_active_after_handoff(True, out.yielded)
      if tick % 2 == 0:  # 50 Hz DAS_steeringControl
        tracker.note_steer_tx(lat_active, engaged=True)
      tick += 1
      rec.append((clk.t, out.authority, out.yielded, out.blending, lat_active, tracker.full_control()))
  return rec


def violations(rec):
  bad = []
  last_lat_change = rec[0][0]
  prev_lat = rec[0][4]
  for t, authority, _yielded, _blending, lat_active, full in rec:
    if lat_active != prev_lat:
      last_lat_change = t
      prev_lat = lat_active
    # Partial authority (the take-back blend) must never read as full control.
    if 0.02 < authority < 0.995 and lat_active and full:
      bad.append((round(t, 3), "blend reads as full control", round(authority, 3)))
    # Once the type-1 stream has been quiet for RECENT_S the yield must read as yielded.
    if not lat_active and (t - last_lat_change) > ly.RECENT_S + 2 * DT and full:
      bad.append((round(t, 3), "settled yield reads as full control"))
    # A settled return (past the blend) is full control again.
    if lat_active and authority >= 1.0 and (t - last_lat_change) > dlh.BLEND_TIME_S + ly.GRACE_S and not full:
      bad.append((round(t, 3), "settled return does not read as full control"))
  return bad


STEADY = (0.5, 0.0, 0, False)
SCRIPTS = {
  "hands-gated push then release": [STEADY, (0.30, 3.0, 1, True), (2.5, 0.0, 0, False)],
  "early push, hands never seen": [STEADY, (0.30, 3.0, 0, True), (2.5, 0.0, 0, False)],
  "shortest yield": [STEADY, (0.10, 3.0, 1, True), (2.5, 0.0, 0, False)],
  "long dodge": [STEADY, (1.50, 2.0, 1, True), (2.5, 0.0, 0, False)],
  "regrab mid-blend then release": [STEADY, (0.30, 3.0, 1, True), (0.45, 0.0, 0, False),
                                    (0.30, 1.2, 1, True), (2.5, 0.0, 0, False)],
  "two quick yields": [STEADY, (0.20, 3.0, 1, True), (0.30, 0.0, 0, False),
                       (0.20, 3.0, 1, True), (2.5, 0.0, 0, False)],
}


@pytest.mark.parametrize("name", sorted(SCRIPTS))
def test_blend_and_yield_read_correctly_through_the_real_handoff(name):
  rec = simulate(SCRIPTS[name])
  assert any(r[2] for r in rec), "script never yielded: scenario is not testing anything"
  assert not violations(rec), (name, violations(rec)[:5], UPDATE_FIRST)


def test_a_steady_session_with_no_yield_is_full_control():
  rec = simulate([(2.0, 0.0, 0, False)])
  assert rec[-1][5] and not any(r[2] or r[3] for r in rec)


# -- 5. self-check: the behavioral guard fails for a divergent handoff -----

def test_guard_catches_a_longer_blend(monkeypatch):
  monkeypatch.setattr(dlh, "BLEND_TIME_S", 1.6)
  rec = simulate(SCRIPTS["hands-gated push then release"], DriverLateralHandoff(enabled=True))
  assert violations(rec), "a 1.6 s blend outlasts the 1.0 s grace and must be flagged"


def test_guard_catches_a_shorter_hands_off_confirm(monkeypatch):
  # Release shorter than GAP_S: the lapse is not seen as a yield, so no grace.
  monkeypatch.setattr(dlh, "RELEASE_HOLD_S", 0.05)
  rec = simulate(SCRIPTS["shortest yield"], DriverLateralHandoff(enabled=True))
  assert violations(rec), "a 0.05 s release hides the yield from the inference and must be flagged"
