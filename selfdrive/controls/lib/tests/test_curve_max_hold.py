"""Curve MAX snapshot/restore: temp slow through the bend, then pre-curve set."""
from pathlib import Path

import pytest
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.curve_max_hold import (
  CURVE_CAP_SETTLE_S,
  CURVE_COMFORT_LAT_MS2,
  CURVE_ENTER_LAT_MS2,
  CURVE_EXIT_HOLD_S,
  CURVE_LAT_TAU_S,
  CURVE_MIN_V_EGO_MS,
  CURVE_RESTORE_A_DEFAULT_MS2,
  CurveMaxHold,
  cornering_lat_accel_ms2,
  curve_speed_from_lat,
  curve_speed_ms,
  is_sharp_curve,
  steer_lat_accel_ms2,
)
from openpilot.selfdrive.mapd.constants import MODE_FOLLOW
from openpilot.selfdrive.mapd.map_speed_policy import MapCruiseHold, decide_map_cruise

# Tesla Model S Pre-AP (same as planner tests).
SR = 15.75
WB = 2.959
V_60 = 60.0 * CV.MPH_TO_MS
V_55 = 55.0 * CV.MPH_TO_MS


def _kph(mph: float) -> float:
  return mph * CV.MPH_TO_KPH


def _sharp_steer_deg(v_ms: float = V_60, a_y: float = 2.4) -> float:
  """Steer that produces about a_y at v (above enter, well above exit)."""
  return (a_y * SR * WB) / (max(v_ms, 1e-3) ** 2) * CV.RAD_TO_DEG


def _follow_seed(hold: MapCruiseHold, posted_kph: float, sticky_kph: float | None = None):
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=posted_kph, posted_kph=posted_kph,
    engage_rising=True, now=0.0,
  )
  if sticky_kph is not None:
    decide_map_cruise(
      hold, engaged=True, mode=MODE_FOLLOW, raw_kph=sticky_kph, posted_kph=posted_kph,
      engage_rising=False, now=1.0, stalk_pressed=True,
    )


def test_lat_accel_matches_planner_formula():
  a_y = steer_lat_accel_ms2(V_60, 10.0, SR, WB)
  expect = (V_60 ** 2) * 10.0 * CV.DEG_TO_RAD / (SR * WB)
  assert abs(a_y - abs(expect)) < 1e-9


def test_straight_is_not_a_curve():
  assert not is_sharp_curve(V_60, 2.0, SR, WB, active=False)
  assert curve_speed_ms(V_60, 0.0, SR, WB) is None
  assert not is_sharp_curve(CURVE_MIN_V_EGO_MS - 0.5, 40.0, SR, WB, active=False)


def test_sharp_steer_is_a_curve_and_caps_below_60():
  steer = _sharp_steer_deg()
  assert is_sharp_curve(V_60, steer, SR, WB, active=False)
  v_cap = curve_speed_ms(V_60, steer, SR, WB)
  assert v_cap is not None
  assert v_cap < V_60
  # Comfort a_lat: v = sqrt(a / kappa)
  kappa = abs(steer) * CV.DEG_TO_RAD / (SR * WB)
  assert abs(v_cap - (CURVE_COMFORT_LAT_MS2 / kappa) ** 0.5) < 1e-6


def _run_bend(curve: CurveMaxHold, hold: MapCruiseHold, *, last_hud, posted_now,
              steer, v_ego=V_60, dt=0.01, hud_overlay=None, stalk=False, take=False,
              curvature=None, yaw_rate=None, restore_a_ms2=1.0e4):
  policy_posted, freeze = curve.begin_cycle(
    hold, last_hud_kph=last_hud, posted_kph=posted_now,
    v_ego_ms=v_ego, angle_steers_deg=steer, steer_ratio=SR, wheelbase=WB,
    engaged=True, take_speed_now=take, dt=dt,
    curvature=curvature, yaw_rate=yaw_rate,
  )
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW,
    raw_kph=hold.held_max_kph or last_hud,
    posted_kph=policy_posted,
    engage_rising=False, now=2.0, stalk_pressed=stalk, take_speed_now=take,
  )
  overlay = hud_overlay if hud_overlay is not None else dec.driver_kph
  out = curve.finish(
    hud_kph=overlay, hold=hold, posted_kph=posted_now,
    v_ego_ms=v_ego, angle_steers_deg=steer, steer_ratio=SR, wheelbase=WB,
    engaged=True, stalk_pressed=stalk, take_speed_now=take, dt=dt,
    curvature=curvature, yaw_rate=yaw_rate, restore_a_ms2=restore_a_ms2,
  )
  return out, freeze, dec


def test_justin_hypermile_curve_restores_60_not_eco_55():
  """Sticky/display MAX 60 must survive a bend even if posted looks lower.

  Through a sharp curve OSM may flicker (or HUD is temporarily capped). After
  exit MAX must be 60, not a lower posted/eco target the flicker rebase
  would invent. (Live Hypermile eco is posted-scaled; 55 here is a lower
  posted stand-in, not a claim that posted 60 still snaps −5.)
  """
  posted_eco = _kph(55)  # lower posted / eco-like target vs sticky 60
  sticky_60 = _kph(60)
  hold = MapCruiseHold()
  _follow_seed(hold, posted_eco, sticky_60)
  assert abs(hold.held_max_kph - sticky_60) < 1e-6
  assert hold.sticky_set_kph is not None

  curve = CurveMaxHold()
  steer = _sharp_steer_deg()
  flicker = _kph(25)

  out, freeze, dec = _run_bend(
    curve, hold, last_hud=sticky_60, posted_now=flicker, steer=steer,
  )
  assert freeze
  assert curve.active
  assert curve.snapshot is not None
  assert abs(curve.snapshot.hud_max_kph - sticky_60) < 1e-6
  # Flicker must not wipe sticky / rebase held to 25.
  assert dec.sticky
  assert abs(hold.held_max_kph - sticky_60) < 1e-6
  assert abs(hold.sticky_set_kph - sticky_60) < 1e-6
  # Temporary MAX may drop for the corner. Must not write that into hold.
  assert out.hud_kph < sticky_60
  assert out.restore_seed_kph is None
  assert abs(hold.held_max_kph - sticky_60) < 1e-6

  # Still in the bend a few frames (flicker then eco posted again).
  for _ in range(8):
    out, _, _ = _run_bend(
      curve, hold, last_hud=out.hud_kph, posted_now=posted_eco, steer=steer,
    )
    assert curve.active
    assert abs(hold.held_max_kph - sticky_60) < 1e-6

  # Straight-ish: hold exit debounce, then restore 60.
  restored = None
  for _ in range(int(CURVE_EXIT_HOLD_S / 0.01) + 3):
    out, _, _ = _run_bend(
      curve, hold, last_hud=out.hud_kph, posted_now=posted_eco, steer=1.5,
    )
    if out.restore_seed_kph is not None:
      restored = out
      break
  assert restored is not None
  assert abs(restored.hud_kph - sticky_60) < 1e-6
  assert abs(restored.restore_seed_kph - sticky_60) < 1e-6
  assert abs(hold.held_max_kph - sticky_60) < 1e-6
  assert abs(hold.sticky_set_kph - sticky_60) < 1e-6
  assert not curve.active
  assert curve.snapshot is None


def test_eco_follow_without_sticky_restores_55():
  """Steady Hypermile Follow (no stalk hold) snapshots 55 and returns 55."""
  posted_eco = _kph(55)
  hold = MapCruiseHold()
  _follow_seed(hold, posted_eco, None)
  assert abs(hold.held_max_kph - posted_eco) < 1e-6
  assert hold.sticky_set_kph is None

  curve = CurveMaxHold()
  steer = _sharp_steer_deg()
  out, _, _ = _run_bend(
    curve, hold, last_hud=posted_eco, posted_now=posted_eco, steer=steer,
    hud_overlay=_kph(40),
  )
  assert out.hud_kph <= posted_eco
  assert abs(hold.held_max_kph - posted_eco) < 1e-6

  restored = None
  hud = out.hud_kph
  for _ in range(int(CURVE_EXIT_HOLD_S / 0.01) + 3):
    out, _, _ = _run_bend(
      curve, hold, last_hud=hud, posted_now=posted_eco, steer=1.5,
      hud_overlay=posted_eco,
    )
    hud = out.hud_kph
    if out.restore_seed_kph is not None:
      restored = out
      break
  assert restored is not None
  assert abs(restored.hud_kph - posted_eco) < 1e-6


def test_real_posted_change_through_curve_does_not_restore_old_max():
  """60→45 zone that is still 45 after the bend is a real rebase, not flicker."""
  posted_a = _kph(55)
  posted_b = _kph(40)
  sticky_60 = _kph(60)
  hold = MapCruiseHold()
  _follow_seed(hold, posted_a, sticky_60)
  curve = CurveMaxHold()
  steer = _sharp_steer_deg()
  _run_bend(curve, hold, last_hud=sticky_60, posted_now=posted_a, steer=steer)
  assert curve.snapshot is not None
  assert abs(curve.snapshot.posted_kph - posted_a) < 1.0

  out = None
  for _ in range(int(CURVE_EXIT_HOLD_S / 0.01) + 3):
    out, _, _ = _run_bend(
      curve, hold, last_hud=sticky_60, posted_now=posted_b, steer=1.5,
      hud_overlay=posted_b,
    )
  assert out is not None
  assert out.restore_seed_kph is None
  assert abs(out.hud_kph - posted_b) < 1e-6
  assert not curve.active


def test_stalk_during_curve_updates_snapshot():
  posted = _kph(55)
  sticky_60 = _kph(60)
  new_set = _kph(50)
  hold = MapCruiseHold()
  _follow_seed(hold, posted, sticky_60)
  curve = CurveMaxHold()
  steer = _sharp_steer_deg()
  _run_bend(curve, hold, last_hud=sticky_60, posted_now=posted, steer=steer)
  out, _, _ = _run_bend(
    curve, hold, last_hud=sticky_60, posted_now=posted, steer=steer,
    hud_overlay=new_set, stalk=True,
  )
  assert curve.snapshot is not None
  assert abs(curve.snapshot.hud_max_kph - new_set) < 1e-6

  restored = None
  for _ in range(int(CURVE_EXIT_HOLD_S / 0.01) + 3):
    out, _, _ = _run_bend(
      curve, hold, last_hud=new_set, posted_now=posted, steer=1.5,
      hud_overlay=new_set,
    )
    if out.restore_seed_kph is not None:
      restored = out
      break
  assert restored is not None
  assert abs(restored.hud_kph - new_set) < 1e-6


def test_disengage_clears_curve_snapshot():
  hold = MapCruiseHold()
  _follow_seed(hold, _kph(55), _kph(60))
  curve = CurveMaxHold()
  steer = _sharp_steer_deg()
  _run_bend(curve, hold, last_hud=_kph(60), posted_now=_kph(55), steer=steer)
  assert curve.active
  curve.finish(
    hud_kph=_kph(40), hold=hold, posted_kph=_kph(55),
    v_ego_ms=V_60, angle_steers_deg=steer, steer_ratio=SR, wheelbase=WB,
    engaged=False, dt=0.01,
  )
  assert not curve.active
  assert curve.snapshot is None


def test_take_speed_now_clears_curve_snapshot():
  hold = MapCruiseHold()
  _follow_seed(hold, _kph(55), _kph(60))
  curve = CurveMaxHold()
  steer = _sharp_steer_deg()
  _run_bend(curve, hold, last_hud=_kph(60), posted_now=_kph(55), steer=steer)
  out, freeze, _ = _run_bend(
    curve, hold, last_hud=_kph(60), posted_now=_kph(55), steer=steer, take=True,
  )
  assert not freeze
  assert not curve.active
  assert out.restore_seed_kph is None


def test_temp_cap_does_not_raise_max():
  """Curve speed above the set must not lift HUD MAX."""
  hold = MapCruiseHold()
  posted = _kph(40)
  _follow_seed(hold, posted, None)
  curve = CurveMaxHold()
  # Mild steer that still enters via angle at this speed, but v_curve > 40 mph.
  steer = 14.5
  assert is_sharp_curve(V_55, steer, SR, WB, active=False)
  v_cap = curve_speed_ms(V_55, steer, SR, WB)
  assert v_cap is not None
  out, _, _ = _run_bend(
    curve, hold, last_hud=posted, posted_now=posted, steer=steer,
    v_ego=V_55, hud_overlay=posted,
  )
  assert out.hud_kph <= posted + 1e-6


def test_card_and_docs_wire_curve_max_restore():
  root = Path(__file__).resolve().parents[4]
  card = (root / "selfdrive/car/card.py").read_text()
  planner = (root / "selfdrive/controls/lib/longitudinal_planner.py").read_text()
  docs = (root / "docs-nap/map-speed.md").read_text()
  hyper = (root / "docs-nap/hypermile.md").read_text()
  releases = (root / "RELEASES.md").read_text()
  assert "CurveMaxHold" in card
  assert "begin_cycle" in card
  assert "restore_seed_kph" in card
  assert "limit_accel_in_turns" in planner
  assert "curve" in docs.lower() and "MAX" in docs
  assert "pre-curve" in docs.lower() or "before the curve" in docs.lower()
  assert "curve" in hyper.lower()
  hm = next(p for p in releases.split("\n\n") if "curve" in p.lower() and p.startswith("NAP"))
  assert "MAX" in hm


def test_card_curve_curvature_comes_from_car_control():
  """#239's 100 Hz controlsState socket does not belong on card.

  Vehicle-model curvature is already on carControl.currentCurvature.
  livePose remains the yaw-rate fallback.
  """
  root = Path(__file__).resolve().parents[4]
  card = (root / "selfdrive/car/card.py").read_text()
  sm = card.split("messaging.SubMaster([", 1)[1].split("])", 1)[0]
  assert "controlsState" not in sm
  assert "currentCurvature" in card
  assert "livePose" in sm


def test_curvature_lat_accel_not_raw_steer():
  """Route 115: steer-model a_y read ~1.84× the real corner.

  A sweeper whose vehicle-model curvature is 1.4 m/s² at 70 mph must
  not drop MAX. The same wheel angle through the stock steer model
  (ratio 15, no angle offset, no understeer) would.
  """
  v = 70.0 * CV.MPH_TO_MS
  real_ay = 1.4
  kappa = real_ay / (v * v)
  assert cornering_lat_accel_ms2(v, 7.0, SR, WB, curvature=kappa) == pytest.approx(real_ay)
  assert cornering_lat_accel_ms2(
    v, 20.0, SR, WB, yaw_rate=real_ay / v,
  ) == pytest.approx(real_ay)
  steer_ay = steer_lat_accel_ms2(v, 7.0, 15.0, WB)
  assert steer_ay > real_ay * 1.5
  assert curve_speed_from_lat(v, real_ay) > v
  assert curve_speed_from_lat(v, steer_ay) < v

  posted = _kph(70)
  hold = MapCruiseHold()
  _follow_seed(hold, posted, None)
  curve = CurveMaxHold()
  out, _, _ = _run_bend(
    curve, hold, last_hud=posted, posted_now=posted, steer=7.0,
    v_ego=v, hud_overlay=posted, curvature=kappa,
  )
  assert out.hud_kph == pytest.approx(posted, abs=0.05)
  # Same steer, no curvature: the inflated model does cap.
  steer_only = curve_speed_ms(v, 7.0, 15.0, WB)
  assert steer_only is not None and steer_only * CV.MS_TO_KPH < posted


def test_lat_accel_smooths_and_cap_holds_then_ramps():
  """Cap tracks a smoothed a_y, does not pump up mid-bend, ramps out.

  First sample matches the raw corner so entry is immediate. After the
  settle window a looser reading must not raise the cap; a clearly
  tighter one may drop it. Exit restores MAX at the Accel rate, not
  in one frame, and does not rebase sticky.
  """
  v = V_60
  posted = _kph(60)
  hold = MapCruiseHold()
  _follow_seed(hold, posted, posted)
  curve = CurveMaxHold()
  # a_y 2.4 → comfort speed below 60. kappa = a_y / v².
  kappa_tight = 2.4 / (v * v)
  out, _, _ = _run_bend(
    curve, hold, last_hud=posted, posted_now=posted, steer=4.0,
    v_ego=v, hud_overlay=posted, curvature=kappa_tight, restore_a_ms2=0.40,
  )
  assert curve._ay_s == pytest.approx(2.4, abs=1e-6)
  assert out.hud_kph < posted - 1.0
  capped = out.hud_kph
  # One noisy spike must not jump the smoothed a_y to the spike.
  kappa_spike = 4.0 / (v * v)
  _run_bend(
    curve, hold, last_hud=capped, posted_now=posted, steer=4.0,
    v_ego=v, hud_overlay=posted, curvature=kappa_spike, restore_a_ms2=0.40,
  )
  alpha = 0.01 / (CURVE_LAT_TAU_S + 0.01)
  assert curve._ay_s < 2.4 + alpha * (4.0 - 2.4) + 0.05
  assert curve._ay_s < 3.0

  # Settle, then a looser bend must not raise the cap.
  for _ in range(int(CURVE_CAP_SETTLE_S / 0.01) + 5):
    out, _, _ = _run_bend(
      curve, hold, last_hud=posted, posted_now=posted, steer=4.0,
      v_ego=v, hud_overlay=posted, curvature=kappa_tight, restore_a_ms2=0.40,
    )
  held = out.hud_kph
  kappa_loose = 1.6 / (v * v)
  assert kappa_loose * v * v >= CURVE_ENTER_LAT_MS2
  for _ in range(100):
    out, _, _ = _run_bend(
      curve, hold, last_hud=posted, posted_now=posted, steer=4.0,
      v_ego=v, hud_overlay=posted, curvature=kappa_loose, restore_a_ms2=0.40,
    )
    assert out.hud_kph <= held + 0.05
  assert abs(hold.held_max_kph - posted) < 1e-6

  # Clearly tighter: cap may drop.
  kappa_hard = 3.2 / (v * v)
  dropped = held
  for _ in range(int(CURVE_CAP_SETTLE_S / 0.01) + 8):
    out, _, _ = _run_bend(
      curve, hold, last_hud=posted, posted_now=posted, steer=4.0,
      v_ego=v, hud_overlay=posted, curvature=kappa_hard, restore_a_ms2=0.40,
    )
    dropped = min(dropped, out.hud_kph)
  assert dropped < held - 1.0

  # Exit ramps toward the snapshot. Sticky stays put until the ramp ends.
  saw_partial = False
  first_rise = None
  restored = None
  prev = out.hud_kph
  for _ in range(int((CURVE_EXIT_HOLD_S + 30.0) / 0.01)):
    out, _, _ = _run_bend(
      curve, hold, last_hud=posted, posted_now=posted, steer=1.0,
      v_ego=v, hud_overlay=posted, curvature=0.0, restore_a_ms2=CURVE_RESTORE_A_DEFAULT_MS2,
    )
    if out.restore_seed_kph is None and out.hud_kph > prev + 1e-4:
      saw_partial = True
      if first_rise is None:
        first_rise = out.hud_kph
      assert abs(hold.sticky_set_kph - posted) < 1e-6
    prev = out.hud_kph
    if out.restore_seed_kph is not None:
      restored = out
      break
  assert saw_partial and first_rise is not None
  assert first_rise < posted - 1.0
  assert restored is not None
  assert restored.hud_kph == pytest.approx(posted, abs=0.05)
  assert abs(hold.held_max_kph - posted) < 1e-6
  assert not curve.active
