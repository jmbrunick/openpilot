from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import (
  ACCEL_DEFAULT, DECREASE_START_MARGIN_M, LATE_APEX_CURV_SCALE, LATE_APEX_Y_M,
  LOOKAHEAD_EARLY, LOOKAHEAD_NORMAL, LOOKAHEAD_OFF,
  LOOKAHEAD_TUNING, MODE_CAP, MODE_DISPLAY, MODE_FOLLOW, MODE_OFF, OSM_SIGN_LEAD_S,
  TRACK_DEADBAND_MS, TRACK_TAPER_MS,
  accel_scale_factor, map_accel_a_ms2, map_brake_a_ms2, map_comfort_a_ms2,
)
from openpilot.selfdrive.mapd.map_speed_policy import (
  SOURCE_CRUISE, SOURCE_LEAD0, V_CRUISE_UNSET,
  MapCruiseHold, anticipatory_limit_ms, apply_late_apex_curvature, apply_map_speed_kph,
  blinker_turn_direction, blinker_turn_holds_alc, blinker_turn_limit_ms,
  cap_planner_v_cruise_ms, decide_map_cruise, effective_map_limit_ms,
  hud_with_blinker_turn_kph, is_cruise_stalk_step, late_apex_ramp, late_apex_y_offset_m,
  longitudinal_obstacle_source, map_in_track_deadband, map_slew_a_ms2,
  map_track_accel_ms2, map_track_decel_ms2, should_write_preap_pedal, slew_map_speed_ms,
  turn_speed_ms,
)
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  MAP_SPEED_ACCEL, MAP_SPEED_ACCEL_DEFAULT, MAP_SPEED_LOOKAHEAD,
)

# Keep in sync with long_mpc (avoid importing cereal/acados here).
_COMFORT_BRAKE = 2.5
_STOP_DISTANCE = 6.0
_T_FOLLOW = 1.45  # LongitudinalPersonality.standard


def _cruise_obstacle_m(v_cruise_ms: float) -> float:
  return (v_cruise_ms ** 2) / (2 * _COMFORT_BRAKE) + _T_FOLLOW * v_cruise_ms + _STOP_DISTANCE


def _lead_obstacle_m(d_rel: float, v_lead: float) -> float:
  return d_rel + (v_lead ** 2) / (2 * _COMFORT_BRAKE)


def test_off_and_display_never_change_driver_set():
  for mode in (MODE_OFF, MODE_DISPLAY):
    assert apply_map_speed_kph(100, 70, mode=mode, engaged=True, op_long_software_cruise=True) == 100


def test_cap_lowers_but_does_not_raise():
  assert apply_map_speed_kph(100, 70, mode=MODE_CAP, engaged=True, op_long_software_cruise=True) == 70
  assert apply_map_speed_kph(50, 70, mode=MODE_CAP, engaged=True, op_long_software_cruise=True) == 50


def test_follow_tracks_map_unless_override():
  assert apply_map_speed_kph(100, 70, mode=MODE_FOLLOW, engaged=True, op_long_software_cruise=True) == 70
  assert apply_map_speed_kph(50, 70, mode=MODE_FOLLOW, engaged=True, op_long_software_cruise=True) == 70
  assert apply_map_speed_kph(100, 70, mode=MODE_FOLLOW, engaged=True, op_long_software_cruise=True,
                             driver_override=True) == 100


def test_follow_raises_when_offset_posted_is_already_higher():
  """GNSS lag: posted at v*1.5 s is already 45 → Follow MAX 45. Far-ahead next does not."""
  a = 25 * CV.MPH_TO_KPH
  b = 45 * CV.MPH_TO_KPH
  assert apply_map_speed_kph(a, b, mode=MODE_FOLLOW, engaged=True, op_long_software_cruise=True) == b
  hold = MapCruiseHold()
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=b,
    engage_rising=False, now=1.0, stalk_pressed=False,
  )
  assert not dec.sticky
  assert dec.seed_kph is not None
  assert abs(dec.seed_kph - b) < 1e-6
  assert abs(_follow_hud(dec, b) - b) < 1e-6
  # Lookahead path is still decrease-only (not this 1.5 s offset).
  v25 = 25 * CV.MPH_TO_MS
  v45 = 45 * CV.MPH_TO_MS
  assert anticipatory_limit_ms(v25, v45, 80.0, v25, LOOKAHEAD_EARLY) is None
  assert effective_map_limit_ms(v25, v45, 80.0, v25, LOOKAHEAD_NORMAL) == v25


def test_pcm_cruise_never_gets_control_overlay():
  # No-pedal stock CC: do not invent a parallel set-speed path
  assert apply_map_speed_kph(100, 70, mode=MODE_CAP, engaged=True, op_long_software_cruise=False) == 100
  assert apply_map_speed_kph(100, 70, mode=MODE_FOLLOW, engaged=True, op_long_software_cruise=False) == 100


def test_disengaged_or_unset_passthrough():
  assert apply_map_speed_kph(100, 70, mode=MODE_CAP, engaged=False, op_long_software_cruise=True) == 100
  assert apply_map_speed_kph(V_CRUISE_UNSET, 70, mode=MODE_CAP, engaged=True, op_long_software_cruise=True) == V_CRUISE_UNSET
  assert apply_map_speed_kph(100, None, mode=MODE_CAP, engaged=True, op_long_software_cruise=True) == 100


def test_offset_applies_to_map_target():
  offset = 5 * CV.MPH_TO_KPH
  out = apply_map_speed_kph(120, 70, mode=MODE_CAP, offset_kph=offset, engaged=True, op_long_software_cruise=True)
  assert abs(out - (70 + offset)) < 0.05


def test_planner_cap_is_down_only():
  lim = 25.0  # m/s
  # Trust HUD: min() with posted snapped Cap/Follow when GPS entered a lower zone.
  assert cap_planner_v_cruise_ms(30.0, lim, mode=MODE_CAP) == 30.0
  assert cap_planner_v_cruise_ms(20.0, lim, mode=MODE_CAP) == 20.0
  assert cap_planner_v_cruise_ms(30.0, lim, mode=MODE_DISPLAY) == 30.0
  assert cap_planner_v_cruise_ms(30.0, None, mode=MODE_FOLLOW) == 30.0


def test_follow_planner_trusts_hud_not_posted():
  """Sticky 66 with posted 60 must not clip MPC v_cruise to 60. Cap eases the same."""
  hud = 66 * CV.MPH_TO_MS
  posted = 60 * CV.MPH_TO_MS
  assert cap_planner_v_cruise_ms(hud, posted, mode=MODE_FOLLOW) == hud
  below = 55 * CV.MPH_TO_MS
  assert cap_planner_v_cruise_ms(below, posted, mode=MODE_FOLLOW) == below
  assert cap_planner_v_cruise_ms(hud, posted, mode=MODE_CAP) == hud


def test_no_lead_tracks_map_capped_cruise():
  """No radar lead: fake far/fast lead, so cruise (map ceiling) is the tightest obstacle."""
  v_ego = 31.29  # ~70 mph
  v_cruise = cap_planner_v_cruise_ms(31.29, 31.29, mode=MODE_CAP)
  assert abs(v_cruise - 31.29) < 1e-6
  cruise = _cruise_obstacle_m(v_cruise)
  fake = _lead_obstacle_m(50.0, v_ego + 10.0)
  assert longitudinal_obstacle_source(fake, fake, cruise) == SOURCE_CRUISE


def test_slower_lead_still_commands_below_map_ceiling():
  """Close slower lead must win min(lead, cruise) regardless of map v_cruise."""
  lead = _lead_obstacle_m(40.0, 15.0)
  for v_cruise in (20.0, 31.29, 40.0):
    capped = cap_planner_v_cruise_ms(v_cruise, 31.29, mode=MODE_CAP)
    cruise = _cruise_obstacle_m(capped)
    assert longitudinal_obstacle_source(lead, 1e8, cruise) == SOURCE_LEAD0
    assert min(lead, cruise) == lead


def test_upcoming_lower_limit_lowers_target_early():
  current = 70 * CV.MPH_TO_MS
  nxt = 45 * CV.MPH_TO_MS
  # Well inside Normal horizon (400 m) and braking window.
  early = anticipatory_limit_ms(current, nxt, 200.0, current, LOOKAHEAD_NORMAL)
  assert early is not None
  assert nxt - 1e-6 <= early < current
  # Closer to the sign → closer to the new limit (smooth profile).
  closer = anticipatory_limit_ms(current, nxt, 60.0, current, LOOKAHEAD_NORMAL)
  assert closer is not None and closer <= early + 1e-9
  assert closer < current
  eff = effective_map_limit_ms(current, nxt, 200.0, current, LOOKAHEAD_NORMAL)
  assert eff is not None and eff < current
  # HUD/planner use this as the map ceiling (Cap never raises).
  assert apply_map_speed_kph(
    120, eff * CV.MS_TO_KPH, mode=MODE_CAP, engaged=True, op_long_software_cruise=True,
  ) < 70 * CV.MPH_TO_KPH + 0.1


def test_upcoming_higher_limit_does_not_raise_early():
  current = 45 * CV.MPH_TO_MS
  nxt = 70 * CV.MPH_TO_MS
  assert anticipatory_limit_ms(current, nxt, 80.0, current, LOOKAHEAD_EARLY) is None
  assert effective_map_limit_ms(current, nxt, 80.0, current, LOOKAHEAD_NORMAL) == current
  # Follow still uses the *current* match (raises only once GPS is on the faster way).
  assert apply_map_speed_kph(
    50, current * CV.MS_TO_KPH, mode=MODE_FOLLOW, engaged=True, op_long_software_cruise=True,
  ) == current * CV.MS_TO_KPH


def test_lookahead_off_and_far_away_keep_current():
  current = 70 * CV.MPH_TO_MS
  nxt = 35 * CV.MPH_TO_MS
  assert anticipatory_limit_ms(current, nxt, 80.0, current, LOOKAHEAD_OFF) is None
  assert effective_map_limit_ms(current, nxt, 80.0, current, LOOKAHEAD_OFF) == current
  # Beyond Normal horizon (600 m) — do not start yet.
  assert anticipatory_limit_ms(current, nxt, 620.0, current, LOOKAHEAD_NORMAL) is None
  # Display never changes MAX even if we computed an anticipatory ceiling.
  assert apply_map_speed_kph(
    100, 45, mode=MODE_DISPLAY, engaged=True, op_long_software_cruise=True,
  ) == 100


def test_anticipatory_cap_still_loses_to_slower_lead():
  current = 70 * CV.MPH_TO_MS
  nxt = 45 * CV.MPH_TO_MS
  eff = effective_map_limit_ms(current, nxt, 150.0, current, LOOKAHEAD_NORMAL)
  assert eff is not None
  # Planner trusts HUD (already eased); min() with posted would snap at the zone.
  v_cruise = cap_planner_v_cruise_ms(eff, current, mode=MODE_CAP)
  assert v_cruise == eff
  assert v_cruise < current
  lead = _lead_obstacle_m(40.0, 15.0)
  cruise = _cruise_obstacle_m(v_cruise)
  assert longitudinal_obstacle_source(lead, 1e8, cruise) == SOURCE_LEAD0
  assert min(lead, cruise) == lead


def test_accel_default_five_matches_prior_normal_curve():
  assert ACCEL_DEFAULT == 5
  assert MAP_SPEED_ACCEL_DEFAULT == 5
  assert MAP_SPEED_ACCEL == list(range(1, 11))
  assert MAP_SPEED_LOOKAHEAD == [0, 1, 2, 3]
  assert abs(accel_scale_factor(5) - 1.0) < 1e-9
  assert abs(map_comfort_a_ms2(LOOKAHEAD_NORMAL, 5) - 0.80) < 1e-9
  assert abs(map_comfort_a_ms2(LOOKAHEAD_NORMAL, 1) - 0.36) < 1e-9
  assert abs(map_comfort_a_ms2(LOOKAHEAD_NORMAL, 10) - 1.60) < 1e-9
  current = 70 * CV.MPH_TO_MS
  nxt = 45 * CV.MPH_TO_MS
  a5 = anticipatory_limit_ms(current, nxt, 200.0, current, LOOKAHEAD_NORMAL, 5)
  a_default = anticipatory_limit_ms(current, nxt, 200.0, current, LOOKAHEAD_NORMAL)
  assert a5 is not None and a_default is not None
  assert abs(a5 - a_default) < 1e-9
  # Accel 1–10 must not change anticipatory decreases (brake locked at 5).
  a1 = anticipatory_limit_ms(current, nxt, 200.0, current, LOOKAHEAD_NORMAL, 1)
  a10 = anticipatory_limit_ms(current, nxt, 200.0, current, LOOKAHEAD_NORMAL, 10)
  assert a1 is not None and a10 is not None
  assert abs(a1 - a5) < 1e-9 and abs(a10 - a5) < 1e-9
  assert anticipatory_limit_ms(current, nxt, 300.0, current, LOOKAHEAD_NORMAL, 1) is not None
  assert anticipatory_limit_ms(current, nxt, 300.0, current, LOOKAHEAD_NORMAL, 10) is not None


def test_fifty_to_thirty_starts_one_hundred_ten_m_before_kinematic():
  """Justin: 50→30 still ~42 mph at the sign. Keep brake 0.80; start kin+110 m earlier."""
  v50 = 50 * CV.MPH_TO_MS
  v30 = 30 * CV.MPH_TO_MS
  assert abs(v50 - 22.352) < 0.002
  assert abs(18.776 - 42 * CV.MPH_TO_MS) < 0.002
  assert abs(v30 - 13.411) < 0.002
  a = map_brake_a_ms2(LOOKAHEAD_NORMAL)
  assert abs(a - 0.80) < 1e-9
  assert abs(DECREASE_START_MARGIN_M - 110.0) < 1e-9
  assert abs(OSM_SIGN_LEAD_S - 1.5) < 1e-9
  # Sign lead is time*speed, not an extra meter constant on decreases.
  assert a < 1.5  # Tesla pre-AP clip
  for la, tun in LOOKAHEAD_TUNING.items():
    if tun[0] <= 0:
      continue
    assert tun[0] < 1.5
    extra = DECREASE_START_MARGIN_M + (120.0 if la == LOOKAHEAD_EARLY else 0.0)
    assert abs(tun[1] - extra) < 1e-9
  kin_m = (v50 * v50 - v30 * v30) / (2.0 * a)
  assert abs(kin_m - 200.0) < 1.0
  leftover_m = (18.776 * 18.776 - v30 * v30) / (2.0 * 0.80)
  assert abs(leftover_m - 108.0) < 2.0
  # Window opens at kin+110; MAX is still 50 on that edge, then falls toward 30.
  at_open = anticipatory_limit_ms(v50, v30, kin_m + DECREASE_START_MARGIN_M, v50, LOOKAHEAD_NORMAL)
  assert at_open is not None
  assert abs(at_open - v50) < 0.6
  assert anticipatory_limit_ms(v50, v30, kin_m + DECREASE_START_MARGIN_M + 5.0, v50, LOOKAHEAD_NORMAL) is None
  at_kin = anticipatory_limit_ms(v50, v30, kin_m, v50, LOOKAHEAD_NORMAL)
  assert at_kin is not None
  assert at_kin < v50 - 1.0
  near_sign = anticipatory_limit_ms(v50, v30, 5.0, v50, LOOKAHEAD_NORMAL)
  assert near_sign is not None
  assert abs(near_sign - v30) < 1.5
  # Same extra start margin on any decrease — not a 50→30-only window.
  v70 = 70 * CV.MPH_TO_MS
  v45 = 45 * CV.MPH_TO_MS
  kin_7045 = (v70 * v70 - v45 * v45) / (2.0 * a)
  assert anticipatory_limit_ms(v70, v45, kin_7045 + DECREASE_START_MARGIN_M, v70, LOOKAHEAD_NORMAL) is not None
  assert anticipatory_limit_ms(v70, v45, kin_7045 + DECREASE_START_MARGIN_M + 5.0, v70, LOOKAHEAD_NORMAL) is None
  # Higher nextSpeedLimit far ahead must not raise MAX (lookahead is decrease-only).
  # The 1.5 s GNSS offset raising posted is tested separately.
  assert anticipatory_limit_ms(v30, v50, 80.0, v30, LOOKAHEAD_EARLY) is None
  assert anticipatory_limit_ms(v45, v70, 80.0, v45, LOOKAHEAD_EARLY) is None


def test_sixty_to_fifty_uses_kin_plus_110_not_min_decrease_skip():
  """10 mph 60→50 must open at kin+110 m. MIN_DECREASE is ~1 mph, not 10."""
  v60 = 60 * CV.MPH_TO_MS
  v50 = 50 * CV.MPH_TO_MS
  assert (v60 - v50) > 4.0
  a = map_brake_a_ms2(LOOKAHEAD_NORMAL)
  assert abs(a - 0.80) < 1e-9
  kin_m = (v60 * v60 - v50 * v50) / (2.0 * a)
  assert abs(kin_m - 137.0) < 2.0
  assert anticipatory_limit_ms(v60, v50, kin_m + DECREASE_START_MARGIN_M, v60, LOOKAHEAD_NORMAL) is not None
  assert anticipatory_limit_ms(v60, v50, kin_m + DECREASE_START_MARGIN_M + 5.0, v60, LOOKAHEAD_NORMAL) is None


def test_sixty_to_fifty_eases_with_lookahead_not_posted_cliff():
  """10 mph 60→50 must interpolate MAX, not seed-snap to 50."""
  v60 = 60 * CV.MPH_TO_MS
  v50 = 50 * CV.MPH_TO_MS
  a = map_brake_a_ms2(LOOKAHEAD_NORMAL)
  kin_m = (v60 * v60 - v50 * v50) / (2.0 * a)
  at_open = anticipatory_limit_ms(v60, v50, kin_m + DECREASE_START_MARGIN_M, v60, LOOKAHEAD_NORMAL)
  assert at_open is not None
  assert abs(at_open - v60) < 0.6
  mid = anticipatory_limit_ms(v60, v50, 80.0, v60, LOOKAHEAD_NORMAL)
  assert mid is not None
  assert v50 < mid < v60
  at_zone = anticipatory_limit_ms(v60, v50, 0.0, v60, LOOKAHEAD_NORMAL)
  assert at_zone is not None and abs(at_zone - v50) < 1e-6
  hold = MapCruiseHold()
  a_kph = 60 * CV.MPH_TO_KPH
  b_kph = 50 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a_kph, posted_kph=a_kph,
    engage_rising=True, now=0.0,
  )
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a_kph, posted_kph=b_kph,
    engage_rising=False, now=1.0, stalk_pressed=False,
  )
  assert not dec.sticky
  assert dec.seed_kph is None
  raise_dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=b_kph, posted_kph=a_kph,
    engage_rising=False, now=2.0, stalk_pressed=False,
  )
  assert raise_dec.seed_kph is not None
  assert abs(raise_dec.seed_kph - a_kph) < 1e-6


def test_all_posted_decreases_ease_like_fifty_to_thirty():
  """10/15/20 mph and 50→30: kin+110 m interpolate; Cap/Follow must not seed-snap."""
  a = map_brake_a_ms2(LOOKAHEAD_NORMAL)
  assert abs(a - 0.80) < 1e-9
  for hi_mph, lo_mph in ((50, 30), (60, 50), (60, 45), (60, 40)):
    v_hi = hi_mph * CV.MPH_TO_MS
    v_lo = lo_mph * CV.MPH_TO_MS
    kin_m = (v_hi * v_hi - v_lo * v_lo) / (2.0 * a)
    at_open = anticipatory_limit_ms(v_hi, v_lo, kin_m + DECREASE_START_MARGIN_M, v_hi, LOOKAHEAD_NORMAL)
    assert at_open is not None, (hi_mph, lo_mph)
    assert abs(at_open - v_hi) < 0.6
    mid = anticipatory_limit_ms(v_hi, v_lo, max(25.0, 0.35 * kin_m), v_hi, LOOKAHEAD_NORMAL)
    assert mid is not None and v_lo < mid < v_hi, (hi_mph, lo_mph, mid)
    hi_kph, lo_kph = hi_mph * CV.MPH_TO_KPH, lo_mph * CV.MPH_TO_KPH
    for mode in (MODE_FOLLOW, MODE_CAP):
      hold = MapCruiseHold()
      decide_map_cruise(
        hold, engaged=True, mode=mode, raw_kph=hi_kph, posted_kph=hi_kph,
        engage_rising=True, now=0.0,
      )
      dec = decide_map_cruise(
        hold, engaged=True, mode=mode, raw_kph=hi_kph, posted_kph=lo_kph,
        engage_rising=False, now=1.0, stalk_pressed=False,
      )
      assert not dec.sticky
      assert dec.seed_kph is None
      # Driver stays previous so Cap min(driver, eased map) cannot cliff to lo.
      assert abs(dec.driver_kph - hi_kph) < 1e-6
      eased = (v_hi + v_lo) * 0.5 * CV.MS_TO_KPH
      cap_out = apply_map_speed_kph(
        dec.driver_kph, eased, mode=MODE_CAP, engaged=True, op_long_software_cruise=True,
      )
      assert abs(cap_out - eased) < 1e-6


def test_sticky_skips_anticipatory_lookahead():
  """Upcoming 50 must not lower the ceiling while a sticky set is active."""
  current = 60 * CV.MPH_TO_MS
  nxt = 50 * CV.MPH_TO_MS
  a = map_brake_a_ms2(LOOKAHEAD_NORMAL)
  kin_m = (current * current - nxt * nxt) / (2.0 * a)
  dist = kin_m  # well inside the kin+110 m window
  lowered = effective_map_limit_ms(current, nxt, dist, current, LOOKAHEAD_NORMAL)
  assert lowered is not None and lowered < current
  held = effective_map_limit_ms(current, nxt, dist, current, LOOKAHEAD_NORMAL, sticky=True)
  assert held == current
  # map_track_decel tracks HUD, not nextSpeedLimit. At the sticky set, hold.
  sticky = 66 * CV.MPH_TO_MS
  assert map_track_decel_ms2(sticky, sticky, a) is None
  assert map_track_accel_ms2(sticky, sticky, a) is None
  assert map_in_track_deadband(sticky, sticky)
  # Ego at sticky MAX: do not brake toward upcoming 50 just because OSM is lower.
  assert map_track_decel_ms2(sticky, sticky, a) is None
  # After posted changes, lookahead resumes (sticky=False).
  assert effective_map_limit_ms(current, nxt, dist, current, LOOKAHEAD_NORMAL, sticky=False) == lowered


def test_hold_deadband_does_not_climb_or_brake():
  """Inside the band: no Accel climb and no map_track_decel (lead still wins)."""
  v_set = 66 * CV.MPH_TO_MS
  a5 = map_brake_a_ms2(LOOKAHEAD_NORMAL)
  a10 = map_accel_a_ms2(LOOKAHEAD_NORMAL, 10)
  assert map_in_track_deadband(v_set, v_set)
  assert map_in_track_deadband(v_set + TRACK_DEADBAND_MS, v_set)
  assert map_in_track_deadband(v_set - TRACK_DEADBAND_MS, v_set)
  assert not map_in_track_deadband(v_set + TRACK_DEADBAND_MS + 0.01, v_set)
  assert map_track_accel_ms2(v_set, v_set, a10) is None
  assert map_track_decel_ms2(v_set, v_set, a5) is None
  # Below the band: climb at Accel 1–10. Above: brake at locked Accel 5.
  below = v_set - TRACK_TAPER_MS
  assert map_track_accel_ms2(below, v_set, a10) == a10
  assert map_track_decel_ms2(below, v_set, a5) is None
  above = v_set + TRACK_TAPER_MS
  assert map_track_decel_ms2(above, v_set, a5) == -a5
  # Lead still wins vs map brake.
  assert min(-2.0, -a5) == -2.0


def test_map_track_decel_matches_comfort_curve_when_above_max():
  a5 = map_brake_a_ms2(LOOKAHEAD_NORMAL)
  assert abs(a5 - 0.80) < 1e-9
  assert abs(map_brake_a_ms2(LOOKAHEAD_NORMAL) - map_comfort_a_ms2(LOOKAHEAD_NORMAL, 5)) < 1e-9
  v_ego = 70 * CV.MPH_TO_MS
  v_max = 45 * CV.MPH_TO_MS
  # Well above MAX → full comfort decel (locked Accel 5 = 0.80 m/s²).
  assert v_ego - v_max > TRACK_TAPER_MS
  assert map_track_decel_ms2(v_ego, v_max, a5) == -a5
  # Helper still accepts other a for unit math; planner must pass a5.
  assert map_track_decel_ms2(v_ego, v_max, 0.36) == -0.36
  assert map_track_decel_ms2(v_max, v_max, a5) is None
  assert map_track_decel_ms2(v_max - 1.0, v_max, a5) is None
  assert map_track_decel_ms2(v_max + TRACK_DEADBAND_MS, v_max, a5) is None
  mid = v_max + 0.5 * (TRACK_DEADBAND_MS + TRACK_TAPER_MS)
  a_mid = map_track_decel_ms2(mid, v_max, a5)
  assert a_mid is not None
  assert abs(a_mid - (-0.5 * a5)) < 1e-9


def test_accel_setting_does_not_change_brake_a():
  assert abs(map_brake_a_ms2(LOOKAHEAD_NORMAL) - 0.80) < 1e-9
  v_ego = 70 * CV.MPH_TO_MS
  v_max = 45 * CV.MPH_TO_MS
  locked = map_track_decel_ms2(v_ego, v_max, map_brake_a_ms2(LOOKAHEAD_NORMAL))
  assert locked == -0.80
  # Accel 1 vs 10 change climb a only.
  assert abs(map_accel_a_ms2(LOOKAHEAD_NORMAL, 1) - 0.36) < 1e-9
  assert abs(map_accel_a_ms2(LOOKAHEAD_NORMAL, 10) - 1.60) < 1e-9
  assert map_slew_a_ms2(30.0, 20.0, LOOKAHEAD_NORMAL, 1) == map_slew_a_ms2(30.0, 20.0, LOOKAHEAD_NORMAL, 10)
  assert abs(map_slew_a_ms2(30.0, 20.0, LOOKAHEAD_NORMAL, 10) - 0.80) < 1e-9
  assert abs(map_slew_a_ms2(20.0, 30.0, LOOKAHEAD_NORMAL, 1) - 0.36) < 1e-9
  assert abs(map_slew_a_ms2(20.0, 30.0, LOOKAHEAD_NORMAL, 10) - 1.60) < 1e-9
  a1 = map_track_accel_ms2(20.0, 31.29, map_accel_a_ms2(LOOKAHEAD_NORMAL, 1))
  a10 = map_track_accel_ms2(20.0, 31.29, map_accel_a_ms2(LOOKAHEAD_NORMAL, 10))
  assert a1 is not None and a10 is not None
  assert abs(a1 - 0.36) < 1e-9 and abs(a10 - 1.60) < 1e-9


def test_map_track_decel_loses_to_stronger_lead_brake():
  """Planner applies min(mpc, map_track). A slower lead still wins."""
  a_map = map_track_decel_ms2(31.29, 20.12, 0.80)
  assert a_map == -0.80
  a_lead = -2.0
  assert min(a_lead, a_map) == a_lead
  # MPC holding ~0 (no-lead cruise obstacle not binding) → map decel wins.
  assert min(0.0, a_map) == a_map


def _follow_hud(dec, map_kph: float) -> float:
  """Mirror card.py: seed / sticky win; else apply_map_speed (Follow)."""
  if dec.seed_kph is not None:
    return dec.seed_kph
  if dec.sticky:
    return dec.driver_kph
  return apply_map_speed_kph(
    dec.driver_kph, map_kph, mode=MODE_FOLLOW, engaged=True,
    op_long_software_cruise=True, driver_override=dec.follow_override,
  )


def test_engage_seeds_max_to_posted_limit():
  hold = MapCruiseHold()
  posted = 45 * CV.MPH_TO_KPH
  ego = 70 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=ego, posted_kph=posted,
    engage_rising=True, now=0.0,
  )
  assert dec.seed_kph is not None
  assert abs(dec.seed_kph - posted) < 1e-6
  assert abs(dec.driver_kph - posted) < 1e-6
  assert not dec.sticky
  assert hold.follow_override_until == 0.0
  # Failed pedal write-back / DI_digitalSpeed next frames must not look like a
  # stalk and must not arm a sticky hold.
  for t in (0.05, 5.0, 11.0):
    dec = decide_map_cruise(
      hold, engaged=True, mode=MODE_FOLLOW, raw_kph=ego, posted_kph=posted,
      engage_rising=False, now=t, stalk_pressed=False,
    )
    assert not dec.sticky
    assert hold.follow_override_until == 0.0
    assert abs(_follow_hud(dec, posted) - posted) < 1e-6
  # No map: keep ego capture.
  hold2 = MapCruiseHold()
  dec2 = decide_map_cruise(
    hold2, engaged=True, mode=MODE_CAP, raw_kph=ego, posted_kph=None,
    engage_rising=True, now=0.0,
  )
  assert dec2.seed_kph is None
  assert abs(dec2.driver_kph - ego) < 1e-6


def test_sticky_manual_below_limit_until_posted_changes():
  hold = MapCruiseHold()
  a = 45 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  below = a - 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=True,
  )
  assert dec.sticky
  assert dec.seed_kph is not None  # one-shot write of the stalk step
  assert abs(dec.driver_kph - below) < 1e-6
  assert hold.follow_override_until == 0.0
  # Still a: hold, do not Follow back to a, do not write pedal every frame.
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=a,
    engage_rising=False, now=20.0, stalk_pressed=False,
  )
  assert dec.sticky
  assert dec.seed_kph is None
  assert abs(dec.driver_kph - below) < 1e-6
  assert not should_write_preap_pedal(dec.seed_kph, _follow_hud(dec, a), below)
  # Limit changes to b: drop sticky. Do not seed-snap MAX to b.
  b = 35 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=b,
    engage_rising=False, now=21.0, stalk_pressed=False,
  )
  assert not dec.sticky
  assert dec.seed_kph is None
  assert abs(_follow_hud(dec, b) - b) < 1e-6


def test_disengage_clears_sticky_and_follow_timer():
  hold = MapCruiseHold()
  a = 45 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  below = a - 5 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=True,
  )
  assert hold.sticky_set_kph is not None
  dec = decide_map_cruise(
    hold, engaged=False, mode=MODE_FOLLOW, raw_kph=below, posted_kph=a,
    engage_rising=False, now=2.0,
  )
  assert not dec.sticky
  assert dec.seed_kph is None
  assert hold.sticky_set_kph is None
  assert hold.follow_override_until == 0.0


def test_sticky_below_limit_survives_past_ten_seconds():
  """Stalk to a-5 must still be a-5 after >10s with unchanged posted limit."""
  hold = MapCruiseHold()
  a = 45 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  below = a - 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=True,
  )
  assert dec.sticky
  assert hold.follow_override_until == 0.0
  for t in (11.5, 15.0, 60.0):
    dec = decide_map_cruise(
      hold, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=a,
      engage_rising=False, now=t, stalk_pressed=False,
    )
    assert dec.sticky, f"sticky lost at t={t}"
    assert hold.follow_override_until == 0.0
    assert dec.seed_kph is None
    assert abs(_follow_hud(dec, a) - below) < 1e-6
    # apply_map_speed without sticky would Follow-raise to a; card must not.
    expired = apply_map_speed_kph(
      dec.driver_kph, a, mode=MODE_FOLLOW, engaged=True,
      op_long_software_cruise=True, driver_override=False,
    )
    assert abs(expired - a) < 1e-6
    assert abs(_follow_hud(dec, a) - expired) > 1.0


def test_cruise_stalk_step_is_1_or_5_not_ego_jump():
  a = 45 * CV.MPH_TO_KPH
  assert is_cruise_stalk_step(a, a - 5 * CV.MPH_TO_KPH)
  assert is_cruise_stalk_step(a, a + 1 * CV.MPH_TO_KPH)
  assert is_cruise_stalk_step(a, a - 1.0)  # metric 1 kph
  assert is_cruise_stalk_step(a, a + 5.0)
  assert not is_cruise_stalk_step(a, 70 * CV.MPH_TO_KPH)
  assert not is_cruise_stalk_step(a, a)


def test_stalk_plus_minus_changes_set_without_button_events():
  """pre-AP buttonEvents are unreliable; a 5 mph pedal_speed step must move MAX."""
  hold = MapCruiseHold()
  a = 45 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  down = a - 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=down, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=False,
  )
  assert dec.sticky
  assert abs(_follow_hud(dec, a) - down) < 1e-6
  up = down + 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=up, posted_kph=a,
    engage_rising=False, now=2.0, stalk_pressed=False,
  )
  assert abs(_follow_hud(dec, a) - up) < 1e-6
  above = a + 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=3.0, stalk_pressed=False,
  )
  assert dec.sticky
  assert abs(_follow_hud(dec, a) - above) < 1e-6
  # Cap: stalk up cannot exceed posted; stalk down still lowers MAX.
  hold_c = MapCruiseHold()
  decide_map_cruise(
    hold_c, engaged=True, mode=MODE_CAP, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  dec = decide_map_cruise(
    hold_c, engaged=True, mode=MODE_CAP, raw_kph=above, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=False,
  )
  cap_out = apply_map_speed_kph(
    dec.driver_kph, a, mode=MODE_CAP, engaged=True, op_long_software_cruise=True,
    driver_override=dec.follow_override,
  )
  assert abs(cap_out - a) < 1e-6
  decide_map_cruise(
    hold_c, engaged=True, mode=MODE_CAP, raw_kph=a, posted_kph=a,
    engage_rising=False, now=2.0, stalk_pressed=False,
  )
  dec = decide_map_cruise(
    hold_c, engaged=True, mode=MODE_CAP, raw_kph=down, posted_kph=a,
    engage_rising=False, now=3.0, stalk_pressed=False,
  )
  assert dec.sticky
  assert abs(dec.driver_kph - down) < 1e-6


def test_sticky_hud_ignores_anticipatory_map_kph():
  """Card sticky path must keep 66 even if map_kph slewed toward upcoming 50."""
  hold = MapCruiseHold()
  a = 60 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  above = a + 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=False,
  )
  assert dec.sticky
  anticipated = 50 * CV.MPH_TO_KPH
  assert abs(_follow_hud(dec, anticipated) - above) < 1e-6
  assert dec.seed_kph is not None  # real 5 mph stalk step writes once
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=2.0, stalk_pressed=False,
  )
  assert dec.sticky
  assert dec.seed_kph is None
  assert abs(_follow_hud(dec, anticipated) - above) < 1e-6
  assert not should_write_preap_pedal(dec.seed_kph, _follow_hud(dec, anticipated), above)


def test_follow_holds_absolute_set_until_posted_changes():
  """Justin: set 55 in a 50 stays 55 until posted changes; set 45 in a 50, same."""
  hold = MapCruiseHold()
  a = 50 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  above = 55 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=True,
  )
  assert dec.sticky
  assert hold.follow_override_until == 0.0
  assert abs(_follow_hud(dec, a) - above) < 1e-6
  for t in (11.1, 15.0, 60.0):
    dec = decide_map_cruise(
      hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
      engage_rising=False, now=t, stalk_pressed=False,
    )
    assert dec.sticky, f"55 mph sticky lost at t={t}"
    assert dec.seed_kph is None
    assert abs(_follow_hud(dec, a) - above) < 1e-6
  b = 35 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=b,
    engage_rising=False, now=61.0, stalk_pressed=False,
  )
  assert not dec.sticky
  assert dec.seed_kph is None  # decrease must not cliff MAX to posted
  assert abs(_follow_hud(dec, b) - b) < 1e-6

  hold2 = MapCruiseHold()
  decide_map_cruise(
    hold2, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  below = 45 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold2, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=True,
  )
  assert dec.sticky
  assert abs(_follow_hud(dec, a) - below) < 1e-6
  dec = decide_map_cruise(
    hold2, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=a,
    engage_rising=False, now=60.0, stalk_pressed=False,
  )
  assert dec.sticky
  assert dec.seed_kph is None
  assert abs(_follow_hud(dec, a) - below) < 1e-6
  dec = decide_map_cruise(
    hold2, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=b,
    engage_rising=False, now=61.0, stalk_pressed=False,
  )
  assert not dec.sticky
  assert dec.seed_kph is None
  assert abs(_follow_hud(dec, b) - b) < 1e-6


def test_cap_does_not_exceed_limit_when_raising():
  hold = MapCruiseHold()
  a = 45 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_CAP, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  above = a + 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_CAP, raw_kph=above, posted_kph=a,
    engage_rising=False, now=1.0,
  )
  assert not dec.sticky
  out = apply_map_speed_kph(
    dec.driver_kph, a, mode=MODE_CAP, engaged=True, op_long_software_cruise=True,
    driver_override=dec.follow_override,
  )
  assert abs(out - a) < 1e-6


def test_slew_rate_limits_map_max_steps():
  # 0.80 m/s² × 0.1 s = 0.08 m/s max step
  out = slew_map_speed_ms(30.0, 20.0, 0.1, 0.80)
  assert abs(out - 29.92) < 1e-9
  done = slew_map_speed_ms(20.05, 20.0, 0.1, 0.80)
  assert done == 20.0


def test_should_write_preap_pedal_on_raise_not_every_frame():
  a = 45 * CV.MPH_TO_KPH
  b = 50 * CV.MPH_TO_KPH
  # Engage / posted / stalk-step seed writes once.
  assert should_write_preap_pedal(a, a, a)
  assert should_write_preap_pedal(a, a, None)
  # Sticky hold / same MAX every Follow frame: do not write (that ate stalk).
  assert not should_write_preap_pedal(None, a, a)
  # Stalk up / Follow posted raise: HUD rose vs last pedal write.
  assert should_write_preap_pedal(None, b, a)
  # No last write yet and no seed: leave CI pedal alone.
  assert not should_write_preap_pedal(None, b, None)
  # Decrease without seed: planner brakes from HUD MAX; do not clobber stalk down.
  assert not should_write_preap_pedal(None, a, b)


def test_sticky_hold_does_not_overwrite_ci_stalk_step():
  """After CI.update steps pedal, card must not write the old sticky hold.

  Order in card.py: CI.update (stalk +/-) then overlay. Writing seed_kph
  every sticky frame undoes that 1/5 mph step.
  """
  hold = MapCruiseHold()
  a = 50 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  last_pedal = a
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=False,
  )
  hud = _follow_hud(dec, a)
  assert not dec.sticky
  assert dec.seed_kph is None
  assert not should_write_preap_pedal(dec.seed_kph, hud, last_pedal)

  above = a + 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=2.0, stalk_pressed=False,
  )
  hud = _follow_hud(dec, a)
  assert dec.sticky
  assert abs(hud - above) < 1e-6
  assert should_write_preap_pedal(dec.seed_kph, hud, last_pedal)
  write_val = dec.seed_kph if dec.seed_kph is not None else hud
  assert abs(write_val - above) < 1e-6

  last_pedal = above
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=3.0, stalk_pressed=False,
  )
  hud = _follow_hud(dec, a)
  assert dec.sticky
  assert dec.seed_kph is None
  assert abs(hud - above) < 1e-6
  assert not should_write_preap_pedal(dec.seed_kph, hud, last_pedal)

  down = above - 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=down, posted_kph=a,
    engage_rising=False, now=4.0, stalk_pressed=False,
  )
  hud = _follow_hud(dec, a)
  assert dec.sticky
  assert abs(hud - down) < 1e-6
  write_val = dec.seed_kph if dec.seed_kph is not None else hud
  assert abs(write_val - down) < 1e-6

  # Button event this frame, CS.speed still the old hold: do not write `a`
  # over CI's in-flight +5.
  hold2 = MapCruiseHold()
  decide_map_cruise(
    hold2, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  dec = decide_map_cruise(
    hold2, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=False, now=2.0, stalk_pressed=True,
  )
  hud = _follow_hud(dec, a)
  assert not should_write_preap_pedal(dec.seed_kph, hud, a)
  assert dec.seed_kph is None


def test_hud_current_speed_is_wheel_ego_not_cluster_or_max():
  """Top-middle 3X speed was ~56 mph vs ~45 actual — that is 90 kph MAX.

  vEgoCluster is DI_digitalSpeed, which pre-AP also uses as cruiseState.speed.
  Map-speed writes MAX into vCruise / pedal_speed / cruiseState.speed. Live
  speed must be ESP/wheel vEgo only.
  """
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  for rel in (
    "selfdrive/ui/onroad/hud_renderer.py",
    "selfdrive/ui/mici/onroad/hud_renderer.py",
  ):
    src = (root / rel).read_text()
    assert "self.speed = max(0.0, float(car_state.vEgo) * speed_conversion)" in src
    assert "v_ego = v_ego_cluster if" not in src
    assert "self.speed = md.speedLimit" not in src
    assert "self.speed = self.map_speed_limit" not in src
    assert "self.speed = self.set_speed" not in src
    # MAX box is vCruiseCluster; LIMIT sign is OSM. Do not paint either as live.
  card = (root / "selfdrive/car/card.py").read_text()
  assert "vEgoCluster" not in card
  assert "CS.vEgo =" not in card
  assert "CS.vCruise =" in card


def test_map_speed_submenu_wires_params():
  """Menu wiring without importing raylib / cereal UI."""
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  tici = (root / "selfdrive/ui/layouts/settings/map_speed.py").read_text()
  mici = (root / "selfdrive/ui/mici/layouts/settings/map_speed.py").read_text()
  nap = (root / "selfdrive/ui/layouts/settings/nap.py").read_text()
  nap_mici = (root / "selfdrive/ui/mici/layouts/settings/nap.py").read_text()
  for src in (tici, mici):
    for key in ("NAPMapSpeedMode", "NAPMapSpeedOffsetMph", "NAPMapSpeedLookahead", "NAPMapSpeedAccel"):
      assert key in src
  assert "self._scroller.add_widgets" in mici
  assert "Brake to a lower MAX is locked" in tici
  assert "0.80 m/s² at Normal" in tici
  assert "1.20 m/s² at Normal" not in tici
  assert "pauses Follow for 10s" not in tici
  assert "holds until the posted limit changes" in tici
  assert "lookahead pauses while that set is active" in tici
  assert "decreases still ease with lookahead" not in tici
  assert "every decrease eases with kin+110 m" in tici
  assert "1.5 s GPS lag raises posted at the" in tici
  assert "A higher limit far ahead never raises MAX" in tici
  assert "A higher limit ahead never raises MAX early" not in tici
  assert "acceleration only" in mici
  assert "Map Speed Limit" in nap
  assert "Radar Settings" in nap
  assert "map speed limit" in nap_mici
  assert "radar settings" in nap_mici
  assert "gradual ease-off farther back" in nap
  assert "not a harder brake" in nap
  assert "follow distance" in nap_mici
  assert "Refresh maps" in tici
  assert '"Back To"' in tici
  assert "←" not in tici
  assert "Check for map updates" not in tici
  assert "refresh maps" in mici
  assert "check for map updates" not in mici
  assert "scripts.nap.refresh_osm_maps" in nap
  assert "scripts.nap.refresh_osm_maps" in mici
  assert tici.index("_all_items.append(self._refresh_btn)") < tici.index("_all_items.append(self._download_btn)")
  mici_widgets = mici.split("self._scroller.add_widgets", 1)[1]
  assert mici_widgets.index("refresh_maps_btn") < mici_widgets.index("download_maps_btn")
  assert "NAPMapSpeedAccel" in (root / "common/params_keys.h").read_text()
  assert "NAPMapSpeedDbRevision" in (root / "common/params_keys.h").read_text()
  assert "NAPMapSpeedDbSha256" in (root / "common/params_keys.h").read_text()


def test_planner_and_mpc_keep_radar_after_map_cap():
  """Regression: map cap runs before mpc.update(radarState, v_cruise)."""
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  planner = (root / "selfdrive/controls/lib/longitudinal_planner.py").read_text()
  mpc = (root / "selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py").read_text()
  cap_at = planner.find("cap_planner_v_cruise_ms")
  mpc_at = planner.find("self.mpc.update(sm['radarState'], v_cruise")
  hold_at = planner.find("map_in_track_deadband(v_ego, v_hud_ms)")
  track_at = planner.find("a_brake = map_track_decel_ms2")
  lead_at = planner.find("a_lead = lead_approach_decel_ms2")
  assert 0 <= cap_at < mpc_at
  assert 0 <= mpc_at < hold_at < track_at < lead_at
  assert "map_brake_a_ms2" in planner
  assert "map_track_accel_ms2" in planner
  assert "min(float(output_a_target), a_brake)" in planner
  assert "if float(output_a_target) >= 0.0:" in planner
  assert "output_a_target = 0.0" in planner
  assert "np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])" in mpc
  assert "self.params[:,2] = np.min(x_obstacles, axis=1)" in mpc
  card = (root / "selfdrive/car/card.py").read_text()
  assert "pedalLongActive" in card
  assert "stalk_pressed=stalk_pressed" in card
  assert "_write_preap_pedal_speed" in card
  assert "should_write_preap_pedal" in card
  # Must not seed/overlay on lateral-only first pull (CC.enabled).
  assert "engage_rising = long_active and not long_active_prev" in card
  # Must not clobber stalk by writing Follow HUD onto pedal every frame.
  assert "should_write_preap_pedal(dec.seed_kph, preap_v_cruise_kph, self._last_pedal_kph)" in card
  assert "if long_active and dec.seed_kph is not None:" not in card
  # Sticky hold must not slew toward nextSpeedLimit.
  assert "not dec.sticky:" in card
  assert "sticky_hold = self._map_hold.sticky_set_kph is not None" not in card
  planner_src = planner
  assert "output_a_target = a_up" in planner_src
  assert "min(float(output_a_target), a_up)" not in planner_src
  mapd = (root / "selfdrive/mapd/mapd.py").read_text()
  osm = (root / "selfdrive/mapd/osm_db.py").read_text()
  constants = (root / "selfdrive/mapd/constants.py").read_text()
  assert "v_ego_ms=v_ego_ms" in mapd
  assert "osm_sign_lead_m(v_ego_ms)" in osm
  assert "OSM_SIGN_LEAD_S = 1.5" in constants
  assert "OSM_SIGN_LEAD_M" not in constants
  assert "DECREASE_START_MARGIN_M = 110.0" in constants
  assert "Do not snap posted down" in osm
  policy = (root / "selfdrive/mapd/map_speed_policy.py").read_text()
  assert "float(posted_kph) if raised else None" in policy
  # Sticky / stalk path unchanged.
  assert "should_write_preap_pedal(dec.seed_kph, preap_v_cruise_kph, self._last_pedal_kph)" in card
  assert "blinker_turn_limit_ms" in card
  assert "hud_with_blinker_turn_kph" in card
  assert "turnSignalStalkState" in card
  assert "lookup_intersection" in mapd
  assert "lookup_intersection" in osm
  assert "TURN_SPEED_DEFAULT_MPH = 15.0" in constants
  dh = (root / "selfdrive/controls/lib/desire_helper.py").read_text()
  assert "hold_for_intersection" in dh
  modeld = (root / "selfdrive/modeld/modeld.py").read_text()
  assert "apply_late_apex_curvature" in modeld
  assert "blinker_turn_direction" in modeld
  assert "Desire.turnLeft" not in modeld
  lead = (root / "selfdrive/controls/lib/lead_approach.py").read_text()
  assert "LEAD_APPROACH_A_MS2 = 0.80" in lead
  assert "LEAD_APPROACH_MARGIN_M = 110.0" in lead
  assert "LATE_APEX_Y_M = 0.30" in constants
  assert "LATE_APEX_CURV_SCALE = 0.90" in constants


def test_turn_speed_table_12_15_18():
  posted = 45 * CV.MPH_TO_MS
  assert abs(turn_speed_ms(25 * CV.MPH_TO_MS, posted) - 12 * CV.MPH_TO_MS) < 0.05
  assert abs(turn_speed_ms(35 * CV.MPH_TO_MS, posted) - 15 * CV.MPH_TO_MS) < 0.05
  assert abs(turn_speed_ms(45 * CV.MPH_TO_MS, posted) - 18 * CV.MPH_TO_MS) < 0.05
  assert abs(turn_speed_ms(0.0, posted) - 15 * CV.MPH_TO_MS) < 0.05
  # Never above dest or posted.
  assert abs(turn_speed_ms(10 * CV.MPH_TO_MS, posted) - 10 * CV.MPH_TO_MS) < 0.05
  slow_posted = 12 * CV.MPH_TO_MS
  assert abs(turn_speed_ms(45 * CV.MPH_TO_MS, slow_posted) - slow_posted) < 0.05


def test_blinker_turn_uses_stalk_not_lamp():
  assert blinker_turn_direction(1, True, True) == 1
  assert blinker_turn_direction(2, True, True) == 2
  assert blinker_turn_direction(0, True, True) == 0  # lamp-only / idle lever
  assert blinker_turn_direction(1, False, True) == 0  # left stalk, only right junction
  assert blinker_turn_direction(2, True, False) == 0
  assert blinker_turn_holds_alc(1, True, False, 80.0)
  assert not blinker_turn_holds_alc(0, True, True, 80.0)


def test_blinker_turn_eases_like_map_decrease_and_yields_sticky():
  posted = 45 * CV.MPH_TO_MS
  dest = 25 * CV.MPH_TO_MS
  target = turn_speed_ms(dest, posted)
  v_ego = 45 * CV.MPH_TO_MS
  far = blinker_turn_limit_ms(
    stalk_state=1, has_left=True, has_right=True, dist_m=2000.0,
    left_dest_ms=dest, right_dest_ms=dest, posted_ms=posted, v_ego_ms=v_ego,
    lookahead=LOOKAHEAD_NORMAL,
  )
  assert far is None
  close = blinker_turn_limit_ms(
    stalk_state=1, has_left=True, has_right=True, dist_m=80.0,
    left_dest_ms=dest, right_dest_ms=dest, posted_ms=posted, v_ego_ms=v_ego,
    lookahead=LOOKAHEAD_NORMAL,
  )
  assert close is not None
  assert target - 0.2 <= close < posted
  at = blinker_turn_limit_ms(
    stalk_state=1, has_left=True, has_right=True, dist_m=0.0,
    left_dest_ms=dest, right_dest_ms=dest, posted_ms=posted, v_ego_ms=v_ego,
    lookahead=LOOKAHEAD_NORMAL,
  )
  assert at is not None and abs(at - target) < 0.2
  # Idle stalk: no turn ceiling.
  assert blinker_turn_limit_ms(
    stalk_state=0, has_left=True, has_right=True, dist_m=80.0,
    left_dest_ms=dest, right_dest_ms=dest, posted_ms=posted, v_ego_ms=v_ego,
    lookahead=LOOKAHEAD_NORMAL,
  ) is None
  # Lookahead Off still slows for a blinker turn (uses Normal ease).
  off = blinker_turn_limit_ms(
    stalk_state=2, has_left=True, has_right=True, dist_m=80.0,
    left_dest_ms=dest, right_dest_ms=dest, posted_ms=posted, v_ego_ms=v_ego,
    lookahead=LOOKAHEAD_OFF,
  )
  assert off is not None and off < posted

  hold = MapCruiseHold()
  a = 45 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  above = a + 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=False,
  )
  assert dec.sticky
  # Without blinker, sticky still ignores upcoming posted 30.
  assert abs(_follow_hud(dec, 30 * CV.MPH_TO_KPH) - above) < 1e-6
  turn_kph = close * CV.MS_TO_KPH
  hud = hud_with_blinker_turn_kph(
    dec, 30 * CV.MPH_TO_KPH, turn_kph,
    mode=MODE_FOLLOW, offset_kph=0.0, engaged=True,
  )
  assert abs(hud - turn_kph) < 1e-6
  # Blinker cancel restores sticky.
  restored = hud_with_blinker_turn_kph(
    dec, 30 * CV.MPH_TO_KPH, None,
    mode=MODE_FOLLOW, offset_kph=0.0, engaged=True,
  )
  assert abs(restored - above) < 1e-6
  assert hold.sticky_set_kph is not None


def test_blinker_turn_does_not_add_map_offset():
  hold = MapCruiseHold()
  posted = 45 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=posted, posted_kph=posted,
    engage_rising=True, now=0.0,
  )
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=posted, posted_kph=posted,
    engage_rising=False, now=1.0, stalk_pressed=False,
  )
  turn = 15 * CV.MPH_TO_KPH
  offset = 5 * CV.MPH_TO_KPH
  hud = hud_with_blinker_turn_kph(
    dec, posted, turn, mode=MODE_FOLLOW, offset_kph=offset, engaged=True,
  )
  assert abs(hud - turn) < 1e-6


def test_late_apex_identity_when_not_intersection_turn():
  """ALC / idle stalk: curvature and path y must not change."""
  k = 0.04
  assert apply_late_apex_curvature(k, 20.0, 0) == k
  # Stalk on, no OSM junction (multi-lane ALC).
  assert blinker_turn_direction(1, False, False) == 0
  assert blinker_turn_direction(2, False, False) == 0
  assert not blinker_turn_holds_alc(1, False, False, 80.0)
  assert apply_late_apex_curvature(k, 20.0, blinker_turn_direction(1, False, False)) == k
  assert late_apex_y_offset_m(0) == 0.0
  assert blinker_turn_limit_ms(
    stalk_state=1, has_left=False, has_right=False, dist_m=80.0,
    left_dest_ms=0.0, right_dest_ms=0.0, posted_ms=25.0, v_ego_ms=20.0,
    lookahead=LOOKAHEAD_NORMAL,
  ) is None


def test_late_apex_biases_outside_and_does_not_cut_inside():
  """Left turn: y negative (right/outside), kappa less left. Right is the mirror."""
  assert abs(LATE_APEX_Y_M - 0.30) < 1e-9
  assert abs(LATE_APEX_CURV_SCALE - 0.90) < 1e-9
  assert late_apex_y_offset_m(1) == -LATE_APEX_Y_M
  assert late_apex_y_offset_m(2) == LATE_APEX_Y_M
  assert late_apex_ramp(0.0) == 0.0
  assert late_apex_ramp(2.0) == 1.0
  assert 0.0 < late_apex_ramp(1.0) < 1.0
  k_left = 0.05
  out_left = apply_late_apex_curvature(k_left, 15.0, 1)
  assert out_left < k_left * LATE_APEX_CURV_SCALE + 1e-9
  assert out_left < k_left
  k_right = -0.05
  out_right = apply_late_apex_curvature(k_right, 15.0, 2)
  assert out_right > k_right * LATE_APEX_CURV_SCALE - 1e-9
  assert out_right > k_right
  # Idle / ALC still identity at the same kappa.
  assert apply_late_apex_curvature(k_left, 15.0, 0) == k_left


