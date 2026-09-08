from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import (
  ACCEL_DEFAULT, DRIVER_OVERRIDE_S, LOOKAHEAD_EARLY, LOOKAHEAD_NORMAL, LOOKAHEAD_OFF,
  MODE_CAP, MODE_DISPLAY, MODE_FOLLOW, MODE_OFF,
  TRACK_DEADBAND_MS, TRACK_TAPER_MS,
  accel_scale_factor, map_accel_a_ms2, map_brake_a_ms2, map_comfort_a_ms2,
)
from openpilot.selfdrive.mapd.map_speed_policy import (
  SOURCE_CRUISE, SOURCE_LEAD0, V_CRUISE_UNSET,
  MapCruiseHold, anticipatory_limit_ms, apply_map_speed_kph, cap_planner_v_cruise_ms,
  decide_map_cruise, effective_map_limit_ms, is_cruise_stalk_step,
  longitudinal_obstacle_source, map_slew_a_ms2, map_track_accel_ms2,
  map_track_decel_ms2, slew_map_speed_ms,
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
  assert cap_planner_v_cruise_ms(30.0, lim, mode=MODE_CAP) == lim
  assert cap_planner_v_cruise_ms(20.0, lim, mode=MODE_CAP) == 20.0
  assert cap_planner_v_cruise_ms(30.0, lim, mode=MODE_DISPLAY) == 30.0
  assert cap_planner_v_cruise_ms(30.0, None, mode=MODE_FOLLOW) == 30.0


def test_no_lead_tracks_map_capped_cruise():
  """No radar lead: fake far/fast lead, so cruise (map ceiling) is the tightest obstacle."""
  v_ego = 31.29  # ~70 mph
  v_cruise = cap_planner_v_cruise_ms(40.0, 31.29, mode=MODE_CAP)
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
  # Beyond Normal horizon (400 m) — do not start yet.
  assert anticipatory_limit_ms(current, nxt, 500.0, current, LOOKAHEAD_NORMAL) is None
  # Display never changes MAX even if we computed an anticipatory ceiling.
  assert apply_map_speed_kph(
    100, 45, mode=MODE_DISPLAY, engaged=True, op_long_software_cruise=True,
  ) == 100


def test_anticipatory_cap_still_loses_to_slower_lead():
  current = 70 * CV.MPH_TO_MS
  nxt = 45 * CV.MPH_TO_MS
  eff = effective_map_limit_ms(current, nxt, 150.0, current, LOOKAHEAD_NORMAL)
  assert eff is not None
  v_cruise = cap_planner_v_cruise_ms(40.0, eff, mode=MODE_CAP)
  assert v_cruise <= current
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
  # stalk and must not arm the 10s Follow timer.
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
  assert dec.seed_kph is not None  # pedal write-back for sticky
  assert abs(dec.driver_kph - below) < 1e-6
  assert hold.follow_override_until == 0.0
  # Still a: hold, do not Follow back to a.
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=a,
    engage_rising=False, now=20.0, stalk_pressed=False,
  )
  assert dec.sticky
  assert abs(dec.driver_kph - below) < 1e-6
  # Limit changes to b: resume at b, drop a-5.
  b = 35 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=b,
    engage_rising=False, now=21.0, stalk_pressed=False,
  )
  assert not dec.sticky
  assert dec.seed_kph is not None
  assert abs(dec.seed_kph - b) < 1e-6


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


def test_sticky_below_limit_survives_past_ten_second_override():
  """Regression: stalk to a-5 must still be a-5 after >10s with unchanged limit.

  The Follow raise-above timer (DRIVER_OVERRIDE_S) must never clear this.
  """
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
  assert DRIVER_OVERRIDE_S == 10.0
  for t in (1.0 + DRIVER_OVERRIDE_S + 0.5, 15.0, 60.0):
    dec = decide_map_cruise(
      hold, engaged=True, mode=MODE_FOLLOW, raw_kph=below, posted_kph=a,
      engage_rising=False, now=t, stalk_pressed=False,
    )
    assert dec.sticky, f"sticky lost at t={t}"
    assert hold.follow_override_until == 0.0
    assert abs(_follow_hud(dec, a) - below) < 1e-6
    # Passing driver_override=False (expired timer) must not be how card runs.
    expired = apply_map_speed_kph(
      dec.driver_kph, a, mode=MODE_FOLLOW, engaged=True,
      op_long_software_cruise=True, driver_override=False,
    )
    assert abs(expired - a) < 1e-6  # what the old 10s path would do
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
  assert not dec.sticky
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


def test_follow_ten_second_override_is_raise_above_limit_only():
  hold = MapCruiseHold()
  a = 45 * CV.MPH_TO_KPH
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=a, posted_kph=a,
    engage_rising=True, now=0.0,
  )
  above = a + 5 * CV.MPH_TO_KPH
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=1.0, stalk_pressed=True,
  )
  assert not dec.sticky
  assert hold.follow_override_until == 1.0 + DRIVER_OVERRIDE_S
  assert abs(_follow_hud(dec, a) - above) < 1e-6
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=above, posted_kph=a,
    engage_rising=False, now=1.0 + DRIVER_OVERRIDE_S + 0.1, stalk_pressed=False,
  )
  assert not dec.sticky
  assert abs(_follow_hud(dec, a) - a) < 1e-6


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
  assert "acceleration only" in mici
  assert "Map Speed Limit" in nap
  assert "Radar Settings" in nap
  assert "map speed limit" in nap_mici
  assert "radar settings" in nap_mici
  assert "NAPMapSpeedAccel" in (root / "common/params_keys.h").read_text()


def test_planner_and_mpc_keep_radar_after_map_cap():
  """Regression: map cap runs before mpc.update(radarState, v_cruise)."""
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  planner = (root / "selfdrive/controls/lib/longitudinal_planner.py").read_text()
  mpc = (root / "selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py").read_text()
  cap_at = planner.find("cap_planner_v_cruise_ms")
  mpc_at = planner.find("self.mpc.update(sm['radarState'], v_cruise")
  track_at = planner.find("a_brake = map_track_decel_ms2")
  assert 0 <= cap_at < mpc_at
  assert 0 <= mpc_at < track_at
  assert "map_brake_a_ms2" in planner
  assert "map_track_accel_ms2" in planner
  assert "min(float(output_a_target), a_brake)" in planner
  assert "np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])" in mpc
  assert "self.params[:,2] = np.min(x_obstacles, axis=1)" in mpc
  card = (root / "selfdrive/car/card.py").read_text()
  assert "pedalLongActive" in card
  assert "stalk_pressed=stalk_pressed" in card
  assert "_write_preap_pedal_speed" in card
  # Must not seed/overlay on lateral-only first pull (CC.enabled).
  assert "engage_rising = long_active and not long_active_prev" in card
  # Must not clobber stalk by writing Follow HUD onto pedal every frame.
  assert "if long_active and dec.seed_kph is not None:" in card
