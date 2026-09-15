"""Hypermile: eco snap/restore, posted-scaled offset, stock 1–7 stalk follow."""
from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.hypermile import (
  ECO_MAP_ACCEL,
  ECO_MAP_LOOKAHEAD_EARLY,
  ECO_MAP_MODE_CAP,
  ECO_MAP_MODE_FOLLOW,
  ECO_MAP_OFFSET_MPH,
  ECO_OFFSET_FULL_MPH,
  ECO_OFFSET_START_MPH,
  FOLLOW_DISTANCE_DEFAULT,
  FOLLOW_DISTANCE_MAX,
  FOLLOW_DISTANCE_MIN,
  PARAM_FOLLOW_DISTANCE,
  PARAM_HYPERMILE,
  PARAM_SAVED,
  PARAM_STEP_DOWN,
  STEP_DOWN_MPH,
  CRUISE_STALK_DN_1ST,
  CRUISE_STALK_IDLE,
  CRUISE_STALK_UP_1ST,
  CRUISE_STALK_UP_2ND,
  FollowStalkGesture,
  apply_hypermile_toggle,
  button_event_closer,
  button_event_released,
  consume_follow_stalk,
  detect_follow_stalk,
  eco_map_offset_mph,
  eco_preset_from,
  effective_nap_follow_dist,
  follow_distance_hud_text,
  poll_follow_distance_hud,
  map_target_offset_kph,
  maps_posted_known,
  persist_follow_distance,
  read_follow_distance,
  read_hypermile_params,
  read_hypermile_step_down,
  step_down_applies,
  step_down_offset_mph,
  stepped_map_target_kph,
  stalk_adjusts_follow,
  step_follow_distance,
)
from openpilot.selfdrive.controls.lib.lead_approach import NAP_T_FOLLOW


class FakeParams:
  def __init__(self, ints=None, bools=None):
    self.ints = dict(ints or {})
    self.bools = dict(bools or {})

  def get(self, key, return_default=False):
    return self.ints.get(key)

  def get_bool(self, key):
    return bool(self.bools.get(key, False))

  def put(self, key, value):
    self.ints[key] = value

  def put_bool(self, key, value):
    self.bools[key] = bool(value)

  def remove(self, key):
    self.ints.pop(key, None)


def test_default_off_and_stock_follow_four():
  assert read_hypermile_params(FakeParams()) is False
  assert read_follow_distance(FakeParams()) == FOLLOW_DISTANCE_DEFAULT


def test_on_snaps_eco_and_off_restores_prior():
  params = FakeParams(
    ints={
      "NAPMapSpeedMode": 0,
      "NAPMapSpeedOffsetMph": 5,
      "NAPMapSpeedLookahead": 0,
      "NAPMapSpeedAccel": 8,
      "NAPFollowDistance": 4,
    },
    bools={
      "NAPAdaptiveAccel": False,
      "NAPDriverLatHandoff": True,
      "NAPDmSimulateLooking": False,
      "NAPDmFalseAlertIgnore": False,
    },
  )
  lat_before = params.get_bool("NAPDriverLatHandoff")
  dm_before = params.get_bool("NAPDmSimulateLooking")
  fai_before = params.get_bool("NAPDmFalseAlertIgnore")
  follow_before = params.get("NAPFollowDistance")

  assert apply_hypermile_toggle(params, True) is True
  assert params.get_bool(PARAM_HYPERMILE) is True
  assert params.get_bool("NAPAdaptiveAccel") is True
  assert params.get("NAPMapSpeedMode") == ECO_MAP_MODE_FOLLOW
  # Do not snap a flat −5 — that forced town 30→25. Offset stays the user's.
  assert params.get("NAPMapSpeedOffsetMph") == 5
  assert params.get("NAPMapSpeedLookahead") == ECO_MAP_LOOKAHEAD_EARLY
  assert params.get("NAPMapSpeedAccel") == ECO_MAP_ACCEL
  assert ECO_MAP_LOOKAHEAD_EARLY == 3
  assert ECO_MAP_ACCEL == 1
  assert params.get_bool("NAPDriverLatHandoff") is lat_before
  assert params.get_bool("NAPDmSimulateLooking") is dm_before
  assert params.get_bool("NAPDmFalseAlertIgnore") is fai_before
  assert params.get("NAPFollowDistance") == follow_before
  assert params.get(PARAM_SAVED)

  # Second On is a no-op (does not re-snapshot the eco values).
  apply_hypermile_toggle(params, True)
  assert params.get("NAPMapSpeedOffsetMph") == 5

  assert apply_hypermile_toggle(params, False) is False
  assert params.get_bool(PARAM_HYPERMILE) is False
  assert params.get_bool("NAPAdaptiveAccel") is False
  assert params.get("NAPMapSpeedMode") == 0
  assert params.get("NAPMapSpeedOffsetMph") == 5
  assert params.get("NAPMapSpeedLookahead") == 0
  assert params.get("NAPMapSpeedAccel") == 8
  assert params.get_bool("NAPDriverLatHandoff") is True
  assert params.get_bool("NAPDmSimulateLooking") is False
  assert params.get_bool("NAPDmFalseAlertIgnore") is False
  assert params.get("NAPFollowDistance") == 4
  assert not params.get(PARAM_SAVED)
  # Step-down / Hill Climb are independent — eco snap/restore must not touch them.
  params.put_bool(PARAM_STEP_DOWN, True)
  params.put_bool("NAPHypermileHillClimb", False)
  apply_hypermile_toggle(params, True)
  apply_hypermile_toggle(params, False)
  assert params.get_bool(PARAM_STEP_DOWN) is True
  assert params.get_bool("NAPHypermileHillClimb") is False


def test_eco_comfort_bias_early_light_not_late_bite():
  """Justin: early gentle regen, not late hard regen that makes people sick."""
  from openpilot.selfdrive.mapd.constants import LOOKAHEAD_EARLY, LOOKAHEAD_LATE, LOOKAHEAD_NORMAL
  from openpilot.selfdrive.mapd.constants import map_brake_a_ms2, map_accel_a_ms2

  current = {
    "NAPAdaptiveAccel": True,
    "NAPMapSpeedMode": ECO_MAP_MODE_CAP,
    "NAPMapSpeedOffsetMph": 0,
    "NAPMapSpeedLookahead": 1,  # Late — the max-bite setting
    "NAPMapSpeedAccel": 5,
  }
  eco = eco_preset_from(current)
  assert eco["NAPMapSpeedMode"] == ECO_MAP_MODE_CAP
  assert eco["NAPMapSpeedLookahead"] == ECO_MAP_LOOKAHEAD_EARLY == LOOKAHEAD_EARLY
  assert eco["NAPMapSpeedLookahead"] != LOOKAHEAD_LATE
  assert eco["NAPMapSpeedLookahead"] != LOOKAHEAD_NORMAL
  assert "NAPMapSpeedOffsetMph" not in eco
  assert eco["NAPMapSpeedAccel"] == ECO_MAP_ACCEL == 1
  # Early brake is lighter than Late; climb Accel 1 is below default 5.
  assert map_brake_a_ms2(LOOKAHEAD_EARLY) < map_brake_a_ms2(LOOKAHEAD_LATE)
  assert map_accel_a_ms2(LOOKAHEAD_EARLY, 1) < map_accel_a_ms2(LOOKAHEAD_NORMAL, 5)


def test_step_down_scales_with_posted_not_flat_minus_fifteen():
  """Hypermile On + Step Down On: same 50→80 scale as eco, −15 at 80."""
  assert STEP_DOWN_MPH == 15.0
  assert read_hypermile_step_down(FakeParams()) is False
  assert step_down_applies(True, False) is False
  assert step_down_applies(False, True) is False
  assert step_down_applies(True, True) is True

  def _approx(a, b):
    assert abs(float(a) - float(b)) < 1e-6, (a, b)

  _approx(step_down_offset_mph(30), 0.0)
  _approx(step_down_offset_mph(50), 0.0)
  _approx(step_down_offset_mph(65), -7.5)
  _approx(step_down_offset_mph(75), -12.5)
  _approx(step_down_offset_mph(80), -15.0)
  _approx(step_down_offset_mph(90), -15.0)
  _approx(step_down_offset_mph(None), 0.0)

  eco_off = -5.0 * CV.MPH_TO_KPH
  posted_75 = 75.0 * CV.MPH_TO_KPH
  # Eco alone still uses the −8 scale (75 → ~68.333). Step Down replaces it.
  _approx(stepped_map_target_kph(
    posted_75, hypermile_on=True, step_down_on=False, map_offset_kph=eco_off,
  ), (75.0 + eco_map_offset_mph(75.0)) * CV.MPH_TO_KPH)

  cases = (
    (30.0, 30.0),
    (50.0, 50.0),
    (65.0, 57.5),
    (75.0, 62.5),
    (80.0, 65.0),
    (90.0, 75.0),
  )
  for posted_mph, want_mph in cases:
    posted = posted_mph * CV.MPH_TO_KPH
    got = stepped_map_target_kph(
      posted, hypermile_on=True, step_down_on=True, map_offset_kph=eco_off,
    )
    _approx(got, want_mph * CV.MPH_TO_KPH)
    # Eco −8 does not stack; user slider is ignored.
    _approx(got, (posted_mph + step_down_offset_mph(posted_mph)) * CV.MPH_TO_KPH)

  # Hypermile Off: step-down param On is inert; user offset still applies.
  _approx(stepped_map_target_kph(
    posted_75, hypermile_on=False, step_down_on=True, map_offset_kph=eco_off,
  ), 70.0 * CV.MPH_TO_KPH)
  # No posted: do not invent a −15 drop.
  _approx(map_target_offset_kph(eco_off, hypermile_on=True, step_down_on=True), 0.0)
  # Never more than 15 under; never above posted while stepping down.
  raw = 80.0 * CV.MPH_TO_KPH
  target = stepped_map_target_kph(raw, hypermile_on=True, step_down_on=True, map_offset_kph=-50.0)
  _approx(target, 65.0 * CV.MPH_TO_KPH)
  assert target <= raw
  _approx(raw - target, 15.0 * CV.MPH_TO_KPH)


def test_eco_offset_scales_with_posted_not_flat_minus_five():
  """Hypermile On + Step Down Off: town stays posted; highway eases to −8."""
  assert ECO_OFFSET_START_MPH == 50.0
  assert ECO_OFFSET_FULL_MPH == 80.0
  assert ECO_MAP_OFFSET_MPH == -8

  def _approx(a, b):
    assert abs(float(a) - float(b)) < 1e-6, (a, b)

  # mph-domain scale (the posted/OSM limit, not a param write).
  _approx(eco_map_offset_mph(30), 0.0)
  _approx(eco_map_offset_mph(49.9), 0.0)
  _approx(eco_map_offset_mph(50), 0.0)
  _approx(eco_map_offset_mph(65), -4.0)
  _approx(eco_map_offset_mph(80), -8.0)
  _approx(eco_map_offset_mph(90), -8.0)
  _approx(eco_map_offset_mph(None), 0.0)
  _approx(eco_map_offset_mph(0), 0.0)

  user_plus_five = 5.0 * CV.MPH_TO_KPH
  user_minus_five = -5.0 * CV.MPH_TO_KPH

  cases = (
    (30.0, 30.0),
    (50.0, 50.0),
    (65.0, 61.0),
    (80.0, 72.0),
    (90.0, 82.0),
  )
  for posted_mph, want_mph in cases:
    posted = posted_mph * CV.MPH_TO_KPH
    # User slider is ignored while Hypermile is On (Step Down Off).
    got = stepped_map_target_kph(
      posted, hypermile_on=True, step_down_on=False, map_offset_kph=user_plus_five,
    )
    _approx(got, want_mph * CV.MPH_TO_KPH)
    got_neg = stepped_map_target_kph(
      posted, hypermile_on=True, step_down_on=False, map_offset_kph=user_minus_five,
    )
    _approx(got_neg, want_mph * CV.MPH_TO_KPH)
    _approx(
      map_target_offset_kph(
        user_minus_five, hypermile_on=True, step_down_on=False, posted_kph=posted,
      ),
      eco_map_offset_mph(posted_mph) * CV.MPH_TO_KPH,
    )

  # No posted: do not invent a town drop.
  _approx(map_target_offset_kph(user_minus_five, hypermile_on=True, step_down_on=False), 0.0)
  # Hypermile Off: user offset still applies (30→25 if they chose −5).
  _approx(stepped_map_target_kph(
    30.0 * CV.MPH_TO_KPH, hypermile_on=False, step_down_on=False, map_offset_kph=user_minus_five,
  ), 25.0 * CV.MPH_TO_KPH)
  # Step Down On: town stays 30; 80→65 (replaces eco, no stack).
  _approx(stepped_map_target_kph(
    30.0 * CV.MPH_TO_KPH, hypermile_on=True, step_down_on=True, map_offset_kph=user_plus_five,
  ), 30.0 * CV.MPH_TO_KPH)
  _approx(stepped_map_target_kph(
    80.0 * CV.MPH_TO_KPH, hypermile_on=True, step_down_on=True, map_offset_kph=0.0,
  ), 65.0 * CV.MPH_TO_KPH)


def test_hypermile_offsets_maps_only_no_invent_without_posted():
  """Eco / Step Down never invent a drop without maps + a real posted limit."""
  user_minus_five = -5.0 * CV.MPH_TO_KPH
  posted_80 = 80.0 * CV.MPH_TO_KPH

  def _approx(a, b):
    assert abs(float(a) - float(b)) < 1e-6, (a, b)

  assert maps_posted_known(None) is False
  assert maps_posted_known(0.0) is False
  assert maps_posted_known(-1.0) is False
  assert maps_posted_known(posted_80, maps_posted=False) is False
  assert maps_posted_known(posted_80, maps_posted=True) is True
  assert maps_posted_known(30.0 * CV.MPH_TO_KPH) is True

  unknown = (None, 0.0, -1.0)
  for posted in unknown:
    for step in (False, True):
      _approx(map_target_offset_kph(
        user_minus_five, hypermile_on=True, step_down_on=step, posted_kph=posted,
      ), 0.0)
  # Maps off / no match: even a leftover 80 mph number must not drop MAX.
  _approx(map_target_offset_kph(
    user_minus_five, hypermile_on=True, step_down_on=False,
    posted_kph=posted_80, maps_posted=False,
  ), 0.0)
  _approx(map_target_offset_kph(
    user_minus_five, hypermile_on=True, step_down_on=True,
    posted_kph=posted_80, maps_posted=False,
  ), 0.0)
  # Maps + known posted: scale applies.
  _approx(map_target_offset_kph(
    user_minus_five, hypermile_on=True, step_down_on=False,
    posted_kph=posted_80, maps_posted=True,
  ), -8.0 * CV.MPH_TO_KPH)
  _approx(map_target_offset_kph(
    user_minus_five, hypermile_on=True, step_down_on=True,
    posted_kph=posted_80, maps_posted=True,
  ), -15.0 * CV.MPH_TO_KPH)
  # Town 30 with maps: still no drop.
  _approx(stepped_map_target_kph(
    30.0 * CV.MPH_TO_KPH, hypermile_on=True, step_down_on=False,
  ), 30.0 * CV.MPH_TO_KPH)
  _approx(stepped_map_target_kph(
    30.0 * CV.MPH_TO_KPH, hypermile_on=True, step_down_on=True,
  ), 30.0 * CV.MPH_TO_KPH)
  # Hypermile Off: user slider still returned (apply_map_speed ignores it
  # when map_kph is None — no invented posted).
  _approx(map_target_offset_kph(
    user_minus_five, hypermile_on=False, step_down_on=True, posted_kph=None,
    maps_posted=False,
  ), user_minus_five)


def test_effective_follow_is_stock_slider_hypermile_or_not():
  """Hypermile no longer remaps follow — planner uses NAPFollowDistance 1–7."""
  for dist in range(FOLLOW_DISTANCE_MIN, FOLLOW_DISTANCE_MAX + 1):
    assert effective_nap_follow_dist(True, dist) == dist
  # Invalid / missing slider → personality fallback.
  assert effective_nap_follow_dist(True, 0) is None
  assert effective_nap_follow_dist(True, 8) is None
  assert effective_nap_follow_dist(True, None) is None
  assert effective_nap_follow_dist(False, 4) is None
  # Closest stock 1 is allowed (old Hypermile floor of 2 is gone).
  assert effective_nap_follow_dist(True, 1) == 1
  assert NAP_T_FOLLOW[0] < NAP_T_FOLLOW[1]


def test_stalk_with_lead_writes_nap_follow_distance_on_and_off():
  """Stalk + radar lead writes NAPFollowDistance whether Hypermile is On or Off."""
  assert step_follow_distance(4, closer=True) == 3
  assert step_follow_distance(1, closer=True) == 1
  assert step_follow_distance(7, closer=False) == 7
  assert stalk_adjusts_follow(has_lead=True) is True
  assert stalk_adjusts_follow(has_lead=False) is False

  for hm_on in (False, True):
    params = FakeParams(bools={PARAM_HYPERMILE: hm_on}, ints={PARAM_FOLLOW_DISTANCE: 4})
    assert persist_follow_distance(params, closer=True) == 3
    assert params.get(PARAM_FOLLOW_DISTANCE) == 3
    persist_follow_distance(params, closer=True)
    persist_follow_distance(params, closer=True)
    persist_follow_distance(params, closer=True)
    assert params.get(PARAM_FOLLOW_DISTANCE) == 1
    persist_follow_distance(params, closer=False)
    assert params.get(PARAM_FOLLOW_DISTANCE) == 2

    # 1 kph tip: undo MAX on press; Follow commits on return to IDLE.
    tip_prev = 100.0
    tip_cur = tip_prev - 1.0
    g = FollowStalkGesture()
    level, undo = consume_follow_stalk(
      params, has_lead=True, button_closer=False, raw_kph=tip_cur, prev_raw_kph=tip_prev,
      detent=CRUISE_STALK_DN_1ST, gesture=g,
    )
    assert level is None
    assert undo == tip_prev
    assert params.get(PARAM_FOLLOW_DISTANCE) == 2
    level, undo = consume_follow_stalk(
      params, has_lead=True, button_closer=None, raw_kph=tip_prev, prev_raw_kph=tip_prev,
      detent=CRUISE_STALK_IDLE, gesture=g,
    )
    assert level == 3
    assert undo is None
    assert params.get(PARAM_FOLLOW_DISTANCE) == 3

    # A Follow/posted MAX jump is not a 1/5 mph stalk step.
    jump_level, jump_undo = consume_follow_stalk(
      params, has_lead=True, button_closer=None, raw_kph=120.0, prev_raw_kph=100.0,
    )
    assert jump_level is None and jump_undo is None
    assert params.get(PARAM_FOLLOW_DISTANCE) == 3


def test_stalk_no_lead_leaves_max_and_does_not_write_follow():
  """No radar lead: tip and hold both stay MAX / RES+/−. Follow Distance is not written."""
  tip_prev = 65.0 * CV.MPH_TO_KPH
  tip_cur = tip_prev + 1.0 * CV.MPH_TO_KPH
  hold_cur = tip_prev + 5.0 * CV.MPH_TO_KPH
  for hm_on in (False, True):
    params = FakeParams(bools={PARAM_HYPERMILE: hm_on}, ints={PARAM_FOLLOW_DISTANCE: 4})
    before = params.get(PARAM_FOLLOW_DISTANCE)
    none_level, none_undo = consume_follow_stalk(
      params, has_lead=False, button_closer=True, raw_kph=tip_cur, prev_raw_kph=tip_prev,
    )
    assert none_level is None and none_undo is None
    assert params.get(PARAM_FOLLOW_DISTANCE) == before
    hold_level, hold_undo = consume_follow_stalk(
      params, has_lead=False, button_closer=True, raw_kph=hold_cur, prev_raw_kph=tip_prev,
    )
    assert hold_level is None and hold_undo is None
    assert params.get(PARAM_FOLLOW_DISTANCE) == before
    is_stalk, closer, undo = detect_follow_stalk(
      has_lead=False, button_closer=True, raw_kph=tip_cur, prev_raw_kph=tip_prev,
    )
    assert is_stalk is False and closer is None and undo is None


def test_lead_tip_remaps_follow_hold_keeps_max():
  """Lead + 1 mph tip → Follow Distance; lead + 5 mph hold → MAX kept."""
  tip_prev = 65.0 * CV.MPH_TO_KPH
  tip_cur = tip_prev + 1.0 * CV.MPH_TO_KPH
  hold_cur = tip_prev + 5.0 * CV.MPH_TO_KPH
  metric_tip = 100.0 + 1.0
  metric_hold = 100.0 + 5.0

  params = FakeParams(bools={PARAM_HYPERMILE: False}, ints={PARAM_FOLLOW_DISTANCE: 4})
  g = FollowStalkGesture()
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=None, raw_kph=tip_cur, prev_raw_kph=tip_prev,
    detent=CRUISE_STALK_UP_1ST, gesture=g,
  )
  assert level is None
  assert undo == tip_prev
  assert params.get(PARAM_FOLLOW_DISTANCE) == 4
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=None, raw_kph=tip_prev, prev_raw_kph=tip_prev,
    detent=CRUISE_STALK_IDLE, gesture=g,
  )
  assert level == 3
  assert undo is None
  assert params.get(PARAM_FOLLOW_DISTANCE) == 3

  # Metric 1 kph tip also remaps (and undoes MAX) after IDLE.
  g = FollowStalkGesture()
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=None, raw_kph=metric_tip, prev_raw_kph=100.0,
    detent=CRUISE_STALK_UP_1ST, gesture=g,
  )
  assert level is None
  assert undo == 100.0
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=None, raw_kph=100.0, prev_raw_kph=100.0,
    detent=CRUISE_STALK_IDLE, gesture=g,
  )
  assert level == 2
  assert params.get(PARAM_FOLLOW_DISTANCE) == 2

  # 5 mph full press: MAX kept, Follow Distance unchanged.
  before = params.get(PARAM_FOLLOW_DISTANCE)
  hold_level, hold_undo = consume_follow_stalk(
    params, has_lead=True, button_closer=None, raw_kph=hold_cur, prev_raw_kph=tip_prev,
    detent=CRUISE_STALK_UP_2ND,
  )
  assert hold_level is None and hold_undo is None
  assert params.get(PARAM_FOLLOW_DISTANCE) == before

  # Metric 5 kph hold, even with a button edge: delta is source of truth.
  hold_level, hold_undo = consume_follow_stalk(
    params, has_lead=True, button_closer=True, raw_kph=metric_hold, prev_raw_kph=100.0,
  )
  assert hold_level is None and hold_undo is None
  assert params.get(PARAM_FOLLOW_DISTANCE) == before

  is_stalk, closer, undo = detect_follow_stalk(
    has_lead=True, button_closer=True, raw_kph=hold_cur, prev_raw_kph=tip_prev,
  )
  assert is_stalk is False and closer is None and undo is None


def test_lead_tip_then_2nd_detent_keeps_max_and_does_not_remap_follow():
  """Full press walks through first detent; Follow must not change."""
  tip_prev = 65.0 * CV.MPH_TO_KPH
  tip_cur = tip_prev + 1.0 * CV.MPH_TO_KPH
  hold_cur = tip_prev + 5.0 * CV.MPH_TO_KPH
  params = FakeParams(bools={PARAM_HYPERMILE: False}, ints={PARAM_FOLLOW_DISTANCE: 4})
  g = FollowStalkGesture()

  # Frame 1: physical lever hits first detent. Undo the +1 MAX, do not write Follow.
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=True, raw_kph=tip_cur, prev_raw_kph=tip_prev,
    detent=CRUISE_STALK_UP_1ST, gesture=g,
  )
  assert level is None
  assert undo == tip_prev
  assert params.get(PARAM_FOLLOW_DISTANCE) == 4
  assert g.is_pending is True

  # Frame 2: 2nd detent / +5. Cancel pending Follow; keep MAX.
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=True, raw_kph=hold_cur, prev_raw_kph=tip_prev,
    detent=CRUISE_STALK_UP_2ND, gesture=g,
  )
  assert level is None and undo is None
  assert params.get(PARAM_FOLLOW_DISTANCE) == 4
  assert g.is_pending is False

  # Release to IDLE must not commit the canceled tip.
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=None, raw_kph=hold_cur, prev_raw_kph=hold_cur,
    detent=CRUISE_STALK_IDLE, button_released=True, gesture=g,
  )
  assert level is None and undo is None
  assert params.get(PARAM_FOLLOW_DISTANCE) == 4


def test_button_only_press_is_not_a_completed_tip():
  """Pre-AP buttonEvents collapse tip and 2nd detent — press is not a tip."""
  params = FakeParams(bools={PARAM_HYPERMILE: False}, ints={PARAM_FOLLOW_DISTANCE: 4})
  g = FollowStalkGesture()
  is_stalk, closer, undo = detect_follow_stalk(
    has_lead=True, button_closer=True, raw_kph=None, prev_raw_kph=None, gesture=g,
  )
  assert is_stalk is False and closer is None and undo is None
  assert g.is_pending is True
  assert params.get(PARAM_FOLLOW_DISTANCE) == 4

  # Release without 2nd detent / 5 mph: one Follow step. Press+release must not double-step.
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=None, raw_kph=None, prev_raw_kph=None,
    button_released=True, gesture=g,
  )
  assert level == 3
  assert undo is None
  assert params.get(PARAM_FOLLOW_DISTANCE) == 3

  # A second press while pending (no detent) is treated as 2nd detent.
  g = FollowStalkGesture()
  consume_follow_stalk(
    params, has_lead=True, button_closer=True, raw_kph=None, prev_raw_kph=None, gesture=g,
  )
  before = params.get(PARAM_FOLLOW_DISTANCE)
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=True, raw_kph=None, prev_raw_kph=None, gesture=g,
  )
  assert level is None and undo is None
  assert params.get(PARAM_FOLLOW_DISTANCE) == before
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=None, raw_kph=None, prev_raw_kph=None,
    button_released=True, gesture=g,
  )
  assert level is None
  assert params.get(PARAM_FOLLOW_DISTANCE) == before


def test_follow_hud_announces_every_param_change_after_seed():
  """First read seeds; every later 1–7 change is a Follow Distance toast."""
  prev, announce = poll_follow_distance_hud(None, 4)
  assert prev == 4 and announce is False
  prev, announce = poll_follow_distance_hud(prev, 3)
  assert prev == 3 and announce is True
  prev, announce = poll_follow_distance_hud(prev, 3)
  assert announce is False
  prev, announce = poll_follow_distance_hud(prev, 2)
  assert prev == 2 and announce is True
  prev, announce = poll_follow_distance_hud(prev, None)
  assert prev == 2 and announce is False
  prev, announce = poll_follow_distance_hud(None, None)
  assert prev is None and announce is False


def test_persist_follow_marks_hud_pending_even_at_limit():
  """Tip already at 1 or 7 still requests the Follow Distance HUD."""
  from openpilot.selfdrive.controls.lib.hypermile import (
    PARAM_FOLLOW_DISTANCE, PARAM_FOLLOW_HUD_PENDING, persist_follow_distance,
  )

  class P:
    def __init__(self, level):
      self.d = {PARAM_FOLLOW_DISTANCE: level, PARAM_FOLLOW_HUD_PENDING: False}
    def get(self, key, return_default=True):
      return self.d.get(key)
    def put(self, key, value):
      self.d[key] = value
    def put_bool(self, key, value):
      self.d[key] = bool(value)
    def get_bool(self, key):
      return bool(self.d.get(key))

  p = P(1)
  assert persist_follow_distance(p, closer=True) == 1
  assert p.get(PARAM_FOLLOW_DISTANCE) == 1
  assert p.get_bool(PARAM_FOLLOW_HUD_PENDING) is True
  p = P(7)
  assert persist_follow_distance(p, closer=False) == 7
  assert p.get_bool(PARAM_FOLLOW_HUD_PENDING) is True


def test_button_events_and_hud_text():
  up = SimpleNamespace(type=SimpleNamespace(name="accelCruise"), pressed=True)
  down = SimpleNamespace(type=SimpleNamespace(name="decelCruise"), pressed=True)
  release = SimpleNamespace(type=SimpleNamespace(name="accelCruise"), pressed=False)
  assert button_event_closer([up]) is True
  assert button_event_closer([down]) is False
  assert button_event_closer([release]) is None
  assert button_event_released([release]) is True
  assert button_event_released([up]) is False
  assert follow_distance_hud_text(3) == "Follow Distance: 3"
  assert follow_distance_hud_text(1) == "Follow Distance: 1"
  is_stalk, closer, undo = detect_follow_stalk(
    has_lead=True, button_closer=True, raw_kph=None, prev_raw_kph=None,
  )
  assert is_stalk is False and closer is None and undo is None


def test_settings_and_docs_wire_hypermile():
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  tici = (root / "selfdrive/ui/layouts/settings/driving_mannerisms.py").read_text()
  mici = (root / "selfdrive/ui/mici/layouts/settings/driving_mannerisms.py").read_text()
  nap = (root / "selfdrive/ui/layouts/settings/nap.py").read_text()
  content = (root / "selfdrive/ui/layouts/settings/nap_content.py").read_text()
  keys = (root / "common/params_keys.h").read_text()
  planner = (root / "selfdrive/controls/lib/longitudinal_planner.py").read_text()
  card = (root / "selfdrive/car/card.py").read_text()
  events = (root / "selfdrive/selfdrived/events.py").read_text()
  selfdrived = (root / "selfdrive/selfdrived/selfdrived.py").read_text()
  docs = (root / "docs-nap/hypermile.md").read_text()
  readme = (root / "docs-nap/README.md").read_text()
  releases = (root / "RELEASES.md").read_text()

  for src in (tici, mici):
    assert "Hypermile" in src or "hypermile" in src
    assert "NAP_HYPERMILE" in src
    assert "NAP_HYPERMILE_STEP_DOWN" in src
    assert "NAP_HYPERMILE_HILL_CLIMB" in src
    assert "apply_hypermile_toggle" in src
  assert "Hypermile" in nap
  assert "put_bool(NAP_HYPERMILE, False)" in nap
  assert "put_bool(NAP_HYPERMILE_STEP_DOWN, False)" in nap
  assert "put_bool(NAP_HYPERMILE_HILL_CLIMB, True)" in nap
  assert "Step Down Speed" in tici
  assert "Hill Climb" in tici
  assert "step down speed" in mici
  assert "hill climb" in mici
  assert "map_target_offset_kph" in card
  assert "_live_map_offset_kph" in card
  assert "maps_posted" in card
  assert "never invent" in card
  assert "_refresh_map_speed_params" in card
  hm_src = (root / "selfdrive/controls/lib/hypermile.py").read_text()
  assert "eco_map_offset_mph" in hm_src
  assert "maps_posted_known" in hm_src
  assert "ECO_OFFSET_START_MPH" in hm_src
  assert '"NAPMapSpeedOffsetMph": ECO_MAP_OFFSET_MPH' not in hm_src
  assert "NAPHypermile" in keys
  assert "NAPOnePedalLong" in keys
  assert 'BOOL, "0"' in next(ln for ln in keys.splitlines() if '"NAPOnePedalLong"' in ln)
  assert "NAPHypermileFollowLevel" not in keys
  assert "NAPHypermileFollowLevel" not in tici
  assert "NAPHypermileFollowLevel" not in mici
  assert "NAPHypermileFollowLevel" not in nap
  assert "NAPHypermileFollowLevel" not in content
  assert "Hypermile Follow" not in tici
  assert "hypermile follow" not in mici
  assert "set_visible(not hypermile_on)" not in tici
  assert "FOLLOW_DISTANCE" in tici
  assert "FOLLOW_DISTANCE" in mici
  assert "NAPMapSpeedAccel" in tici
  assert "NAPMapSpeedAccel" in mici
  assert "Acceleration" in tici
  assert '"acceleration"' in mici
  assert "NAPHypermileStepDown" in keys
  assert "NAPHypermileHillClimb" in keys
  assert 'BOOL, "0"' in next(ln for ln in keys.splitlines() if '"NAPHypermileStepDown"' in ln)
  assert 'BOOL, "1"' in next(ln for ln in keys.splitlines() if '"NAPHypermileHillClimb"' in ln)
  assert 'BOOL, "0"' in next(ln for ln in keys.splitlines() if '"NAPHypermile"' in ln)
  assert "NAPFollowDistance" in keys
  assert 'INT, "4"' in next(ln for ln in keys.splitlines() if '"NAPFollowDistance"' in ln)
  assert "effective_nap_follow_dist" in planner
  assert "Stalk Follow Distance 1–7 must land on the next plan" in planner
  assert "FollowStalkGesture" in card
  assert "_preap_cruise_detent" in card
  assert "persist_follow_distance" in card
  assert "radarState" in card
  assert "hypermileFollowChanged" in events
  assert "follow_distance_hud_text" in events
  assert "poll_follow_distance_hud" in selfdrived
  assert "NAPFollowDistance" in selfdrived
  assert "NAPFollowHudPending" in selfdrived
  assert "_follow_hud_until" in selfdrived
  assert "NAPFollowHudPending" in keys
  assert "ET.PERMANENT: hypermile_follow_changed_alert" in events
  assert "NAPHypermile" in content
  assert "comfort-biased" in content.lower()
  assert "not max" in content.lower()
  assert "stock slider" in content.lower()
  assert "hypermile.md" in readme
  assert "Hypermile" in docs
  assert "NAPHypermileFollowLevel" not in docs
  assert "Follow Distance: N" in docs
  assert "NAPFollowDistance" in docs
  assert "1 mph" in docs and "5 mph" in docs
  assert "full press" in docs.lower() or "full press" in releases.lower()
  assert "Step Down Speed" in docs
  assert "Hill Climb" in docs
  assert "15 mph under" in docs or "−15 at 80" in docs
  assert "step_down_offset_mph" in hm_src
  assert "early, light regenerative" in docs.lower() or "early, light" in docs.lower()
  assert "not maximum regen" in docs.lower() or "not max regen" in docs.lower()
  assert "posted-scaled" in docs.lower() or "scaled" in docs.lower()
  assert "30 stays 30" in docs or "town 30" in docs.lower()
  assert "−8" in docs
  assert "maps" in docs.lower() and ("unknown" in docs.lower() or "invent" in docs.lower())
  # "NAP Hypermile Hill Climb" must not steal the scaled-eco RELEASES block.
  hm_rel = next(p for p in releases.split("\n\n") if p.startswith("NAP Hypermile ("))
  assert "Hypermile" in hm_rel
  assert "Early" in hm_rel
  assert "30" in hm_rel and "25" in hm_rel
  assert "−8" in hm_rel
  assert "−15" in hm_rel
  assert "Maps-only" in hm_rel or "invent" in hm_rel.lower()
  assert "nap-release" not in docs.lower() or "not a nap-release" in docs.lower()
