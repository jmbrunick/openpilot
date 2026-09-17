"""Stock Follow Distance stalk remap: lead → 1–7, no lead → MAX.

Not Hypermile. No 1–5 band, no ≤50 forced far gap, no eco / Step Down.
"""
from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.follow_stalk import (
  CRUISE_STALK_DN_1ST,
  CRUISE_STALK_IDLE,
  CRUISE_STALK_UP_1ST,
  CRUISE_STALK_UP_2ND,
  FOLLOW_DEFAULT,
  FOLLOW_MAX,
  FOLLOW_MIN,
  FollowStalkGesture,
  PARAM_FOLLOW,
  STALK_COOLDOWN_S,
  button_event_closer,
  button_event_released,
  clamp_follow_distance,
  consume_follow_stalk,
  detect_follow_stalk,
  follow_distance_hud_text,
  persist_follow_distance,
  poll_follow_distance_hud,
  PARAM_FOLLOW_HUD_PENDING,
  stalk_adjusts_follow,
  step_follow_distance,
)


class FakeParams:
  def __init__(self, follow=FOLLOW_DEFAULT):
    self._ints = {PARAM_FOLLOW: int(follow)}
    self._bools = {}

  def get(self, key, return_default=False):
    if key in self._ints:
      return self._ints[key]
    return FOLLOW_DEFAULT if return_default else None

  def put(self, key, value):
    self._ints[key] = int(value)

  def put_bool(self, key, value):
    self._bools[key] = bool(value)

  def get_bool(self, key):
    return bool(self._bools.get(key, False))


def _tip_then_idle(params, *, closer, raw_kph, prev_raw_kph, apply=True):
  detent = CRUISE_STALK_UP_1ST if closer else CRUISE_STALK_DN_1ST
  g = FollowStalkGesture()
  press_level, press_undo = consume_follow_stalk(
    params, has_lead=True, button_closer=closer, raw_kph=raw_kph, prev_raw_kph=prev_raw_kph,
    detent=detent, gesture=g, apply=apply,
  )
  idle_level, idle_undo = consume_follow_stalk(
    params, has_lead=True, button_closer=None, raw_kph=prev_raw_kph, prev_raw_kph=prev_raw_kph,
    detent=CRUISE_STALK_IDLE, gesture=g, apply=apply,
  )
  return press_level, press_undo, idle_level, idle_undo


def test_lead_present_stalk_writes_nap_follow_distance_and_undoes_max():
  params = FakeParams(follow=4)
  # 1 kph tip: undo MAX on press; Follow commits on return to IDLE.
  press_level, press_undo, level, idle_undo = _tip_then_idle(
    params, closer=True, raw_kph=101.0, prev_raw_kph=100.0,
  )
  assert press_level is None
  assert press_undo == 100.0
  assert level == 3
  assert idle_undo is None
  assert params.get(PARAM_FOLLOW) == 3

  press_level, press_undo, level, idle_undo = _tip_then_idle(
    params, closer=False, raw_kph=99.0, prev_raw_kph=100.0,
  )
  assert press_level is None
  assert press_undo == 100.0
  assert level == 4
  assert params.get(PARAM_FOLLOW) == 4


def test_no_lead_leaves_max_step_untouched():
  params = FakeParams(follow=4)
  before = params.get(PARAM_FOLLOW)
  tip_prev = 65.0 * CV.MPH_TO_KPH
  tip_cur = tip_prev + 1.0 * CV.MPH_TO_KPH
  hold_cur = tip_prev + 5.0 * CV.MPH_TO_KPH
  level, undo = consume_follow_stalk(
    params, has_lead=False, button_closer=True, raw_kph=tip_cur, prev_raw_kph=tip_prev,
  )
  assert level is None and undo is None
  assert params.get(PARAM_FOLLOW) == before
  hold_level, hold_undo = consume_follow_stalk(
    params, has_lead=False, button_closer=True, raw_kph=hold_cur, prev_raw_kph=tip_prev,
  )
  assert hold_level is None and hold_undo is None
  assert params.get(PARAM_FOLLOW) == before
  assert stalk_adjusts_follow(has_lead=False) is False
  assert stalk_adjusts_follow(has_lead=True) is True


def test_lead_tip_remaps_follow_hold_keeps_max():
  """Lead + 1 mph tip → Follow Distance; lead + 5 mph hold → MAX kept."""
  tip_prev = 65.0 * CV.MPH_TO_KPH
  tip_cur = tip_prev + 1.0 * CV.MPH_TO_KPH
  hold_cur = tip_prev + 5.0 * CV.MPH_TO_KPH

  params = FakeParams(follow=4)
  press_level, press_undo, level, idle_undo = _tip_then_idle(
    params, closer=True, raw_kph=tip_cur, prev_raw_kph=tip_prev,
  )
  assert press_level is None
  assert press_undo == tip_prev
  assert level == 3
  assert idle_undo is None
  assert params.get(PARAM_FOLLOW) == 3

  before = params.get(PARAM_FOLLOW)
  hold_level, hold_undo = consume_follow_stalk(
    params, has_lead=True, button_closer=None, raw_kph=hold_cur, prev_raw_kph=tip_prev,
  )
  assert hold_level is None and hold_undo is None
  assert params.get(PARAM_FOLLOW) == before

  # 5 mph + button edge: delta is source of truth — still MAX, not follow.
  hold_level, hold_undo = consume_follow_stalk(
    params, has_lead=True, button_closer=True, raw_kph=hold_cur, prev_raw_kph=tip_prev,
  )
  assert hold_level is None and hold_undo is None
  assert params.get(PARAM_FOLLOW) == before

  is_stalk, closer, undo = detect_follow_stalk(
    FakeParams(follow=4),
    has_lead=True, button_closer=True, raw_kph=hold_cur, prev_raw_kph=tip_prev,
  )
  assert is_stalk is False and closer is None and undo is None


def test_full_stock_one_to_seven_including_closest():
  assert FOLLOW_MIN == 1 and FOLLOW_MAX == 7
  assert step_follow_distance(4, closer=True) == 3
  assert step_follow_distance(1, closer=True) == 1
  assert step_follow_distance(7, closer=False) == 7
  assert clamp_follow_distance(0) == 1
  assert clamp_follow_distance(8) == 7
  assert clamp_follow_distance(None) == FOLLOW_DEFAULT

  params = FakeParams(follow=4)
  persist_follow_distance(params, closer=True)
  persist_follow_distance(params, closer=True)
  persist_follow_distance(params, closer=True)
  assert params.get(PARAM_FOLLOW) == 1
  persist_follow_distance(params, closer=True)
  assert params.get(PARAM_FOLLOW) == 1
  for _ in range(8):
    persist_follow_distance(params, closer=False)
  assert params.get(PARAM_FOLLOW) == 7


def test_no_50_mph_forced_far_gap():
  """Stock 1–7 is the same below and above 50 mph. Hypermile split is not here."""
  from pathlib import Path
  src = (Path(__file__).resolve().parents[1] / "lib/follow_stalk.py").read_text()
  assert "NAPHypermile" not in src
  assert "PARAM_HYPERMILE" not in src
  assert "SAFE_FLOOR" not in src
  assert "SPLIT" not in src
  for v_ego_mph in (30.0, 50.0, 65.0):
    _ = v_ego_mph * CV.MPH_TO_MS
    params = FakeParams(follow=1)
    press_level, press_undo, level, idle_undo = _tip_then_idle(
      params, closer=False, raw_kph=99.0, prev_raw_kph=100.0,
    )
    assert press_level is None
    assert press_undo == 100.0
    assert level == 2
    assert params.get(PARAM_FOLLOW) == 2


def test_posted_jump_is_not_a_stalk_step():
  params = FakeParams(follow=4)
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=None, raw_kph=120.0, prev_raw_kph=100.0,
  )
  assert level is None and undo is None
  assert params.get(PARAM_FOLLOW) == 4


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
  assert follow_distance_hud_text(7) == "Follow Distance: 7"


def test_follow_hud_announces_every_param_change_after_seed():
  prev, announce = poll_follow_distance_hud(None, 4)
  assert prev == 4 and announce is False
  prev, announce = poll_follow_distance_hud(prev, 3)
  assert prev == 3 and announce is True
  prev, announce = poll_follow_distance_hud(prev, 3)
  assert announce is False
  prev, announce = poll_follow_distance_hud(prev, None)
  assert prev == 3 and announce is False


def test_persist_follow_marks_hud_pending_even_at_limit():
  p = FakeParams(follow=1)
  assert persist_follow_distance(p, closer=True) == 1
  assert p.get(PARAM_FOLLOW) == 1
  assert p.get_bool(PARAM_FOLLOW_HUD_PENDING) is True
  p = FakeParams(follow=7)
  assert persist_follow_distance(p, closer=False) == 7
  assert p.get_bool(PARAM_FOLLOW_HUD_PENDING) is True
  is_stalk, closer, undo = detect_follow_stalk(
    FakeParams(follow=4),
    has_lead=True, button_closer=True, raw_kph=None, prev_raw_kph=None,
  )
  assert is_stalk is False and closer is None and undo is None


def test_cooldown_detects_without_writing():
  params = FakeParams(follow=4)
  press_level, press_undo, level, idle_undo = _tip_then_idle(
    params, closer=True, raw_kph=101.0, prev_raw_kph=100.0, apply=False,
  )
  assert press_level is None
  assert press_undo == 100.0
  assert level == 4
  assert params.get(PARAM_FOLLOW) == 4
  assert STALK_COOLDOWN_S == 0.25


def test_lead_tip_then_2nd_detent_keeps_max_and_does_not_remap_follow():
  """Full press walks through first detent; Follow must not change."""
  tip_prev = 65.0 * CV.MPH_TO_KPH
  tip_cur = tip_prev + 1.0 * CV.MPH_TO_KPH
  hold_cur = tip_prev + 5.0 * CV.MPH_TO_KPH
  params = FakeParams(follow=4)
  g = FollowStalkGesture()

  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=True, raw_kph=tip_cur, prev_raw_kph=tip_prev,
    detent=CRUISE_STALK_UP_1ST, gesture=g,
  )
  assert level is None
  assert undo == tip_prev
  assert params.get(PARAM_FOLLOW) == 4
  assert g.is_pending is True

  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=True, raw_kph=hold_cur, prev_raw_kph=tip_prev,
    detent=CRUISE_STALK_UP_2ND, gesture=g,
  )
  assert level is None and undo is None
  assert params.get(PARAM_FOLLOW) == 4

  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=None, raw_kph=hold_cur, prev_raw_kph=hold_cur,
    detent=CRUISE_STALK_IDLE, button_released=True, gesture=g,
  )
  assert level is None and undo is None
  assert params.get(PARAM_FOLLOW) == 4


def test_button_only_press_is_not_a_completed_tip():
  """Pre-AP buttonEvents collapse tip and 2nd detent — press is not a tip."""
  params = FakeParams(follow=4)
  g = FollowStalkGesture()
  is_stalk, closer, undo = detect_follow_stalk(
    params, has_lead=True, button_closer=True, raw_kph=None, prev_raw_kph=None, gesture=g,
  )
  assert is_stalk is False and closer is None and undo is None
  assert g.is_pending is True

  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=None, raw_kph=None, prev_raw_kph=None,
    button_released=True, gesture=g,
  )
  assert level == 3
  assert params.get(PARAM_FOLLOW) == 3

  g = FollowStalkGesture()
  consume_follow_stalk(
    params, has_lead=True, button_closer=True, raw_kph=None, prev_raw_kph=None, gesture=g,
  )
  before = params.get(PARAM_FOLLOW)
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=True, raw_kph=None, prev_raw_kph=None, gesture=g,
  )
  assert level is None
  assert params.get(PARAM_FOLLOW) == before
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=None, raw_kph=None, prev_raw_kph=None,
    button_released=True, gesture=g,
  )
  assert level is None
  assert params.get(PARAM_FOLLOW) == before


def test_card_and_hud_wire_stock_follow_only():
  from pathlib import Path

  root = Path(__file__).resolve().parents[3]
  card = (root / "selfdrive/car/card.py").read_text()
  events = (root / "selfdrive/selfdrived/events.py").read_text()
  selfdrived = (root / "selfdrive/selfdrived/selfdrived.py").read_text()
  cereal = (root / "cereal/log.capnp").read_text()
  manner = (root / "selfdrive/ui/layouts/settings/driving_mannerisms.py").read_text()
  manner_mici = (root / "selfdrive/ui/mici/layouts/settings/driving_mannerisms.py").read_text()
  content = (root / "selfdrive/ui/layouts/settings/nap_content.py").read_text()
  helper = (root / "selfdrive/controls/lib/follow_stalk.py").read_text()

  assert "from openpilot.selfdrive.controls.lib.follow_stalk import" in card
  assert "_maybe_follow_stalk" in card
  assert "radarState" in card
  assert "stalk_pressed = (not follow_stalk) and self._preap_stalk_set_pressed(CS)" in card
  assert "STALK_COOLDOWN_S" in card
  assert "FollowStalkGesture" in card
  assert "_preap_cruise_detent" in card
  assert "persist_follow_distance" in card
  assert "_write_preap_pedal_speed(CS, undo)" in card
  assert "hypermile" not in card.lower()
  assert "NAPHypermile" not in card

  assert "followDistanceChanged" in cereal
  assert "follow_distance_changed_alert" in events
  assert "follow_distance_hud_text" in events
  assert "followDistanceChanged" in selfdrived
  assert "NAPFollowDistance" in selfdrived
  assert "NAPFollowHudPending" in selfdrived
  assert "_follow_hud_until" in selfdrived
  assert "poll_follow_distance_hud" in selfdrived
  assert "ET.PERMANENT: follow_distance_changed_alert" in events
  keys = (root / "common/params_keys.h").read_text()
  assert "NAPFollowHudPending" in keys
  releases = (root / "RELEASES.md").read_text()
  docs = (root / "docs-nap/engagement.md").read_text()
  assert "Follow Distance: N" in releases
  assert "Not Hypermile" in releases
  assert "Follow Distance stalk" in docs
  workflow = (root / ".github/workflows/tests.yaml").read_text()
  assert "selfdrive/controls/tests/test_follow_stalk.py" in workflow

  assert "NAPFollowDistance" in helper or "PARAM_FOLLOW" in helper
  assert "NAPHypermile" not in helper
  assert "PARAM_HYPERMILE" not in helper
  assert "FOLLOW_DISTANCE_DESCRIPTION" in manner
  assert "steps the active 1–7" in content
  assert "No lead:" in content and "MAX" in content
  assert "1 mph" in helper or "tip" in helper
  assert "5 mph" in helper or "hold" in helper
  assert "1 mph" in docs and "5 mph" in docs
  assert "self.refresh()" in manner
  assert "self._follow_distance_city._load_value()" in manner_mici
  assert "self._follow_distance_hwy._load_value()" in manner_mici
  assert "def show_event" in manner_mici
  assert "hypermile" not in manner.lower()
  assert "Hypermile" not in manner_mici
