"""Stock Follow Distance stalk remap: lead → 1–7, no lead → MAX.

Not Hypermile. No 1–5 band, no ≤50 forced far gap, no eco / Step Down.
"""
from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.follow_stalk import (
  FOLLOW_DEFAULT,
  FOLLOW_MAX,
  FOLLOW_MIN,
  PARAM_FOLLOW,
  STALK_COOLDOWN_S,
  button_event_closer,
  clamp_follow_distance,
  consume_follow_stalk,
  detect_follow_stalk,
  follow_distance_hud_text,
  persist_follow_distance,
  read_follow_distance,
  stalk_adjusts_follow,
  step_follow_distance,
)


class FakeParams:
  def __init__(self, follow=FOLLOW_DEFAULT):
    self._ints = {PARAM_FOLLOW: int(follow)}

  def get(self, key, return_default=False):
    if key in self._ints:
      return self._ints[key]
    return FOLLOW_DEFAULT if return_default else None

  def put(self, key, value):
    self._ints[key] = int(value)


def test_lead_present_stalk_writes_nap_follow_distance_and_undoes_max():
  params = FakeParams(follow=4)
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=True, raw_kph=105.0, prev_raw_kph=100.0,
  )
  assert level == 3
  assert params.get(PARAM_FOLLOW) == 3
  assert undo == 100.0

  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=False, raw_kph=100.0, prev_raw_kph=105.0,
  )
  assert level == 4
  assert params.get(PARAM_FOLLOW) == 4
  assert undo == 105.0


def test_no_lead_leaves_max_step_untouched():
  params = FakeParams(follow=4)
  before = params.get(PARAM_FOLLOW)
  level, undo = consume_follow_stalk(
    params, has_lead=False, button_closer=True, raw_kph=105.0, prev_raw_kph=100.0,
  )
  assert level is None and undo is None
  assert params.get(PARAM_FOLLOW) == before
  assert stalk_adjusts_follow(has_lead=False) is False
  assert stalk_adjusts_follow(has_lead=True) is True


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
  assert "hypermile" not in src.lower()
  assert "SAFE_FLOOR" not in src
  assert "SPLIT" not in src
  for v_ego_mph in (30.0, 50.0, 65.0):
    _ = v_ego_mph * CV.MPH_TO_MS
    params = FakeParams(follow=1)
    level, undo = consume_follow_stalk(
      params, has_lead=True, button_closer=False, raw_kph=100.0, prev_raw_kph=105.0,
    )
    assert level == 2
    assert undo == 105.0
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
  assert follow_distance_hud_text(3) == "Follow Distance: 3"
  assert follow_distance_hud_text(7) == "Follow Distance: 7"
  is_stalk, closer, undo = detect_follow_stalk(
    FakeParams(follow=4),
    has_lead=True, button_closer=True, raw_kph=None, prev_raw_kph=None,
  )
  assert is_stalk is True and closer is True and undo is None


def test_cooldown_detects_without_writing():
  params = FakeParams(follow=4)
  level, undo = consume_follow_stalk(
    params, has_lead=True, button_closer=True, raw_kph=105.0, prev_raw_kph=100.0,
    apply=False,
  )
  assert level == 4
  assert undo == 100.0
  assert params.get(PARAM_FOLLOW) == 4
  assert STALK_COOLDOWN_S == 0.25


def test_card_and_hud_wire_stock_follow_only():
  from pathlib import Path

  root = Path(__file__).resolve().parents[3]
  card = (root / "selfdrive/car/card.py").read_text()
  events = (root / "selfdrive/selfdrived/events.py").read_text()
  selfdrived = (root / "selfdrive/selfdrived/selfdrived.py").read_text()
  cereal = (root / "cereal/log.capnp").read_text()
  manner = (root / "selfdrive/ui/layouts/settings/driving_mannerisms.py").read_text()
  manner_mici = (root / "selfdrive/ui/mici/layouts/settings/driving_mannerisms.py").read_text()
  helper = (root / "selfdrive/controls/lib/follow_stalk.py").read_text()

  assert "from openpilot.selfdrive.controls.lib.follow_stalk import" in card
  assert "_maybe_follow_stalk" in card
  assert "radarState" in card
  assert "stalk_pressed = (not follow_stalk) and self._preap_stalk_set_pressed(CS)" in card
  assert "STALK_COOLDOWN_S" in card
  assert "persist_follow_distance" in card
  assert "_write_preap_pedal_speed(CS, undo)" in card
  assert "hypermile" not in card.lower()
  assert "NAPHypermile" not in card

  assert "followDistanceChanged" in cereal
  assert "follow_distance_changed_alert" in events
  assert "follow_distance_hud_text" in events
  assert "followDistanceChanged" in selfdrived
  assert "NAPFollowDistance" in selfdrived
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
  assert "stalk up/down steps this 1–7" in manner
  assert "No lead: stalk still adjusts MAX" in manner
  assert "self.refresh()" in manner
  assert "self._follow_distance._load_value()" in manner_mici
  assert "def show_event" in manner_mici
  assert "hypermile" not in manner.lower()
  assert "Hypermile" not in manner_mici
