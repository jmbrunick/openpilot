"""Hypermile phase 1+2: eco snap/restore, speed-split follow, stalk 1–5, safe floor."""
from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.hypermile import (
  ECO_MAP_ACCEL,
  ECO_MAP_LOOKAHEAD_NORMAL,
  ECO_MAP_MODE_CAP,
  ECO_MAP_MODE_FOLLOW,
  ECO_MAP_OFFSET_MPH,
  FOLLOW_LEVEL_DEFAULT,
  LOW_SPEED_STOCK,
  PARAM_FOLLOW_LEVEL,
  PARAM_HYPERMILE,
  PARAM_SAVED,
  SAFE_FLOOR_STOCK,
  SPLIT_MS,
  apply_hypermile_toggle,
  button_event_closer,
  consume_hypermile_stalk,
  detect_hypermile_stalk,
  eco_preset_from,
  effective_nap_follow_dist,
  follow_level_hud_text,
  hypermile_level_to_stock,
  persist_follow_level,
  read_hypermile_params,
  stalk_adjusts_follow,
  step_hypermile_level,
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


def test_default_off_and_level_three():
  on, level = read_hypermile_params(FakeParams())
  assert on is False
  assert level == FOLLOW_LEVEL_DEFAULT


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
    },
  )
  lat_before = params.get_bool("NAPDriverLatHandoff")
  dm_before = params.get_bool("NAPDmSimulateLooking")
  follow_before = params.get("NAPFollowDistance")

  assert apply_hypermile_toggle(params, True) is True
  assert params.get_bool(PARAM_HYPERMILE) is True
  assert params.get_bool("NAPAdaptiveAccel") is True
  assert params.get("NAPMapSpeedMode") == ECO_MAP_MODE_FOLLOW
  assert params.get("NAPMapSpeedOffsetMph") == ECO_MAP_OFFSET_MPH
  assert params.get("NAPMapSpeedLookahead") == ECO_MAP_LOOKAHEAD_NORMAL
  assert params.get("NAPMapSpeedAccel") == ECO_MAP_ACCEL
  assert params.get_bool("NAPDriverLatHandoff") is lat_before
  assert params.get_bool("NAPDmSimulateLooking") is dm_before
  assert params.get("NAPFollowDistance") == follow_before
  assert params.get(PARAM_SAVED)

  # Second On is a no-op (does not re-snapshot the eco values).
  apply_hypermile_toggle(params, True)
  assert params.get("NAPMapSpeedOffsetMph") == ECO_MAP_OFFSET_MPH

  assert apply_hypermile_toggle(params, False) is False
  assert params.get_bool(PARAM_HYPERMILE) is False
  assert params.get_bool("NAPAdaptiveAccel") is False
  assert params.get("NAPMapSpeedMode") == 0
  assert params.get("NAPMapSpeedOffsetMph") == 5
  assert params.get("NAPMapSpeedLookahead") == 0
  assert params.get("NAPMapSpeedAccel") == 8
  assert params.get_bool("NAPDriverLatHandoff") is True
  assert params.get_bool("NAPDmSimulateLooking") is False
  assert params.get("NAPFollowDistance") == 4
  assert not params.get(PARAM_SAVED)


def test_eco_keeps_existing_cap_and_on_lookahead():
  current = {
    "NAPAdaptiveAccel": True,
    "NAPMapSpeedMode": ECO_MAP_MODE_CAP,
    "NAPMapSpeedOffsetMph": 0,
    "NAPMapSpeedLookahead": 3,
    "NAPMapSpeedAccel": 5,
  }
  eco = eco_preset_from(current)
  assert eco["NAPMapSpeedMode"] == ECO_MAP_MODE_CAP
  assert eco["NAPMapSpeedLookahead"] == 3
  assert eco["NAPMapSpeedOffsetMph"] == ECO_MAP_OFFSET_MPH
  assert eco["NAPMapSpeedAccel"] == ECO_MAP_ACCEL


def test_le_50_uses_far_gap_gt_50_uses_stalk_level():
  v_low = 50.0 * CV.MPH_TO_MS
  v_high = 50.0 * CV.MPH_TO_MS + 0.1
  assert abs(v_low - SPLIT_MS) < 1e-9
  for level in range(1, 6):
    assert effective_nap_follow_dist(True, 4, True, level, v_low) == LOW_SPEED_STOCK
    assert effective_nap_follow_dist(True, 4, True, level, 0.0) == LOW_SPEED_STOCK
    assert effective_nap_follow_dist(True, 4, True, level, v_high) == hypermile_level_to_stock(level)
  # Off uses the stock 1–7 slider.
  assert effective_nap_follow_dist(True, 4, False, 1, v_high) == 4
  assert effective_nap_follow_dist(False, 4, True, 1, v_high) is None


def test_stalk_up_down_steps_one_to_five_and_persists():
  params = FakeParams(bools={PARAM_HYPERMILE: True}, ints={PARAM_FOLLOW_LEVEL: 3})
  assert step_hypermile_level(3, closer=True) == 2
  assert step_hypermile_level(1, closer=True) == 1
  assert step_hypermile_level(5, closer=False) == 5
  assert persist_follow_level(params, closer=True) == 2
  assert params.get(PARAM_FOLLOW_LEVEL) == 2
  persist_follow_level(params, closer=True)
  persist_follow_level(params, closer=True)
  persist_follow_level(params, closer=True)
  assert params.get(PARAM_FOLLOW_LEVEL) == 1
  persist_follow_level(params, closer=False)
  assert params.get(PARAM_FOLLOW_LEVEL) == 2

  level, undo = consume_hypermile_stalk(
    params, has_lead=True, button_closer=False, raw_kph=100.0, prev_raw_kph=105.0,
  )
  assert level == 3
  assert undo == 105.0
  assert params.get(PARAM_FOLLOW_LEVEL) == 3

  # A Follow/posted MAX jump is not a 1/5 mph stalk step.
  jump_level, jump_undo = consume_hypermile_stalk(
    params, has_lead=True, button_closer=None, raw_kph=120.0, prev_raw_kph=100.0,
  )
  assert jump_level is None and jump_undo is None

  # No lead: leave stalk as MAX (no write).
  before = params.get(PARAM_FOLLOW_LEVEL)
  none_level, none_undo = consume_hypermile_stalk(
    params, has_lead=False, button_closer=True, raw_kph=110.0, prev_raw_kph=105.0,
  )
  assert none_level is None and none_undo is None
  assert params.get(PARAM_FOLLOW_LEVEL) == before
  assert stalk_adjusts_follow(hypermile_on=True, has_lead=False) is False


def test_safe_floor_never_uses_stock_one():
  assert SAFE_FLOOR_STOCK == 2
  assert hypermile_level_to_stock(1) >= SAFE_FLOOR_STOCK
  for level in range(1, 6):
    stock = hypermile_level_to_stock(level)
    assert stock >= SAFE_FLOOR_STOCK
    assert stock <= 6
    t_follow = NAP_T_FOLLOW[stock - 1]
    assert t_follow >= NAP_T_FOLLOW[SAFE_FLOOR_STOCK - 1]
  far = NAP_T_FOLLOW[LOW_SPEED_STOCK - 1]
  draft = NAP_T_FOLLOW[hypermile_level_to_stock(1) - 1]
  assert far > draft
  # High-speed band is tighter than the low-speed far gap.
  v = 60.0 * CV.MPH_TO_MS
  assert effective_nap_follow_dist(True, 4, True, 5, v) < LOW_SPEED_STOCK


def test_button_events_and_hud_text():
  up = SimpleNamespace(type=SimpleNamespace(name="accelCruise"), pressed=True)
  down = SimpleNamespace(type=SimpleNamespace(name="decelCruise"), pressed=True)
  release = SimpleNamespace(type=SimpleNamespace(name="accelCruise"), pressed=False)
  assert button_event_closer([up]) is True
  assert button_event_closer([down]) is False
  assert button_event_closer([release]) is None
  assert follow_level_hud_text(3) == "Hypermile: Follow 3"
  is_stalk, closer, undo = detect_hypermile_stalk(
    FakeParams(bools={PARAM_HYPERMILE: True}, ints={PARAM_FOLLOW_LEVEL: 3}),
    has_lead=True, button_closer=True, raw_kph=None, prev_raw_kph=None,
  )
  assert is_stalk is True and closer is True and undo is None


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
  docs = (root / "docs-nap/hypermile.md").read_text()
  readme = (root / "docs-nap/README.md").read_text()
  releases = (root / "RELEASES.md").read_text()

  for src in (tici, mici):
    assert "Hypermile" in src or "hypermile" in src
    assert "NAP_HYPERMILE" in src
    assert "apply_hypermile_toggle" in src
  assert "Hypermile" in nap
  assert "put_bool(NAP_HYPERMILE, False)" in nap
  assert "NAPHypermile" in keys
  assert "NAPHypermileFollowLevel" in keys
  assert 'BOOL, "0"' in next(ln for ln in keys.splitlines() if '"NAPHypermile"' in ln)
  assert 'INT, "3"' in next(ln for ln in keys.splitlines() if '"NAPHypermileFollowLevel"' in ln)
  assert "effective_nap_follow_dist" in planner
  assert "detect_hypermile_stalk" in card
  assert "radarState" in card
  assert "hypermileFollowChanged" in events
  assert "follow_level_hud_text" in events
  assert "NAPHypermile" in content
  assert "hypermile.md" in readme
  assert "Hypermile" in docs
  assert "Hypermile" in releases.split("\n\n", 1)[0]
  assert "nap-release" not in docs.lower() or "not a nap-release" in docs.lower()
