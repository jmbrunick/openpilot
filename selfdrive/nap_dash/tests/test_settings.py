"""Dash settings API writes Justin's Mannerisms Params only."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from openpilot.selfdrive.controls.lib.hypermile import (
  ECO_MAP_ACCEL,
  ECO_MAP_LOOKAHEAD_EARLY,
  PARAM_HYPERMILE,
  PARAM_SAVED,
)
from openpilot.selfdrive.monitoring.dm_toggles import (
  PARAM_DM_FALSE_ALERT_IGNORE,
  PARAM_DM_SIMULATE_LOOKING,
)
from openpilot.selfdrive.nap_dash.settings import (
  PARAM_ACCEL,
  PARAM_ADAPTIVE_ACCEL,
  PARAM_EXPERIMENTAL,
  PARAM_EXPERIMENTAL_CONFIRMED,
  PARAM_FAI,
  PARAM_FOLLOW_DISTANCE,
  PARAM_MAP_LOOKAHEAD,
  PARAM_MAP_MODE,
  PARAM_MAP_OFFSET,
  PARAM_PERSONALITY,
  PARAM_SL,
  REJECTED_SETTING_NAMES,
  SettingError,
  read_settings,
  setting_catalog,
  write_setting,
)
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  NAP_DRIVER_LAT_HANDOFF,
  NAP_HYPERMILE_HILL_CLIMB,
  NAP_HYPERMILE_STEP_DOWN,
  NAP_ONE_PEDAL_LONG,
)

ROOT = Path(__file__).resolve().parents[3]


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


def test_follow_distance_round_trip_same_param_as_mannerisms():
  params = FakeParams(ints={PARAM_FOLLOW_DISTANCE: 4})
  snap = write_setting(params, "follow_distance", 2)
  assert snap["follow_distance"] == 2
  assert params.ints[PARAM_FOLLOW_DISTANCE] == 2
  snap = write_setting(params, PARAM_FOLLOW_DISTANCE, 7)
  assert snap["follow_distance"] == 7


def test_accel_and_adaptive_round_trip():
  params = FakeParams(ints={PARAM_ACCEL: 5}, bools={PARAM_ADAPTIVE_ACCEL: True})
  write_setting(params, "accel", 8)
  write_setting(params, "adaptive_accel", False)
  assert params.ints[PARAM_ACCEL] == 8
  assert params.bools[PARAM_ADAPTIVE_ACCEL] is False
  with pytest.raises(SettingError):
    write_setting(params, "accel", 11)


def test_hypermile_uses_apply_toggle_snap_and_restore():
  params = FakeParams(
    ints={
      PARAM_MAP_MODE: 0,
      PARAM_MAP_OFFSET: 0,
      PARAM_MAP_LOOKAHEAD: 2,
      PARAM_ACCEL: 7,
    },
    bools={PARAM_ADAPTIVE_ACCEL: False, PARAM_HYPERMILE: False},
  )
  snap = write_setting(params, "hypermile", True)
  assert snap["hypermile"] is True
  assert params.bools[PARAM_HYPERMILE] is True
  assert params.ints[PARAM_ACCEL] == ECO_MAP_ACCEL
  assert params.ints[PARAM_MAP_LOOKAHEAD] == ECO_MAP_LOOKAHEAD_EARLY
  assert params.bools[PARAM_ADAPTIVE_ACCEL] is True
  saved = json.loads(params.ints[PARAM_SAVED])
  assert saved[PARAM_ACCEL] == 7
  snap = write_setting(params, "hypermile", False)
  assert snap["hypermile"] is False
  assert params.ints[PARAM_ACCEL] == 7
  assert params.bools[PARAM_ADAPTIVE_ACCEL] is False


def test_sl_fai_are_mutually_exclusive():
  params = FakeParams(bools={PARAM_SL: True, PARAM_FAI: False})
  snap = write_setting(params, "fai", True)
  assert snap["fai"] is True
  assert snap["sl"] is False
  assert params.bools[PARAM_DM_FALSE_ALERT_IGNORE] is True
  assert params.bools[PARAM_DM_SIMULATE_LOOKING] is False
  snap = write_setting(params, "sl", True)
  assert snap["sl"] is True
  assert snap["fai"] is False


def test_rejects_philip_only_names():
  params = FakeParams()
  for name in ("speed_trim", "speed_offset", "city_turns", "tap_lc", "corner_assist", "lane_centering"):
    with pytest.raises(SettingError, match="rejected|unknown"):
      write_setting(params, name, 1)
  assert REJECTED_SETTING_NAMES


def test_map_speed_and_stock_params():
  params = FakeParams()
  write_setting(params, "map_speed_mode", 3)
  write_setting(params, "map_speed_offset_mph", -5)
  write_setting(params, "map_speed_lookahead", 3)
  write_setting(params, "personality", 2)
  write_setting(params, "experimental", True)
  write_setting(params, "driver_lat_handoff", False)
  write_setting(params, "one_pedal_long", True)
  write_setting(params, "hypermile_step_down", True)
  write_setting(params, "hypermile_hill_climb", False)
  snap = read_settings(params)
  assert snap["map_speed_mode"] == 3
  assert snap["map_speed_offset_mph"] == -5
  assert snap["map_speed_lookahead"] == 3
  assert snap["personality"] == 2
  assert snap["experimental"] is True
  assert params.bools[PARAM_EXPERIMENTAL] is True
  assert params.bools[PARAM_EXPERIMENTAL_CONFIRMED] is True
  assert snap["driver_lat_handoff"] is False
  assert snap["one_pedal_long"] is True
  assert snap["hypermile_step_down"] is True
  assert snap["hypermile_hill_climb"] is False
  with pytest.raises(SettingError):
    write_setting(params, "map_speed_offset_mph", 3)


def test_catalog_matches_mannerisms_order():
  names = [item["name"] for item in setting_catalog()]
  assert names[:8] == [
    "accel", "adaptive_accel", "follow_distance", "driver_lat_handoff",
    "one_pedal_long", "hypermile", "hypermile_step_down", "hypermile_hill_climb",
  ]
  params = {item["param"] for item in setting_catalog()}
  for key in (
    PARAM_ACCEL, PARAM_ADAPTIVE_ACCEL, PARAM_FOLLOW_DISTANCE, NAP_DRIVER_LAT_HANDOFF,
    NAP_ONE_PEDAL_LONG, PARAM_HYPERMILE, NAP_HYPERMILE_STEP_DOWN, NAP_HYPERMILE_HILL_CLIMB,
    PARAM_MAP_MODE, PARAM_MAP_OFFSET, PARAM_MAP_LOOKAHEAD, PARAM_PERSONALITY, PARAM_EXPERIMENTAL,
    PARAM_SL, PARAM_FAI,
  ):
    assert key in params


def test_package_has_no_philip_settings_file_or_funnel():
  pkg = ROOT / "selfdrive" / "nap_dash"
  html = (pkg / "dashboard.html").read_text(encoding="utf-8")
  server = (pkg / "server.py").read_text(encoding="utf-8")
  assert "NAP_SETTINGS_FILE" not in server
  assert "update_nap_settings_file" not in server
  assert "/api/nav" not in server
  assert "setv('speed_trim'" not in html
  assert "toggle('speed_offset')" not in html
  assert "Driving Mannerisms" in html
  assert "Follow Distance" in html
  assert "Hypermile" in html
  assert ">SL<" in html and ">FAI<" in html
  assert "Cruise speed trim" not in html
  assert "Phone guidance" not in html
