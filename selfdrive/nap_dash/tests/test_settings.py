"""Web writes follow the discovered device controls. No engage, no cereal."""
from __future__ import annotations

import pytest

from openpilot.selfdrive.nap_dash.ui_api import SettingError, read_settings, write_setting


class FakeParams:
  def __init__(self, values=None, bools=None):
    self.values = dict(values or {})
    self.bools = dict(bools or {})
    self.writes: list[tuple[str, object]] = []

  def get(self, key, return_default=False):
    if key in self.values:
      return self.values[key]
    return None

  def get_bool(self, key):
    return bool(self.bools.get(key, False))

  def put(self, key, value, block=False):
    self.writes.append((key, value))
    self.values[key] = value

  def put_bool(self, key, value, block=False):
    self.writes.append((key, bool(value)))
    self.bools[key] = bool(value)

  def remove(self, key):
    self.values.pop(key, None)
    self.bools.pop(key, None)


def test_read_settings_does_not_write():
  params = FakeParams(bools={"IsMetric": True})
  snap = read_settings(params)
  assert snap["values"]["IsMetric"] is True
  assert params.writes == []
  assert snap["onroad"] is False
  assert snap["engaged"] is False


def test_metric_round_trip_and_unknown_name_rejected():
  params = FakeParams()
  snap = write_setting(params, "IsMetric", True)
  assert snap["values"]["IsMetric"] is True
  assert params.bools["IsMetric"] is True
  for name in ("speed_trim", "city_turns", "tap_lc", "corner_assist", "lane_centering", "engage"):
    with pytest.raises(SettingError):
      write_setting(params, name, 1)


def test_experimental_requires_confirmation_then_sets_confirmed():
  params = FakeParams()
  with pytest.raises(SettingError, match="confirmation"):
    write_setting(params, "ExperimentalMode", True)
  assert "ExperimentalMode" not in params.bools
  snap = write_setting(params, "ExperimentalMode", True, confirm=True)
  assert snap["values"]["ExperimentalMode"] is True
  assert params.bools["ExperimentalModeConfirmed"] is True
  write_setting(params, "ExperimentalMode", False)
  assert params.bools["ExperimentalMode"] is False


def test_onroad_and_engaged_locks():
  params = FakeParams(bools={"IsOnroad": True})
  with pytest.raises(SettingError, match="car is on"):
    write_setting(params, "AdbEnabled", True)
  assert "AdbEnabled" not in params.bools

  parked = FakeParams(bools={"IsEngaged": True})
  with pytest.raises(SettingError, match="engaged"):
    write_setting(parked, "OpenpilotEnabledToggle", False)
  write_setting(parked, "IsMetric", True)
  assert parked.bools["IsMetric"] is True


def test_simulate_look_and_false_alert_are_exclusive():
  params = FakeParams(bools={"NAPDmSimulateLooking": True, "NAPDmFalseAlertIgnore": False})
  snap = write_setting(params, "NAPDmFalseAlertIgnore", True)
  assert snap["values"]["NAPDmFalseAlertIgnore"] is True
  assert snap["values"]["NAPDmSimulateLooking"] is False
  snap = write_setting(params, "NAPDmSimulateLooking", True)
  assert snap["values"]["NAPDmSimulateLooking"] is True
  assert snap["values"]["NAPDmFalseAlertIgnore"] is False


def test_map_speed_choice_rejects_unknown_value():
  params = FakeParams()
  snap = write_setting(params, "NAPMapSpeedMode", 3)
  assert snap["values"]["NAPMapSpeedMode"] == 3
  with pytest.raises(SettingError):
    write_setting(params, "NAPMapSpeedMode", 9)
  snap = write_setting(params, "NAPMapSpeedOffsetMph", -5)
  assert snap["values"]["NAPMapSpeedOffsetMph"] == -5


def test_speed_sign_write_is_plain_param_put():
  """Discovered from the device UI. The write is put_bool, not a new code path."""
  params = FakeParams()
  write_setting(params, "NAPSpeedSignLog", True)
  assert params.bools["NAPSpeedSignLog"] is True
  assert [item for item in params.writes if item[0] == "NAPSpeedSignLog"] == [("NAPSpeedSignLog", True)]
  assert "OnroadCycleRequested" not in params.bools


def test_hypermile_uses_device_helper():
  params = FakeParams(bools={"NAPHypermile": False, "NAPAdaptiveAccel": True})
  snap = write_setting(params, "NAPHypermile", True)
  assert snap["values"]["NAPHypermile"] is True
  assert params.bools["NAPHypermile"] is True


def test_reboot_requires_confirmation_and_blocks_while_engaged():
  params = FakeParams()
  with pytest.raises(SettingError, match="confirmation"):
    write_setting(params, "DoReboot", True)
  assert "DoReboot" not in params.bools
  write_setting(params, "DoReboot", True, confirm=True)
  assert params.bools["DoReboot"] is True

  engaged = FakeParams(bools={"IsEngaged": True})
  with pytest.raises(SettingError, match="engaged"):
    write_setting(engaged, "DoShutdown", True, confirm=True)


def test_disabled_device_control_is_not_writable():
  params = FakeParams()
  with pytest.raises(SettingError):
    write_setting(params, "NAPiBoosterEnabled", True)
