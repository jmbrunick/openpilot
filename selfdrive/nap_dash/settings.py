"""Settings API for the comma web UI.

Controls are discovered from the on-device settings sources. This module
does not keep a second list of toggles.
"""
from openpilot.selfdrive.nap_dash.ui_api import (
  SettingError,
  read_settings,
  setting_catalog,
  write_setting,
)

__all__ = ["SettingError", "read_settings", "setting_catalog", "write_setting"]
