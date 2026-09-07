"""mici NAP submenu: Map Speed Limit controls."""
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.big_multi_value_param import BigMultiValueParamToggle
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.mici.layouts.settings.nap_script import launch_script
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  DOWNLOAD_US_MAPS_INSTRUCTIONS,
  MAP_SPEED_ACCEL, MAP_SPEED_ACCEL_DEFAULT, MAP_SPEED_ACCEL_LABELS,
  MAP_SPEED_LOOKAHEAD, MAP_SPEED_LOOKAHEAD_LABELS,
  MAP_SPEED_MODES, MAP_SPEED_MODE_LABELS, MAP_SPEED_OFFSETS_MPH,
)
from openpilot.selfdrive.mapd.fetch_maps import installed_db_summary
from openpilot.selfdrive.ui.ui_state import ui_state


class MapSpeedLimitLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    map_mode = BigMultiValueParamToggle(
      "map speed (max)",
      "NAPMapSpeedMode",
      values=MAP_SPEED_MODES,
      labels=[s.lower() for s in MAP_SPEED_MODE_LABELS],
      default_value=0,
    )
    map_offset = BigMultiValueParamToggle(
      "map speed offset",
      "NAPMapSpeedOffsetMph",
      values=list(MAP_SPEED_OFFSETS_MPH),
      labels=["-5 mph", "0", "+5 mph"],
      default_value=0,
    )
    map_lookahead = BigMultiValueParamToggle(
      "lookahead",
      "NAPMapSpeedLookahead",
      values=list(MAP_SPEED_LOOKAHEAD),
      labels=[s.lower() for s in MAP_SPEED_LOOKAHEAD_LABELS],
      default_value=2,
    )
    map_accel = BigMultiValueParamToggle(
      "acceleration",
      "NAPMapSpeedAccel",
      values=list(MAP_SPEED_ACCEL),
      labels=MAP_SPEED_ACCEL_LABELS,
      default_value=MAP_SPEED_ACCEL_DEFAULT,
    )
    map_db_status = BigButton("osm map data", installed_db_summary())
    download_maps_btn = BigButton("download us maps", "start")
    download_maps_btn.set_click_callback(
      lambda: launch_script("Download US Maps", DOWNLOAD_US_MAPS_INSTRUCTIONS,
                            "scripts.nap.fetch_osm_maps",
                            ))
    download_maps_btn.set_enabled(ui_state.is_offroad)
    self.add_widgets([
      map_mode,
      map_offset,
      map_lookahead,
      map_accel,
      map_db_status,
      download_maps_btn,
    ])


def open_map_speed_limit_menu(page: MapSpeedLimitLayoutMici | None = None) -> MapSpeedLimitLayoutMici:
  panel = page or MapSpeedLimitLayoutMici()
  gui_app.push_widget(panel)
  return panel
