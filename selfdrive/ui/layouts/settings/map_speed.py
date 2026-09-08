"""TICI NAP submenu: Map Speed Limit controls."""
from openpilot.common.params import Params
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import multiple_button_item, button_item, text_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  DOWNLOAD_US_MAPS_INSTRUCTIONS,
  MAP_SPEED_ACCEL, MAP_SPEED_ACCEL_DEFAULT, MAP_SPEED_ACCEL_LABELS,
  MAP_SPEED_LOOKAHEAD, MAP_SPEED_LOOKAHEAD_LABELS,
  MAP_SPEED_MODES, MAP_SPEED_MODE_LABELS, MAP_SPEED_OFFSETS_MPH,
  REFRESH_MAPS_INSTRUCTIONS,
)
from openpilot.selfdrive.mapd.fetch_maps import installed_db_summary, installed_revision_summary
from openpilot.selfdrive.ui.ui_state import ui_state


class MapSpeedLimitLayout(Widget):
  """Nested NAP page for OSM map-speed mode, offset, lookahead, and accel."""

  def __init__(self, on_back, on_download, on_refresh):
    super().__init__()
    self._params = Params()
    self._on_back = on_back
    self._on_download = on_download
    self._on_refresh = on_refresh
    self._build_items()
    self._scroller = Scroller(self._all_items, line_separator=True, spacing=0)

  def _build_items(self):
    self._all_items = []
    self._all_items.append(button_item(
      "← Back",
      "NAP",
      description="Return to NAP settings.",
      callback=self._on_back,
    ))

    map_mode = int(self._params.get("NAPMapSpeedMode", return_default=True) or 0)
    self._mode_buttons = multiple_button_item(
      "Map Speed (MAX)",
      "OpenStreetMap posted limit for HUD MAX. " +
      "Off: no change. Display: LIMIT sign only. Cap: never exceed the limit. " +
      "Follow (preferred): track the limit. Stalk below holds that set until you stalk again or the sign changes. " +
      "Stalk above pauses Follow for 10s. Cap/Follow need the pedal interceptor; stock CC stays display-only.",
      buttons=MAP_SPEED_MODE_LABELS,
      button_width=150,
      selected_index=max(0, min(3, map_mode)),
      callback=self._on_mode,
    )
    self._all_items.append(self._mode_buttons)

    offset_mph = int(self._params.get("NAPMapSpeedOffsetMph", return_default=True) or 0)
    self._offset_buttons = multiple_button_item(
      "Map Speed Offset",
      "Added to the OSM limit for Cap/Follow (mph). Below-limit sets stay put; above-limit Follow override lasts 10 seconds.",
      buttons=["-5 mph", "0", "+5 mph"],
      button_width=150,
      selected_index=self._offset_index(offset_mph),
      callback=self._on_offset,
    )
    self._all_items.append(self._offset_buttons)

    lookahead = int(self._params.get("NAPMapSpeedLookahead", return_default=True) or 2)
    self._lookahead_buttons = multiple_button_item(
      "Lookahead",
      "Cap/Follow: ease MAX down for a lower limit ahead. " +
      "Off: wait until GPS is on that way. Late / Normal / Early: farther preview. " +
      "A higher limit ahead never raises MAX early. Radar lead still outranks map.",
      buttons=MAP_SPEED_LOOKAHEAD_LABELS,
      button_width=150,
      selected_index=self._lookahead_index(lookahead),
      callback=self._on_lookahead,
    )
    self._all_items.append(self._lookahead_buttons)

    accel = int(self._params.get("NAPMapSpeedAccel", return_default=True) or MAP_SPEED_ACCEL_DEFAULT)
    self._accel_buttons = multiple_button_item(
      "Acceleration",
      "Follow only: how quickly the car climbs when MAX rises. " +
      "1=gentlest (0.36 m/s² at Normal), 5=0.80, 10=quickest (1.60, clamped). " +
      "Brake to a lower MAX is locked at Accel 5 (0.80 m/s² at Normal). " +
      "A slower lead can still brake harder.",
      buttons=MAP_SPEED_ACCEL_LABELS,
      button_width=72,
      selected_index=self._accel_index(accel),
      callback=self._on_accel,
    )
    self._all_items.append(self._accel_buttons)

    self._db_status = text_item(
      "OSM Map Data",
      installed_db_summary,
      description="US OpenStreetMap maxspeed ways (ODbL, © OpenStreetMap contributors). " +
      "Not in git — tap Download US Maps over Wi-Fi after flash.",
    )
    self._all_items.append(self._db_status)

    self._revision_status = text_item(
      "Map revision",
      installed_revision_summary,
      description="Published US pack revision recorded after a successful download or refresh.",
    )
    self._all_items.append(self._revision_status)

    self._download_btn = button_item(
      "Download US Maps",
      "Start",
      description=DOWNLOAD_US_MAPS_INSTRUCTIONS.split("\n", 1)[0],
      callback=self._on_download,
    )
    self._download_btn.action_item.set_enabled(ui_state.is_offroad)
    self._all_items.append(self._download_btn)

    self._refresh_btn = button_item(
      "Check for map updates",
      "Refresh maps",
      description=REFRESH_MAPS_INSTRUCTIONS.split("\n", 1)[0],
      callback=self._on_refresh,
    )
    self._refresh_btn.action_item.set_enabled(ui_state.is_offroad)
    self._all_items.append(self._refresh_btn)

  def _offset_index(self, offset_mph: int) -> int:
    if offset_mph in MAP_SPEED_OFFSETS_MPH:
      return MAP_SPEED_OFFSETS_MPH.index(offset_mph)
    return 1

  def _lookahead_index(self, value: int) -> int:
    if value in MAP_SPEED_LOOKAHEAD:
      return MAP_SPEED_LOOKAHEAD.index(value)
    return 2

  def _accel_index(self, value: int) -> int:
    if value in MAP_SPEED_ACCEL:
      return MAP_SPEED_ACCEL.index(value)
    return MAP_SPEED_ACCEL.index(MAP_SPEED_ACCEL_DEFAULT)

  def _on_mode(self, index: int):
    self._params.put("NAPMapSpeedMode", MAP_SPEED_MODES[index])

  def _on_offset(self, index: int):
    self._params.put("NAPMapSpeedOffsetMph", int(MAP_SPEED_OFFSETS_MPH[index]))

  def _on_lookahead(self, index: int):
    self._params.put("NAPMapSpeedLookahead", MAP_SPEED_LOOKAHEAD[index])

  def _on_accel(self, index: int):
    self._params.put("NAPMapSpeedAccel", MAP_SPEED_ACCEL[index])

  def refresh(self):
    map_mode = int(self._params.get("NAPMapSpeedMode", return_default=True) or 0)
    self._mode_buttons.action_item.set_selected_button(max(0, min(3, map_mode)))
    offset_mph = int(self._params.get("NAPMapSpeedOffsetMph", return_default=True) or 0)
    self._offset_buttons.action_item.set_selected_button(self._offset_index(offset_mph))
    lookahead = int(self._params.get("NAPMapSpeedLookahead", return_default=True) or 2)
    self._lookahead_buttons.action_item.set_selected_button(self._lookahead_index(lookahead))
    accel = int(self._params.get("NAPMapSpeedAccel", return_default=True) or MAP_SPEED_ACCEL_DEFAULT)
    self._accel_buttons.action_item.set_selected_button(self._accel_index(accel))
    self._download_btn.action_item.set_enabled(ui_state.is_offroad)
    self._refresh_btn.action_item.set_enabled(ui_state.is_offroad)

  def show_event(self):
    self._scroller.show_event()
    self.refresh()

  def _render(self, rect):
    self._scroller.render(rect)
