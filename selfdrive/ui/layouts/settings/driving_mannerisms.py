"""TICI NAP submenu: Driving Mannerisms controls."""
from openpilot.common.params import Params
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import toggle_item, multiple_button_item, button_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  DRIVER_LAT_HANDOFF_DESCRIPTION,
  DM_HANDS_ON_RESET_DESCRIPTION,
  NAP_DM_HANDS_ON_RESET,
  NAP_DRIVER_LAT_HANDOFF,
)
from opendbc.car.tesla.preap.nap_params import NAPParamKeys


class DrivingMannerismsLayout(Widget):
  """Nested NAP page for accel feel, follow distance, soft-lat, and DM reset."""

  def __init__(self, on_back):
    super().__init__()
    self._params = Params()
    self._on_back = on_back
    self._build_items()
    self._scroller = Scroller(self._all_items, line_separator=True, spacing=0)

  def _build_items(self):
    self._all_items = []
    self._all_items.append(button_item(
      "Back To",
      "NAP",
      description="Return to NAP settings.",
      callback=self._on_back,
    ))

    self._adaptive_accel = toggle_item(
      "Adaptive Accel Limits",
      description="Reduces acceleration authority when close to a lead car to prevent overshoot. Full accel on open road or when closing a large gap.",
      initial_state=self._params.get_bool(NAPParamKeys.ADAPTIVE_ACCEL),
      callback=self._on_adaptive_accel,
    )
    self._all_items.append(self._adaptive_accel)

    follow_dist = self._params.get(NAPParamKeys.FOLLOW_DISTANCE, return_default=True)
    self._follow_buttons = multiple_button_item(
      "Follow Distance",
      "Follow distance (1=closest, 7=farthest). A slower car ahead starts a " +
      "gradual ease-off farther back (more distance, not a harder brake). " +
      "Overridden by cruise stalk if present.",
      buttons=["1", "2", "3", "4", "5", "6", "7"],
      button_width=80,
      selected_index=max(0, min(6, follow_dist - 1)),
      callback=self._on_follow_distance,
    )
    self._all_items.append(self._follow_buttons)

    self._lat_handoff = toggle_item(
      "Soft Lateral Handoff",
      description=DRIVER_LAT_HANDOFF_DESCRIPTION,
      initial_state=self._params.get_bool(NAP_DRIVER_LAT_HANDOFF),
      callback=self._on_lat_handoff,
    )
    self._all_items.append(self._lat_handoff)

    self._dm_hands_on = toggle_item(
      "Hands-On Look-at-Road Reset",
      description=DM_HANDS_ON_RESET_DESCRIPTION,
      initial_state=self._params.get_bool(NAP_DM_HANDS_ON_RESET),
      callback=self._on_dm_hands_on,
    )
    self._all_items.append(self._dm_hands_on)

  def _on_adaptive_accel(self, state):
    self._params.put_bool(NAPParamKeys.ADAPTIVE_ACCEL, state)

  def _on_follow_distance(self, index: int):
    self._params.put(NAPParamKeys.FOLLOW_DISTANCE, index + 1)

  def _on_lat_handoff(self, state):
    self._params.put_bool(NAP_DRIVER_LAT_HANDOFF, state)

  def _on_dm_hands_on(self, state):
    self._params.put_bool(NAP_DM_HANDS_ON_RESET, state)

  def refresh(self):
    self._adaptive_accel.action_item.set_state(self._params.get_bool(NAPParamKeys.ADAPTIVE_ACCEL))
    follow_dist = self._params.get(NAPParamKeys.FOLLOW_DISTANCE, return_default=True)
    self._follow_buttons.action_item.set_selected_button(max(0, min(6, follow_dist - 1)))
    self._lat_handoff.action_item.set_state(self._params.get_bool(NAP_DRIVER_LAT_HANDOFF))
    self._dm_hands_on.action_item.set_state(self._params.get_bool(NAP_DM_HANDS_ON_RESET))

  def show_event(self):
    self._scroller.show_event()
    self.refresh()

  def _render(self, rect):
    self._scroller.render(rect)
