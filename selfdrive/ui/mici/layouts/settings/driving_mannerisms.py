"""mici NAP submenu: Driving Mannerisms controls."""
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.big_multi_value_param import BigMultiValueParamToggle
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  FOLLOW_DISTANCE_DEFAULT,
  FOLLOW_DISTANCE_LABELS,
  FOLLOW_DISTANCE_VALUES,
  NAP_DM_HANDS_ON_RESET,
  NAP_DRIVER_LAT_HANDOFF,
)
from opendbc.car.tesla.preap.nap_params import NAPParamKeys


class DrivingMannerismsLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    adaptive_accel = BigParamControl("adaptive accel limits", NAPParamKeys.ADAPTIVE_ACCEL)

    follow_distance = BigMultiValueParamToggle(
      "follow distance",
      NAPParamKeys.FOLLOW_DISTANCE,
      values=FOLLOW_DISTANCE_VALUES,
      labels=FOLLOW_DISTANCE_LABELS,
      default_value=FOLLOW_DISTANCE_DEFAULT,
    )

    lat_handoff = BigParamControl("soft lateral handoff", NAP_DRIVER_LAT_HANDOFF)
    lat_handoff.set_value("On — free-wheel yield; Off if false-yield")

    dm_hands_on = BigParamControl("hands-on look-at-road reset", NAP_DM_HANDS_ON_RESET)
    dm_hands_on.set_value("On — rim contact; first prompt 2.0–4.5 s random")

    self._scroller.add_widgets([
      adaptive_accel,
      follow_distance,
      lat_handoff,
      dm_hands_on,
    ])


def open_driving_mannerisms_menu(page: DrivingMannerismsLayoutMici | None = None) -> DrivingMannerismsLayoutMici:
  panel = page or DrivingMannerismsLayoutMici()
  gui_app.push_widget(panel)
  return panel
