"""mici NAP submenu: Driving Mannerisms controls."""
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.big_multi_value_param import BigMultiValueParamToggle
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  FOLLOW_DISTANCE_DEFAULT,
  FOLLOW_DISTANCE_LABELS,
  FOLLOW_DISTANCE_VALUES,
  MAP_SPEED_ACCEL, MAP_SPEED_ACCEL_DEFAULT, MAP_SPEED_ACCEL_LABELS,
  NAP_DRIVER_LAT_HANDOFF,
  NAP_FOLLOW_DISTANCE_CITY,
  NAP_FOLLOW_DISTANCE_HWY,
  NAP_ONE_PEDAL_LONG,
)
from opendbc.car.tesla.preap.nap_params import NAPParamKeys


class DrivingMannerismsLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    self._accel = BigMultiValueParamToggle(
      "acceleration",
      "NAPMapSpeedAccel",
      values=list(MAP_SPEED_ACCEL),
      labels=MAP_SPEED_ACCEL_LABELS,
      default_value=MAP_SPEED_ACCEL_DEFAULT,
    )

    adaptive_accel = BigParamControl("adaptive accel", NAPParamKeys.ADAPTIVE_ACCEL)
    adaptive_accel.set_value("Softer near a lead")

    self._follow_distance_city = BigMultiValueParamToggle(
      "city follow distance",
      NAP_FOLLOW_DISTANCE_CITY,
      values=FOLLOW_DISTANCE_VALUES,
      labels=FOLLOW_DISTANCE_LABELS,
      default_value=FOLLOW_DISTANCE_DEFAULT,
    )

    self._follow_distance_hwy = BigMultiValueParamToggle(
      "highway follow distance",
      NAP_FOLLOW_DISTANCE_HWY,
      values=FOLLOW_DISTANCE_VALUES,
      labels=FOLLOW_DISTANCE_LABELS,
      default_value=FOLLOW_DISTANCE_DEFAULT,
    )

    lat_handoff = BigParamControl("soft lateral handoff", NAP_DRIVER_LAT_HANDOFF)
    lat_handoff.set_value("On — free-wheel yield; Off if false-yield")

    one_pedal = BigParamControl("one-pedal long", NAP_ONE_PEDAL_LONG)
    one_pedal.set_value("Off default — gas pause stays until SET")

    self._scroller.add_widgets([
      self._accel,
      adaptive_accel,
      self._follow_distance_city,
      self._follow_distance_hwy,
      lat_handoff,
      one_pedal,
    ])

  def show_event(self):
    super().show_event()
    from openpilot.selfdrive.controls.lib.follow_distance import migrate_follow_distance_params
    from openpilot.common.params import Params
    migrate_follow_distance_params(Params())
    self._accel._load_value()
    self._follow_distance_city._load_value()
    self._follow_distance_hwy._load_value()


def open_driving_mannerisms_menu(page: DrivingMannerismsLayoutMici | None = None) -> DrivingMannerismsLayoutMici:
  panel = page or DrivingMannerismsLayoutMici()
  gui_app.push_widget(panel)
  return panel
