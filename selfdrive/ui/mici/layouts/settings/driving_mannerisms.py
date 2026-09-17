"""mici NAP submenu: Driving Mannerisms controls."""
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.big_multi_value_param import BigMultiValueParamToggle
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl
from openpilot.selfdrive.controls.lib.hypermile import apply_hypermile_toggle
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  FOLLOW_DISTANCE_DEFAULT,
  FOLLOW_DISTANCE_LABELS,
  FOLLOW_DISTANCE_VALUES,
  MAP_SPEED_ACCEL, MAP_SPEED_ACCEL_DEFAULT, MAP_SPEED_ACCEL_LABELS,
  NAP_DRIVER_LAT_HANDOFF,
  NAP_FOLLOW_DISTANCE_CITY,
  NAP_FOLLOW_DISTANCE_HWY,
  NAP_HYPERMILE,
  NAP_HYPERMILE_HILL_CLIMB,
  NAP_HYPERMILE_STEP_DOWN,
  NAP_ONE_PEDAL_LONG,
)
from opendbc.car.tesla.preap.nap_params import NAPParamKeys


class DrivingMannerismsLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    self._params = Params()

    def on_hypermile(checked):
      apply_hypermile_toggle(self._params, bool(checked))
      self._accel.refresh()

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

    hypermile = BigParamControl("hypermile", NAP_HYPERMILE, toggle_callback=on_hypermile)
    hypermile.set_value("Off default — early light eco, not max regen")

    step_down = BigParamControl("step down speed", NAP_HYPERMILE_STEP_DOWN)
    step_down.set_value("Off — scaled drop, −15 at 80 when On")
    step_down.set_visible(lambda: self._params.get_bool(NAP_HYPERMILE))

    hill_climb = BigParamControl("hill climb", NAP_HYPERMILE_HILL_CLIMB)
    hill_climb.set_value("On — grade hold on climbs; light crest ease")
    hill_climb.set_visible(lambda: self._params.get_bool(NAP_HYPERMILE))

    self._scroller.add_widgets([
      self._accel,
      adaptive_accel,
      self._follow_distance_city,
      self._follow_distance_hwy,
      lat_handoff,
      one_pedal,
      hypermile,
      step_down,
      hill_climb,
    ])

  def show_event(self):
    super().show_event()
    from openpilot.selfdrive.controls.lib.follow_distance import migrate_follow_distance_params
    migrate_follow_distance_params(self._params)
    self._accel.refresh()
    self._follow_distance_city._load_value()
    self._follow_distance_hwy._load_value()


def open_driving_mannerisms_menu(page: DrivingMannerismsLayoutMici | None = None) -> DrivingMannerismsLayoutMici:
  panel = page or DrivingMannerismsLayoutMici()
  gui_app.push_widget(panel)
  return panel
