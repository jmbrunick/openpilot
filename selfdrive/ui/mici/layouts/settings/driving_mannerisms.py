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
  HYPERMILE_FOLLOW_DEFAULT,
  HYPERMILE_FOLLOW_LABELS,
  HYPERMILE_FOLLOW_VALUES,
  NAP_DM_SIMULATE_LOOKING,
  NAP_DRIVER_LAT_HANDOFF,
  NAP_HYPERMILE,
  NAP_HYPERMILE_FOLLOW_LEVEL,
  NAP_HYPERMILE_STEP_DOWN,
)
from opendbc.car.tesla.preap.nap_params import NAPParamKeys


class DrivingMannerismsLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    self._params = Params()

    def on_hypermile(checked):
      apply_hypermile_toggle(self._params, bool(checked))

    hypermile = BigParamControl("hypermile", NAP_HYPERMILE, toggle_callback=on_hypermile)
    hypermile.set_value("Off default — early light eco, not max regen")

    step_down = BigParamControl("step down speed", NAP_HYPERMILE_STEP_DOWN)
    step_down.set_value("Off — 15 mph under posted when On")
    step_down.set_visible(lambda: self._params.get_bool(NAP_HYPERMILE))

    adaptive_accel = BigParamControl("adaptive accel limits", NAPParamKeys.ADAPTIVE_ACCEL)

    follow_distance = BigMultiValueParamToggle(
      "follow distance",
      NAPParamKeys.FOLLOW_DISTANCE,
      values=FOLLOW_DISTANCE_VALUES,
      labels=FOLLOW_DISTANCE_LABELS,
      default_value=FOLLOW_DISTANCE_DEFAULT,
    )
    follow_distance.set_visible(lambda: not self._params.get_bool(NAP_HYPERMILE))

    hypermile_follow = BigMultiValueParamToggle(
      "hypermile follow",
      NAP_HYPERMILE_FOLLOW_LEVEL,
      values=HYPERMILE_FOLLOW_VALUES,
      labels=HYPERMILE_FOLLOW_LABELS,
      default_value=HYPERMILE_FOLLOW_DEFAULT,
    )
    hypermile_follow.set_visible(lambda: self._params.get_bool(NAP_HYPERMILE))

    lat_handoff = BigParamControl("soft lateral handoff", NAP_DRIVER_LAT_HANDOFF)
    lat_handoff.set_value("On — free-wheel yield; Off if false-yield")

    dm_sim_looking = BigParamControl("simulate look-at-road", NAP_DM_SIMULATE_LOOKING)
    dm_sim_looking.set_value("On — hold glance at random in 1–3 s of drain")

    self._scroller.add_widgets([
      hypermile,
      step_down,
      adaptive_accel,
      follow_distance,
      hypermile_follow,
      lat_handoff,
      dm_sim_looking,
    ])


def open_driving_mannerisms_menu(page: DrivingMannerismsLayoutMici | None = None) -> DrivingMannerismsLayoutMici:
  panel = page or DrivingMannerismsLayoutMici()
  gui_app.push_widget(panel)
  return panel
