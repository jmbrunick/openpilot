"""TICI NAP submenu: Driving Mannerisms controls."""
from openpilot.common.params import Params
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import toggle_item, multiple_button_item, button_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.selfdrive.controls.lib.hypermile import apply_hypermile_toggle
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  DRIVER_LAT_HANDOFF_DESCRIPTION,
  DM_SIMULATE_LOOKING_DESCRIPTION,
  HYPERMILE_DESCRIPTION,
  HYPERMILE_FOLLOW_DESCRIPTION,
  HYPERMILE_HILL_CLIMB_DESCRIPTION,
  HYPERMILE_STEP_DOWN_DESCRIPTION,
  NAP_DM_SIMULATE_LOOKING,
  NAP_DRIVER_LAT_HANDOFF,
  NAP_HYPERMILE,
  NAP_HYPERMILE_FOLLOW_LEVEL,
  NAP_HYPERMILE_HILL_CLIMB,
  NAP_HYPERMILE_STEP_DOWN,
)
from opendbc.car.tesla.preap.nap_params import NAPParamKeys


class DrivingMannerismsLayout(Widget):
  """Nested NAP page for Hypermile, accel feel, follow distance, soft-lat, and DM look-sim."""

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

    self._hypermile = toggle_item(
      "Hypermile",
      description=HYPERMILE_DESCRIPTION,
      initial_state=self._params.get_bool(NAP_HYPERMILE),
      callback=self._on_hypermile,
    )
    self._all_items.append(self._hypermile)

    self._step_down = toggle_item(
      "Step Down Speed",
      description=HYPERMILE_STEP_DOWN_DESCRIPTION,
      initial_state=self._params.get_bool(NAP_HYPERMILE_STEP_DOWN),
      callback=self._on_step_down,
    )
    self._all_items.append(self._step_down)

    self._hill_climb = toggle_item(
      "Hill Climb",
      description=HYPERMILE_HILL_CLIMB_DESCRIPTION,
      initial_state=self._params.get_bool(NAP_HYPERMILE_HILL_CLIMB),
      callback=self._on_hill_climb,
    )
    self._all_items.append(self._hill_climb)

    self._adaptive_accel = toggle_item(
      "Adaptive Accel Limits",
      description="Reduces acceleration authority when close to a lead car to prevent overshoot. Full accel on open road (no lead). Coming up behind a lead uses the Accel 1–10 close cap, not a full-gap punch.",
      initial_state=self._params.get_bool(NAPParamKeys.ADAPTIVE_ACCEL),
      callback=self._on_adaptive_accel,
    )
    self._all_items.append(self._adaptive_accel)

    follow_dist = self._params.get(NAPParamKeys.FOLLOW_DISTANCE, return_default=True)
    self._follow_buttons = multiple_button_item(
      "Follow Distance",
      "Follow distance (1=closest, 7=farthest). A slower car ahead starts a " +
      "gradual ease-off farther back (more distance, not a harder brake). " +
      "Overridden by cruise stalk if present. Hidden while Hypermile is On.",
      buttons=["1", "2", "3", "4", "5", "6", "7"],
      button_width=80,
      selected_index=max(0, min(6, follow_dist - 1)),
      callback=self._on_follow_distance,
    )
    self._all_items.append(self._follow_buttons)

    hm_level = int(self._params.get(NAP_HYPERMILE_FOLLOW_LEVEL, return_default=True) or 3)
    self._hypermile_follow = multiple_button_item(
      "Hypermile Follow",
      HYPERMILE_FOLLOW_DESCRIPTION,
      buttons=["1", "2", "3", "4", "5"],
      button_width=80,
      selected_index=max(0, min(4, hm_level - 1)),
      callback=self._on_hypermile_follow,
    )
    self._all_items.append(self._hypermile_follow)

    self._lat_handoff = toggle_item(
      "Soft Lateral Handoff",
      description=DRIVER_LAT_HANDOFF_DESCRIPTION,
      initial_state=self._params.get_bool(NAP_DRIVER_LAT_HANDOFF),
      callback=self._on_lat_handoff,
    )
    self._all_items.append(self._lat_handoff)

    self._dm_sim_looking = toggle_item(
      "Simulate Look-at-Road",
      description=DM_SIMULATE_LOOKING_DESCRIPTION,
      initial_state=self._params.get_bool(NAP_DM_SIMULATE_LOOKING),
      callback=self._on_dm_sim_looking,
    )
    self._all_items.append(self._dm_sim_looking)

  def _on_hypermile(self, state):
    apply_hypermile_toggle(self._params, bool(state))
    self.refresh()

  def _on_step_down(self, state):
    self._params.put_bool(NAP_HYPERMILE_STEP_DOWN, state)

  def _on_hill_climb(self, state):
    self._params.put_bool(NAP_HYPERMILE_HILL_CLIMB, state)

  def _on_adaptive_accel(self, state):
    self._params.put_bool(NAPParamKeys.ADAPTIVE_ACCEL, state)

  def _on_follow_distance(self, index: int):
    self._params.put(NAPParamKeys.FOLLOW_DISTANCE, index + 1)

  def _on_hypermile_follow(self, index: int):
    self._params.put(NAP_HYPERMILE_FOLLOW_LEVEL, index + 1)

  def _on_lat_handoff(self, state):
    self._params.put_bool(NAP_DRIVER_LAT_HANDOFF, state)

  def _on_dm_sim_looking(self, state):
    self._params.put_bool(NAP_DM_SIMULATE_LOOKING, state)

  def refresh(self):
    hypermile_on = self._params.get_bool(NAP_HYPERMILE)
    self._hypermile.action_item.set_state(hypermile_on)
    self._step_down.action_item.set_state(self._params.get_bool(NAP_HYPERMILE_STEP_DOWN))
    self._step_down.set_visible(hypermile_on)
    self._hill_climb.action_item.set_state(self._params.get_bool(NAP_HYPERMILE_HILL_CLIMB))
    self._hill_climb.set_visible(hypermile_on)
    self._adaptive_accel.action_item.set_state(self._params.get_bool(NAPParamKeys.ADAPTIVE_ACCEL))
    follow_dist = self._params.get(NAPParamKeys.FOLLOW_DISTANCE, return_default=True)
    self._follow_buttons.action_item.set_selected_button(max(0, min(6, follow_dist - 1)))
    hm_level = int(self._params.get(NAP_HYPERMILE_FOLLOW_LEVEL, return_default=True) or 3)
    self._hypermile_follow.action_item.set_selected_button(max(0, min(4, hm_level - 1)))
    self._follow_buttons.set_visible(not hypermile_on)
    self._hypermile_follow.set_visible(hypermile_on)
    self._lat_handoff.action_item.set_state(self._params.get_bool(NAP_DRIVER_LAT_HANDOFF))
    self._dm_sim_looking.action_item.set_state(self._params.get_bool(NAP_DM_SIMULATE_LOOKING))

  def show_event(self):
    self._scroller.show_event()
    self.refresh()

  def _render(self, rect):
    self._scroller.render(rect)
