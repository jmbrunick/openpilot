"""TICI side popup: Force Offroad + Simulate Look + False Alert Ignore.

Opened by a NAP sidebar triple-tap. Not a full settings page and not part
of Driving Mannerisms. Tap outside the card (primary), the X, Escape, or
the NAP sidebar item again to dismiss. Simulate Look and False Alert
Ignore are mutually exclusive — enabling one clears the other.
"""
from collections.abc import Callable

import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.monitoring.dm_toggles import (
  apply_dm_false_alert_ignore,
  apply_dm_simulate_looking,
  read_exclusive_dm_toggles,
)
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  DM_FALSE_ALERT_IGNORE_DESCRIPTION,
  DM_SIMULATE_LOOKING_DESCRIPTION,
  FORCE_OFFROAD_DESCRIPTION,
  NAP_DM_FALSE_ALERT_IGNORE,
  NAP_DM_SIMULATE_LOOKING,
  NAP_FORCE_OFFROAD,
)
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.hardware.nap_force_offroad import apply_force_offroad_toggle
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.system.ui.widgets.list_view import ITEM_BASE_HEIGHT, toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller

CARD_WIDTH = 980
CARD_HEIGHT = 630
CARD_MARGIN = 24
HEADER_H = 72
CLOSE_SIZE = 64
DIM = rl.Color(0, 0, 0, 140)
CARD_BG = rl.Color(30, 30, 30, 255)
TITLE_COLOR = rl.Color(201, 201, 201, 255)


class HiddenTogglesPopup(Widget):
  """Compact side card with the three hidden NAP toggles."""

  def __init__(self, on_dismiss: Callable[[], None]):
    super().__init__()
    self._on_dismiss = on_dismiss
    self._params = Params()
    self._card_rect = rl.Rectangle(0, 0, 0, 0)
    self._font = gui_app.font(FontWeight.BOLD)

    self._offroad_item = toggle_item(
      "Force Offroad",
      description=FORCE_OFFROAD_DESCRIPTION,
      initial_state=self._params.get_bool(NAP_FORCE_OFFROAD),
      callback=self._on_force_offroad,
    )
    self._dm_item = toggle_item(
      "Simulate Look",
      description=DM_SIMULATE_LOOKING_DESCRIPTION,
      initial_state=self._params.get_bool(NAP_DM_SIMULATE_LOOKING),
      callback=self._on_dm_sim_looking,
    )
    self._fai_item = toggle_item(
      "False Alert Ignore",
      description=DM_FALSE_ALERT_IGNORE_DESCRIPTION,
      initial_state=self._params.get_bool(NAP_DM_FALSE_ALERT_IGNORE),
      callback=self._on_false_alert_ignore,
    )
    self._scroller = Scroller(
      [self._offroad_item, self._dm_item, self._fai_item],
      line_separator=True,
      spacing=0,
    )
    self._close_btn = Button(
      "X",
      click_callback=self._dismiss,
      font_size=40,
      button_style=ButtonStyle.LIST_ACTION,
      border_radius=32,
      text_padding=0,
    )

  def _on_dm_sim_looking(self, state):
    apply_dm_simulate_looking(self._params, bool(state))
    self._fai_item.action_item.set_state(self._params.get_bool(NAP_DM_FALSE_ALERT_IGNORE))

  def _on_false_alert_ignore(self, state):
    apply_dm_false_alert_ignore(self._params, bool(state))
    self._dm_item.action_item.set_state(self._params.get_bool(NAP_DM_SIMULATE_LOOKING))

  def _on_force_offroad(self, state):
    apply_force_offroad_toggle(self._params, bool(state), started=bool(ui_state.started))

  def refresh(self):
    read_exclusive_dm_toggles(self._params)
    self._dm_item.action_item.set_state(self._params.get_bool(NAP_DM_SIMULATE_LOOKING))
    self._fai_item.action_item.set_state(self._params.get_bool(NAP_DM_FALSE_ALERT_IGNORE))
    self._offroad_item.action_item.set_state(self._params.get_bool(NAP_FORCE_OFFROAD))

  def show_event(self):
    super().show_event()
    self.refresh()

  def contains_card(self, mouse_pos: MousePos) -> bool:
    return rl.check_collision_point_rec(mouse_pos, self._card_rect)

  def _dismiss(self):
    self._on_dismiss()

  def _layout(self):
    width = min(CARD_WIDTH, max(620, self._rect.width - 2 * CARD_MARGIN))
    height = min(CARD_HEIGHT, max(480, self._rect.height - 2 * CARD_MARGIN))
    self._card_rect = rl.Rectangle(
      self._rect.x + CARD_MARGIN,
      self._rect.y + CARD_MARGIN,
      width,
      height,
    )

  def _handle_mouse_release(self, mouse_pos: MousePos) -> None:
    if not self.contains_card(mouse_pos):
      self._dismiss()

  def _render(self, rect: rl.Rectangle):
    rl.draw_rectangle_rec(rect, DIM)
    rl.draw_rectangle_rounded(self._card_rect, 0.04, 20, CARD_BG)

    close_rect = rl.Rectangle(
      self._card_rect.x + self._card_rect.width - CLOSE_SIZE - 16,
      self._card_rect.y + (HEADER_H - CLOSE_SIZE) / 2,
      CLOSE_SIZE,
      CLOSE_SIZE,
    )
    title_rect = rl.Rectangle(
      self._card_rect.x + 28,
      self._card_rect.y,
      self._card_rect.width - CLOSE_SIZE - 50,
      HEADER_H,
    )
    rl.draw_text_ex(
      self._font, "NAP",
      rl.Vector2(title_rect.x, title_rect.y + 18),
      40, 0, TITLE_COLOR,
    )
    self._close_btn.render(close_rect)

    content = rl.Rectangle(
      self._card_rect.x + 8,
      self._card_rect.y + HEADER_H,
      self._card_rect.width - 16,
      self._card_rect.height - HEADER_H - 12,
    )
    # Three 170px rows; keep a floor so the scroller can expand descriptions.
    if content.height < ITEM_BASE_HEIGHT * 3:
      content.height = ITEM_BASE_HEIGHT * 3
    self._scroller.render(content)
    # No on the confirm dialog clears the param; keep the switch in sync.
    self._offroad_item.action_item.set_state(self._params.get_bool(NAP_FORCE_OFFROAD))

    if rl.is_key_pressed(rl.KeyboardKey.KEY_ESCAPE):
      self._dismiss()
