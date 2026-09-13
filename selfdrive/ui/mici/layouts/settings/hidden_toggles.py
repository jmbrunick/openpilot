"""mici overlay: Force Offroad + Simulate Look + False Alert Ignore.

Opened by a triple-tap on the Settings **nap** button. Tap outside the
card to dismiss. Not shown on Driving Mannerisms or the main NAP list.
Simulate Look and False Alert Ignore are mutually exclusive.
"""
import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.monitoring.dm_toggles import (
  apply_dm_false_alert_ignore,
  apply_dm_simulate_looking,
  read_exclusive_dm_toggles,
)
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  NAP_DM_FALSE_ALERT_IGNORE,
  NAP_DM_SIMULATE_LOOKING,
  NAP_FORCE_OFFROAD,
)
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.hardware.nap_force_offroad import apply_force_offroad_toggle
from openpilot.system.ui.lib.application import MousePos
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl

DIM = rl.Color(0, 0, 0, 160)
CARD_BG = rl.Color(40, 40, 40, 255)
CARD_PAD = 16


class HiddenTogglesOverlayMici(Widget):
  """Thin full-screen overlay with the three hidden NAP toggles."""

  def __init__(self):
    super().__init__()
    self._params = Params()
    self._card_rect = rl.Rectangle(0, 0, 0, 0)

    # Must stay enabled while onroad — that is the point of Force Offroad.
    self._offroad = BigParamControl("force offroad", NAP_FORCE_OFFROAD,
                                    toggle_callback=self._on_force_offroad)
    self._offroad.set_value("WARNING: stops OP — drive manually")

    self._dm = BigParamControl("simulate look", NAP_DM_SIMULATE_LOOKING,
                               toggle_callback=self._on_simulate_look)
    self._dm.set_value("On — no-face glance in 1–3 s of drain")

    self._fai = BigParamControl("false alert ignore", NAP_DM_FALSE_ALERT_IGNORE,
                                toggle_callback=self._on_false_alert_ignore)
    self._fai.set_value("On — ignore false phone alerts")

  def _on_force_offroad(self, state):
    apply_force_offroad_toggle(self._params, bool(state), started=bool(ui_state.started))

  def _on_simulate_look(self, state):
    apply_dm_simulate_looking(self._params, bool(state))
    self._fai.refresh()

  def _on_false_alert_ignore(self, state):
    apply_dm_false_alert_ignore(self._params, bool(state))
    self._dm.refresh()

  def show_event(self):
    super().show_event()
    read_exclusive_dm_toggles(self._params)
    self._dm.refresh()
    self._fai.refresh()
    self._offroad.refresh()

  def _layout(self):
    w1, h1 = self._dm.rect.width, self._dm.rect.height
    w2, h2 = self._offroad.rect.width, self._offroad.rect.height
    w3, h3 = self._fai.rect.width, self._fai.rect.height
    inner_w = max(w1, w2, w3)
    inner_h = h1 + h2 + h3 + 2 * CARD_PAD
    card_w = inner_w + 2 * CARD_PAD
    card_h = inner_h + 2 * CARD_PAD
    # Left-biased card so it reads as a side panel, not a full page.
    x = self._rect.x + max(20, (self._rect.width - card_w) * 0.12)
    y = self._rect.y + max(20, (self._rect.height - card_h) / 2)
    self._card_rect = rl.Rectangle(x, y, card_w, card_h)

  def _handle_mouse_release(self, mouse_pos: MousePos) -> None:
    if not rl.check_collision_point_rec(mouse_pos, self._card_rect):
      self.dismiss()

  def _render(self, _):
    rl.draw_rectangle_rec(self._rect, DIM)
    rl.draw_rectangle_rounded(self._card_rect, 0.12, 10, CARD_BG)

    x = self._card_rect.x + CARD_PAD
    y = self._card_rect.y + CARD_PAD
    self._offroad.render(rl.Rectangle(x, y, self._offroad.rect.width, self._offroad.rect.height))
    y += self._offroad.rect.height + CARD_PAD
    self._dm.render(rl.Rectangle(x, y, self._dm.rect.width, self._dm.rect.height))
    y += self._dm.rect.height + CARD_PAD
    self._fai.render(rl.Rectangle(x, y, self._fai.rect.width, self._fai.rect.height))
    self._offroad.refresh()
