"""On-road Yes/No before Force Offroad stock-CC handoff.

Justin: big, hard to miss while driving. Same ConfirmDialog stack as
other 3X NAP modals (gui_app.push_widget), larger type + buttons.
Not a Settings row.
"""
from collections.abc import Callable

import pyray as rl

from openpilot.common.params import Params
from openpilot.system.hardware.nap_force_offroad import (
  CONFIRM_NO,
  CONFIRM_PROMPT,
  CONFIRM_YES,
  CONFIRMED_PARAM,
  HANDOFF_READY_PARAM,
  PARAM,
  apply_force_offroad_toggle,
  cancel_force_offroad,
  confirm_force_offroad,
  needs_driver_confirm,
)
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.label import Label

# Larger than stock ConfirmDialog (70 / 160) so it reads at speed.
_PROMPT_SIZE = 90
_BUTTON_HEIGHT = 200
_BUTTON_FONT = 80
_OUTER = 80
_MARGIN = 40
_DIM = rl.Color(0, 0, 0, 180)
_CARD = rl.Color(27, 27, 27, 255)


class ForceOffroadConfirmDialog(ConfirmDialog):
  """Full-screen dim + large Yes / No. Copy is Justin's prompt."""

  def __init__(self, callback: Callable[[DialogResult], None] | None = None):
    super().__init__(CONFIRM_PROMPT, CONFIRM_YES, cancel_text=CONFIRM_NO, callback=callback)
    self._label = Label(
      CONFIRM_PROMPT, _PROMPT_SIZE, FontWeight.BOLD,
      text_color=rl.Color(201, 201, 201, 255),
    )
    self._cancel_button = Button(
      CONFIRM_NO, self._cancel_button_callback, font_size=_BUTTON_FONT,
    )
    self._confirm_button = Button(
      CONFIRM_YES, self._confirm_button_callback, font_size=_BUTTON_FONT,
      button_style=ButtonStyle.PRIMARY,
    )

  def _render(self, rect: rl.Rectangle):
    rl.draw_rectangle_rec(rect, _DIM)
    dialog_x = _OUTER
    dialog_y = _OUTER
    dialog_width = gui_app.width - 2 * dialog_x
    dialog_height = gui_app.height - 2 * dialog_y
    dialog_rect = rl.Rectangle(dialog_x, dialog_y, dialog_width, dialog_height)

    bottom = dialog_rect.y + dialog_rect.height
    button_width = (dialog_rect.width - 3 * _MARGIN) // 2
    cancel_button_x = dialog_rect.x + _MARGIN
    confirm_button_x = dialog_rect.x + dialog_rect.width - button_width - _MARGIN
    button_y = bottom - _BUTTON_HEIGHT - _MARGIN
    cancel_button = rl.Rectangle(cancel_button_x, button_y, button_width, _BUTTON_HEIGHT)
    confirm_button = rl.Rectangle(confirm_button_x, button_y, button_width, _BUTTON_HEIGHT)

    rl.draw_rectangle_rec(dialog_rect, _CARD)

    text_rect = rl.Rectangle(
      dialog_rect.x + _MARGIN, dialog_rect.y + 24,
      dialog_rect.width - 2 * _MARGIN,
      dialog_rect.height - _BUTTON_HEIGHT - _MARGIN - 48,
    )
    self._label.render(text_rect)

    if rl.is_key_pressed(rl.KeyboardKey.KEY_ENTER):
      self._confirm_button_callback()
    elif rl.is_key_pressed(rl.KeyboardKey.KEY_ESCAPE):
      self._cancel_button_callback()

    self._confirm_button.render(confirm_button)
    self._cancel_button.render(cancel_button)


_dialog_open = False


def _on_result(result: DialogResult):
  global _dialog_open
  _dialog_open = False
  params = Params()
  if result == DialogResult.CONFIRM:
    confirm_force_offroad(params)
  else:
    cancel_force_offroad(params)


def show_force_offroad_confirm() -> bool:
  """Push the dialog once. Returns True if it was shown this call."""
  global _dialog_open
  if _dialog_open:
    return False
  _dialog_open = True
  gui_app.push_widget(ForceOffroadConfirmDialog(callback=_on_result))
  return True


def maybe_show_force_offroad_confirm(params=None, started: bool | None = None) -> bool:
  """MainLayout tick: show when onroad Force Offroad is waiting for Yes."""
  if params is None:
    params = Params()
  if started is None:
    from openpilot.selfdrive.ui.ui_state import ui_state
    started = bool(ui_state.started)
  force = bool(params.get_bool(PARAM))
  confirmed = bool(params.get_bool(CONFIRMED_PARAM))
  if not force:
    if confirmed or params.get_bool(HANDOFF_READY_PARAM):
      apply_force_offroad_toggle(params, False, started=False)
    return False
  if not needs_driver_confirm(force, confirmed, started):
    return False
  return show_force_offroad_confirm()
