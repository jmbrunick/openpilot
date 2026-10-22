"""MCU Yes/No must close the comma's Force Offroad dialog (one prompt per action).

Loads selfdrive/ui/onroad/force_offroad_confirm.py with the GUI stubbed so it
runs without pyray / a display.
"""
import importlib.util
import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


class _Params(dict):
  def get_bool(self, k):
    return bool(self.get(k, False))

  def put_bool(self, k, v):
    self[k] = bool(v)


class _App:
  def __init__(self):
    self._nav_stack = ["main"]
    self.ticks = []

  def push_widget(self, w):
    self._nav_stack.append(w)

  def pop_widget(self, idx=None):
    self._nav_stack.pop(len(self._nav_stack) - 1 if idx is None else idx)

  def add_nav_stack_tick(self, fn):
    if fn not in self.ticks:
      self.ticks.append(fn)

  def remove_nav_stack_tick(self, fn):
    if fn in self.ticks:
      self.ticks.remove(fn)


def _load(monkeypatch):
  app = _App()

  class _Dialog:
    def __init__(self, *a, callback=None, **k):
      self._callback = callback

  stubs = {
    "pyray": types.SimpleNamespace(Color=lambda *a: a, Rectangle=lambda *a: a),
    "openpilot.common.params": types.SimpleNamespace(Params=_Params),
    "openpilot.system.ui.lib.application": types.SimpleNamespace(gui_app=app, FontWeight=types.SimpleNamespace(BOLD=0)),
    "openpilot.system.ui.widgets": types.SimpleNamespace(DialogResult=types.SimpleNamespace(CONFIRM=1, CANCEL=0)),
    "openpilot.system.ui.widgets.button": types.SimpleNamespace(Button=object, ButtonStyle=types.SimpleNamespace(PRIMARY=0)),
    "openpilot.system.ui.widgets.confirm_dialog": types.SimpleNamespace(ConfirmDialog=_Dialog),
    "openpilot.system.ui.widgets.label": types.SimpleNamespace(Label=object),
  }
  spec = importlib.util.spec_from_file_location("nfo", ROOT / "system/hardware/nap_force_offroad.py")
  nfo = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(nfo)
  stubs["openpilot.system.hardware.nap_force_offroad"] = nfo
  for k, v in stubs.items():
    monkeypatch.setitem(sys.modules, k, v)
  spec = importlib.util.spec_from_file_location("foc", ROOT / "selfdrive/ui/onroad/force_offroad_confirm.py")
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  mod.ForceOffroadConfirmDialog.__init__ = lambda self, callback=None: setattr(self, "_callback", callback)
  return mod, app, nfo


def _pending(nfo):
  p = _Params()
  nfo.apply_force_offroad_toggle(p, True, started=True)
  return p


def test_mcu_yes_closes_comma_dialog_without_rewriting(monkeypatch):
  mod, app, nfo = _load(monkeypatch)
  p = _pending(nfo)
  assert mod.maybe_show_force_offroad_confirm(p, started=True)
  assert len(app._nav_stack) == 2
  nfo.confirm_force_offroad(p)  # what nap_dash POST /api/force_offroad yes does
  assert not mod.maybe_show_force_offroad_confirm(p, started=True)
  assert app._nav_stack == ["main"]
  assert p.get_bool(nfo.PARAM) and p.get_bool(nfo.CONFIRMED_PARAM)


def test_mcu_no_closes_comma_dialog(monkeypatch):
  mod, app, nfo = _load(monkeypatch)
  p = _pending(nfo)
  mod.maybe_show_force_offroad_confirm(p, started=True)
  nfo.cancel_force_offroad(p)
  mod.maybe_show_force_offroad_confirm(p, started=True)
  assert app._nav_stack == ["main"]
  assert not p.get_bool(nfo.PARAM)


def test_next_request_shows_again(monkeypatch):
  mod, app, nfo = _load(monkeypatch)
  p = _pending(nfo)
  mod.maybe_show_force_offroad_confirm(p, started=True)
  nfo.confirm_force_offroad(p)
  mod.maybe_show_force_offroad_confirm(p, started=True)
  nfo.apply_force_offroad_toggle(p, False, started=False)
  p = _pending(nfo)
  assert mod.maybe_show_force_offroad_confirm(p, started=True)


def _arm_tick(monkeypatch, mod, p):
  """The frame tick reads Params() and ui_state.started, not the test's arguments."""
  mod.Params = lambda: p
  ui = types.SimpleNamespace(started=True)
  monkeypatch.setitem(sys.modules, "openpilot.selfdrive.ui.ui_state", types.SimpleNamespace(ui_state=ui))


def test_mcu_yes_closes_comma_dialog_from_the_nav_tick(monkeypatch):
  """C3X does not render MainLayout while this dialog is on top, so nothing calls maybe_show again."""
  mod, app, nfo = _load(monkeypatch)
  p = _pending(nfo)
  _arm_tick(monkeypatch, mod, p)
  assert mod.maybe_show_force_offroad_confirm(p, started=True)
  assert len(app.ticks) == 1
  nfo.confirm_force_offroad(p)
  app.ticks[0]()
  assert app._nav_stack == ["main"]
  assert p.get_bool(nfo.PARAM) and p.get_bool(nfo.CONFIRMED_PARAM)
  app.ticks[0]()
  assert app._nav_stack == ["main"]
  assert p.get_bool(nfo.CONFIRMED_PARAM)


def test_mcu_no_closes_comma_dialog_from_the_nav_tick(monkeypatch):
  mod, app, nfo = _load(monkeypatch)
  p = _pending(nfo)
  _arm_tick(monkeypatch, mod, p)
  mod.maybe_show_force_offroad_confirm(p, started=True)
  nfo.cancel_force_offroad(p)
  app.ticks[0]()
  assert app._nav_stack == ["main"]
  assert not p.get_bool(nfo.PARAM)
  app.ticks[0]()
  assert app._nav_stack == ["main"]
  assert not p.get_bool(nfo.PARAM)


def _mcu_pending(nfo, p) -> bool:
  """Same condition as nap_dash force_offroad_api.read_ack."""
  return nfo.needs_driver_confirm(
    p.get_bool(nfo.PARAM), p.get_bool(nfo.CONFIRMED_PARAM), p.get_bool("IsOnroad"),
  )


def test_comma_yes_and_no_clear_the_mcu_prompt(monkeypatch):
  mod, app, nfo = _load(monkeypatch)
  p = _pending(nfo)
  p.put_bool("IsOnroad", True)
  _arm_tick(monkeypatch, mod, p)
  mod.maybe_show_force_offroad_confirm(p, started=True)
  assert _mcu_pending(nfo, p) is True
  app.pop_widget()
  mod._on_result(mod.DialogResult.CONFIRM)
  assert app._nav_stack == ["main"]
  assert _mcu_pending(nfo, p) is False
  app.ticks[0]()
  assert p.get_bool(nfo.PARAM) and p.get_bool(nfo.CONFIRMED_PARAM)

  p2 = _pending(nfo)
  p2.put_bool("IsOnroad", True)
  _arm_tick(monkeypatch, mod, p2)
  mod.maybe_show_force_offroad_confirm(p2, started=True)
  app.pop_widget()
  mod._on_result(mod.DialogResult.CANCEL)
  assert app._nav_stack == ["main"]
  assert _mcu_pending(nfo, p2) is False
  assert not p2.get_bool(nfo.PARAM)
