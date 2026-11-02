"""PR #391: LIMIT-plate long-press popup model (no pyray)."""
from openpilot.selfdrive.ui.onroad.limit_popup import HOLD_S, LimitPopupModel, hit_name, popup_rects


class Clock:
  def __init__(self):
    self.t = 100.0

  def __call__(self):
    return self.t


def mk():
  c, out = Clock(), []
  m = LimitPopupModel(writer=lambda k, v: out.append((k, v)), clock=c)
  return m, c, out


def open_popup(m, c, cur=30, tesla=30, osm=55):
  m.press_sign()
  c.t += HOLD_S
  assert m.tick(True, cur, tesla, osm, False)


def test_short_press_does_not_open():
  m, c, _ = mk()
  m.press_sign()
  c.t += 1.0
  assert not m.tick(True, 30, 30, 55, False)
  m.release()
  c.t += 2.0
  assert not m.tick(False, 30, 30, 55, False) and not m.is_open


def test_two_second_hold_opens_with_current_value():
  m, c, _ = mk()
  open_popup(m, c, cur=32)
  assert m.is_open and m.value == 30


def test_tap_osm_sets_it_immediately():
  m, c, out = mk()
  open_popup(m, c)
  assert m.tap("osm") == "osm"
  assert out == [("manual", 55)] and not m.is_open


def test_tap_tesla_and_missing_value():
  m, c, out = mk()
  open_popup(m, c, tesla=0)
  assert m.tap("tesla") is None and m.is_open and out == []
  m.tesla_mph = 30
  assert m.tap("tesla") == "tesla" and out == [("manual", 30)]


def test_arrows_then_save():
  m, c, out = mk()
  open_popup(m, c, cur=30)
  m.tap("up")
  m.tap("up")
  m.tap("down")
  assert m.value == 35
  m.tap("save")
  assert out == [("manual", 35)] and not m.is_open


def test_clear_source_and_close():
  m, c, out = mk()
  open_popup(m, c)
  assert m.tap("source") == "source" and m.is_open and out == [("source", True)]
  m.tap("clear")
  assert out[-1] == ("clear", True) and not m.is_open
  open_popup(m, c)
  assert m.tap("panel") is None and m.is_open
  assert m.tap(None) == "close" and not m.is_open


def test_layout_hits():
  r = popup_rects(0, 0, 2160, 1080)
  for k in ("tesla", "osm", "up", "down", "save", "clear", "source", "close"):
    x, y, w, h = r[k]
    assert hit_name(x + w / 2, y + h / 2, r) == k
  assert hit_name(5, 5, r) is None
