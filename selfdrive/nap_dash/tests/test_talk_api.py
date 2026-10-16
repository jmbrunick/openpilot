from pathlib import Path

from openpilot.selfdrive.nap_dash.talk_api import STALE_S, TalkFlag, parse_active

HERE = Path(__file__).resolve().parents[1]


class Clock:
  def __init__(self):
    self.t = 100.0

  def __call__(self):
    return self.t


def test_tap_is_consumed_once():
  c = Clock()
  f = TalkFlag(c)
  assert f.poll()["talk"] is False
  f.tap()
  assert f.state()["pending"] is True
  assert f.poll()["talk"] is True
  assert f.poll()["talk"] is False


def test_stale_tap_dropped_and_active_reported():
  c = Clock()
  f = TalkFlag(c)
  f.tap()
  c.t += STALE_S + 1
  assert f.poll(active=True)["talk"] is False
  assert f.state()["active"] is True
  c.t += 10  # phone stopped polling
  assert f.state()["active"] is False


def test_parse_active():
  assert parse_active("active=1") is True
  assert parse_active("active=0") is False
  assert parse_active("") is None


def test_talk_api_has_no_car_paths():
  src = (HERE / "talk_api.py").read_text(encoding="utf-8")
  imports = [ln for ln in src.splitlines() if ln.startswith(("import ", "from "))]
  for bad in ("params", "messaging", "cereal", "panda", "opendbc", "selfdrive.car"):
    assert not any(bad in ln for ln in imports), imports
  html = (HERE / "dashboard.html").read_text(encoding="utf-8")
  assert 'tk.id = "napTalk"' in html
  assert html.index('bm.id = "bookmark"') < html.index('tk.id = "napTalk"')
