"""nap-release switches: speed sign reader off, Roundabout Steering Assist off,
log-only wouldSteer off, and Force Offroad prompts only while moving in Drive."""
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.common import nap_release
from openpilot.system.hardware.nap_force_offroad import CONFIRMED_PARAM, PARAM, prompt_needed

ROOT = Path(__file__).resolve().parents[2]


class _Params:
  def __init__(self, **vals):
    self.vals = dict(vals)

  def get_bool(self, key, *a, **k):
    return bool(self.vals.get(key, False))

  def put_bool(self, key, value):
    self.vals[key] = bool(value)

  def get(self, key, *a, **k):
    return self.vals.get(key)


def test_release_flags():
  assert nap_release.NAP_RELEASE
  assert not nap_release.SPEED_SIGN_ENABLED
  assert not nap_release.ROUNDABOUT_ASSIST_ENABLED
  assert not nap_release.LOG_ONLY_EXTRAS


def test_speedsignd_never_starts_even_with_the_param_on():
  text = (ROOT / "system/manager/process_config.py").read_text()
  assert "SPEED_SIGN_ENABLED and started" in text
  from openpilot.system.manager.process_config import speed_sign_log
  assert not speed_sign_log(True, _Params(NAPSpeedSignLog=True), None)


def test_speed_sign_ui_is_hidden_but_schema_and_package_stay():
  tici = (ROOT / "selfdrive/ui/layouts/settings/nap.py").read_text()
  mici = (ROOT / "selfdrive/ui/mici/layouts/settings/nap.py").read_text()
  assert "if not SPEED_SIGN_ENABLED:" in tici and "del self._main_items[_speed_sign_start:]" in tici
  assert "if SPEED_SIGN_ENABLED else []" in mici
  for hud in ("selfdrive/ui/onroad/hud_renderer.py", "selfdrive/ui/mici/onroad/hud_renderer.py"):
    assert "if SPEED_SIGN_ENABLED:" in (ROOT / hud).read_text()
  ui_state = (ROOT / "selfdrive/ui/ui_state.py").read_text()
  assert "SPEED_SIGN_ENABLED and self.params.get_bool(\"NAPSpeedSignLog\")" in ui_state
  # Schema, service and the shared NV12 helper are kept.
  assert "liveSpeedSignNAP @108" in (ROOT / "cereal/log.capnp").read_text()
  assert '"liveSpeedSignNAP"' in (ROOT / "cereal/services.py").read_text()
  assert (ROOT / "selfdrive/speedsignd/nv12.py").exists()


def test_roundabout_assist_stays_off_with_the_param_on():
  from openpilot.selfdrive.controls.lib.roundabout_guide import RoundaboutAssist
  rb = RoundaboutAssist(True, _Params(NAPRoundaboutAssist=True))
  for _ in range(5):
    rb._read_toggle()
  assert rb.enabled is False
  ui = (ROOT / "selfdrive/ui/layouts/settings/driving_mannerisms.py").read_text()
  assert "if ROUNDABOUT_ASSIST_ENABLED:\n      self._all_items.append(self._rb_assist)" in ui


def test_cone_line_would_steer_is_not_logged_on_release():
  text = (ROOT / "selfdrive/controls/lib/cone_line.py").read_text()
  assert "dest.wouldSteer = float(sample.would_steer) if nap_release.LOG_ONLY_EXTRAS else 0.0" in text


def test_selfdrived_keeps_path_obstacle_literal():
  assert "'pathObstacleNAP'" in (ROOT / "selfdrive/selfdrived/selfdrived.py").read_text()


@pytest.mark.parametrize("drive,v,want", [
  (True, 12.0, True),
  (True, 0.01, True),
  (True, 0.0, False),
  (False, 5.0, False),   # Reverse / Neutral rolling: no prompt
  (False, 0.0, False),   # Park
  (True, float("nan"), True),  # unknown speed: ask
])
def test_force_offroad_prompt_needed(drive, v, want):
  assert prompt_needed(drive, v) is want


def _cs(gear, v):
  return SimpleNamespace(gearShifter=gear, vEgo=v)


def test_force_offroad_parked_goes_offroad_without_prompt(monkeypatch):
  pytest.importorskip("pyray")
  from openpilot.selfdrive.ui.onroad import force_offroad_confirm as fc
  from cereal import car
  shown = []
  monkeypatch.setattr(fc, "show_force_offroad_confirm", lambda: shown.append(1) or True)
  park = car.CarState.GearShifter.park
  drive = car.CarState.GearShifter.drive
  for cs in (_cs(park, 0.0), _cs(drive, 0.0)):
    params = _Params(**{PARAM: True})
    assert fc.maybe_show_force_offroad_confirm(params, started=True, car_state=cs) is False
    assert params.get_bool(CONFIRMED_PARAM) is True
  assert shown == []
  # Rolling in Drive: the Yes/No is shown and nothing is confirmed yet.
  params = _Params(**{PARAM: True})
  assert fc.maybe_show_force_offroad_confirm(params, started=True, car_state=_cs(drive, 8.0)) is True
  assert shown == [1]
  assert params.get_bool(CONFIRMED_PARAM) is False
