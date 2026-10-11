"""NAP Force Offroad: param, hardwared started gate, settings, docs."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# Load the helper without importing system.hardware (cereal / params_pyx).
_spec = importlib.util.spec_from_file_location(
  "nap_force_offroad", ROOT / "system" / "hardware" / "nap_force_offroad.py",
)
_mod = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
_spec.loader.exec_module(_mod)
PARAM = _mod.PARAM
HANDOFF_READY_PARAM = _mod.HANDOFF_READY_PARAM
CONFIRMED_PARAM = _mod.CONFIRMED_PARAM
CONFIRM_PROMPT = _mod.CONFIRM_PROMPT
allows_onroad = _mod.allows_onroad
apply_force_offroad_toggle = _mod.apply_force_offroad_toggle
cancel_force_offroad = _mod.cancel_force_offroad
confirm_force_offroad = _mod.confirm_force_offroad
handoff_wait_timed_out = _mod.handoff_wait_timed_out
needs_driver_confirm = _mod.needs_driver_confirm
should_start_now = _mod.should_start_now


def _assert_force_offroad_param_line(keys: str, name: str):
  line = next(ln for ln in keys.splitlines() if f'"{name}"' in ln)
  assert "CLEAR_ON_MANAGER_START" in line
  assert "CLEAR_ON_IGNITION_ON" in line
  assert "PERSISTENT" not in line
  # Going offroad is what the toggle does — that flag would undo it.
  assert "CLEAR_ON_OFFROAD" not in line
  assert 'BOOL, "0"' in line


def test_param_default_off_and_clears_on_ignition_and_manager():
  keys = (ROOT / "common" / "params_keys.h").read_text(encoding="utf-8")
  assert PARAM == "NAPForceOffroad"
  assert HANDOFF_READY_PARAM == "NAPForceOffroadHandoffReady"
  assert CONFIRMED_PARAM == "NAPForceOffroadConfirmed"
  assert CONFIRM_PROMPT == "Ready to resume steering control?"
  _assert_force_offroad_param_line(keys, PARAM)
  _assert_force_offroad_param_line(keys, HANDOFF_READY_PARAM)
  _assert_force_offroad_param_line(keys, CONFIRMED_PARAM)


def test_allows_onroad_default_and_toggle():
  assert allows_onroad(False) is True
  assert allows_onroad(True) is False


def test_allows_onroad_holds_while_handoff_pending():
  """Already onroad + Force Offroad: keep started until stock CC ENABLED or timeout."""
  assert allows_onroad(True, already_started=True, handoff_ready=False) is True
  assert allows_onroad(True, already_started=True, handoff_ready=True) is False
  assert allows_onroad(True, already_started=True, handoff_ready=False, timed_out=True) is False
  # Parked / not started: do not wait — stay offroad.
  assert allows_onroad(True, already_started=False, handoff_ready=False) is False
  assert handoff_wait_timed_out(None, 10.0) is False
  assert handoff_wait_timed_out(0.0, 2.9) is False
  assert handoff_wait_timed_out(0.0, 3.0) is True


def test_onroad_confirm_holds_started_until_yes():
  assert needs_driver_confirm(True, False, True) is True
  assert needs_driver_confirm(True, True, True) is False
  assert needs_driver_confirm(True, False, False) is False
  assert needs_driver_confirm(False, False, True) is False
  # Yes not tapped: stay onroad even if handoff_ready leaked.
  assert allows_onroad(True, already_started=True, confirmed=False, handoff_ready=True) is True
  assert allows_onroad(True, already_started=True, confirmed=True, handoff_ready=True) is False


def test_apply_toggle_and_no_clears_intent():
  class P:
    def __init__(self):
      self.d = {}
    def put_bool(self, k, v):
      self.d[k] = bool(v)
    def get_bool(self, k):
      return bool(self.d.get(k, False))

  p = P()
  apply_force_offroad_toggle(p, True, started=True)
  assert p.get_bool(PARAM) is True
  assert p.get_bool(CONFIRMED_PARAM) is False
  confirm_force_offroad(p)
  assert p.get_bool(CONFIRMED_PARAM) is True
  cancel_force_offroad(p)
  assert p.get_bool(PARAM) is False
  assert p.get_bool(CONFIRMED_PARAM) is False
  assert p.get_bool(HANDOFF_READY_PARAM) is False

  apply_force_offroad_toggle(p, True, started=False)
  assert p.get_bool(CONFIRMED_PARAM) is True


def test_force_offroad_blocks_should_start_while_ignition_on():
  onroad = {
    "ignition": True,
    "not_onroad_cycle": True,
    "device_temp_good": True,
    "not_force_offroad": allows_onroad(True),
  }
  startup = {"up_to_date": True, "accepted_terms": True}
  assert should_start_now(onroad, startup, already_started=True) is False
  assert should_start_now(onroad, startup, already_started=False) is False

  onroad["not_force_offroad"] = allows_onroad(False)
  assert should_start_now(onroad, startup, already_started=True) is True
  assert should_start_now(onroad, startup, already_started=False) is True

  # Missing a startup condition still blocks a cold start, not a stay-onroad.
  startup["up_to_date"] = False
  assert should_start_now(onroad, startup, already_started=True) is True
  assert should_start_now(onroad, startup, already_started=False) is False


def test_hardwared_hooks_param_before_ign_edge():
  text = (ROOT / "system" / "hardware" / "hardwared.py").read_text(encoding="utf-8")
  assert "not_force_offroad" in text
  assert "nap_force_offroad_allows_onroad" in text
  assert "should_start_now" in text
  assert "NAP_FORCE_OFFROAD_PARAM" in text
  assert "NAP_FORCE_OFFROAD_HANDOFF_READY_PARAM" in text
  assert "NAP_FORCE_OFFROAD_CONFIRMED_PARAM" in text
  assert "handoff_ready" in text
  assert "force_offroad_confirmed" in text
  cond = text.index('onroad_conditions["not_force_offroad"]')
  edge = text.index("ign_edge =")
  assert cond < edge


def test_settings_and_docs_cover_warning_and_reset():
  nap = (ROOT / "selfdrive" / "ui" / "layouts" / "settings" / "nap.py").read_text(encoding="utf-8")
  mici = (ROOT / "selfdrive" / "ui" / "mici" / "layouts" / "settings" / "nap.py").read_text(encoding="utf-8")
  popup = (ROOT / "selfdrive" / "ui" / "layouts" / "settings" / "hidden_toggles.py").read_text(encoding="utf-8")
  overlay = (ROOT / "selfdrive" / "ui" / "mici" / "layouts" / "settings" / "hidden_toggles.py").read_text(encoding="utf-8")
  content = (ROOT / "selfdrive" / "ui" / "layouts" / "settings" / "nap_content.py").read_text(encoding="utf-8")
  docs = (ROOT / "docs-nap" / "force-offroad.md").read_text(encoding="utf-8")
  readme = (ROOT / "docs-nap" / "README.md").read_text(encoding="utf-8")
  ui_state = (ROOT / "selfdrive" / "ui" / "ui_state.py").read_text(encoding="utf-8")

  assert "NAP_FORCE_OFFROAD" in nap
  assert "put_bool(NAP_FORCE_OFFROAD, False)" in nap
  assert 'self._add_toggle(\n      NAP_FORCE_OFFROAD' not in nap

  assert "Force Offroad" in popup
  assert "NAP_FORCE_OFFROAD" in popup
  assert "is_offroad" not in popup

  assert "NAP_FORCE_OFFROAD" not in mici
  assert "force offroad" in overlay
  assert "NAP_FORCE_OFFROAD" in overlay
  assert "drive manually" in overlay
  assert "set_enabled(ui_state.is_offroad)" not in overlay

  assert "WARNING" in content
  assert "Drive manually" in content
  assert "NAPForceOffroad" in docs
  assert "NAPForceOffroadHandoffReady" in docs
  assert "NAPForceOffroadConfirmed" in docs
  assert "Ready to resume steering control?" in docs
  assert "CLEAR_ON_IGNITION_ON" in docs
  assert "deviceState.started" in docs
  assert "triple-tap" in docs
  assert "STANDBY" in docs
  assert "enableLongControl" in docs
  assert "force-offroad.md" in readme
  assert "NAP Force Offroad" in ui_state


def test_params_roundtrip_if_compiled():
  """Compiled Params picks up params_keys.h. Skip when params_pyx is not built."""
  import pytest
  params_mod = pytest.importorskip("openpilot.common.params")
  Params = params_mod.Params
  ParamKeyFlag = params_mod.ParamKeyFlag
  params = Params()
  assert params.get_bool(PARAM) is False
  params.put_bool(PARAM, True, block=True)
  assert params.get_bool(PARAM) is True
  params.clear_all(ParamKeyFlag.CLEAR_ON_IGNITION_ON)
  assert params.get_bool(PARAM) is False
  params.put_bool(PARAM, True, block=True)
  params.clear_all(ParamKeyFlag.CLEAR_ON_MANAGER_START)
  assert params.get_bool(PARAM) is False
  params.put_bool(PARAM, True, block=True)
  params.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)
  assert params.get_bool(PARAM) is True
