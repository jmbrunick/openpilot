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
allows_onroad = _mod.allows_onroad
should_start_now = _mod.should_start_now


def test_param_default_off_and_clears_on_ignition_and_manager():
  keys = (ROOT / "common" / "params_keys.h").read_text(encoding="utf-8")
  assert PARAM == "NAPForceOffroad"
  line = next(ln for ln in keys.splitlines() if '"NAPForceOffroad"' in ln)
  assert "CLEAR_ON_MANAGER_START" in line
  assert "CLEAR_ON_IGNITION_ON" in line
  assert "PERSISTENT" not in line
  # Going offroad is what the toggle does — that flag would undo it.
  assert "CLEAR_ON_OFFROAD" not in line
  assert 'BOOL, "0"' in line


def test_allows_onroad_default_and_toggle():
  assert allows_onroad(False) is True
  assert allows_onroad(True) is False


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
  cond = text.index('onroad_conditions["not_force_offroad"]')
  edge = text.index("ign_edge =")
  assert cond < edge


def test_settings_and_docs_cover_warning_and_reset():
  nap = (ROOT / "selfdrive" / "ui" / "layouts" / "settings" / "nap.py").read_text(encoding="utf-8")
  mici = (ROOT / "selfdrive" / "ui" / "mici" / "layouts" / "settings" / "nap.py").read_text(encoding="utf-8")
  content = (ROOT / "selfdrive" / "ui" / "layouts" / "settings" / "nap_content.py").read_text(encoding="utf-8")
  docs = (ROOT / "docs-nap" / "force-offroad.md").read_text(encoding="utf-8")
  readme = (ROOT / "docs-nap" / "README.md").read_text(encoding="utf-8")
  ui_state = (ROOT / "selfdrive" / "ui" / "ui_state.py").read_text(encoding="utf-8")

  assert "Force Offroad" in nap
  assert "NAP_FORCE_OFFROAD" in nap
  assert "put_bool(NAP_FORCE_OFFROAD, False)" in nap
  toggle_block = nap.split('self._add_toggle(\n      NAP_FORCE_OFFROAD')[1].split("NAPParamKeys.PEDAL_ENABLED")[0]
  assert "is_offroad" not in toggle_block

  assert "force offroad" in mici
  assert "NAP_FORCE_OFFROAD" in mici
  assert "drive manually" in mici
  assert "set_enabled(ui_state.is_offroad)" not in mici.split("force_offroad")[1].split("pedal_enabled")[0]

  assert "WARNING" in content
  assert "Drive manually" in content
  assert "NAPForceOffroad" in docs
  assert "CLEAR_ON_IGNITION_ON" in docs
  assert "deviceState.started" in docs
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
