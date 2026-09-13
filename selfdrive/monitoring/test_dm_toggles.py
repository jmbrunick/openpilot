"""Simulate Look / False Alert Ignore mutual exclusion (no GUI / no cereal)."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

_spec = importlib.util.spec_from_file_location(
  "dm_toggles", ROOT / "selfdrive" / "monitoring" / "dm_toggles.py",
)
_mod = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
_spec.loader.exec_module(_mod)

PARAM_DM_SIMULATE_LOOKING = _mod.PARAM_DM_SIMULATE_LOOKING
PARAM_DM_FALSE_ALERT_IGNORE = _mod.PARAM_DM_FALSE_ALERT_IGNORE
DEFAULT_SIMULATE_LOOKING = _mod.DEFAULT_SIMULATE_LOOKING
DEFAULT_FALSE_ALERT_IGNORE = _mod.DEFAULT_FALSE_ALERT_IGNORE
exclusive_dm_toggle_states = _mod.exclusive_dm_toggle_states
apply_dm_simulate_looking = _mod.apply_dm_simulate_looking
apply_dm_false_alert_ignore = _mod.apply_dm_false_alert_ignore
read_exclusive_dm_toggles = _mod.read_exclusive_dm_toggles


class FakeParams:
  def __init__(self, **bools):
    self.d = dict(bools)

  def get_bool(self, name):
    if name not in self.d:
      raise KeyError(name)
    return bool(self.d[name])

  def put_bool(self, name, value):
    self.d[name] = bool(value)


def test_param_names_and_nap_release_defaults():
  keys = (ROOT / "common" / "params_keys.h").read_text(encoding="utf-8")
  assert PARAM_DM_SIMULATE_LOOKING == "NAPDmSimulateLooking"
  assert PARAM_DM_FALSE_ALERT_IGNORE == "NAPDmFalseAlertIgnore"
  assert DEFAULT_SIMULATE_LOOKING is False
  assert DEFAULT_FALSE_ALERT_IGNORE is False
  sim_line = next(ln for ln in keys.splitlines() if f'"{PARAM_DM_SIMULATE_LOOKING}"' in ln)
  fai_line = next(ln for ln in keys.splitlines() if f'"{PARAM_DM_FALSE_ALERT_IGNORE}"' in ln)
  assert 'BOOL, "0"' in sim_line
  assert 'BOOL, "0"' in fai_line
  assert "PERSISTENT" in sim_line and "PERSISTENT" in fai_line


def test_exclusive_states_both_on_prefers_simulate_look():
  assert exclusive_dm_toggle_states(True, True) == (True, False)
  assert exclusive_dm_toggle_states(True, False) == (True, False)
  assert exclusive_dm_toggle_states(False, True) == (False, True)
  assert exclusive_dm_toggle_states(False, False) == (False, False)


def test_apply_simulate_look_on_clears_fai():
  p = FakeParams(**{PARAM_DM_SIMULATE_LOOKING: False, PARAM_DM_FALSE_ALERT_IGNORE: True})
  sim, fai = apply_dm_simulate_looking(p, True)
  assert (sim, fai) == (True, False)
  assert p.get_bool(PARAM_DM_SIMULATE_LOOKING) is True
  assert p.get_bool(PARAM_DM_FALSE_ALERT_IGNORE) is False


def test_apply_fai_on_clears_simulate_look():
  p = FakeParams(**{PARAM_DM_SIMULATE_LOOKING: True, PARAM_DM_FALSE_ALERT_IGNORE: False})
  sim, fai = apply_dm_false_alert_ignore(p, True)
  assert (sim, fai) == (False, True)
  assert p.get_bool(PARAM_DM_SIMULATE_LOOKING) is False
  assert p.get_bool(PARAM_DM_FALSE_ALERT_IGNORE) is True


def test_apply_off_leaves_the_other_alone():
  p = FakeParams(**{PARAM_DM_SIMULATE_LOOKING: True, PARAM_DM_FALSE_ALERT_IGNORE: False})
  sim, fai = apply_dm_simulate_looking(p, False)
  assert (sim, fai) == (False, False)
  assert p.get_bool(PARAM_DM_SIMULATE_LOOKING) is False
  assert p.get_bool(PARAM_DM_FALSE_ALERT_IGNORE) is False

  p = FakeParams(**{PARAM_DM_SIMULATE_LOOKING: False, PARAM_DM_FALSE_ALERT_IGNORE: True})
  sim, fai = apply_dm_false_alert_ignore(p, False)
  assert (sim, fai) == (False, False)
  assert p.get_bool(PARAM_DM_FALSE_ALERT_IGNORE) is False
  assert p.get_bool(PARAM_DM_SIMULATE_LOOKING) is False


def test_read_resolves_stale_both_on_and_persists():
  p = FakeParams(**{PARAM_DM_SIMULATE_LOOKING: True, PARAM_DM_FALSE_ALERT_IGNORE: True})
  sim, fai = read_exclusive_dm_toggles(p)
  assert (sim, fai) == (True, False)
  assert p.get_bool(PARAM_DM_SIMULATE_LOOKING) is True
  assert p.get_bool(PARAM_DM_FALSE_ALERT_IGNORE) is False


def test_read_without_persist_does_not_write():
  p = FakeParams(**{PARAM_DM_SIMULATE_LOOKING: True, PARAM_DM_FALSE_ALERT_IGNORE: True})
  sim, fai = read_exclusive_dm_toggles(p, persist=False)
  assert (sim, fai) == (True, False)
  assert p.get_bool(PARAM_DM_FALSE_ALERT_IGNORE) is True


def test_read_both_off_stays_off():
  p = FakeParams(**{PARAM_DM_SIMULATE_LOOKING: False, PARAM_DM_FALSE_ALERT_IGNORE: False})
  assert read_exclusive_dm_toggles(p) == (False, False)
