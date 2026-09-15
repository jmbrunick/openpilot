"""Triple-tap detector + hidden-NAP-toggle placement (no GUI)."""
from pathlib import Path

from openpilot.selfdrive.ui.layouts.settings.triple_tap import (
  MICI_NAP_OPEN_DELAY_S,
  TAP_COUNT,
  TripleTapDetector,
  WINDOW_S,
)

ROOT = Path(__file__).resolve().parents[3]


def _code_without_comments(src: str) -> str:
  """Drop '# ...' tails so source-string checks ignore comments."""
  return "\n".join(line.split("#", 1)[0] for line in src.splitlines())


def test_window_and_count_constants():
  assert WINDOW_S == 1.0
  assert TAP_COUNT == 3
  assert MICI_NAP_OPEN_DELAY_S == 0.40
  assert MICI_NAP_OPEN_DELAY_S < WINDOW_S


def test_three_taps_inside_window_fire():
  d = TripleTapDetector()
  assert d.tap(0.00) is False
  assert d.pending_count == 1
  assert d.tap(0.40) is False
  assert d.tap(0.90) is True
  assert d.pending_count == 0


def test_three_taps_exactly_at_window_edge_fire():
  d = TripleTapDetector()
  assert d.tap(1.00) is False
  assert d.tap(1.50) is False
  assert d.tap(2.00) is True


def test_slow_taps_do_not_fire():
  d = TripleTapDetector()
  assert d.tap(0.00) is False
  assert d.tap(0.60) is False
  assert d.tap(1.61) is False
  assert d.pending_count == 1  # only the last tap remains


def test_sliding_window_keeps_recent_pair():
  d = TripleTapDetector()
  assert d.tap(0.00) is False
  assert d.tap(0.70) is False
  # 0.00 drops out; [0.70, 1.50] is only two taps
  assert d.tap(1.50) is False
  assert d.pending_count == 2
  assert d.tap(1.60) is True


def test_reset_clears_burst():
  d = TripleTapDetector()
  d.tap(0.00)
  d.tap(0.20)
  d.reset()
  assert d.pending_count == 0
  assert d.tap(0.30) is False


def test_hidden_toggles_removed_from_normal_lists():
  tici_dm = (ROOT / "selfdrive/ui/layouts/settings/driving_mannerisms.py").read_text()
  mici_dm = (ROOT / "selfdrive/ui/mici/layouts/settings/driving_mannerisms.py").read_text()
  nap = (ROOT / "selfdrive/ui/layouts/settings/nap.py").read_text()
  nap_mici = (ROOT / "selfdrive/ui/mici/layouts/settings/nap.py").read_text()
  popup = (ROOT / "selfdrive/ui/layouts/settings/hidden_toggles.py").read_text()
  overlay = (ROOT / "selfdrive/ui/mici/layouts/settings/hidden_toggles.py").read_text()
  settings = (ROOT / "selfdrive/ui/layouts/settings/settings.py").read_text()
  settings_mici = (ROOT / "selfdrive/ui/mici/layouts/settings/settings.py").read_text()

  assert "Simulate Look" not in tici_dm
  assert "False Alert Ignore" not in tici_dm
  assert "NAP_DM_SIMULATE_LOOKING" not in tici_dm
  assert "NAP_DM_FALSE_ALERT_IGNORE" not in tici_dm
  assert "simulate look" not in mici_dm
  assert "false alert ignore" not in mici_dm
  assert "NAP_DM_SIMULATE_LOOKING" not in mici_dm
  assert "NAP_DM_FALSE_ALERT_IGNORE" not in mici_dm

  assert 'self._add_toggle(\n      NAP_FORCE_OFFROAD' not in nap
  assert "Go Offline" not in nap
  assert "simulate look" not in nap
  assert "false alert ignore" not in nap
  assert 'put_bool(NAP_FORCE_OFFROAD, False)' in nap
  assert "NAPForceOffroadConfirmed" in nap
  assert 'put_bool(NAP_DM_SIMULATE_LOOKING, True)' in nap
  assert 'put_bool(NAP_DM_FALSE_ALERT_IGNORE, False)' in nap

  assert 'BigParamControl("force offroad"' not in nap_mici
  assert "NAP_FORCE_OFFROAD" not in nap_mici

  assert "Simulate Look" in popup
  assert "False Alert Ignore" in popup
  assert "Force Offroad" in popup
  assert "[self._offroad_item, self._dm_item, self._fai_item]" in popup
  assert "NAP_DM_SIMULATE_LOOKING" in popup
  assert "NAP_DM_FALSE_ALERT_IGNORE" in popup
  assert "NAP_FORCE_OFFROAD" in popup
  assert "apply_force_offroad_toggle" in popup
  assert "apply_dm_simulate_looking" in popup
  assert "apply_dm_false_alert_ignore" in popup
  assert "read_exclusive_dm_toggles" in popup
  # Live switch: paint sibling from apply() return, not a stale get_bool.
  on_sim = _code_without_comments(
    popup[popup.index("def _on_dm_sim_looking"):popup.index("def _on_false_alert_ignore")]
  )
  on_fai = _code_without_comments(
    popup[popup.index("def _on_false_alert_ignore"):popup.index("def _on_force_offroad")]
  )
  assert "apply_dm_simulate_looking(self._params, bool(state))" in on_sim
  assert "set_state(fai)" in on_sim
  assert "get_bool(" not in on_sim
  assert "apply_dm_false_alert_ignore(self._params, bool(state))" in on_fai
  assert "set_state(sim)" in on_fai
  assert "get_bool(" not in on_fai
  assert "set_enabled(ui_state.is_offroad)" not in popup
  assert popup.index('"Force Offroad"') < popup.index('"Simulate Look"')
  assert popup.index('"Simulate Look"') < popup.index('"False Alert Ignore"')

  assert "simulate look" in overlay
  assert "false alert ignore" in overlay
  assert "force offroad" in overlay
  assert overlay.index('BigParamControl("force offroad"') < overlay.index('BigParamControl("simulate look"')
  assert overlay.index('BigParamControl("simulate look"') < overlay.index('BigParamControl("false alert ignore"')
  assert "NAP_DM_SIMULATE_LOOKING" in overlay
  assert "NAP_DM_FALSE_ALERT_IGNORE" in overlay
  assert "NAP_FORCE_OFFROAD" in overlay
  assert "apply_dm_simulate_looking" in overlay
  assert "apply_dm_false_alert_ignore" in overlay
  assert "read_exclusive_dm_toggles" in overlay
  on_sim_m = _code_without_comments(
    overlay[overlay.index("def _on_simulate_look"):overlay.index("def _on_false_alert_ignore")]
  )
  on_fai_m = _code_without_comments(
    overlay[overlay.index("def _on_false_alert_ignore"):overlay.index("def show_event")]
  )
  assert "apply_dm_simulate_looking(self._params, bool(state))" in on_sim_m
  assert "set_checked(fai)" in on_sim_m
  assert "refresh()" not in on_sim_m
  assert "get_bool(" not in on_sim_m
  assert "apply_dm_false_alert_ignore(self._params, bool(state))" in on_fai_m
  assert "set_checked(sim)" in on_fai_m
  assert "refresh()" not in on_fai_m
  assert "get_bool(" not in on_fai_m
  assert "set_enabled(ui_state.is_offroad)" not in overlay

  assert "TripleTapDetector" in settings
  assert "_on_nap_sidebar_tap" in settings
  assert "_handle_mouse_release" in settings
  assert "TripleTapDetector" in settings_mici
  assert "_on_nap_clicked" in settings_mici
  assert "HiddenTogglesOverlayMici" in settings_mici

  # Soft-lat / One-Pedal Long / Hypermile / Hill Climb / Acceleration stay on the normal mannerisms list.
  assert "Soft Lateral Handoff" in tici_dm
  assert "One-Pedal Long" in tici_dm
  assert "Hill Climb" in tici_dm
  assert "Hypermile" in tici_dm
  assert "Acceleration" in tici_dm
  assert "NAPMapSpeedAccel" in tici_dm
  assert "hill climb" in mici_dm
  assert '"acceleration"' in mici_dm
  assert "NAPMapSpeedAccel" in mici_dm
