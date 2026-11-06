"""Comma four NAP UI keeps the 3X controls that exist on this branch."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def test_comma_four_has_the_3x_lateral_controls():
  mici = (ROOT / "selfdrive/ui/mici/layouts/settings/driving_mannerisms.py").read_text()
  tici = (ROOT / "selfdrive/ui/layouts/settings/driving_mannerisms.py").read_text()
  for key in ("NAP_LAT_TURN_GEOM", "NAP_LAT_REF_OFFSET", "NAP_ROUNDABOUT_ASSIST"):
    assert key in tici and key in mici
  widgets = mici.split("self._scroller.add_widgets", 1)[1]
  assert widgets.index("self._turn_in_delay") < widgets.index("turn_geom")
  assert widgets.index("turn_geom") < widgets.index("self._ref_offset")
  assert widgets.index("self._ref_offset") < widgets.index("roundabout")
  assert widgets.index("roundabout") < widgets.index("lat_handoff")


def test_comma_four_nap_page_has_reset_and_emergency():
  nap = (ROOT / "selfdrive/ui/mici/layouts/settings/nap.py").read_text()
  for text in ("emergency disable", "reset to defaults", "apply_nap_factory_defaults"):
    assert text in nap
  widgets = nap.split("self._scroller.add_widgets", 1)[1]
  assert widgets.index("restore_epas_btn") < widgets.index("emergency_disable_btn")
  assert widgets.index("emergency_disable_btn") < widgets.index("reset_defaults_btn")


def test_comma_four_limit_chip_stays_up():
  hud = (ROOT / "selfdrive/ui/mici/onroad/hud_renderer.py").read_text()
  assert "gap_lock_hud_chip" in hud
  assert 'f"OSM {lim}"' not in hud
  assert "def _draw_map_limit" in hud
  set_speed = hud.split("def _draw_set_speed", 1)[1].split("def _draw_map_limit", 1)[0]
  assert "map_speed_limit" not in set_speed
