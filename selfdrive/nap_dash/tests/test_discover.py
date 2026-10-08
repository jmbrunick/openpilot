"""Toggle auto-discovery reads the device UI source. It does not import it."""
from __future__ import annotations

from pathlib import Path

from openpilot.selfdrive.nap_dash.discover import discover_manifest

ROOT = Path(__file__).resolve().parents[3]
UI = ROOT / "selfdrive" / "ui" / "layouts" / "settings"


def _params(manifest):
  return [ctrl["param"] for ctrl in manifest["controls"] if ctrl.get("param")]


def _by_param(manifest, param):
  for ctrl in manifest["controls"]:
    if ctrl.get("param") == param:
      return ctrl
  raise AssertionError(param)


def test_stock_toggles_come_from_device_source():
  manifest = discover_manifest(ROOT)
  enable = _by_param(manifest, "OpenpilotEnabledToggle")
  assert enable["title"] == "Enable openpilot"
  assert enable["panel"] == "toggles"
  assert enable["lock_engaged"] is True
  assert enable["needs_restart"] is True
  assert "attention is required" in enable["description"]

  experimental = _by_param(manifest, "ExperimentalMode")
  assert experimental["confirm"] is True
  assert experimental["writer"] == "experimental"
  assert "End-to-End" in experimental["description"] or "end-to-end" in experimental["description"].lower() or "chill mode" in experimental["description"]

  personality = _by_param(manifest, "LongitudinalPersonality")
  assert personality["kind"] == "choice"
  assert personality["choices"] == ["Aggressive", "Standard", "Relaxed"]
  assert personality["values"] == [0, 1, 2]

  # Inserted after Disengage, matching the device list.
  titles = [ctrl["title"] for ctrl in manifest["controls"] if ctrl["panel"] == "toggles"]
  assert titles.index("Disengage on Accelerator Pedal") + 1 == titles.index("Driving Personality")


def test_mannerisms_and_map_speed_are_discovered_not_hand_listed():
  manifest = discover_manifest(ROOT)
  handoff = _by_param(manifest, "NAPDriverLatHandoff")
  assert handoff["panel"] == "nap_mannerisms"
  assert handoff["title"] == "Soft Lateral Handoff"
  hyper = _by_param(manifest, "NAPHypermile")
  assert hyper["writer"] == "hypermile"
  mode = _by_param(manifest, "NAPMapSpeedMode")
  assert mode["panel"] == "nap_map"
  assert mode["choices"] == ["Off", "Display", "Cap", "Follow"]
  assert mode["values"] == [0, 1, 2, 3]
  sign = _by_param(manifest, "NAPSpeedSignLog")
  assert sign["writer"] == "bool"
  assert sign["panel"] == "nap"
  panels = [panel["title"] for panel in manifest["panels"]]
  assert panels[:5] == ["Device", "Network", "Toggles", "Software", "NAP"]
  assert "Driving Mannerisms" in panels
  assert "Map Speed Limit" in panels


def test_new_toggle_in_device_ui_source_shows_up_in_manifest():
  """Adding a toggle to the device UI source is enough. No web-side list edit."""
  mannerisms = (UI / "driving_mannerisms.py").read_text(encoding="utf-8")
  needle = "  def _on_hypermile"
  assert needle in mannerisms
  snippet = """\
    self._probe = toggle_item(
      "Discovery Probe",
      description="Added in the device UI source.",
      initial_state=self._params.get_bool("NAPDiscoveryProbe"),
      callback=self._on_probe,
    )
    self._all_items.append(self._probe)

"""
  patched = mannerisms.replace(needle, snippet + needle, 1)
  toggles = (UI / "toggles.py").read_text(encoding="utf-8")
  toggles_needle = '      "IsMetric": ('
  assert toggles_needle in toggles
  toggles_snippet = """\
      "NAPWebDiscoveryProbe": (
        lambda: tr("Web Discovery Probe"),
        "Shown when the device UI defines it.",
        "metric.png",
        False,
      ),
"""
  toggles_patched = toggles.replace(toggles_needle, toggles_snippet + toggles_needle, 1)
  manifest = discover_manifest(ROOT, sources={
    "driving_mannerisms.py": patched,
    "toggles.py": toggles_patched,
  })
  probe = _by_param(manifest, "NAPDiscoveryProbe")
  assert probe["title"] == "Discovery Probe"
  assert probe["panel"] == "nap_mannerisms"
  assert probe["description"] == "Added in the device UI source."
  stock = _by_param(manifest, "NAPWebDiscoveryProbe")
  assert stock["title"] == "Web Discovery Probe"
  assert stock["panel"] == "toggles"
  assert stock["description"] == "Shown when the device UI defines it."

  live = discover_manifest(ROOT)
  assert "NAPDiscoveryProbe" not in _params(live)
  assert "NAPWebDiscoveryProbe" not in _params(live)


def test_nap_param_keys_resolve_when_opendbc_source_is_present(tmp_path):
  package = tmp_path / "opendbc" / "car" / "tesla" / "preap"
  package.mkdir(parents=True)
  (package / "nap_params.py").write_text(
    '''\
class NAPParamKeys:
  PEDAL_ENABLED = "NAPPedalEnabled"
  ADAPTIVE_ACCEL = "NAPAdaptiveAccel"
  FOLLOW_DISTANCE_CITY = "NAPFollowDistanceCity"
  FOLLOW_DISTANCE_HWY = "NAPFollowDistanceHwy"
  PEDAL_CAN_BUS = "NAPPedalCanBus"
  PEDAL_CALIB_DONE = "NAPPedalCalibDone"
  IBOOSTER_ENABLED = "NAPiBoosterEnabled"
  BRAKE_FACTOR = "NAPBrakeFactor"
  FORCE_PRE_AP = "NAPForcePreAP"
  RADAR_ENABLED = "NAPRadarEnabled"
  RADAR_HUD = "NAPRadarHud"
  RADAR_IGNORE_HW_FAIL = "NAPRadarIgnoreHwFail"
  RADAR_EPAS_TYPE = "NAPRadarEpasType"
  RADAR_POSITION = "NAPRadarPosition"
''',
    encoding="utf-8",
  )
  manifest = discover_manifest(ROOT, extra_module_dirs=[tmp_path])
  pedal = _by_param(manifest, "NAPPedalEnabled")
  assert pedal["title"] == "Pedal Interceptor"
  assert pedal["lock_onroad"] is True
  assert pedal["reboot_hint"] is True
  city = _by_param(manifest, "NAPFollowDistanceCity")
  assert city["values"] == [1, 2, 3, 4, 5, 6, 7]


def test_discovery_does_not_import_ui_or_cereal():
  src = (ROOT / "selfdrive" / "nap_dash" / "discover.py").read_text(encoding="utf-8")
  assert "import cereal" not in src
  assert "SubMaster" not in src
  assert "import messaging" not in src
  assert "import openpilot.selfdrive.ui" not in src
  assert "import openpilot.selfdrive.mapd" not in src
  assert "import openpilot.selfdrive.speedsignd" not in src


def test_hidden_and_developer_rules():
  manifest = discover_manifest(ROOT)
  sim = _by_param(manifest, "NAPDmSimulateLooking")
  assert sim["writer"] == "dm_sim"
  assert sim["panel"] == "nap_hidden"
  fai = _by_param(manifest, "NAPDmFalseAlertIgnore")
  assert fai["writer"] == "dm_fai"
  offroad = _by_param(manifest, "NAPForceOffroad")
  assert offroad["writer"] == "force_offroad"
  assert offroad["lock_onroad"] is False
  adb = _by_param(manifest, "AdbEnabled")
  assert adb["lock_onroad"] is True
  alpha = _by_param(manifest, "AlphaLongitudinalEnabled")
  assert alpha["confirm"] is True
  assert alpha["needs_restart"] is True
  assert alpha["release_hidden"] is True
  joy = _by_param(manifest, "JoystickDebugMode")
  assert "LongitudinalManeuverMode" in joy["also_clear"]
  assert joy["lock_onroad"] is True
