"""Justin NAP Params / Driving Mannerisms bridge for the companion Dash.

Writes go only through Params (and Hypermile / SL-FAI helpers). This module
is the settings API — no /data/nap_settings.json, no cruise-trim injector,
and no Philip-only City Turns / Tap LC / Corner Assist / Lane Centering.
"""
from __future__ import annotations

from typing import Any

from openpilot.selfdrive.controls.lib.hypermile import apply_hypermile_toggle
from openpilot.selfdrive.monitoring.dm_toggles import (
  apply_dm_false_alert_ignore,
  apply_dm_simulate_looking,
  read_exclusive_dm_toggles,
)
from openpilot.selfdrive.ui.layouts.settings.nap_content import (
  MAP_SPEED_ACCEL,
  MAP_SPEED_ACCEL_DEFAULT,
  MAP_SPEED_LOOKAHEAD,
  MAP_SPEED_MODES,
  MAP_SPEED_OFFSETS_MPH,
  NAP_DRIVER_LAT_HANDOFF,
  NAP_HYPERMILE,
  NAP_HYPERMILE_HILL_CLIMB,
  NAP_HYPERMILE_STEP_DOWN,
  NAP_ONE_PEDAL_LONG,
)

# API name -> registered Params key. Keep in sync with Settings → NAP.
PARAM_ADAPTIVE_ACCEL = "NAPAdaptiveAccel"
PARAM_FOLLOW_DISTANCE = "NAPFollowDistance"
PARAM_ACCEL = "NAPMapSpeedAccel"
PARAM_MAP_MODE = "NAPMapSpeedMode"
PARAM_MAP_OFFSET = "NAPMapSpeedOffsetMph"
PARAM_MAP_LOOKAHEAD = "NAPMapSpeedLookahead"
PARAM_PERSONALITY = "LongitudinalPersonality"
PARAM_EXPERIMENTAL = "ExperimentalMode"
PARAM_EXPERIMENTAL_CONFIRMED = "ExperimentalModeConfirmed"
PARAM_SL = "NAPDmSimulateLooking"
PARAM_FAI = "NAPDmFalseAlertIgnore"

# Philip-only / rejected names. Do not alias these to Justin params.
REJECTED_SETTING_NAMES = frozenset({
  "speed_trim",
  "speed_offset",
  "napspeedtrim",
  "napspeedoffset",
  "city_turns",
  "cityturns",
  "tap_lc",
  "taplc",
  "corner_assist",
  "cornerassist",
  "lane_centering",
  "lanecentering",
  "nap_settings",
})

PERSONALITIES = {0: "aggressive", 1: "standard", 2: "chill"}

# Driving Mannerisms + Map Speed + stock UI + optional SL/FAI.
_INT_SPECS: dict[str, dict[str, Any]] = {
  "accel": {"param": PARAM_ACCEL, "allowed": set(MAP_SPEED_ACCEL), "default": MAP_SPEED_ACCEL_DEFAULT},
  "follow_distance": {"param": PARAM_FOLLOW_DISTANCE, "min": 1, "max": 7, "default": 4},
  "map_speed_mode": {"param": PARAM_MAP_MODE, "allowed": set(MAP_SPEED_MODES), "default": 0},
  "map_speed_offset_mph": {"param": PARAM_MAP_OFFSET, "allowed": set(MAP_SPEED_OFFSETS_MPH), "default": 0},
  "map_speed_lookahead": {"param": PARAM_MAP_LOOKAHEAD, "allowed": set(MAP_SPEED_LOOKAHEAD), "default": 2},
  "personality": {"param": PARAM_PERSONALITY, "min": 0, "max": 2, "default": 1},
}

_BOOL_SPECS: dict[str, dict[str, Any]] = {
  "adaptive_accel": {"param": PARAM_ADAPTIVE_ACCEL, "default": True},
  "driver_lat_handoff": {"param": NAP_DRIVER_LAT_HANDOFF, "default": True},
  "one_pedal_long": {"param": NAP_ONE_PEDAL_LONG, "default": False},
  "hypermile_step_down": {"param": NAP_HYPERMILE_STEP_DOWN, "default": False},
  "hypermile_hill_climb": {"param": NAP_HYPERMILE_HILL_CLIMB, "default": True},
  "experimental": {"param": PARAM_EXPERIMENTAL, "default": False},
}

_SPECIAL = {
  "hypermile": NAP_HYPERMILE,
  "sl": PARAM_SL,
  "fai": PARAM_FAI,
}

_PARAM_TO_API = {spec["param"]: name for name, spec in {**_INT_SPECS, **_BOOL_SPECS}.items()}
_PARAM_TO_API.update({param: name for name, param in _SPECIAL.items()})


class SettingError(ValueError):
  """Invalid Dash setting name or value."""


def _norm_name(name: str) -> str:
  return str(name or "").strip()


def _api_name(name: str) -> str:
  key = _norm_name(name)
  if not key:
    raise SettingError("missing setting name")
  folded = key.lower().replace("-", "_")
  if folded in REJECTED_SETTING_NAMES or key in REJECTED_SETTING_NAMES:
    raise SettingError(f"rejected Philip-only setting: {key}")
  if key in _INT_SPECS or key in _BOOL_SPECS or key in _SPECIAL:
    return key
  if key in _PARAM_TO_API:
    return _PARAM_TO_API[key]
  raise SettingError(f"unknown setting: {key}")


def _get_int(params, key: str, default: int) -> int:
  try:
    raw = params.get(key, return_default=True)
    if raw is None or raw == "":
      return default
    if isinstance(raw, bytes):
      raw = raw.decode("utf-8", errors="ignore")
    return int(raw)
  except Exception:
    return default


def _get_bool(params, key: str, default: bool = False) -> bool:
  try:
    return bool(params.get_bool(key))
  except Exception:
    return default


def _as_bool(value: Any) -> bool:
  if isinstance(value, bool):
    return value
  if isinstance(value, (int, float)) and value in (0, 1):
    return bool(value)
  if isinstance(value, str):
    folded = value.strip().lower()
    if folded in ("1", "true", "yes", "on"):
      return True
    if folded in ("0", "false", "no", "off", ""):
      return False
  raise SettingError(f"invalid bool: {value!r}")


def _clamp_int(name: str, value: Any) -> int:
  spec = _INT_SPECS[name]
  try:
    parsed = int(value)
  except (TypeError, ValueError) as exc:
    raise SettingError(f"invalid int for {name}") from exc
  allowed = spec.get("allowed")
  if allowed is not None and parsed not in allowed:
    raise SettingError(f"{name} must be one of {sorted(allowed)}")
  lo, hi = spec.get("min"), spec.get("max")
  if lo is not None and parsed < lo:
    raise SettingError(f"{name} must be >= {lo}")
  if hi is not None and parsed > hi:
    raise SettingError(f"{name} must be <= {hi}")
  return parsed


def read_settings(params) -> dict[str, Any]:
  """Read the Dash-exposed Justin params. Safe with missing/test doubles."""
  out: dict[str, Any] = {}
  for name, spec in _INT_SPECS.items():
    out[name] = _get_int(params, spec["param"], spec["default"])
  for name, spec in _BOOL_SPECS.items():
    out[name] = _get_bool(params, spec["param"], spec["default"])
  out["hypermile"] = _get_bool(params, NAP_HYPERMILE, False)
  sl, fai = read_exclusive_dm_toggles(params, persist=False)
  out["sl"] = sl
  out["fai"] = fai
  out["personality_raw"] = out["personality"]
  out["personality_name"] = PERSONALITIES.get(out["personality"], "unknown")
  return out


def write_setting(params, name: str, value: Any) -> dict[str, Any]:
  """Write one Dash control through Justin's Params / helpers. Returns the new snapshot."""
  api = _api_name(name)
  if api == "hypermile":
    apply_hypermile_toggle(params, _as_bool(value))
  elif api == "sl":
    apply_dm_simulate_looking(params, _as_bool(value))
  elif api == "fai":
    apply_dm_false_alert_ignore(params, _as_bool(value))
  elif api in _BOOL_SPECS:
    parsed = _as_bool(value)
    params.put_bool(_BOOL_SPECS[api]["param"], parsed)
    if api == "experimental" and parsed:
      try:
        params.put_bool(PARAM_EXPERIMENTAL_CONFIRMED, True)
      except Exception:
        pass
  else:
    params.put(_INT_SPECS[api]["param"], _clamp_int(api, value))
  return read_settings(params)


def setting_catalog() -> list[dict[str, Any]]:
  """UI metadata: Mannerisms first, then Map Speed, then stock / SL-FAI."""
  return [
    {"name": "accel", "param": PARAM_ACCEL, "section": "mannerisms", "label": "Acceleration", "kind": "int", "min": 1, "max": 10},
    {"name": "adaptive_accel", "param": PARAM_ADAPTIVE_ACCEL, "section": "mannerisms", "label": "Adaptive Accel", "kind": "bool"},
    {"name": "follow_distance", "param": PARAM_FOLLOW_DISTANCE, "section": "mannerisms", "label": "Follow Distance", "kind": "int", "min": 1, "max": 7},
    {"name": "driver_lat_handoff", "param": NAP_DRIVER_LAT_HANDOFF, "section": "mannerisms", "label": "Soft Lateral Handoff", "kind": "bool"},
    {"name": "one_pedal_long", "param": NAP_ONE_PEDAL_LONG, "section": "mannerisms", "label": "One-Pedal Long", "kind": "bool"},
    {"name": "hypermile", "param": NAP_HYPERMILE, "section": "mannerisms", "label": "Hypermile", "kind": "bool"},
    {"name": "hypermile_step_down", "param": NAP_HYPERMILE_STEP_DOWN,
     "section": "mannerisms", "label": "Step Down Speed", "kind": "bool", "visible_if": "hypermile"},
    {"name": "hypermile_hill_climb", "param": NAP_HYPERMILE_HILL_CLIMB,
     "section": "mannerisms", "label": "Hill Climb", "kind": "bool", "visible_if": "hypermile"},
    {"name": "map_speed_mode", "param": PARAM_MAP_MODE, "section": "map_speed",
     "label": "Map Speed (MAX)", "kind": "choice", "choices": ["Off", "Display", "Cap", "Follow"]},
    {"name": "map_speed_offset_mph", "param": PARAM_MAP_OFFSET, "section": "map_speed",
     "label": "Map Speed Offset", "kind": "choice", "values": [-5, 0, 5], "choices": ["-5 mph", "0", "+5 mph"]},
    {"name": "map_speed_lookahead", "param": PARAM_MAP_LOOKAHEAD, "section": "map_speed",
     "label": "Lookahead", "kind": "choice", "choices": ["Off", "Late", "Normal", "Early"]},
    {"name": "personality", "param": PARAM_PERSONALITY, "section": "stock",
     "label": "Driving Personality", "kind": "choice",
     "choices": ["Aggressive", "Standard", "Chill"], "values": [0, 1, 2]},
    {"name": "experimental", "param": PARAM_EXPERIMENTAL, "section": "stock", "label": "Experimental Mode", "kind": "bool"},
    {"name": "sl", "param": PARAM_SL, "section": "dm", "label": "SL", "kind": "bool"},
    {"name": "fai", "param": PARAM_FAI, "section": "dm", "label": "FAI", "kind": "bool"},
  ]
