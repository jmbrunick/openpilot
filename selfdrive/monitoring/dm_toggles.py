"""Simulate Look / False Alert Ignore — mutually exclusive hidden DM prefs.

Only one may be On. Both Off is allowed. Stale both-On (old installs that
defaulted both On) resolves to Simulate Look On + False Alert Ignore Off
on first read — matches nap-dev default intent.
"""

PARAM_DM_SIMULATE_LOOKING = "NAPDmSimulateLooking"
PARAM_DM_FALSE_ALERT_IGNORE = "NAPDmFalseAlertIgnore"

# nap-release Reset-All / missing-param defaults are both Off.
# Both-On migration (old nap-dev install) still prefers Simulate Look.
DEFAULT_SIMULATE_LOOKING = False
DEFAULT_FALSE_ALERT_IGNORE = False


def exclusive_dm_toggle_states(simulate_looking: bool, false_alert_ignore: bool) -> tuple[bool, bool]:
  """Resolve a pair. Both-On → Simulate Look On / FAI Off. Both Off stays Off."""
  sim = bool(simulate_looking)
  fai = bool(false_alert_ignore)
  if sim and fai:
    return True, False
  return sim, fai


def _get_bool(params, name: str, default: bool) -> bool:
  if params is None:
    return default
  try:
    return bool(params.get_bool(name))
  except Exception:
    return default


def _put_bool(params, name: str, value: bool) -> None:
  if params is None:
    return
  params.put_bool(name, bool(value))


def apply_dm_simulate_looking(params, on: bool) -> tuple[bool, bool]:
  """Write Simulate Look. On forces False Alert Ignore Off."""
  sim = bool(on)
  fai = False if sim else _get_bool(params, PARAM_DM_FALSE_ALERT_IGNORE, DEFAULT_FALSE_ALERT_IGNORE)
  _put_bool(params, PARAM_DM_SIMULATE_LOOKING, sim)
  if sim:
    _put_bool(params, PARAM_DM_FALSE_ALERT_IGNORE, False)
    fai = False
  return sim, fai


def apply_dm_false_alert_ignore(params, on: bool) -> tuple[bool, bool]:
  """Write False Alert Ignore. On forces Simulate Look Off."""
  fai = bool(on)
  sim = False if fai else _get_bool(params, PARAM_DM_SIMULATE_LOOKING, DEFAULT_SIMULATE_LOOKING)
  _put_bool(params, PARAM_DM_FALSE_ALERT_IGNORE, fai)
  if fai:
    _put_bool(params, PARAM_DM_SIMULATE_LOOKING, False)
    sim = False
  return sim, fai


def read_exclusive_dm_toggles(params, *, persist: bool = True) -> tuple[bool, bool]:
  """Read both params. Persist Simulate Look-wins if a stale both-On is found."""
  sim = _get_bool(params, PARAM_DM_SIMULATE_LOOKING, DEFAULT_SIMULATE_LOOKING)
  fai = _get_bool(params, PARAM_DM_FALSE_ALERT_IGNORE, DEFAULT_FALSE_ALERT_IGNORE)
  resolved_sim, resolved_fai = exclusive_dm_toggle_states(sim, fai)
  if persist and (resolved_sim != sim or resolved_fai != fai):
    try:
      _put_bool(params, PARAM_DM_SIMULATE_LOOKING, resolved_sim)
      _put_bool(params, PARAM_DM_FALSE_ALERT_IGNORE, resolved_fai)
    except Exception:
      pass
  return resolved_sim, resolved_fai
