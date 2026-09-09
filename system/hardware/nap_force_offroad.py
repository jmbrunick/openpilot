"""NAP Force Offroad / Go Offline — started=false while the car can still move.

Settings → NAP toggle. hardwared applies this as
onroad_conditions['not_force_offroad'] so deviceState.started stays false.
"""

PARAM = "NAPForceOffroad"


def allows_onroad(force_offroad: bool) -> bool:
  """True unless the Force Offroad toggle is on."""
  return not force_offroad


def should_start_now(onroad_conditions: dict[str, bool], startup_conditions: dict[str, bool],
                     already_started: bool) -> bool:
  """deviceState.started decision. already_started skips startup_conditions."""
  should_start = all(onroad_conditions.values())
  if not already_started:
    should_start = should_start and all(startup_conditions.values())
  return should_start
