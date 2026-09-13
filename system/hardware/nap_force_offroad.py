"""NAP Force Offroad / Go Offline — started=false while the car can still move.

Settings → NAP toggle. hardwared applies this as
onroad_conditions['not_force_offroad'] so deviceState.started stays false.

When Pre-AP software long is active, card holds this off until stock CC is
ENABLED (or a short timeout) so pedal long does not drop into hard regen.
See selfdrive/car/tesla/preap_force_offroad_handoff.py.
"""

PARAM = "NAPForceOffroad"
HANDOFF_READY_PARAM = "NAPForceOffroadHandoffReady"
CONFIRMED_PARAM = "NAPForceOffroadConfirmed"

# On-road Yes/No copy. Keep Justin's wording.
CONFIRM_PROMPT = "Ready to resume steering control?"
CONFIRM_YES = "Yes"
CONFIRM_NO = "No"

# hardwared fallback if card never writes HandoffReady (non-Pre-AP card
# miss, card already dying). Card's own budget is 2.5 s; keep this a bit
# longer so the onroad SET can finish first.
HANDOFF_TIMEOUT_S = 3.0


def needs_driver_confirm(force_offroad: bool, confirmed: bool, already_started: bool) -> bool:
  """On-road Force Offroad waits for the Yes/No popup before any handoff."""
  return bool(force_offroad) and bool(already_started) and not confirmed


def apply_force_offroad_toggle(params, on: bool, *, started: bool) -> None:
  """Write the toggle and confirm/handoff flags. Parked skips the popup."""
  params.put_bool(PARAM, bool(on))
  if not on:
    params.put_bool(CONFIRMED_PARAM, False)
    params.put_bool(HANDOFF_READY_PARAM, False)
    return
  params.put_bool(CONFIRMED_PARAM, not started)
  if not started:
    return
  params.put_bool(HANDOFF_READY_PARAM, False)


def confirm_force_offroad(params) -> None:
  params.put_bool(CONFIRMED_PARAM, True)


def cancel_force_offroad(params) -> None:
  """No: drop all Force Offroad intent. Leave NAP/OP as it was."""
  apply_force_offroad_toggle(params, False, started=False)


def allows_onroad(force_offroad: bool, *, handoff_ready: bool = True,
                  already_started: bool = False, timed_out: bool = False,
                  confirmed: bool = True) -> bool:
  """True unless Force Offroad may flip started=false.

  Default kwargs keep the old one-arg call: toggle ON + not already
  started → block start (parked / already offroad). While onroad, hold
  started until the driver taps Yes *and* card reports stock-CC ENABLED
  (or timeout). Do not start the timeout until Yes.
  """
  if not force_offroad:
    return True
  if not already_started:
    return False
  if not confirmed:
    return True
  return not (handoff_ready or timed_out)


def handoff_wait_timed_out(wait_started_mono: float | None, now_mono: float,
                           timeout_s: float = HANDOFF_TIMEOUT_S) -> bool:
  if wait_started_mono is None:
    return False
  return (now_mono - wait_started_mono) >= timeout_s


def should_start_now(onroad_conditions: dict[str, bool], startup_conditions: dict[str, bool],
                     already_started: bool) -> bool:
  """deviceState.started decision. already_started skips startup_conditions."""
  should_start = all(onroad_conditions.values())
  if not already_started:
    should_start = should_start and all(startup_conditions.values())
  return should_start
