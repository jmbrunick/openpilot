"""Housekeeping commIssue policy for deviceState and managerState.

Those two 2 Hz messages are not on the driving path. SubMaster's strict
alive check is 10 periods (5 s at 2 Hz), which is what turned a hardwared
publish stall into "Communication Issue Between Processes" while carState,
controls, model, and radar all stayed healthy.

They are left out of that strict alive/freq/valid check. This module
raises commIssue only after either one has been silent for more than
HOUSEKEEPING_SILENCE_S, measured from the receive timestamp or, when that
was never recorded, from logMonoTime.
"""

HOUSEKEEPING_SERVICES = ("deviceState", "managerState")
HOUSEKEEPING_SILENCE_S = 10.0


def message_silence_s(recv_time: float, log_mono_time: int, now: float) -> float | None:
  """Seconds since the last message.

  `recv_time` is SubMaster.recv_time (time.monotonic when the message
  arrived). `log_mono_time` is the publisher logMonoTime in nanoseconds.
  Arrival is what "silent" means, so a recorded recv_time wins. logMonoTime
  covers the case where the only timestamp we have is the publish time,
  compared on the same second-scale clock as `now`. None means the service
  has never been observed.
  """
  if recv_time > 0.0:
    return now - recv_time
  if log_mono_time > 0:
    return now - (log_mono_time * 1e-9)
  return None


def device_state_seen(recv_time: float, log_mono_time: int) -> bool:
  """True once deviceState has arrived at least once.

  Thermal, free space, and memory reads must keep using that last message
  across a short stall. A zeroed stand-in (free space 0) is not a real sample.
  """
  return recv_time > 0.0 or log_mono_time > 0


def housekeeping_silent_services(
  recv_times: dict[str, float],
  log_mono_times: dict[str, int],
  now: float,
  *,
  frame: int = 0,
  dt: float = 0.01,
  skip: tuple[str, ...] = (),
) -> list[str]:
  """Services in HOUSEKEEPING_SERVICES that have been silent for more than 10 s.

  A service that has never been observed ages with the controller frame clock
  (`frame * dt`) so a hardwared that never starts still disengages, about 5 s
  later than the old 5 s alive window. `skip` is for modes that already drop
  a service entirely (managerState under SIMULATION / REPLAY).
  """
  silent: list[str] = []
  for service in HOUSEKEEPING_SERVICES:
    if service in skip:
      continue
    age = message_silence_s(
      recv_times.get(service, 0.0),
      log_mono_times.get(service, 0),
      now,
    )
    if age is None:
      age = frame * dt
    if age > HOUSEKEEPING_SILENCE_S:
      silent.append(service)
  return silent


def classify_comm_issue(
  *,
  all_alive: bool,
  all_freq_ok: bool,
  all_valid: bool,
  housekeeping_silent: list[str],
) -> str | None:
  """commIssue / commIssueAvgFreq for one selfdrived step.

  `all_*` are the strict SubMaster results and must already exclude
  housekeeping services. A housekeeping silence is commIssue, same as a
  driving service that is not alive. Frequency and valid failures of every
  other service are unchanged.
  """
  if housekeeping_silent or not all_alive:
    return "commIssue"
  if not all_freq_ok:
    return "commIssueAvgFreq"
  if not all_valid:
    return "commIssue"
  return None


def device_health_events(
  thermal_status,
  free_space_percent: float,
  memory_usage_percent: int,
  *,
  simulation: bool,
  overheated,
) -> list[str]:
  """Events from the last deviceState sample.

  Callers pass the message still held by SubMaster. Do not substitute zeros
  when the publisher stalls, and do not consult alive / freq / valid here.
  """
  events: list[str] = []
  if thermal_status >= overheated:
    events.append("overheat")
  if free_space_percent < 7 and not simulation:
    events.append("outOfSpace")
  if memory_usage_percent > 90 and not simulation:
    events.append("lowMemory")
  return events


def submaster_ignore_services(
  sensor_packets: list[str],
  gps_packets: list[str],
  *,
  simulation: bool,
  replay: bool,
) -> list[str]:
  """Services dropped from SubMaster alive, average-frequency, and valid checks.

  Housekeeping is included so a short deviceState stall cannot fail
  all_checks(). Every driving service stays on the strict checks.
  """
  ignore = list(sensor_packets) + list(gps_packets) + [
    "alertDebug",
    "lateralManeuverPlan",
    "pathObstacleNAP",
    "pathObstacleVisionNAP",
    *HOUSEKEEPING_SERVICES,
  ]
  if simulation:
    ignore += ["driverCameraState", "managerState"]
  if replay:
    # no vipc in replay will make them ignored anyways
    # sanitized fixtures omit driverCameraState/managerState; ignore them in replay
    ignore += ["roadCameraState", "wideRoadCameraState", "driverCameraState", "managerState"]
  return ignore
