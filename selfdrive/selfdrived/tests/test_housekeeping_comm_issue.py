"""deviceState / managerState stalls must not raise commIssue before 10 s.

Driving services stay on SubMaster's strict alive window (10 / frequency).
"""
import re
from pathlib import Path

from openpilot.selfdrive.selfdrived.housekeeping_comm import (
  HOUSEKEEPING_SERVICES,
  classify_comm_issue,
  device_health_events,
  device_state_seen,
  housekeeping_silent_services,
  message_silence_s,
  submaster_ignore_services,
)

ROOT = Path(__file__).resolve().parents[3]
NOW = 1_000_000.0

# Services selfdrived still strict-checks. carState is a separate socket and
# is not in this SubMaster; its readers are unchanged.
DRIVING_SERVICES = (
  "pandaStates",
  "modelV2",
  "controlsState",
  "radarState",
  "carControl",
  "carOutput",
  "longitudinalPlan",
  "livePose",
  "liveCalibration",
)


def _freq(name: str) -> float:
  text = (ROOT / "cereal" / "services.py").read_text()
  match = re.search(rf'"{name}":\s*\((?:True|False),\s*([0-9.]+)', text)
  assert match is not None, name
  return float(match.group(1))


def _strict_alive(age_s: float, frequency: float) -> bool:
  """SubMaster.alive: age < 10/frequency. Locked against the source below."""
  return age_s < (10.0 / frequency)


def _kind(device_age_s: float, *, manager_age_s: float = 0.5, all_alive: bool = True,
          all_freq_ok: bool = True, all_valid: bool = True, log_mono_only: bool = False) -> str | None:
  if log_mono_only:
    recv = {"deviceState": 0.0, "managerState": NOW - manager_age_s}
  else:
    recv = {"deviceState": NOW - device_age_s, "managerState": NOW - manager_age_s}
  mono = {
    "deviceState": int((NOW - device_age_s) * 1e9),
    "managerState": int((NOW - manager_age_s) * 1e9),
  }
  silent = housekeeping_silent_services(recv, mono, NOW, frame=100000, dt=0.01)
  return classify_comm_issue(
    all_alive=all_alive,
    all_freq_ok=all_freq_ok,
    all_valid=all_valid,
    housekeeping_silent=silent,
  )


def test_six_second_device_state_gap_does_not_raise_comm_issue():
  # Route 176 segment 4 stalled for 5.8 s. A 6 s gap is the same class.
  assert _kind(6.0) is None
  assert message_silence_s(NOW - 6.0, int((NOW - 6.0) * 1e9), NOW) == 6.0


def test_device_state_gap_over_ten_seconds_raises_comm_issue():
  assert _kind(10.0) is None
  assert _kind(10.1) == "commIssue"
  assert _kind(11.0) == "commIssue"


def test_device_state_gap_over_ten_seconds_uses_log_mono_time():
  assert _kind(6.0, log_mono_only=True) is None
  assert _kind(10.1, log_mono_only=True) == "commIssue"


def test_fresh_recv_time_wins_over_stale_log_mono_time():
  # A message that just arrived is not silent, even if its publish timestamp is old.
  age = message_silence_s(NOW - 1.0, int((NOW - 30.0) * 1e9), NOW)
  assert age == 1.0
  silent = housekeeping_silent_services(
    {"deviceState": NOW - 1.0, "managerState": NOW},
    {"deviceState": int((NOW - 30.0) * 1e9), "managerState": int(NOW * 1e9)},
    NOW,
  )
  assert silent == []


def test_manager_state_gap_over_ten_seconds_raises_comm_issue():
  assert _kind(0.5, manager_age_s=6.0) is None
  assert _kind(0.5, manager_age_s=10.1) == "commIssue"


def test_replay_does_not_require_manager_state():
  silent = housekeeping_silent_services(
    {"deviceState": NOW, "managerState": 0.0},
    {"deviceState": int(NOW * 1e9), "managerState": 0},
    NOW,
    frame=2000,
    dt=0.01,
    skip=("managerState",),
  )
  assert silent == []


def test_other_services_timeouts_unchanged():
  messaging = (ROOT / "cereal" / "messaging" / "__init__.py").read_text()
  assert "(cur_time - self.recv_time[s]) < (10. / SERVICE_LIST[s].frequency)" in messaging

  # Old 2 Hz window is still 5 s. A 6 s gap fails it, which is why housekeeping
  # is excluded from that check instead of changing the formula.
  assert _freq("deviceState") == 2.0
  assert _freq("managerState") == 2.0
  assert not _strict_alive(6.0, _freq("deviceState"))
  assert _strict_alive(4.9, _freq("deviceState"))

  assert _freq("carState") == 100.0
  assert _freq("controlsState") == 100.0
  assert _strict_alive(0.09, _freq("carState"))
  assert not _strict_alive(0.1, _freq("carState"))
  assert _freq("pandaStates") == 10.0
  assert _strict_alive(0.99, _freq("pandaStates"))
  assert not _strict_alive(1.0, _freq("pandaStates"))
  assert _freq("modelV2") == 20.0
  assert _freq("radarState") == 20.0
  assert _strict_alive(0.49, _freq("radarState"))
  assert not _strict_alive(0.5, _freq("radarState"))

  ignore = submaster_ignore_services(
    ["accelerometer", "gyroscope"], ["gpsLocationExternal"], simulation=False, replay=False)
  assert set(HOUSEKEEPING_SERVICES) <= set(ignore)
  for name in DRIVING_SERVICES:
    assert name not in ignore
  # Replay fixtures and the obstacle-chime wiring both grep these literals.
  selfd = (ROOT / "selfdrive" / "selfdrived" / "selfdrived.py").read_text()
  assert "['alertDebug', 'lateralManeuverPlan', 'pathObstacleNAP', 'pathObstacleVisionNAP']" in selfd
  assert "ignore += ['roadCameraState', 'wideRoadCameraState', 'driverCameraState', 'managerState']" in selfd
  assert "ignore += list(HOUSEKEEPING_SERVICES)" in selfd

  # Strict failure of any remaining service still raises, at that service's own window.
  assert _kind(0.4, all_alive=False) == "commIssue"
  assert _kind(0.4, all_freq_ok=False) == "commIssueAvgFreq"
  assert _kind(0.4, all_valid=False) == "commIssue"
  # A housekeeping gap must not be reported as the average-frequency alert.
  assert _kind(6.0) is None
  assert _kind(10.1) == "commIssue"


def test_last_device_state_values_survive_a_short_stall():
  # Last sample before the stall: ~10% free, thermal ok, memory fine.
  assert device_state_seen(NOW - 6.0, int((NOW - 6.0) * 1e9))
  events = device_health_events(0, 10.4, 65, simulation=False, overheated=1)
  assert events == []
  # Zeros are not a real sample. Callers only pass the held message once seen.
  assert not device_state_seen(0.0, 0)
  assert "outOfSpace" in device_health_events(0, 0.0, 0, simulation=False, overheated=1)
  # An actual overheat / low memory in the last sample is still visible.
  assert device_health_events(1, 10.4, 65, simulation=False, overheated=1) == ["overheat"]
  assert "lowMemory" in device_health_events(0, 10.4, 91, simulation=False, overheated=1)


def test_selfdrived_wires_ignore_list_and_last_device_state():
  src = (ROOT / "selfdrive" / "selfdrived" / "selfdrived.py").read_text()
  assert "ignore_alive=ignore, ignore_avg_freq=ignore" in src
  assert "ignore_valid=ignore" in src
  assert "ignore += list(HOUSEKEEPING_SERVICES)" in src
  assert "ds.thermalStatus, ds.freeSpacePercent, ds.memoryUsagePercent" in src
  assert "fan_desired = ds.fanSpeedPercentDesired if ds_seen else 0" in src
  # No new carState subscriber.
  assert src.count("sub_sock('carState'") == 1
