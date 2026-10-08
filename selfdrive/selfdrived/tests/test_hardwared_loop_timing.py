"""A hardware_thread iteration over 1 s names the blocking step."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

# Load the module by path. Importing openpilot.system.hardware pulls in cereal,
# which these timing checks do not need.
_spec = importlib.util.spec_from_file_location(
  "hardwared_loop_timing", ROOT / "system" / "hardware" / "loop_timing.py")
assert _spec is not None and _spec.loader is not None
_loop_timing = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_loop_timing)
HardwareLoopTimer = _loop_timing.HardwareLoopTimer
slow_loop_fields = _loop_timing.slow_loop_fields


class _Clock:
  def __init__(self):
    self.t = 0.0

  def __call__(self):
    return self.t


def test_loop_at_one_second_is_not_logged():
  clock = _Clock()
  timer = HardwareLoopTimer(clock)
  clock.t = 1.0
  timer.mark("statvfs")
  assert slow_loop_fields(timer) is None


def test_slow_loop_names_step_durations():
  clock = _Clock()
  timer = HardwareLoopTimer(clock)
  clock.t = 0.05
  timer.mark("sm_update")
  clock.t = 0.06
  timer.mark("force_offroad_params")
  clock.t = 5.86
  timer.mark("statvfs")
  fields = slow_loop_fields(timer)
  assert fields is not None
  assert fields["sm_update_ms"] == 50.0
  assert fields["force_offroad_params_ms"] == 10.0
  assert fields["statvfs_ms"] == 5800.0
  assert fields["total_ms"] == 5860.0


def test_hardwared_times_steps_without_new_subscribers():
  src = (ROOT / "system" / "hardware" / "hardwared.py").read_text()
  assert 'PubMaster([\'deviceState\'])' in src
  assert 'SubMaster(["peripheralState", "gpsLocationExternal", "selfdriveState", "pandaStates"], poll="pandaStates")' in src
  assert "carState" not in src
  assert 'cloudlog.warning({"event": "hardwared slow loop"' in src
  assert 'cloudlog.event("hardwared slow loop", error=True' in src
  assert 'timer.mark("statvfs")' in src
  assert 'timer.mark("sm_update")' in src
  assert 'timer.mark("uptime_params")' in src
