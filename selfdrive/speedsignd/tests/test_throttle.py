"""speedsignd yields to modeld: 1 Hz default, one-infer spacing, low nice."""
from __future__ import annotations

import os
from types import SimpleNamespace

from openpilot.selfdrive.speedsignd.detect import (
  CAP_MS_DEFAULT,
  CAP_MS_ENV,
  THREADS_DEFAULT,
  THREADS_ENV,
  limit_infer_threads,
  parse_infer_cap_ms,
  parse_infer_threads,
)
from openpilot.selfdrive.speedsignd.speedsignd import (
  HZ_ENV,
  HZ_MAX,
  HZ_MIN,
  INFER_BUDGET_MS,
  SM_HZ,
  SPEEDSIGND_CORES,
  SPEEDSIGND_HZ,
  SPEEDSIGND_NICE,
  HostPressure,
  backoff_extra_s,
  format_backoff_timing,
  format_read_timing,
  infer_overran,
  next_detect_mono,
  parse_detect_hz,
  pressure_reason,
  read_gap_s,
  reset_ratekeeper_if_behind,
  steady_core_duty,
  yield_to_modeld,
)


def test_default_detect_hz_is_1():
  assert SPEEDSIGND_HZ == 1.0
  assert 10.0 <= SM_HZ <= 20.0
  assert SM_HZ > SPEEDSIGND_HZ
  assert INFER_BUDGET_MS == 100.0
  assert SPEEDSIGND_NICE == 19
  assert SPEEDSIGND_CORES == (0, 1, 2, 3)
  assert THREADS_DEFAULT == 1
  assert CAP_MS_DEFAULT == 800.0
  assert parse_detect_hz(None) == 1.0
  assert parse_detect_hz("") == 1.0
  assert parse_detect_hz("nope") == 1.0
  assert parse_detect_hz("nan") == 1.0


def test_detect_hz_env_clamped():
  assert parse_detect_hz("0.5") == 0.5
  assert parse_detect_hz("2") == 2.0
  assert parse_detect_hz("0.01") == HZ_MIN
  assert parse_detect_hz("99") == HZ_MAX
  assert HZ_ENV == "NAP_SPEED_SIGN_HZ"


def _start_gap(end: float, infer_s: float, period: float, budget: float, cap: float = 0.0, backoff: float = 0.0) -> float:
  nxt = next_detect_mono(end, infer_s, period, budget, cap, backoff_s=backoff)
  return nxt - (end - infer_s)


def test_cheap_infer_waits_out_the_period():
  period, budget = 1.0, 0.100
  infer_s = 0.050
  assert not infer_overran(infer_s, period, budget)
  # Period is longer than the infer, so starts are one period apart.
  assert abs(_start_gap(10.0, infer_s, period, budget) - period) < 1e-9


def test_over_budget_but_under_period_keeps_the_period():
  period, budget = 1.0, 0.100
  infer_s = 0.350  # over the 100 ms budget, still under the 1 Hz period
  assert infer_overran(infer_s, period, budget)
  # The budget flag must not add another period after the infer.
  assert abs(_start_gap(10.0, infer_s, period, budget, cap=0.800) - period) < 1e-9


def test_over_period_spaces_one_infer_not_two():
  period, budget = 1.0, 0.100
  infer_s = 1.5
  assert infer_overran(infer_s, period, budget)
  assert abs(_start_gap(5.0, infer_s, period, budget) - infer_s) < 1e-9


def test_reset_ratekeeper_drops_backlog():
  rk = SimpleNamespace(remaining=-0.4, _next_frame_time=3.0, _interval=1.0)
  assert reset_ratekeeper_if_behind(rk, now=10.0) is True
  assert rk._next_frame_time == 11.0
  rk.remaining = 0.2
  rk._next_frame_time = 12.0
  assert reset_ratekeeper_if_behind(rk, now=11.8) is False
  assert rk._next_frame_time == 12.0


def test_4hz_would_saturate_typical_onnx_budget():
  """Hypothesis: 4 Hz * ~250–400 ms tinygrad infer ≥ 100% of a core. Default stays 1 Hz."""
  infer_s = 0.30
  assert 4.0 * infer_s >= 1.0
  assert SPEEDSIGND_HZ == 1.0
  assert SPEEDSIGND_HZ * infer_s <= 0.40
  duty = steady_core_duty(infer_s, 1.0 / SPEEDSIGND_HZ)
  assert abs(duty - infer_s) < 1e-9


def test_yield_to_modeld_sets_nice_and_drops_rt(monkeypatch):
  called: dict[str, object] = {}
  fake_rt = SimpleNamespace(
    drop_realtime=lambda: called.setdefault("drop", True),
    set_core_affinity=lambda cores: called.setdefault("cores", tuple(cores)),
  )
  monkeypatch.setitem(__import__("sys").modules, "openpilot.common.realtime", fake_rt)

  def _nice(n):
    called["nice"] = n
    return n

  monkeypatch.setattr(os, "nice", _nice)
  yield_to_modeld()
  assert called.get("drop") is True
  assert called.get("nice") == SPEEDSIGND_NICE
  assert called.get("cores") == SPEEDSIGND_CORES


def test_infer_threads_default_one_and_clamped():
  assert parse_infer_threads(None) == 1
  assert parse_infer_threads("") == 1
  assert parse_infer_threads("nope") == 1
  assert parse_infer_threads("8") == 2
  assert parse_infer_threads("0") == 1
  assert THREADS_ENV == "NAP_SPEED_SIGN_THREADS"
  n = limit_infer_threads(1)
  assert n == 1
  assert os.environ["OMP_NUM_THREADS"] == "1"
  assert os.environ["OPENBLAS_NUM_THREADS"] == "1"


def test_infer_cap_ms_default_and_zero_disables():
  assert parse_infer_cap_ms(None) == 800.0
  assert parse_infer_cap_ms("") == 800.0
  assert parse_infer_cap_ms("nope") == 800.0
  assert parse_infer_cap_ms("0") == 0.0
  assert parse_infer_cap_ms("50") == 50.0
  assert parse_infer_cap_ms("99999") == 5000.0
  assert CAP_MS_ENV == "NAP_SPEED_SIGN_INFER_CAP_MS"


def test_device_infer_is_one_infer_apart_not_two_plus_a_second():
  """Offline eval: 3.8–5.4 s infer started ~8.6–11.8 s apart (2× infer + 1 s cap)."""
  period, budget, cap = 1.0, 0.100, 0.800
  for infer_s, old_gap in ((3.8, 8.6), (5.4, 11.8)):
    gap = _start_gap(100.0, infer_s, period, budget, cap)
    assert abs(gap - infer_s) < 1e-9
    assert abs((2.0 * infer_s + period) - old_gap) < 1e-9
    assert gap < old_gap - 3.0
  # Cap no longer adds the extra period on a 1.82 s session.
  assert abs(_start_gap(10.0, 1.82, period, budget, cap) - 1.82) < 1e-9


def test_period_wins_when_it_is_longer_than_infer():
  assert abs(read_gap_s(0.35, 1.0) - 1.0) < 1e-9
  assert abs(_start_gap(10.0, 0.35, 1.0, 0.100, 0.800) - 1.0) < 1e-9


def test_pressure_backoff_halves_one_little_core():
  infer_s, period = 4.0, 1.0
  healthy = HostPressure(procs_running=6, procs_blocked=0, ncpu=8, alert_text="")
  assert pressure_reason(healthy) == ""
  assert backoff_extra_s(infer_s, period, "") == 0.0
  # One little core flat out, none of cores 4–7. 25% of the little cluster.
  assert abs(steady_core_duty(infer_s, period) - 1.0) < 1e-9
  assert abs(steady_core_duty(infer_s, period) / len(SPEEDSIGND_CORES) - 0.25) < 1e-9

  hot = HostPressure(procs_running=8 + 2 + 1, procs_blocked=0, ncpu=8)
  assert pressure_reason(hot) == "cpu-load"
  extra = backoff_extra_s(infer_s, period, pressure_reason(hot))
  assert extra == infer_s
  assert abs(steady_core_duty(infer_s, period, extra) - 0.5) < 1e-9
  assert abs(_start_gap(20.0, infer_s, period, 0.1, 0.8, backoff=extra) - 8.0) < 1e-9

  lag = HostPressure(alert_text="Driving Model Lagging 22.0% frames dropped", procs_running=4, ncpu=8)
  assert pressure_reason(lag) == "model-lag"
  controls = HostPressure(alert_text="Communication Issue Between Processes", procs_running=3, ncpu=8)
  assert pressure_reason(controls) == "controls-lag"
  disk = HostPressure(procs_running=4, procs_blocked=4, ncpu=8)
  assert pressure_reason(disk) == "disk-stall"


def test_timing_lines_are_greppable():
  line = format_read_timing(
    read_interval_ms=4200, infer_ms=3980, engaged=True, backoff=False, reason="pace",
  )
  assert line.startswith("speedsignd timing ")
  assert "read_interval_ms=4200" in line
  assert "infer_ms=3980" in line
  assert "engaged=1" in line
  assert "backoff=0" in line
  event = format_backoff_timing(extra_ms=3980, infer_ms=3980, engaged=True, reason="model-lag")
  assert event.startswith("speedsignd timing ")
  assert "backoff=1" in event and "reason=model-lag" in event and "engaged=1" in event


def test_driving_cores_stay_with_modeld_and_controlsd():
  """modeld is FIFO on core 7; controlsd is CTRL_HIGH on core 4. speedsignd is not."""
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  modeld = (root / "selfdrive" / "modeld" / "modeld.py").read_text(encoding="utf-8")
  controlsd = (root / "selfdrive" / "controls" / "controlsd.py").read_text(encoding="utf-8")
  assert "config_realtime_process(7, 54)" in modeld
  assert "config_realtime_process(4, Priority.CTRL_HIGH)" in controlsd
  assert 7 not in SPEEDSIGND_CORES
  assert 4 not in SPEEDSIGND_CORES
  assert SPEEDSIGND_NICE >= 19
  assert THREADS_DEFAULT == 1
