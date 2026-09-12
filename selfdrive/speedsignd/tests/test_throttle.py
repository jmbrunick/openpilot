"""speedsignd yields to modeld: 1 Hz default, skip-on-overrun, low nice."""
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
  infer_overran,
  next_detect_mono,
  parse_detect_hz,
  reset_ratekeeper_if_behind,
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


def test_cheap_infer_does_not_force_extra_skip():
  period, budget = 1.0, 0.100
  infer_s = 0.050
  assert not infer_overran(infer_s, period, budget)
  end = 10.0
  assert next_detect_mono(end, infer_s, period, budget) == end


def test_over_budget_skips_until_free():
  period, budget = 1.0, 0.100
  infer_s = 0.350  # typical YOLOv8s tinygrad CPU
  assert infer_overran(infer_s, period, budget)
  end = 10.0
  nxt = next_detect_mono(end, infer_s, period, budget)
  # Wait a full period after the infer ends — no Ratekeeper catch-up burst.
  assert nxt == end + period
  assert nxt - (end - infer_s) >= period  # start-to-start >= period + infer


def test_over_period_pays_back_infer_time():
  period, budget = 1.0, 0.100
  infer_s = 1.5
  assert infer_overran(infer_s, period, budget)
  end = 5.0
  assert next_detect_mono(end, infer_s, period, budget) == end + infer_s


def test_reset_ratekeeper_drops_backlog():
  rk = SimpleNamespace(remaining=-0.4, _next_frame_time=3.0, _interval=1.0)
  assert reset_ratekeeper_if_behind(rk, now=10.0) is True
  assert rk._next_frame_time == 11.0
  rk.remaining = 0.2
  rk._next_frame_time = 12.0
  assert reset_ratekeeper_if_behind(rk, now=11.8) is False
  assert rk._next_frame_time == 12.0


def test_4hz_would_saturate_typical_onnx_budget():
  """Hypothesis: 4 Hz * ~250–400 ms tinygrad infer ≥ 100% of a core."""
  infer_s = 0.30
  assert 4.0 * infer_s >= 1.0
  assert SPEEDSIGND_HZ * infer_s <= 0.40
  # After overrun at 1 Hz, duty cycle is infer / (infer + period) ≤ 50% when
  # infer ≤ period, and ≤ 50% when infer > period (pay-back rest).
  nxt = next_detect_mono(infer_s, infer_s, 1.0 / SPEEDSIGND_HZ, INFER_BUDGET_MS / 1000.0)
  cycle = nxt
  assert infer_s / cycle <= 0.50 + 1e-9


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


def test_over_cap_adds_an_extra_period():
  """Justin’s 1.8 s tinygrad session must not immediately start another infer."""
  period, budget, cap = 1.0, 0.100, 0.800
  infer_s = 1.82
  end = 10.0
  nxt = next_detect_mono(end, infer_s, period, budget, cap)
  assert nxt == end + infer_s + period
  # Duty cycle with extra skip: infer / (infer + infer + period) ≈ 39%.
  cycle = nxt - (end - infer_s)
  assert infer_s / cycle < 0.45


def test_under_cap_keeps_old_overrun_skip():
  nxt = next_detect_mono(10.0, 0.35, 1.0, 0.100, 0.800)
  assert nxt == 11.0
