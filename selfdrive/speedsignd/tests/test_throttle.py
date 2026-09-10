"""speedsignd yields to modeld: 1 Hz default, skip-on-overrun, low nice."""
from __future__ import annotations

import os
from types import SimpleNamespace

from openpilot.selfdrive.speedsignd.speedsignd import (
  HZ_ENV,
  HZ_MAX,
  HZ_MIN,
  INFER_BUDGET_MS,
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
  assert INFER_BUDGET_MS == 100.0
  assert SPEEDSIGND_NICE == 19
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

  def _drop():
    called["drop"] = True

  def _nice(n):
    called["nice"] = n
    return n

  monkeypatch.setattr("openpilot.common.realtime.drop_realtime", _drop)
  monkeypatch.setattr(os, "nice", _nice)
  yield_to_modeld()
  assert called.get("drop") is True
  assert called.get("nice") == SPEEDSIGND_NICE
