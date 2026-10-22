import math
from typing import Any
from dataclasses import dataclass


SPEEDSIGND_HZ = 1.0

HZ_ENV = "NAP_SPEED_SIGN_HZ"

HZ_MIN = 0.2

HZ_MAX = 4.0

SM_HZ = 20.0

MODEL_HZ = 20.0

UNKNOWN_GRACE_S = 2.0

INFER_BUDGET_MS = 100.0

INFER_LOG_PERIOD_S = 15.0

NCPU_DEFAULT = 8

CPU_OVERSUBSCRIBE_EXTRA = 2

DISK_BLOCKED_MIN = 4

JSONL_LINE_MAX = 240

LAG_ALERT_MARKERS = (
  ("driving model lagging", "model-lag"),
  ("modeldlagging", "model-lag"),
  ("communication issue", "controls-lag"),
  ("commissue", "controls-lag"),
  ("high cpu", "cpu-load"),
  ("highcpuusage", "cpu-load"),
)

SPEEDSIGND_NICE = 19

SPEEDSIGND_CORE = 2

SPEEDSIGND_CORES = (SPEEDSIGND_CORE,)

IDLE_FACTOR = 3.0

CPU_BUDGET = 1.0 / (1.0 + IDLE_FACTOR)

CPU_BUDGET_SLACK = 0.03

DUTY_WINDOW_S = 10.0

BURST_EXTRA = 2

MODEL_DROP_PERC = 5.0

MODEL_EXEC_LIMIT_S = 0.050

MODEL_GUARD_PAUSE_S = 10.0

MODEL_SKIP_BACKOFF_CAP_S = 60.0

SECOND_LOOK_FRAMES = 2

SECOND_LOOK_DEADLINE_S = 0.30

VISION_TIMEOUT_MS = 200

SERVICE_NAME = "liveSpeedSignNAP"

ONNX_RETRY_S = 15.0

STATE_BY_RAW = {
  0: "disabled",
  1: "preEnabled",
  2: "enabled",
  3: "softDisabling",
  4: "overriding",
}

OTHERS_PARAM_PERIOD_S = 5.0

OTHERS_PUBLISH_PERIOD_S = 1.0

def read_gap_s(infer_s: float, period_s: float, backoff_s: float = 0.0) -> float:
  """Start-to-start gap. Idle is at least 3× infer, and never faster than the period.

  A 4 s infer waits 12 s, so the next start is 16 s later and the core
  averages 25%. A 50 ms infer still waits out a 1 s period (5% of a core).
  """
  infer_s = max(0.0, float(infer_s))
  period_s = max(0.0, float(period_s))
  paced = max(period_s, infer_s * (1.0 + IDLE_FACTOR))
  return paced + max(0.0, float(backoff_s))

def next_detect_mono(infer_end: float, infer_s: float, period_s: float, budget_s: float,
                     cap_s: float = 0.0, backoff_s: float = 0.0) -> float:
  """Earliest monotonic time another ONNX infer may start.

  Start-to-start is max(period, 4× infer) plus any extra rest. The road
  test at 795d252f ran the next read as soon as the infer ended (~87% of
  core 0). `budget_s` and `cap_s` stay in the signature; they do not add
  a second wait on top of the duty cycle.
  """
  if not math.isfinite(budget_s) or not math.isfinite(cap_s):
    raise ValueError("budget and cap must be finite")
  gap = read_gap_s(infer_s, period_s, backoff_s)
  return (float(infer_end) - float(infer_s)) + gap

def window_compute(spans: list[tuple[float, float]], t: float, window: float = DUTY_WINDOW_S) -> float:
  """Infer seconds overlapping [t-window, t]. spans are (end, duration)."""
  lo = float(t) - float(window)
  total = 0.0
  for end, dur in spans:
    end_f = float(end)
    dur_f = max(0.0, float(dur))
    start = end_f - dur_f
    a = max(start, lo)
    b = min(end_f, float(t))
    if b > a:
      total += b - a
  return total

def earliest_budget_start(
  spans: list[tuple[float, float]], now: float, next_dur: float,
  window: float = DUTY_WINDOW_S, budget: float = CPU_BUDGET,
) -> float:
  """Earliest t ≥ now such that the infer [t, t+next_dur] keeps the window ≤ budget.

  Past spans age out of the trailing window. A burst that already spent the
  allowance waits here until idle has repaid it.
  """
  now_f = float(now)
  next_dur = max(0.0, float(next_dur))
  allowance = float(window) * float(budget)

  def fits(t: float) -> bool:
    finish = t + next_dur
    used = window_compute(spans, finish, window)
    return used + next_dur <= allowance + 1e-6

  if fits(now_f):
    return now_f
  candidates = [now_f]
  for end, _dur in spans:
    candidates.append(float(end) + float(window) - next_dur)
  for t in sorted(candidates):
    t = max(now_f, t)
    if fits(t):
      return t
  newest = max((float(end) for end, _dur in spans), default=now_f)
  return max(now_f, newest + float(window) - next_dur)

def schedule_after_read(
  *,
  now: float,
  infer_s: float,
  period_s: float,
  budget_s: float,
  cap_s: float,
  backoff_s: float,
  sign_like: bool,
  window_used: int,
  window_count: int,
  burst_left: int,
  burst_window: int,
  burst_cluster_s: float,
  spans: list[tuple[float, float]],
) -> tuple[float, int, int, int, float, bool]:
  """Next detect time plus burst state.

  Returns (next_mono, burst_left, burst_window, next_window, cluster_s, bursting_next).

  Steady reads keep max(period, 4× infer). A sign-like score schedules up to
  BURST_EXTRA more reads on the same tile, immediately, while that cluster
  still fits in the 10 s × 25% allowance. The following non-burst start waits
  until the trailing window is back at 25%. A model-guard backoff cancels the
  burst. `budget_s` and `cap_s` stay on next_detect_mono; they do not add a
  second wait.
  """
  infer_s = max(0.0, float(infer_s))
  spans_now = list(spans) + [(float(now), infer_s)]
  paced = next_detect_mono(float(now), infer_s, period_s, budget_s, cap_s, backoff_s)
  duty_t = earliest_budget_start(spans_now, float(now), infer_s)
  steady = max(paced, duty_t)
  nwin = max(1, int(window_count))
  next_window = (int(window_used) + 1) % nwin
  allowance = DUTY_WINDOW_S * CPU_BUDGET
  if float(backoff_s) > 0.0:
    return steady, 0, int(window_used), next_window, 0.0, False
  if int(burst_left) > 0:
    cluster = float(burst_cluster_s) + infer_s
    left = int(burst_left) - 1
    if left > 0 and cluster + infer_s <= allowance + 1e-6:
      return float(now), left, int(burst_window), int(burst_window), cluster, True
    return steady, 0, int(burst_window), next_window, 0.0, False
  if sign_like:
    cluster = infer_s
    if BURST_EXTRA > 0 and cluster + infer_s <= allowance + 1e-6:
      return float(now), BURST_EXTRA, int(window_used), int(window_used), cluster, True
  return steady, 0, int(window_used), next_window, 0.0, False