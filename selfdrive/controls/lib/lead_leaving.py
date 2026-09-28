"""Lead leaving our path: a continuous weight on the lead's braking.

A lead that slows and turns off (driveway, side street, turn lane) keeps
the planner braking until radard finally drops it, ~2 m off the path.
Justin's drives: Sep 23 21:05 (-3.5 for ~2.5 s while the lead was already
drifting out), Sep 25 11:49 (-2.6...-2.8 while the lead was braking into a
left turn); both overridden with the gas.

The weight is how sure we are that the lead will be out of our way by the
time we get there. It is built from:

  * the lead's lateral offset from the model's predicted path (not raw
    yRel, so a lead in a bend stays on-path),
  * its filtered lateral velocity away from that path, and
  * the trend of the in-path confidence (modelProb falling).

Safety bound, in the math rather than a rule: the offset is projected
forward to our time of arrival (the gap closed at today's speed while the
lead keeps braking at its aLead until it stops; it only moves sideways
while it rolls), and the weight only rises once that projected offset
clears our half-width + the lead's half-width + a margin.
A lead that stalls partway (lateral velocity ~0) projects to where it is,
which does not clear, so it keeps full weight. A lead whose center is still
inside our vehicle's half of the lane gets full weight regardless of its
lateral velocity (overlap gate), which also keeps ordinary in-lane drift
from ever releasing anything.

The weight only moves a lead-induced *brake* toward the lead-free command
(hold speed, or the MAX / map / curve decel if that is deeper). It never
adds +a, so MAX stays a hard ceiling and the EV settle is untouched.
Radard's lead selection is shared by the legacy and unified paths, so the
planner applies this one weight to both.
"""
from __future__ import annotations

import math

# Geometry (m). Model S body is 1.96 m without mirrors; a typical lead ~1.9 m.
EGO_HALF_WIDTH_M = 0.98
LEAD_HALF_WIDTH_M = 0.90
CLEAR_MARGIN_M = 0.30
CLEAR_MARGIN_PER_MS = 0.01    # +0.2 m at 20 m/s
CLEAR_BAND_M = 0.5            # smooth rise once the projection clears

# Overlap gate: lead center still within our half of the lane -> no release.
# 0.9 m is where a 1.9 m car hugging a 3.7 m lane's line sits; it has to be
# crossing its lane line before any release begins.
OVERLAP_Y0_M = 0.9
OVERLAP_BAND_M = 0.5

# Projection.
T_ARRIVE_MAX_S = 2.5
PROJ_MAX_M = 2.5
VY_DEADBAND_MS = 0.15         # radar / path noise must not project
TRUST_BASE = 0.7              # share of vy*t used with a steady modelProb
PROB_DROP_FULL_1_S = 0.5      # modelProb falling this fast -> full trust

# Estimator.
TAU_Y_S = 0.15
TAU_VY_S = 0.3
TAU_VY_SLOW_S = 0.7          # both must agree the lead is moving out
VY_RAW_MAX_MS = 2.5          # a car turning off at 15 m/s moves out ~1-3 m/s
TAU_P_S = 0.3
TAU_DP_S = 0.5
WARMUP_S = 0.25
WARMUP_BAND_S = 0.4
RISE_1_S = 1.5                # 0 -> 1 in ~0.7 s
FALL_1_S = 8.0                # braking comes back within ~0.1 s
LEAD_TWO_CLEAR_M = 2.0
LEAD_TWO_BAND_M = 0.6


def _smooth01(x: float) -> float:
  if x <= 0.0:
    return 0.0
  if x >= 1.0:
    return 1.0
  return x * x * (3.0 - 2.0 * x)


def _filt(prev: float | None, sample: float, dt: float, tau: float) -> float:
  if prev is None:
    return float(sample)
  a = dt / (tau + dt)
  return float(prev) + a * (float(sample) - float(prev))


def clearance_needed_m(v_ego: float) -> float:
  """Lateral center-to-path offset at which the lead's body clears ours."""
  return EGO_HALF_WIDTH_M + LEAD_HALF_WIDTH_M + CLEAR_MARGIN_M + CLEAR_MARGIN_PER_MS * max(0.0, float(v_ego))


def time_to_arrive_s(gap: float, v_ego: float, v_lead: float | None = None,
                     a_lead: float = 0.0) -> float:
  """When we pull alongside the lead at today's speed.

  The lead keeps moving forward, braking at a_lead until it stops (lead
  +a is ignored: arriving earlier is the conservative side). Without a lead
  speed this is gap / v_ego, the lead treated as stopped. Capped.
  """
  g = max(0.0, float(gap))
  v_e = max(1.0, float(v_ego))
  if v_lead is None or not math.isfinite(float(v_lead)):
    return min(T_ARRIVE_MAX_S, g / v_e)
  v_l = max(0.0, float(v_lead))
  brake = max(0.0, -float(a_lead)) if math.isfinite(float(a_lead)) else 0.0
  big_a = 0.5 * brake
  b = v_e - v_l
  disc = b * b + 4.0 * big_a * g
  den = b + math.sqrt(max(0.0, disc))
  t = T_ARRIVE_MAX_S if den <= 1e-6 else 2.0 * g / den
  if brake > 1e-6 and t > v_l / brake:
    # Lead stops first: we reach its stopping point.
    t = (g + v_l * v_l / (2.0 * brake)) / v_e
  return min(T_ARRIVE_MAX_S, max(g / v_e, t))


def lateral_time_s(t_arrive: float, v_lead: float | None = None, a_lead: float = 0.0) -> float:
  """A car only moves sideways while it rolls: a lead braking to a stop
  mid-turn stops moving out when it stops."""
  t = max(0.0, float(t_arrive))
  if v_lead is None or not math.isfinite(float(v_lead)) or not math.isfinite(float(a_lead)):
    return t
  brake = max(0.0, -float(a_lead))
  if brake <= 1e-6:
    return t
  return min(t, max(0.0, float(v_lead)) / brake)


def projected_offset_m(abs_y: float, vy_away: float, gap: float, v_ego: float,
                       prob_rate: float = 0.0, v_lead: float | None = None,
                       a_lead: float = 0.0) -> float:
  trust = TRUST_BASE + (1.0 - TRUST_BASE) * _smooth01(-float(prob_rate) / PROB_DROP_FULL_1_S)
  vy_eff = max(0.0, float(vy_away) - VY_DEADBAND_MS)
  t_lat = lateral_time_s(time_to_arrive_s(gap, v_ego, v_lead, a_lead), v_lead, a_lead)
  travel = min(PROJ_MAX_M, vy_eff * t_lat * trust)
  return max(0.0, float(abs_y)) + travel


def leave_weight(abs_y: float, vy_away: float, gap: float, v_ego: float,
                 prob_rate: float = 0.0, v_lead: float | None = None,
                 a_lead: float = 0.0) -> float:
  """0 = full lead weight, 1 = lead will be clear of us when we get there.

  Continuous in every input. Clearance bound x overlap gate.
  """
  if not (math.isfinite(abs_y) and math.isfinite(vy_away) and math.isfinite(gap) and math.isfinite(v_ego)):
    return 0.0
  if not math.isfinite(prob_rate):
    prob_rate = 0.0
  if v_lead is not None and not math.isfinite(float(v_lead)):
    v_lead = None
  if not math.isfinite(float(a_lead)):
    a_lead = 0.0
  y = abs(float(abs_y))
  y_proj = projected_offset_m(y, vy_away, gap, v_ego, prob_rate, v_lead, a_lead)
  clear_w = _smooth01((y_proj - clearance_needed_m(v_ego)) / CLEAR_BAND_M)
  overlap_w = _smooth01((y - OVERLAP_Y0_M) / OVERLAP_BAND_M)
  return clear_w * overlap_w


def release_lead_brake(a_cmd: float, weight: float, a_free: float) -> float:
  """Move a lead brake toward the lead-free command. Never raises above it."""
  w = min(1.0, max(0.0, float(weight)))
  a = float(a_cmd)
  free = float(a_free)
  if w <= 0.0 or a >= free:
    return a
  return a + w * (free - a)


class LeadLeavingEstimator:
  """Filters offset / lateral velocity / confidence trend per lead track."""

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self._lead_id = None
    self._y = None
    self._vy = 0.0
    self._vy_fast = 0.0
    self._vy_slow = 0.0
    self._p = None
    self._dp = 0.0
    self._age = 0.0
    self._gap = 0.0
    self._v_lead = None
    self._a_lead = 0.0
    self._hold_age = 0.0
    self.weight = 0.0
    self.target = 0.0
    self.vy_away = 0.0
    self.abs_y = 0.0

  def update(self, *, dt: float, present: bool, held: bool = False, lead_id=None,
             path_lat: float | None = None, gap: float = 0.0, v_ego: float = 0.0,
             v_lead: float | None = None, a_lead: float = 0.0,
             model_prob: float | None = None, lead_two_path_lat: float | None = None,
             suppress: bool = False) -> float:
    frame_dt = 0.05 if dt is None or float(dt) <= 1e-6 else float(dt)
    if suppress:
      # FCW / MPC crash / should-stop / force-decel: full braking now.
      self.weight = 0.0
      self.target = 0.0
      return 0.0
    if not present:
      self.reset()
      return 0.0
    if held:
      # Short status drop (the planner's lead hold). Radard usually drops a
      # turning lead at its ~2 m gate: dead-reckon the last offset / speed
      # so a lead that was moving out keeps being released, and one that
      # had stalled keeps full weight.
      if self._y is None:
        return self.weight
      self._hold_age += frame_dt
      rolling = 1.0 if self._v_lead is None else float(self._v_lead > 0.5)
      vy_eff = max(0.0, self._vy) * rolling
      self._y += (1.0 if self._y >= 0.0 else -1.0) * vy_eff * frame_dt
      v_l = self._v_lead
      if v_l is not None:
        self._v_lead = max(0.0, v_l + min(0.0, self._a_lead) * frame_dt)
      closing = float(v_ego) - (0.0 if v_l is None else v_l)
      self._gap = max(0.0, self._gap - closing * frame_dt)
      self.abs_y = abs(self._y)
      target = leave_weight(self.abs_y, self._vy * rolling, self._gap, v_ego, self._dp, self._v_lead, self._a_lead)
      return self._slew(target, frame_dt)
    if lead_id != self._lead_id:
      self.reset()
      self._lead_id = lead_id
    self._hold_age = 0.0
    self._gap = max(0.0, float(gap))
    self._v_lead = None if v_lead is None or not math.isfinite(float(v_lead)) else float(v_lead)
    self._a_lead = float(a_lead) if math.isfinite(float(a_lead)) else 0.0

    if path_lat is None or not math.isfinite(float(path_lat)):
      target = 0.0
      self._y = None
    else:
      y_raw = float(path_lat)
      y_prev = self._y
      self._y = _filt(self._y, y_raw, frame_dt, TAU_Y_S)
      vy_signed = 0.0 if y_prev is None else (self._y - y_prev) / frame_dt
      side = 1.0 if self._y >= 0.0 else -1.0
      vy_raw = min(VY_RAW_MAX_MS, max(-VY_RAW_MAX_MS, vy_signed * side))
      self._vy_fast = _filt(self._vy_fast, vy_raw, frame_dt, TAU_VY_S)
      self._vy_slow = _filt(self._vy_slow, vy_raw, frame_dt, TAU_VY_SLOW_S)
      # Association / path noise swings the fast estimate; a real turn-off
      # moves both. A stall drops the fast one, so braking returns quickly.
      self._vy = min(self._vy_fast, self._vy_slow)
      if model_prob is not None and math.isfinite(float(model_prob)):
        p_prev = self._p
        self._p = _filt(self._p, float(model_prob), frame_dt, TAU_P_S)
        dp = 0.0 if p_prev is None else (self._p - p_prev) / frame_dt
        self._dp = _filt(self._dp, dp, frame_dt, TAU_DP_S)
      self._age += frame_dt
      warm = _smooth01((self._age - WARMUP_S) / WARMUP_BAND_S)
      self.abs_y = abs(self._y)
      self.vy_away = self._vy
      target = warm * leave_weight(self.abs_y, self._vy, self._gap, v_ego, self._dp, self._v_lead, self._a_lead)

    # A second lead on our path behind the one turning off keeps full braking.
    if lead_two_path_lat is not None and math.isfinite(float(lead_two_path_lat)):
      target *= _smooth01((abs(float(lead_two_path_lat)) - (LEAD_TWO_CLEAR_M - LEAD_TWO_BAND_M)) / LEAD_TWO_BAND_M)

    return self._slew(target, frame_dt)

  def _slew(self, target: float, frame_dt: float) -> float:
    self.target = target
    if target > self.weight:
      self.weight = min(target, self.weight + RISE_1_S * frame_dt)
    else:
      self.weight = max(target, self.weight - FALL_1_S * frame_dt)
    return self.weight
