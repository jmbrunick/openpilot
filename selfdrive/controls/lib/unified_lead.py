"""Continuous lead-follow controller.

One equation replaces the stacked follow overrides when NAPLongUnified is on.
The planner still runs MPC. This module is a few arithmetic ops per frame.

  a_cmd = k_g(gap - gap_set) + k_v(v_lead - v_ego) + k_a·a_lead

gap_set is the selected follow gap (t_follow·v_lead + standstill). Gains grow
smoothly with gap error and closing rate: a small miss stays near the mild
regen the EV already settles at (about −0.22 m/s²); a large miss is firmer.
A kinematic bound

  a_req = -(v_ego - v_lead)² / (2·max(gap - gap_set, eps))

is applied only while closing, with the braking part of a_lead added, so the
car is speed-matched as it arrives on the setpoint. Inside the setpoint the
same term recovers, down to about −1 m/s² under half a second of headway, and
to full regen only when the gap is actually running out.

No threshold steps the command. A jerk limit slews the result (faster when
the kinematic bound is deeper than the current command, gentler when
releasing). v_lead and a_lead are filtered so one radar blip cannot dump
regen. A new lead blends in from the command already in force.
"""
from __future__ import annotations

import math

# Same standstill gap long_mpc uses for the follow obstacle.
STOP_DISTANCE_M = 6.0

# Hard plant bounds. Full regen is ACCEL_MIN; the comfort cap below keeps
# ordinary closes well above it.
A_MIN_MS2 = -3.5
A_MAX_MS2 = 2.0

# Gain endpoints. Severity in [0, 1] interpolates low → high.
K_G_LO = 0.012  # 1/s². Kept modest: large gap error is mostly the kinematic term.
K_G_HI = 0.030
K_V_LO = 0.16   # 1/s. Small closing errors land near mild regen.
K_V_HI = 0.10   # Large closes let a_lead and a_req own the depth.
# a_lead weight: 1 next to the gap, easing off when the lead is still far.
K_A_NEAR = 1.0
K_A_FAR = 0.45
K_A_NEAR_M = 18.0
K_A_FAR_M = 70.0

GAP_SEV_M = 16.0
CLOSE_SEV_MS = 3.2

# Glide: nearly matched at the setpoint contributes nothing you can feel.
GLIDE_GAP_M = 3.5
GLIDE_V_MS = 0.40
GLIDE_KEEP = 0.90  # fraction of the gap/speed term removed at perfect glide

# Kinematic floor. Slack below this uses the 1 m denominator, then the
# inside-gap recovery takes over as slack goes negative.
PHYS_EPS_M = 1.0
INSIDE_BLEND_M = 3.0
# Far from the gap, do not chase a lead's brake below this. The kinematic
# term may go deeper when the remaining gap actually requires it.
COMFORT_FAR_MS2 = -1.0

# Filters. One 50 ms blip moves a_lead by ~1/6 of the spike.
TAU_V_S = 0.20
TAU_A_S = 0.28

# Jerk. Release is gentle; building brake is quicker; the kinematic bound
# may use the fast rate. Continuity tests treat PHYS as the maximum.
JERK_RELEASE_MS3 = 0.45
JERK_RELEASE_OPEN_MS3 = 1.35
JERK_BUILD_MS3 = 1.15
JERK_PHYS_MS3 = 2.50
UNIFIED_JERK_LIMIT_MS3 = JERK_PHYS_MS3

# New lead / cut-in / rematch. Blend from the command already published.
CUTIN_S = 0.45
# Toggle engage/disengage blend lives in the planner (~1 s).
MODE_BLEND_S = 1.0

# Speed ceiling: a <= (v_cap - v_ego) / tau. At the cap this is 0.
SPEED_CEILING_TAU_S = 2.4

# Lateral fade. On-path (|y| under ~1.2 m, after curve geometry) stays.
# A lead walking off past ~2.5 m fades the follow command to 0.
DEPART_Y0_M = 1.2
DEPART_Y1_M = 2.5


def _smooth01(x: float) -> float:
  """Smoothstep on [0, 1]. Constant outside. Value and slope match at the ends."""
  if x <= 0.0:
    return 0.0
  if x >= 1.0:
    return 1.0
  return x * x * (3.0 - 2.0 * x)


def _filt(prev: float | None, sample: float, dt: float, tau: float) -> float:
  if prev is None or tau <= 1e-4:
    return float(sample)
  a = float(dt) / (float(tau) + float(dt))
  return float(prev) + a * (float(sample) - float(prev))


def gap_set_m(v_lead: float, t_follow: float) -> float:
  """Selected follow gap, including standstill distance."""
  return float(t_follow) * max(0.0, float(v_lead)) + STOP_DISTANCE_M


def _severity(slack: float, v_err: float) -> float:
  g = float(slack) / GAP_SEV_M
  v = float(v_err) / CLOSE_SEV_MS
  return 1.0 - math.exp(-(g * g) - (v * v))


def _k_a(slack: float) -> float:
  t = _smooth01((max(float(slack), 0.0) - K_A_NEAR_M) / max(1e-3, K_A_FAR_M - K_A_NEAR_M))
  return K_A_NEAR + (K_A_FAR - K_A_NEAR) * t


def _glide(slack: float, v_err: float, a_lead: float) -> float:
  """1 when matched and the lead is not braking. A real lead brake kills it."""
  g = math.exp(-((float(slack) / GLIDE_GAP_M) ** 2) - ((float(v_err) / GLIDE_V_MS) ** 2))
  brake = min(float(a_lead), 0.0)
  g *= math.exp(-((brake / 0.30) ** 2))
  return g


def _inside_recovery(slack: float, v_close: float, gap: float, v_ego: float,
                     t_follow: float, v_lead: float) -> float:
  """Road-relative recovery once the gap is inside the setpoint.

  Headway under about 0.5 s is allowed down to −1 m/s². A longer headway
  stays near the kinematic need (the 08:03 sample is about −0.4).
  """
  v = max(0.0, float(v_close))
  remain = float(gap) - STOP_DISTANCE_M - 0.5 * float(t_follow) * max(0.0, float(v_lead))
  remain = max(remain, 3.0)
  kin = -(v * v) / (2.0 * remain)
  depth = min(max(-float(slack), 0.0), 15.0)
  a = kin - 0.02 * depth
  hw = float(gap) / max(float(v_ego), 1.0)
  extra = 0.40 * _smooth01((1.0 - hw) / 0.5)
  lo = -0.60 - extra
  if v > 0.0:
    a = min(-0.22, max(lo, a))
    # Stay at least as firm as −0.35 while still closing; release only as
    # the relative speed itself goes away (no 1.5 m/s gate).
    hold = _smooth01((v - 0.12) / 0.18)
    a = min(a, -0.35 * hold)
    # Below half a second of headway, pull toward −1. At 0.6 s this weight is 0.
    short = _smooth01((0.50 - hw) / 0.20)
    if short > 0.0:
      deep = (-1.0 * short) + (a * (1.0 - short))
      a = min(a, deep)
  return a


def _a_out(slack: float, v_close: float) -> float:
  """Stopping accel outside the setpoint. Soft until the gap is actually short.

  Raw -v²/(2·slack) is the arrival bound. Far away it is capped near −0.55
  so a long close does not demand full regen; the cap fades as time-to-slack
  or the remaining gap gets short, which is when physics actually requires it.
  """
  v = max(0.0, float(v_close))
  if v <= 0.0:
    return 0.0
  raw = -(v * v) / (2.0 * max(float(slack), PHYS_EPS_M))
  ttc = max(float(slack), 0.0) / v
  urgent = max(
    _smooth01((3.2 - ttc) / 2.2),
    _smooth01((5.0 - float(slack)) / 4.0),
  )
  soft = raw if raw > -0.55 else -0.55
  hard = raw if raw > A_MIN_MS2 else A_MIN_MS2
  return ((1.0 - urgent) * soft) + (urgent * hard)


def _a_kin(slack: float, v_close: float, gap: float, v_ego: float,
           t_follow: float, v_lead: float) -> float:
  """Continuous kinematic accel. Blends into inside recovery as slack goes negative."""
  v = max(0.0, float(v_close))
  a_out = _a_out(slack, v)
  a_in = _inside_recovery(min(float(slack), 0.0), v, gap, v_ego, t_follow, v_lead)
  w = _smooth01((float(slack) + INSIDE_BLEND_M) / INSIDE_BLEND_M)
  return (w * a_out) + ((1.0 - w) * a_in)


def _depart_fade(y_rel: float, curvature: float, gap: float) -> float:
  """0 on the lane, 1 when the lead has left the path. Curve geometry is removed."""
  lane = abs(0.5 * float(curvature) * float(gap) * float(gap))
  y = max(0.0, abs(float(y_rel)) - lane)
  return _smooth01((y - DEPART_Y0_M) / max(1e-3, DEPART_Y1_M - DEPART_Y0_M))


def unified_follow_desired(gap: float, v_ego: float, v_lead: float, a_lead: float,
                           t_follow: float, *, v_ceiling: float | None = None,
                           a_map: float | None = None, y_rel: float = 0.0,
                           curvature: float = 0.0) -> float:
  """Unslewed follow accel from filtered lead signals. Continuous in its inputs."""
  gap_f = max(0.0, float(gap))
  v_l = max(0.0, float(v_lead))
  v_e = max(0.0, float(v_ego))
  a_l = float(a_lead)
  gap_set = gap_set_m(v_l, t_follow)
  slack = gap_f - gap_set
  v_err = v_l - v_e  # lead faster is positive, matching k_v(v_lead - v_ego)
  v_close = max(0.0, -v_err)

  sev = _severity(slack, v_err)
  k_g = K_G_LO + (K_G_HI - K_G_LO) * sev
  # High severity uses the smaller velocity gain (K_V_HI < K_V_LO).
  k_v = K_V_LO + (K_V_HI - K_V_LO) * sev
  k_a = _k_a(slack)
  glide = _glide(slack, v_err, a_l)
  keep = 1.0 - GLIDE_KEEP * glide
  a_gv = (k_g * slack + k_v * v_err) * keep
  a_pd = a_gv + k_a * a_l

  a_kin = _a_kin(slack, v_close, gap_f, v_e, t_follow, v_l)
  # Braking-lead contribution on the bound. Positive a_lead stays in a_pd only.
  a_bound = a_kin + min(a_l, 0.0) * k_a
  gate = _smooth01(v_close / 0.40) if v_close > 0.0 else 0.0
  limited = min(a_pd, a_bound)
  a_out_cmd = ((1.0 - gate) * a_pd) + (gate * limited)

  # Inside the setpoint, recovery (and a firm a_lead) owns the command.
  # Blended with the outside equation so slack = 0 does not step.
  a_in = _inside_recovery(min(slack, 0.0), v_close, gap_f, v_e, t_follow, v_l)
  if a_l < 0.0:
    a_in = min(a_in, k_a * a_l)
  w_out = _smooth01((slack + INSIDE_BLEND_M) / INSIDE_BLEND_M)
  a_cmd = (w_out * a_out_cmd) + ((1.0 - w_out) * a_in)

  # Far lead: cap ordinary depth near −1 unless kinematics are already deeper.
  # Weight grows only past ~15 m of slack, so a near-gap firm lead is not lifted.
  if a_kin > COMFORT_FAR_MS2:
    far = _smooth01((slack - 15.0) / 22.0)
    if far > 0.0:
      capped = a_cmd if a_cmd > COMFORT_FAR_MS2 else COMFORT_FAR_MS2
      a_cmd = ((1.0 - far) * a_cmd) + (far * capped)

  fade = _depart_fade(y_rel, curvature, gap_f)
  if fade > 0.0:
    a_cmd *= (1.0 - fade)

  if v_ceiling is not None and float(v_ceiling) > 0.5:
    a_cmd = min(a_cmd, (float(v_ceiling) - v_e) / SPEED_CEILING_TAU_S)
  if a_map is not None:
    a_cmd = min(a_cmd, float(a_map))
  if a_cmd < A_MIN_MS2:
    a_cmd = A_MIN_MS2
  elif a_cmd > A_MAX_MS2:
    a_cmd = A_MAX_MS2
  return a_cmd


def _limit_jerk(prev: float, target: float, dt: float, *, down: float, up: float) -> float:
  delta = float(target) - float(prev)
  if delta > up:
    return float(prev) + up
  if delta < -down:
    return float(prev) - down
  return float(target)


class UnifiedLeadController:
  """Stateful filters, cut-in blend, and jerk limit around unified_follow_desired."""

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self._v_f: float | None = None
    self._a_f: float | None = None
    self._prev: float | None = None
    self._lead_id = None
    self._cutin_w = 1.0
    self._cutin_from = 0.0
    self.last_desired = 0.0
    self.last_v_lead = 0.0
    self.last_a_lead = 0.0

  def step(self, *, dt: float, present: bool, gap: float, v_ego: float,
           v_lead: float, a_lead: float, t_follow: float, lead_id,
           seed_a: float = 0.0, v_ceiling: float | None = None,
           a_map: float | None = None, y_rel: float = 0.0,
           curvature: float = 0.0, a_max: float | None = None) -> float:
    """One planner frame. Returns the slewed road-relative accel, or 0 with no lead."""
    frame_dt = 0.05 if dt is None or float(dt) <= 1e-6 else float(dt)
    if not present:
      self.reset()
      return 0.0

    if lead_id != self._lead_id:
      self._cutin_from = float(seed_a if self._prev is None else self._prev)
      self._cutin_w = 0.0
      self._lead_id = lead_id
      # Do not inherit the previous lead's accel. Speed starts at this sample.
      self._v_f = float(v_lead)
      self._a_f = 0.0
      if self._prev is None:
        self._prev = float(seed_a)

    self._v_f = _filt(self._v_f, float(v_lead), frame_dt, TAU_V_S)
    self._a_f = _filt(self._a_f, float(a_lead), frame_dt, TAU_A_S)
    self.last_v_lead = float(self._v_f)
    self.last_a_lead = float(self._a_f)

    desired = unified_follow_desired(
      gap, v_ego, self._v_f, self._a_f, t_follow,
      v_ceiling=v_ceiling, a_map=a_map, y_rel=y_rel, curvature=curvature,
    )
    if a_max is not None:
      desired = min(desired, float(a_max))
    self.last_desired = desired

    self._cutin_w = min(1.0, self._cutin_w + frame_dt / CUTIN_S)
    target = ((1.0 - self._cutin_w) * self._cutin_from) + (self._cutin_w * desired)

    prev = float(self._prev if self._prev is not None else seed_a)
    v_err = float(self._v_f) - float(v_ego)
    slack = float(gap) - gap_set_m(self._v_f, t_follow)
    opening = v_err > 0.25 and slack > 1.0
    fade = _depart_fade(y_rel, curvature, gap)
    up = (JERK_RELEASE_OPEN_MS3 if (opening or fade > 0.35) else JERK_RELEASE_MS3) * frame_dt
    # Faster build only when the kinematic bound itself is deeper than now.
    a_kin = _a_kin(slack, max(0.0, -v_err), max(0.0, float(gap)), float(v_ego),
                   t_follow, float(self._v_f))
    deepen_fast = a_kin < prev - 0.08
    down = (JERK_PHYS_MS3 if deepen_fast else JERK_BUILD_MS3) * frame_dt
    nxt = _limit_jerk(prev, target, frame_dt, down=down, up=up)
    self._prev = nxt
    return nxt
