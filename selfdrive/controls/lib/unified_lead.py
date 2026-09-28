"""Continuous lead-follow controller.

One equation replaces the stacked follow overrides when NAPLongUnified is on.
The planner still runs MPC. This module is a few arithmetic ops per frame.

  a_cmd = k_g·e + k_v(v_lead - v_ego) + k_a·a_lead,   bounded by a_req

e = gap - gap_set is one signed gap error. gap_set is the selected follow
gap (t_follow·v_lead + standstill). There is no separate inside-gap or
outside-gap formula: every term is a smooth function of e, so nothing steps
at the follow distance.

The kinematic required decel

  a_req = -v_close² / (2·D(e))

is applied while closing (Sep 25 10:07:59 / 10:10:13). D(e) is the room to
match speed: outside the gap the gap error shortened with closing speed,
plus the part of the follow gap we may eat into (it fades in over the last
few seconds of approach and shrinks as e goes negative), minus a short
reaction distance. Slow closes give small values; a fast close from 60 m is
firm straight away; a small miss at the gap edge is not a -3.5 spike
(Sep 25 09:21:44). There is no far soft cap and no far comfort cap.

A pulling-away lead near the gap gets a trickle, not a catch-up lunge
(Sep 25 09:47 / 10:54). Speed ceilings (MAX, curve) are a speed-error term:
never accelerate above the cap, and above it start near the EV's mild
-0.22 m/s² settle, firmer only when well over (Sep 25 10:43:56).

A lead is weighted by a continuous confidence: consistent, on-path, radar
or confident-vision readings ramp to full weight; a fast close ramps within
a fraction of a second; a one-frame blip barely moves the command. A new
lead (cut-in) ramps in the same way from the command already in force.

A jerk limit slews the result. Build and release rates grow with how far
the command is from the target: fast when far too firm, gentle when close.

Driver-likeness (Justin's fingerprint, Sep 20–27 qlogs):
- A lead that starts braking is anticipated: its filtered decel onset rate
  extends a_lead ahead by up to 0.8 m/s², weighted continuously by how hard
  it already brakes, how short the headway is, and how fast the decel builds.
- Mild lead slowing is mirrored under 1:1 (0.75×) so the ease stays gradual;
  real braking is over-matched (1.10×). One smooth blend, no threshold.
- Inside the gap the glide keeps a small ease until the gap recovers.
- A far, slow close coasts: the closing gain fades with time-to-gap 12→30 s.
"""
from __future__ import annotations

import math

from openpilot.selfdrive.controls.lib.lead_leaving import CLEAR_MARGIN_M, EGO_HALF_WIDTH_M, LEAD_HALF_WIDTH_M

# Same standstill gap long_mpc uses for the follow obstacle.
STOP_DISTANCE_M = 6.0

# Hard plant bounds. Full regen is ACCEL_MIN.
A_MIN_MS2 = -3.5
A_MAX_MS2 = 2.0

# Gap-error gain. Severity in [0, 1] interpolates low → high.
K_G_LO = 0.012  # 1/s²
K_G_HI = 0.030
# Relative-speed gain while closing. Small closes land near mild regen.
K_V_LO = 0.16   # 1/s
K_V_HI = 0.10   # Large closes let a_lead and a_req own the depth.
# a_lead weight: 1 next to the gap, easing off when the lead is still far.
K_A_NEAR = 1.10  # slightly over 1: a closing ego must shed more than the lead brakes
K_A_FAR = 0.45
K_A_NEAR_M = 18.0
K_A_FAR_M = 70.0
# The a_lead weight also grows with how hard the lead brakes: mild slowing is
# under-matched (a gradual, matched ease), real braking over-matched.
K_A_MILD = 0.75
K_A_FIRM = 1.10
K_A_FIRM_A0_MS2 = 0.30
K_A_FIRM_BAND_MS2 = 0.50
# Far, slow closes coast: the closing gain fades with time-to-gap.
K_V_FAR_TTG_LO_S = 12.0
K_V_FAR_TTG_HI_S = 30.0
K_V_FAR_MIN = 0.40

GAP_SEV_M = 16.0
CLOSE_SEV_MS = 3.2

# Glide: nearly matched at the setpoint contributes nothing you can feel.
GLIDE_GAP_M = 3.5
GLIDE_V_MS = 0.40
GLIDE_KEEP = 0.90  # fraction of the gap/speed term removed at perfect glide
# Inside the gap the glide removes less, so a small ease persists until the
# gap recovers instead of holding speed a few meters too close.
GLIDE_KEEP_INSIDE = 0.60

# Kinematic room D(e): outside the gap the gap error shortened with closing
# speed (e / (1 + v/V0)); at and inside the setpoint a share of the follow
# gap is intrusion room that fades in over the last seconds of approach and
# shrinks one-for-one as e goes negative, down to a floor. Minus reaction.
ARRIVE_V0_MS = 8.0  # outside room = e / (1 + v_close / V0)
ARRIVE_SOFT_M = 2.0
INSIDE_ALLOW_FRAC = 0.65  # of min(t_follow, cap)·v_lead
INSIDE_ALLOW_TF_MAX_S = 1.0  # follow time beyond ~1 s is reserve, not room
ALLOW_FADE_TTG_S = 4.0  # intrusion room fades in over the last seconds to the setpoint
ALLOW_FADE_V_MIN_MS = 0.3
INSIDE_ALLOW_MIN_M = 3.0
REACTION_S = 0.3
KIN_ROOM_MIN_M = 1.5
# Closing speed below this contributes no kinematic bound (fades in).
KIN_GATE_MS = 0.40

# Short headway: pull firmer below ~0.6 s even at small closing.
SHORT_HW_S = 0.60
SHORT_HW_BAND_S = 0.30
SHORT_HW_A = 0.60

# Pulling away near the gap: positive command compresses to a trickle.
# The compression fades out once the slack is large (real catch-up).
TRICKLE_A = 0.12
TRICKLE_SCALE = 0.15
TRICKLE_FAR_LO_M = 25.0
TRICKLE_FAR_HI_M = 60.0
TRICKLE_IN_M = 6.0
# Opening lead: the relative-speed gain is small near the gap and grows
# with slack, so a lead pulling away is not chased.
K_V_OPEN_NEAR = 0.07
K_V_OPEN_FAR = 0.16
OPEN_INSIDE_M = 6.0  # inside by this much: opening uses the closing gain

# Speed ceiling (MAX, curve). Below the cap: a <= (v_cap - v)/tau (0 at cap).
# Above it: start at the EV mild settle, firmer only when well over.
SPEED_CEILING_TAU_S = 4.0
OVERSPEED_SETTLE_MS2 = 0.22
OVERSPEED_RAMP_MS = 0.45
OVERSPEED_K = 0.12       # m/s² per m/s beyond the ramp
OVERSPEED_MAX_MS2 = 1.20  # soft saturation

# Filters. One 50 ms blip moves a_lead by ~1/4 of the spike.
TAU_V_S = 0.20
TAU_A_S = 0.15

# Lead-decel anticipation: a_eff = a_lead - min(MAX, T·max(0, -da_lead/dt)·w).
# w grows continuously with the lead's decel (A0/BAND), short headway
# (HW/HW_BAND) and the decel onset rate (J0/JBAND). A steady lead adds nothing.
LEAD_ANTICIPATE_T_S = 2.2
LEAD_ANTICIPATE_MAX_MS2 = 0.80
LEAD_JERK_TAU_S = 0.25
LEAD_ANTICIPATE_A0_MS2 = 0.10
LEAD_ANTICIPATE_ABAND_MS2 = 0.35
LEAD_ANTICIPATE_HW_S = 2.5
LEAD_ANTICIPATE_HW_BAND_S = 1.0
LEAD_ANTICIPATE_J0_MS3 = 0.20
LEAD_ANTICIPATE_JBAND_MS3 = 0.40

# Jerk. Rates grow with the distance between command and target.
JERK_RELEASE_MS3 = 0.45
JERK_RELEASE_OPEN_MS3 = 1.35
JERK_BUILD_MS3 = 1.15
JERK_PHYS_MS3 = 2.50
JERK_PROP_1_S = 2.0  # m/s³ extra per m/s² of error
UNIFIED_JERK_LIMIT_MS3 = JERK_PHYS_MS3

# Lead confidence. dw/dt = (R0 + R1·urgency·w)·(target - w) while rising.
CONF_RATE_BASE_1_S = 2.0
CONF_RATE_URGENT_1_S = 14.0
CONF_RATE_FALL_1_S = 4.0
CONF_URGENT_LO_MS2 = 0.5
CONF_URGENT_BAND_MS2 = 1.5
CONF_VISION_MIN = 0.35
# Kept for callers that referenced the old fixed cut-in blend.
CUTIN_S = 0.45
# Toggle engage/disengage blend lives in the planner (~1 s).
MODE_BLEND_S = 1.0

# Static lateral fade from the predicted travel path. It starts only where
# a lead that is not moving sideways already clears our body: our half-width
# + its half-width + margin (lead_leaving). A stalled half-out lead keeps
# full weight. A lead that is *moving* out is released earlier, and only
# when its projection clears us at our arrival, by the planner's leaving
# weight (`leave_w`), combined with this fade below.
DEPART_Y0_M = EGO_HALF_WIDTH_M + LEAD_HALF_WIDTH_M + CLEAR_MARGIN_M
DEPART_Y1_M = DEPART_Y0_M + 0.7


def _smooth01(x: float) -> float:
  """Smoothstep on [0, 1]. Constant outside. Value and slope match at the ends."""
  if x <= 0.0:
    return 0.0
  if x >= 1.0:
    return 1.0
  return x * x * (3.0 - 2.0 * x)


def _softplus(x: float, scale: float) -> float:
  """Smooth max(x, 0). Equals x well above 0, 0 well below."""
  s = max(1e-6, float(scale))
  z = float(x) / s
  if z > 30.0:
    return float(x)
  return s * math.log1p(math.exp(z))


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


def _k_a_brake_scale(a_lead: float) -> float:
  """0.75× for mild lead slowing, 1.10× for real braking, smooth in between."""
  t = _smooth01((-float(a_lead) - K_A_FIRM_A0_MS2) / K_A_FIRM_BAND_MS2)
  return K_A_MILD + (K_A_FIRM - K_A_MILD) * t


def lead_decel_anticipation(a_lead: float, lead_jerk: float, gap: float, v_ego: float) -> float:
  """Extra decel (>= 0) ahead of a lead whose braking is still building.

  Continuous in every input: 0 for a steady or recovering lead, growing with
  how hard the lead already brakes, how short the headway is, and how fast
  its decel builds. Capped so a gradual slowdown cannot become a wall.
  """
  building = max(0.0, -float(lead_jerk))
  if building <= 0.0:
    return 0.0
  hw = float(gap) / max(float(v_ego), 1.0)
  w = _smooth01((-float(a_lead) - LEAD_ANTICIPATE_A0_MS2) / LEAD_ANTICIPATE_ABAND_MS2)
  w *= _smooth01((LEAD_ANTICIPATE_HW_S - hw) / LEAD_ANTICIPATE_HW_BAND_S)
  w *= _smooth01((building - LEAD_ANTICIPATE_J0_MS3) / LEAD_ANTICIPATE_JBAND_MS3)
  return min(LEAD_ANTICIPATE_MAX_MS2, LEAD_ANTICIPATE_T_S * building * w)


def _glide(slack: float, v_err: float, a_lead: float) -> float:
  """1 when matched and the lead is not braking. A real lead brake kills it."""
  g = math.exp(-((float(slack) / GLIDE_GAP_M) ** 2) - ((float(v_err) / GLIDE_V_MS) ** 2))
  brake = min(float(a_lead), 0.0)
  g *= math.exp(-((brake / 0.30) ** 2))
  return g


def kinematic_room_m(slack: float, v_close: float, t_follow: float, v_lead: float) -> float:
  """Room to match speed. One smooth function of the signed gap error.

  Outside the gap the room is the gap error itself, shortened as the
  closing speed grows (plan to be matched earlier on a fast close). At and
  inside the setpoint part of the follow gap is usable intrusion room; it
  fades in across the last few meters of approach and shrinks one-for-one
  as the gap error goes negative, down to a floor.
  """
  e = float(slack)
  v = max(0.0, float(v_close))
  outside = _softplus(e, ARRIVE_SOFT_M) / (1.0 + v / ARRIVE_V0_MS)
  tf_room = min(float(t_follow), INSIDE_ALLOW_TF_MAX_S)
  allow = max(INSIDE_ALLOW_MIN_M, INSIDE_ALLOW_FRAC * tf_room * max(0.0, float(v_lead)))
  # min(e, 0) has a kink at 0 but its value is continuous.
  inside = _softplus(allow + min(e, 0.0) - INSIDE_ALLOW_MIN_M, 1.0) + INSIDE_ALLOW_MIN_M
  # The intrusion room fades in over the last ALLOW_FADE_TTG_S of approach
  # (time to reach the setpoint at the current closing speed).
  ttg = e / max(v, ALLOW_FADE_V_MIN_MS)
  inside *= 1.0 - _smooth01(ttg / ALLOW_FADE_TTG_S)
  room = outside + inside - REACTION_S * v
  return max(KIN_ROOM_MIN_M, room)


def kinematic_required_accel(slack: float, v_close: float, t_follow: float, v_lead: float) -> float:
  """a_req = -v_close² / (2·D). 0 when not closing."""
  v = max(0.0, float(v_close))
  if v <= 0.0:
    return 0.0
  room = kinematic_room_m(slack, v, t_follow, v_lead)
  return max(A_MIN_MS2, -(v * v) / (2.0 * room))


def _a_kin(slack: float, v_close: float, gap: float, v_ego: float,
           t_follow: float, v_lead: float) -> float:
  """Kinematic bound used by the build-rate choice. Same term as the command."""
  _ = gap, v_ego
  return kinematic_required_accel(slack, v_close, t_follow, v_lead)


def _short_headway(gap: float, v_ego: float, v_close: float) -> float:
  """Firmer pull below ~0.6 s headway unless the lead is pulling away."""
  hw = float(gap) / max(float(v_ego), 1.0)
  w = _smooth01((SHORT_HW_S - hw) / SHORT_HW_BAND_S)
  if w <= 0.0:
    return 0.0
  closing_w = _smooth01((float(v_close) + 0.5) / 1.0)
  return -SHORT_HW_A * w * closing_w


def _depart_fade(y_rel: float, curvature: float, gap: float,
                 path_lat: float | None = None) -> float:
  """0 on the travel path, 1 when the lead has left it.

  `path_lat` is the lateral offset from the model's predicted path at the
  lead's distance, so a lead in a bend stays on-path and an adjacent-lane
  car is off-path. Without a path, the curve geometry is removed from yRel.
  """
  if path_lat is not None and math.isfinite(float(path_lat)):
    y = abs(float(path_lat))
  else:
    lane = abs(0.5 * float(curvature) * float(gap) * float(gap))
    y = max(0.0, abs(float(y_rel)) - lane)
  return _smooth01((y - DEPART_Y0_M) / max(1e-3, DEPART_Y1_M - DEPART_Y0_M))


def _leave_fade(y_rel: float, curvature: float, gap: float,
                path_lat: float | None = None, leave_w: float = 0.0) -> float:
  """Static off-path fade combined with the planner's leaving weight."""
  static = _depart_fade(y_rel, curvature, gap, path_lat)
  w = 0.0 if leave_w is None or not math.isfinite(float(leave_w)) else min(1.0, max(0.0, float(leave_w)))
  return 1.0 - (1.0 - static) * (1.0 - w)


def speed_ceiling_accel(v_ego: float, v_cap: float) -> float:
  """Speed-error term for MAX / curve caps.

  Below the cap it only limits acceleration (0 at the cap). Above it the
  decel starts at the mild settle and grows with overspeed, saturating
  softly well short of full regen.
  """
  over = float(v_ego) - float(v_cap)
  if over <= 0.0:
    return -over / SPEED_CEILING_TAU_S
  mild = OVERSPEED_SETTLE_MS2 * _smooth01(over / OVERSPEED_RAMP_MS)
  extra = OVERSPEED_K * max(0.0, over - OVERSPEED_RAMP_MS)
  raw = mild + extra
  return -OVERSPEED_MAX_MS2 * math.tanh(raw / OVERSPEED_MAX_MS2)


def unified_follow_desired(gap: float, v_ego: float, v_lead: float, a_lead: float,
                           t_follow: float, *, v_ceiling: float | None = None,
                           a_map: float | None = None, y_rel: float = 0.0,
                           curvature: float = 0.0,
                           path_lat: float | None = None,
                           leave_w: float = 0.0) -> float:
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
  k_v_close = K_V_LO + (K_V_HI - K_V_LO) * sev
  # A far, slow close coasts: the closing gain fades with time-to-gap.
  ttg_far = slack / max(v_close, 0.3)
  k_v_close *= 1.0 - (1.0 - K_V_FAR_MIN) * _smooth01(
    (ttg_far - K_V_FAR_TTG_LO_S) / max(1e-3, K_V_FAR_TTG_HI_S - K_V_FAR_TTG_LO_S))
  # Opening: small gain at/over the setpoint (a lead pulling away is not
  # chased), growing with slack. Inside the gap the opening speed keeps the
  # closing gain so braking unwinds as soon as the gap starts recovering.
  open_far = _smooth01((slack - 5.0) / 30.0)
  k_v_open = K_V_OPEN_NEAR + (K_V_OPEN_FAR - K_V_OPEN_NEAR) * open_far
  inside_w = _smooth01(-slack / OPEN_INSIDE_M)
  k_v_open = inside_w * k_v_close + (1.0 - inside_w) * k_v_open
  # Blend the two across v_err = 0 so the gain has no step.
  side = _smooth01((v_err + 0.3) / 0.6)
  k_v = (1.0 - side) * k_v_close + side * k_v_open
  k_a = _k_a(slack) * _k_a_brake_scale(a_l)
  glide = _glide(slack, v_err, a_l)
  glide_keep = GLIDE_KEEP + (GLIDE_KEEP_INSIDE - GLIDE_KEEP) * _smooth01(-slack / GLIDE_GAP_M)
  keep = 1.0 - glide_keep * glide
  a_gv = (k_g * slack + k_v * v_err) * keep
  a_gv += _short_headway(gap_f, v_e, v_close)
  a_pd = a_gv + k_a * a_l

  a_kin = _a_kin(slack, v_close, gap_f, v_e, t_follow, v_l)
  # Braking-lead contribution on the bound. Positive a_lead stays in a_pd only.
  a_bound = a_kin + min(a_l, 0.0) * k_a
  gate = _smooth01(v_close / KIN_GATE_MS) if v_close > 0.0 else 0.0
  limited = min(a_pd, a_bound)
  a_cmd = ((1.0 - gate) * a_pd) + (gate * limited)

  # Pulling away near the gap: trickle, not a catch-up lunge.
  if a_cmd > 0.0:
    # Compression weight rises across the setpoint (recovering from inside
    # is not compressed) and fades once the slack is a real catch-up.
    far = _smooth01((slack - TRICKLE_FAR_LO_M) / (TRICKLE_FAR_HI_M - TRICKLE_FAR_LO_M))
    w = _smooth01((slack + TRICKLE_IN_M) / TRICKLE_IN_M) * (1.0 - far)
    trickle = TRICKLE_A * math.tanh(a_cmd / TRICKLE_SCALE)
    a_cmd = w * min(a_cmd, trickle) + (1.0 - w) * a_cmd

  # Off-path / leaving lead: its command (brake included) fades toward 0;
  # the MAX / map / curve terms below still apply in full.
  fade = _leave_fade(y_rel, curvature, gap_f, path_lat, leave_w)
  if fade > 0.0:
    a_cmd *= (1.0 - fade)

  if v_ceiling is not None and float(v_ceiling) > 0.5:
    a_cmd = min(a_cmd, speed_ceiling_accel(v_e, float(v_ceiling)))
  if a_map is not None:
    a_cmd = min(a_cmd, float(a_map))
  if a_cmd < A_MIN_MS2:
    a_cmd = A_MIN_MS2
  elif a_cmd > A_MAX_MS2:
    a_cmd = A_MAX_MS2
  return a_cmd


def lead_quality(model_prob: float | None = None, radar: bool | None = None) -> float:
  """Per-frame reading quality in [0, 1]. Radar-associated reads full.

  Vision-only reads scale with model probability, never below a floor so
  a real vision lead still owns the command after it has been consistent.
  """
  if radar is True or model_prob is None:
    return 1.0
  p = float(model_prob)
  if not math.isfinite(p):
    return 1.0
  return max(CONF_VISION_MIN, min(1.0, p / 0.6))


def _limit_jerk(prev: float, target: float, dt: float, *, down: float, up: float) -> float:
  delta = float(target) - float(prev)
  if delta > up:
    return float(prev) + up
  if delta < -down:
    return float(prev) - down
  return float(target)


class UnifiedLeadController:
  """Stateful filters, lead confidence, and jerk limit around unified_follow_desired."""

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self._v_f: float | None = None
    self._a_f: float | None = None
    self._lead_jerk = 0.0
    self._prev: float | None = None
    self._lead_id = None
    self._conf = 1.0
    self._cutin_from = 0.0
    self.last_desired = 0.0
    self.last_v_lead = 0.0
    self.last_a_lead = 0.0
    self.last_a_lead_eff = 0.0
    self.last_confidence = 0.0

  @property
  def _cutin_w(self) -> float:
    return self._conf

  def step(self, *, dt: float, present: bool, gap: float, v_ego: float,
           v_lead: float, a_lead: float, t_follow: float, lead_id,
           seed_a: float = 0.0, v_ceiling: float | None = None,
           a_map: float | None = None, y_rel: float = 0.0,
           curvature: float = 0.0, a_max: float | None = None,
           path_lat: float | None = None, model_prob: float | None = None,
           radar: bool | None = None, leave_w: float = 0.0) -> float:
    """One planner frame. Returns the slewed road-relative accel, or 0 with no lead."""
    frame_dt = 0.05 if dt is None or float(dt) <= 1e-6 else float(dt)
    if not present:
      self.reset()
      return 0.0

    if lead_id != self._lead_id:
      # New lead or cut-in: blend in from the command already in force.
      self._cutin_from = float(seed_a if self._prev is None else self._prev)
      self._conf = 0.0
      self._lead_id = lead_id
      # Do not inherit the previous lead's accel. Speed starts at this sample.
      self._v_f = float(v_lead)
      self._a_f = 0.0
      self._lead_jerk = 0.0
      if self._prev is None:
        self._prev = float(seed_a)

    self._v_f = _filt(self._v_f, float(v_lead), frame_dt, TAU_V_S)
    a_prev = self._a_f
    self._a_f = _filt(self._a_f, float(a_lead), frame_dt, TAU_A_S)
    jerk_raw = (self._a_f - a_prev) / frame_dt if a_prev is not None else 0.0
    self._lead_jerk = _filt(self._lead_jerk, jerk_raw, frame_dt, LEAD_JERK_TAU_S)
    a_eff = self._a_f - lead_decel_anticipation(self._a_f, self._lead_jerk, gap, v_ego)
    self.last_v_lead = float(self._v_f)
    self.last_a_lead = float(self._a_f)
    self.last_a_lead_eff = float(a_eff)

    desired = unified_follow_desired(
      gap, v_ego, self._v_f, a_eff, t_follow,
      v_ceiling=v_ceiling, a_map=a_map, y_rel=y_rel, curvature=curvature,
      path_lat=path_lat, leave_w=leave_w,
    )
    if a_max is not None:
      desired = min(desired, float(a_max))
    self.last_desired = desired

    v_err = float(self._v_f) - float(v_ego)
    slack = float(gap) - gap_set_m(self._v_f, t_follow)
    a_kin = _a_kin(slack, max(0.0, -v_err), max(0.0, float(gap)), float(v_ego),
                   t_follow, float(self._v_f))

    # Confidence: consistent, on-path, good-quality readings ramp to 1.
    # A fast close (deep kinematic demand) ramps within a fraction of a
    # second; a one-frame reading barely moves it.
    # Confidence reads the static off-path geometry only. A leaving lead is
    # released through `desired` (leave_w above), toward the lead-free
    # command; a confidence drop would blend toward the command in force,
    # which is the lead's brake itself.
    fade = _leave_fade(y_rel, curvature, gap, path_lat, leave_w)
    on_path = 1.0 - _depart_fade(y_rel, curvature, gap, path_lat)
    target_conf = on_path * lead_quality(model_prob, radar)
    urgency = _smooth01((-a_kin - CONF_URGENT_LO_MS2) / CONF_URGENT_BAND_MS2)
    if target_conf >= self._conf:
      rate = CONF_RATE_BASE_1_S + CONF_RATE_URGENT_1_S * urgency * self._conf
    else:
      rate = CONF_RATE_FALL_1_S
    self._conf = self._conf + min(1.0, rate * frame_dt) * (target_conf - self._conf)
    self._conf = min(1.0, max(0.0, self._conf))
    self.last_confidence = float(self._conf)

    # Off-path leads already fade inside `desired`; confidence blends a new
    # or weak reading from the command in force before this lead.
    w = self._conf if lead_id == self._lead_id else 0.0
    target = ((1.0 - w) * self._cutin_from) + (w * desired)
    prev = float(self._prev if self._prev is not None else seed_a)

    opening = v_err > 0.25 and slack > 1.0
    release_base = JERK_RELEASE_OPEN_MS3 if (opening or fade > 0.35) else JERK_RELEASE_MS3
    err = float(target) - prev
    # Proportional: far too firm releases fast, near the target gently.
    up = min(JERK_PHYS_MS3, release_base + JERK_PROP_1_S * max(0.0, err)) * frame_dt
    build_base = JERK_PHYS_MS3 if a_kin < prev - 0.08 else JERK_BUILD_MS3
    down = min(JERK_PHYS_MS3, build_base + JERK_PROP_1_S * max(0.0, -err)) * frame_dt
    nxt = _limit_jerk(prev, target, frame_dt, down=down, up=up)
    self._prev = nxt
    # Once a lead is trusted, the blend-from value follows the published
    # command so a later confidence dip fades toward current behavior.
    self._cutin_from += (nxt - self._cutin_from) * self._conf * min(1.0, frame_dt / MODE_BLEND_S)
    return nxt
