"""Earlier, gentler catch-up to a slower radar lead.

Comfort peak is Early map brake (0.55 m/s²), not Normal 0.80. MPC 2.5 m/s²
still owns close-in / stopping if needed. FCW is untouched.

Map drops use road-distance kinematics plus 110 m because the sign does not
move. A radar lead does: (v_lead² - v_ego²) / (2 * slack) matches speed after
ego travels `slack` meters of road, while the lead moves almost as far, so the
gap barely closes. That hang sits at radar range and pulses as the track
comes and goes.

This uses relative kinematics so extra speed is bled while closing onto the
selected Follow Distance (t_follow), not while holding outside radar.

On a slight grade, radar `v_rel` / slack chatter around the follow gap used
to snap this overlay on/off (regen bite → Accel-1 crawl → bite). Enter/exit
hysteresis plus a per-frame slew on more-negative `a` hold a steady ease
instead of chattering. Off / milder `a` is immediate so rematch is not stuck
in regen.

Positive close-the-gap accel is a separate cap (`lead_close_accel_ms2`).
Map Accel 1–10 used to gate only MAX-rise climb; Adaptive Accel used the
full cruise profile (1.6–0.6) when the gap was large. That is the punch.
"""
from __future__ import annotations

from openpilot.selfdrive.mapd.constants import accel_scale_factor

# Keep in sync with long_mpc.STOP_DISTANCE (acados cruise/lead obstacle).
STOP_DISTANCE = 6.0
# Comfort peak |a|. Early map brake, not Normal 0.80 — Tesla VirtualDAS
# regen at 0.80 then Accel-1 rematch chatters on a slight grade. Do not
# raise. MPC 2.5 / FCW still own danger.
LEAD_APPROACH_A_MS2 = 0.55
# Seconds of current closing-speed added before the last-second catch.
# 12 s vs 8 s: ~18 m / ~4 s earlier on a 10 mph close, lighter a at the open,
# same comfort peak near Follow Distance. Still inside radar.
LEAD_APPROACH_HEADSTART_S = 12.0
# Bosch-range ceiling so we do not open on a flickering 160 m track.
LEAD_APPROACH_MAX_START_M = 140.0

# Enter / exit (hysteresis). A single v_rel / slack gate chatters around
# the follow gap on a slight incline (regen ↔ Accel-1).
LEAD_APPROACH_DV_MS = 0.5          # enter: ~1 mph closing; ignore radar jitter
LEAD_APPROACH_DV_OFF_MS = 0.20     # exit: ~0.45 mph; hold through ±0.15 noise
LEAD_APPROACH_SLACK_ON_M = 1.0     # enter only with slack above Follow Distance
LEAD_APPROACH_SLACK_OFF_M = 0.0    # stay until at/inside the follow gap
LEAD_APPROACH_NEED_HOLD_M = 4.0    # extra slack (m) before dropping after open

# Gradual regen onset (planner frame). Same step as accel_clip slew.
# At DT_MDL=0.05 s → 1.0 m/s²/s. Release / milder a is immediate.
LEAD_APPROACH_SLEW_MS2 = 0.05

NAP_T_FOLLOW = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)

# Max +a when coming up behind a radar lead (gap close / catch-up).
# Accel 5 → 0.30; Accel 1 → 0.20; Accel 10 → 0.50. Cruise get_max_accel is
# 1.6–0.6; do not raise the min. Does not change MPC danger / hard brake.
LEAD_CLOSE_A_BASE_MS2 = 0.30
LEAD_CLOSE_A_MIN_MS2 = 0.20
LEAD_CLOSE_A_MAX_MS2 = 0.50


def nap_t_follow(nap_follow_dist: int | None) -> float | None:
  if nap_follow_dist in range(1, len(NAP_T_FOLLOW) + 1):
    return NAP_T_FOLLOW[nap_follow_dist - 1]
  return None


def lead_close_accel_ms2(accel_level: int = 5) -> float:
  """Max positive a (m/s²) when closing the gap on a radar lead.

  Accel 1–10 scales this. Separate from map MAX-rise climb (0.36–1.60)
  and from lead_approach decel (0.55). MPC −a / danger is unchanged.
  """
  a = LEAD_CLOSE_A_BASE_MS2 * accel_scale_factor(int(accel_level))
  return max(LEAD_CLOSE_A_MIN_MS2, min(LEAD_CLOSE_A_MAX_MS2, a))


def lead_close_should_cap(d_rel) -> bool:
  """True when a radar lead is in the close-cap window (not a 160 m flicker)."""
  if d_rel is None:
    return False
  d = float(d_rel)
  return 0.0 < d <= LEAD_APPROACH_MAX_START_M


def lead_approach_need_m(v_ego, v_lead, a_comfort=LEAD_APPROACH_A_MS2, t_follow=None) -> float:
  """Meters of slack (gap above Follow Distance) at which the ease starts."""
  v_rel = float(v_ego) - max(0.0, float(v_lead))
  vt = max(0.0, float(v_lead))
  if v_rel <= 0.0 or a_comfort <= 0:
    return 0.0
  # Relative comfort catch, plus head-start so we begin gently — not map's +110 m.
  need = (v_rel * v_rel) / (2.0 * float(a_comfort)) + v_rel * LEAD_APPROACH_HEADSTART_S
  if t_follow is not None and float(t_follow) > 0:
    d_follow = float(t_follow) * vt + STOP_DISTANCE
    need = min(need, max(0.0, LEAD_APPROACH_MAX_START_M - d_follow))
  return need


def slew_lead_approach_a(target, prev, slew=LEAD_APPROACH_SLEW_MS2):
  """Ramp more-negative overlay a. Immediate milder / off.

  Onset is the Tesla regen bite. Off / less brake must not stay latched
  in regen after speeds match or slack is gone (Accel 1 rematch).
  """
  if target is None:
    return None
  t = float(target)
  p = 0.0 if prev is None else float(prev)
  if t < p:
    return max(t, p - float(slew))
  return t


def lead_approach_decel_ms2(v_ego, v_lead, d_rel, t_follow, a_comfort=LEAD_APPROACH_A_MS2,
                           active=False):
  """Comfort decel to close onto the Follow Distance gap, or None.

  a = -v_rel² / (2 * slack) so we arrive at the selected gap with matching
  speed. |a| at the open is below the comfort peak and only reaches that
  peak near the gap. None when speeds match, the lead is faster, or the
  lead is still outside the window (no crawl / no radar-edge hang).

  `active` is last frame's overlay (hysteresis). Enter uses DV_MS / SLACK_ON;
  hold uses DV_OFF / SLACK_OFF / need+NEED_HOLD so small radar noise does
  not chatter regen ↔ accel.
  """
  if t_follow is None or float(t_follow) <= 0 or a_comfort <= 0:
    return None
  if d_rel is None or float(d_rel) <= 0:
    return None
  v0 = float(v_ego)
  vt = max(0.0, float(v_lead))
  v_rel = v0 - vt
  dv_gate = LEAD_APPROACH_DV_OFF_MS if active else LEAD_APPROACH_DV_MS
  if v_rel < dv_gate:
    return None
  d_follow = float(t_follow) * vt + STOP_DISTANCE
  slack = float(d_rel) - d_follow
  slack_min = LEAD_APPROACH_SLACK_OFF_M if active else LEAD_APPROACH_SLACK_ON_M
  if slack <= slack_min:
    return None
  need_m = lead_approach_need_m(v0, vt, a_comfort, t_follow)
  need_gate = need_m + (LEAD_APPROACH_NEED_HOLD_M if active else 0.0)
  if slack > need_gate:
    return None
  a_needed = -(v_rel * v_rel) / (2.0 * slack)
  return max(float(a_needed), -float(a_comfort))
