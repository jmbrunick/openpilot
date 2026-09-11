"""Catch a slower radar lead onto the selected Follow Distance.

Overlay peak is 0.80 m/s² (map Normal brake), never harder. MPC 2.5 m/s²
still owns close-in / stopping if needed.

Keep closing until Follow Distance. Do not match speed far back (map-style
road kinematics plus 110 m did that: the lead moves, the gap barely closes,
and the car hangs at Bosch range).

A fast close starts early enough that the remaining slack still fits 0.80 —
not a token tap that arrives hot. A small speed difference starts lighter.
Either way a = -v_rel² / (2 * slack) so extra speed is bled while the gap
shrinks onto t_follow, not while holding outside it.
"""
from __future__ import annotations

# Keep in sync with long_mpc.STOP_DISTANCE (acados cruise/lead obstacle).
STOP_DISTANCE = 6.0
# Map Normal brake. Gentler than overlay 1.0 and MPC 2.5. Do not raise.
LEAD_APPROACH_A_MS2 = 0.80
# Small-delta light open: seconds of closing-speed on top of the 0.80 catch.
LEAD_APPROACH_HEADSTART_S = 12.0
# Fast-close floor: open at this multiple of the 0.80 catch distance so a
# rapid close is a real ease (a_open ≈ 0.32), not a last-second 0.80 slam.
LEAD_APPROACH_FAST_FACTOR = 2.5
LEAD_APPROACH_DV_MS = 0.5  # ~1 mph; ignore radar jitter
NAP_T_FOLLOW = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)


def nap_t_follow(nap_follow_dist: int | None) -> float | None:
  if nap_follow_dist in range(1, len(NAP_T_FOLLOW) + 1):
    return NAP_T_FOLLOW[nap_follow_dist - 1]
  return None


def lead_approach_rel_need_m(v_ego, v_lead, a_comfort=LEAD_APPROACH_A_MS2) -> float:
  """Slack (m) needed to match speed at 0.80 without arriving hot."""
  v_rel = float(v_ego) - max(0.0, float(v_lead))
  if v_rel <= 0.0 or a_comfort <= 0:
    return 0.0
  return (v_rel * v_rel) / (2.0 * float(a_comfort))


def lead_approach_need_m(v_ego, v_lead, a_comfort=LEAD_APPROACH_A_MS2, t_follow=None) -> float:
  """Meters of slack (gap above Follow Distance) at which the ease starts.

  max(fast-close floor, light small-delta head-start). t_follow is accepted
  for callers; start distance is not capped at 140 m — that delay made a
  rapid close wait and arrive hot.
  """
  v_rel = float(v_ego) - max(0.0, float(v_lead))
  if v_rel <= 0.0 or a_comfort <= 0:
    return 0.0
  rel_need = lead_approach_rel_need_m(v_ego, v_lead, a_comfort)
  light_m = rel_need + v_rel * LEAD_APPROACH_HEADSTART_S
  fast_m = LEAD_APPROACH_FAST_FACTOR * rel_need
  # t_follow is part of the public signature (planner / tests). Start distance
  # is not capped by Follow Distance or 140 m — that delay arrived hot.
  _ = t_follow
  return max(light_m, fast_m)


def lead_approach_decel_ms2(v_ego, v_lead, d_rel, t_follow, a_comfort=LEAD_APPROACH_A_MS2):
  """Comfort decel to close onto the Follow Distance gap, or None.

  a = -v_rel² / (2 * slack) so we arrive at the selected gap with matching
  speed. Fast closes open early enough for 0.80 to finish; small deltas open
  lighter. None when speeds match, the lead is faster, or the lead is still
  outside the window (no crawl / no hang at radar range).
  """
  if t_follow is None or float(t_follow) <= 0 or a_comfort <= 0:
    return None
  if d_rel is None or float(d_rel) <= 0:
    return None
  v0 = float(v_ego)
  vt = max(0.0, float(v_lead))
  v_rel = v0 - vt
  if v_rel < LEAD_APPROACH_DV_MS:
    return None
  d_follow = float(t_follow) * vt + STOP_DISTANCE
  slack = float(d_rel) - d_follow
  if slack <= 1.0:
    return None
  need_m = lead_approach_need_m(v0, vt, a_comfort, t_follow)
  if slack > need_m:
    return None
  a_needed = -(v_rel * v_rel) / (2.0 * slack)
  return max(float(a_needed), -float(a_comfort))
