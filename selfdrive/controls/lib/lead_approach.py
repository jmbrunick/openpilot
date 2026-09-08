"""Earlier, smoother catch-up to a slower radar lead.

MPC still owns close-in / stopping (COMFORT_BRAKE 2.5 m/s²). This overlay
starts a 1.0 m/s² ease when the remaining gap is inside kinematic d plus the
NAP Follow Distance time-gap, so we do not wait until late then brake hard.
Steady-state following distance is unchanged (t_follow from NAPFollowDistance).
"""
from __future__ import annotations

# Keep in sync with long_mpc.STOP_DISTANCE (acados cruise/lead obstacle).
STOP_DISTANCE = 6.0
# Gentler than MPC 2.5 m/s² so the catch starts farther back.
LEAD_APPROACH_A_MS2 = 1.0
LEAD_APPROACH_DV_MS = 0.5  # ~1 mph; ignore radar jitter
NAP_T_FOLLOW = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)


def nap_t_follow(nap_follow_dist: int | None) -> float | None:
  if nap_follow_dist in range(1, len(NAP_T_FOLLOW) + 1):
    return NAP_T_FOLLOW[nap_follow_dist - 1]
  return None


def lead_approach_decel_ms2(v_ego, v_lead, d_rel, t_follow, a_comfort=LEAD_APPROACH_A_MS2):
  """Comfort decel to match a slower lead at the Follow Distance gap, or None.

  `t_follow` is NAPFollowDistance (1=closest/later, 7=farthest/earlier).
  None when speeds match, the lead is faster, or the lead is still outside
  the window (no crawl). Close-in / stopping stay with MPC.
  """
  if t_follow is None or float(t_follow) <= 0 or a_comfort <= 0:
    return None
  if d_rel is None or float(d_rel) <= 0:
    return None
  v0 = float(v_ego)
  vt = max(0.0, float(v_lead))
  if v0 < vt + LEAD_APPROACH_DV_MS:
    return None
  d_follow = float(t_follow) * vt + STOP_DISTANCE
  slack = float(d_rel) - d_follow
  if slack <= 1.0:
    return None
  need_m = (v0 * v0 - vt * vt) / (2.0 * float(a_comfort))
  if slack > need_m:
    return None
  a_needed = (vt * vt - v0 * v0) / (2.0 * slack)
  return max(float(a_needed), -float(a_comfort))
