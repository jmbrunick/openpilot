"""Earlier, gentler catch-up to a slower radar lead.

Same idea as map drops: more start distance, not more aggressive a.
Overlay peak is 0.80 m/s² (map Normal brake), never harder. MPC 2.5 m/s²
still owns close-in / stopping if needed. Target gap is the selected
Follow Distance (t_follow); this only starts the close sooner so the gap
shrinks smoothly instead of a late catch.
"""
from __future__ import annotations

# Keep in sync with long_mpc.STOP_DISTANCE (acados cruise/lead obstacle).
STOP_DISTANCE = 6.0
# Map Normal brake / DECREASE_START_MARGIN_M. Gentler than overlay 1.0 and MPC 2.5.
LEAD_APPROACH_A_MS2 = 0.80
LEAD_APPROACH_MARGIN_M = 110.0
LEAD_APPROACH_DV_MS = 0.5  # ~1 mph; ignore radar jitter
NAP_T_FOLLOW = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)


def nap_t_follow(nap_follow_dist: int | None) -> float | None:
  if nap_follow_dist in range(1, len(NAP_T_FOLLOW) + 1):
    return NAP_T_FOLLOW[nap_follow_dist - 1]
  return None


def lead_approach_need_m(v_ego, v_lead, a_comfort=LEAD_APPROACH_A_MS2) -> float:
  """Meters of slack (gap above Follow Distance) at which the ease starts."""
  v0 = float(v_ego)
  vt = max(0.0, float(v_lead))
  if v0 <= vt or a_comfort <= 0:
    return 0.0
  return (v0 * v0 - vt * vt) / (2.0 * float(a_comfort)) + LEAD_APPROACH_MARGIN_M


def lead_approach_decel_ms2(v_ego, v_lead, d_rel, t_follow, a_comfort=LEAD_APPROACH_A_MS2):
  """Comfort decel to close smoothly onto the Follow Distance gap, or None.

  Starts at kinematic d + 110 m (map-style extra distance). |a| at the open
  is below 0.80 and only reaches 0.80 near the selected gap — not a harder
  peak. `t_follow` is NAPFollowDistance. None when speeds match, the lead
  is faster, or the lead is still outside the window (no crawl).
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
  need_m = lead_approach_need_m(v0, vt, a_comfort)
  if slack > need_m:
    return None
  a_needed = (vt * vt - v0 * v0) / (2.0 * slack)
  return max(float(a_needed), -float(a_comfort))
