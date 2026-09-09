"""Earlier, gentler catch-up to a slower radar lead.

Overlay peak is 0.80 m/s² (map Normal brake), never harder. MPC 2.5 m/s²
still owns close-in / stopping if needed.

Map drops use road-distance kinematics plus 110 m because the sign does not
move. A radar lead does: (v_lead² - v_ego²) / (2 * slack) matches speed after
ego travels `slack` meters of road, while the lead moves almost as far, so the
gap barely closes. That hang sits at radar range and pulses as the track
comes and goes.

This uses relative kinematics so extra speed is bled while closing onto the
selected Follow Distance (t_follow), not while holding outside radar.
"""
from __future__ import annotations

# Keep in sync with long_mpc.STOP_DISTANCE (acados cruise/lead obstacle).
STOP_DISTANCE = 6.0
# Map Normal brake. Gentler than overlay 1.0 and MPC 2.5. Do not raise.
LEAD_APPROACH_A_MS2 = 0.80
# Seconds of current closing-speed added before the last-second 0.80 catch.
# Keeps the open gentle (~0.2 m/s²) and the start inside typical radar.
LEAD_APPROACH_HEADSTART_S = 8.0
# Bosch-range ceiling so we do not open on a flickering 160 m track.
LEAD_APPROACH_MAX_START_M = 140.0
LEAD_APPROACH_DV_MS = 0.5  # ~1 mph; ignore radar jitter
NAP_T_FOLLOW = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)


def nap_t_follow(nap_follow_dist: int | None) -> float | None:
  if nap_follow_dist in range(1, len(NAP_T_FOLLOW) + 1):
    return NAP_T_FOLLOW[nap_follow_dist - 1]
  return None


def lead_approach_need_m(v_ego, v_lead, a_comfort=LEAD_APPROACH_A_MS2, t_follow=None) -> float:
  """Meters of slack (gap above Follow Distance) at which the ease starts."""
  v_rel = float(v_ego) - max(0.0, float(v_lead))
  vt = max(0.0, float(v_lead))
  if v_rel <= 0.0 or a_comfort <= 0:
    return 0.0
  # Relative 0.80 catch, plus head-start so we begin gently — not map's +110 m.
  need = (v_rel * v_rel) / (2.0 * float(a_comfort)) + v_rel * LEAD_APPROACH_HEADSTART_S
  if t_follow is not None and float(t_follow) > 0:
    d_follow = float(t_follow) * vt + STOP_DISTANCE
    need = min(need, max(0.0, LEAD_APPROACH_MAX_START_M - d_follow))
  return need


def lead_approach_decel_ms2(v_ego, v_lead, d_rel, t_follow, a_comfort=LEAD_APPROACH_A_MS2):
  """Comfort decel to close onto the Follow Distance gap, or None.

  a = -v_rel² / (2 * slack) so we arrive at the selected gap with matching
  speed. |a| at the open is below 0.80 and only reaches 0.80 near that gap.
  None when speeds match, the lead is faster, or the lead is still outside
  the window (no crawl / no radar-edge hang).
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
