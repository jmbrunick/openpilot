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

Justin: start that planned comfort ease / speed-match as soon as radar has
reasonable feedback on a closing lead (`leadOne` valid and closing) — do not
wait until late in the gap. Farther ceiling + longer head-start + a clear-
close path (large slack is OK) so later brakes are not as hard. Still soft
only; MPC / FCW win via min(), except a far/gentle nibble must not
steal lead-close catch-up +a.

On a slight grade, radar `v_rel` / slack chatter around the follow gap used
to snap this overlay on/off (regen bite → Accel-1 crawl → bite). Enter/exit
hysteresis plus a per-frame slew on more-negative `a` hold a steady ease
instead of chattering. Off / milder `a` is immediate so rematch is not stuck
in regen. After the first hysteresis pass, leftover occasional bump-pull
was still that gap-edge rematch (overlay |a| ~0.06–0.13, then Accel-1),
not the 0.55 peak. Raise enter only so rematch does not re-bite; keep the
0.20 exit so we still close onto Follow Distance (a lower exit parked far).

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
# 24 s vs 12 s: ~54 m / ~12 s earlier on a 10 mph close (need path).
# Clear-close (v_rel ≥ CLEAR_DV) skips need and starts at first reliable
# track. Same 0.55 peak near Follow Distance.
LEAD_APPROACH_HEADSTART_S = 24.0
# Usable Bosch ceiling. Old 140 m waited until late in the gap; 200 m is
# still inside typical Pre-AP Bosch reports. Anti-flicker is quality +
# hysteresis, not a short ceiling — see lead_approach_track_ok.
LEAD_APPROACH_MAX_START_M = 200.0
# Hold a few meters past the ceiling so a track at 199–201 m does not chatter.
LEAD_APPROACH_MAX_HOLD_M = 8.0
# Inside this, leadOne.status is enough. Beyond it, require radar + modelProb.
# LeadData exposes those two; Track.cnt age is not published on Pre-AP.
LEAD_APPROACH_RELIABLE_M = 140.0
LEAD_APPROACH_MODEL_PROB_MIN = 0.50  # radard association gate
# Clearly closing: skip the need window and ease from first reliable track.
# 2.5 m/s (~5.6 mph). 1.0 stole Accel-1 catch-up / Follow Distance close
# (overlay min() beat +a while slack was still large). 10 mph still skips need.
LEAD_APPROACH_CLEAR_DV_MS = 2.5

# Enter / exit (hysteresis). A single v_rel / slack gate chatters around
# the follow gap on a slight incline (regen ↔ Accel-1). First pass was
# 0.50 / 0.20; leftover bump-pull was rematch re-crossing 0.50. Raise
# enter only — a 0.12 exit held ease too long and parked far back.
LEAD_APPROACH_DV_MS = 0.55         # enter: ~1.2 mph closing; ignore radar jitter
LEAD_APPROACH_DV_OFF_MS = 0.20     # exit: ~0.45 mph; drop so rematch can finish the close
LEAD_APPROACH_SLACK_ON_M = 1.0     # enter only with slack above Follow Distance
LEAD_APPROACH_SLACK_OFF_M = 0.0    # stay until at/inside the follow gap
LEAD_APPROACH_NEED_HOLD_M = 4.0    # extra slack (m) before dropping after open

# Gradual regen onset (planner frame). Same step as accel_clip slew.
# At DT_MDL=0.05 s → 1.0 m/s²/s. Release / milder a is immediate.
LEAD_APPROACH_SLEW_MS2 = 0.05
# Softer than this is a nibble (matching-traffic / far slack, |a| ~0.06–0.13).
# Catch-up +a may ignore it; real ease and MPC 0/−a still use min().
LEAD_APPROACH_NIBBLE_MS2 = 0.15

NAP_T_FOLLOW = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)

# Catch-up +a cap stays at the old 140 m flicker-safe window. Ease start
# grew; punching MAX-rise toward a 180 m same-speed lead is a different
# product and is not expanded here.
LEAD_CLOSE_MAX_M = 140.0
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
  return 0.0 < d <= LEAD_CLOSE_MAX_M


def lead_approach_track_ok(d_rel, model_prob=None, radar=None, active=False) -> bool:
  """Far-track anti-flicker. LeadData has modelProb + radar; no track age.

  Inside RELIABLE_M, `leadOne.status` is enough (planner already gated).
  Beyond it, enter needs a radar-associated lead (`radar=True`) and
  modelProb at/above radard's 0.5 association gate. Missing quality
  args (unit kinematics) are treated as ok. Once active, hold through
  a brief modelProb dip so far tracks do not chatter.
  """
  if d_rel is None:
    return False
  d = float(d_rel)
  if d <= LEAD_APPROACH_RELIABLE_M or active:
    return True
  if radar is False:
    return False
  if model_prob is not None and float(model_prob) < LEAD_APPROACH_MODEL_PROB_MIN:
    return False
  return True


def lead_approach_need_m(v_ego, v_lead, a_comfort=LEAD_APPROACH_A_MS2, t_follow=None) -> float:
  """Meters of slack (gap above Follow Distance) at which a *marginal* close starts.

  Clear-close (`v_rel` ≥ CLEAR_DV) skips this and eases from the first
  reliable track, still capped by MAX_START. Need is never past the
  Bosch ceiling (no map-style +110 m road hang).
  """
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


def apply_lead_approach_overlay(output_a, a_lead, nibble=LEAD_APPROACH_NIBBLE_MS2):
  """Soft overlay via min(), except a nibble must not steal catch-up +a.

  Matching-traffic / far-slack overlay sits at |a| ~0.06–0.13. That must
  not beat lead-close +a (Follow Distance close / rematch). Real ease and
  MPC 0 / −a still use min().
  """
  if a_lead is None:
    return float(output_a)
  out = float(output_a)
  a = float(a_lead)
  if out <= 0.0 or a <= -float(nibble):
    return min(out, a)
  return out


def lead_approach_decel_ms2(v_ego, v_lead, d_rel, t_follow, a_comfort=LEAD_APPROACH_A_MS2,
                           active=False, model_prob=None, radar=None):
  """Comfort decel to close onto the Follow Distance gap, or None.

  a = -v_rel² / (2 * slack) so we arrive at the selected gap with matching
  speed. |a| at the open is below the comfort peak and only reaches that
  peak near the gap. None when speeds match, the lead is faster, or the
  lead is still outside the window.

  `active` is last frame's overlay (hysteresis). Enter uses DV_MS / SLACK_ON;
  hold uses DV_OFF / SLACK_OFF / need+NEED_HOLD so small radar noise does
  not chatter regen ↔ accel.

  When `v_rel` is clearly positive (≥ CLEAR_DV) and the track is reliable,
  large slack is allowed — speed-match from the first reasonable radar
  feedback, still capped at 0.55 and slewed by the planner.
  """
  if t_follow is None or float(t_follow) <= 0 or a_comfort <= 0:
    return None
  if d_rel is None or float(d_rel) <= 0:
    return None
  d = float(d_rel)
  d_max = LEAD_APPROACH_MAX_START_M + (LEAD_APPROACH_MAX_HOLD_M if active else 0.0)
  if d > d_max:
    return None
  if not lead_approach_track_ok(d, model_prob, radar, active=active):
    return None
  v0 = float(v_ego)
  vt = max(0.0, float(v_lead))
  v_rel = v0 - vt
  dv_gate = LEAD_APPROACH_DV_OFF_MS if active else LEAD_APPROACH_DV_MS
  if v_rel < dv_gate:
    return None
  d_follow = float(t_follow) * vt + STOP_DISTANCE
  slack = d - d_follow
  slack_min = LEAD_APPROACH_SLACK_OFF_M if active else LEAD_APPROACH_SLACK_ON_M
  if slack <= slack_min:
    return None
  clear = v_rel >= LEAD_APPROACH_CLEAR_DV_MS
  if not clear:
    need_m = lead_approach_need_m(v0, vt, a_comfort, t_follow)
    need_gate = need_m + (LEAD_APPROACH_NEED_HOLD_M if active else 0.0)
    if slack > need_gate:
      return None
  a_needed = -(v_rel * v_rel) / (2.0 * slack)
  return max(float(a_needed), -float(a_comfort))
