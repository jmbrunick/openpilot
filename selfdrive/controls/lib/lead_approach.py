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
close path (large slack is OK) so later brakes are not as hard.

Mild / normal closes were still a hard let-off: `a = -v_rel² / (2 * slack)`
ramps every close to the 0.55 peak near Follow Distance. Cap non-rapid
closes at light regen (MILD 0.22). Rapid / dumping (high `v_rel`) still
uses kinematics up to 0.55. Start early and light; do not delay ease
(that forces a late bite). MPC / FCW win via min().

A far/gentle nibble must not steal large-gap *same-speed* catch-up +a
(cruise / MAX climb). A live or held lead that is already closing
(≳ 1.0–1.5 m/s) never rematches +a — coast or match-speed −a only.
aLead ≲ −0.2 owns match-speed only near the follow gap; a far or
opening lead (da 2026-09-18: dRel~64 m, v_rel~−1.44 opening,
aLeadK~−1.46) must not snap cruise +a to aLeadK. Near the follow
gap, a nibble min()s so we can mesh into lead speed at the set
Follow Distance. Rematch after ease (v_rel flips / slack growing)
trickles +a — do not slam regen → Accel.

On a slight grade, radar `v_rel` / slack chatter around the follow gap used
to snap this overlay on/off (regen bite → rematch crawl → bite). Enter/exit
hysteresis plus a per-frame slew on more-negative *and* milder `a` hold a
steady ease instead of chattering. After the first hysteresis pass, leftover
occasional bump-pull was still that gap-edge rematch (overlay |a| ~0.06–0.13,
then rematch), not the 0.55 peak. Raise enter only so rematch does not
re-bite; keep the 0.20 exit so we still close onto Follow Distance.

Positive close-the-gap accel uses the same Accel 1–10 envelope as open-road
/ MAX climb (`lead_close_accel_ms2` = Mannerisms Accel, with the same
last-mph gradient). Map Accel used to gate only MAX-rise; Adaptive Accel
and cruise 1.6–0.6 punched when the gap was large; the close-cap used to
stop at 140 m so a 160–180 m lead still got that punch. With a lead in
Bosch range, close slack at Accel 1–10 — never hotter. Near-gap rematch
trickles. After speeds match (`|v_rel| < 0.5` for ~0.75 s near the follow
gap), do not pin Accel-ceil rematch +a while the gap is OK or already
opening (d7 20:25:33: +0.323 for 7 s as dRel 49→59). Rematch above a
trickle only if slack ≳ 20 m and the lead is pulling away; a ≳ 15 m gap
error with `|closing| < 1` may use a small Accel-proportional hunt, not
full Accel every pulse. Large same-speed gaps that never matched still
use Accel catch-up *under MAX*. At or above MAX (vEgo ≥ vCruise −
deadband) rematch / remaining-close / lead-close +a is 0 — never chase
a faster lead past set/MAX (ea 11:46: +0.36 at 62 on a 60 MAX).
Rate-limit +a across cruise↔lead flips when not
rapidly closing. A brief `leadOne` drop holds the last in-window lead so
the cap cannot be bypassed. Vision-only far flicker does not cap
empty-road climb. Non-rapid MPC −a is floored at MILD (slight lift).
On Justin's Pre-AP Model S EV a slight lift already regenerates hard;
full throttle lift / deep −a is reserved for emergency / rapid-close /
near-bumper / FCW only. Closing ≳ 1.5 or a near-gap braking lead must
not skip that floor (that was still dumping −1 to −2 on mild town
closes). Large-slack small adjustments (e4 −2.33) stay floored.
Rapid / near-bumper / FCW / crash still own danger.

First lead latch used to punch MPC regen then rematch +a in ~0.5 s
(e4 09:53:19: −0.46 → +0.05 at 118 m; 10:48 −0.996 at 80–130 m).
During the acquire window, slew aTarget both ways so that spike cannot
yo-yo. Rapid / near-bumper / FCW stay immediate. Closing ≳ 1.5 at
range does not skip the window.

After that first latch, e8 still undershot Follow Distance (~23 m)
and yo-yoed ±a once speeds matched. Keep acquire slew and #216's
MILD comfort floor. While still closing, aim a few meters long of
the set gap so kinematics finish with leftover slack. Inside FD, a
slow close commands that mild floor — not rematch +a, not a dump.
When |v_rel| is small at/long of the gap, glide: a≈0 deadband +
hysteresis (EV slight lift only). Already inside FD is a too-close
recovery — rematch +a and mild −a stand. Rapid / bumper / FCW dump.
"""
from __future__ import annotations

from openpilot.selfdrive.mapd.constants import LOOKAHEAD_NORMAL, TRACK_DEADBAND_MS, map_accel_a_ms2

# Keep in sync with long_mpc.STOP_DISTANCE (acados cruise/lead obstacle).
STOP_DISTANCE = 6.0
# Comfort peak |a|. Early map brake, not Normal 0.80 — Tesla VirtualDAS
# regen at 0.80 then rematch chatters on a slight grade. Do not raise.
# MPC 2.5 / FCW still own danger. Rapid closes only.
LEAD_APPROACH_A_MS2 = 0.55
# Seconds of current closing-speed added before the last-second catch.
# 24 s vs 12 s: ~54 m / ~12 s earlier on a 10 mph close (need path).
# Clear-close (v_rel ≥ CLEAR_DV) skips need and starts at first reliable
# track. Mild closes stay at MILD; rapid still 0.55 near Follow Distance.
LEAD_APPROACH_HEADSTART_S = 24.0
# Usable Bosch ceiling. Old 140 m waited until late in the gap; 200 m is
# still inside typical Pre-AP Bosch reports. Anti-flicker is quality +
# hysteresis, not a short ceiling — see lead_approach_track_ok.
LEAD_APPROACH_MAX_START_M = 200.0
# Hold a few meters past the ceiling so a track at 199–201 m does not chatter.
LEAD_APPROACH_MAX_HOLD_M = 8.0
# Inside this, leadOne.status is enough. Beyond it, require radar
# association so vision-only far flicker cannot own the plan. Radar
# tracks do not wait on modelProb. LeadData has no track age on Pre-AP.
LEAD_APPROACH_RELIABLE_M = 140.0
LEAD_APPROACH_MODEL_PROB_MIN = 0.50  # radard association gate
# Clearly closing: skip the need window and ease from first reliable track.
# 1.5 m/s — same as the rematch-block / floor-skip close gate — so a far
# radar lock that is already closing starts ease immediately. 1.0 still
# stays Accel-1 catch-up. Vision-only flicker does not skip need.
LEAD_APPROACH_CLEAR_DV_MS = 1.5
# Mild-close comfort ceiling. Kinematics used to hit 0.55 on a 3–10 mph
# close right at the gap (hard let-off). Light regen / ease-off only.
# Rapid (high closing rate) keeps the 0.55 path.
LEAD_APPROACH_MILD_A_MS2 = 0.22
# Time-to-follow-gap window. Far TTC stays kinematic nibble; inside this
# a mild close is the light ceiling, not a delayed 0.55 bite.
LEAD_APPROACH_TTC_START_S = 20.0
# Rapid / dumping: closing rate high (much faster than lead). ~13 mph.
# Short TTC at a mild v_rel is "almost at the gap", not dumping — do not
# promote that to 0.55.
LEAD_APPROACH_RAPID_DV_MS = 6.0
LEAD_APPROACH_RAPID_TTC_S = 8.0
# Skip the non-rapid −MILD MPC floor when matching a slowing lead. 1.5 m/s
# (~3.4 mph) is above overlay-enter jitter (0.55) so rematch chatter still
# floors, and at/below the town-entry log that pinned aTarget at −0.22
# while closing rose 1.6→4.4 m/s under the 6 m/s rapid gate.
LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS = 1.5
# Lead clearly braking. −0.2 is a real coast/brake, not aLeadK noise at 0.
LEAD_APPROACH_SOFT_LIMIT_ALEAD_MS2 = -0.2
# Never rematch / cruise +a into a live or held closing gap. 1.0 m/s
# (~2.2 mph) is above rematch-enter jitter; 1.5 is match-speed / ownership.
LEAD_CLOSING_REMATCH_BLOCK_MS = 1.0
LEAD_CLOSING_MATCH_MS = LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS
LEAD_CLOSING_ALEAD_MS2 = LEAD_APPROACH_SOFT_LIMIT_ALEAD_MS2
LEAD_CLOSING_MATCH_GAIN = 0.25
# aLead-only ownership / match-aLeadK. Opening or slack ≳ 20 m must not
# force aTarget ≤ aLeadK (far braking lead, gap opening). Near-gap
# braking-lead protection still matches. 20 m is inside the 15–25 m tune.
LEAD_ALEAD_MATCH_SLACK_M = 20.0
# Planner frames of high v_rel before the 0.55 path. One radar blip must
# not fire hard regen; mild ease stays immediate. Count only in-window
# closes (do not pre-arm from a far flicker). 4 × DT_MDL ≈ 0.20 s.
LEAD_APPROACH_RAPID_CONFIRM_N = 4

# Enter / exit (hysteresis). A single v_rel / slack gate chatters around
# the follow gap on a slight incline (regen ↔ accel). First pass was
# 0.50 / 0.20; leftover bump-pull was rematch re-crossing 0.50. Raise
# enter only — a 0.12 exit held ease too long and parked far back.
LEAD_APPROACH_DV_MS = 0.55         # enter: ~1.2 mph closing; ignore radar jitter
LEAD_APPROACH_DV_OFF_MS = 0.20     # exit: ~0.45 mph; drop so rematch can finish the close
LEAD_APPROACH_SLACK_ON_M = 1.0     # enter only with slack above Follow Distance
LEAD_APPROACH_SLACK_OFF_M = 0.0    # stay until at/inside the follow gap
LEAD_APPROACH_NEED_HOLD_M = 4.0    # extra slack (m) before dropping after open

# Gradual regen onset (planner frame). Same step as accel_clip slew.
# At DT_MDL=0.05 s → 1.0 m/s²/s. Milder / off slews toward 0 so rematch
# is not a regen→Accel slam.
LEAD_APPROACH_SLEW_MS2 = 0.05
LEAD_APPROACH_RELEASE_SLEW_MS2 = 0.025
# Softer than this is a nibble (matching-traffic / far slack, |a| ~0.06–0.13).
# Large-gap catch-up +a may ignore it; real ease, near-gap rematch, and
# MPC 0/−a still use min() / ease-off.
LEAD_APPROACH_NIBBLE_MS2 = 0.15

NAP_T_FOLLOW = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)

# Same Bosch ceiling as ease. A 160–180 m same-speed lead used to skip the
# cap and punch cruise / MAX-rise to close Follow Distance. Do not.
LEAD_CLOSE_MAX_M = LEAD_APPROACH_MAX_START_M
# Catch-up +a is Mannerisms Accel (same as open-road / MAX climb). Not a
# separate 0.12/0.18/0.28 or 0.20/0.30/0.50 punch curve. Accel 1/5/10 =
# 0.36/0.80/1.60 at Lookahead Normal. Planner also applies the Accel 1–10
# last-mph taper. Does not change MPC danger / hard brake.
LEAD_CLOSE_A_BASE_MS2 = map_accel_a_ms2(LOOKAHEAD_NORMAL, 5)
LEAD_CLOSE_A_MIN_MS2 = map_accel_a_ms2(LOOKAHEAD_NORMAL, 1)
LEAD_CLOSE_A_MAX_MS2 = map_accel_a_ms2(LOOKAHEAD_NORMAL, 10)
# Near Follow Distance, rematch after ease must trickle. Large-gap
# catch-up uses Mannerisms Accel (same as open-road).
LEAD_CLOSE_OPENING_A_MS2 = 0.08
LEAD_CLOSE_REMATCH_A_MS2 = 0.12
LEAD_CLOSE_REMATCH_SLACK_M = 12.0
# After |v_rel| < 0.5 for 0.5–1 s near the follow gap, Accel-ceil rematch
# is deadbanded. Arm settle only while slack is still follow-like so a
# 160 m same-speed catch-up stays Accel-owned. Do not arm while still
# inside Follow Distance (recovering from too-close). Once settled, hold
# through a growing gap (do not re-arm Accel ceil as dRel 49→59).
LEAD_SETTLE_VREL_MS = 0.5
LEAD_SETTLE_HOLD_S = 0.75
LEAD_SETTLE_SLACK_M = 25.0
# Remaining close after settle. Slight opening at 2–4 m must still
# trickle onto Follow Distance; 8 m+ opening stays deadbanded.
LEAD_SETTLE_FINISH_SLACK_M = 4.0
# Settled rematch: trickle/0 while gap is OK or opening. Hunt (small
# Accel-proportional, not full ceil) when gap error ≳ 15 m and |closing|
# < 1. Rematch above trickle if slack ≳ 20 m *and* lead pulling away.
LEAD_HUNT_SLACK_M = 15.0
LEAD_REMATCH_PULL_SLACK_M = 20.0
LEAD_HUNT_A_FRAC = 0.35
LEAD_HUNT_SLACK_SPAN_M = 25.0  # 15→40 m scales hunt 0→frac
# Rate-limit +a across cruise↔lead ownership flips. Same step as the
# accel_clip slew. Closing ≳ 1.0 / −a is immediate (do not delay #190)
# *after* the first-latch window. First acquire slews both ways.
LEAD_ATARGET_SLEW_MS2 = 0.05
# First-latch window. e4 09:53:19 punched −0.46 then rematched +0.05 in
# ~0.5 s at 118 m. Slew both directions for this long so the spike
# cannot yo-yo. Rapid / near-bumper skip the window.
LEAD_ACQUIRE_HOLD_S = 0.75
LEAD_ACQUIRE_SLEW_MS2 = LEAD_ATARGET_SLEW_MS2
# Inside this dRel, never soften MPC −a (near bumper).
LEAD_MPC_SOFT_NEAR_M = 12.0
# Aim a few meters long of Follow Distance while still closing so
# kinematics finish with leftover slack (e8 arrived at slack=0 still
# closing ~1.25 m/s). HUD t_follow / dFollow unchanged.
LEAD_SETTLE_GAP_BIAS_M = 3.0
# Inside FD, still closing slowly: command the #216 MILD floor (not
# rematch +a, not a dump). Rapid / bumper / FCW stay full −a.
LEAD_SLOW_CLOSE_MS = 0.8
# Matched-speed glide: kill leftover mild −a near the gap. Rematch
# trickle (+0.08) still finishes the last meters / holds grade.
# Still-closing stays off (a 0.5 m/s window coasted through Follow 1).
# Slack covers the grade-hold band (~5–10 m long of FD), not only
# the last 4 m — that left plant-aligned grade 0.12 m/s slow.
LEAD_GLIDE_VREL_MS = 0.25
LEAD_GLIDE_VREL_OFF_MS = 0.40
LEAD_GLIDE_SLACK_M = 10.0
LEAD_GLIDE_SLACK_OFF_M = 14.0
LEAD_GLIDE_A_MS2 = LEAD_CLOSE_OPENING_A_MS2
# Rematch trickle ↔ mild floor. Glide zeros −a in this band.
LEAD_GLIDE_CHATTER_LO_MS2 = -(LEAD_APPROACH_MILD_A_MS2 + 0.02)
LEAD_GLIDE_CHATTER_HI_MS2 = LEAD_CLOSE_REMATCH_A_MS2 + 0.02
# Near-gap small ±a slew (rematch trickle ↔ leftover mild).
LEAD_NEAR_GAP_SLACK_M = 15.0
LEAD_NEAR_GAP_SLEW_MS2 = 0.02
# Brief hold of the last in-window lead when `leadOne.status` drops so
# cruise punch cannot leak through a radar flicker. ~10 planner frames.
LEAD_CLOSE_HOLD_S = 0.50


def nap_t_follow(nap_follow_dist: int | None) -> float | None:
  if nap_follow_dist in range(1, len(NAP_T_FOLLOW) + 1):
    return NAP_T_FOLLOW[nap_follow_dist - 1]
  return None


def lead_follow_slack_m(d_rel, v_lead, t_follow):
  """Meters above the selected Follow Distance, or None if unknown."""
  if d_rel is None or t_follow is None or float(t_follow) <= 0:
    return None
  d_follow = float(t_follow) * max(0.0, float(v_lead)) + STOP_DISTANCE
  return float(d_rel) - d_follow


def lead_kinematic_slack_m(slack, v_rel) -> float:
  """Slack used for approach kinematics. Aim a few meters long while closing.

  HUD Follow Distance is unchanged. Near the gap, never use a tinier
  remaining than the real slack (avoids a late 1/x punch).
  """
  s = float(slack)
  if v_rel is None or float(v_rel) < LEAD_APPROACH_DV_OFF_MS:
    return s
  return max(s - LEAD_SETTLE_GAP_BIAS_M, min(s, 0.75))


def lead_inside_slow_close_a_ms2(v_rel, slack):
  """Slight-lift MILD when inside FD and still closing slowly.

  #216: comfort stays at −0.22. This only ensures we command that
  floor (not rematch +a) while closing ≳ 0.8 inside FD. Rapid /
  bumper / FCW stay dump.
  """
  if slack is None or float(slack) > 0.0:
    return None
  if v_rel is None:
    return None
  v = float(v_rel)
  if v < LEAD_SLOW_CLOSE_MS or lead_approach_is_rapid(v):
    return None
  return -LEAD_APPROACH_MILD_A_MS2


def lead_is_glide_sample(v_rel, slack) -> bool:
  """True when matched or slightly slower near Follow Distance.

  Last-meter closing must keep −a so we do not coast through FD1.
  A slight close with slack still in the grade-hold band (~5–10 m)
  may glide. Real speed sag (v_rel ≲ −0.5) is Accel-owned.
  Already inside FD is a too-close recovery.
  """
  if v_rel is None or slack is None:
    return False
  v = float(v_rel)
  s = float(slack)
  if s < 0.0 or s > LEAD_GLIDE_SLACK_M:
    return False
  if v <= -LEAD_SETTLE_VREL_MS or v >= LEAD_SETTLE_VREL_MS:
    return False
  if v > LEAD_APPROACH_DV_OFF_MS and s <= LEAD_SETTLE_FINISH_SLACK_M:
    return False
  return True


def update_lead_glide(active, v_rel, slack, d_rel=None, fcw=False,
                      crash_cnt=0, allow_rapid=False, acquiring=False,
                      a_lead=None):
  """Arm / hold matched-speed glide. Danger and first-latch skip it."""
  if acquiring or fcw or int(crash_cnt) > 0:
    return False
  if d_rel is not None and float(d_rel) <= LEAD_MPC_SOFT_NEAR_M:
    return False
  if v_rel is not None and lead_approach_is_rapid(v_rel):
    return False
  if allow_rapid and v_rel is not None and lead_approach_is_rapid(v_rel):
    return False
  if lead_alead_owns_match(v_rel, a_lead, slack):
    return False
  if active:
    if v_rel is None or slack is None:
      return False
    if float(v_rel) > LEAD_GLIDE_VREL_OFF_MS:
      return False
    if float(v_rel) <= -LEAD_SETTLE_VREL_MS:
      return False
    if float(slack) < 0.0 or float(slack) > LEAD_GLIDE_SLACK_OFF_M:
      return False
    return True
  return lead_is_glide_sample(v_rel, slack)


def apply_lead_glide_a(output_a, gliding):
  """Deadband leftover mild −a to coast; rematch trickle may still finish.

  Accel-ceil grade hold and MPC dump sit outside the chatter band and
  pass through. Negative chatter becomes 0 (no felt regen bite).
  Positive chatter caps at the rematch trickle so last-meter finish
  and slight grade sag still work.
  """
  if (not gliding) or output_a is None:
    return output_a
  a = float(output_a)
  if LEAD_GLIDE_CHATTER_LO_MS2 <= a <= LEAD_GLIDE_CHATTER_HI_MS2:
    if a <= 0.0:
      return 0.0
    return min(a, LEAD_GLIDE_A_MS2)
  return a


def _near_gap_small_bite(a) -> bool:
  # Covers mild floor ↔ ~0.70 MPC bite and rematch trickle.
  return (LEAD_GLIDE_CHATTER_LO_MS2 - 0.50) <= float(a) <= LEAD_GLIDE_CHATTER_HI_MS2


def slew_near_gap_small_a(target, prev, v_rel, d_rel=None, slack=None,
                          allow_rapid=False, fcw=False, crash_cnt=0,
                          slew=LEAD_NEAR_GAP_SLEW_MS2):
  """Slew small near-gap ±a so floor↔release cannot step 0.3 in one frame.

  Full authority (rapid / bumper / FCW / hard kinematics) is immediate.
  Far slack and bites outside the small band pass through. First-latch
  acquire slew is a separate path and must stay unchanged.
  """
  if target is None:
    return target
  t = float(target)
  if lead_mpc_needs_full_authority(
    v_rel, d_rel, slack, allow_rapid=allow_rapid, fcw=fcw,
    crash_cnt=crash_cnt, confirm_rapid=True,
  ):
    return t
  # Inside FD progressive −a must not wait on this slew. Chatter to
  # soften is the near-gap floor↔release band (slack > 0).
  if slack is None or float(slack) <= 0.0 or float(slack) > LEAD_NEAR_GAP_SLACK_M:
    return t
  if prev is None:
    return t
  p = float(prev)
  if not (_near_gap_small_bite(t) and _near_gap_small_bite(p)):
    return t
  # Do not delay the first onset of mild ease (0 → −0.22). Overlay
  # already slews; this path only softens floor↔release chatter.
  if abs(p) < 1e-6:
    return t
  # Do not delay rematch / grade +a. Plant-aligned hold sat 0.12 m/s
  # slow when −0.22 → +0.08 walked 0.02 / frame.
  if t >= 0.0:
    return t
  if t > p:
    return min(t, p + float(slew))
  if t < p:
    return max(t, p - float(slew))
  return t


def lead_is_settled_sample(v_rel, slack) -> bool:
  """True when this frame can count toward match-settle.

  Match means speeds already agree *and* the gap is still follow-like.
  A 160 m same-speed lead is catch-up, not a settled follow. Still
  inside Follow Distance (negative slack) is a too-close recovery, not
  a matched follow — Accel/MPC must be allowed to open back to the gap.
  """
  if v_rel is None:
    return False
  if abs(float(v_rel)) >= LEAD_SETTLE_VREL_MS:
    return False
  if slack is None or float(slack) > LEAD_SETTLE_SLACK_M:
    return False
  if float(slack) < 0.0:
    return False
  return True


def update_lead_settle(age, settled, v_rel, slack, dt, present=True,
                       closing_hard=False):
  """Arm settle after |v_rel| < 0.5 for ~0.75 s near the follow gap.

  Once settled, hold while the lead is present even if slack grows
  (do not re-arm Accel-ceil rematch). Clear on lead loss or closing
  ≳ 1.5 (lead owns the plan — not a follow rematch).
  Returns `(age, settled)`.
  """
  if (not present) or closing_hard:
    return 0.0, False
  if settled:
    return max(float(age), LEAD_SETTLE_HOLD_S), True
  if lead_is_settled_sample(v_rel, slack):
    nxt = float(age) + max(0.0, float(dt))
    return nxt, nxt >= LEAD_SETTLE_HOLD_S
  return 0.0, False


def lead_hunt_accel_ms2(a, slack) -> float:
  """Small Accel-proportional close after settle. Never full Accel ceil."""
  a = max(0.0, float(a))
  s = 0.0 if slack is None else float(slack)
  excess = max(0.0, s - LEAD_HUNT_SLACK_M)
  scale = min(1.0, excess / LEAD_HUNT_SLACK_SPAN_M) if LEAD_HUNT_SLACK_SPAN_M > 0 else 0.0
  a_hunt = a * LEAD_HUNT_A_FRAC * scale
  return min(a, max(LEAD_CLOSE_OPENING_A_MS2, a_hunt))


def lead_at_or_above_max(v_ego, v_cruise) -> bool:
  """True when rematch / remaining-close must not command +a past MAX.

  Same edge as map-track climb (`v_cruise − v_ego ≤ TRACK_DEADBAND`):
  in or above the last ~0.9 mph, MAX is a hard ceiling. A faster lead
  may pull away. Missing / unset speeds leave the gate off so unit
  kinematics stay unchanged.
  """
  if v_ego is None or v_cruise is None:
    return False
  if float(v_ego) <= 0.0 or float(v_cruise) <= 0.0:
    return False
  return float(v_ego) >= float(v_cruise) - TRACK_DEADBAND_MS


def lead_settled_rematch_a_ms2(a, v_rel, slack) -> float:
  """After match: no Accel-ceil rematch while the gap is OK or opening.

  Comfortable opening stays 0 (49→59 was Accel ceil +0.32 for 7 s).
  Ego clearly slower (`v_rel` ≲ −0.5) may use Accel so grade/cruise can
  recover speed *under MAX* — a 0 ceiling parked the closed-loop plant
  at 24.38. At/above MAX the caller zeros `a` so this cannot chase.
  Still inside Follow Distance is a too-close recovery, not rematch.
  At the gap with speeds matched, Accel may hold grade. Gap error ≳ 15 m
  with `|closing| < 1` may hunt. Rematch above trickle if slack ≳ 20 m
  *and* the lead is pulling away. Closing ≳ 1.0 is already 0 from the
  caller.
  """
  s = None if slack is None else float(slack)
  v = 0.0 if v_rel is None else float(v_rel)
  opening = v <= 0.0
  # Real speed sag (ego clearly slower): cruise/grade hold, not rematch.
  if s is not None and s >= 0.0 and v <= -LEAD_SETTLE_VREL_MS:
    return a
  if s is None or s < LEAD_HUNT_SLACK_M:
    if opening and s is not None and (s > LEAD_SETTLE_FINISH_SLACK_M or s < 0.0):
      return 0.0
    if s is not None and s <= 0.5 and abs(v) < LEAD_SETTLE_VREL_MS:
      return a
    return min(a, LEAD_CLOSE_OPENING_A_MS2)
  if opening and s < LEAD_REMATCH_PULL_SLACK_M:
    return 0.0
  return lead_hunt_accel_ms2(a, s)


def lead_close_accel_ms2(accel_level: int = 5, v_rel=None, slack=None,
                         a_personality=None, settled=False, v_ego=None,
                         v_cruise=None) -> float:
  """Max positive a (m/s²) when closing the gap on a radar lead.

  Same Accel 1–10 envelope as open-road / MAX climb — not a separate
  hotter (or cooler) catch-up curve. `a_personality` is that envelope
  (peak or last-mph tapered). At/above MAX that envelope is 0 so
  rematch / hunt / speed-sag cannot chase a faster lead past set.
  lead_approach decel (0.55) and MPC −a / danger are unchanged.

  Near the follow gap, a lead pulling away / slow rematch trickles +a
  so ease→Accel does not surge. After settle, Accel-ceil rematch is
  deadbanded; large same-speed gaps that never matched still use Accel
  *under MAX*.
  """
  a = map_accel_a_ms2(LOOKAHEAD_NORMAL, int(accel_level))
  if a_personality is not None:
    a = min(a, max(0.0, float(a_personality)))
  if lead_at_or_above_max(v_ego, v_cruise):
    a = 0.0
  if v_rel is not None and float(v_rel) >= LEAD_CLOSING_REMATCH_BLOCK_MS:
    # Large-gap catch-up still uses Accel (#187) even while closing ≳ 1.0.
    # #190 is near-gap rematch-block; last meters may trickle; ≳ 1.5 owns.
    large_unsettled = (
      (not settled) and slack is not None and float(slack) > LEAD_CLOSE_REMATCH_SLACK_M
    )
    if not large_unsettled:
      if (slack is None or float(slack) > LEAD_SETTLE_FINISH_SLACK_M
          or float(v_rel) >= LEAD_CLOSING_MATCH_MS):
        return 0.0
  if settled:
    return lead_settled_rematch_a_ms2(a, v_rel, slack)
  if slack is None or float(slack) > LEAD_CLOSE_REMATCH_SLACK_M:
    return a
  if v_rel is not None and float(v_rel) <= 0.0:
    return min(a, LEAD_CLOSE_OPENING_A_MS2)
  if v_rel is not None and float(v_rel) < LEAD_APPROACH_DV_MS:
    return min(a, LEAD_CLOSE_REMATCH_A_MS2)
  return a


def lead_remaining_close_a_ms2(output_a, v_rel, slack, v_ego=None, v_cruise=None):
  """Command trickle +a to finish Follow Distance when MPC/cruise sat at 0.

  lead_close_accel_ms2 is a +a *ceiling*. A same-speed hang 2–4 m long of
  FD therefore stays at a_target=0 unless something commands the rematch
  trickle. Same when ego sags below the lead at/near FD. Ego clearly
  slower (`v_rel` ≲ −0.5) commands Accel 1 so grade can recover *under
  MAX* — the ceiling alone left the plant 0.015 m/s short. At/above
  MAX, do not raise +a for speed-sag / rematch (ea 11:46: Accel-1
  while already 1–2 mph over). Real overlay ease (|a| ≥ nibble) and
  emergency −a still win. Do not rematch into a ≳ 1.5 close.
  """
  if output_a is None or slack is None:
    return output_a
  if float(output_a) <= -LEAD_APPROACH_NIBBLE_MS2:
    return output_a
  if v_rel is not None and float(v_rel) >= LEAD_CLOSING_MATCH_MS:
    return output_a
  if lead_at_or_above_max(v_ego, v_cruise):
    return output_a
  s = float(slack)
  v = 0.0 if v_rel is None else float(v_rel)
  finish = 0.0 < s <= LEAD_SETTLE_FINISH_SLACK_M
  sag = 0.0 <= s <= LEAD_SETTLE_FINISH_SLACK_M and v < 0.0
  speed_sag = s >= 0.0 and v <= -LEAD_SETTLE_VREL_MS
  if finish or sag:
    return max(float(output_a), LEAD_CLOSE_OPENING_A_MS2)
  if speed_sag:
    return max(float(output_a), LEAD_CLOSE_A_MIN_MS2)
  return output_a


def slew_follow_plus_a(target, prev, v_rel, slew=LEAD_ATARGET_SLEW_MS2):
  """Rate-limit +a across cruise↔lead ownership flips.

  Closing ≳ 1.0 or a more-negative command is immediate so #190
  match-speed −a is not delayed. Positive a slews by `slew` per frame.
  First-latch uses slew_lead_acquire_a so a one-frame MPC punch cannot
  yo-yo regen→accel (e4 09:53:19).
  """
  if target is None:
    return target
  t = float(target)
  p = t if prev is None else float(prev)
  if lead_is_closing(v_rel) or t <= 0.0:
    return t
  if t > p:
    return min(t, p + float(slew))
  if t < p:
    return max(t, p - float(slew))
  return t


def lead_mpc_needs_full_authority(v_rel, d_rel, slack=None, allow_rapid=False,
                                 fcw=False, crash_cnt=0, confirm_rapid=True):
  """True when MPC −a must not be slewed or floored.

  Emergency only: FCW / crash / confirmed rapid close / near bumper
  (or already inside the stop gap). Mild town closes, near-gap
  match-speed, and slack ≤ 0 same-speed recovery stay on the MILD
  slight-lift path — EV regen from a slight lift is already firm.
  A one-frame v_rel blip still waits on `allow_rapid` when
  `confirm_rapid` (soft-limit anti-chatter). First-latch acquire
  passes `confirm_rapid=False` so a dumping lock bites immediately.
  """
  _ = slack
  if fcw or int(crash_cnt) > 0:
    return True
  if v_rel is not None and lead_approach_is_rapid(v_rel):
    if (not confirm_rapid) or allow_rapid:
      return True
  if d_rel is not None and float(d_rel) <= LEAD_MPC_SOFT_NEAR_M:
    return True
  if d_rel is not None and (float(d_rel) - STOP_DISTANCE) <= 0.0:
    return True
  return False


def update_lead_acquire(age, present, dt, hold_s=LEAD_ACQUIRE_HOLD_S):
  """Track time since first in-window latch. Reset on lead loss.

  Returns `(age, acquiring)`. Hold through status flicker (`present`
  includes the close-hold) so a radar dip does not re-slew acquire.
  """
  if not present:
    return 0.0, False
  nxt = float(age) + max(0.0, float(dt))
  return nxt, nxt <= float(hold_s)


def slew_lead_acquire_a(target, prev, v_rel, d_rel=None, slack=None,
                        acquiring=False, allow_rapid=False, fcw=False,
                        crash_cnt=0, a_lead=None, slew=LEAD_ACQUIRE_SLEW_MS2):
  """Slew first-latch aTarget both ways so MPC cannot punch regen→accel.

  e4 09:53:19: aTarget −0.46 then +0.05 in ~0.5 s at 118 m. Instant −a
  (slew_follow_plus_a) applied the punch; rematch slewed back over
  ~0.5 s. During the acquire window, step toward the new command.
  Rapid / near-bumper / FCW stay immediate. Closing ≳ 1.5 or a
  near-gap aLead does *not* skip the window — that punched −0.996
  at 80–130 m (10:48) and delayed only the comfort path. After the
  window, same as slew_follow_plus_a.
  """
  _ = a_lead
  if target is None:
    return target
  t = float(target)
  p = t if prev is None else float(prev)
  if (not acquiring) or lead_mpc_needs_full_authority(
    v_rel, d_rel, slack, allow_rapid=allow_rapid, fcw=fcw,
    crash_cnt=crash_cnt, confirm_rapid=False,
  ):
    return slew_follow_plus_a(t, p, v_rel, slew=slew)
  if t > p:
    return min(t, p + float(slew))
  if t < p:
    return max(t, p - float(slew))
  return t


def lead_close_should_cap(d_rel, model_prob=None, radar=None, active=False) -> bool:
  """True when a detected lead is in the close-cap window.

  Bosch ceiling matches ease (200 m). Far tracks need radar association
  so a 160 m vision flicker cannot cap empty-road climb. Radar-only
  far locks do cap. Missing quality args (unit tests) are ok.
  """
  if d_rel is None:
    return False
  d = float(d_rel)
  d_max = LEAD_CLOSE_MAX_M + (LEAD_APPROACH_MAX_HOLD_M if active else 0.0)
  if d <= 0.0 or d > d_max:
    return False
  return lead_approach_track_ok(d, model_prob, radar, active=active)


def resolve_lead_close_hold(status, d_rel, v_lead, held_d, held_v, held_age, dt,
                            hold_s=LEAD_CLOSE_HOLD_S, model_prob=None, radar=None):
  """Lead used for the +a close cap, with a brief hold on status flicker.

  Live in-window lead wins, including a far Bosch track (160–200 m).
  A dropped `leadOne.status` keeps the last in-window lead for `hold_s`
  so cruise 1.6 cannot punch through a flicker. Past Bosch, or a
  vision-only far flicker with no hold, drops so empty-road MAX-rise
  is not stuck capped.

  Returns `(d_use, v_use, held_d, held_v, held_age)`. `d_use` is None
  when the cap should not apply.
  """
  holding = held_d is not None
  if status and lead_close_should_cap(d_rel, model_prob, radar, active=holding) and v_lead is not None:
    d = float(d_rel)
    v = float(v_lead)
    return d, v, d, v, 0.0

  past_bosch = (
    d_rel is not None and float(d_rel) > LEAD_CLOSE_MAX_M + LEAD_APPROACH_MAX_HOLD_M
  )
  if past_bosch or held_d is None or held_v is None:
    return None, None, None, None, 0.0

  age = float(held_age) + float(dt)
  if age > float(hold_s):
    return None, None, None, None, 0.0
  return float(held_d), float(held_v), float(held_d), float(held_v), age


def lead_approach_track_ok(d_rel, model_prob=None, radar=None, active=False) -> bool:
  """Far-track anti-flicker. LeadData has modelProb + radar; no track age.

  Inside RELIABLE_M, `leadOne.status` is enough (planner already gated).
  Beyond it, enter needs a radar-associated lead (`radar=True`) so a
  160–200 m Bosch lock can ease / show immediately. Vision-only far
  flicker stays off. Missing quality args (unit kinematics) are ok.
  Once active, hold through a brief quality dip so far tracks do not
  chatter.
  """
  if d_rel is None:
    return False
  d = float(d_rel)
  if d <= LEAD_APPROACH_RELIABLE_M or active:
    return True
  if radar is False:
    return False
  # Solid Bosch association: ease / cap from first far lock. Do not wait
  # on modelProb — that delayed yellow-arrow reaction on radar-only tracks.
  # Vision-only far flicker still stays off.
  if radar is True:
    return True
  if model_prob is not None and float(model_prob) < LEAD_APPROACH_MODEL_PROB_MIN:
    return False
  return True


def lead_alead_owns_match(v_rel, a_lead, slack=None,
                         a_lead_ms2=LEAD_CLOSING_ALEAD_MS2) -> bool:
  """True when aLead alone may own match-speed −a / closing.

  aLead ≲ −0.2 used to own regardless of gap. A far or opening lead
  (da 2026-09-18: dRel~64 m, v_rel~−1.44 opening, aLeadK~−1.46) then
  snapped cruise +a to aLeadK. Gate: slack ≳ 20 m never owns from
  aLead alone. Opening owns only when slack is known and near the
  follow gap (braking-lead protection). Real closing ≳ 1.0–1.5 is a
  separate v_rel gate.
  """
  if a_lead is None or float(a_lead) > float(a_lead_ms2):
    return False
  if slack is not None and float(slack) >= LEAD_ALEAD_MATCH_SLACK_M:
    return False
  if v_rel is not None and float(v_rel) < 0.0:
    return slack is not None
  return True


def lead_is_closing(v_rel, a_lead=None, close_ms=LEAD_CLOSING_REMATCH_BLOCK_MS,
                    a_lead_ms2=LEAD_CLOSING_ALEAD_MS2, slack=None) -> bool:
  """True when a live/held lead is closing or a near-gap braking lead."""
  if v_rel is not None and float(v_rel) >= float(close_ms):
    return True
  return lead_alead_owns_match(v_rel, a_lead, slack, a_lead_ms2=a_lead_ms2)


def lead_owns_plan(v_rel, a_lead=None, slack=None) -> bool:
  """Hold-window planner ownership: last close ≥ 1.5 or near-gap aLead."""
  return lead_is_closing(v_rel, a_lead, close_ms=LEAD_CLOSING_MATCH_MS, slack=slack)


def cap_closing_lead_accel(output_a, v_rel, a_lead=None, lead_present=False,
                           owned=False, slack=None, d_rel=None):
  """Never rematch +a into a closing / near-gap braking live or held lead.

  Closing ≳ 1.0 m/s (or hold-owned) hard-caps a at 0, except large-gap
  catch-up (#187) while closing 1.0–1.5. Closing ≳ 1.5 prefers
  match-speed −a (`aLead − k·v_rel`). aLead ≲ −0.2 matches only near
  the follow gap — not when the gap is opening or slack is large.
  Same-speed far catch-up +a is unchanged. Past Bosch, do not apply
  match-speed −a (no extra crawl on a 215 m lock). Inside FD, a slow
  close commands the MILD floor, not rematch +a.
  """
  if output_a is None or not (lead_present or owned):
    return output_a
  if d_rel is not None and float(d_rel) > LEAD_CLOSE_MAX_M + LEAD_APPROACH_MAX_HOLD_M:
    return float(output_a)
  v = 0.0 if v_rel is None else float(v_rel)
  # Inside FD, still closing: never rematch +a. Slow close commands
  # the MILD floor; 0.5–0.8 coasts (glide owns match). Rapid dumps.
  if slack is not None and float(slack) <= 0.0 and v >= LEAD_SETTLE_VREL_MS:
    a = min(float(output_a), 0.0)
    a_slow = lead_inside_slow_close_a_ms2(v_rel, slack)
    if a_slow is not None:
      a = min(a, a_slow)
    if not lead_approach_is_rapid(v):
      return a
  if not (owned or lead_is_closing(v_rel, a_lead, slack=slack)):
    return float(output_a)
  near_finish = slack is not None and 0.0 < float(slack) <= LEAD_SETTLE_FINISH_SLACK_M
  large_gap = slack is not None and float(slack) > LEAD_CLOSE_REMATCH_SLACK_M
  if (near_finish or large_gap) and (not owned) and v < LEAD_CLOSING_MATCH_MS:
    return float(output_a)
  a = min(float(output_a), 0.0)
  # Extra match-speed −a is emergency only (rapid / near bumper).
  # Mild closing ≳ 1.5 or near-gap aLead must not undo the MILD
  # slight-lift floor. Still cap +a at 0 so rematch cannot punch
  # into a shrinking gap.
  rapid = v >= LEAD_APPROACH_RAPID_DV_MS
  near_bumper = d_rel is not None and float(d_rel) <= LEAD_MPC_SOFT_NEAR_M
  match_speed = rapid or near_bumper
  if match_speed:
    a_k = 0.0 if a_lead is None else float(a_lead)
    a = min(a, a_k - LEAD_CLOSING_MATCH_GAIN * max(0.0, v), 0.0)
  return a


def lead_approach_ttc_s(slack, v_rel) -> float:
  """Seconds to the Follow Distance gap at the current closing speed."""
  s = float(slack)
  v = float(v_rel)
  if s <= 0.0 or v <= 0.0:
    return float("inf")
  return s / v


def lead_approach_is_rapid(v_rel, ttc=None) -> bool:
  """True when closing rate is high enough for the firm 0.55 path.

  Short TTC at a mild `v_rel` is arriving at Follow Distance, not dumping.
  `ttc` is for callers; it does not promote a 5–10 mph close to 0.55.
  Planner still requires consecutive samples via lead_approach_rapid_gate.
  """
  _ = ttc
  return float(v_rel) >= LEAD_APPROACH_RAPID_DV_MS


def lead_approach_rapid_gate(v_rel, prev_count, need_n=LEAD_APPROACH_RAPID_CONFIRM_N,
                             sample_ok=True):
  """Confirm rapid close over consecutive in-window frames.

  One outlier / far flicker does not commit. Mild ease does not wait.
  Returns `(allow_rapid, new_count)`. Non-rapid, missing v_rel, or
  `sample_ok=False` (overlay not in play) resets the count.
  """
  if (not sample_ok) or v_rel is None or not lead_approach_is_rapid(v_rel):
    return False, 0
  count = int(prev_count) + 1
  return count >= int(need_n), count


def soft_limit_mpc_a_target(output_a, v_ego, v_lead, d_rel, fcw=False, crash_cnt=0,
                            allow_rapid=False, a_lead=None, slack=None,
                            prev_floored=False):
  """Floor non-emergency MPC −a to slight-lift MILD.

  Overlay min(MPC, mild) cannot stop MPC commanding ~−2.5 on radar noise
  or a mild town close. Apply this to the MPC (and map) command *before*
  the overlay so a confirmed rapid 0.55 path is not also floored.

  Comfort path is always MILD. Closing ≳ 1.5 / near-gap aLead / 10 mph
  town entry must not skip the floor — EV slight lift already regen-
  erates hard. Rapid / near-bumper / FCW / crash own danger. A
  one-frame v_rel blip still waits on `allow_rapid`.
  """
  _ = a_lead
  _ = prev_floored
  if output_a is None:
    return output_a
  a = float(output_a)
  if a >= -LEAD_APPROACH_MILD_A_MS2:
    return a
  if d_rel is None or v_lead is None or v_ego is None:
    return a
  v_rel = float(v_ego) - max(0.0, float(v_lead))
  if lead_mpc_needs_full_authority(
    v_rel, d_rel, slack, allow_rapid=allow_rapid, fcw=fcw,
    crash_cnt=crash_cnt, confirm_rapid=True,
  ):
    return a
  return -LEAD_APPROACH_MILD_A_MS2


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


def slew_lead_approach_a(target, prev, slew=LEAD_APPROACH_SLEW_MS2,
                         release_slew=LEAD_APPROACH_RELEASE_SLEW_MS2):
  """Ramp overlay a both ways. Onset is gradual; rematch is not a slam.

  More-negative uses `slew` (Tesla regen bite). Milder / off steps toward
  0 at `release_slew` so gap-opening rematch trickles instead of jumping
  regen → +a. Fully released (at/above 0) is None.
  """
  p = 0.0 if prev is None else float(prev)
  if target is None:
    if prev is None or p >= -1e-6:
      return None
    nxt = p + float(release_slew)
    return None if nxt >= -1e-6 else nxt
  t = float(target)
  if t < p:
    return max(t, p - float(slew))
  if t > p:
    return min(t, p + float(release_slew))
  return t


def apply_lead_approach_overlay(output_a, a_lead, nibble=LEAD_APPROACH_NIBBLE_MS2,
                                v_rel=None, slack=None):
  """Soft overlay via min(), except a far nibble must not steal cruise +a.

  Matching-traffic / far-slack overlay sits at |a| ~0.06–0.13. That must
  not beat rematch / cruise / MAX +a on a *large same-speed* gap close.
  Real ease (|a| ≥ nibble) and MPC 0 / −a still use min().

  Near the follow gap, a nibble min()s so we mesh into lead speed at the
  set gap and release slew is not a regen→Accel punch. Large-gap catch-up
  closing 1.0–1.5 may still keep +a (#187). Match-speed ≳ 1.5 always
  min()s — never keep rematch +a into a hard close.
  """
  if a_lead is None:
    return float(output_a)
  out = float(output_a)
  a = float(a_lead)
  if out <= 0.0 or a <= -float(nibble):
    return min(out, a)
  v = 0.0 if v_rel is None else float(v_rel)
  large_gap = slack is not None and float(slack) > LEAD_CLOSE_REMATCH_SLACK_M
  if lead_is_closing(v_rel) and not (large_gap and v < LEAD_CLOSING_MATCH_MS):
    return min(out, a)
  near = slack is not None and float(slack) <= LEAD_CLOSE_REMATCH_SLACK_M
  if near:
    return min(out, a)
  return out


def lead_approach_decel_ms2(v_ego, v_lead, d_rel, t_follow, a_comfort=LEAD_APPROACH_A_MS2,
                           active=False, model_prob=None, radar=None, allow_rapid=False):
  """Comfort decel to close onto the Follow Distance gap, or None.

  a = -v_rel² / (2 * slack) so we arrive at the selected gap with matching
  speed. |a| at the open is below the comfort peak. Mild closes stay at
  MILD (light regen) and do not wait on the rapid gate. Rapid closes
  reach the 0.55 peak only after the planner confirms (`allow_rapid`);
  default is off so a single v_rel blip stays mild. None when speeds
  match, the lead is faster, or the lead is still outside the window.

  `active` is last frame's *kinematic* overlay (hysteresis), not release
  slew. Enter uses DV_MS / SLACK_ON; hold uses DV_OFF / SLACK_OFF /
  need+NEED_HOLD so small radar noise does not chatter regen ↔ accel.

  When `v_rel` is clearly positive (≥ CLEAR_DV) on a radar lock, large
  slack is allowed — speed-match from the first reasonable track, still
  capped (mild / rapid 0.55) and slewed. Vision-only flicker still waits
  on the need window. Overlay apply must not turn a far nibble into
  cruise-killing −a; mesh regen is near the set gap.
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
  # Radar lock only: a vision flicker must not early-start past need.
  clear = v_rel >= LEAD_APPROACH_CLEAR_DV_MS and radar is not False
  if not clear:
    need_m = lead_approach_need_m(v0, vt, a_comfort, t_follow)
    need_gate = need_m + (LEAD_APPROACH_NEED_HOLD_M if active else 0.0)
    if slack > need_gate:
      return None
  kin_slack = lead_kinematic_slack_m(slack, v_rel)
  if kin_slack <= 0.0:
    return None
  a_needed = -(v_rel * v_rel) / (2.0 * kin_slack)
  ttc = lead_approach_ttc_s(slack, v_rel)
  rapid = bool(allow_rapid) and lead_approach_is_rapid(v_rel, ttc)
  a_cap = float(a_comfort) if rapid else LEAD_APPROACH_MILD_A_MS2
  return max(float(a_needed), -float(a_cap))
