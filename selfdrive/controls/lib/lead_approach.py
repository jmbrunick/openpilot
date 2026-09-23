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
empty-road climb. Non-rapid MPC −a is floored at MILD (slight lift)
when matched or slow-close. On an owned / path-synced lead,
residual close (worsened beyond ego a) or near-gap aLead
(slack ≲ 20 m) skips that floor immediately so a held lead
cannot pin −0.22 (07:55: closing 1.8→4.4, aLead ~−1, dRel
38→25 under the 6 m/s rapid gate). Closing ≥ 1.5 still
reacts. Large-slack small adjustments (e4 −2.33) and far /
opening aLead stay floored. Firm 0.55 / full hard-brake still
wait on confirmed rapid / near-bumper / FCW.

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

After acquire, a mid-gap cruise rematch still pulsed Accel +a while
slowly closing (ef 10:18: slack ~12–50 m, v_rel ≈ 1.4, +0.25 ↔ −0.02
every 2–4 s). Soft-cap that rematch to a trickle. Keep Accel for
true catch-up (opening / same-speed, or slack > 50 — large-gap
rematch / recovery). Post-acquire, slew small ±a both ways so tiny
cruise bites cannot flip gas↔regen. When planner aTarget is ~0, do
not let the plant dump firm regen (ef 10:18:42: act −1.23 then
rematch +0.40). Grade / pitch hold and planner ≤ −0.5 / rapid / FCW
keep full −a/+a.

Map FOLLOW above MAX used to skip that MILD floor when the lead was
not classified closing, so a steady mid-gap radar lead (slack ~19 m,
closing ~0.5) took the raw cruise cliff (−1.4…−3.5) and then Accel
climbed back through the limit. Comfort-floor that mid-gap sample.
After the cliff, hold the 0.10 rematch cap through the opening
re-catch so Accel cannot relight the next dump. Large-gap / no-lead
catch-up stays Accel. FCW, confirmed rapid, and near-bumper stay raw.

The same mid-gap geometry under the limit (Scallywag 18:09, ego 1–3
mph under 70) still cliffed: post-MPC aTarget went to −1.4…−3.5 while
plan accels stayed ~0, and the Pre-AP pedal plant full-lifted to the
regen rail on a coasting / MILD command. Floor that coasting mid-gap
sample under or over the map (MILD under, map comfort −0.55 over).
Match-speed close ≥ 1.5 stays raw. Near-gap match-aLead (slack ≲ 15 m,
aLead ≲ −0.2, including a hold-owned brake) stays raw while the plan
horizon is still cruise — that is lead braking, not the slack-~19 m
cliff. Expand the plant steady band through MILD so a commanded slight
lift cannot unlock the regen rail.

Near Follow Distance the same coasting-plan cliff still fired under the
mid-gap floor (Scallywag 23:16–23:22: slack ~2–8 m, closing ≲ 0.7,
aLead ~0, plan_min ~0, aTarget −2.9…−3.5 for a frame). Floor that to
MILD. Do not floor when the lead is actually braking: closing ≳ 1 m/s,
aLead ≲ −0.25, #222 residual / rapid already armed, FCW, shouldStop,
or near-bumper. The 23:31 hard brake (aLead ~−1.2, closing ~1.1) stays
raw.

Scallywag 09:57 / 10:05: an owned on-path lead with positive slack was
opening (radar vRel > 0) while filtered aLeadK sat slightly negative,
and firm −a (about −3.5) stayed on because aLead alone skipped the
mild floor. Clearly opening kinematics are not that brake. aLeadK may
corroborate a close; it must not own firm −a while the gap is opening
with slack left. Several samples of opening vRel and a growing gap
latch the release so one jitter frame cannot re-arm it. Inside the
follow gap, a real close, and #222 residual / rapid close stay firm.

08:57:50–52: a real on-path close (yRel ~0.5, closing ~4 m/s) then
walked off to |yRel| ~4.6 with slack still ~10–15 m while aTarget
held −3.5. Past ~2.25 m lateral (or moving away past 1.5 m) with
slack ≳ 8 m, fade that firmness within ~0.4 s. |yRel| ≲ 1.5 and
still closing keeps full firmness.
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
# Owned-lead rising close. Above rematch-enter jitter (0.55) so a
# matched follow whose closing starts climbing is the brake signal
# before 1.5. One-frame radar chatter must not count as a rise.
LEAD_SOFT_LIMIT_RISE_MS = 0.15
# Unexplained closing accel after subtracting ego's own a. Same
# spirit as aLead ≤ −0.2: residual ≥ 0.2 means the lead is braking.
LEAD_SOFT_LIMIT_RESIDUAL_MS2 = 0.2
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
# Mid-gap rematch while still slowly closing (ef 10:18). Slack above
# the near-gap rematch band and not past 50 m, closing ~0.8–2.0 m/s:
# trickle, not Accel ceil. Slack > 50 is large-gap catch-up (Accel),
# even while still closing in-band. Opening / same-speed is Accel.
# Rapid / match-speed ≥ 1.5 still owns −a.
LEAD_MID_GAP_SLACK_M = 50.0
LEAD_MID_GAP_CLOSE_LO_MS = 0.8
LEAD_MID_GAP_CLOSE_HI_MS = 2.0
LEAD_MID_GAP_REMATCH_A_MS2 = 0.10
# Map FOLLOW above MAX with a steady mid-gap lead (Scallywag 16:22:
# slack ~19 m, dRel ~60 m, closing ~0.5). Do not pass a −1.4…−3.5
# cruise/map cliff through. Comfort floor is the early map brake
# (−0.55), inside −0.4…−0.6, so the limit still comes back.
# Slack ~8–50 m. Farther than that, map decel still mins in.
LEAD_MAP_MIDGAP_SLACK_LO_M = 8.0
LEAD_MAP_MIDGAP_FLOOR_MS2 = 0.55
LEAD_MAP_MIDGAP_DREL_HI_M = 80.0
# After a cliff, hold the 0.10 rematch cap through the opening
# re-catch so Accel cannot climb back through the limit. Raw |a|
# at or below this arms the hold; MILD and the comfort floor do not.
LEAD_POST_DUMP_A_MS2 = 1.0
LEAD_POST_DUMP_HOLD_S = 12.0
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
# Post-acquire small ±a chatter (ef 10:18 rematch ↔ ~0). Wider than
# the 0.75 s first-latch window. Firm −a / rematch ceil sit outside.
LEAD_FOLLOW_CHATTER_LO_MS2 = -0.08
LEAD_FOLLOW_CHATTER_HI_MS2 = 0.32
LEAD_FOLLOW_CHATTER_SLEW_MS2 = LEAD_NEAR_GAP_SLEW_MS2
LEAD_FOLLOW_CHATTER_DEADBAND_MS2 = 0.03
# Plant regen dump while the planner is coasting or easing. The steady
# band used to be |a| ≤ 0.08, so a commanded MILD (−0.22) unlocked the
# Pre-AP regen rail (18:10:08: aTarget −0.218, act −1.32). Cover
# |aTarget| ≤ MILD + ε so slight lift cannot full-lift. Planner ≤ −0.5
# / FCW / confirmed rapid / near-bumper still pass through.
LEAD_FOLLOW_STEADY_EPS_MS2 = 0.03
LEAD_FOLLOW_STEADY_A_MS2 = LEAD_APPROACH_MILD_A_MS2 + LEAD_FOLLOW_STEADY_EPS_MS2
LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2 = -LEAD_APPROACH_MILD_A_MS2
LEAD_FOLLOW_ACT_REGEN_CMD_MS2 = -0.50
# Plan accels at or above this (less negative) are "≈0" for the
# mid-gap coast floor. A real MPC brake below it stays raw.
LEAD_PLAN_COAST_A_MS2 = 0.15
# Near-FD settle, under the #232 mid-gap floor (slack ≳ 8 m).
# Scallywag 23:16–23:22: slack ~2–8 m, closing ≲ 0.7, aLead ~0,
# plan still coasting, aTarget steps to ACCEL_MIN for a frame.
# Closing at or above ~1 m/s, or aLead at or below ~−0.25, is a
# real brake (23:31 KEEP) and stays raw.
LEAD_NEAR_FD_COAST_CLOSE_MS = 1.0
LEAD_NEAR_FD_COAST_ALEAD_MS2 = -0.25
# Internal v_rel = v_ego − v_lead (positive closes). Radar vRel is the
# opposite sign. Clearly opening, past match noise: 10:05 onset was
# about −0.56 (radar +0.56) with slack ~9 m and aLeadK only −0.32.
# A single sample milder than this can still be near-gap noise; a
# run of any opening vRel latches the release below.
LEAD_OPENING_VREL_MS = -0.25
# Several planner frames (~0.35 s, inside the 0.3–0.5 s fade) of
# opening vRel and a non-shrinking gap. Then firm −a stays off until
# the gap is no longer opening.
LEAD_OPENING_RELEASE_S = 0.35
# dRel radar noise. A real close of ≳ 4 m/s still shrinks faster.
LEAD_OPENING_GROW_TOL_M = 0.20
# Departing cut-in (08:57:50–52). Full firmness at |yRel| ≲ 1.5 while
# closing. Past ~2.25 m, or moving away through 1.5–2.25, with slack
# ≳ 8 m: fade within the same ~0.35 s. Not a near-bumper gate.
LEAD_DEPART_YREL_FIRM_M = 1.5
LEAD_DEPART_YREL_M = 2.25
LEAD_DEPART_SLACK_M = 8.0
LEAD_DEPART_RELEASE_S = 0.35
LEAD_DEPART_YREL_GROW_M = 0.05
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
                          slew=LEAD_NEAR_GAP_SLEW_MS2, opening_release=False,
                          depart_release=False):
  """Slew small near-gap ±a so floor↔release cannot step 0.3 in one frame.

  Full authority (rapid / bumper / FCW / hard kinematics) is immediate.
  Far slack and bites outside the small band pass through. First-latch
  acquire slew is a separate path and must stay unchanged.

  An opening or departing release must publish the mild floor on this
  frame. A 0.02 step from a small +a lands on 0, and the follow-chatter
  deadband then holds that previous ~0 forever (10:05 published 0.02
  instead of −0.22). On-path close does not set these latches.
  """
  if target is None:
    return target
  t = float(target)
  if opening_release or depart_release:
    return t
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


def _follow_chatter_bite(a) -> bool:
  return LEAD_FOLLOW_CHATTER_LO_MS2 <= float(a) <= LEAD_FOLLOW_CHATTER_HI_MS2


def slew_follow_chatter_a(target, prev, v_rel, d_rel=None, slack=None,
                          allow_rapid=False, fcw=False, crash_cnt=0,
                          slew=LEAD_FOLLOW_CHATTER_SLEW_MS2, catchup=False,
                          opening_release=False, depart_release=False):
  """Slew small post-acquire ±a so rematch ↔ ~0 cannot flip gas↔regen.

  After the 0.75 s first-latch window, cruise still bit +0.15…+0.27
  then dropped to −0.02 every few seconds (ef 10:18). Both-ways step
  plus a tiny deadband around 0 hold aTarget. Rapid / bumper / FCW /
  bites outside the chatter band are immediate. First-latch acquire
  slew is a separate, faster path. Latched catch-up / too-close
  recovery is immediate so settle cannot hold a wrong gap.

  Opening / departing release is also immediate. The deadband must
  not keep a ~0 coast once firm authority has faded to the mild floor.
  """
  if target is None:
    return target
  t = float(target)
  if opening_release or depart_release:
    return t
  if catchup or (slack is not None and float(slack) < 0.0):
    return t
  if lead_mpc_needs_full_authority(
    v_rel, d_rel, slack, allow_rapid=allow_rapid, fcw=fcw,
    crash_cnt=crash_cnt, confirm_rapid=True,
  ):
    return t
  if prev is None:
    return t
  p = float(prev)
  if not (_follow_chatter_bite(t) and _follow_chatter_bite(p)):
    return t
  if (abs(t) <= LEAD_FOLLOW_CHATTER_DEADBAND_MS2
      and abs(p) <= LEAD_FOLLOW_CHATTER_DEADBAND_MS2):
    return p
  if t > p:
    return min(t, p + float(slew))
  if t < p:
    return max(t, p - float(slew))
  return t


def lead_midgap_comfort_excluded(v_rel, d_rel, slack, fcw=False, crash_cnt=0,
                                allow_rapid=False) -> bool:
  """True when a mid-gap comfort cap must not apply.

  FCW, confirmed rapid, near-bumper, and match-speed close ≥ 1.5 stay
  raw. Outside the slack ~8–50 m band (or the dRel stand-in) is not
  this settle. |v_rel| ≥ ~2 m/s is not a slow close.
  """
  if fcw or int(crash_cnt) > 0:
    return True
  if d_rel is not None and float(d_rel) <= LEAD_MPC_SOFT_NEAR_M:
    return True
  if d_rel is not None and (float(d_rel) - STOP_DISTANCE) <= 0.0:
    return True
  if (allow_rapid and v_rel is not None
      and lead_approach_is_rapid(float(v_rel))):
    return True
  if not lead_mid_gap_map_band(slack, d_rel):
    return True
  if v_rel is None:
    return False
  v = float(v_rel)
  if v >= LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS:
    return True
  if abs(v) >= LEAD_MID_GAP_CLOSE_HI_MS:
    return True
  return False


def lead_near_gap_alead_raw(v_rel, a_lead, slack, owned=False) -> bool:
  """True when near-gap lead braking must stay raw.

  Slack at or under the near-gap band with a meaningful negative aLead
  is match-aLead (dRel = follow + 8 m, closing ~0.4 m/s, aLeadK −0.80)
  even while the MPC horizon is still cruise. Hold-owned is the same
  brake. A slack-~19 m cliff (−1.4…−3.5) with a coasting plan is not
  this path. Clearly opening with positive slack is not this path
  either — filtered aLeadK does not keep the cliff.
  """
  if slack is None or float(slack) > LEAD_NEAR_GAP_SLACK_M:
    return False
  if lead_kinematics_opening(v_rel, slack):
    return False
  if lead_alead_owns_match(v_rel, a_lead, slack):
    return True
  if not owned or a_lead is None:
    return False
  return float(a_lead) <= LEAD_CLOSING_ALEAD_MS2


def guard_follow_actuator_regen(actuator_a, planner_a, v_rel=None, d_rel=None,
                                slack=None, fcw=False, crash_cnt=0,
                                allow_rapid=False, v_ego=None, v_cruise=None,
                                a_lead=None, owned=False):
  """Do not dump firm plant regen on a coast or a mild ease.

  ef 10:18:42: aTarget ≈ 0 while actuators.accel hit −1.23. 18:09 /
  18:10: a coasting or MILD aTarget still reached the Pre-AP regen rail.
  When |aTarget| is inside the steady band (through MILD + ε), clip
  the actuator to the MILD slight-lift floor. Mid-gap slow close /
  settle hard-caps to that floor even if the planner command is
  already a cliff — PID / feedforward windup must not full-lift.
  Near-gap match-aLead is a real brake and is not that hard cap.
  Planner ≤ −0.5 outside that settle, FCW, confirmed rapid, and
  near-bumper pass through.
  """
  if actuator_a is None or planner_a is None:
    return actuator_a
  a = float(actuator_a)
  p = float(planner_a)
  floor = LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2
  emergency = (
    fcw or int(crash_cnt) > 0
    or (d_rel is not None and float(d_rel) <= LEAD_MPC_SOFT_NEAR_M)
    or (allow_rapid and v_rel is not None and lead_approach_is_rapid(float(v_rel)))
  )
  if emergency:
    return a
  if (not lead_near_gap_alead_raw(v_rel, a_lead, slack, owned=owned)
      and not lead_midgap_comfort_excluded(
        v_rel, d_rel, slack, fcw=False, crash_cnt=0, allow_rapid=False,
      )):
    # Under the map, slight lift. Over the map, keep the #231 comfort
    # brake (−0.55) so a limit return is not lifted back to MILD.
    if lead_map_decel_above_max(v_ego, v_cruise):
      floor = -LEAD_MAP_MIDGAP_FLOOR_MS2
    return a if a >= floor else floor
  if p <= LEAD_FOLLOW_ACT_REGEN_CMD_MS2:
    return a
  if abs(p) > LEAD_FOLLOW_STEADY_A_MS2:
    return a
  # Coast allows a slight lift. A command already at or below that
  # floor is tracked, but the plant may not go past it to the rail.
  allowed = p if p < floor else floor
  return a if a >= allowed else allowed


def plan_horizon_is_coasting(accels, coast_ms2=LEAD_PLAN_COAST_A_MS2) -> bool:
  """True when the published plan accel horizon is not a real brake."""
  if accels is None:
    return False
  vals = [float(a) for a in list(accels)]
  if not vals:
    return False
  return min(vals) >= -float(coast_ms2)


def floor_midgap_coast_a_target(a, plan_coasting, v_rel, d_rel, slack,
                                v_ego=None, v_cruise=None, fcw=False,
                                crash_cnt=0, allow_rapid=False,
                                a_lead=None, owned=False):
  """Floor a post-MPC cliff while the plan itself is coasting.

  Under the map the floor is MILD (slight lift). Over the map it is
  the #231 comfort brake (−0.55) so the limit can still come back.
  FCW, confirmed rapid, near-bumper, and match-speed close ≥ 1.5 stay
  raw. Near-gap aLead match / hold-owned lead braking stays raw too.
  A plan horizon that is already braking is not this path.
  """
  if a is None or not plan_coasting:
    return a
  if lead_near_gap_alead_raw(v_rel, a_lead, slack, owned=owned):
    return a
  if lead_midgap_comfort_excluded(
    v_rel, d_rel, slack, fcw=fcw, crash_cnt=crash_cnt, allow_rapid=allow_rapid,
  ):
    return a
  out = float(a)
  if lead_map_decel_above_max(v_ego, v_cruise):
    floor = -LEAD_MAP_MIDGAP_FLOOR_MS2
  else:
    floor = -LEAD_APPROACH_MILD_A_MS2
  if out >= floor:
    return out
  return floor


def lead_near_fd_settle_band(slack) -> bool:
  """True for slack inside Follow Distance's settle, under the mid-gap floor.

  0 m up to but not including the #232 mid-gap start (8 m). That band
  already has its own coast floor. Negative slack is a too-close
  recovery and stays on that path.
  """
  if slack is None:
    return False
  s = float(slack)
  return 0.0 <= s < LEAD_MAP_MIDGAP_SLACK_LO_M


def near_fd_coast_floor_applies(plan_coasting, v_rel, d_rel, slack, *,
                                fcw=False, crash_cnt=0, allow_rapid=False,
                                a_lead=None, owned=False, should_stop=False,
                                prev_v_rel=None, a_ego=None, dt=None,
                                opening_release=False, depart_release=False) -> bool:
  """True only for a coasting near-FD settle, not a #222 firm match.

  All of: plan horizon coasting, owned lead, slack in the near-FD
  band, closing under ~1 m/s, aLead milder than ~−0.25, and none of
  FCW / crash / shouldStop / near-bumper / confirmed rapid / the
  existing residual-close skip. A braking lead (23:31: aLead ~−1.2,
  closing ~1.1) does not match.
  """
  if not plan_coasting or not owned or should_stop:
    return False
  if fcw or int(crash_cnt) > 0:
    return False
  if not lead_near_fd_settle_band(slack):
    return False
  if d_rel is not None and float(d_rel) <= LEAD_MPC_SOFT_NEAR_M:
    return False
  if allow_rapid:
    return False
  if v_rel is None:
    return False
  v = float(v_rel)
  if lead_approach_is_rapid(v) or v >= LEAD_NEAR_FD_COAST_CLOSE_MS:
    return False
  if a_lead is None:
    return False
  # Slightly negative aLeadK on an opening gap is not a brake. A
  # closing lead at the brake line (23:31) still stays raw.
  if (float(a_lead) <= LEAD_NEAR_FD_COAST_ALEAD_MS2
      and not lead_kinematics_opening(v, slack)):
    return False
  # #222 residual / rising close / near-gap aLead match. Do not put
  # the mild floor back on a path that already released it. An
  # opening or departing release is the opposite: the floor stays.
  if lead_soft_limit_skip(
    v, a_lead, slack, owned=True, acquiring=False,
    prev_v_rel=prev_v_rel, a_ego=a_ego, dt=dt,
    opening_release=opening_release, depart_release=depart_release,
  ):
    return False
  return True


def floor_near_fd_coast_a_target(a, plan_coasting, v_rel, d_rel, slack, *,
                                 fcw=False, crash_cnt=0, allow_rapid=False,
                                 a_lead=None, owned=False, should_stop=False,
                                 prev_v_rel=None, a_ego=None, dt=None,
                                 opening_release=False, depart_release=False):
  """Floor a one-frame ACCEL_MIN cliff while near-FD settle is coasting.

  Publishes MILD this frame. Slewing down from ACCEL_MIN would still
  command a firm brake on the frame the cliff appears. Anything already
  milder than MILD is left alone. #222 geometry stays raw.
  """
  if a is None or not near_fd_coast_floor_applies(
    plan_coasting, v_rel, d_rel, slack, fcw=fcw, crash_cnt=crash_cnt,
    allow_rapid=allow_rapid, a_lead=a_lead, owned=owned,
    should_stop=should_stop, prev_v_rel=prev_v_rel, a_ego=a_ego, dt=dt,
    opening_release=opening_release, depart_release=depart_release,
  ):
    return a
  out = float(a)
  floor = -LEAD_APPROACH_MILD_A_MS2
  if out >= floor:
    return out
  return floor


def plant_regen_effort_limits(a_cmd, limits):
  """VirtualDAS effort bounds while the command is coast or mild.

  Returns `limits` unchanged when the command is a firm brake. A
  coasting / MILD command cannot use the regen rail: the lower bound
  rises to the slight-lift floor. An existing tighter bound (engage
  grace) is kept.
  """
  if a_cmd is None:
    return limits
  p = float(a_cmd)
  # Deeper than map comfort is a real brake: leave the regen rail open.
  if p < -LEAD_MAP_MIDGAP_FLOOR_MS2 - 0.05:
    return limits
  mild = LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2
  if p >= 0.0 or abs(p) <= LEAD_FOLLOW_STEADY_A_MS2:
    # Coast / throttle: slight lift only. A mild negative command is
    # tracked if it is already under that floor.
    floor = p if p < mild else mild
  else:
    # Between MILD and map comfort: track the command, do not open the rail.
    floor = p
  if limits is None:
    from opendbc.car.tesla.preap.nap_conf import ACCEL_MAX
    return (floor, float(ACCEL_MAX))
  lo, hi = float(limits[0]), float(limits[1])
  return (max(lo, floor), hi)


def install_preap_plant_regen_guard():
  """Keep Pre-AP pedal effort on a slight lift when the command is mild.

  LongControl is pure feedforward. The regen rail is VirtualDAS effort
  (grade + inner integral) after that seam. Card and the controller
  share a process, so the cap is installed on VirtualDAS.update.
  """
  from opendbc.car.tesla.pedal.controller import PEDAL_RAMP_RATE_UP
  from opendbc.car.tesla.preap.virtual_das import VirtualDAS

  current = VirtualDAS.update
  if getattr(current, "_nap_plant_regen_guard", False):
    return

  def update(self, a_cmd, v_ego, prev_pedal_di, a_ego=0.0, freeze_integrator=False,
             orientation_ned=None, accel_effort_limits=None,
             pedal_ramp_rate_up=PEDAL_RAMP_RATE_UP):
    return current(
      self, a_cmd, v_ego, prev_pedal_di, a_ego=a_ego,
      freeze_integrator=freeze_integrator, orientation_ned=orientation_ned,
      accel_effort_limits=plant_regen_effort_limits(a_cmd, accel_effort_limits),
      pedal_ramp_rate_up=pedal_ramp_rate_up,
    )

  update._nap_plant_regen_guard = True
  VirtualDAS.update = update


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


def lead_map_decel_above_max(v_ego, v_cruise) -> bool:
  """True when map is commanding over-MAX decel (past the deadband).

  lead_at_or_above_max includes sitting *at* MAX. First-latch acquire
  slew and the MILD floor must still apply there (e4 09:53:19). Map
  brake only starts once ego is faster than MAX by TRACK_DEADBAND —
  same edge as map_track_decel_ms2.
  """
  if v_ego is None or v_cruise is None:
    return False
  if float(v_ego) <= 0.0 or float(v_cruise) <= 0.0:
    return False
  return float(v_ego) > float(v_cruise) + TRACK_DEADBAND_MS


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


def lead_mid_gap_map_band(slack, d_rel=None) -> bool:
  """True when a live lead is in the mid-gap band for map comfort.

  Slack ~8–50 m when known (dig slack ~19 m). Without slack, dRel
  past the bumper and not a far lock (dig ~60 m) still qualifies.
  Near-bumper and large-gap catch-up stay out.
  """
  if slack is not None:
    s = float(slack)
    return LEAD_MAP_MIDGAP_SLACK_LO_M <= s <= LEAD_MID_GAP_SLACK_M
  if d_rel is None:
    return False
  d = float(d_rel)
  return LEAD_MPC_SOFT_NEAR_M < d <= LEAD_MAP_MIDGAP_DREL_HI_M


def lead_mid_gap_slow_close(v_rel, slack) -> bool:
  """True when mid-gap rematch should trickle, not Accel-ceil.

  Slack in the 12–50 m rematch band and still closing ~0.8–2.0 m/s
  (ef 10:18 yo-yo). Same-speed / opening, or slack > 50 (large-gap
  rematch / follow recovery), stay Accel catch-up. Closing ≥ 2.0 is
  ownership, not rematch. A latched same-speed / opening catch-up
  (`lead_mid_gap_catchup_latch`) also keeps Accel through this band.
  """
  if v_rel is None or slack is None:
    return False
  v = float(v_rel)
  s = float(slack)
  if s <= LEAD_CLOSE_REMATCH_SLACK_M or s > LEAD_MID_GAP_SLACK_M:
    return False
  return LEAD_MID_GAP_CLOSE_LO_MS <= v < LEAD_MID_GAP_CLOSE_HI_MS


def lead_mid_gap_catchup_latch(prev, v_rel, slack, prev_slack=None,
                               settled=False) -> bool:
  """Hold Accel catch-up through the 12–50 m slow-close rematch band.

  Same-speed / opening / barely-closing with slack > 12 latches so a
  100 m start can finish Follow Distance. Inside FD (too-close) also
  latches so recovery cannot settle-and-hold while the gap is wrong.
  Already closing 0.8–2.0 in-band without that latch (ef 10:18) stays
  trickle. A *settled* follow that opens or hunts is not catch-up —
  keep the rematch deadband (d7 20:25:33). Keep the latch through
  slack ≤ 12 after too-close / large-gap catch-up so settle cannot
  freeze rematch while the gap is still wrong.
  """
  if v_rel is None or slack is None:
    return False
  v = float(v_rel)
  s = float(slack)
  if s < 0.0:
    return True
  if s <= LEAD_CLOSE_REMATCH_SLACK_M:
    return bool(prev)
  if settled:
    return False
  if prev_slack is not None and float(prev_slack) < 0.0:
    return True
  if v < LEAD_MID_GAP_CLOSE_LO_MS:
    return True
  return bool(prev)


def lead_close_accel_ms2(accel_level: int = 5, v_rel=None, slack=None,
                         a_personality=None, settled=False, v_ego=None,
                         v_cruise=None, catchup=False, post_dump=False) -> float:
  """Max positive a (m/s²) when closing the gap on a radar lead.

  Same Accel 1–10 envelope as open-road / MAX climb — not a separate
  hotter (or cooler) catch-up curve. `a_personality` is that envelope
  (peak or last-mph tapered). At/above MAX that envelope is 0 so
  rematch / hunt / speed-sag cannot chase a faster lead past set.
  lead_approach decel (0.55) and MPC −a / danger are unchanged.

  Near the follow gap, a lead pulling away / slow rematch trickles +a
  so ease→Accel does not surge. After settle, Accel-ceil rematch is
  deadbanded; large same-speed gaps that never matched still use Accel
  *under MAX*. Mid-gap slack (12–50 m) while still slowly closing
  trickles (not Accel ceil) so cruise rematch cannot pulse. Slack
  > 50 stays Accel catch-up. After a dump, and while over map with a
  live mid-gap lead, that same 0.10 cap holds through opening /
  re-catch so the catch-up latch cannot Accel-ceil back through the
  limit. Large-gap slack stays Accel.
  """
  a = map_accel_a_ms2(LOOKAHEAD_NORMAL, int(accel_level))
  if a_personality is not None:
    a = min(a, max(0.0, float(a_personality)))
  if lead_at_or_above_max(v_ego, v_cruise):
    a = 0.0
  if v_rel is not None and float(v_rel) >= LEAD_CLOSING_REMATCH_BLOCK_MS:
    # Large-gap catch-up still uses Accel (#187) even while closing ≳ 1.0
    # *when same-speed / opening, only barely closing, or slack > 50*.
    # Mid-gap slack 12–50 m that is still closing ~1–2 m/s trickles
    # instead (ef 10:18).
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
    slow_close = (
      (not settled) and (not catchup) and lead_mid_gap_slow_close(v_rel, slack)
    )
    # Opening after a dump, or any rematch while over map: the catch-up
    # latch must not Accel-ceil a live mid-gap lead back through the limit.
    hold_trickle = (
      (not settled)
      and (post_dump or lead_map_decel_above_max(v_ego, v_cruise))
      and lead_mid_gap_map_band(slack)
    )
    if slow_close or hold_trickle:
      return min(a, LEAD_MID_GAP_REMATCH_A_MS2)
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
                                 fcw=False, crash_cnt=0, confirm_rapid=True,
                                 a_lead=None, skip_mild_floor=False,
                                 owned=False, acquiring=False, prev_v_rel=None,
                                 a_ego=None, dt=None, depart_release=False,
                                 opening_release=False):
  """True when MPC −a must not be slewed or floored.

  Firm / emergency: FCW / crash / confirmed rapid close / near bumper
  (or already inside the stop gap). Acquire and near-gap slew use this
  path so a one-off radar blip cannot dump hard regen.

  Soft-limit may pass `skip_mild_floor=True` so an already-owned /
  path-synced lead whose closing is rising (or residual close / aLead
  shows brake) releases the MILD floor without waiting for rapid ≥ 6.
  That does not skip acquire slew or promote the 0.55 path — firm /
  full still waits on confirm.

  A departing-lead release drops firmness even if closing is still
  high, once |yRel| has left the lane with slack left. Near-bumper /
  FCW stay firm. An opening release drops aLead-only firmness while
  the gap is still opening; a real close ≥ 1.5 is not that release.
  """
  if fcw or int(crash_cnt) > 0:
    return True
  if d_rel is not None and float(d_rel) <= LEAD_MPC_SOFT_NEAR_M:
    return True
  if d_rel is not None and (float(d_rel) - STOP_DISTANCE) <= 0.0:
    return True
  if depart_release:
    return False
  if v_rel is not None and lead_approach_is_rapid(v_rel):
    if (not confirm_rapid) or allow_rapid:
      return True
  hard_close = (
    v_rel is not None and float(v_rel) >= LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS
  )
  if opening_release and not hard_close:
    return False
  if skip_mild_floor and lead_soft_limit_skip(
    v_rel, a_lead, slack, owned=owned, acquiring=acquiring,
    prev_v_rel=prev_v_rel, a_ego=a_ego, dt=dt,
    opening_release=opening_release, depart_release=depart_release,
  ):
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
                        crash_cnt=0, a_lead=None, slew=LEAD_ACQUIRE_SLEW_MS2,
                        v_ego=None, v_cruise=None):
  """Slew first-latch aTarget both ways so MPC cannot punch regen→accel.

  e4 09:53:19: aTarget −0.46 then +0.05 in ~0.5 s at 118 m. Instant −a
  (slew_follow_plus_a) applied the punch; rematch slewed back over
  ~0.5 s. During the acquire window, step toward the new command.
  Rapid / near-bumper / FCW stay immediate. Closing ≳ 1.5 or a
  near-gap aLead does *not* skip the window — that punched −0.996
  at 80–130 m (10:48) and delayed only the comfort path. Over MAX
  (past the deadband), map decel still mins in even with a
  same-speed lead — do not hold that brake behind acquire slew.
  Sitting *at* MAX still slews (e4 first latch). After the window,
  same as slew_follow_plus_a.
  """
  _ = a_lead
  if target is None:
    return target
  t = float(target)
  p = t if prev is None else float(prev)
  if ((not acquiring) or lead_map_decel_above_max(v_ego, v_cruise)
      or lead_mpc_needs_full_authority(
        v_rel, d_rel, slack, allow_rapid=allow_rapid, fcw=fcw,
        crash_cnt=crash_cnt, confirm_rapid=False,
      )):
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


def lead_opening_sample(v_rel, slack) -> bool:
  """True when internal v_rel is opening and slack is still positive.

  Inside the follow gap (slack ≤ 0) is a near-bumper / too-close bite,
  not this release. Radar vRel positive is this sign flipped.
  """
  if v_rel is None or slack is None:
    return False
  return float(v_rel) < 0.0 and float(slack) > 0.0


def lead_kinematics_opening(v_rel, slack) -> bool:
  """True when the gap is clearly opening, not match-noise.

  aLeadK alone must not own firm −a here. A milder negative v_rel can
  still be one noisy sample; `update_opening_release` latches a run
  of those.
  """
  if not lead_opening_sample(v_rel, slack):
    return False
  return float(v_rel) < LEAD_OPENING_VREL_MS


def lead_gap_growing(d_rel, prev_d, tol_m=LEAD_OPENING_GROW_TOL_M) -> bool:
  """True when dRel is not shrinking. First sample counts as growing."""
  if d_rel is None:
    return False
  if prev_d is None:
    return True
  return float(d_rel) >= float(prev_d) - float(tol_m)


def update_opening_release(age, released, v_rel, d_rel, prev_d, slack, dt,
                           release_s=LEAD_OPENING_RELEASE_S):
  """Latch firm-authority release after a short opening run.

  Returns `(age, released, prev_d)`. Cleared as soon as the lead is
  not opening, is inside the gap, or is closing ≥ 1.5 — so a real
  #222 brake is not stuck behind a stale latch. While latched, a
  slightly negative aLeadK cannot re-own firm −a.
  """
  prev = None if d_rel is None else float(d_rel)
  hard_close = (
    v_rel is not None and float(v_rel) >= LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS
  )
  inside = slack is not None and float(slack) <= 0.0
  if hard_close or inside or not lead_opening_sample(v_rel, slack):
    return 0.0, False, prev
  if lead_gap_growing(d_rel, prev_d):
    age = float(age) + max(0.0, float(dt))
  if age >= float(release_s):
    released = True
  return age, bool(released), prev


def lead_lateral_departing(y_rel, prev_abs_y, slack) -> bool:
  """True when an owned lead is leaving the path with room to spare.

  |yRel| ≲ 1.5 stays on path (full firmness if still closing). Past
  ~2.25 m with slack ≳ 8 m is departing even before another sample.
  Between those, only a growing |yRel| counts.
  """
  if y_rel is None or slack is None or float(slack) < LEAD_DEPART_SLACK_M:
    return False
  ay = abs(float(y_rel))
  if ay <= LEAD_DEPART_YREL_FIRM_M:
    return False
  if ay >= LEAD_DEPART_YREL_M:
    return True
  if prev_abs_y is None:
    return False
  return ay > float(prev_abs_y) + LEAD_DEPART_YREL_GROW_M


def update_depart_release(age, released, y_rel, prev_abs_y, slack, v_rel, dt,
                          release_s=LEAD_DEPART_RELEASE_S):
  """Latch lateral firm-authority release. Returns `(age, released, abs_y)`.

  Back on path (|yRel| ≲ 1.5) clears it so a cut-in that settles in
  lane can brake again. A missing lead clears it too.
  """
  _ = v_rel
  prev = None if y_rel is None else abs(float(y_rel))
  if y_rel is None:
    return 0.0, False, None
  if lead_lateral_departing(y_rel, prev_abs_y, slack):
    age = float(age) + max(0.0, float(dt))
  else:
    age = 0.0
    if prev is not None and prev <= LEAD_DEPART_YREL_FIRM_M:
      released = False
  if age >= float(release_s):
    released = True
  return age, bool(released), prev


def lead_alead_owns_match(v_rel, a_lead, slack=None,
                         a_lead_ms2=LEAD_CLOSING_ALEAD_MS2) -> bool:
  """True when aLead alone may own match-speed −a / closing.

  aLead ≲ −0.2 used to own regardless of gap. A far or opening lead
  (da 2026-09-18: dRel~64 m, v_rel~−1.44 opening, aLeadK~−1.46) then
  snapped cruise +a to aLeadK. Gate: slack ≳ 20 m never owns from
  aLead alone. Clearly opening with positive slack never owns either
  (09:57 / 10:05: aLeadK ~−0.3…−0.4 on an opening gap must not skip
  the mild floor). Slight opening noise near the gap, and a real
  brake inside the follow gap, still match. Real closing ≳ 1.0–1.5
  is a separate v_rel gate.
  """
  if a_lead is None or float(a_lead) > float(a_lead_ms2):
    return False
  if slack is not None and float(slack) >= LEAD_ALEAD_MATCH_SLACK_M:
    return False
  if lead_kinematics_opening(v_rel, slack):
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


def lead_close_is_rising(v_rel, prev_v_rel, rise_ms=LEAD_SOFT_LIMIT_RISE_MS) -> bool:
  """True when closing rate increased vs the last owned-lead sample."""
  if v_rel is None or prev_v_rel is None:
    return False
  return float(v_rel) > float(prev_v_rel) + float(rise_ms)


def lead_residual_close_ms2(v_rel, prev_v_rel, a_ego, dt):
  """Unexplained closing accel after subtracting ego's own a.

  Δv_rel = a_ego·dt − a_lead·dt. Residual (Δv_rel − a_ego·dt) / dt
  is −a_lead: positive means the lead is braking harder than ego's
  command accounts for. None when a pair of samples is missing.
  """
  if v_rel is None or prev_v_rel is None or dt is None or float(dt) <= 1e-6:
    return None
  expected = 0.0 if a_ego is None else float(a_ego) * float(dt)
  return (float(v_rel) - float(prev_v_rel) - expected) / float(dt)


def lead_soft_limit_skip(v_rel, a_lead=None, slack=None, owned=False,
                         acquiring=False, prev_v_rel=None, a_ego=None, dt=None,
                         opening_release=False, depart_release=False) -> bool:
  """True when the MILD floor should release on this lead sample.

  Primary: an already-owned / path-synced lead (past first latch, or
  hold-owned) whose residual close shows brake (closing worsened
  beyond ego a) or near-gap aLead is negative. Soft ease can
  follow immediately. Closing ≥ 1.5 also skips so a finished rise
  — and a cut-in that is already closing hard — can still react.
  Large slack (e4) stays floored. Firm / full still waits on
  confirm so one radar blip cannot dump.

  Clearly opening + positive slack: negative aLeadK does not skip
  (it is not closing evidence). A latched opening release also
  refuses the skip until the gap stops opening. A latched departing
  lead refuses it even while still closing hard — the track has
  left the path with slack left. On-path residual / rapid close
  is unchanged.
  """
  if depart_release:
    return False
  if slack is not None and float(slack) > LEAD_ALEAD_MATCH_SLACK_M:
    return False
  if v_rel is not None and float(v_rel) >= LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS:
    return True
  if opening_release:
    return False
  if acquiring and not owned:
    return False
  if lead_alead_owns_match(v_rel, a_lead, slack):
    return True
  if (v_rel is not None and float(v_rel) >= LEAD_CLOSING_REMATCH_BLOCK_MS
      and lead_close_is_rising(v_rel, prev_v_rel)):
    return True
  residual = lead_residual_close_ms2(v_rel, prev_v_rel, a_ego, dt)
  if residual is not None and residual >= LEAD_SOFT_LIMIT_RESIDUAL_MS2:
    # Closing must actually worsen. Matched decel (Δv=0, residual ≈
    # −a_ego) is not lead-harder-than-ego. Do not require the 0.15
    # rise gate — that is 3 m/s² per planner frame and hid 07:55.
    if (v_rel is not None and prev_v_rel is not None
        and float(v_rel) >= LEAD_APPROACH_DV_MS
        and float(v_rel) > float(prev_v_rel)):
      return True
  return False


def lead_inferred_decel_ms2(v_rel, prev_v_rel, a_ego, dt, a_lead=None):
  """Measured aLead and/or residual-inferred lead decel. More negative wins.

  Residual ≈ −a_lead: closing that worsens beyond ego's own a is lead
  brake. None when neither signal is present.
  """
  cands = []
  if a_lead is not None:
    cands.append(float(a_lead))
  residual = lead_residual_close_ms2(v_rel, prev_v_rel, a_ego, dt)
  if residual is not None and residual >= LEAD_SOFT_LIMIT_RESIDUAL_MS2:
    cands.append(-float(residual))
  if not cands:
    return None
  return min(cands)


def cap_closing_lead_accel(output_a, v_rel, a_lead=None, lead_present=False,
                           owned=False, slack=None, d_rel=None,
                           acquiring=False, prev_v_rel=None, a_ego=None, dt=None,
                           opening_release=False, depart_release=False):
  """Never rematch +a into a closing / near-gap braking live or held lead.

  Closing ≳ 1.0 m/s (or hold-owned) hard-caps a at 0, except large-gap
  catch-up (#187) while closing 1.0–1.5. On an owned lead, residual
  close / measured aLead matches that decel (MPC authority after the
  mild floor skips). `aLead − k·v_rel` extra is still rapid / near
  bumper only — firm dump waits on confirm. Far / opening aLead and
  large slack stay off. Inside FD, a slow close without residual
  brake stays MILD.
  """
  if output_a is None or not (lead_present or owned):
    return output_a
  if d_rel is not None and float(d_rel) > LEAD_CLOSE_MAX_M + LEAD_APPROACH_MAX_HOLD_M:
    return float(output_a)
  v = 0.0 if v_rel is None else float(v_rel)
  skip = lead_soft_limit_skip(
    v_rel, a_lead, slack, owned=owned, acquiring=acquiring,
    prev_v_rel=prev_v_rel, a_ego=a_ego, dt=dt,
    opening_release=opening_release, depart_release=depart_release,
  )
  a_brake = lead_inferred_decel_ms2(v_rel, prev_v_rel, a_ego, dt, a_lead=a_lead)
  near_bumper = d_rel is not None and float(d_rel) <= LEAD_MPC_SOFT_NEAR_M
  # Off-path with slack: do not re-apply aLead / rapid dump after the
  # mild floor. Near-bumper still matches.
  rapid = v >= LEAD_APPROACH_RAPID_DV_MS and not (depart_release and not near_bumper)

  def _apply_match(a):
    if rapid or near_bumper:
      a_k = 0.0 if a_lead is None else float(a_lead)
      return min(a, a_k - LEAD_CLOSING_MATCH_GAIN * max(0.0, v), 0.0)
    if skip and a_brake is not None:
      return min(a, a_brake)
    return a

  # Inside FD, still closing: never rematch +a. Slow close commands
  # the MILD floor unless residual / aLead already unlocked match.
  if slack is not None and float(slack) <= 0.0 and v >= LEAD_SETTLE_VREL_MS:
    a = min(float(output_a), 0.0)
    if not skip:
      a_slow = lead_inside_slow_close_a_ms2(v_rel, slack)
      if a_slow is not None:
        a = min(a, a_slow)
    return _apply_match(a)
  if not (owned or skip or lead_is_closing(v_rel, a_lead, slack=slack)):
    return float(output_a)
  near_finish = slack is not None and 0.0 < float(slack) <= LEAD_SETTLE_FINISH_SLACK_M
  large_gap = slack is not None and float(slack) > LEAD_CLOSE_REMATCH_SLACK_M
  if ((near_finish or large_gap) and (not owned) and (not skip)
      and v < LEAD_CLOSING_MATCH_MS):
    return float(output_a)
  a = min(float(output_a), 0.0)
  return _apply_match(a)


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
                            prev_floored=False, owned=False, acquiring=False,
                            prev_v_rel=None, a_ego=None, dt=None, v_cruise=None,
                            opening_release=False, depart_release=False):
  """Floor non-emergency MPC −a to slight-lift MILD.

  Overlay min(MPC, mild) cannot stop MPC commanding ~−2.5 on radar noise
  or a matched / slow-close follow. Apply this to the MPC (and map)
  command *before* the overlay so a confirmed rapid 0.55 path is not
  also floored.

  Comfort path stays MILD when matched or slow-close. Skip the floor
  when an already-owned / path-synced lead's residual close (beyond
  ego's own a) or measured aLead shows brake. Do not wait for rapid
  ≥ 6 (07:55 class: held lead, closing 1.8→4.4, aLead ~−1, dRel
  38→25 while aTarget sat at −0.22). Cut-ins may still skip once
  closing ≥ 1.5. Large-slack e4 and far / opening aLead stay
  floored. Over MAX (past the deadband), a far / large-slack
  same-speed lead still passes map decel. A mid-gap lead that is
  not rapid / FCW / near-bumper is comfort-floored so a cruise cliff
  cannot punch at ~60 m. Sitting at MAX still uses MILD. Firm / full
  still waits on confirm.
  """
  _ = prev_floored
  if output_a is None:
    return output_a
  a = float(output_a)
  if a >= -LEAD_APPROACH_MILD_A_MS2:
    return a
  if d_rel is None or v_lead is None or v_ego is None:
    return a
  v_rel = float(v_ego) - max(0.0, float(v_lead))
  # FCW / confirmed rapid / near-bumper / match-speed skip stay raw.
  if lead_mpc_needs_full_authority(
    v_rel, d_rel, slack, allow_rapid=allow_rapid, fcw=fcw,
    crash_cnt=crash_cnt, confirm_rapid=True,
    a_lead=a_lead, skip_mild_floor=True, owned=owned,
    acquiring=acquiring, prev_v_rel=prev_v_rel, a_ego=a_ego, dt=dt,
    opening_release=opening_release, depart_release=depart_release,
  ):
    return a
  if (lead_map_decel_above_max(v_ego, v_cruise)
      and not lead_is_closing(v_rel, a_lead, slack=slack)):
    # Steady mid-gap: comfort floor, not the raw cruise/map cliff.
    # Large-gap / far same-speed still passes map decel through.
    if (lead_mid_gap_map_band(slack, d_rel)
        and not lead_approach_is_rapid(v_rel)):
      return max(a, -LEAD_MAP_MIDGAP_FLOOR_MS2)
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
