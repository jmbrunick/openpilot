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
(≳ 1.0–1.5 m/s, or aLead clearly negative) never rematches +a — coast
or match-speed −a only. Near the follow gap, a nibble min()s so we can
mesh into lead speed at the set Follow Distance. Rematch after ease
(v_rel flips / slack growing) trickles +a — do not slam regen → Accel.

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
trickles. A brief `leadOne` drop holds the last in-window lead so the
cap cannot be bypassed. Vision-only far flicker does not cap empty-road
climb. Non-rapid MPC −a is floored at MILD only as anti-chatter (gap
opening, or small |v_rel| with the lead not braking) so min(MPC, overlay)
cannot dump ~−2.5 on radar noise. Closing / a slowing lead keeps full
MPC match-speed −a; rapid / FCW / emergency still own danger.
"""
from __future__ import annotations

from openpilot.selfdrive.mapd.constants import LOOKAHEAD_NORMAL, map_accel_a_ms2

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


def lead_close_accel_ms2(accel_level: int = 5, v_rel=None, slack=None,
                         a_personality=None) -> float:
  """Max positive a (m/s²) when closing the gap on a radar lead.

  Same Accel 1–10 envelope as open-road / MAX climb — not a separate
  hotter (or cooler) catch-up curve. `a_personality` is that envelope
  (peak or last-mph tapered). lead_approach decel (0.55) and MPC −a /
  danger are unchanged.

  Near the follow gap, a lead pulling away / slow rematch trickles +a
  so ease→Accel does not surge.
  """
  a = map_accel_a_ms2(LOOKAHEAD_NORMAL, int(accel_level))
  if a_personality is not None:
    a = min(a, max(0.0, float(a_personality)))
  if v_rel is not None and float(v_rel) >= LEAD_CLOSING_REMATCH_BLOCK_MS:
    return 0.0
  if slack is None or float(slack) > LEAD_CLOSE_REMATCH_SLACK_M:
    return a
  if v_rel is not None and float(v_rel) <= 0.0:
    return min(a, LEAD_CLOSE_OPENING_A_MS2)
  if v_rel is not None and float(v_rel) < LEAD_APPROACH_DV_MS:
    return min(a, LEAD_CLOSE_REMATCH_A_MS2)
  return a


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


def lead_is_closing(v_rel, a_lead=None, close_ms=LEAD_CLOSING_REMATCH_BLOCK_MS,
                    a_lead_ms2=LEAD_CLOSING_ALEAD_MS2) -> bool:
  """True when a live/held lead is closing or clearly braking."""
  if v_rel is not None and float(v_rel) >= float(close_ms):
    return True
  if a_lead is not None and float(a_lead) <= float(a_lead_ms2):
    return True
  return False


def lead_owns_plan(v_rel, a_lead=None) -> bool:
  """Hold-window planner ownership: last close ≥ 1.5 or aLead clearly negative."""
  return lead_is_closing(v_rel, a_lead, close_ms=LEAD_CLOSING_MATCH_MS)


def cap_closing_lead_accel(output_a, v_rel, a_lead=None, lead_present=False,
                           owned=False):
  """Never rematch +a into a closing / braking live or held lead.

  Closing ≳ 1.0 m/s (or aLead ≲ −0.2, or hold-owned) hard-caps a at 0.
  Closing ≳ 1.5 or a braking lead also prefers match-speed −a
  (`aLead − k·v_rel`). Same-speed far catch-up +a is unchanged.
  """
  if output_a is None or not (lead_present or owned):
    return output_a
  if not (owned or lead_is_closing(v_rel, a_lead)):
    return float(output_a)
  a = min(float(output_a), 0.0)
  v = 0.0 if v_rel is None else float(v_rel)
  match_speed = owned or v >= LEAD_CLOSING_MATCH_MS or (
    a_lead is not None and float(a_lead) <= LEAD_CLOSING_ALEAD_MS2
  )
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
                            allow_rapid=False, a_lead=None):
  """Floor chatter-shaped MPC −a at mild ease. Match-speed / danger still dump.

  Overlay min(MPC, mild) cannot stop MPC commanding ~−2.5 on radar noise
  around a same-speed lead. Apply this to the MPC (and map) command
  *before* the overlay so a confirmed rapid 0.55 path is not also floored.

  Keep the floor only when it is anti-chatter: gap opening, or |v_rel|
  small *and* the lead is not decelerating. Skip it when closing
  (≳ SOFT_LIMIT_CLOSE) or aLead is clearly negative so we can match lead
  speed — including slightly slower than the lead to settle the gap.
  A one-frame v_rel blip below SOFT_LIMIT_CLOSE stays floored; closing
  or aLead skip immediately so match-speed braking is not delayed.
  Four agreeing samples pass `allow_rapid` and skip the floor. FCW /
  crash / stop kinematics still own danger.
  """
  if output_a is None:
    return output_a
  a = float(output_a)
  if a >= -LEAD_APPROACH_MILD_A_MS2:
    return a
  if fcw or int(crash_cnt) > 0:
    return a
  if d_rel is None or v_lead is None or v_ego is None:
    return a
  v_rel = float(v_ego) - max(0.0, float(v_lead))
  if allow_rapid and lead_approach_is_rapid(v_rel):
    return a
  # Closing onto the lead, or the lead is braking: full match-speed −a.
  # Same gate as rematch-block ownership / overlay-MILD skip.
  if lead_owns_plan(v_rel, a_lead):
    return a
  stop_slack = float(d_rel) - STOP_DISTANCE
  if v_rel > 0.0:
    if stop_slack <= 0.0:
      return a
    a_need = -(v_rel * v_rel) / (2.0 * stop_slack)
    if a_need < -LEAD_APPROACH_A_MS2:
      return a
  return max(a, -LEAD_APPROACH_MILD_A_MS2)


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
  set gap and release slew is not a regen→Accel punch. A far lock that
  is already closing (≳ 1.0 m/s) always min()s — never keep rematch +a
  into a shrinking gap. Same-speed far catch-up may still keep +a.
  """
  if a_lead is None:
    return float(output_a)
  out = float(output_a)
  a = float(a_lead)
  if out <= 0.0 or a <= -float(nibble):
    return min(out, a)
  if lead_is_closing(v_rel):
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
  a_needed = -(v_rel * v_rel) / (2.0 * slack)
  ttc = lead_approach_ttc_s(slack, v_rel)
  rapid = bool(allow_rapid) and lead_approach_is_rapid(v_rel, ttc)
  a_cap = float(a_comfort) if rapid else LEAD_APPROACH_MILD_A_MS2
  return max(float(a_needed), -float(a_cap))
