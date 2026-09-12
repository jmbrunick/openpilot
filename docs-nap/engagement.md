# Engagement Flow

Pre-AP has two modes depending on whether a Comma Pedal is installed. Mode is selected by `nap_conf.use_pedal` (NAP settings panel, persisted via Params). Both modes share the same stalk FSM in `engagement.py`.

## State vocabulary

- `cruiseEnabled` — NAP is engaged at the FSM level (lateral on)
- `enableLongControl` — NAP is actively modulating longitudinal (pedal mode only)
- `enableJustCC` — NAP is lateral-only, stock CC is (or will be) driving speed
- `pedal_long_allowed = use_pedal AND pedal_transform_valid` — pedal hardware is usable
- `long_control_allowed = (not use_pedal) OR pedal_transform_valid` — NAP's FSM may transition into the "cruise" state (pedal mode requires a valid pedal transform)

## Pedal mode

`use_pedal=True`, pedal hardware present:

1. Driver double-pulls stalk → `cruiseEnabled=True`, `enableLongControl=True`
2. `carcontroller.py` sends `GAS_COMMAND` (0x551) based on `actuators.accel` from the long planner
3. Zero-torque learning tracks the resting pedal position that produces zero torque at the current speed. On engage, we seed `prev_pedal_di` to this value so there is no regen spike
4. Brake rising edge → drops `enableLongControl=False`, keeps `cruiseEnabled=True` (steering stays on, pedal drops). `enableJustCC` flips true so the carcontroller knows to spoof stock CC cancel. This is a **silent long pause**, not a full disengage: no `EventName.pedalCruiseDisabled` / no disengage chime / no HUD “Pedal Cruise Disengaged”. Held MAX (`pedal_speed_kph` / `MapCruiseHold.held_max_kph` / sticky set) is remembered. Soft-wheel handoff is a separate lat-only path and does not use this pause. **Exception:** an emergency/hard brake *during a soft-lat yield* (or within 2 s of yield entry) fully cancels the session — see below. Light brake alone never takes that path.
5. A latched **driver turn** blinker (held stalk / flash-latched lamps, not ALC tip or keep-alive) also drops `enableLongControl` the same way, with the same sticky-MAX rules and the same silent pause (no pedal-cruise disengage sound). Long stays off after the lamps go dark unless the driver SETs. **Soft-lat Off:** lateral still pauses on lamp latch and resumes only after hand release. **Soft-lat On:** blinker-on alone does **not** drop `latActive`; the driver may still soft-yield by pushing, and soft-lat will not take lateral back while the driver-turn blinker is latched. After the blinker clears, a yielded turn resumes through soft yield + the normal 0.15 s hands-off confirm + 1 s blend (no dedicated blinker blend). **One stalk SET** restores **longitudinal only**, MAX = held MAX (which may already have rebased if posted changed under maps). At a stop, that SET keeps held MAX but does not take long until a light throttle. That resume is quiet — not a full-stack engage fanfare. Lat may still be yielded for hand-release. The default double-pull first-pull path is skipped so a second SET is not required to get long back. SET while a turn lamp or held LEFT/RIGHT is still showing does not stick. **Double SET** (second pull inside `double_pull_window_ms`) forgets sticky and ensures lat+long on: maps on + posted known → MAX = current posted; maps off / posted unknown → MAX = current traveled speed. Tip ALC must not use this long-pause path.
6. Gas press → `OVERRIDE_LONGITUDINAL`. Pedal command passes through with `enable=0` — driver's foot controls throttle directly, NAP tracks position for smooth resume. Long engage/disengage prompts follow `enableLongControl` for **session** edges (initial engage / full cancel), not interceptor handshake, not gas override, and not a brake/turn long pause. `enableLongControl` stays true while the driver is on the pedal, so press and release are silent.
7. Stalk cancel (real, not spoof echo) → full disengage. `AudibleAlert.disengage` / `EventName.pedalCruiseDisabled` fire on that session teardown, including when openpilot is already going disabled. Door / **Reverse (or any gear out of Drive)** / permanent steer fault / hands-on ≥ 2 (when not blinker-latched) are the same hard cancel: session down, held MAX forgotten, soft-lat reset, `preap_cc_cancel_needed` so panda’s cruise latch can re-arm, disengage prompt kept. Orig `check_can_engage` only zeroed `cruiseEnabled` / long — that left sticky MAX and did not spoof CANCEL. Panda `tesla_preap` already sets `controls_allowed=false` on leaving Drive *without* `pcm_cruise_check(false)`, so a later Drive SET enabled selfdrived while panda stayed latched → **Controls Mismatch**. After returning to Drive, a normal double SET is a new engage (not a sticky-MAX resume) and must not require a manual disable dance.

Interface flags for this mode: `openpilotLongitudinalControl=True`, `pcmCruise=False`. Long planner runs; accel goes to pedal.

## No-pedal mode

`use_pedal=False`, no pedal hardware:

1. Driver double-pulls stalk → `cruiseEnabled=True`, `preap_cc_engage_needed=True`
2. `carstate` publishes `ret.cruiseState.enabled=True`
3. selfdrived sees this with `pcmCruise=True` → engages op lateral
4. `carcontroller` sends `RES_ACCEL` stalk spoof via `0x45` → stock Tesla CC engages and holds current speed
5. `CC.longActive=False` always (`openpilotLongitudinalControl=False`) — no planner accel is produced or consumed
6. Stalk cancel → `preap_cc_cancel_needed=True` → carcontroller spoofs `CANCEL` stalk → stock CC drops
7. Driver's own stalk +/- buttons adjust stock CC speed directly (stock CC is on the same CAN bus)

Interface flags for this mode: `openpilotLongitudinalControl=False`, `pcmCruise=True`. Mirrors the honda/hyundai/volkswagen pattern.

## Stalk spoofing and echo filters

NAP both reads the real stalk and sends spoofed stalk messages. This creates an echo risk: our own spoofed message could be read back as a real driver press. Two windows suppress this:

- `CANCEL_ECHO_WINDOW_MS = 600` — ignore `CANCEL` events within 600 ms of our last non-cancel stalk activity (so auto-cancel echoes from engage spoofs don't look like the driver pressing cancel)
- `SPOOF_ECHO_WINDOW_MS = 300` — ignore `CANCEL` events within 300 ms of our last explicit spoof send

Both tuned on real drive logs. If you see stale "driver canceled" events in logs, these are the knobs.

## Double-pull vs single-pull

Single stalk pull only engages lateral if `enableDoublePull` is off. In the default double-pull mode:

- First pull → pending engage state, lateral-only, window open
- Second pull within `double_pull_window_ms` → full engage (pedal or stock CC depending on mode)
- Window expires → stays lateral-only

This is a driver-training feature: prevents accidental full engage from a bump of the stalk. Once already engaged, a brake or driver-turn drop of long is not a new first pull: one SET restores long with the held MAX. A second SET inside the window is **take speed now** (forget sticky; posted if known, else current traveled speed) and keeps lat+long on. The same double SET from fully disengaged is initial engage.

### Sticky MAX (pedal mode)

HUD MAX is `CS.vCruise` / `pedal_speed_kph`. Policy lives in `MapCruiseHold` (`selfdrive/mapd/map_speed_policy.py`):

| Field | Role |
|--------|------|
| `held_max_kph` | MAX to resume after a long pause. Survives brake / driver-turn. Forgotten on hard cancel or double SET. |
| `sticky_set_kph` | Driver hold that Follow must not overwrite while posted is unchanged. |
| `last_posted_kph` | Last *known* OSM posted (+ offset). GPS/match drop does not clear it or invent a posted. |

- **Maps on / posted known:** MAX rebases **only** when the posted value itself changes (e.g. 65→45 or 65→70), including while long-paused. Driver can hold 55 in a 65. Do not overwrite sticky toward posted every frame.
- **Maps off / posted unknown:** MAX never auto-rebases. Never invent a posted value. GPS glitch → keep held MAX. Do not latch pause ego (`cruiseState.speed`) into held. Stalk +/- while long is active updates the MAX one SET will resume.
- **One SET** (session already up, typical after long pause): resume long only; write `held_max_kph` (already rebased if posted changed). `resume_held` is a one-shot; `engage_rising` can arrive a frame later (`pedalLongActive` lags `enableLongControl`). That delayed rising edge must not take-now and overwrite a held MAX with current traveled speed. **At a stop** (`vEgo <= 0.3 m/s` / standstill — same floor as controlsd lat): one SET does **not** take long or creep from 0. It still means “I want resume” (held MAX is kept). A **light throttle / gas** touch then resumes at that held MAX. Rolling (not at stop): one-SET resume is unchanged. Double SET / forget-sticky / full engage unchanged unless a hard cancel already cleared them.
- **Double SET:** forget sticky; maps+posted known → write current posted; else write current traveled speed. Same gesture from fully disengaged is initial engage.

No-pedal / stock CC does not own software MAX. Brake does not pause NAP long (stock CC handles brake). Double-pull still engages lat; set speed stays with Tesla CC.

## Steering disengage

`handle_steering_disengage()` runs on every frame. Rising edge of `steeringDisengage` (hands-on level ≥ 2 or EPAS rejecting) fully tears down FSM state: `cruiseEnabled=False`, `enableLongControl=False`, `pedal_speed_kph=0`, pull timers cleared.

On Pre-AP that falling `cruiseState.enabled` is `EventName.pcmDisable`. The 3X HUD string is **"Steering Disengaged"** (`pcm_disable_alert`) — not `EventName.steerDisengage`, which is sound-only. Do not hide `pcmDisable` after cruise is already down; keep `cruiseEnabled` during a blinker-held / lat-paused turn so the alert never fires. Hard-gate teardown on one lamp, physical LEFT/RIGHT, or the flash-latch hold. Door / Reverse / stalk cancel / permanent steer fault still fully disengage (`hard_cancel_session` from `check_can_engage`). The Reverse HUD (“TAKE CONTROL IMMEDIATELY” / `EventName.reverseGear`) is that session end — not a pause that stays half-engaged.

This is the primary user override path when the driver is *not* in a blinker turn. The panda also enforces it independently via EPAS error codes 6–9 and hands-on level ≥ 2 — except during a blinker-latched driver turn, where those conditions must not drop `controls_allowed` (otherwise selfdrived fires `controlsMismatch` after 2s and fully cancels while the lamps are still flashing).

Panda firmware must be flashed after this safety change. Software update / on-device `scons` rebuilds the panda image from `opendbc_repo/opendbc/safety/modes/tesla_preap.h` (latch in `tesla_preap_blinker.h`); reboot so `pandad` sees the new signature and flashes. Python `BlinkerLateralHold` alone cannot keep `controls_allowed`.

## Driver-wheel temporary lateral handoff

**Default On.** Settings → NAP → Driving Mannerisms → Soft Lateral Handoff can be turned **Off**. Yield is a **light purposeful push** with a hand on the rim that **frees the EPS** (generally avoiding something), not gravel spike trains / wind / road-crown false pressure.

#79 still felt like wrestling for two reasons: (1) OP kept **full lateral authority until the 0.70 Nm / 140 ms debounce completed**, and (2) after “yield” it kept `latActive=True` and commanded **measured angle** (`apply_lat_authority(0)` / `DAS_steeringControlType=1`). That is closed-loop follow-the-rim — the driver could nudge off-path but OP was still holding/steering. Yield now reuses the **blinker lat-pause EPS release** (`latActive` false → `DAS_steeringControlType=0`). Hands stay detected via `EPAS_handsOnLevel`. Resume is still pin-to-wheel + **1 s blend**; hands still on stay yielded (do not blend onto the model). A driver-turn blinker does **not** itself free the EPS when Soft Lateral Handoff is On.

Pothole / obstacle dodges — including fighting OP to leave the path — without a full disengage, unless the driver then emergency-brakes (full cancel). Software-only; panda hands-on ≥ 2 and `STEER_THRESHOLD` are unchanged.

### Signals (Pre-AP)

| Signal | Source | Type | Role |
|--------|--------|------|------|
| `EPAS_torsionBarTorque` / `CS.steeringTorque` | `EPAS_sysStatus` 0x370 | continuous Nm (0.01, −20.5) | sustained directional torque (not spikes) |
| `StW_AnglHP_Spd` / `CS.steeringRateDeg` | `STW_ANGLHP_STAT` 0x0E | continuous deg/s (0.5, −4096), negated | **not** an entry gate. Weak wind filter only when torsion is below the yield trigger |
| `EPAS_handsOnLevel` | same EPAS msg | discrete 0/1/2/3 | required for intent (≥ 1). ≥ 1 holds yield. ≥ 2 remains hard disengage |
| path / tracking error | `desired_curvature − measured` | 1/m | high error **and** torsion below trigger → disturbance (wind / crown). High torsion is intent even if error is large |
| digital brake Applied | `DI_brakePedal` / `BrakeMessage.driverBrakeStatus` | boolean | light brake = silent long pause. Not `CS.brakePressed` (forced false) |
| `CS.aEgo` | speed KF | m/s² | emergency decel qualifier (no analog pressure on parsed buses) |

`steeringPressed` is still `|torsion| > STEER_THRESHOLD` (**1.0 Nm**) with **5-frame** debounce (~50 ms). That fires `EventName.steerOverride` (`OVERRIDE_LATERAL`, stays enabled) and is the current software override effort.

### Intent to enter yield

All of the following, consecutive frames (gaps reset — gravel spike trains do not accumulate):

1. `|torsion| >= 0.55 Nm` (55% of `STEER_THRESHOLD`; a light purposeful push, still below `steeringPressed` 1.0 Nm)
2. `handsOnLevel >= 1` (hand on the rim) — required so rumble / wind with hands off or resting does not free-yield
3. Held for **90 ms** at the 0.55 Nm floor. Fewer frames as torsion approaches `STEER_THRESHOLD` (down to **60 ms** at 1.0 Nm) so the soft path beats hands-on ≥ 2. Never below 60 ms (5-frame gravel bursts stay rejected).

Do **not** require high steer rate. An isometric fight (firm torsion, wheel barely moving, high tracking error) **must** yield. Rate may stay as a weak wind filter only when torsion is **below** 0.55 Nm — it never blocks a firm push and never promotes low torsion to intent.

Disturbance veto only when torsion is **below** the yield trigger: `|desired − measured| curvature >= 0.0025` **and** `|torsion| < 0.55 Nm` is wind / crown — do not yield. High torsion is intent even if tracking error is large (driver is leaving the path).

Release hysteresis stays **0.40 Nm** (press-latch only). Do **not** go back to raw 0.5 Nm / 80 ms *torsion-only* (no hands gate) — that was #71 rumble. ≥ 2 stays the hard/safety path.

### Behavior (lat yield)

**Yield = free wheel, not follow-angle.** Same EPS release as blinker lat-pause.

1. OP engaged and providing lateral. Intent detected → `carControl.latActive` goes **false**. Pre-AP carcontroller sends `DAS_steeringControlType=0` and `apply_steer_angle_limits_vm` snaps `apply_angle` to measured (stock inactive). The driver steers the car; OP is not angle-controlling to the rim. `desired_curvature` is **pinned to measured** so resume starts from the wheel. Longitudinal and `cruiseEnabled` stay up. Stalk cancel / doors / hands-on ≥ 2 still hard-disengage. An occasional light torsion probe is OK to confirm he is still there — hold is `EPAS_handsOnLevel`, not continuous angle-hold.
2. Stay yielded (EPS free) while still maneuvering: **`EPAS_handsOnLevel >= 1`** **or** a renewed **≥ 0.55 Nm** push. Mid-dodge torsion dips must **not** start the hand-back. Rate is not a hold signal (caster / road after release).
3. After hands go to **0** for **~0.15 s**, lat comes back and a **1.0 s** smoothstep (`t²(3−2t)`) blends authority 0 → 1. `QUIET_WAIT_S` stays 0 (no torsion-quiet — that blended on mid-dodge dips). The pin lifts when blend starts. `apply_lat_authority` is skipped at authority=1 and while lat is down.
4. Renewed **hands-on** or a firm push ≥ 0.55 Nm during that wait or the blend immediately re-yields (EPS free again) and resets the delay. Hands off ~0.15 s and the 1 s blend retries.
5. Hand-back is the S-curve from the wheel, not a grab onto the model. There is **no** dedicated blinker rising-edge 1 s blend.
6. HUD: `controlsState.latHandoffPaused` stays set until authority ≥ **0.70**. Gray **override**, no sounds. Green + on-screen path is not enough — the wheel must unwind onto that path.

**Driver-turn blinker + soft-lat (On):** lamp latch / held stalk must **not** clear `latActive` or force EPS free. If soft-lat (or stock lat) still has control when the blinker rises, **keep control**. Soft-lat may still yield if the driver pushes (intent / hands) — that is the driver taking control, not the blinker stripping it. While the driver-turn blinker is latched, do **not** re-enable: if already yielded or blending, stay yielded / do not finish a take-back blend. After the blinker clears, do **not** special-case blend; if we were yielded / blending / lat-down, **land in soft yield** and let the normal resume own it (hands on / firm push stay free; hands 0 for `HANDS_OFF_CONFIRM_S` ~0.15 s, then the 1 s smoothstep). ALC tip / keep-alive flashes are **not** a driver turn. **Soft-lat Off** keeps today’s blinker lat-pause so a held turn still frees the wheel. Long sticky-MAX turn pause + one SET / double SET and panda blinker latch are unchanged. Hazards still do not pause.

### Emergency / hard brake → full OP disable

Pre-AP has **no analog brake pressure** on the buses we parse (`test_preap_brake_signals.py`: `BrakeMessage` is a 2-bit Applied enum; no `IBST_` / iBooster travel). Light brake is that digital bit and **must** keep the existing sticky-MAX silent long pause + one SET. Do not turn every brake into a full cancel.

**Emergency definition** (testable, in `driver_lateral_handoff.py`):

| Check | Value | Why |
|--------|--------|-----|
| Digital Applied | `DI_brakePedal==1` **or** `driverBrakeStatus==APPLIED`, published on `CS.brake` (not `brakePressed`) | Driver is on the pedal. OP map-track / planner decel does not set this. |
| Measured decel | `aEgo <= −3.5 m/s²` for **80 ms** (8 frames) | ~0.36 g. Comfort Accel-5 is **−0.80 m/s²**; pre-AP planner clip is **−1.5 m/s²**. A lone pothole spike is shorter than 80 ms. |
| Speed | `vEgo >= 1.0 m/s` | Reject standstill KF chatter. |
| Timing | currently yielded **or** blending **or** within **2.0 s** of yield entry | Same timeframe as the soft-lat takeover. |

When that fires, card calls `hard_cancel_session()`: `cruiseEnabled=False`, long down, held MAX forgotten, `preap_cc_cancel_needed`. That is the normal hard-cancel path: `EventName.pcmDisable` / “Steering Disengaged” + disengage chime, and `pedalCruiseDisabled` if long was on (session down, not a silent pause). controlsd also sets `CC.cruiseControl.cancel`. Light brake while yielded or not **does not** set this flag.

### Where to look

- `selfdrive/controls/lib/driver_lateral_handoff.py` — firm-push detector (torsion + hands; rate not an entry gate), `lat_active_after_handoff` (yield = free EPS), low-torsion disturbance veto, emergency definition, hands-on hold, smoothstep, curvature pin
- `selfdrive/controls/controlsd.py` — drop `CC.latActive` while yielded; pin `desired_curvature` while lat down; pass torsion / rate / hands / tracking error / digital brake / aEgo; cancel on emergency
- `opendbc_repo/opendbc/car/tesla/carcontroller.py` — `DAS_steeringControlType` from `CC.latActive` (0 = EPS free, same as blinker pause)
- `selfdrive/car/tesla/preap_blinker_lat_pause.py` — publish hands-on + digital brake; card-local handoff; `hard_cancel_session`
- `selfdrive/ui/ui_state.py` — gray override chrome while paused
- `selfdrive/controls/lib/tests/test_driver_lateral_handoff.py`

### On-car test plan (Pre-AP Model S / comma 3X)

Do these at a quiet road / parking lot first, then a known pothole stretch. Pedal or no-pedal both OK. Do **not** expect a panda flash.

1. **Engage** with a double-pull. Confirm green engaged chrome and that long/cruise is holding speed.
2. **Intentional dodge** (hands on, light purposeful push ≥ ~0.55 Nm for ~90 ms — the wheel does **not** have to be moving). Includes fighting OP to leave the path (high tracking error, low rate). The wheel should **just let go** (EPS free, not still holding/steering to the rim) **before** a hard yank / hands-on ≥ 2. Speed control must stay on. HUD should go **gray override**, not “Steering Disengaged”, and must not play the disengage chime.
3. **Hold** through a pothole dodge (hands still on the rim). Authority must stay yielded even if torsion dips.
4. **Release.** Hands clearly off the rim. After ~0.15 s the wheel eases back onto the path over about **one second**. A brief hands-off or right-then-left crossover during the dodge must **not** snatch. When green returns, the wheel must follow the on-screen path.
5. **Re-grab during the wait or blend.** Hands back on or a firm push: yield again (delay resets). Hands off ~0.15 s and the 1 s blend retries.
6. **Road rumble / crosswind / crown** with hands resting: must **not** gray. Wind that makes NAP fight the path (high tracking error, low torsion) must **not** yield. If it still does, turn Soft Lateral Handoff **Off**. Do not lower panda / `STEER_THRESHOLD`.
7. **Light brake** (tap, not a panic stop) while engaged, yielded or not: silent long pause + held MAX; one SET resumes **if already moving**. At a stop, one SET must **not** roll the car; tap the accelerator, then long resumes at held MAX. No disengage chime. Sticky MAX (#72+#77 / nap-release #76) unchanged.
8. **Hard brake during / just after a soft-lat dodge** (firm pedal, strong decel): full OP cancel, “Steering Disengaged”, disengage chime. Not a silent pause. One SET is a new engage, not resume-at-held-MAX.
9. **Blinker turn** (held stalk), Soft-lat **On**: blinker-on alone must **not** free the wheel. A push may still yield. Soft-lat must not take lat back while the driver-turn lamps are latched. After the blinker clears, if lat was yielded, ease back through yield + ~0.15 s hands-off + **1 s** blend (not an immediate blinker blend). Soft-lat **Off**: existing lamp-latch lat pause, long drop, one SET resume. Tip ALC must not latch a driver turn.
10. **ALC tip** still arms a lane change; wheel nudge at 1 Nm still starts it.
11. **Hard yank / hands-on 2**, stalk cancel, door: full disengage, unchanged.
12. **Reverse while engaged:** session ends (not a pause). HUD reverse / take-control, then after Drive a normal double SET must engage without Controls Mismatch and without a prior on-device disable. Soft-lat / blinker / sticky MAX must not stay wedged.

## Where to look

- `opendbc_repo/opendbc/car/tesla/preap/engagement.py` — stalk FSM
- `selfdrive/car/tesla/preap_blinker_lat_pause.py` — blinker-turn long drop + one-SET resume (patches the FSM)
- `opendbc_repo/opendbc/car/tesla/preap/carstate.py:120+` — button event pump + cruise state publish
- `opendbc_repo/opendbc/car/tesla/preap/carcontroller.py:40+` — pedal TX + stalk spoof scheduling
- `opendbc_repo/opendbc/car/tesla/preap/tests/test_preap_engagement.py` — FSM tests (brake drop, steering disengage, double-pull)
- `selfdrive/car/tesla/tests/test_preap_blinker_lat_pause.py` — blinker lat pause, turn drops long, one SET resumes
- `selfdrive/car/tesla/tests/test_preap_sticky_max.py` — sticky MAX / silent long pause / one SET / double SET
- `selfdrive/mapd/tests/test_map_speed_policy.py` — maps rebase only on posted-value change
