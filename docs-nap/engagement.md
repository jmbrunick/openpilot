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
4. Brake rising edge → drops `enableLongControl=False`, keeps `cruiseEnabled=True` (steering stays on, pedal drops). `enableJustCC` flips true so the carcontroller knows to spoof stock CC cancel. This is a **silent long pause**, not a full disengage: no `EventName.pedalCruiseDisabled` / no disengage chime / no HUD “Pedal Cruise Disengaged”. Held MAX (`pedal_speed_kph` / `MapCruiseHold.held_max_kph` / sticky set) is remembered. Soft-wheel handoff is a separate lat-only path and does not use this pause. **Exception:** an emergency/hard brake *during a soft-lat yield* (or within 2 s of yield entry) fully cancels the session — see below. Light brake alone never takes that path. After the long drop, tip vs hold is classified from **Applied duration** (like the stalk) plus firm `aEgo` — digital Applied only, no travel/pressure. A **short tip** (< 0.30 s, then lift, `aEgo` > −1.5) keeps interceptor ENABLE and runs a **comfort-shaped regen ramp** over **~2.5 s**: gentle and noticeable on the first frame (`a ≈ −0.30`, no stock / full-regen bite), then quadratic ease-in to fairly hard / `REGEN_MAX` toward the end. The 10–15 mph band guides `a_end` when reachable; otherwise the same shape still runs to `REGEN_MAX`, then hands off. Not a coast plateau, not a constant-a step, and not the reverted 0.75 s fade. When ego reaches ~12 mph or the window expires: **RELEASE** to stock. Already in the 10–15 mph band: no ramp. **Held** (≥ 0.30 s) or **hard** (`aEgo <= −1.5`): RELEASE immediately so Tesla friction + stock regen stay firm. FCW / AEB / hard lead while long is still on are unchanged.
5. A latched **driver turn** blinker (held stalk / flash-latched lamps, not ALC tip or keep-alive) does **not** drop `enableLongControl`. Long stays engaged; normal lead/map braking and accel apply through the turn. **Soft-lat Off:** lateral still pauses on lamp latch and resumes only after hand release. **Soft-lat On:** blinker-on alone does **not** drop `latActive`; the driver may still soft-yield by pushing, and soft-lat will not take lateral back while the driver-turn blinker is latched. After the blinker clears, a yielded turn resumes through soft yield + the normal 0.15 s hands-off confirm + 1 s blend (no dedicated blinker blend). Brake still uses the silent long pause + sticky MAX + one SET / double SET. Tip ALC is not a driver turn and already does not drop long.
6. Gas press → `OVERRIDE_LONGITUDINAL` **when One-Pedal Long is Off** (default). Pedal command passes through with `enable=0` — driver's foot controls throttle directly, NAP tracks position for smooth resume. Long engage/disengage prompts follow `enableLongControl` for **session** edges (initial engage / full cancel), not interceptor handshake, not gas override, and not a brake / one-pedal long pause. `enableLongControl` stays true while the driver is on the pedal, so press and release are silent. On the gas falling edge, ACQUIRE expires the 0.5 s engage-grace a=0 floor and seeds VDAS from `max(last non-negative aEgo, planner climb)` (clamped to the personality MAX). Open-road planner climb is Mannerisms **Acceleration** 1–10 toward HUD MAX. Lead hard decel / FCW / should-stop (`actuators.accel < 0`) still seed 0 and pass through immediately. Brake clears the pending handoff so a later SET resume keeps the normal grace ramp. First engage without prior gas is unchanged.
   - **One-Pedal Long On** (Settings → NAP → Driving Mannerisms, default Off): if OP long is **already on** and the accelerator is at rest then pressed (rising `gasPressed` from interceptor DI, not `DI_pedalPos`), **pause** `enableLongControl` with the same silent long pause as brake (`_drop_longitudinal_keep_lateral`: lat stays, sticky MAX, one SET resumes). Do **not** `USER_DISABLE` / `hard_cancel_session` / take-control. **Engage while gas is already pressed** (one SET or two SET) does **not** pause — long stays armed; lift still ACQUIREs with A+B grace expire + aEgo / A3 climb. Same-frame SET + first gas sample is engage-with-gas, not a from-rest kick. After a from-rest pause, interceptor **RELEASEs** and stays RELEASED on lift — Tesla physical pedal / stock lift-regen is the one-pedal path (REGEN_MAX envelope, ~−1.5 m/s²). Do **not** re-ACQUIRE on lift after that pause (ENABLE 0↔1 chatter) and do **not** rewrite `GAS_COMMAND` DI while ENABLE=1. Standstill wait-for-gas resume and a SET that just restored long are not paused. Brake cancel, tip-brake glide, FCW/AEB unchanged.
7. Stalk cancel (real, not spoof echo) → full disengage. `AudibleAlert.disengage` / `EventName.pedalCruiseDisabled` fire on that session teardown, including when openpilot is already going disabled. Door / **Reverse (or any gear out of Drive)** / permanent steer fault / hands-on ≥ 2 (when not blinker-latched) are the same hard cancel: session down, held MAX forgotten, soft-lat reset, `preap_cc_cancel_needed` so stock CC drops. Reverse `USER_DISABLE` is quiet (no take-control-immediately alarm); other hard cancels keep the disengage prompt. Orig `check_can_engage` only zeroed `cruiseEnabled` / long — that left sticky MAX and did not spoof CANCEL. Panda `tesla_preap` uses `pcm_cruise_check(false)` **while not in Drive** so `cruise_engaged_prev` is already clear before Drive return (same cleanup as stalk cancel). Python's CANCEL spoof is TX-only and cannot clear that latch. Drive SET while `!controls_allowed` is last-resort only; in-Drive mismatch checks and engagement are otherwise unchanged. selfdrived holds the 3X/OP `mismatch_counter` at 0 **for the whole R/P period**, not on Drive entry. After returning to Drive, a normal double SET is a new engage (not a sticky-MAX resume). **Flash the panda** after this safety change.

Interface flags for this mode: `openpilotLongitudinalControl=True`, `pcmCruise=False`. Long planner runs; accel goes to pedal.

**Stock CC vs pedal long:** on the rising edge of `enableLongControl` (and while DI stays **ENABLED** or **STANDBY**), card keeps `preap_cc_cancel_needed` until `di_cruise_state` is neither. One CANCEL from ENABLED typically lands in STANDBY; a second CANCEL takes it OFF. That closes the gap where stock CC stayed armed and could fight the pedal. `PreAPLongController` already one-shots CANCEL on long rising / falling / a stalk press — the persist-until-off path is in `preap_force_offroad_handoff.py`.

**Force Offroad while software long is on:** on-road, a big **Yes / No** (**Ready to resume steering control?**) comes first. **No** clears the toggle and leaves long as it was. **Yes**, then do **not** drop `started` until stock CC is ENABLED at current speed (CANCEL → STANDBY → drop OP long → SET_ACCEL). See [force-offroad.md](force-offroad.md). The handoff suppresses the engage-kill so it can park DI in STANDBY and SET.

**Follow Distance + Hypermile (nap-dev):** Settings → NAP → Driving Mannerisms. Behind a radar lead, a stalk **tip** (first detent, 1 mph / 1 kph) undoes that frame’s MAX and steps stock Follow Distance 1–7 (`NAPFollowDistance`) on return to IDLE; a **full press** (2nd detent, 5 mph / 5 kph) still steps MAX +5/−5 and does not remap Follow, even though the lever passes through first detent. Same whether Hypermile is On or Off. The Driving Mannerisms slider stays visible and updates live. No lead: tip and hold both still adjust MAX. Hypermile (default Off) is eco snap / Step Down / Hill Climb only; it does not own follow levels. Opt-in **Step Down Speed** (default Off) uses the same posted-scale as eco, larger drop (−15 at 80; town 30 stays 30); Follow stalk SET can still hold above that until posted changes. **Hill Climb** (default On, inert unless Hypermile is On) uses IMU pitch to hold Accel 1 on grades and ease over a crest — no maps-elevation lookahead. See [hypermile.md](hypermile.md). Soft-lat / DM / blinker / sticky MAX / one-SET / standstill gas-gate / reverse hard-cancel unchanged.

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

This is a driver-training feature: prevents accidental full engage from a bump of the stalk. Once already engaged, a brake drop of long is not a new first pull: one SET restores long with the held MAX. A second SET inside the window is **take speed now** (forget sticky; posted if known, else current traveled speed) and keeps lat+long on. The same double SET from fully disengaged is initial engage. A latched driver turn does not drop long.

### Sticky MAX (pedal mode)

HUD MAX is `CS.vCruise` / `pedal_speed_kph`. Policy lives in `MapCruiseHold` (`selfdrive/mapd/map_speed_policy.py`):

| Field | Role |
|--------|------|
| `held_max_kph` | MAX to resume after a long pause. Survives brake. Forgotten on hard cancel or double SET. |
| `sticky_set_kph` | Driver hold that Follow must not overwrite while posted is unchanged. |
| `last_posted_kph` | Last *known* OSM posted (+ offset). GPS/match drop does not clear it or invent a posted. |

- **Maps on / posted known:** MAX rebases **only** when the posted value itself changes (e.g. 65→45 or 65→70), including while long-paused. Driver can hold 55 in a 65. Do not overwrite sticky toward posted every frame.
- **Maps off / posted unknown:** MAX never auto-rebases. Never invent a posted value. GPS glitch → keep held MAX.
- **One SET** (session already up, typical after long pause): resume long only; write `held_max_kph` (already rebased if posted changed). **At a stop** (`vEgo <= 0.3 m/s` / standstill — same floor as controlsd lat): one SET does **not** take long or creep from 0. It still means “I want resume” (held MAX is kept). A **light throttle / gas** touch then resumes at that held MAX. Rolling (not at stop): one-SET resume is unchanged. Double SET / forget-sticky / full engage unchanged unless a hard cancel already cleared them.
- **Double SET:** forget sticky; maps+posted known → write current posted; else write current traveled speed. Same gesture from fully disengaged is initial engage.

No-pedal / stock CC does not own software MAX. Brake does not pause NAP long (stock CC handles brake). Double-pull still engages lat; set speed stays with Tesla CC.

## Steering disengage

`handle_steering_disengage()` runs on every frame. Rising edge of `steeringDisengage` (hands-on level ≥ 2 or EPAS rejecting) fully tears down FSM state: `cruiseEnabled=False`, `enableLongControl=False`, `pedal_speed_kph=0`, pull timers cleared.

On Pre-AP that falling `cruiseState.enabled` is `EventName.pcmDisable`. The 3X HUD string is **"Steering Disengaged"** (`pcm_disable_alert`) — not `EventName.steerDisengage`, which is sound-only. Do not hide `pcmDisable` after cruise is already down; keep `cruiseEnabled` during a blinker-held / lat-paused turn so the alert never fires. Hard-gate teardown on one lamp, physical LEFT/RIGHT, or the flash-latch hold. Door / Reverse / stalk cancel / permanent steer fault still fully disengage (`hard_cancel_session` from `check_can_engage`). Reverse is that session end — not a pause that stays half-engaged — but the Reverse `USER_DISABLE` is **quiet** (no “TAKE CONTROL IMMEDIATELY” / `warningImmediate`). A silent PERMANENT “Reverse Gear” overlay is OK; lat/long stay off until Drive + a new engage.

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

**Driver-turn blinker + soft-lat (On):** lamp latch / held stalk must **not** clear `latActive` or force EPS free. If soft-lat (or stock lat) still has control when the blinker rises, **keep control**. Soft-lat may still yield if the driver pushes (intent / hands) — that is the driver taking control, not the blinker stripping it. While the driver-turn blinker is latched, do **not** re-enable: if already yielded or blending, stay yielded / do not finish a take-back blend. After the blinker clears, do **not** special-case blend; if we were yielded / blending / lat-down, **land in soft yield** and let the normal resume own it (hands on / firm push stay free; hands 0 for `HANDS_OFF_CONFIRM_S` ~0.15 s, then the 1 s smoothstep). **v_ego strictly below 10 mph** (`LAT_REENABLE_MIN_V_EGO_MPH`) uses that **same** re-enable inhibit (OR, not a parallel path): keep control if we still have it; already yielded / blending stay yielded; after speed crosses 10 mph, land in yield then the normal resume. Blinker-on still blocks take-back at any speed. Above 10 mph with the blinker off, prior resume is unchanged. ALC tip / keep-alive flashes are **not** a driver turn. **Soft-lat Off** keeps today’s blinker lat-pause so a held turn still frees the wheel (speed does not pause lat on that path). A latched driver turn does **not** pause long. Brake sticky-MAX silent pause + one SET / double SET and panda blinker latch are unchanged. Hazards still do not pause.

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

- `selfdrive/controls/lib/driver_lateral_handoff.py` — firm-push detector (torsion + hands; rate not an entry gate), `lat_active_after_handoff` (yield = free EPS), low-torsion disturbance veto, emergency definition, hands-on hold, smoothstep, curvature pin, `lat_reenable_inhibited` (blinker OR v_ego < 10 mph)
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
7. **Light brake** (tap, not a panic stop) while engaged, yielded or not: silent long pause + held MAX; one SET resumes **if already moving**. At a stop, one SET must **not** roll the car; tap the accelerator, then long resumes at held MAX. No disengage chime. Sticky MAX (#72+#77) unchanged. Brake that knocks long off must **RELEASE interceptor immediately** to stock Tesla regen (firm, not a 0.75 s ramp or coast-then-bite). FCW / AEB / hard lead braking must still bite.
8. **Hard brake during / just after a soft-lat dodge** (firm pedal, strong decel): full OP cancel, “Steering Disengaged”, disengage chime. Not a silent pause. One SET is a new engage, not resume-at-held-MAX.
9. **Blinker turn** (held stalk), Soft-lat **On**: blinker-on alone must **not** free the wheel. A push may still yield. Soft-lat must not take lat back while the driver-turn lamps are latched. After the blinker clears, if lat was yielded, ease back through yield + ~0.15 s hands-off + **1 s** blend (not an immediate blinker blend). Soft-lat **Off**: existing lamp-latch lat pause (free wheel). Long stays engaged through the turn on both settings. Tip ALC must not latch a driver turn.
10. **Below 10 mph**, Soft-lat **On**: same no-lat-re-engage as blinker-on. If lat is still on, keep it. If already yielded / blending / lat-down, stay free until **above 10 mph**, then the normal 0.15 s hands-off + 1 s blend. Blinker-on still blocks take-back at any speed. Long / follow unchanged. Tip ALC / held-turn classification unchanged.
11. **ALC tip** still arms a lane change; wheel nudge at 1 Nm still starts it.
12. **Hard yank / hands-on 2**, stalk cancel, door: full disengage, unchanged.
13. **Reverse while engaged:** session ends (not a pause). No take-control-immediately alarm — OP just goes off. After Drive a normal double SET must engage without Controls Mismatch and without a prior on-device disable. Soft-lat / blinker / sticky MAX must not stay wedged.
14. **Gas-lift open-road climb (A3):** below MAX, no lead: engage-on-gas then lift (or tap gas while long is on). Takeover should climb toward MAX at Mannerisms **Acceleration** 1–10 — Accel 1 lazy, Accel 10 quicker — not sit at post-lift aEgo / coast. Must not punch past HUD MAX. **Lead / FCW:** must still slow; no climb through a car ahead. **Engage-on-gas then lift** still works with **One-Pedal Long On**. **Tap gas while long is already on** is One-Pedal **Off** (On would pause long).
15. **One-Pedal Long On:** Mannerisms toggle Off by default. Turn On. Foot off, long holding speed: press gas from rest — long should **pause** silently (lat stays, no disengage chime, session still up). Drive on the pedal; lift should regen (Tesla stock via interceptor RELEASE, not OP climbing back to MAX). Press gas again: accel, still no OP long, still not a full disable. One SET resumes at held MAX (same as brake pause). **Foot already on the accelerator, then one or two SET pulls:** long must arm; lift must start long (A+B / A3) — must not pause. Toggle Off: gas tap then lift should resume long and climb (A3). Brake still pauses long. At a stop, one SET then gas still resumes (does not pause).

## Driver monitoring simulate looking

Stock DM stays on while engaged. Vision timeouts stay **3 / 5 / 11 s** (`DRIVER_MONITOR_SETTINGS` in `selfdrive/monitoring/policy.py`). The #103 hands-on-only first-band reset did not stop look-at-road nagging: light rim contact is not what `dmonitoringd` treats as looking, orange/red still fired, and `maybe_distracted` (no face / uncertain) kept draining.

Triple-tap **NAP** (1.0 s window → side popup; not under Driving Mannerisms) order: **Force Offroad**, **Simulate Look**, **False Alert Ignore**. On nap-dev, Simulate Look defaults **On** and False Alert Ignore defaults **Off**. They are **mutually exclusive** — turning one On clears the other (and aborts the other in-flight path). Both Off is allowed. A stale both-On from older installs resolves to Simulate Look On / FAI Off on first read. Shared cadence: after awareness has gone **past 1.0 s**, fire uniform in **(1.0 s, 3.0 s]** of that countdown and **hold** (minimum 0.5 s, until awareness is 1.0). Not a mute and not `driver_interacting` full reset.

`NAPDmSimulateLooking` restores the pre-FAI **full looking-path wipe** on that cadence: face + low std + `driver_distracted` / filter + pose / eye / phone type bits. That recovers no-face, uncertain, phone, pose, and eye false nags the way Simulate Look did before the False Alert Ignore split. It is not a mute of hard cancels.

`NAPDmFalseAlertIgnore` (only when Simulate Look is Off) owns the **phone-only** path: when `phoneProb` is above `_PHONE_THRESH` (0.5) and pose/eye are **not** alarming, it soft-clears only `distracted_types['phone']` so a false device “Driver Distracted” can recover without a real glance. If pose or eye are actively alarming, it does **not** start or keep a hold — those timers keep draining. Phone + pose together: pose still drains; once pose/eye are clear, remaining phone can soft-clear on the same cadence.

Below **2 mph** (`DM_LOOKAWAY_GATE_MPH`; strictly below, ~0.89 m/s) looking-away / distraction / eyes-off-road alerts do **not** fire, whether Simulate Look / FAI are On or Off. This reuses the stock standstill exemption (pause before the green prompt; recover if already orange) and ORs in ego speed, because Pre-AP `CS.standstill` is only true when fully stopped — creeping at a light still nagged. Toggles stay as set. Above 2 mph, Sim Look / FAI / stock 3 / 5 / 11 s are unchanged. FCW / AEB / hard cancels are not this path.

Always-on DM when not engaged does **not** simulate or ignore. Either toggle **Off** = that path is stock. Mutex: only one On. Hands-on ≥ 2 / steer disengage, door, reverse, and stalk cancel are unchanged.

## Where to look

- `opendbc_repo/opendbc/car/tesla/preap/engagement.py` — stalk FSM
- `opendbc_repo/opendbc/car/tesla/preap/carstate.py` — interceptor `gasPressed` then One-Pedal Long pause
- `selfdrive/car/tesla/preap_blinker_lat_pause.py` — blinker lat pause / soft-lat gate (does not drop long on driver turn)
- `opendbc_repo/opendbc/car/tesla/preap/carstate.py:120+` — button event pump + cruise state publish
- `opendbc_repo/opendbc/car/tesla/preap/carcontroller.py` — pedal TX, engage grace, gas-lift VDAS seed, tip-brake glide authority
- `selfdrive/controls/tests/test_tesla_preap_gas_lift_handoff.py` — gas-lift grace expire + Accel 1–10 climb seed
- `selfdrive/controls/tests/test_tesla_preap_one_pedal_long.py` — One-Pedal Long pause + RELEASE regen (no ENABLE chatter / no USER_DISABLE)
- `selfdrive/controls/tests/test_tesla_preap_brake_cancel_regen.py` — tip-brake comfort ramp vs hold/firm RELEASE
- `opendbc_repo/opendbc/car/tesla/preap/brake_tip_glide.py` — tip/hold classifier + comfort-shaped regen ramp
- `opendbc_repo/opendbc/car/tesla/preap/tests/test_brake_tip_glide.py` — tip vs hold + gentle→strong ramp
- `selfdrive/car/tesla/preap_force_offroad_handoff.py` — Force Offroad cancel→STANDBY→drop long→SET; kill stock CC on OP long engage
- `opendbc_repo/opendbc/car/tesla/preap/tests/test_preap_engagement.py` — FSM tests (brake drop, steering disengage, double-pull)
- `selfdrive/car/tesla/tests/test_preap_force_offroad_handoff.py` — handoff ordering + kill-on-long
- `selfdrive/car/tesla/tests/test_preap_blinker_lat_pause.py` — blinker lat pause, latched turn keeps long, brake still drops long
- `selfdrive/car/tesla/tests/test_preap_sticky_max.py` — sticky MAX across brake pause, one vs double SET, cancel
- `selfdrive/monitoring/dm_toggles.py` — mutually exclusive write/read helpers (Simulate Look wins stale both-On)
- `selfdrive/monitoring/policy.py` — Simulate Look (`NAPDmSimulateLooking`) full looking-path wipe; False Alert Ignore (`NAPDmFalseAlertIgnore`) phone-only soft-clear; `DM_LOOKAWAY_GATE_MPH` (2 mph) look-away pause
- `selfdrive/monitoring/test_monitoring.py` — Simulate Look full-wipe recover; FAI phone-only vs pose/eye; Off is stock; mutual exclusion; look-away pause below 2 mph
