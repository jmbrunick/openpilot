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
5. A latched **driver turn** blinker (held stalk / flash-latched lamps, not ALC tip or keep-alive) also drops `enableLongControl` the same way, with the same sticky-MAX rules and the same silent pause (no pedal-cruise disengage sound). Long stays off after the lamps go dark unless the driver SETs. Lateral still resumes only after hand release. **One stalk SET** restores **longitudinal only**, MAX = held MAX (which may already have rebased if posted changed under maps). That resume is quiet — not a full-stack engage fanfare. Lat may still be paused for hand-release. The default double-pull first-pull path is skipped so a second SET is not required to get long back. SET while a turn lamp or held LEFT/RIGHT is still showing does not stick. **Double SET** (second pull inside `double_pull_window_ms`) forgets sticky and ensures lat+long on: maps on + posted known → MAX = current posted; maps off / posted unknown → MAX = current traveled speed. Tip ALC must not use this long-pause path.
6. Gas press → `OVERRIDE_LONGITUDINAL`. Pedal command passes through with `enable=0` — driver's foot controls throttle directly, NAP tracks position for smooth resume. Long engage/disengage prompts follow `enableLongControl` for **session** edges (initial engage / full cancel), not interceptor handshake, not gas override, and not a brake/turn long pause. `enableLongControl` stays true while the driver is on the pedal, so press and release are silent.
7. Stalk cancel (real, not spoof echo) → full disengage. `AudibleAlert.disengage` / `EventName.pedalCruiseDisabled` fire on that session teardown, including when openpilot is already going disabled. Door / gear / permanent steer fault / hands-on ≥ 2 (when not blinker-latched) are the same hard cancel: session down, held MAX forgotten, disengage prompt kept.

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
- **Maps off / posted unknown:** MAX never auto-rebases. Never invent a posted value. GPS glitch → keep held MAX.
- **One SET** (session already up, typical after long pause): resume long only; write `held_max_kph` (already rebased if posted changed).
- **Double SET:** forget sticky; maps+posted known → write current posted; else write current traveled speed. Same gesture from fully disengaged is initial engage.

No-pedal / stock CC does not own software MAX. Brake does not pause NAP long (stock CC handles brake). Double-pull still engages lat; set speed stays with Tesla CC.

## Steering disengage

`handle_steering_disengage()` runs on every frame. Rising edge of `steeringDisengage` (hands-on level ≥ 2 or EPAS rejecting) fully tears down FSM state: `cruiseEnabled=False`, `enableLongControl=False`, `pedal_speed_kph=0`, pull timers cleared.

On Pre-AP that falling `cruiseState.enabled` is `EventName.pcmDisable`. The 3X HUD string is **"Steering Disengaged"** (`pcm_disable_alert`) — not `EventName.steerDisengage`, which is sound-only. Do not hide `pcmDisable` after cruise is already down; keep `cruiseEnabled` during a blinker-held / lat-paused turn so the alert never fires. Hard-gate teardown on one lamp, physical LEFT/RIGHT, or the flash-latch hold. Door / gear / stalk cancel / permanent steer fault still fully disengage.

This is the primary user override path when the driver is *not* in a blinker turn. The panda also enforces it independently via EPAS error codes 6–9 and hands-on level ≥ 2 — except during a blinker-latched driver turn, where those conditions must not drop `controls_allowed` (otherwise selfdrived fires `controlsMismatch` after 2s and fully cancels while the lamps are still flashing).

Panda firmware must be flashed after this safety change. Software update / on-device `scons` rebuilds the panda image from `opendbc_repo/opendbc/safety/modes/tesla_preap.h` (latch in `tesla_preap_blinker.h`); reboot so `pandad` sees the new signature and flashes. Python `BlinkerLateralHold` alone cannot keep `controls_allowed`.

## Driver-wheel temporary lateral handoff

**Default On.** Settings → NAP → Soft Lateral Handoff can be turned **Off**. Yield is a **sustained driver push** with a hand on the rim (generally avoiding something), not gravel spike trains / wind / road-crown false pressure. #71–#75 tuned torsion + hands-on hold; those hold / 1 s resume rules stay. #78 also required aligned steer rate and treated high tracking error without that rate as a disturbance — that blocked the on-car *fight the wheel* dodge (isometric push, high path error, low rate) until EPAS `handsOnLevel >= 2` / `STEER_THRESHOLD` hard-cancelled. Soft yield must win **before** that hard path.

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

1. `|torsion| >= 0.70 Nm` (70% of `STEER_THRESHOLD`; still above 0.5 Nm rumble, still below `steeringPressed` 1.0 Nm)
2. `handsOnLevel >= 1` (hand on the rim)
3. Held for **140 ms** at the 0.70 Nm floor. Fewer frames as torsion approaches `STEER_THRESHOLD` (down to **80 ms** at 1.0 Nm) so the soft path beats hands-on ≥ 2. Never below 80 ms (5-frame gravel bursts stay rejected).

Do **not** require high steer rate. An isometric fight (firm torsion, wheel barely moving, high tracking error) **must** yield. Rate may stay as a weak wind filter only when torsion is **below** 0.70 Nm — it never blocks a firm push and never promotes low torsion to intent.

Disturbance veto only when torsion is **below** the yield trigger: `|desired − measured| curvature >= 0.0025` **and** `|torsion| < 0.70 Nm` is wind / crown — do not yield. High torsion is intent even if tracking error is large (driver is leaving the path).

Release hysteresis stays **0.40 Nm** (press-latch only). Do **not** go back to raw 0.5 Nm / 80 ms torsion-only. ≥ 2 stays the hard/safety path.

### Behavior (lat yield)

1. OP engaged and providing lateral. Intent detected → lateral authority goes to 0. `carControl.latActive` stays **true** (no `LaC.reset`, no snap in `apply_steer_angle_limits_vm`). While yielded, `desired_curvature` is **pinned to measured** so LaC cannot run ahead of the wheel. Longitudinal and `cruiseEnabled` stay up. Stalk cancel / doors / hands-on ≥ 2 still hard-disengage.
2. Stay at 0% while still maneuvering: **`EPAS_handsOnLevel >= 1`** **or** a renewed **≥ 0.70 Nm** firm push. Mid-dodge torsion dips must **not** start the hand-back. Rate is not a hold signal (caster / road after release).
3. After hands go to **0** for **~80 ms**, a **1.0 s** smoothstep (`t²(3−2t)`) blends authority 0 → 1. `QUIET_WAIT_S` stays 0. The pin lifts when blend starts. `apply_lat_authority` is skipped at authority=1.
4. Renewed **hands-on** or a firm push ≥ 0.70 Nm during the blend immediately yields (rate agreement is not re-required mid-maneuver). Hands off ~80 ms and the 1 s blend retries.
5. Yield smoothing: authority drops in one 10 ms cycle; the Tesla VM limiter still slews the CAN angle. Hand-back is the S-curve, not a step.
6. HUD: `controlsState.latHandoffPaused` stays set until authority ≥ **0.70**. Gray **override**, no sounds. Green + on-screen path is not enough — the wheel must unwind onto that path.

Blinker tip/hold ALC, lat pause, long drop + one SET, and panda blinker latch are unchanged. Soft-yield is **gated off** during a blinker lat-pause. Resume still **pins to the wheel while lat is down** and starts the same **1 s blend** on the rising edge.

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

- `selfdrive/controls/lib/driver_lateral_handoff.py` — firm-push detector (torsion + hands; rate not an entry gate), low-torsion disturbance veto, emergency definition, hands-on hold, smoothstep, curvature pin
- `selfdrive/controls/controlsd.py` — pin `desired_curvature` while yielded; pass torsion / rate / hands / tracking error / digital brake / aEgo; cancel on emergency
- `selfdrive/car/tesla/preap_blinker_lat_pause.py` — publish hands-on + digital brake; card-local handoff; `hard_cancel_session`
- `selfdrive/ui/ui_state.py` — gray override chrome while paused
- `selfdrive/controls/lib/tests/test_driver_lateral_handoff.py`

### On-car test plan (Pre-AP Model S / comma 3X)

Do these at a quiet road / parking lot first, then a known pothole stretch. Pedal or no-pedal both OK. Do **not** expect a panda flash.

1. **Engage** with a double-pull. Confirm green engaged chrome and that long/cruise is holding speed.
2. **Intentional dodge** (hands on, sustained torsion ≥ ~0.70 Nm for ~0.14 s — the wheel does **not** have to be moving). Includes fighting OP to leave the path (high tracking error, low rate). Lateral should go slack **before** a hard yank / hands-on ≥ 2. Speed control must stay on. HUD should go **gray override**, not “Steering Disengaged”, and must not play the disengage chime.
3. **Hold** through a pothole dodge (hands still on the rim). Authority must stay yielded even if torsion dips.
4. **Release.** Hands clearly off the rim. After ~80 ms the wheel eases back onto the path over about **one second**. When green returns, the wheel must follow the on-screen path.
5. **Re-grab during the blend.** Hands back on or a firm push: yield again. Hands off ~80 ms and the 1 s blend retries.
6. **Road rumble / crosswind / crown** with hands resting: must **not** gray. Wind that makes NAP fight the path (high tracking error, low torsion) must **not** yield. If it still does, turn Soft Lateral Handoff **Off**. Do not lower panda / `STEER_THRESHOLD`.
7. **Light brake** (tap, not a panic stop) while engaged, yielded or not: silent long pause + held MAX; one SET resumes. No disengage chime. Sticky MAX (#72+#77) unchanged.
8. **Hard brake during / just after a soft-lat dodge** (firm pedal, strong decel): full OP cancel, “Steering Disengaged”, disengage chime. Not a silent pause. One SET is a new engage, not resume-at-held-MAX.
9. **Blinker turn** (held stalk): existing lat pause, long drop, one SET resume. Soft-yield must not steal this or arm ALC from a tip. After the turn, lat must **ease** back over ~1 s from the wheel.
10. **ALC tip** still arms a lane change; wheel nudge at 1 Nm still starts it.
11. **Hard yank / hands-on 2**, stalk cancel, door: full disengage, unchanged.

## Where to look

- `opendbc_repo/opendbc/car/tesla/preap/engagement.py` — stalk FSM
- `selfdrive/car/tesla/preap_blinker_lat_pause.py` — blinker-turn long drop + one-SET resume (patches the FSM)
- `opendbc_repo/opendbc/car/tesla/preap/carstate.py:120+` — button event pump + cruise state publish
- `opendbc_repo/opendbc/car/tesla/preap/carcontroller.py:40+` — pedal TX + stalk spoof scheduling
- `opendbc_repo/opendbc/car/tesla/preap/tests/test_preap_engagement.py` — FSM tests (brake drop, steering disengage, double-pull)
- `selfdrive/car/tesla/tests/test_preap_blinker_lat_pause.py` — blinker lat pause, turn drops long, one SET resumes
- `selfdrive/car/tesla/tests/test_preap_sticky_max.py` — sticky MAX across brake/turn pause, one vs double SET, cancel
