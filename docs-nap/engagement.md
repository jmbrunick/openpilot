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
4. Brake rising edge → drops `enableLongControl=False`, keeps `cruiseEnabled=True` (steering stays on, pedal drops). `enableJustCC` flips true so the carcontroller knows to spoof stock CC cancel
5. A latched **driver turn** blinker (held stalk / flash-latched lamps, not ALC tip or keep-alive) also drops `enableLongControl` the same way. Long stays off after the lamps go dark unless the driver SETs. Lateral still resumes only after hand release. One stalk SET restores long, whether lat is still paused or already active, including during the ~1s dark latch after the last flash — the default double-pull first-pull path is skipped for that resume so a second SET is not required. SET while a turn lamp or held LEFT/RIGHT is still showing does not stick.
6. Gas press → `OVERRIDE_LONGITUDINAL`. Pedal command passes through with `enable=0` — driver's foot controls throttle directly, NAP tracks position for smooth resume. Long engage/disengage prompts follow `enableLongControl` (stalk/brake/driver-turn), not interceptor handshake and not gas override: `enableLongControl` stays true while the driver is on the pedal, so press and release are silent.
7. Stalk cancel (real, not spoof echo) → full disengage. Lat and long prompts fire on those falling edges, including when openpilot is already going disabled.

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

This is a driver-training feature: prevents accidental full engage from a bump of the stalk. Once already engaged, a brake or driver-turn drop of long is not a new first pull: one SET restores long.

## Steering disengage

`handle_steering_disengage()` runs on every frame. Rising edge of `steeringDisengage` (hands-on level ≥ 2 or EPAS rejecting) fully tears down FSM state: `cruiseEnabled=False`, `enableLongControl=False`, `pedal_speed_kph=0`, pull timers cleared.

On Pre-AP that falling `cruiseState.enabled` is `EventName.pcmDisable`. The 3X HUD string is **"Steering Disengaged"** (`pcm_disable_alert`) — not `EventName.steerDisengage`, which is sound-only. Do not hide `pcmDisable` after cruise is already down; keep `cruiseEnabled` during a blinker-held / lat-paused turn so the alert never fires. Hard-gate teardown on one lamp, physical LEFT/RIGHT, or the flash-latch hold. Door / gear / stalk cancel / permanent steer fault still fully disengage.

This is the primary user override path when the driver is *not* in a blinker turn. The panda also enforces it independently via EPAS error codes 6–9 and hands-on level ≥ 2 — except during a blinker-latched driver turn, where those conditions must not drop `controls_allowed` (otherwise selfdrived fires `controlsMismatch` after 2s and fully cancels while the lamps are still flashing).

Panda firmware must be flashed after this safety change. Software update / on-device `scons` rebuilds the panda image from `opendbc_repo/opendbc/safety/modes/tesla_preap.h` (latch in `tesla_preap_blinker.h`); reboot so `pandad` sees the new signature and flashes. Python `BlinkerLateralHold` alone cannot keep `controls_allowed`.

## Driver-wheel temporary lateral handoff

Pothole / obstacle dodges without fighting NAP and without a full disengage. Software-only; panda hands-on ≥ 2 and `STEER_THRESHOLD` are unchanged.

### Signals (Pre-AP)

| Signal | Source | Type | Role |
|--------|--------|------|------|
| `EPAS_torsionBarTorque` | `EPAS_sysStatus` 0x370 | continuous Nm (0.01, −20.5) | soft-yield |
| `StW_AnglHP_Spd` | `STW_ANGLHP_STAT` 0x0E | continuous deg/s (0.5, −4096) | quiet / still-turning |
| `EPAS_handsOnLevel` | same EPAS msg | discrete 0/1/2/3 | **not** the soft trigger. ≥ 2 remains hard disengage |

`steeringPressed` is still `|torsion| > STEER_THRESHOLD` (**1.0 Nm**) with **5-frame** debounce (~50 ms). That fires `EventName.steerOverride` (`OVERRIDE_LATERAL`, stays enabled) and is the current software override effort.

Soft-yield trigger is **0.5 Nm** (50% of that software threshold) with **8-frame** debounce (~80 ms) and release hysteresis at **0.3 Nm**. `handsOnLevel >= 1` is not used: stock EPAS already takes ~0.5 Nm for 0.25 s to raise level 1 (too slow for a dodge), and level 1 is a normal hands-on posture while OP is engaged.

### Behavior

1. OP engaged and providing lateral. Light wheel input → lateral authority goes to 0. `carControl.latActive` stays **true** (no `LaC.reset`, no snap in `apply_steer_angle_limits_vm`). Longitudinal and `cruiseEnabled` stay up. Stalk cancel / doors / hands-on ≥ 2 still hard-disengage.
2. Stay at 0% while the driver keeps torque above 0.3 Nm or steering rate above 25 deg/s (held correction / still turning). Rate is ignored during the hand-back blend because OP itself turns the wheel.
3. After **0.25 s** of quiet, a **1.0 s** smoothstep (`t²(3−2t)`) blends authority 0 → 1. Max slope 1.5 / s.
4. Renewed input ≥ 0.5 Nm during the blend immediately yields, resets the 0.25 s timer, and retries only after quiet.
5. Yield smoothing: authority drops in one 10 ms cycle; the Tesla VM limiter still slews the CAN angle (`MAX_ANGLE_RATE` = 5 deg / 20 ms = 250 deg/s, plus ~3.6 m/s³ jerk). Do not jump desired angle. Hand-back is the S-curve, not a step.
6. HUD: `controlsState.latHandoffPaused` stays set until authority ≥ **0.70**. 3X chrome uses the existing gray **override** (paused / driver-control) indication, not disengaged, and not the green lateral-engaged state. No engage/disengage sounds. Crossing 70% is latched so 69% cannot flicker green.

Blinker tip/hold ALC, lat pause, long drop + one SET, and panda blinker latch are unchanged. This path does not require blinkers and does not arm ALC (handoff is gated off while `laneChangeState != off`).

### Where to look

- `selfdrive/controls/lib/driver_lateral_handoff.py` — detector, quiet timer, smoothstep, tunables
- `selfdrive/controls/controlsd.py` — blend commanded angle/torque/curvature; publish `latAuthority` / `latHandoffPaused`
- `selfdrive/ui/ui_state.py` — gray override chrome while paused
- `selfdrive/controls/lib/tests/test_driver_lateral_handoff.py`

### On-car test plan (Pre-AP Model S / comma 3X)

Do these at a quiet road / parking lot first, then a known pothole stretch. Pedal or no-pedal both OK. Do **not** expect a panda flash.

1. **Engage** with a double-pull. Confirm green engaged chrome and that long/cruise is holding speed.
2. **Light rim input** (about half of the usual “OP is fighting me” effort, well below a yank). Lateral should go slack almost immediately. Speed control must stay on. HUD should go **gray override**, not “Steering Disengaged”, and must not play the disengage chime.
3. **Hold** a small offset for >1 s. Authority must stay yielded (no snap back to the lane).
4. **Release and sit quiet.** After a beat (~0.25 s) the wheel should ease back onto the path over about **one second**, not jump. Green engaged chrome returns only late in that blend (around 70%+), not at the first twitch.
5. **Re-grab during the blend.** The return must stop immediately and yield again. Gray chrome stays. After another quiet beat, the 1 s blend retries.
6. **Road rumble** with hands resting, no dodge. Must **not** repeatedly gray/green. If it does, the 0.5 Nm trigger is too low for that stretch — do not lower panda / `STEER_THRESHOLD`.
7. **Blinker turn** (held stalk): existing lat pause, long drop, one SET resume. Soft-yield must not steal this or arm ALC from a tip.
8. **ALC tip** (LEFT/RIGHT then IDLE within 0.40 s) still arms a lane change; wheel nudge at 1 Nm still starts it.
9. **Hard yank / hands-on 2**, stalk cancel, door: full disengage, “Steering Disengaged”, chime. That is still the safety path.

## Where to look

- `opendbc_repo/opendbc/car/tesla/preap/engagement.py` — stalk FSM
- `selfdrive/car/tesla/preap_blinker_lat_pause.py` — blinker-turn long drop + one-SET resume (patches the FSM)
- `opendbc_repo/opendbc/car/tesla/preap/carstate.py:120+` — button event pump + cruise state publish
- `opendbc_repo/opendbc/car/tesla/preap/carcontroller.py:40+` — pedal TX + stalk spoof scheduling
- `opendbc_repo/opendbc/car/tesla/preap/tests/test_preap_engagement.py` — FSM tests (brake drop, steering disengage, double-pull)
- `selfdrive/car/tesla/tests/test_preap_blinker_lat_pause.py` — blinker lat pause, turn drops long, one SET resumes
