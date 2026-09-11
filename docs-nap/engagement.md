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
4. Brake rising edge → drops `enableLongControl=False`, keeps `cruiseEnabled=True` (steering stays on, pedal drops). `enableJustCC` flips true so the carcontroller knows to spoof stock CC cancel. This is a **silent long pause**, not a full disengage: no `EventName.pedalCruiseDisabled` / no disengage chime / no HUD “Pedal Cruise Disengaged”. Held MAX (`pedal_speed_kph` / `MapCruiseHold.held_max_kph` / sticky set) is remembered.
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
- **Maps off / posted unknown:** MAX never auto-rebases. Never invent a posted value. GPS glitch → keep held MAX. Do not latch pause ego (`cruiseState.speed`) into held. Stalk +/- while long is active updates the MAX one SET will resume.
- **One SET** (session already up, typical after long pause): resume long only; write `held_max_kph` (already rebased if posted changed). `resume_held` is a one-shot; `engage_rising` can arrive a frame later (`pedalLongActive` lags `enableLongControl`). That delayed rising edge must not take-now and overwrite a held MAX with current traveled speed.
- **Double SET:** forget sticky; maps+posted known → write current posted; else write current traveled speed. Same gesture from fully disengaged is initial engage.

No-pedal / stock CC does not own software MAX. Brake does not pause NAP long (stock CC handles brake). Double-pull still engages lat; set speed stays with Tesla CC.

## Steering disengage

`handle_steering_disengage()` runs on every frame. Rising edge of `steeringDisengage` (hands-on level ≥ 2 or EPAS rejecting) fully tears down FSM state: `cruiseEnabled=False`, `enableLongControl=False`, `pedal_speed_kph=0`, pull timers cleared.

On Pre-AP that falling `cruiseState.enabled` is `EventName.pcmDisable`. The 3X HUD string is **"Steering Disengaged"** (`pcm_disable_alert`) — not `EventName.steerDisengage`, which is sound-only. Do not hide `pcmDisable` after cruise is already down; keep `cruiseEnabled` during a blinker-held / lat-paused turn so the alert never fires. Hard-gate teardown on one lamp, physical LEFT/RIGHT, or the flash-latch hold. Door / gear / stalk cancel / permanent steer fault still fully disengage.

This is the primary user override path when the driver is *not* in a blinker turn. The panda also enforces it independently via EPAS error codes 6–9 and hands-on level ≥ 2 — except during a blinker-latched driver turn, where those conditions must not drop `controls_allowed` (otherwise selfdrived fires `controlsMismatch` after 2s and fully cancels while the lamps are still flashing).

Panda firmware must be flashed after this safety change. Software update / on-device `scons` rebuilds the panda image from `opendbc_repo/opendbc/safety/modes/tesla_preap.h` (latch in `tesla_preap_blinker.h`); reboot so `pandad` sees the new signature and flashes. Python `BlinkerLateralHold` alone cannot keep `controls_allowed`.

## Where to look

- `opendbc_repo/opendbc/car/tesla/preap/engagement.py` — stalk FSM
- `selfdrive/car/tesla/preap_blinker_lat_pause.py` — blinker-turn long drop + one-SET resume (patches the FSM)
- `opendbc_repo/opendbc/car/tesla/preap/carstate.py:120+` — button event pump + cruise state publish
- `opendbc_repo/opendbc/car/tesla/preap/carcontroller.py:40+` — pedal TX + stalk spoof scheduling
- `opendbc_repo/opendbc/car/tesla/preap/tests/test_preap_engagement.py` — FSM tests (brake drop, steering disengage, double-pull)
- `selfdrive/car/tesla/tests/test_preap_blinker_lat_pause.py` — blinker lat pause, turn drops long, one SET resumes
- `selfdrive/car/tesla/tests/test_preap_sticky_max.py` — sticky MAX / silent long pause / one SET / double SET
- `selfdrive/mapd/tests/test_map_speed_policy.py` — maps rebase only on posted-value change
