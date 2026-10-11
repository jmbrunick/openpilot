# Force Offroad (Go Offline)

Hidden Settings → NAP toggle (triple-tap the **NAP** sidebar / **nap** button) so the comma can enter the same **started=false / offroad** path used for map download, Refresh maps, software install, and other offroad-only NAP actions **while the car is still moving**. Not on the normal NAP list.

## Why

Those actions gate on `ui_state.is_offroad()` (`not started`). Stock openpilot only goes offroad when ignition is off (or thermal / onroad-cycle). Parking for every pull around Benson is the pain this avoids.

## Param

| | |
|---|---|
| Name | `NAPForceOffroad` |
| Default | Off (`0`) |
| Flags | `CLEAR_ON_MANAGER_START \| CLEAR_ON_IGNITION_ON` (not persistent) |

On-road confirm (UI → params), then handshake (card → hardwared):

| | |
|---|---|
| Name | `NAPForceOffroadHandoffReady` |
| Default | Off (`0`) |
| Flags | `CLEAR_ON_MANAGER_START \| CLEAR_ON_IGNITION_ON` (not persistent) |

| | |
|---|---|
| Name | `NAPForceOffroadConfirmed` |
| Default | Off (`0`) |
| Flags | `CLEAR_ON_MANAGER_START \| CLEAR_ON_IGNITION_ON` (not persistent) |

Do **not** flag either param `CLEAR_ON_OFFROAD_TRANSITION` — going offroad is what the toggle does, so that flag would immediately undo it.

## How it hooks

`hardwared` reads `NAPForceOffroad` every loop into `onroad_conditions["not_force_offroad"]`. When the toggle is on **and** the device is not already onroad, `should_start` is false → `deviceState.started = false` (today's parked path). No confirm popup when already offroad / parked.

If the device is **already onroad**:

1. A full-screen **Yes / No** modal (not a Settings row): **“Ready to resume steering control?”** Large type and large buttons so it is hard to miss while driving. Same `gui_app.push_widget` stack as other 3X ConfirmDialogs (`selfdrive/ui/onroad/force_offroad_confirm.py`).
2. **No** (or Escape) cancels the whole Force Offroad process: clears the toggle, confirm, and handoff-ready flags. NAP/OP stay as they were. No cancel/SET, no `started=false`.
3. **Yes** sets `NAPForceOffroadConfirmed`. Only then card may start the stock-CC handoff.
4. hardwared then holds `started` until card writes `NAPForceOffroadHandoffReady` (or **3.0 s** fallback, which does **not** run until Yes) so the onroad carcontroller can still spoof stalk CANCEL / SET. Then `started=false`.

Then the existing chain runs:

1. **manager** sees `deviceState.started` false → stops the onroad stack (`controlsd` / `selfdrived` / …) and writes `IsOffroad=true` / `IsOnroad=false`.
2. **ui_state** `started = deviceState.started and ignition` → `is_offroad()` true → Download US Maps, Refresh maps, software install, pedal/EPAS scripts unlock.

There is no UI-only override. A cosmetic button would leave openpilot able to engage.

## Pre-AP stock-CC handoff (pedal / software long)

Gate: `enableLongControl` (NAP is commanding software long). Not the interceptor handshake / `pedalLongActive`. Lat-only, stock-CC mode, or not engaged: skip the handoff and write ready immediately.

CANCEL / SET live in `StockCCSpoofer` (onroad). **Do not** flip `started=false` until stock CC is **ENABLED** (prefer) or the timeout.

Sequence when Force Offroad turns ON while software long is active (**after Yes**):

1. Drive DI toward **STANDBY**. `DI_cruiseState` from `DI_state`:
   - **ENABLED** / STANDSTILL / OVERRIDE / PRE_CANCEL → spoof **CANCEL** (existing `preap_cc_cancel_needed`). Tesla typically drops ENABLED → STANDBY. Keep OP long on during this so something still holds speed.
   - **STANDBY** → already ready to SET.
   - **OFF** (SET_ACCEL is ignored on an unarmed DI) → spoof **MAIN** (RWD pull) to arm. If MAIN SETs in one step, skip the extra SET.
2. Once **STANDBY** (or DI already ENABLED from MAIN): **drop `enableLongControl`** (silent; `cruiseEnabled` stays so lat does not dump before offroad). Pedal long falling would otherwise set `preap_cc_cancel_needed` and abort ENGAGING — that cancel is suppressed for this handoff.
3. **Immediately** `preap_cc_engage_needed` → SET_ACCEL at current ego (existing ENGAGING FSM, retries on the 10 Hz slot, 500 ms timeout).
4. When DI is **ENABLED** (or STANDSTILL), write `NAPForceOffroadHandoffReady`. Only then hardwared sets `started=false`.

Timing (100 Hz card / carcontroller):

| Step | Budget |
|------|--------|
| CANCEL_DELAY_FRAMES | 10 (100 ms) + `frame % 10` slot |
| STANDBY / ARM wait | 0.80 s then fallback |
| ENGAGING retries | `CC_ENGAGE_TIMEOUT_FRAMES` = 50 (500 ms) |
| ENABLED wait | 0.80 s then fallback |
| card overall | 2.5 s then write ready anyway |
| hardwared fallback | 3.0 s if card never writes ready |

Prefer waiting for ENABLED. The timeouts are so maps still unlock if DI never takes SET (below min cruise speed, unarmed, CAN miss).

## Clear policy

Default **Off**. Safer to auto-clear than to stay forced-offroad on the next drive.

| How | Result |
|---|---|
| Toggle Off | Goes onroad again if ignition is on and other start conditions pass |
| Reset to Defaults | Writes `NAPForceOffroad=false` (3X NAP panel) |
| Reboot / manager start | Param cleared |
| Next ignition ON | Param cleared (park, then start → assist is back) |

## How to use (on the road)

1. Settings → triple-tap **NAP** (3 taps within 1.0 s, sliding window) → **Force Offroad** on the side popup (3X) or the nap-button overlay (mici / comma 4). Tap outside the card to dismiss. Leave it available while onroad — that is the point. Not on the normal NAP list or Driving Mannerisms.
2. On-road: a big popup asks **Ready to resume steering control?** **Yes** continues; **No** turns Force Offroad back off and leaves assist as it was.
3. If pedal long was holding speed **and you tapped Yes**: stock CC should take over at about the current speed, then the onroad stack stops. **Drive manually** after that. Do not expect steering or accel assist.
4. Settings → NAP → Map Speed Limit → **Download US Maps** / **Refresh maps** (or Software install) should be tappable, same as when parked.
5. Toggle Off when done, or park and cycle ignition.

## Retest (Justin — Pre-AP Model S + comma 3X)

Hold a steady speed on **pedal long**, then triple-tap NAP → Force Offroad **ON**:

1. Big popup: **Ready to resume steering control?** **No** must leave pedal long / OP as they were (toggle goes back Off, no regen dump).
2. **Yes**, then stock CC should go **ENABLED** at about the current speed (no intentional “nothing holding speed” window / hard regen bite).
3. Then maps / Refresh / install unlock (comma is offroad).
4. If already lat-only or stock-CC mode: Yes still required on-road; then offroad immediately (no CC handoff). Parked / already offroad: no popup.

Also: engage OP **long** with stock CC already **ENABLED** or **STANDBY** — stock CC must cancel off so it does not fight the pedal. See [engagement.md](engagement.md).

## Warning

Forces offroad and disengages openpilot. Drive manually. Do not expect assist while this is on.
