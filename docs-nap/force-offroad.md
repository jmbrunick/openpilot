# Force Offroad (Go Offline)

Settings → NAP toggle so the comma can enter the same **started=false / offroad** path used for map download, Refresh maps, software install, and other offroad-only NAP actions **while the car is still moving**.

## Why

Those actions gate on `ui_state.is_offroad()` (`not started`). Stock openpilot only goes offroad when ignition is off (or thermal / onroad-cycle). Parking for every pull around Benson is the pain this avoids.

## Param

| | |
|---|---|
| Name | `NAPForceOffroad` |
| Default | Off (`0`) |
| Flags | `CLEAR_ON_MANAGER_START \| CLEAR_ON_IGNITION_ON` (not persistent) |

## How it hooks

`hardwared` reads the param every loop into `onroad_conditions["not_force_offroad"]`. When the toggle is on, `should_start` is false → `deviceState.started = false`.

Then the existing chain runs:

1. **manager** sees `deviceState.started` false → stops the onroad stack (`controlsd` / `selfdrived` / …) and writes `IsOffroad=true` / `IsOnroad=false`.
2. **ui_state** `started = deviceState.started and ignition` → `is_offroad()` true → Download US Maps, Refresh maps, software install, pedal/EPAS scripts unlock.

There is no UI-only override. A cosmetic button would leave openpilot able to engage.

## Clear policy

Default **Off**. Safer to auto-clear than to stay forced-offroad on the next drive.

| How | Result |
|---|---|
| Toggle Off | Goes onroad again if ignition is on and other start conditions pass |
| Reset to Defaults | Writes `NAPForceOffroad=false` (3X NAP panel) |
| Reboot / manager start | Param cleared |
| Next ignition ON | Param cleared (park, then start → assist is back) |

Do **not** flag this param `CLEAR_ON_OFFROAD_TRANSITION` — going offroad is what the toggle does, so that flag would immediately undo it.

## How to use (on the road)

1. Settings → **NAP** → **Force Offroad** (3X) or **force offroad** (mici / comma 4). Leave it available while onroad — that is the point.
2. openpilot disengages and the onroad stack stops. **Drive manually.** Do not expect steering or accel assist.
3. Settings → NAP → Map Speed Limit → **Download US Maps** / **Refresh maps** (or Software install) should be tappable, same as when parked.
4. Toggle Off when done, or park and cycle ignition.

## Warning

Forces offroad and disengages openpilot. Drive manually. Do not expect assist while this is on.

## Removing later

This feature is optional on `nap-release`. To take it out:

- revert the Force Offroad PR, or
- delete `system/hardware/nap_force_offroad.py`, `common/tests/test_nap_force_offroad.py`, `docs-nap/force-offroad.md`, the `NAPForceOffroad` param, the `hardwared` hook (`not_force_offroad` / `should_start_now`), and the Settings → NAP Force Offroad toggle (3X + mici). Drop the `docs-nap/README.md` / `map-speed.md` / `RELEASES.md` / `ui_state.py` markers.
