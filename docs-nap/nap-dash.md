# NAP companion Dash (lite)

Lean hotspot page on branch `cursor/nap-dash-lite-36e3`. Process: `nap_dash` (`selfdrive.nap_dash.server`). Starts with openpilot when `NAPDashEnabled` is On (default).

**Do not merge** this pull request into `nap-dev` until Justin confirms on-road engage is clean. The full Philip-style Dash (PR #227) drained high-rate cereal and caused `EventName.commIssue`. This build does not do that. PR #228 reverted Dash off `nap-dev`.

## No high-rate sockets

Settings and software need only Params and HTTP. Confirmed in `settings.py` and `system_api.py`: both talk to `Params` (software also sends `SIGUSR1` to `updated`). Neither reads car messages.

`selfdrive/nap_dash/server.py` does not construct a cereal subscriber. There is no live loop, no `can`, no `modelV2`, and no other on-road service drain. `/api/state` is an empty stub so an old page does not 500.

Removed versus the Philip dashboard:

- telemetry subscriber and 10 Hz loop (`carState`, `selfdriveState`, `controlsState`, `radarState`, `deviceState`, `modelV2`, `can`)
- BMS decode, cluster path drawing, lead counters, engagement distance from cereal
- dashcam route browser, ffmpeg HUD export, and `/stream` from `realdata`

Kept:

- HTTP on `:7070`
- `GET/POST` settings → Driving Mannerisms Params
- `GET/POST /api/software` (`set_branch`, `fetch`, `download`, `set_offline` / `DisableUpdates`)
- optional manager process so a Dash crash does not raise `processNotRunning`

## Engage safety

`nap_dash` is an **optional / non-critical** manager process. A Dash crash, `:7070` bind failure, or preimport error must **not** raise `processNotRunning` (`Process Not Running` / `nap_dash`) and must **not** fail `manager` start. That alert is NO_ENTRY + SOFT_DISABLE — it prevents engage entirely.

No carstate / cereal / panda / pedal-interceptor changes. Dash does not write engagement Params at startup. Stalk and long mannerisms stay on the nap-dev driving code; Dash only reads and writes the same Params.

## What it is

Phone-friendly settings page on the comma device:

- Driving Mannerisms + Map Speed + stock personality / Experimental Mode + SL / FAI
- Software page: `GET/POST /api/software` (`UpdaterTargetBranch` + SIGUSR1, `set_offline` / `DisableUpdates`)

Hypermile is not on this page.

## What it is not

- Not a live cluster, BMS view, or dashcam
- Not Philip City Turns / Tap LC / Corner Assist / Lane Centering
- Not cruise trim / +5 mph speed-offset injector
- Not nav injection / phone nav remote
- Not Cloudflare / ngrok / Tailscale Funnel — no unauthenticated public internet
- Not Hypermile (that feature stays in the on-device Settings UI only)
- Not engage, uninstall, or path traversal (`/api/software` rejects those)

## How to flash

This is a Python + `NAPDashEnabled` Params-key change. **No panda flash.** The installer rebuilds `common` for the new key; panda firmware is unchanged.

On the comma 3X, Software → Custom Fork (or the installer URL):

```
https://installer.comma.ai/jmbrunick/openpilot/cursor/nap-dash-lite-36e3
```

Wait for the update to finish, then reboot so manager starts `nap_dash`.

## Dash off workaround

If Dash misbehaves, disable the process. Engage stays available:

SSH:

```
python3 -c "from openpilot.common.params import Params; Params().put_bool('NAPDashEnabled', False)"
```

Then reboot. Or set `BLOCK=nap_dash` in the launch environment.

Re-enable with `NAPDashEnabled=True` and a reboot. Default is On.

## How to open Dash

1. Join the **comma hotspot** (or the same LAN as the device).
2. In a phone/laptop browser: `http://<device-ip>:7070`

The hotspot address is often `http://192.168.43.1:7070`. `/` and `/phone` serve the same page.

The server binds `0.0.0.0:7070` so the hotspot can reach it. That is **LAN / hotspot only**. Do not expose the port with Funnel, ngrok, or Cloudflare. Override with `NAP_DASH_HOST` / `NAP_DASH_PORT` if you need a tighter bind.

## Settings map (Dash → Mannerisms Params)

Writes go only through `Params`. SL / FAI use the exclusive helpers. Hypermile is not writable from this Dash.

### Settings → NAP → Driving Mannerisms

| Dash control | Param |
|--------------|--------|
| Acceleration 1–10 | `NAPMapSpeedAccel` |
| Adaptive Accel | `NAPAdaptiveAccel` |
| Follow Distance 1–7 | `NAPFollowDistance` |
| Soft Lateral Handoff | `NAPDriverLatHandoff` |
| One-Pedal Long | `NAPOnePedalLong` |

### Map Speed (Settings → NAP → Map Speed Limit)

| Dash control | Param |
|--------------|--------|
| Map Speed (MAX) Off / Display / Cap / Follow | `NAPMapSpeedMode` |
| Offset −5 / 0 / +5 mph | `NAPMapSpeedOffsetMph` |
| Lookahead Off / Late / Normal / Early | `NAPMapSpeedLookahead` |

### Stock UI + optional DM

| Dash control | Param |
|--------------|--------|
| Driving Personality | `LongitudinalPersonality` |
| Experimental Mode | `ExperimentalMode` |
| SL | `NAPDmSimulateLooking` |
| FAI | `NAPDmFalseAlertIgnore` |

Toggling a control in Dash must change the same Params the on-device NAP UI reads. After a Dash write, Settings → NAP should show the new value (and the reverse).

### Software (`/api/software`)

| Action | Effect |
|--------|--------|
| `GET /api/software` | Current branch, target branch, available branches, updater state, offline |
| `set_branch` | Writes `UpdaterTargetBranch`, then `SIGUSR1` to `updated` |
| `fetch` / `download` | `SIGUSR1` to `updated` (same wake-up as the comma Software UI) |
| `set_offline` | Writes `DisableUpdates` |

## CPU

`nap_dash` is an idle Python HTTP server. It does not poll cereal. There is no dashcam export.

## Tests

`selfdrive/nap_dash/tests/test_settings.py` — Params round-trip, SL/FAI exclusivity, reject Philip-only and Hypermile names.

`selfdrive/nap_dash/tests/test_server_smoke.py` — process_config wiring, HTTP `/api/set` follow-distance write, no cereal subscriber, no `/api/nav`.

`selfdrive/nap_dash/tests/test_system_api.py` — `/api/software` branch switch, fetch/download ping, Go Offline.
