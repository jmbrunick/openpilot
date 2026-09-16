# NAP companion Dash

In-tree hotspot web UI for Justin's tree. Process: `nap_dash` (`selfdrive.nap_dash.server`). Always starts with openpilot.

**Do not merge this PR until Justin flashes the PR branch tip and road-tests it.** First-use is flash-the-tip, not merge-to-nap-dev.

## What it is

Phone-friendly page on the comma device:

- Live cluster (speed, MAX, steer, lead / lane sketch, pedals)
- Driving Mannerisms + Map Speed + stock personality / Experimental Mode + SL / FAI
- Tesla BMS decode (read-only)
- Dashcam viewer + on-demand HUD export (ffmpeg; opt-in, leave the tab closed while driving)

Adapted from Philip's NAP-Dash `server_v21` **UI shell**. The settings bridge is Justin's Params / Driving Mannerisms, not Philip's JSON trim path.

## What it is not

- Not Philip City Turns / Tap LC / Corner Assist / Lane Centering
- Not `/data/nap_settings.json` cruise trim / +5 mph speed-offset injector
- Not nav injection / phone nav remote
- Not Cloudflare / ngrok / Tailscale Funnel — no unauthenticated public internet

## How to flash the PR tip

This is a Python-only change. **No panda flash.**

On the comma 3X, Software → Custom Fork (or the installer URL):

```
https://installer.comma.ai/jmbrunick/openpilot/<this-pr-branch>
```

Use the **PR branch name**, not `nap-dev`. Wait for the update to finish, then reboot so manager starts `nap_dash`.

Confirm the tip SHA on-device matches the PR head before the first road test (`Settings → Software` / `git rev-parse HEAD` over SSH).

## How to open Dash

1. Join the **comma hotspot** (or the same LAN as the device).
2. In a phone/laptop browser: `http://<device-ip>:7070`

The hotspot address is often `http://192.168.43.1:7070`. `/` and `/phone` serve the same page.

The server binds `0.0.0.0:7070` so the hotspot can reach it. That is **LAN / hotspot only**. Do not expose the port with Funnel, ngrok, or Cloudflare. Override with `NAP_DASH_HOST` / `NAP_DASH_PORT` if you need a tighter bind.

## Settings map (Dash → Mannerisms Params)

Writes go only through `Params` (Hypermile uses `apply_hypermile_toggle` so snap/restore still works; SL / FAI use the exclusive helpers).

### Settings → NAP → Driving Mannerisms

| Dash control | Param |
|--------------|--------|
| Acceleration 1–10 | `NAPMapSpeedAccel` |
| Adaptive Accel | `NAPAdaptiveAccel` |
| Follow Distance 1–7 | `NAPFollowDistance` |
| Soft Lateral Handoff | `NAPDriverLatHandoff` |
| One-Pedal Long | `NAPOnePedalLong` |
| Hypermile | `NAPHypermile` (snap/restore via existing helper) |
| Step Down Speed | `NAPHypermileStepDown` |
| Hill Climb | `NAPHypermileHillClimb` |

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

## CPU

`nap_dash` is a light Python HTTP server + ~10 Hz SubMaster. Dashcam **export** is on-demand only. Do not leave a high-res export running on-road.

## Tests

`selfdrive/nap_dash/tests/test_settings.py` — Params round-trip, Hypermile snap/restore, SL/FAI exclusivity, reject Philip-only names.

`selfdrive/nap_dash/tests/test_server_smoke.py` — process_config wiring, HTTP `/api/set` follow-distance write, no `/api/nav`.
