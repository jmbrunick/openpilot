# NAP companion Dash

In-tree hotspot web UI for Justin's tree. Process: `nap_dash` (`selfdrive.nap_dash.server`). Starts with openpilot when `NAPDashEnabled` is On (default).

**Do not merge this PR until Justin flashes the PR branch tip and road-tests it.** First-use is flash-the-tip, not merge-to-nap-release.

## Engage safety (why #177 blocked energize)

`nap_dash` is an **optional / non-critical** manager process. A Dash crash, `:7070` bind failure, or preimport error must **not** raise `processNotRunning` (`Process Not Running` / `nap_dash`) and must **not** fail `manager` start. That alert is NO_ENTRY + SOFT_DISABLE — it prevents engage entirely and is easy to read as “some other error” (not controls mismatch).

#177 registered Dash as always-on and **required**. If the process died or never stayed running, selfdrived blocked engage. This fix keeps the page, but Dash cannot take down the stack.

No carstate / cereal / panda / pedal-interceptor changes. Dash does not write engagement Params at startup.

## What it is

Phone-friendly page on the comma device:

- Live cluster (speed, MAX, steer, lead / lane sketch, pedals)
- Driving Mannerisms + Map Speed + stock personality / Experimental Mode + SL / FAI
- Tesla BMS decode (read-only)
- Dashcam viewer + on-demand HUD export (ffmpeg; opt-in, leave the tab closed while driving)

Adapted from Philip's NAP-Dash `server_v21` **UI shell**. The settings bridge is Justin's Params / Driving Mannerisms on **nap-release**, not Philip's JSON trim path. Hypermile is nap-dev-only and is not on this page.

## What it is not

- Not Philip City Turns / Tap LC / Corner Assist / Lane Centering
- Not cruise trim / +5 mph speed-offset injector
- Not nav injection / phone nav remote
- Not Cloudflare / ngrok / Tailscale Funnel — no unauthenticated public internet
- Not Hypermile (that feature stays nap-dev-only)

## How to flash the PR tip

This is a Python + `NAPDashEnabled` Params-key change. **No panda flash.** The installer rebuilds `common` for the new key; panda firmware is unchanged.

On the comma 3X, Software → Custom Fork (or the installer URL):

```
https://installer.comma.ai/jmbrunick/openpilot/cursor/nap-dash-engage-fix-bdc4
```

Use the **fix PR branch name**, not `nap-release`, not `nap-dev`, and not the original `#177` branch (`cursor/nap-dash-dev-e946`) that blocked engage. Wait for the update to finish, then reboot so manager starts `nap_dash`.

Confirm the tip SHA on-device matches the PR head before the first road test (`Settings → Software` / `git rev-parse HEAD` over SSH).

## Dash off workaround (no reflash of nap-release)

If Dash still misbehaves, disable the process. Engage stays on nap-release behavior:

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

## CPU

`nap_dash` is a light Python HTTP server + ~10 Hz SubMaster. Dashcam **export** is on-demand only. Do not leave a high-res export running on-road.

## Tests

`selfdrive/nap_dash/tests/test_settings.py` — Params round-trip, SL/FAI exclusivity, reject Philip-only and Hypermile names.

`selfdrive/nap_dash/tests/test_server_smoke.py` — process_config wiring, HTTP `/api/set` follow-distance write, no `/api/nav`.
