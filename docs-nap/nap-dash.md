# Comma web UI

The comma 3X serves its own settings page. Two devices: the comma and the car (the Tesla MCU browser, or a phone on the same network). There is no phone-hub app and no video or audio stream.

Process: `nap_dash` (`selfdrive.nap_dash.server`). It starts with openpilot when `NAPDashEnabled` is On (default).

**Do not merge** this branch into `nap-dev` until Justin flashes it and confirms on-road engage is clean. No panda flash. The installer rebuilds `common` only if a Params key changed; this branch does not add one.

## URL

```
http://100.99.9.1/
```

That is port 80 on the hotspot. The server binds `0.0.0.0`, so the same page is also served on home Wi-Fi at `http://<comma-lan-ip>/`. If port 80 cannot be bound, it falls back to `http://100.99.9.1:7070/` and keeps running. Override with `NAP_DASH_HOST` / `NAP_DASH_PORT`.

This is LAN / hotspot only. Do not expose it with Funnel, ngrok, or Cloudflare.

## Hotspot subnet

The comma hotspot (Settings → Network → Enable Tethering) uses **100.99.9.0/24**, comma at **100.99.9.1**.

Pre-AP Tesla browsers would load local pages on `100.99.9.x` and not on the default `192.168.43.0/24` or `10.42.0.0/24` ranges. `TETHERING_IP_ADDRESS` in `system/ui/lib/wifi_manager.py` is what NetworkManager stores on the hotspot profile (`method=shared`, prefix 24, `never-default`). That profile is on disk under NetworkManager's system connections, so it survives reboot.

On every UI start the comma rewrites the hotspot profile if it still has an old address. Joining a normal Wi-Fi network is a different profile (`method=auto`) and is not touched. `shared` still NATs clients through the comma's default route, so LTE internet sharing keeps working. A phone or the MCU that uses the hotspot for internet is using the comma's LTE data.

## What the page is

Tabs follow the on-device settings sidebar (Device, Network, Toggles, Software, NAP, Firehose, Developer). NAP subpages (Driving Mannerisms, Map Speed Limit, Radar, and the triple-tap Hidden card) are discovered from those screens.

The page is laid out for the Pre-AP Model S 17" portrait MCU with the map on top and the browser on the bottom: a wide, short pane. Compact top tabs use a blue highlight on the selected tab. Each setting is a dark row with the control on the right, in the same order and sections as the comma screen. The tab bar stays put and the open panel scrolls inside itself, so the bottom of the pane is not clipped. A phone gets the same rows in one column.

Toggles, choices, titles, and descriptions are parsed from the device UI source at startup (`selfdrive/ui/layouts/settings/*.py` and the constants those files import). There is no second hand-written toggle list. A new toggle added in that UI source shows up here after restart, with no web-side edit.

Writes go through Params the same way the device does: plain `put` / `put_bool`, and the same helpers for Experimental Mode, Simulate Look / False Alert Ignore, Force Offroad, and Hypermile. Toggles the device locks while the car is on stay locked while `IsOnroad` is set. Toggles it locks while engaged stay locked while `IsEngaged` is set. Those two Params are written by `hardwared`; this process does not subscribe to cereal to read them. Dangerous toggles (Experimental Mode, alpha longitudinal, reboot, power off) ask for confirmation.

Software / branch switching stays: `GET/POST /api/software` (`set_branch`, `fetch`, `download`, `install`). The Software tab has one button that steps Check → Download → Install & reboot.

## What it is not

- Not a live cluster, BMS view, or camera stream
- Not engage. There is no engage control. `POST /api/set` with action `engage` is rejected.
- No high-rate sockets. No `SubMaster`, no `can`, no `modelV2`.
- Not nav injection
- Script actions (EPAS flash, pedal calibrate, map download, uninstall, reset calibration) stay on the comma screen

Speed-limit sign reading, map speed, and speed-limit control are not modified by this branch. If those panels define a toggle, the page shows it only because it was parsed from that source, and the write is the same Params write the device already does.

## No high-rate sockets

`selfdrive/nap_dash/server.py` does not construct a cereal subscriber. There is no live loop. `/api/state` is an empty stub so an old page does not 500.

`nap_dash` is an **optional / non-critical** manager process. A crash, a port bind failure, or a preimport error must **not** raise `processNotRunning` (`Process Not Running` / `nap_dash`) and must **not** fail `manager` start. That alert is NO_ENTRY + SOFT_DISABLE — it prevents engage entirely.

## Setup

1. Flash:

```
https://installer.comma.ai/jmbrunick/openpilot/cursor/mcu-web-ui-c588
```

2. Reboot so `nap_dash` starts and the UI rewrites the hotspot profile.
3. On the comma: Settings → Network → Enable Tethering. Note the network name (`weedle-` plus the first 4 characters of the dongle id) and the password (default `swagswagcomma` until you change it).
4. On the Tesla MCU, join that Wi-Fi network.
5. Open `http://100.99.9.1/` and bookmark it.
6. A phone on the same hotspot, or any browser on the comma's home Wi-Fi, can open the same page.

## Dash off

SSH:

```
python3 -c "from openpilot.common.params import Params; Params().put_bool('NAPDashEnabled', False)"
```

Then reboot. Or set `BLOCK=nap_dash` in the launch environment. Re-enable with `NAPDashEnabled=True` and a reboot. Default is On.

## Tests

`selfdrive/nap_dash/tests/test_discover.py` — parsing the device UI, including a new toggle added only in that source.

`selfdrive/nap_dash/tests/test_settings.py` — Params writes, onroad / engaged locks, confirmation, no engage.

`selfdrive/nap_dash/tests/test_server_smoke.py` — process wiring, HTTP, no cereal subscriber.

`selfdrive/nap_dash/tests/test_system_api.py` — `/api/software` branch switch.

`system/ui/lib/tests/test_tethering_subnet.py` — hotspot address `100.99.9.1/24`.

## Not verified on hardware

The Tesla MCU browser, NetworkManager DHCP on the new subnet, LTE NAT, and an on-road engage check need the comma and the car. This tree was not flashed.
