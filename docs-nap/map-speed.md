# OSM Map Speed (MAX)

Live OpenStreetMap speed limits on comma 3X, wired into NAP's existing cruise **MAX** set speed (`CS.vCruise` / HUD "MAX"). This is not a second longitudinal controller.

## What "mac speed" is

The on-road HUD box labeled **MAX** is the cruise set speed (`carState.vCruiseCluster`). This feature maps OSM `maxspeed` → that value (and the long planner's `v_cruise` target) while NAP already owns software cruise.

## Data path

```
GNSS (ublox/qcom → gpsLocationExternal)
  → mapd (offline SQLite R-tree of OSM ways)
  → liveMapDataNAP.speedLimit  (m/s)
  → card.py  (pre-AP pedal software cruise only)
       CS.vCruise / vCruiseCluster   ← HUD MAX  (cruise ceiling)
  → plannerd cap_planner_v_cruise_ms (never raises)
  → LongitudinalMpc.update(radarState, v_cruise)
       constraint = min(lead0, lead1, cruise_obstacle(v_cruise))
```

Licenses: pfeiferj/mapd and sunnypilot SLA are MIT; we did **not** vendor the Go `mapd` binary or sunnypilot UI. NAP keeps a Python OSM querier (ODbL map data) on the existing `vCruise` path. The sunnypilot-specific stack stays on `naponsp-dev` per [contributing.md](contributing.md).

Map data is © OpenStreetMap contributors ([ODbL](https://www.openstreetmap.org/copyright)).

## Safety invariants

- **Pedal mode** (`openpilotLongitudinalControl`, not `pcmCruise`): Cap/Follow may change `vCruise`. Same field card.py already copies from `pedal_speed_kph`.
- **No-pedal / stock CC**: display only. We do not spoof stalk +/- to chase map limits.
- **Cap** (recommended): `MAX = min(driver set, OSM limit + offset)`. Never raises.
- **Follow**: MAX tracks the OSM limit; stalk +/- pauses follow for 10s then resumes.
- **Display / Off**: no control change.
- **Lead vehicles outrank map speed.** Map only sets the cruise target ceiling (`vCruise` / planner `v_cruise`). Radar ACC (`radarState.leadOne` / `leadTwo` in `LongitudinalMpc`) still commands **below** that ceiling when a lead is slower. Map speed does not clear, replace, or bypass lead obstacles. `mpc.update` is always `mpc.update(radarState, v_cruise)` after the map cap.
- Panda TX whitelist, pedal gating, and engagement FSM are unchanged.
- Default **Off** until US maps are downloaded and you pick a mode.

## How US map data is shipped

The US speed-limits sqlite is **not in git** (a continental extract is hundreds of MB; GitHub clones would suffer, and ODbL still requires attribution).

After flash, download a **prebuilt US-wide** asset from a GitHub Release on `jmbrunick/openpilot`:

| | |
|---|---|
| Release tag | `osm-us-speed-limits-v1` |
| Asset | `speed_limits_us.sqlite.zst` |
| SHA-256 | of the **zst** (`ASSET_SHA256`), verified on the downloaded `.zst` **before** decompress. Dest sqlite is not hashed unless `SQLITE_SHA256` is set. `--sha256 ''` skips. |
| Install path | `/data/media/0/osm/speed_limits.sqlite` |
| Staging | `/data/media/0/osm/.download/` on the **dest filesystem** (not `/tmp`) |
| Contents | OSM ways with a numeric `maxspeed` tag (US). Taginfo ~3.4M ways (2026-09). |
| Measured size | **~204 MiB zst → ~516 MiB sqlite** (`osm-us-speed-limits-v1`) |

**Free space:** fetch fails early unless the dest filesystem has **800 MiB** free (`800 * 1024 * 1024 = 838,860,800` bytes = 204 + 516 + 80 MiB margin). Peak on `/data` is the zst plus `speed_limits.sqlite.partial`; the zst is deleted as soon as decompress finishes. `/tmp` (tmpfs on device) is not used.

If download dies with `[Errno 28] No space left on device`:

```bash
rm -f /data/media/0/osm/*.partial /data/media/0/osm/.download/*
df -h /data /data/media/0/osm
# still short? delete old routes/videos under /data/media/0/
```

A previous good `speed_limits.sqlite` is left in place. After cleanup, run Download US Maps again.

### On the comma 3X (preferred)

1. Flash this branch. Connect **Wi-Fi**. Confirm `df -h /data` shows ≥ 800 MiB free.
2. **Settings → NAP → Download US Maps** (offroad). Progress prints in the script runner.
3. Or SSH: `cd /data/openpilot && python -m scripts.nap.fetch_osm_maps`

`mapd` reloads the sqlite every ~15s onroad. No reboot required after a successful download.

### Publish the Release (one-time, PC with disk)

Geofabrik `us-latest.osm.pbf` is ~11 GB. Filter to maxspeed ways, then build:

```bash
osmium tags-filter us-latest.osm.pbf w/highway w/maxspeed -o us-maxspeed.osm.pbf
python scripts/nap/build_osm_speed_limits.py --pbf us-maxspeed.osm.pbf \
  --out speed_limits_us.sqlite --zst
# attach speed_limits_us.sqlite.zst to GitHub Release tag osm-us-speed-limits-v1
```

Needs `pyosmium` (`pip install osmium`) for `--pbf`. Put the **zst asset** SHA-256 in `selfdrive/mapd/maps_manifest.py` (`ASSET_SHA256`) — fetch verifies the downloaded `.zst` before decompress.

If the device does not have 800 MiB free for the US pack, build a **smaller region** on a PC and scp it to `/data/media/0/osm/speed_limits.sqlite`:

```bash
python scripts/nap/download_osm_speed_limits.py --lat 37.7749 --lon -122.4194 --radius-km 30
```

Optional override path: param `NAPMapSpeedDbPath`.

## What you must do on the device

1. **Flash** this `nap-dev` build to the comma 3X as usual (installer / `updated` target branch). Rebuild so `params_keys.h` is compiled in.
2. **Download US maps** (Wi-Fi): Settings → NAP → Download US Maps, or `python -m scripts.nap.fetch_osm_maps`.
3. **Settings → NAP**:
   - **Map Speed (MAX)**: Off / Display / Cap / Follow
   - **Map Speed Offset**: -5 / 0 / +5 mph (added to the OSM limit)
   - Pedal interceptor must be on for Cap/Follow to change set speed
4. `mapd` starts onroad. With GPS fix and a matching way, a **LIMIT** sign appears next to MAX. In Cap/Follow, MAX itself updates as you cross OSM speed changes.

## How to test

**Off-device (this repo):**

```bash
pytest selfdrive/mapd/tests/test_map_speed_policy.py \
  selfdrive/mapd/tests/test_osm_db.py \
  selfdrive/mapd/tests/test_fetch_maps.py -q
```

Policy tests include lead precedence: no lead → map-capped cruise; slower lead → lead obstacle stays tighter than the map ceiling. Fetch test serves a tiny sqlite over HTTP.

**On comma 3X, parked, GPS lock:**

1. Mode = Display. Confirm LIMIT sign matches a known posted limit near you (OSM, not NAR).
2. `cd /data/openpilot && python -c "from cereal.messaging import SubMaster; sm=SubMaster(['liveMapDataNAP']);
import time
for _ in range(6):
  sm.update(1000); d=sm['liveMapDataNAP']; print(sm.valid['liveMapDataNAP'], d.speedLimit, d.roadName, d.dbLoaded)"`

**On-road, pedal mode:**

1. Cap, **no lead**: set MAX above the posted limit; MAX should drop to OSM (+ offset); raising the stalk cannot exceed the cap.
2. Cap/Follow, **slower lead**: MAX / map ceiling may be 65 while the car still slows to follow radar `leadOne`. Map must not prevent that slowing.
3. Drive across a limit change (e.g. 45 → 35). MAX should update within ~1s of a good match.
4. Follow: MAX should rise and fall with OSM; one stalk tap holds your speed for ~10s.
5. Cancel / brake still uses the existing engagement FSM. Hands-on / panda limits unchanged.

**No-pedal:** LIMIT sign only; stock CC set speed is unchanged.

## Build notes

- Python-only `selfdrive/mapd`. No extra native deps (stdlib `sqlite3` R-tree). Device needs `zstandard` (already an openpilot dep) to decompress the Release asset.
- Cereal: `liveMapDataNAP` on custom reserved 0. Rebuild so `params_keys.h` picks up the new params (`scons` / device compile).
- `mapd` is an onroad managed process in `system/manager/process_config.py`.
