# OSM Map Speed (MAX)

Live OpenStreetMap speed limits on comma 3X, wired into NAP's existing cruise **MAX** (`CS.vCruise` / HUD "MAX"). This is not a second longitudinal controller.

## What MAX is

The on-road HUD box labeled **MAX** is the cruise set speed (`carState.vCruiseCluster`). This feature maps OSM `maxspeed` onto that value (and the long planner's `v_cruise`) while NAP already owns software cruise.

```
GNSS → mapd (offline OSM sqlite) → liveMapDataNAP
  → card.py (pre-AP pedal software cruise only)
       CS.vCruise / vCruiseCluster   ← HUD MAX
  → plannerd cap + map_track_decel (comfort a; lead can still brake more)
  → LongitudinalMpc.update(radarState, v_cruise)
       constraint = min(lead0, lead1, cruise_obstacle(v_cruise))
```

Licenses: pfeiferj/mapd and sunnypilot SLA are MIT; we did **not** vendor the Go `mapd` binary or sunnypilot UI. NAP keeps a Python OSM querier (ODbL map data) on the existing `vCruise` path. The sunnypilot-specific stack stays on `naponsp-dev` per [contributing.md](contributing.md). Map data is © OpenStreetMap contributors ([ODbL](https://www.openstreetmap.org/copyright)).

## Safety invariants

- **Pedal mode** (`openpilotLongitudinalControl`, not `pcmCruise`): Cap/Follow may change `vCruise`.
- **No-pedal / stock CC**: display only. We do not spoof stalk +/- to chase map limits.
- **Cap**: `MAX = min(driver set, OSM limit + offset)`. Never raises.
- **Follow** (preferred): MAX tracks the OSM limit. A manual set **below** the posted limit is **sticky** (no 10s timeout) until another stalk or the posted value changes. The Follow 10s timer applies only to a set **above** the limit, and must never clear a below-limit hold.
- **Engage seed:** on `pedalLongActive` rising (second pull, not lateral-only) with Cap/Follow and a valid OSM limit, HUD MAX and `pedal_speed_kph` initialize to that limit (+ offset). No valid limit → existing ego capture. Pedal write-back is **seed / sticky**, plus a **raise** when HUD MAX increased (stalk up, or Follow because the posted limit increased). Do not write the same Follow/Cap MAX every frame (that ate stalk +/-). Stalk +/- is a 1 or 5 mph `pedal_speed` step; button events are extra. An ego jump is not a stalk.
- **Display / Off**: no control change.
- **Lookahead (Cap/Follow):** a **lower** OSM maxspeed ahead eases MAX down so you reach about the new limit as you enter that way. A **higher** limit ahead does **not** raise MAX early — Follow raises only once GPS is on the faster segment.
- **A falling MAX must decelerate** (Cap/Follow, pedal mode, no overriding lead). The stock MPC cruise column is a virtual lead ~`get_safe_obstacle_distance(v_ego)` ahead with `V_EGO_COST=0`, so a 70→45 mph drop would not bind. Planner `min()`s MPC with `map_track_decel` at the **Accel 5** comfort `a` (0.80 m/s² at Normal). Accel 1–10 does not change this brake. Tesla `get_preap_accel_limits` still clips to −1.5 m/s².
- **A rising MAX must accelerate** (Follow, no overriding lead). MPC also will not climb to a higher MAX. Planner commands `map_track_accel` at Accel 1–10 when ego is below MAX and MPC is not braking. Stalk up / Follow posted raise also writes that higher MAX onto `pedal_speed`.
- **Lead outranks map.** Map only sets the cruise ceiling plus comfort decel/accel. `mpc.update` is always `mpc.update(radarState, v_cruise)` after the map cap.
- Panda TX whitelist, pedal gating, and engagement FSM are unchanged.
- Default **Off** until US maps are downloaded and you pick a mode.

## US map data

The US speed-limits sqlite is **not in git** (too large; ODbL still requires attribution). After flash, download the prebuilt asset over Wi-Fi:

| | |
|---|---|
| Release tag | `osm-us-speed-limits-v2` on `jmbrunick/openpilot` |
| Asset | `speed_limits_us.sqlite.zst` |
| SHA-256 | of the **zst** (`ASSET_SHA256`), verified **before** decompress. Dest sqlite is not hashed unless `SQLITE_SHA256` is set. `--sha256 ''` skips. |
| Install path | `/data/media/0/osm/speed_limits.sqlite` |
| Staging | `/data/media/0/osm/.download/` on the dest filesystem (not `/tmp`) |
| Size | **~204 MiB zst → ~516 MiB sqlite**. Fetch needs **800 MiB** free on `/data`. |

On the comma 3X: **Settings → NAP → Map Speed Limit → Download US Maps** (offroad), or `python -m scripts.nap.fetch_osm_maps`. That is the first-install of the published US pack. Frequent local updates: **Refresh maps** (or `python -m scripts.nap.refresh_osm_maps`) — live OSM within **100 miles** (~160.9 km), merged into the installed sqlite. `mapd` reloads the sqlite every ~15s onroad — no reboot.

If a previous download died with ENOSPC, remove leftovers then retry:

```bash
rm -f /data/media/0/osm/*.partial /data/media/0/osm/.download/*
df -h /data
```

A previous good sqlite is left in place. Optional override path: param `NAPMapSpeedDbPath`.

Publish a new US pack from a PC (Geofabrik PBF ~11 GB):

```bash
osmium tags-filter us-latest.osm.pbf w/highway w/maxspeed -o us-maxspeed.osm.pbf
python scripts/nap/build_osm_speed_limits.py --pbf us-maxspeed.osm.pbf \
  --out speed_limits_us.sqlite --zst
```

Put the **zst** SHA-256 in `selfdrive/mapd/maps_manifest.py` **and** bump `selfdrive/mapd/maps-index.json` (see [Publishing a US pack](#publishing-a-us-pack)). For a PC-only smaller region file: `python scripts/nap/download_osm_speed_limits.py --lat … --lon … --radius-km 30`.

## Refresh maps (100 miles)

**Refresh maps** is one button: a local Overpass update, not a published-pack download. Radius is **100 miles** (~160.9 km) around the vehicle — the bounding box that contains that circle.

On the 3X (offroad, Wi-Fi):

1. **Location.** Wait up to **45s** for a GNSS fix (`gpsLocationExternal`, then `gpsLocation`). Refresh accepts the last received plausible lat/lon (not 0,0), including samples older than 2.5s and accuracy up to 200 m (yard/tree cover). mapd's onroad MAX match still uses 2.5s / 50 m. Otherwise last stored GPS (`LastGPSPosition` param or `/data/params/d/LastGPSPosition`). A successful fix is persisted so the next offroad tap works. If still nothing: start openpilot onroad until the GPS icon/fix is up for about a minute, then retry. Maps will **not** guess a city.
2. **Query OSM.** Reuses Overpass / `scripts.nap.download_osm_speed_limits` / mapd builders for highway+maxspeed ways in that box. Does **not** download the full US Geofabrik PBF on the 3X.
3. **Merge.** Copy the installed `/data/media/0/osm/speed_limits.sqlite` onto `/data` (not `/tmp`), delete/replace `way_id`s whose bbox intersects the 100-mile box, insert the Overpass ways, keep the rest of the US pack. If no sqlite is installed yet, Download US Maps runs first, then the overlay — never a 100-mile-only dest file.
4. **Atomic install.** `os.replace` onto dest. Overpass timeout / HTTP / merge failure prints a real error and leaves the previous good sqlite. Retry-safe. mapd reloads ~15s onroad — no reboot. Progress (querying OSM, merging, installing) shows on the existing script-runner UI.

Overpass can be slow in a dense metro. Wait or retry; the old maps stay put.

## Publishing a US pack

Download US Maps still uses the GitHub Release + `maps-index.json`. Bump those when you republish a full US sqlite from a PC:

1. Build a new US sqlite and zst on a PC (same `build_osm_speed_limits.py` flow as above).
2. Attach `speed_limits_us.sqlite.zst` to a **new** GitHub Release on `jmbrunick/openpilot` (e.g. `osm-us-speed-limits-v3`). Do not replace the in-git JSON with the 204MB zst.
3. SHA-256 the **zst** (`sha256sum speed_limits_us.sqlite.zst`).
4. Bump `selfdrive/mapd/maps-index.json`:
   - `revision` — integer or dotted semver, must be **greater** than the previous value (current first-install is `"2"`)
   - `asset_url` — Release download URL for the new zst
   - `asset_name` — usually `speed_limits_us.sqlite.zst`
   - `sha256` — hex digest of the zst
   - `bytes` — zst size (optional)
   - `notes` — optional
5. Keep first-install constants in `selfdrive/mapd/maps_manifest.py` in sync when you intend a new clone/flash to get this pack (`RELEASE_TAG`, `ASSET_SHA256`, `RELEASE_REVISION`, sizes).
6. Merge the JSON (and manifest) to `nap-dev`. Download US Maps fetches:

   `https://raw.githubusercontent.com/jmbrunick/openpilot/nap-dev/selfdrive/mapd/maps-index.json`

## Settings → NAP → Map Speed Limit

All map-speed controls live in this submenu (main NAP stays uncluttered). TICI and mici:

- **Map Speed (MAX)** (`NAPMapSpeedMode`): Off / Display / Cap / Follow
- **Map Speed Offset** (`NAPMapSpeedOffsetMph`): -5 / 0 / +5 mph
- **Lookahead** (`NAPMapSpeedLookahead`): Off / Late / Normal (default) / Early
- **Acceleration** (`NAPMapSpeedAccel`): 1–10, Follow climb only (default 5). Brake to a lower MAX is locked at 5.
- **Map revision**: published US pack revision after Download US Maps (Refresh maps does not bump this)
- **Refresh maps**: live OSM within 100 miles, merged into the installed US sqlite (listed above Download)
- **Download US Maps**: first install of the current published pack
- Cap/Follow require the pedal interceptor

## Anticipatory decreases and Accel

`mapd` probes 40–600 m along GPS heading and publishes `nextSpeedLimit` / `nextSpeedLimitDistance`. Policy uses **decreases only**.

| Lookahead | Comfort decel | Extra margin | Start no farther than |
|---|---|---|---|
| Off | — | — | change only after GPS is on the slower way |
| Late | 1.20 m/s² | 110 m | 360 m |
| Normal (default) | 0.80 m/s² | 110 m | 600 m |
| Early | 0.55 m/s² | 230 m | 600 m |

When the upcoming drop is inside that window, MAX interpolates from the current limit at `d = kinematic + margin` to the new limit at the sign. Brake stays Accel-5 **0.80 m/s²** at Normal (not raised). The +110 m is the measured 50→30 shortfall (42 mph at the sign vs 30); every decrease starts at `kin + 110 m`, not a 50→30-only window.

**HUD current speed** (top-middle on the 3X) is wheel/ESP `vEgo` only. `vEgoCluster` is Tesla `DI_digitalSpeed`, which pre-AP also uses as `cruiseState.speed`. Map-speed writes MAX into `vCruise` / `pedal_speed` / `cruiseState.speed` (90 kph = **56 mph**). LIMIT/MAX may show the map limit; the live number must not.

**Acceleration** scales **Follow climb only**. Brake uses Accel 5 (`map_brake_a_ms2` / `map_track_decel`).

| Accel | Factor | `a` at Lookahead=Normal | Used for |
|---|---|---|---|
| 1 | 0.45 | **0.36 m/s²** | climb only (gentlest) |
| **5** | 1.00 | **0.80 m/s²** | climb *and* all map braking |
| 10 | 2.00 | **1.60 m/s²** | climb only (quickest, clamped) |

`a = clamp(0.30, 1.60, a_lookahead × factor)`. Changing Accel 1 vs 10 must not change brake feel. A higher limit ahead may still be published as `nextSpeedLimit`; Cap/Follow ignore it until `speedLimit` itself is the higher value.

## How to test

**Off-device:**

```bash
pytest selfdrive/mapd/tests/test_map_speed_policy.py \
  selfdrive/mapd/tests/test_osm_db.py \
  selfdrive/mapd/tests/test_fetch_maps.py \
  selfdrive/mapd/tests/test_local_refresh.py -q
```

Policy tests include lead precedence, fetch of a tiny sqlite over HTTP, and Refresh maps location/merge (no network).

**Offroad Refresh maps (3X, Wi-Fi):**

1. After a drive with GPS (so `LastGPSPosition` is stored), park, Settings → NAP → Map Speed Limit → **Refresh maps** → Start.
2. Script runner should show location source (GNSS or last stored GPS), querying OSM, merging, installing. Must not name a guessed city.
3. `ls -l --time-style=full-iso /data/media/0/osm/speed_limits.sqlite` — mtime updates; size stays US-pack scale (hundreds of MB), not a tiny 100-mile-only file.
4. Unplug GPS / no LastGPSPosition (fresh flash, never driven): Refresh maps must error and leave any existing sqlite.
5. Airplane mode / Overpass down: error text, previous sqlite kept. Retry later.
6. No sqlite yet: Refresh maps downloads the US pack first, then overlays. Download US Maps still does first-install only.

**On comma 3X, parked, GPS lock:** Mode = Display. Confirm the LIMIT sign matches a known posted limit (OSM, not NAR). After Refresh maps, a local OSM edit (new maxspeed) should show within ~15s onroad — no reboot.

**On-road, pedal mode:**

1. Cap, **no lead**: set MAX above the posted limit; the car must decelerate toward that MAX (Accel 5 ≈ 0.80 m/s²). Raising the stalk cannot exceed the cap.
2. Cap/Follow, **slower lead**: the car still slows for radar `leadOne`.
3. Follow, Lookahead Normal, **no lead**. On a known drop (50→30), Accel **1 and 10 must feel the same brake**, and ego should be near 30 at the sign (not still ~42). On a known rise (35→45, after GPS is on the faster way), Accel 1 climbs lazily and Accel 10 quicker — the pedal target and the car must actually speed up. Lookahead = Off: MAX and decel start only after GPS matches the slower way.
4. Drive toward a **higher** limit: MAX must **not** rise until you are on the faster segment.
5. Follow: stalk **down** below the posted limit stays past 10 seconds until another stalk or a posted-limit change (then resume at the new limit). Stalk **up** above the limit: 10s hold, then Follow. Double-pull engage with a valid limit: MAX **and** the car start at the posted limit immediately.
6. Top-middle live speed must match wheel/ESP (about 45 if that is actual), not MAX (56) and not LIMIT.
6. Cancel / brake still uses the existing engagement FSM.

**No-pedal:** LIMIT sign only; stock CC set speed is unchanged.

## Build notes

- Python-only `selfdrive/mapd`. Device needs `zstandard` (already an openpilot dep) to decompress the Release asset.
- Cereal: `liveMapDataNAP` on custom reserved 0. Rebuild so `params_keys.h` picks up the new params.
- `mapd` is an onroad managed process in `system/manager/process_config.py`.
