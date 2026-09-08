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
- **Engage seed:** on `pedalLongActive` rising (second pull, not lateral-only) with Cap/Follow and a valid OSM limit, HUD MAX and `pedal_speed_kph` initialize to that limit (+ offset). No valid limit → existing ego capture. Pedal write-back is **seed / sticky only** — not every Follow/Cap HUD frame (that ate stalk +/-). Stalk +/- is a 1 or 5 mph `pedal_speed` step; button events are extra. An ego jump is not a stalk.
- **Display / Off**: no control change.
- **Lookahead (Cap/Follow):** a **lower** OSM maxspeed ahead eases MAX down so you reach about the new limit as you enter that way. A **higher** limit ahead does **not** raise MAX early — Follow raises only once GPS is on the faster segment.
- **A falling MAX must decelerate** (Cap/Follow, pedal mode, no overriding lead). The stock MPC cruise column is a virtual lead ~`get_safe_obstacle_distance(v_ego)` ahead with `V_EGO_COST=0`, so a 70→45 mph drop would not bind. Planner `min()`s MPC with `map_track_decel` at the **Accel 5** comfort `a` (0.80 m/s² at Normal). Accel 1–10 does not change this brake. Tesla `get_preap_accel_limits` still clips to −1.5 m/s².
- **Lead outranks map.** Map only sets the cruise ceiling plus comfort decel. `mpc.update` is always `mpc.update(radarState, v_cruise)` after the map cap.
- Panda TX whitelist, pedal gating, and engagement FSM are unchanged.
- Default **Off** until US maps are downloaded and you pick a mode.

## US map data

The US speed-limits sqlite is **not in git** (too large; ODbL still requires attribution). After flash, download the prebuilt asset over Wi-Fi:

| | |
|---|---|
| Release tag | `osm-us-speed-limits-v1` on `jmbrunick/openpilot` |
| Asset | `speed_limits_us.sqlite.zst` |
| SHA-256 | of the **zst** (`ASSET_SHA256`), verified **before** decompress. Dest sqlite is not hashed unless `SQLITE_SHA256` is set. `--sha256 ''` skips. |
| Install path | `/data/media/0/osm/speed_limits.sqlite` |
| Staging | `/data/media/0/osm/.download/` on the dest filesystem (not `/tmp`) |
| Size | **~204 MiB zst → ~516 MiB sqlite**. Fetch needs **800 MiB** free on `/data`. |

On the comma 3X: **Settings → NAP → Map Speed Limit → Download US Maps** (offroad), or `python -m scripts.nap.fetch_osm_maps`. `mapd` reloads the sqlite every ~15s onroad — no reboot.

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

Put the **zst** SHA-256 in `selfdrive/mapd/maps_manifest.py`. For a smaller region: `python scripts/nap/download_osm_speed_limits.py --lat … --lon … --radius-km 30`.

## Settings → NAP → Map Speed Limit

All map-speed controls live in this submenu (main NAP stays uncluttered):

- **Map Speed (MAX)** (`NAPMapSpeedMode`): Off / Display / Cap / Follow
- **Map Speed Offset** (`NAPMapSpeedOffsetMph`): -5 / 0 / +5 mph
- **Lookahead** (`NAPMapSpeedLookahead`): Off / Late / Normal (default) / Early
- **Acceleration** (`NAPMapSpeedAccel`): 1–10, Follow climb only (default 5). Brake to a lower MAX is locked at 5.
- Cap/Follow require the pedal interceptor

## Anticipatory decreases and Accel

`mapd` probes 40–600 m along GPS heading and publishes `nextSpeedLimit` / `nextSpeedLimitDistance`. Policy uses **decreases only**.

| Lookahead | Comfort decel | Extra margin | Start no farther than |
|---|---|---|---|
| Off | — | — | change only after GPS is on the slower way |
| Late | 1.20 m/s² | 40 m | 250 m |
| Normal (default) | 0.80 m/s² | 80 m | 400 m |
| Early | 0.55 m/s² | 120 m | 600 m |

When the upcoming drop is inside that window, MAX follows `v = sqrt(v_next² + 2 a d)`.

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
  selfdrive/mapd/tests/test_fetch_maps.py -q
```

Policy tests include lead precedence and fetch of a tiny sqlite over HTTP.

**On comma 3X, parked, GPS lock:** Mode = Display. Confirm the LIMIT sign matches a known posted limit (OSM, not NAR).

**On-road, pedal mode:**

1. Cap, **no lead**: set MAX above the posted limit; the car must decelerate toward that MAX (Accel 5 ≈ 0.80 m/s²). Raising the stalk cannot exceed the cap.
2. Cap/Follow, **slower lead**: the car still slows for radar `leadOne`.
3. Follow, Lookahead Normal, **no lead**. On a known drop (45→35), Accel **1 and 10 must feel the same brake**. On a known rise (35→45, after GPS is on the faster way), Accel 1 climbs lazily and Accel 10 quicker. Lookahead = Off: MAX and decel start only after GPS matches the slower way.
4. Drive toward a **higher** limit: MAX must **not** rise until you are on the faster segment.
5. Follow: stalk **down** below the posted limit stays past 10 seconds until another stalk or a posted-limit change (then resume at the new limit). Stalk **up** above the limit: 10s hold, then Follow. Double-pull engage with a valid limit: MAX **and** the car start at the posted limit immediately.
6. Cancel / brake still uses the existing engagement FSM.

**No-pedal:** LIMIT sign only; stock CC set speed is unchanged.

## Build notes

- Python-only `selfdrive/mapd`. Device needs `zstandard` (already an openpilot dep) to decompress the Release asset.
- Cereal: `liveMapDataNAP` on custom reserved 0. Rebuild so `params_keys.h` picks up the new params.
- `mapd` is an onroad managed process in `system/manager/process_config.py`.
