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
       CS.vCruise / vCruiseCluster   ← HUD MAX
  → plannerd (cap safety net, never raises)
```

Licenses: pfeiferj/mapd and sunnypilot SLA are MIT; we did **not** vendor the Go `mapd` binary or sunnypilot UI. NAP keeps a Python OSM querier (ODbL map data) on the existing `vCruise` path. The sunnypilot-specific stack stays on `naponsp-dev` per [contributing.md](contributing.md).

## Safety invariants

- **Pedal mode** (`openpilotLongitudinalControl`, not `pcmCruise`): Cap/Follow may change `vCruise`. Same field card.py already copies from `pedal_speed_kph`.
- **No-pedal / stock CC**: display only. We do not spoof stalk +/- to chase map limits.
- **Cap** (recommended): `MAX = min(driver set, OSM limit + offset)`. Never raises.
- **Follow**: MAX tracks the OSM limit; stalk +/- pauses follow for 10s then resumes.
- **Display / Off**: no control change.
- Panda TX whitelist, pedal gating, and engagement FSM are unchanged.
- Default **Off** until you download a region DB and pick a mode.

## What you must do on the device

1. **Flash** this `nap-dev` build to the comma 3X as usual (installer / `updated` target branch).
2. **Build a region DB** (do this on a PC; Overpass of a whole US state can time out):

```bash
python scripts/nap/download_osm_speed_limits.py \
  --lat 37.7749 --lon -122.4194 --radius-km 30 \
  --out speed_limits.sqlite
```

Or `--bbox south,west,north,east`. Data is © OpenStreetMap contributors (ODbL).

3. **Copy the DB** onto the 3X:

```bash
ssh comma@<device> 'mkdir -p /data/media/0/osm'
scp speed_limits.sqlite comma@<device>:/data/media/0/osm/speed_limits.sqlite
```

Optional override path: param `NAPMapSpeedDbPath`.

4. **Settings → NAP**:
   - **Map Speed (MAX)**: Off / Display / Cap / Follow
   - **Map Speed Offset**: -5 / 0 / +5 mph (added to the OSM limit)
   - Pedal interceptor must be on for Cap/Follow to change set speed

5. Reboot after flashing. `mapd` starts onroad. With GPS fix and a matching way, a **LIMIT** sign appears next to MAX. In Cap/Follow, MAX itself updates as you cross OSM speed changes.

## How to test

**Off-device (this repo):**

```bash
pytest selfdrive/mapd/tests/test_map_speed_policy.py selfdrive/mapd/tests/test_osm_db.py -q
```

**On comma 3X, parked, GPS lock:**

1. Mode = Display. Confirm LIMIT sign matches a known posted limit near you (OSM, not NAR).
2. `cd /data/openpilot && python -c "from cereal.messaging import SubMaster; sm=SubMaster(['liveMapDataNAP']);
import time
for _ in range(6):
  sm.update(1000); d=sm['liveMapDataNAP']; print(sm.valid['liveMapDataNAP'], d.speedLimit, d.roadName, d.dbLoaded)"`

**On-road, pedal mode:**

1. Cap: set MAX above the posted limit; MAX should drop to OSM (+ offset) when the way matches; raising the stalk cannot exceed the cap.
2. Drive across a limit change (e.g. 45 → 35). MAX should update within ~1s of a good match.
3. Follow: MAX should rise and fall with OSM; one stalk tap holds your speed for ~10s.
4. Cancel / brake still uses the existing engagement FSM. Hands-on / panda limits unchanged.

**No-pedal:** LIMIT sign only; stock CC set speed is unchanged.

## Build notes

- Python-only `selfdrive/mapd`. No extra native deps (stdlib `sqlite3` R-tree).
- Cereal: `liveMapDataNAP` on custom reserved 0. Rebuild so `params_keys.h` picks up the new params (`scons` / device compile).
- `mapd` is an onroad managed process in `system/manager/process_config.py`.
