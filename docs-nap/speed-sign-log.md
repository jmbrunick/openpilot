# Speed Sign Logger (log-only)

On-drive MUTCD numeric speed-sign logger for comma 3X. **Default off.** Log-only: it does not write sqlite, does not change cruise / HUD **MAX** (`vCruise`), and does not query osm.org.

Stock `modelV2` has no `speedSign` head. This is a separate process (`speedsignd`) that reads the ROAD camera + GNSS.

## Enable

1. **Settings → NAP → Speed Sign Logger** → On  
   (comma 4 / mici: **Settings → NAP → speed sign logger**)
2. Or: `Params().put_bool("NAPSpeedSignLog", True)` / `echo -n 1 > /data/params/d/NAPSpeedSignLog`
3. Go onroad. Manager starts `speedsignd` only when the param is true **and** the car is onroad.

Off (default): the process does not run.

Reset to Defaults turns the logger back off.

## Log path

| | |
|---|---|
| Device | `/data/media/0/nap/speed_signs.jsonl` |
| PC / tests | `~/.comma/media/0/nap/speed_signs.jsonl` |
| Override | env `NAP_SPEED_SIGN_LOG` |

One JSON object per line:

```json
{"t":1710000000.12,"lat":45.315,"lon":-95.601,"bearing":87.2,"mph":45,"conf":0.81}
```

| Field | Meaning |
|---|---|
| `t` | Unix time (seconds) |
| `lat`, `lon` | GNSS fix used for the row |
| `bearing` | GNSS bearing (deg), or `null` |
| `mph` | MUTCD posted integer (5–85 step 5, or 100) |
| `conf` | Detector confidence 0–1 |

A row is written only with a live GNSS fix. The same mph near the last write is skipped for ~8 s / ~40 m so 4 Hz does not spam the file. The on-road HUD still updates from every detection (cereal `liveSpeedSignNAP`), including without GPS.

Pull the log off the device:

```bash
scp comma@<device>:/data/media/0/nap/speed_signs.jsonl .
```

## On-road HUD

When the logger is **On**, a large opaque **SIGN** plate shows the mph the camera is reading. Display-only — it does not change MAX or cruise.

| | |
|---|---|
| Where (3X) | Top-right, left of the experimental button (opposite MAX / OSM LIMIT) |
| Where (comma 4) | Top-right |
| Hold | **1.5 s** after the last detection above threshold, then it hides |
| Source | cereal `liveSpeedSignNAP` (not the JSONL file) |

White MUTCD-style plate, black digits, red border, **SIGN** label so it is not confused with HUD MAX or OSM LIMIT.

## Weights (optional)

No large weights in git. The built-in numpy MUTCD R2-1 detector runs with **no extra files**.

A compact ONNX (a few MB) can replace that backend if present:

| | |
|---|---|
| Install path | `/data/media/0/nap/speed_sign.onnx` |
| Override | env `NAP_SPEED_SIGN_ONNX` or `--out` |

On the 3X (SSH or after `scp` of the `.onnx`):

```bash
python -m scripts.nap.install_speed_sign_weights /path/to/speed_sign.onnx
# same as:
mkdir -p /data/media/0/nap
cp /path/to/speed_sign.onnx /data/media/0/nap/speed_sign.onnx
```

Expected ONNX I/O (if you bring your own model):

- Input `image`: float32 `[1,1,H,W]` (Y 0–1) or `[1,3,H,W]` (RGB 0–1)
- Output `dets`: float32 `[N,6]` = `x, y, w, h, mph, conf` in input pixels  
  A `[1,2]` `(mph, conf)` output is also accepted.

Requires `onnxruntime` on the device. If the file is missing or fails to load, speedsignd keeps the built-in detector.

## What this does not do

- No sqlite writes (OSM map speed is unchanged)
- No cruise / `vCruise` / HUD MAX overlay
- No osm.org / Overpass
- No modeld / `modelV2` head
- The SIGN plate is display-only (same process; not a second controller)
