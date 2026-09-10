# Speed Sign Logger (log-only)

On-drive MUTCD numeric speed-sign logger for comma 3X. **Default off.** Log-only: it does not write sqlite, does not change cruise / HUD **MAX** (`vCruise`), and does not query osm.org.

Stock `modelV2` has no `speedSign` head. This is a separate process (`speedsignd`) that reads the ROAD camera + GNSS and runs a compact **YOLOv8 ONNX** (US traffic-sign classes, including 55/60). The old numpy template matcher is tests/dev only — it does not see real roadside signs.

## Enable

1. **Install weights** (once, Wi-Fi) — Settings → NAP → **Install weights** (offroad / Force Offroad), or SSH below. Without `/data/media/0/nap/speed_sign.onnx` the onroad SIGN plate shows **NO WT** and will not light mph on real signs.
2. **Settings → NAP → Speed Sign Logger** → On  
   (comma 4 / mici: **Settings → NAP → speed sign logger**)
3. Or: `Params().put_bool("NAPSpeedSignLog", True)` / `echo -n 1 > /data/params/d/NAPSpeedSignLog`
4. Go onroad. Manager starts `speedsignd` only when the param is true **and** the car is onroad.

Off (default): the process does not run.

Reset to Defaults turns the logger back off. Weights on `/data` stay.

## Install weights on the 3X

No large weights in git (~43 MB ONNX). Same idea as the OSM map pack: fetch onto `/data` with a SHA-256 check.

On the 3X (parked / Force Offroad, Wi-Fi): **Settings → NAP → Install weights** → Start. Settings shows **Speed Sign Weights: Installed** or **Missing**. Progress and errors stay on that runner screen (does not block the rest of the UI, does not reboot).

Or SSH:

```bash
python -m scripts.nap.install_speed_sign_weights
```

That downloads GitHub Release `speed-sign-onnx-v1` / `speed_sign.onnx` and writes:

| | |
|---|---|
| Install path | `/data/media/0/nap/speed_sign.onnx` |
| Override | env `NAP_SPEED_SIGN_ONNX` or `--out` |
| SHA-256 | `weights_manifest.ASSET_SHA256` (checked on download) |

If the Release asset is not up yet, export once on a PC (needs `pip install ultralytics onnx`), then copy:

```bash
# PC
python -m scripts.nap.export_speed_sign_onnx --out ./speed_sign.onnx
scp ./speed_sign.onnx comma@<device>:/tmp/speed_sign.onnx

# 3X
python -m scripts.nap.install_speed_sign_weights /tmp/speed_sign.onnx
```

Or pass `--url` at a local `file://` / HTTP path. `--sha256 ''` skips the digest (local experiments only).

Source checkpoint (MIT): [cvtechniques/JC-Traffic-Sign-Detection](https://huggingface.co/cvtechniques/JC-Traffic-Sign-Detection) YOLOv8s, trained on LISA + US Roboflow traffic-sign photos (classes `speedLimit15`…`speedLimit85`, plus stop/yield/etc. which we ignore). Export is 320×320 RGB, output `[1, 25, 2100]`. Runtime: `onnxruntime` if present, else tinygrad `OnnxRunner` (stock 3X).

## What Justin should see

After weights are installed and the logger is **On**, drive past a **clear, unobstructed MUTCD R2-1** (white SPEED LIMIT plate) in daylight — e.g. a roadside **55** or **60**.

- Within about **0.5 s** (two frames at 4 Hz) a large opaque **SIGN** plate appears with that mph.
- It holds **1.5 s** after the last confirmed detection, then hides.
- A JSONL row is appended only with a live GNSS fix (same mph near the last write is skipped ~8 s / ~40 m).
- HUD **MAX** / cruise / OSM LIMIT do not change.

If the plate never appears, see **SIGN never lights** below.

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

A row is written only with a live GNSS fix. The on-road HUD still updates from every **confirmed** detection (cereal `liveSpeedSignNAP`), including without GPS.

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
| Confirm | **2** detections of the same mph within **0.75 s**, conf ≥ **0.40** |
| Hold | **1.5 s** after the last confirmed detection, then it hides |
| Source | cereal `liveSpeedSignNAP` (not the JSONL file) |

White MUTCD-style plate, black digits, red border, **SIGN** label so it is not confused with HUD MAX or OSM LIMIT.

If the logger is On and ONNX failed to load, the same plate shows **NO WT** (not a mph). After weights are installed, a blank plate means no confirmed detection — that is normal.

## SIGN never lights

Speed Sign Logger On, onroad, but SIGN stays dark or never shows mph — almost always missing weights, not a bad model.

1. Look at the onroad plate: **NO WT** means `/data/media/0/nap/speed_sign.onnx` is absent or failed to load. Settings → NAP → Speed Sign Weights should say **Missing**.
2. Park or turn on Force Offroad. Tap **Install weights** (Wi-Fi). Wait for the runner to finish — do not leave it spinning forever; an error prints on that screen. Or SSH: `python -m scripts.nap.install_speed_sign_weights`.
3. File should be ~43 MB. `ls -l /data/media/0/nap/speed_sign.onnx`. Settings should flip to **Installed**.
4. No full reboot required: speedsignd retries ONNX every ~15 s. `swaglog` should show `speedsignd starting … backend=yolo-onnx` or `ONNX loaded after retry backend=yolo-onnx`.
5. Drive past a clear, unobstructed MUTCD R2-1. Confirmed mph lights SIGN. A blank plate with weights Installed means no detection yet (night, glare, tiny sign — see Accuracy limits).

Do not treat a blank plate as “the detector is running.” Blank + Installed = no confirmed sign. Blank + Missing / **NO WT** = install weights.

## Accuracy limits (honest)

This is a small CPU detector at 4 Hz on a 320² letterbox of the ROAD camera. It is **not** a modeld head and is **not** used for control.

**Usually works:** daylight, dry, a standard white R2-1 facing the car, large enough in the ROAD frame (near / mid roadside, not a speck on the horizon). 55 and 60 are in the trained class set.

**Often misses or misreads:**

- Night, rain, snow, heavy shadow, glare / washout, dirty lens
- Temporary orange / work-zone plaques, electronic VMS, banner-style overlays
- Metric (km/h) plates, yellow advisory plaques, school-zone assemblies when the number is secondary
- Strong perspective (sign almost edge-on), motion blur at high speed, occlusion (trees, trucks)
- Far freeway signs that occupy only a few pixels after the 320² letterbox
- 25 vs 35 vs 55 confusion when the glyph is tiny or partly clipped (similar white plates)
- Non-MUTCD artwork, stickers, or a sign not in the LISA/Roboflow mix

Empty road / sky is quiet at the 0.40 threshold (the model’s background scores are ~0). Debounce exists to kill single-frame flashes, not to invent detections in the dark.

Bring-your-own ONNX is still accepted if it matches the YOLO layout above or the older `[N,6] = x,y,w,h,mph,conf` custom layout.

## What this does not do

- No sqlite writes (OSM map speed is unchanged)
- No cruise / `vCruise` / HUD MAX overlay
- No osm.org / Overpass
- No modeld / `modelV2` head
- The SIGN plate is display-only (same process; not a second controller)
