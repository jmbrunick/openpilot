# Speed Sign Logger (log-only)

On-drive MUTCD numeric speed-sign logger for comma 3X. **Default off.** Log-only: it does not write sqlite, does not change cruise / HUD **MAX** (`vCruise`), and does not query osm.org.

Stock `modelV2` has no `speedSign` head. This is a separate process (`speedsignd`) that reads the ROAD camera + GNSS and runs a compact **YOLOv8 ONNX** (US traffic-sign classes, including 55/60). The old numpy template matcher is tests/dev only — it does not see real roadside signs.

**Driving model first.** Logger On + ONNX weights used to run YOLOv8s at 4 Hz via tinygrad `OnnxRunner` on CPU. That saturates a 3X core and can show “driving model is lagging”, ~35% `modeld` frame drops, **TAKE CONTROL IMMEDIATELY**, or “Communication Issue Between Processes.”

**Logger On does not run YOLO while openpilot is actively controlling** (`selfdriveState.active`, or state in enabled / softDisabling / overriding). The process and SIGN / NO WT HUD still start. The plate shows **WAIT** only in that case — YOLO is skipped so `modeld` is not starved.

**WAIT while the UI looks disengaged is a bug.** Report `swaglog | grep speedsignd` lines (`enabled`, `active`, `state`, alive/valid). Do not treat a stuck WAIT as “drive parked” or “weights are bad.”

**Manual driving is the collection path** — including moving, no assist. This is **not** gated on park or Force Offroad. Drive yourself, Logger On, and log speed-limit signs + GNSS to JSONL. OSM upload is future work.

Unknown cereal is **not** forever-WAIT. After a ~2 s startup, unread `selfdriveState` allows the same 1 Hz throttled detect and logs a warning. SubMaster polls at **20 Hz** so 100 Hz `selfdriveState` alive/valid cannot false-trigger WAIT; YOLO stays ≤ **1 Hz**. After you cancel, WAIT should hide within a fraction of a second — a ~10 s hang was the old 1 Hz + `recv_frame <= 0` bug.

On re-engage, speedsignd **abandons** an in-flight ONNX (does not block the 20 Hz loop for 300–1500 ms) and will not start another infer. One leftover nice-19 infer may finish in the background; its result is dropped.

Disengaged detect is **1 Hz**, skip-on-overrun, `SCHED_OTHER` + nice 19. **If TAKE CONTROL or lag comes back, turn Speed Sign Logger Off and use nap-release.** Do not raise `modeld` priority.

## Enable

1. **Install weights** (once, Wi-Fi) — Settings → NAP → **Install weights** (offroad / Force Offroad), or SSH below. Without `/data/media/0/nap/speed_sign.onnx` the onroad SIGN plate shows **NO WT** and will not light mph on real signs.
2. **Settings → NAP → Speed Sign Logger** → On  
   (comma 4 / mici: **Settings → NAP → speed sign logger**)
3. Or: `Params().put_bool("NAPSpeedSignLog", True)` / `echo -n 1 > /data/params/d/NAPSpeedSignLog`
4. Go onroad. Manager starts `speedsignd` only when the param is true **and** the car is onroad.
5. **Drive manually** (moving is OK). SIGN shows **WAIT** only while openpilot is **actively controlling**. That is YOLO skipped on purpose. After you cancel, WAIT should drop immediately (not ~10 s later). If WAIT stays up while the UI looks disengaged, it is a bug — grab swaglog. Do not SET if you want detections.

Off (default): the process does not run. Logger On never changes that default.

Reset to Defaults turns the logger back off. Weights on `/data` stay.

Optional detect rate when **not controlling** (default 1 Hz, clamped 0.2–4): `NAP_SPEED_SIGN_HZ=0.5` in the process environment. Do not raise this on a 3X. While OP is controlling, detect is always 0 Hz, regardless of this env.

Manual detect is **cheap-path** so Logger On is less likely to starve `modeld` even when YOLO still takes ~1 s:

- ROAD **crop only** (right-biased 1208² on a 3X), downsample, then BT.601 — never a full-frame 1928×1208 RGB convert on the 20 Hz loop
- **1 ONNX / BLAS thread** (`NAP_SPEED_SIGN_THREADS`, max 2)
- pinned to **little cores 0–3** (modeld stays FIFO on core 7)
- `NAP_SPEED_SIGN_INFER_CAP_MS` default **800**: a session over the cap pays back infer time **plus one extra period** (cannot kill an in-flight tinygrad kernel)
- `os.sched_yield()` while an infer is busy

YOLOv8s-320 on tinygrad CPU is ~8.7 GFLOP plus Python `OnnxRunner` overhead. Justin measured **mean infer 1819 ms** on `cursor/speedsignd-manual-mph-b6f2` with full-frame RGB on the main thread. The crop path cuts preprocess (look for `prep=` tens of ms, `sess=` still the YOLO run). **Mean well under 500 ms is not feasible on stock 3X tinygrad with this 43 MB 320² export** — that needs `onnxruntime`, a nano re-export, or a precompiled TinyJit. This branch does not change weights or Hz.

Onroad, `swaglog` prints `speedsignd detect paused (controlling=True enabled=… active=… state=… alive=… valid=…)` when you SET, `speedsignd abandon in-flight ONNX` if a YOLO was still running, and every infer:

```
speedsignd infer 420ms backend=tinygrad frame=1928x1208 letterbox=320 crop=720,0 1208x1208 … peak=0.72/speedLimit65 n_over=1 sl_peak=0.72/speedLimit65 … refine=50:0.71(class=65) luma=90/35 chroma=1 prep=18 sess=400 raw=[(50, 0.71)] hud=[(50, 0.71)] jsonl=[]
```

`refine=` is the crop digit read vs the YOLO class. A parked close **SPEED LIMIT 50** often peaks `speedLimit65:0.73` with `cls=50:0.00` (not a close race). HUD should still be **50** when `refine=50:…(class=65)`. Empty `refine=` means no in-threshold hit (digit OCR never ran). `refine=-(65)` means the crop read failed — SIGN stays blank or holds last-good; it does not light YOLO 65. Last accepted mph holds ~10 s across 1 Hz skip-on-overrun so the plate does not blink blank between multi-second infers.

`speedsignd timing hz=… infer_ms mean=… max=… n=… skip=…` about every 15 s while disengaged. `skip` should climb when an infer overruns. If TAKE CONTROL / “driving model is lagging” / Communication Issue comes back, Logger Off + nap-release. If WAIT is stuck while you are driving manually, those same `speedsignd detect` lines explain why.

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

After weights are installed and the logger is **On**, **drive manually** (do not engage openpilot). Pass a **clear, unobstructed MUTCD R2-1** (white SPEED LIMIT plate) in daylight — e.g. a roadside **55** or **60**. Moving is the intended path. Parked / Force Offroad is not required to detect.

- SIGN shows **WAIT** only while OP is **controlling** — YOLO is off so `modeld` is not starved. Cancel / stay manual and WAIT hides. WAIT + a disengaged UI is a bug (report swaglog).
- Within about **1–4 s** of a disengaged look (two frames at 1 Hz, or after a skip) a large opaque **SIGN** plate appears with that mph on the **left** (driver) side of the onroad UI.
- **Yes** / **No** (“is this accurate?”) sit under the plate on 3X (beside it on comma 4). They are **stubs** right now — they do not change cruise, HUD MAX, or map speed. Later, Yes may confirm the marker (JSONL + optional OSM); No may discard a wrong read.
- It holds **3.0 s** after the last confirmed detection, then hides (Yes/No hide with it).
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
| Where (3X) | Left / driver side, below the MAX box. Large plate (200×248). |
| Where (comma 4) | Left / driver side (top-left), same idea |
| Yes / No | Shown only with a live mph. 3X: stacked under the plate (full-width, ~112 px tall). comma 4: beside the plate. **No-op stubs** (cloudlog debug only). |
| Confirm | **2** detections of the same mph within **4.0 s**, conf ≥ **0.40** |
| Hold | **3.0 s** after the last confirmed detection, then it hides |
| Detect rate | **0 Hz while OP is controlling**. Manual / not-active default **1 Hz** (env `NAP_SPEED_SIGN_HZ`, clamped 0.2–4), including while moving. SubMaster **20 Hz**. Overrun → skip frames until free. |
| Controlling plate | **WAIT** (quiet). No Yes/No. YOLO does not run. If the UI looks disengaged, WAIT is a bug. |
| Source | cereal `liveSpeedSignNAP` (not the JSONL file) |

White MUTCD-style plate, black digits, red border, **SIGN** label so it is not confused with HUD MAX or OSM LIMIT.

If the logger is On and ONNX failed to load, the same left-side plate shows **NO WT** (not a mph) — **no Yes/No**. If the logger is On and openpilot is **controlling**, the plate shows **WAIT** — detect is paused so the driving model is not starved; cancel / drive manually to collect. After weights are installed and OP is not controlling, a blank plate means no confirmed detection — that is normal. Stuck WAIT while the UI looks disengaged is a bug (report swaglog).

Yes/No do not write params, JSONL, or sqlite in this revision. The hook is `on_confirm_accuracy(mph, yes)` — future work may JSONL-confirm a good read and optionally push the marker to OSM.

## SIGN never lights

Speed Sign Logger On, onroad, but SIGN stays dark or never shows mph — almost always missing weights, not a bad model.

1. Look at the onroad plate: **NO WT** means `/data/media/0/nap/speed_sign.onnx` is absent or failed to load. Settings → NAP → Speed Sign Weights should say **Missing**.
2. Park or turn on Force Offroad. Tap **Install weights** (Wi-Fi). Wait for the runner to finish — do not leave it spinning forever; an error prints on that screen. Or SSH: `python -m scripts.nap.install_speed_sign_weights`.
3. File should be ~43 MB. `ls -l /data/media/0/nap/speed_sign.onnx`. Settings should flip to **Installed**.
4. No full reboot required: speedsignd retries ONNX every ~15 s. `swaglog` should show `speedsignd starting … backend=yolo-onnx` or `ONNX loaded after retry backend=yolo-onnx` (also `hz=` / `nice=19`).
5. **Stay manual** (do not engage OP). **WAIT** means detect is paused because openpilot is **controlling**. If WAIT never clears while the UI looks disengaged, it is a bug — `swaglog | grep speedsignd` should show `enabled` / `active` / `state` / alive / valid. Then pass a clear, unobstructed MUTCD R2-1. Confirmed mph lights SIGN. A blank plate with weights Installed and OP not controlling means no detection yet (night, glare, tiny sign — see Accuracy limits).

Do not treat a blank plate as “the detector is running.” Blank + Installed + manual = no confirmed sign. **WAIT** = OP controlling, YOLO off (WAIT + disengaged UI = bug). Blank + Missing / **NO WT** = install weights.

If you see **TAKE CONTROL IMMEDIATELY** or “Communication Issue Between Processes,” turn **Speed Sign Logger Off** and use **nap-release**.

## Accuracy limits (honest)

This is a small CPU detector at **1 Hz when not controlling** (was 4 Hz always) on a 320² letterbox of the **right-biased ROAD crop**. It is **not** a modeld head and is **not** used for control. It must not starve `modeld`: **no ONNX while OP is controlling**; little cores + 1 thread + skip-on-overrun + infer-cap extra skip. `swaglog` logs `speedsignd detect paused/running` with `enabled` / `active` / `state` / alive / valid on those edges and every infer `backend=` `peak=` `refine=` `luma=` `chroma=` `prep=` `sess=` `raw=` plus `speedsignd timing … infer_ms mean/max` / `skip=` about every 15 s so you can see cost on the device. On-car ROAD frames often class a clear **50** as **65**; crop digit OCR (`refine=`) overrides that pair for the HUD.

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
- Yes/No accuracy buttons are display-only stubs (no JSONL confirm, no OSM upload)
