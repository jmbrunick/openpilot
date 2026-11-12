# Stoplight green chime (`stoplightd`)

While Scallywag sits stopped in Drive, `stoplightd` looks at the top of the
road camera frame about 4 times a second. When it sees a lit red signal it
puts **RED LIGHT** in the HUD (next to MAX). When that red goes out and a
green comes on in the same signal head, the HUD shows **GREEN - GO** for 3 s
and soundd plays one ding (`light_green.wav`, E6 to A6 bell, never quieter
than 0.7 volume). One ding per stop.

It never brakes, steers, or changes engagement. It works engaged or not.

## Toggle

`NAPStoplightChime` (default **On**). Settings → Driving Mannerisms →
"Green light chime" (3X and comma four). Off stops the process, so no
camera reads at all.

## Pipeline

| piece | file | what |
| --- | --- | --- |
| gate | `selfdrive/stoplightd/gate.py` | stopped in Drive: `livePose.velocityDevice` < 0.25 m/s for 1 s, no `wrongGear` / `reverseGear` in `onroadEvents`. Closes > 0.8 m/s or when either input goes stale. |
| detector | `selfdrive/stoplightd/detect.py` | numpy only. NV12 half-res grid, top 56 % of the frame. Red: V-128 ≥ 40 with red hue (not amber); green: 128-V ≥ 40 and U-128 ≥ -8. 4x4 cells, 8-connected blobs, lamp-sized and compact. ~5 ms per frame on x86. |
| tracker | `selfdrive/stoplightd/tracker.py` | red is confirmed after 4 hits over 0.6 s. Ding needs: a confirmed red that went out, a **new** green born within 1 s of it, seen twice over 0.15 s, below the red in the same head (or beside it for horizontal heads). Greens that were already lit, a red that just goes out (right on red), or a green somewhere else stay quiet. |
| daemon | `selfdrive/stoplightd/stoplightd.py` | connects to the road VisionIPC stream only once the gate opens. Little cores 0-3, nice 19. Publishes `stoplightNAP` at 4 Hz while gated, 1 Hz otherwise. Optional process. |
| sound | `selfdrive/ui/soundd.py`, `selfdrive/ui/stoplight_chime.py` | soundd subscribes to `stoplightNAP` and plays the ding when `chime` is set and no selfdrived alert sound is playing. No new `EventName`, so no alert ordinal drift between nap-dev and nap-alpha. |
| HUD | `selfdrive/ui/onroad/stoplight_chip.py` | chip state; drawn by both HUD renderers. |

`stoplightNAP` reuses `CustomReserved6` (Event @113). It does not read
`carState`: that service is at the msgq reader cap
(`selfdrive/test/test_msgq_reader_budget.py`).

## Calibration data

From Scallywag's own drives, Oct 23 - Nov 11 2026 (routes `000001c2` to
`000001fc`, 60 drives). 63 stops in Drive of 5 s or more were found from
qlogs, checked by eye on qcamera, and the full-res road camera was pulled
for each.

* 24 real red-to-green stops (day, overcast, rain, dusk, night, arrows):
  **24 / 24 ding**, 0.2 - 3.1 s before the car moved (one mid-stop change
  85 s before moving). Same result at half the frame rate.
* 22 stops without a change (lots, garages, stop signs, right on red, a
  digital billboard flipping red to green content): **0 dings**.
* Known: a red store sign can raise the RED LIGHT chip briefly (seen once,
  ~2 s); it did not ding.

Fixtures: `selfdrive/stoplightd/tests/samples/` (22 crops, see its README)
and `selfdrive/stoplightd/tests/sequences.json` (per-frame lamps for all 52
labelled stops).

## Limits

* Road camera only (narrow). First in line under a tall overhead mast the
  head can be above the frame; then there is no ding.
* Does not know which head is yours: it dings for a red-to-green change in
  any head it tracks in the frame.
* A red arrow that turns into a green arrow counts like a round lamp.
