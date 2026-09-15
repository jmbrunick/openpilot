# Pre-AP Architecture

High-level design of NAP's Pre-AP Model S port. For safety-layer details see [safety-model.md](safety-model.md); for the engagement flow see [engagement.md](engagement.md).

## What makes pre-AP different

2012–2014 Model S cars have no Autopilot ECU, no harness relay, and a CAN layout distinct from HW1+ Teslas. Standard openpilot assumes a relay that routes CAN between stock AP and the panda — pre-AP has neither. The port is a spiritual successor to Tinkla (`boggyver/openpilot`, `tesla_unity_betaC3`).

## Layout

```
opendbc_repo/opendbc/
├── car/tesla/preap/          # Python car port
│   ├── interface.py          # CarParams + safety flags
│   ├── carstate.py           # CAN → CarState translation
│   ├── carcontroller.py      # CarControl → CAN (steering + pedal + stalk spoof)
│   ├── engagement.py         # stalk FSM (double-pull, brake, cancel)
│   ├── pedal_feedback.py     # Comma Pedal interceptor parser + health tracking
│   ├── constants.py          # accel profiles, pedal PID tunings
│   └── nap_conf.py           # runtime config (Params-backed)
├── safety/modes/tesla_preap.h  # standalone panda safety mode
└── dbc/tesla_preap.dbc         # CAN message definitions
```

## Major subsystems

### Standalone safety mode (`tesla_preap.h`)

Independent from `tesla_legacy.h` and upstream tesla safety. Its own hooks, counter/checksum, init, and RX/TX/fwd paths. Registered as `SAFETY_TESLA_PREAP`.

Key rationale for each nonstandard setting is commented in the file header. In short:

- `check_relay=false` + `disable_static_blocking=true` on every TX: pre-AP has no harness relay
- `ignore_checksum=true` + `ignore_counter=true` on RX: the pre-AP EPAS checksum algorithm is not fully verified across firmware versions; mismatched validation caused a silent 21-second steering dropout during testing

All actual safety checks are active (steering angle/rate limits, hands-on disengage except during a blinker-latched driver turn, EPAS error codes, door/gear, stalk echo-filtered cancel, AEB block). A tesla_preap.h change requires a panda flash after pull.

### Radar (optional)

Bosch radar with GTW (GateWay ECU) emulation. The panda forwards chassis-bus traffic with a radar-position rewrite when `RADAR_BEHIND_NOSECONE` is set. The emulation hook is `rx_all` (not `rx`) because it must see every CAN frame, not just whitelisted ones.

`tesla_radar_bosch_generated.dbc` and `tesla_radar_continental_generated.dbc` are produced by device scons from `opendbc_repo/opendbc/dbc/generator/` and are not in git. See [contributing.md](contributing.md).

### Comma Pedal (optional)

Pedal interceptor on bus 0 or bus 2 (wiring varies). Interface flag `ENABLE_PEDAL` gates both TX whitelist (`0x551` GAS_COMMAND) and RX route (`0x552` GAS_SENSOR). Without the pedal, pre-AP uses stock Tesla CC via stalk-spoof engage/cancel — see [engagement.md](engagement.md).

### EPAS firmware ownership

The NAP settings panel exposes EPAS firmware switching. Steering only engages when the EPAS variant is known-good for our compute_checksum. See the on-device settings for supported firmware.

## Vehicle model

Physical parameters (mass, wheelbase, steer ratio) come from `CarSpecs` in `opendbc_repo/opendbc/car/tesla/values.py`. Pre-AP is the same platform as HW1/HW2/HW3 Model S; we inherit the HW3 values. Do **not** override in `preap/interface.py` — the framework adds `STD_CARGO_KG` automatically, and overriding double-counts it.

Panda steering params (`PREAP_STEERING_PARAMS` in `tesla_preap.h`) must match the Python-layer `VehicleModel(TESLA_MODEL_S_PREAP)`. Verify:

```python
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor
VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_PREAP"))
print(calc_slip_factor(VM))  # must match slip_factor in tesla_preap.h
```

Any divergence causes the steering angle command limiter to disagree between layers — silent control ceiling.

## Upstream relationship

NAP tracks upstream openpilot closely. We fork `opendbc` because the pre-AP DBCs, car port, and safety mode all live there and are NAP-only. We track upstream `panda` directly — pre-AP safety changes go through the `opendbc_repo/opendbc/safety/` path that panda pulls in as a submodule.

See [Tinkla](https://github.com/boggyver/openpilot/tree/tesla_unity_betaC3) and [xnor-tech/openpilot](https://github.com/xnor-tech/openpilot) for the prior art NAP builds on.

## Turn stalk vs automatic lane change

Pre-AP `TurnIndLvr_Stat` is only IDLE / LEFT / RIGHT / SNA — no tip vs latch bit. NAP classifies from hold time (`selfdrive/controls/lib/stalk_tip_turn.py`):

- **Tip:** LEFT or RIGHT then IDLE within `STALK_TIP_HOLD_S` (0.40s) → arm ALC at any speed (wheel nudge still starts the change; opposite tap cancels; stalk IDLE alone does not).
- **Held turn:** LEFT or RIGHT past 0.40s from idle → do not arm ALC; latch a driver turn (flash-latch ~1s dark). Soft-lat **Off** still pauses lat (hand release resumes). Soft-lat **On** does not clear `latActive` on lamp latch; the driver may still soft-yield, and take-back is inhibited until the latch ends (then yield + normal 0.15 s confirm + 1 s blend). A driver turn does **not** drop longitudinal — `enableLongControl` stays true and lead/map braking and accel apply through the turn. Brake still uses the silent long pause. A short tip keeps interceptor ENABLE for a ~2.5 s comfort-shaped regen ramp (gentle first, full interceptor regen later); held / hard brake RELEASEs immediately to stock regen.
- **During ALC / leftover keep-alive:** same-direction physical stalk held past `STALK_ALC_TURN_HOLD_S` (1.0s) cancels ALC, stops OP blinker keep-alive, and latches a driver turn (Soft-lat Off pauses lat; Soft-lat On inhibits re-enable). A shorter same-direction hold does not force a turn. Opposite brief tap still cancels ALC without becoming a turn; opposite held uses the 0.40s window.

`LANE_CHANGE_SPEED_MIN` (20 mph) still gates *starting* the lane-change maneuver after a nudge, not tip vs turn and not pause vs ALC. Hazards do neither. ALC keep-alive lamps do not pause lat.

## Driver-wheel temporary lateral handoff

**Default On** (`NAPDriverLatHandoff=1`). Yield on **driver intent** (sustained torsion + aligned steer rate + hands on the rim), not gravel/wind/crown. Stay yielded while `EPAS_handsOnLevel >= 1`; 1 s blend starts after ~0.15 s of hands truly off. Emergency/hard brake (`aEgo <= −3.5 m/s²` for 80 ms + digital Applied) during yield / 2 s window fully cancels OP. Light brake is still the silent long pause. A driver-turn blinker does not strip lat when soft-lat is On; after a yielded turn, resume is yield + 0.15 s confirm + 1 s blend (no dedicated blinker blend). Turn Off in Settings → NAP → Driving Mannerisms if false-yields remain — Off also restores stock blinker lat-pause. See [engagement.md](engagement.md#driver-wheel-temporary-lateral-handoff).

## Driver monitoring (simulate looking)

Stock DM timers stay **3 / 5 / 11 s**. Triple-tap **NAP** (1.0 s window; not under Driving Mannerisms) has three hidden toggles: **Force Offroad**, **Simulate Look**, **False Alert Ignore**. On nap-dev, Simulate Look defaults **On** and False Alert Ignore defaults **Off** — they are **mutually exclusive** (both Off is allowed; stale both-On → Simulate Look On). Shared cadence: drain **past 1.0 s**, then fire uniform in **(1.0 s, 3.0 s]**. `NAPDmSimulateLooking` is the pre-FAI **full looking-path wipe** (no-face / uncertain / phone / pose / eye) so common false nags recover on cadence. `NAPDmFalseAlertIgnore` (Sim Look Off) soft-clears **phone/device** false positives (`phoneProb` / `distracted_types['phone']`) only, and only when pose and eye are not alarming. Either toggle Off = that path is stock. Hard cancels (hands-on ≥ 2, stalk, door, reverse) are unchanged. See [engagement.md](engagement.md#driver-monitoring-simulate-looking).

## Light/Rain sensor (Auto wipers)

Justin’s 2014 Pre-AP stalk is **Off/Int/On (no Auto)**, so stock never rain-arms the wipers. Ambient light is expected on chassis `BODY_R1` (`0x283` `LgtSens_*`) with the stalk Off. NAP Int/On/Auto-wet spoof `0x45` **TIPWIPE** (`0x10`), which is **not** Tesla rain-arm `WprSw6Posn` INTERVAL1/2 (`byte6 & 0x07` = 1/2). No DBC rain-intensity field; no VIN rain flag. Keep #159 camera Auto unless a parked INTERVAL1 hold + spray proves BCM rain-wipes. Details: [preap-rain-sensor.md](preap-rain-sensor.md).

## OSM map speed (MAX)

`selfdrive/mapd` looks up OpenStreetMap `maxspeed` from an offline sqlite and publishes `liveMapDataNAP`. In pre-AP **pedal** mode, `card.py` overlays that limit onto software cruise (`CS.vCruise` / HUD **MAX**): Cap never raises, Follow tracks the sign, engage seeds to the posted limit, and a Follow stalk set (above or below) holds until the posted limit changes. A sharp curve snapshots that MAX and restores it after the bend so temporary corner slowing / OSM flicker cannot permanently rebase sticky (Hypermile eco must not bounce a pre-curve 60 down to the live posted-scaled target). HUD current speed is wheel/ESP `vEgo` (not `vEgoCluster` / MAX). When ego is above MAX the planner commands Accel-5 comfort decel (`map_track_decel`); when ego is below MAX and there is **no** radar lead it commands Accel 1–10 climb (`map_track_accel`). A valid lead owns follow — map climb must not replace non-negative MPC `a` (or add Hill Climb `+g·sin`) toward MAX through that lead. `map_track_decel` above MAX still mins in with a lead. Hypermile **Hill Climb** (default On while Hypermile is On; IMU pitch only — **no maps-elevation lookahead**) adds `g·sin(pitch)` to that climb on a real uphill **under** MAX so Accel 1 does not sag, and applies a light crest/downhill ease **only at or above MAX** (not `TRACK_TAPER`); deadband leaves hold 0 (no `+g·sin` past MAX). It never raises MAX and lead/MPC brake still win. Coming up behind a radar lead caps **positive** close-the-gap accel (`lead_close_accel_ms2`, Accel 1 → 0.20 m/s²) so catch-up is not the cruise 1.6–0.6 punch; MPC danger / hard brake is unchanged and it still closes onto Follow Distance. `LongitudinalMpc.update(radarState, v_cruise)` still takes `min(lead, cruise)`. No-pedal stock CC is display-only. US maps are a GitHub Release download; Refresh maps overlays live OSM within 100 miles. Details: [map-speed.md](map-speed.md).

Behind a radar lead, a Pre-AP stalk **tip** (first detent, 1 mph) undoes that frame’s MAX and writes stock Follow Distance 1–7 (`NAPFollowDistance`) when the lever returns to IDLE; a **full press** (2nd detent, 5 mph) still steps MAX +5/−5 and does not remap Follow — even if the press passed through first detent. Hypermile On or Off. The Driving Mannerisms slider stays visible and updates live. No lead: tip and hold both still step MAX. Hypermile does not own follow levels. See [hypermile.md](hypermile.md) and [engagement.md](engagement.md).

## Speed sign logger (log-only)

`selfdrive/speedsignd` is a separate on-road process (default **Off**, Settings → NAP → Speed Sign Logger). Stock `modelV2` has no speed-sign head, so this runs a compact YOLOv8 ONNX on the ROAD camera (+ GNSS for JSONL) and publishes `liveSpeedSignNAP` for a display-only SIGN plate. Weights live on `/data` (not git). It does not write the OSM sqlite, does not change `vCruise`, and does not query osm.org. **No YOLO while openpilot is actively controlling** (`selfdriveState.active`; SIGN shows WAIT). Manual driving (moving OK) still detects at **1 Hz** with skip-on-overrun and nice 19; SubMaster polls at 20 Hz so cereal alive/valid cannot stuck WAIT. Not gated on park / Force Offroad. Unthrottled 4 Hz YOLO on a 3X starved `modeld` and can TAKE CONTROL. Details: [speed-sign-log.md](speed-sign-log.md).
