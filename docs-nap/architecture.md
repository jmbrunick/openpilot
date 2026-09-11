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
- **Held turn:** LEFT or RIGHT past 0.40s from idle → do not arm ALC; lateral pause (flash-latch ~1s dark, then hand release resumes lat). A driver turn also drops longitudinal (`enableLongControl=False`, cruise stays). After the lamps/latch end, one stalk SET restores long whether lat is still paused or already active. Default double-pull does not apply to that resume.
- **During ALC / leftover keep-alive:** same-direction physical stalk held past `STALK_ALC_TURN_HOLD_S` (1.0s) cancels ALC, stops OP blinker keep-alive, and pauses lat as a driver turn. A shorter same-direction hold does not force a turn. Opposite brief tap still cancels ALC without becoming a turn; opposite held uses the 0.40s window.

`LANE_CHANGE_SPEED_MIN` (20 mph) still gates *starting* the lane-change maneuver after a nudge, not tip vs turn and not pause vs ALC. Hazards do neither. ALC keep-alive lamps do not pause lat.

## Driver-wheel temporary lateral handoff

**Default On** (`NAPDriverLatHandoff=1`). A light purposeful push (`|torsion| >= ~0.55 Nm` for ~90 ms + `handsOnLevel >= 1`) **frees the EPS** (`latActive` false → `DAS_steeringControlType=0`), not follow-measured angle hold. Stay yielded while hands are on the rim; 1 s blend starts after ~0.25 s of hands truly off. Emergency/hard brake (`aEgo <= −3.5 m/s²` for 80 ms + digital Applied) during yield / 2 s window fully cancels OP. Light brake is still the silent long pause. Blinker-turn resume uses the same pin-to-wheel + 1 s blend (stay free if hands still on). Turn Off in Settings → NAP if false-yields remain. See [engagement.md](engagement.md#driver-wheel-temporary-lateral-handoff).

## OSM map speed (MAX)

`selfdrive/mapd` looks up OpenStreetMap `maxspeed` from an offline sqlite and publishes `liveMapDataNAP`. In pre-AP **pedal** mode, `card.py` overlays that limit onto software cruise (`CS.vCruise` / HUD **MAX**): Cap never raises, Follow tracks the sign, engage seeds to the posted limit, and a Follow stalk set (above or below) holds until the posted limit changes. HUD current speed is wheel/ESP `vEgo` (not `vEgoCluster` / MAX). When ego is above MAX the planner commands Accel-5 comfort decel (`map_track_decel`); when ego is below MAX it commands Accel 1–10 climb (`map_track_accel`). `LongitudinalMpc.update(radarState, v_cruise)` still takes `min(lead, cruise)`. No-pedal stock CC is display-only. US maps are a GitHub Release download; Refresh maps overlays live OSM within 100 miles. Details: [map-speed.md](map-speed.md).
