# Pre-AP Light/Rain sensor on CAN

Investigation (2014 Model S Pre-AP / HW1, no Autopilot): can NAP read the windshield Light/Rain sensor for Auto wipers instead of (or as a gate on) the comma 3X ROAD camera path from #159?

**Verdict: no usable rain-intensity or auto-wiper-need signal is in the DBC.** The module is real. Rain stays private to the BCM on LIN. Light-side status is on chassis CAN. Keep camera Auto until a live capture proves otherwise. Do not change panda safety.

## Hardware (stock)

Tesla service: *Sensor - Light/Rain (Vehicles without Autopilot and with 1st Generation Autopilot)* sits at the windshield base of the interior mirror.

Service rain test: set the stalk to the **1st or 2nd intermittent** position, spray water in front of the sensor, blades should sweep. Light test: headlights Auto, shine a lamp at the sensor, MCU day/night brightness should change.

2012/2014 LHD wiring (sheet 25, Wiper/Washer): `RainLightSensor` is three wires — 12V, **LIN Data RLS**, LIN GND — into the BCM. The BCM then drives the motor on discrete lines (`WIPER SLOW`, `WIPER FAST CNTRL`, `WIPER PARK SENSE`, `WIPER PARK CNTRL`). The sensor is not a CAN node.

Owner-manual Auto is those same intermittent detents: wipe rate follows how much water the sensor sees. Stalk Off does not auto-wipe.

NAP Auto is a **software** setting that spoofs `0x45` nibble 1 (held TIPWIPE / Low). That is not the physical Auto detent. Camera rain exists because we assumed no CAN rain — this note confirms that assumption in the DBC.

## What the DBC actually has

`opendbc_repo/opendbc/dbc/tesla_preap.dbc` (same names in `tesla_can.dbc`). No `RainSnsr`, `LightRain`, humidity-on-glass, or auto-wiper-request.

| ID | Name | Bus (panda) | What it is | Use for Auto wipe? |
| --- | --- | --- | --- | --- |
| `0x283` (643) | `BODY_R1` | chassis / party **0** (GTW-published) | Light half of the module, plus wiper **park** | Light only. Not rain. |
| `0x45` (69) | `STW_ACTN_RQ` | 0 | Stalk: `WprSw6Posn`, `WprWashSw_Psd` | Driver request, not sensor. |
| `0x3E9` (1001) | `DAS_bodyControls` / `DAS_wiperSpeed` | NAP TX whitelist | Autopilot camera wiper request | Pre-AP has no DAS. NAP keeps this 0. |
| `0x218` (536) | `MCU_chassisControl` / `MCU_enableAutowipers` | 0 | MCU enable bit | Setting, not wetness. |
| `0x30E` (782) | `PARK_status2` / `PARK_sdiNoise=RAIN` | — | Ultrasonic noise | Not windshield rain. |

`BODY_R1` light/wiper bits:

- `LgtSens_Twlgt` (0–7 steps), `LgtSens_Night` (DAY/NIGHT), `LgtSens_Tunnel`, `LgtSens_Flt`, `LgtSens_SNA`
- `ADL_LoBm_On_Rq` (auto low-beam request)
- `WprOutsdPkPosn` — blades in/out of park (**effect** of a wipe, not rain need)

`STW_ACTN_RQ` wiper bits (`WprSw6Posn`): `OFF`, `INTERVAL1`–`INTERVAL4`, `STAGE1`, `STAGE2`. There is no AUTO enum. Physical Auto **is** INTERVAL1/2. NAP’s held `00ff10` is `WprWashSw_Psd=TIPWIPE`, which Pre-AP latches as continuous Low — that is why Auto pulses then rest-cancels.

`preap/carstate.py` does **not** parse `BODY_R1`. `preap_windshield_rain.py` still says there is no rain sensor.

## Panda / forwarding

Panda on Pre-AP is on **party/chassis bus 0**. USB `can` already includes every frame on that bus. Safety RX checks (`0x370`, `0x108`, `0x118`, `0x20A`, `0x368`, `0x318`, `0x45`, `0x155`) do **not** filter host visibility.

`tesla_preap_fwd_hook` returns true and **blocks 0↔2 forwarding** (no AP ECU). That is inter-bus, not “hide BODY_R1 from the 3X”.

Radar GTW emulation remaps `0x45` and `0x30A` (`BC_status`) chassis→radar. It does not invent rain.

TX whitelist already has `0x45` (stalk spoof) and `0x3E9` (blinker-only; `DAS_wiperSpeed` stays 0). **Do not add rain/wiper TX. Do not weaken safety.**

If rain lived only on Body CAN (`CANB`, door modules), panda would not see it. BCM is also on chassis (`CANC_RHS` at the BCM connector), and `BODY_R1` is already gatewaved onto chassis — so **light** is visible. Rain was never named next to it.

## If a signal showed up

A named rain level or BCM auto-wipe request on bus 0 could replace or AND-gate `windshield_rain_needed()` while stalk Auto is selected. Nothing like that is decoded today. `LgtSens_*` is ambient light (headlights / MCU brightness), not wet glass. `WprOutsdPkPosn` only moves after BCM already wiped.

## Live capture (Justin)

Parked, vehicle on, **NAP Wipers = Off** (do not spoof `0x45`). SSH on the 3X:

```bash
cd /data/openpilot
python selfdrive/debug/can_print_changes.py --bus 0
```

Cabana / a route log is finer. Confirm `0x283` is present on src 0 before spraying.

1. Cover/uncover the sensor (or shine a lamp). `BODY_R1` `LgtSens_Night` / `LgtSens_Twlgt` should move. If `LgtSens_SNA` or `LgtSens_Flt` stays set, the 3X mount may be blinding the module (rain side would be dead too).
2. Physical stalk **INTERVAL1 or INTERVAL2**. Spray the glass in front of the sensor. Blades should sweep (stock). Watch whether **any** ID other than `0x283` park bit / motor-side effect changes *before* the sweep. `0x45` should only change if you moved the stalk.
3. Stalk **Off**, same spray. Stock should not wipe. If no ID changes, rain is not on chassis CAN.
4. Optional: repeat with NAP Auto On and camera dry, to see the spoofed `0x45` vs stock.

IDs to keep: `0x283` (`BODY_R1`), `0x45` (`STW_ACTN_RQ`), `0x318` (`GTW_carState` headlights), plus any new address `can_print_changes` prints on spray. Unused bits in `BODY_R1` (about 12, 14–15, 30–31) are the only plausible hidden rain nibble — treat a spray-correlated change as a candidate, not a merge.

Hypothesis: **confirmed in DBC and wiring.** Sensor talks LIN to BCM. BCM wipes when the stalk is already Auto/Int. Chassis CAN publishes the light half (`BODY_R1`), not rain need.
