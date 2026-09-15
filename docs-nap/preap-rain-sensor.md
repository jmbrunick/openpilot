# Pre-AP Light/Rain sensor on CAN

Investigation (Justin’s 2014 Model S Pre-AP): can NAP read the windshield Light/Rain sensor for Auto wipers instead of (or as a gate on) the comma 3X ROAD camera path from #159?

Justin’s stalk is **Off / Int / On only — no Auto detent.** Stock therefore never arms rain-sensing wipers. NAP Auto is a UI setting that spoofs `0x45` nibble 1; it does not move a physical Auto position.

**Verdict: stick with camera Auto.** Ambient light from that module is expected on chassis CAN with the stalk at Off (proves the sensor is alive). Rain is LIN-private to the BCM and is **not** in the DBC as a wetness / auto-wipe-need signal. Without a named always-on rain field, NAP cannot use the module as `rain_needed`. Do not change panda safety.

## Answers

### 1. Can rain/light still appear on CAN without stalk Auto?

**Light: yes (expected).** `BODY_R1` (`0x283`) `LgtSens_*` is chassis body status (doors, temps, lights). Tesla’s light test does **not** use the wiper stalk: headlights Auto on the touchscreen, shine a lamp, MCU day/night brightness changes. Auto headlights use this same module in low light. Nothing in the DBC gates `LgtSens_*` on `WprSw6Posn`.

**Rain: no named CAN signal, Auto or not.** `tesla_preap.dbc` / `tesla_can.dbc` have no `RainSnsr`, rain intensity, or BCM auto-wipe request. Wiring: `RainLightSensor` is 12V + **LIN Data RLS** + GND into the BCM. The BCM may still see rain on LIN whenever the module is powered; it does not republish that as a chassis CAN wetness field next to `LgtSens_*`. Stock only *uses* rain for blades when the stalk is in Tesla’s rain-armed detents (service: 1st/2nd intermittent). This car has none, so stock never rain-wipes. That does not create a CAN rain bit.

### 2. Is ambient light always published (proves sensor alive)?

**Yes — that is the live proof, and it does not need wiper Auto.** With the car awake and stalk Off, `0x283` should keep showing `LgtSens_Night` / `LgtSens_Twlgt` (or `LgtSens_SNA` / `LgtSens_Flt` if the module is missing, unplugged, or blinded). Cover/uncover or a lamp should move those bits. That proves LIN light → BCM → GTW chassis is up. It does **not** prove rain is on CAN. Cabin HVAC humidity is a different part.

If `LgtSens_SNA`/`Flt` stay set with the 3X mounted, the optical block is likely occluded — rain on LIN would be junk too.

### 3. Could NAP Auto use an always-on rain signal without stalk Auto?

**Architecturally yes; in the DBC no.** NAP Auto already ignores the physical stalk: `rain_needed()` (camera today) holds `0x45` TIPWIPE nibble 1, then rest-cancels. A bus-0 rain bit that is valid at stalk Off could feed that same latch (replace or AND-gate camera). Light cannot: `LgtSens_*` is day/night, not wet glass. `WprOutsdPkPosn` is blades-in-park after a wipe.

There is no such rain bit to wire up. A different idea — spoof the **stalk INTERVAL position** so the BCM uses LIN rain itself — is below. That is not a CAN wetness field, and it is **not** what NAP Int sends today.

### 4. Does the sensor only wake with stalk Auto?

**Light: no.** Headlights Auto / MCU brightness run with wipers Off.

**Rain-to-wipers: yes, stock.** Tesla service will not rain-wipe unless intermittent 1/2 is selected. No Auto detent → stock never arms rain wipers.

**Rain-on-LIN: probably still powered** (same 12V connector as light). NAP cannot tap LIN. **Rain-on-CAN: not in the DBC**, so NAP cannot read it even if LIN is wet.

**Recommendation: keep #159 camera Auto** unless a parked INTERVAL1 hold + spray (below) shows the BCM rain-wiping on this VIN. Do not merge a stalk-INTERVAL tip until that is proven.

## Stalk INTERVAL spoof (follow-up)

Hypothesis: SCCM already publishes wiper position on `0x45`; rain is LIN to BCM. If we spoof Tesla’s rain-armed collar value (`INTERVAL1`/`2`), stock rain wipers might run even though the physical stalk has no Auto detent.

**Do not implement yet.** Encoding is clear. Whether *this* BCM rain-arms `INTERVAL1` is not. NAP Int is a different signal.

### 1. Exact `0x45` encodings (`STW_ACTN_RQ`)

Two independent fields. No DBC value named AUTO.

**Collar / 6-position switch — `WprSw6Posn` bits 48–50 (byte 6 bits 0–2).** Overlay must be `(byte6 & ~0x07) | posn` so the live `MC_STW_ACTN_RQ` nibble (byte 6 bits 4–7) stays.

| DBC | Value | Byte 6 low 3 bits | Tesla service / 2014 manual (typical) | Justin Off/Int/On |
| --- | --- | --- | --- | --- |
| OFF | 0 | `0x00` | Off | Off |
| INTERVAL1 | 1 | `0x01` | 1st intermittent / Auto low sensitivity | **candidate rain-arm** |
| INTERVAL2 | 2 | `0x02` | 2nd intermittent / Auto high sensitivity | **candidate rain-arm** |
| INTERVAL3 | 3 | `0x03` | unused on 4-click collar | — |
| INTERVAL4 | 4 | `0x04` | unused on 4-click collar | — |
| STAGE1 | 5 | `0x05` | 3rd: continuous slow (Low) | likely physical On |
| STAGE2 | 6 | `0x06` | 4th: continuous fast (High) | — |
| SNA | 7 | `0x07` | invalid | never send |

**Tip / wash — `WprWashSw_Psd` bits 20–21 (byte 2 bits 4–5), Justin’s `00ffN0` nibble.**

| DBC | Value | Byte 2 | NAP today |
| --- | --- | --- | --- |
| NPSD | 0 | `0x00` | rest / Off |
| TIPWIPE | 1 | `0x10` | **Int, On, and Auto-when-wet** (held) |
| WASH | 2 | `0x20` | never send |
| SNA | 3 | `0x30` | never send |

`WprWash_R_Sw_Posn_V2` (byte 2 bits 6–7): `OFF` / `INTERVAL` / `WASH`. Name looks like rear wash; Model S has no rear wiper. Do not spoof this first.

### 2. INTERVAL1/2 vs NAP Int (nibble 1)

**Distinct.** NAP Int/On/Auto-wet set `WprWashSw_Psd=TIPWIPE` (`0x10`) and **preserve** live `WprSw6Posn` (Off stalk → 0). That is why Pre-AP latches ~32 s Low and Auto must rest-cancel. Tesla rain-arm is `WprSw6Posn=1` or `2` with `WprWashSw_Psd=0`. Same ID `0x45`, different bytes. Panda already allows TX of `0x45`; this would not add a TX ID. Still a last-win fight with bus-0 rest (same as today’s Int hold).

### 3. Will BCM rain-arm on a car that never had Auto?

**Unknown — no VIN/config rain flag in the DBC.** `GTW_carConfig` (`0x398`) has `GTW_dasHw`, `GTW_autopilot`, `GTW_bodyControlsType` (1 bit, undocumented) — nothing named rain/RLS/auto-wiper. `MCU_enableAutowipers` is DAS/MCU, not BCM.

Plausible yes: same Light/Rain module (LIN alive if `LgtSens_*` moves), same BCM, INTERVAL1 is just another `WprSw6Posn` the SCCM never reaches on a 3-click stalk. Plausible no: BCM EEPROM “rain wipers” off; physical Int already *is* INTERVAL1 and still timed-only; 3X blinds the optics (`LgtSens_Flt`/`SNA`).

**First measurement (no spoof):** at physical Off / Int / On, log `0x45` byte 6 `& 0x07` and byte 2. If Int is already `WprSw6Posn=1` and spray does not rain-wipe, spoofing INTERVAL1 cannot help.

### 4. Parked test (no merge, no panda-safety change)

Parked, car on, **NAP Wipers Off**, physical stalk **Off**, headlights Auto on the MCU so light path is up.

1. Dump live `0x45` at Off / Int / On / end-button tip. Confirm INTERVAL vs TIPWIPE vs STAGE1.
2. Cover/uncover sensor: `0x283` `LgtSens_*` must move. SNA/Flt → stop (optics dead).
3. Only if Off is `WprSw6Posn=0` and Int is not already `1`: overlay **one** live `0x45` with `WprSw6Posn=1`, `WprWashSw_Psd=0`, same MC, resign CRC — hold at ~10 Hz like today’s Int so bus-0 rest does not last-win. **Do not** set `0x10` or `0x20`.
4. Dry: stock Auto should **not** wipe. If blades cycle on dry glass, BCM treated INTERVAL as timed Int → abandon, rest-cancel (`WprSw6Posn=0` + today’s nibble-0 rest).
5. Spray glass at the sensor: blades should sweep without camera / without NAP Auto. Then Off overlay (`WprSw6Posn=0`) must cancel.
6. Keep panda TX whitelist as-is (`0x45` only). No new IDs. Parked first; road only after dry-no-wipe and wet-wipe both work.

If step 5 fails, keep camera Auto.

## Hardware (stock)

Tesla service: *Sensor - Light/Rain (Vehicles without Autopilot and with 1st Generation Autopilot)* at the windshield base of the interior mirror.

2012/2014 LHD wiring (sheet 25): BCM drives `WIPER SLOW` / `FAST` / `PARK` discrete. The sensor is not a CAN node.

2014 owner’s manual labels the first two collar clicks “Auto with low/high rain sensitivity.” Justin’s car is Off/Int/On with no rain Auto — treat Int as timed intermittent, not rain-armed.

## What the DBC actually has

| ID | Name | Bus (panda) | What it is | Needs stalk Auto? | Use for Auto wipe? |
| --- | --- | --- | --- | --- | --- |
| `0x283` (643) | `BODY_R1` | chassis / party **0** (GTW) | Light half + wiper **park** | No | Light only. Not rain. |
| `0x45` (69) | `STW_ACTN_RQ` | 0 | Stalk: `WprSw6Posn`, `WprWashSw_Psd` | — | Driver request, not sensor. |
| `0x3E9` (1001) | `DAS_wiperSpeed` | NAP TX whitelist | Autopilot camera wiper request | — | Pre-AP has no DAS. NAP keeps 0. |
| `0x218` (536) | `MCU_enableAutowipers` | 0 | MCU enable bit | — | Setting, not wetness. |
| `0x30E` (782) | `PARK_sdiNoise=RAIN` | — | Ultrasonic noise | — | Not windshield rain. |

`BODY_R1`: `LgtSens_Twlgt` (0–7), `LgtSens_Night` (DAY/NIGHT), `LgtSens_Tunnel`, `LgtSens_Flt`, `LgtSens_SNA`, `ADL_LoBm_On_Rq`, `WprOutsdPkPosn`.

`WprSw6Posn`: `OFF`, `INTERVAL1`–`4`, `STAGE1`, `STAGE2` — no AUTO enum. NAP’s held `00ff10` is `WprWashSw_Psd=TIPWIPE` (BCM Low), which is why Auto pulses then rest-cancels.

`preap/carstate.py` does not parse `BODY_R1`. `preap_windshield_rain.py` is still the Auto-wipe need.

## Panda / forwarding

Panda on Pre-AP is party/chassis **bus 0**. USB `can` already includes every frame on that bus. Safety RX checks do not filter host visibility. `fwd_hook` only blocks 0↔2. No extra forward for `0x283`. **Do not add rain/wiper TX. Do not weaken safety.**

## Live capture (Justin)

Parked, vehicle on, **physical stalk Off**, **NAP Wipers Off**.

```bash
cd /data/openpilot
python selfdrive/debug/can_print_changes.py --bus 0
```

1. Confirm `0x283` on src 0. Cover/uncover the sensor (or a lamp). `LgtSens_Night` / `LgtSens_Twlgt` should move with wipers Off — module alive. SNA/Flt stuck → 3X may be blinding it.
2. Stalk still **Off**. Spray the glass in front of the sensor. Stock should **not** wipe. If no ID changes before any blade motion, rain is not on chassis CAN (expected).
3. Optional: same spray on physical **Int**. If blades still do not follow the spray, Int is not rain-armed (matches Justin). Do not treat INTERVAL spoof as a rain source.
4. Optional: NAP Auto On, camera dry, to see spoofed `0x45` vs rest.

Watch `0x283`, `0x45`, `0x318`, plus any new address on spray. Unused `BODY_R1` bits (~12, 14–15, 30–31) are the only plausible hidden rain nibble — a spray-correlated change at stalk **Off** would be the only candidate to replace camera. None is documented.
