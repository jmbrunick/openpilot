# Hypermile (phase 1+2)

Experimental on **nap-dev only**. Default **Off**. Not a nap-release change.

Settings → NAP → Driving Mannerisms → **Hypermile**.

This is a small controller (`selfdrive/controls/lib/hypermile.py`) that snaps eco knobs. It does not rewrite longcontrol, disable lead braking, or raise MAX above posted Cap/Follow / sticky MAX. Stock Follow Distance 1–7 (`NAPFollowDistance`) is shared with Driving Mannerisms — Hypermile does **not** own follow levels.

## A — Eco preset (comfort-biased, not max regen)

The goal is **early, light regenerative ease** — not the biggest regen bite. Late hard regen that pitches the car and makes the driver sick is the wrong trade. Safety / MPC hard braking is unchanged.

Turning **On** remembers the current values, then snaps:

| Knob | Eco value |
|------|-----------|
| Adaptive Accel Limits | On (softer peak accel when close to a lead) |
| Map Speed mode | Follow (3), or keep Cap if already Cap. Off/Display → Follow |
| Map Speed Offset | **not snapped**. Live eco offset is posted-scaled from the OSM limit (see below). Off restores the user's saved slider. |
| Lookahead | **Early** (3) — starts farther out at 0.55 m/s². Always snapped (Late/Normal/Off become Early). Late is 1.20 m/s² and is *not* kept. |
| Acceleration (Driving Mannerisms; map climb + lead-close) | **1** — laziest Follow climb and lead-close +a (0.20 m/s² catch-up) |

Turning **Off** restores that snapshot. Soft Lateral Handoff, Simulate Look / False Alert Ignore (triple-tap NAP popup), blinker / sticky MAX / one-SET / standstill gas-gate / reverse hard-cancel, and the stock 1–7 Follow Distance param are **not** changed (eco does not snap or restore follow).

**Posted-scaled eco offset** (Hypermile On, Step Down Off), from the raw posted/OSM limit in mph — not a forever `NAPMapSpeedOffsetMph = −5`:

| Posted | Eco offset | MAX target |
|--------|------------|------------|
| &lt; 50 mph (town 30) | **0** | 30 stays 30 |
| 50 mph | **0** (scale starts) | 50 |
| 50–80 mph | linear 0 → −8 | 65 → 61 |
| 80 mph | **−8** | 72 |
| &gt; 80 mph | **−8** (cap) | 90 → 82 |

**Curves:** eco is the live posted-scaled Cap/Follow target. If MAX was 60 before a sharp bend (sticky hold or that displayed set), curve slowing may move HUD MAX through the corner, then **restore 60** — not leave the eco target. The bend must not permanently rebase sticky / map target. See [map-speed.md](map-speed.md#safety-invariants).

Params: `NAPHypermile` (bool, default 0), `NAPHypermileSaved` (JSON snapshot).

## Hill Climb (grade-aware Accel 1)

Third toggle under Hypermile: **Hill Climb** (`NAPHypermileHillClimb`, default **On**). Hidden/inert unless Hypermile is On. Same visibility pattern as Step Down Speed.

**v1 signal is IMU pitch** (`carControl.orientationNED[1]`, the same value `get_coast_accel` already reads). **Maps-elevation lookahead is NOT included** — that is a later phase.

Hypermile snaps Driving Mannerisms **Acceleration** (`NAPMapSpeedAccel`) to **1**. Flat-road `map_track_accel` is then a fixed comfort `a` (Early + Accel 1 → 0.30 m/s²). Grade gravity is not in that number, so a real climb droops under HUD MAX. Stock `get_coast_accel` only lowers +a when `allow_throttle` is false; Pre-AP always allows throttle, so that path does not hold a hill.

| Pitch (NED) | Clearly under HUD MAX (outside deadband below) | At / above MAX (ego ≥ cruise − deadband) |
|-------------|----------------|---------------|
| ≥ **2.0°** uphill (≈3.5% grade) | Add `g·sin(pitch)` to Accel 1 (extra capped at 0.80 m/s²). Leftover Accel 1 still walks toward MAX. | Leave map hold (**0**). **No** `+g·sin` that fights the deadband zero or punches past MAX. **Does not raise MAX.** |
| Flattening crest (pitch falling through < 2° after a climb) | Leave climb alone (no regen while still climbing). | Early **light** regen **0.15 m/s²** — not Late 1.20. |
| ≤ **−2.0°** downhill | Lower +a by the grade (gravity already pulls toward MAX). **No regen** while still several mph under MAX. | Light regen **0.15–0.25 m/s²**. |
| \|pitch\| < 2° (flat / crown) | Existing Accel 1 / map_track / lead-close unchanged. | Unchanged. |

Ease is **not** gated on `TRACK_TAPER` (~4.5 mph / 2.0 m/s under MAX). That taper is map-track climb/brake scaling only. Using it as “near MAX” regen’d around 50 mph under a 54 MAX and hunted 47–51 with regen pulses instead of settling. Climb hold is also off in the deadband so grade does not overshoot into Accel-5 comfort brake.

Cruise `get_max_accel` and the lead-close +a cap still clip after this. Lead-approach and MPC hard brake `min()` after hill climb and still win. Never invents or raises MAX above Cap/Follow / sticky / posted.

**Why these numbers:** 1° is typical road crown + IMU bias; 2° is a real grade Accel 1 (0.30) cannot hold (`g·sin(2°)≈0.34`). Full `g·sin` matches the Pre-AP VirtualDAS plant (not the shallower 5.65 coast fit). Ease stays well under Early map brake (0.55) so it is comfort, not a dump.

## Step Down Speed (mileage defer)

Second toggle under Hypermile: **Step Down Speed** (`NAPHypermileStepDown`, default **Off**). Hidden/inert unless Hypermile is On.

| Hypermile | Step Down | Map MAX target |
|-----------|-----------|----------------|
| On | Off | Posted + posted-scaled eco (30 stays 30; 80→72; 90→82) |
| On | On | Same 50→80 scale, **−15 at 80** (30 stays 30; 75→62.5; 80→65; 90→75). Replaces eco — does not stack |
| Off | On or Off | Step-down ignored. Normal posted / held MAX |

Same breakpoints as eco: **0** at/under 50, linear to the 80 value, cap above 80. Hard cap: never more than 15 mph under posted, never above posted. Legal: step-down only lowers. No flat −15 on town limits.

**Maps-only:** eco and Step Down apply only when maps are present and that road has a known OSM/posted `speedLimit`. Maps off, no match, or posted unknown → **offset 0** — do not invent a drop. Same spirit as sticky MAX (no invented posted without maps). Town 30 with maps stays 30.

This is an offset on the **posted/map target** Cap/Follow already use (`posted_kph` / `apply_map_speed_kph` offset). Sticky / one-SET / rebase are not rewritten.

**What wins**

- **Follow + stalk SET:** driver can hold above the step-down (up to posted). Sticky until the posted *value* changes, then MAX rebases to the new scaled step-down target.
- **Cap:** cannot exceed the stepped posted (80 posted + step-down → cap 65).
- **One-SET** after a long pause: resume held MAX (already rebased if posted or step-down changed).
- **Double SET:** take the current map target (stepped if On).
- Toggling Step Down On/Off mid-drive changes the posted target (scaled eco ↔ scaled step-down), which Follow treats as a posted change and rebases.

## B — Stock Follow Distance 1–7 (not Hypermile-owned)

Follow Distance is the stock Driving Mannerisms slider (`NAPFollowDistance`, 1–7, default 4). It stays **visible** while Hypermile is On. There is no Hypermile 1–5 band, no “never stock 1” floor, and no ≤50 mph forced far-gap (stock 7) override. Planner / MPC always use whatever `NAPFollowDistance` says.

| Stock `NAPFollowDistance` | `T_FOLLOW` |
|---------------------------|------------|
| 1 closest | 0.7 s |
| 2 | 0.9 s |
| 3 | 1.1 s |
| 4 default | 1.3 s |
| 5 | 1.5 s |
| 6 | 1.7 s |
| 7 farthest | 1.9 s |

Gaps come from the existing `NAP_T_FOLLOW` table. Lead braking / MPC danger zone stay on. Cruise ceiling is still HUD MAX (posted Cap/Follow + sticky).

### Stalk remapping (Pre-AP pedal software cruise)

Normally stalk up/down is RES+/RES− and steps MAX / `pedal_speed` 1 or 5 mph.

With a **radar lead present** (Hypermile On **or** Off):

- Stalk **tip / bump** (Tesla first detent, **1 mph** / 1 kph) = Follow Distance 1–7 only. Stalk **up** = closer (toward stock 1). Stalk **down** = farther (toward stock 7). That frame’s MAX / `pedal_speed` step is undone immediately. Follow commits when the lever returns to IDLE after a first-detent tip that never hit 2nd detent. HUD **Follow Distance: N**.
- Stalk **full press** (Tesla 2nd detent, **5 mph** / 5 kph) = keep MAX +5/−5. Do **not** remap Follow Distance, even though a physical full press always walks through first detent.

card.py writes `NAPFollowDistance` on tip-release so the Driving Mannerisms indicator/slider updates live. Cooldown 0.25 s so press+release cannot double-step. Raw `SpdCtrlLvr_Stat` / CruiseButtons (UP_1ST vs UP_2ND, DN_1ST vs DN_2ND) distinguish tip vs hold. Pre-AP `buttonEvents` map both detents to the same `accelCruise`/`decelCruise` and must not be treated as a completed tip.

**No lead:** tip and hold both stay MAX adjust (documented rule). During a long pause, `cruiseState.speed` is ego — detent still classifies tip vs hold.

### HUD

When stock Follow Distance changes onroad, selfdrived fires `EventName.hypermileFollowChanged` — same 1.5 s `NormalPermanentAlert` affordance as Driving Personality: **Follow Distance: N**. First read seeds the HUD baseline (no toast at process start); every later stalk/settings 1–7 change shows the toast. A stalk tip already at 1 or 7 still requests the HUD (`NAPFollowHudPending`). The event is held ~1.5 s (WARNING + PERMANENT) so a one-frame poll does not miss the 3X.

## Safety

- Default Off
- Stay at/under MAX and posted Cap/Follow
- Do not disable lead braking
- Soft-lat / DM / blinker / sticky MAX / one-SET / standstill gas-gate / reverse hard-cancel / One-Pedal Long unchanged
- Hill Climb does not raise MAX and does not disable lead braking
- Eco / Step Down / Hill Climb do not force Follow Distance

## Tests

`selfdrive/controls/tests/test_hypermile.py` — On snaps + Off restore (offset param not written −5; Follow Distance not snapped); posted-scaled eco (30 stays 30; 80→72) and Step Down (30 stays 30; 75→62.5; 80→65); maps-only (no invent without posted); stalk tip+lead writes `NAPFollowDistance` with Hypermile On and Off (full stock 1–7, including closest 1) after tip→IDLE; lead + tip-then-2nd-detent / 5 mph hold keeps MAX and does not remap Follow; button-only press is not a completed tip; no lead still steps MAX on tip and hold; Follow Distance HUD announces every param change after seed; settings/docs wiring.

`selfdrive/controls/tests/test_hill_climb.py` — Hypermile Off / Hill Climb Off inert; uphill under MAX raises Accel 1 authority; several mph under MAX + downhill does **not** regen; deadband + uphill does **not** invent +g·sin; at/above MAX + downhill still eases; never exceeds MAX; lead / MPC brake still wins; under MAX + valid lead does **not** replace non-negative MPC `a` with map climb (no lead still climbs); above MAX + lead still map-decels.

`selfdrive/controls/lib/tests/test_curve_max_hold.py` — curve snapshot/restore (sticky 60 survives a bend; a lower posted/eco target does not replace it).

Existing `test_following_distance` / Pre-AP following tests stay on the stock 1–7 path.
