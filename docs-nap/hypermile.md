# Hypermile (phase 1+2)

Experimental on **nap-dev only**. Default **Off**. Not a nap-release change.

Settings → NAP → Driving Mannerisms → **Hypermile**.

This is a small controller (`selfdrive/controls/lib/hypermile.py`) that snaps eco knobs and publishes an **effective** stock follow-distance index (1–7) into the existing MPC / lead-approach path. It does not rewrite longcontrol, disable lead braking, or raise MAX above posted Cap/Follow / sticky MAX.

## A — Eco preset (comfort-biased, not max regen)

The goal is **early, light regenerative ease** — not the biggest regen bite. Late hard regen that pitches the car and makes the driver sick is the wrong trade. Safety / MPC hard braking is unchanged.

Turning **On** remembers the current values, then snaps:

| Knob | Eco value |
|------|-----------|
| Adaptive Accel Limits | On (softer peak accel when close to a lead) |
| Map Speed mode | Follow (3), or keep Cap if already Cap. Off/Display → Follow |
| Map Speed Offset | **not snapped**. Live eco offset is posted-scaled from the OSM limit (see below). Off restores the user's saved slider. |
| Lookahead | **Early** (3) — starts farther out at 0.55 m/s². Always snapped (Late/Normal/Off become Early). Late is 1.20 m/s² and is *not* kept. |
| Acceleration (map climb + lead-close) | **1** — laziest Follow climb and lead-close +a (0.20 m/s² catch-up) |

Turning **Off** restores that snapshot. Soft Lateral Handoff, Simulate Look-at-Road (triple-tap NAP popup), blinker / sticky MAX / one-SET / standstill gas-gate / reverse hard-cancel, and the stock 1–7 Follow Distance param are **not** changed. The stalk / 50 mph follow design is separate and unchanged by this eco bias.

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

Hypermile snaps Acceleration to **1**. Flat-road `map_track_accel` is then a fixed comfort `a` (Early + Accel 1 → 0.30 m/s²). Grade gravity is not in that number, so a real climb droops under HUD MAX. Stock `get_coast_accel` only lowers +a when `allow_throttle` is false; Pre-AP always allows throttle, so that path does not hold a hill.

| Pitch (NED) | Under HUD MAX | At / near MAX |
|-------------|----------------|---------------|
| ≥ **2.0°** uphill (≈3.5% grade) | Add `g·sin(pitch)` to Accel 1 (extra capped at 0.80 m/s²). Leftover Accel 1 still walks toward MAX. | Hold against gravity only. **Does not raise MAX.** |
| Flattening crest (pitch falling through < 2° after a climb) | Leave climb alone if still well under MAX (grade term is already shrinking). | Early **light** regen **0.15 m/s²** — not Late 1.20. |
| ≤ **−2.0°** downhill | Lower +a by the grade (gravity already pulls toward MAX). No regen bite away from MAX. | Light regen **0.15–0.25 m/s²**. |
| \|pitch\| < 2° (flat / crown) | Existing Accel 1 / map_track / lead-close unchanged. | Unchanged. |

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

## B — Speed-split follow + stalk 1–5

While On, the stock 1–7 Follow Distance UI is hidden. The active control is **Hypermile Follow 1–5** (`NAPHypermileFollowLevel`, default 3, persists across the drive):

| Hypermile | Stock `NAPFollowDistance` | `T_FOLLOW` |
|-----------|---------------------------|------------|
| 1 closest draft | 2 | 0.9 s |
| 2 | 3 | 1.1 s |
| 3 standard | 4 | 1.3 s |
| 4 | 5 | 1.5 s |
| 5 furthest | 6 | 1.7 s |
| vEgo ≤ 50 mph (any stalk) | 7 | 1.9 s far gap |

- **vEgo ≤ 50 mph:** always stock 7 (far). Cuts stop-and-go brake events.
- **vEgo > 50 mph:** stalk 1–5 as a tighter drafting band. Safe floor is stock 2 / 0.9 s — never stock 1 / 0.7 s bumper-draft.
- Gaps come from the existing `NAP_T_FOLLOW` table. Lead braking / MPC danger zone stay on. Cruise ceiling is still HUD MAX (posted Cap/Follow + sticky).

### Stalk remapping (Pre-AP pedal software cruise)

Normally stalk up/down is RES+/RES− and steps MAX / `pedal_speed` 1 or 5 mph.

With Hypermile **On** and a **radar lead present**:

- Stalk **up** = closer (level toward 1)
- Stalk **down** = farther (level toward 5)

card.py undoes that frame’s MAX step so Follow sticky does not arm. Cooldown 0.25 s so a held lever does not race 1→5.

**No lead:** stalk stays MAX adjust (documented rule). During a long pause, `cruiseState.speed` is ego — only button edges remap follow.

### HUD

When the 1–5 level changes onroad, selfdrived fires `EventName.hypermileFollowChanged` — same 1.5 s `NormalPermanentAlert` affordance as Driving Personality: **Hypermile: Follow N**.

## Safety

- Default Off
- Stay at/under MAX and posted Cap/Follow
- Do not disable lead braking
- Tighten only within the 0.9 s floor above 50 mph
- Soft-lat / DM / blinker / sticky MAX / one-SET / standstill gas-gate / reverse hard-cancel unchanged
- Hill Climb does not raise MAX and does not disable lead braking

## Tests

`selfdrive/controls/tests/test_hypermile.py` — On snaps + Off restore (offset param not written −5); posted-scaled eco (30 stays 30; 80→72) and Step Down (30 stays 30; 75→62.5; 80→65); maps-only (no invent without posted); ≤50 far gap; >50 stalk level; stalk up/down 1–5; safe floor; settings/docs wiring.

`selfdrive/controls/tests/test_hill_climb.py` — Hypermile Off / Hill Climb Off inert; uphill under MAX raises Accel 1 authority; crest/downhill ease direction; never exceeds MAX; lead / MPC brake still wins.

`selfdrive/controls/lib/tests/test_curve_max_hold.py` — curve snapshot/restore (sticky 60 survives a bend; a lower posted/eco target does not replace it).

Existing `test_following_distance` / Pre-AP following tests stay on the stock 1–7 path while Hypermile is Off (default).
