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
| Map Speed Offset | −5 mph |
| Lookahead | **Early** (3) — starts farther out at 0.55 m/s². Always snapped (Late/Normal/Off become Early). Late is 1.20 m/s² and is *not* kept. |
| Acceleration (map climb) | **1** — laziest Follow climb (lower peak a / jerk than default 5) |

Turning **Off** restores that snapshot. Soft Lateral Handoff, Simulate Look-at-Road, blinker / sticky MAX / one-SET / standstill gas-gate / reverse hard-cancel, and the stock 1–7 Follow Distance param are **not** changed. The stalk / 50 mph follow design is separate and unchanged by this eco bias.

Params: `NAPHypermile` (bool, default 0), `NAPHypermileSaved` (JSON snapshot).

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

## Tests

`selfdrive/controls/tests/test_hypermile.py` — On snaps + Off restore; ≤50 far gap; >50 stalk level; stalk up/down 1–5; safe floor; settings/docs wiring.

Existing `test_following_distance` / Pre-AP following tests stay on the stock 1–7 path while Hypermile is Off (default).
