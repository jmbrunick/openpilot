"""Map-speed policy and OSM lookup constants."""

# NAPMapSpeedMode
MODE_OFF = 0
MODE_DISPLAY = 1
MODE_CAP = 2
MODE_FOLLOW = 3

# Pedal-mode software cruise is the only path that may change vCruise from maps.
# No-pedal / pcmCruise stock CC is display-only (do not spoof stalk to chase limits).
# Follow only, and only for a manual set *above* the posted limit. A set below
# the limit is sticky (no timeout) until another stalk or the posted value changes.
DRIVER_OVERRIDE_S = 10.0
# Stalk +/- while engaged. 0.4 kph ≈ 0.25 mph; ignore noise / engage 0↔set.
MANUAL_SET_EPS_KPH = 0.4
# Posted OSM maxspeed treated as the same sign (limits are 5 mph / 10 kph steps).
POSTED_LIMIT_EPS_KPH = 1.0

# Spatial match
MAX_MATCH_DISTANCE_M = 35.0
HEADING_ALIGN_DEG = 55.0
# Heading-aligned probes for nextSpeedLimit (any change). Policy uses decreases only.
LOOKAHEAD_STEP_M = 40.0
LOOKAHEAD_MAX_M = 600.0
LOOKAHEAD_M = tuple(float(d) for d in range(int(LOOKAHEAD_STEP_M), int(LOOKAHEAD_MAX_M) + 1, int(LOOKAHEAD_STEP_M)))
SEARCH_PAD_DEG = 0.001  # ~110 m of lat

# NAPMapSpeedLookahead: 0=off, 1=late, 2=normal, 3=early
LOOKAHEAD_OFF = 0
LOOKAHEAD_LATE = 1
LOOKAHEAD_NORMAL = 2
LOOKAHEAD_EARLY = 3
# (comfort decel m/s², extra margin m, max start distance m)
LOOKAHEAD_TUNING = {
  LOOKAHEAD_OFF: (0.0, 0.0, 0.0),
  LOOKAHEAD_LATE: (1.20, 40.0, 250.0),
  LOOKAHEAD_NORMAL: (0.80, 80.0, 400.0),
  LOOKAHEAD_EARLY: (0.55, 120.0, 600.0),
}
MIN_DECREASE_MS = 0.45  # ~1 mph; ignore jitter

# NAPMapSpeedAccel: 1=gentlest, 5=default, 10=quickest. Scales Follow *accel*
# (MAX rising / speeding up). Brake to a lower MAX is locked at ACCEL_DEFAULT.
ACCEL_MIN = 1
ACCEL_DEFAULT = 5
ACCEL_MAX = 10
# factor(1)=0.45, factor(5)=1.00, factor(10)=2.00
ACCEL_FACTOR_LO = 0.45
ACCEL_FACTOR_HI = 2.00
A_CLAMP_MIN = 0.30  # m/s²
A_CLAMP_MAX = 1.60  # m/s²; below MPC cruise min accel magnitude and COMFORT_BRAKE
# Track HUD MAX when ego is faster. MPC cruise_obstacle sits ~safe-follow
# distance ahead (~240 m at 70 mph) and does not bind inside the 10 s horizon
# for a typical posted-limit drop, so v_cruise alone will not command decel.
TRACK_DEADBAND_MS = 0.40  # ~0.9 mph; ignore set-speed / GPS jitter
TRACK_TAPER_MS = 2.00     # ~4.5 mph; full comfort a above this error

# Default offline DB location on comma 3X / PC
DB_FILENAME = "speed_limits.sqlite"


def accel_scale_factor(level: int) -> float:
  lvl = max(ACCEL_MIN, min(ACCEL_MAX, int(level)))
  if lvl <= ACCEL_DEFAULT:
    return ACCEL_FACTOR_LO + (1.0 - ACCEL_FACTOR_LO) * (lvl - ACCEL_MIN) / (ACCEL_DEFAULT - ACCEL_MIN)
  return 1.0 + (ACCEL_FACTOR_HI - 1.0) * (lvl - ACCEL_DEFAULT) / (ACCEL_MAX - ACCEL_DEFAULT)


def map_brake_a_ms2(lookahead: int) -> float:
  """Fixed comfort |a| for map-driven decreases (locked at Accel 5)."""
  return map_comfort_a_ms2(lookahead, ACCEL_DEFAULT)


def map_accel_a_ms2(lookahead: int, accel_level: int) -> float:
  """Comfort |a| for Follow speeding up toward a higher MAX (Accel 1–10)."""
  return map_comfort_a_ms2(lookahead, accel_level)


def map_comfort_a_ms2(lookahead: int, accel_level: int = ACCEL_DEFAULT) -> float:
  """Comfort |a| (m/s²). Lookahead Normal + accel 5 → 0.80 m/s²."""
  if lookahead in LOOKAHEAD_TUNING and LOOKAHEAD_TUNING[lookahead][0] > 0:
    a_base = LOOKAHEAD_TUNING[lookahead][0]
  else:
    a_base = LOOKAHEAD_TUNING[LOOKAHEAD_NORMAL][0]
  return max(A_CLAMP_MIN, min(A_CLAMP_MAX, a_base * accel_scale_factor(accel_level)))
