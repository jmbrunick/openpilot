"""Map-speed policy and OSM lookup constants."""

# NAPMapSpeedMode
MODE_OFF = 0
MODE_DISPLAY = 1
MODE_CAP = 2
MODE_FOLLOW = 3

MODE_NAMES = {
  MODE_OFF: "off",
  MODE_DISPLAY: "display",
  MODE_CAP: "cap",
  MODE_FOLLOW: "follow",
}

# Pedal-mode software cruise is the only path that may change vCruise from maps.
# No-pedal / pcmCruise stock CC is display-only (do not spoof stalk to chase limits).
DRIVER_OVERRIDE_S = 10.0

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

# NAPMapSpeedAccel: 1=gentlest, 5=current Normal a, 10=quickest. Scales LOOKAHEAD_TUNING a.
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


def map_comfort_a_ms2(lookahead: int, accel_level: int = ACCEL_DEFAULT) -> float:
  """Comfort decel (m/s²) for map-driven MAX changes.

  Lookahead Normal + accel 5 → 0.80 m/s² (the previous hard-coded curve).
  """
  if lookahead in LOOKAHEAD_TUNING and LOOKAHEAD_TUNING[lookahead][0] > 0:
    a_base = LOOKAHEAD_TUNING[lookahead][0]
  else:
    a_base = LOOKAHEAD_TUNING[LOOKAHEAD_NORMAL][0]
  return max(A_CLAMP_MIN, min(A_CLAMP_MAX, a_base * accel_scale_factor(accel_level)))
