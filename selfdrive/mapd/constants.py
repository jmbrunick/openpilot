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

# Default offline DB location on comma 3X / PC
DB_FILENAME = "speed_limits.sqlite"
