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
LOOKAHEAD_M = (80.0, 160.0, 250.0)
SEARCH_PAD_DEG = 0.001  # ~110 m of lat

# Default offline DB location on comma 3X / PC
DB_FILENAME = "speed_limits.sqlite"
