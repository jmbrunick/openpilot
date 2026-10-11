"""Sound id for the animal/person chime.

Kept out of soundd so tests can import it without cereal. soundd plays
selfdrive/assets/sounds/animal_chime.wav when the alert type starts with
obstacleChime. AudibleAlert.none stays on the alert itself.
"""
from __future__ import annotations

import math

OBSTACLE_CHIME_ID = -113
OBSTACLE_CHIME_FILE = "animal_chime.wav"
# Quiet-cabin scaling cannot drop this one sound below this level.
OBSTACLE_CHIME_MIN_VOLUME = 0.7


def floor_volume(alert_id, volume: float) -> float:
  """Keep the obstacle chime at or above 0.7. Every other sound is unchanged."""
  if alert_id != OBSTACLE_CHIME_ID:
    return volume
  try:
    level = float(volume)
  except (TypeError, ValueError):
    level = 0.0
  if not math.isfinite(level):
    level = 0.0
  return max(level, OBSTACLE_CHIME_MIN_VOLUME)


def alert_sound_id(raw, alert_type: str = "") -> int:
  """Map a selfdriveState alert onto a sound_list key."""
  text = "" if alert_type is None else str(alert_type)
  if text.startswith("obstacleChime"):
    return OBSTACLE_CHIME_ID
  try:
    return int(raw)
  except (TypeError, ValueError):
    return 0
