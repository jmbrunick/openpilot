"""Sound id for the animal/person chime.

Kept out of soundd so tests can import it without cereal. soundd plays
selfdrive/assets/sounds/animal_chime.wav when the alert type starts with
obstacleChime. AudibleAlert.none stays on the alert itself.
"""
from __future__ import annotations

OBSTACLE_CHIME_ID = -113
OBSTACLE_CHIME_FILE = "animal_chime.wav"


def alert_sound_id(raw, alert_type: str = "") -> int:
  """Map a selfdriveState alert onto a sound_list key."""
  text = "" if alert_type is None else str(alert_type)
  if text.startswith("obstacleChime"):
    return OBSTACLE_CHIME_ID
  try:
    return int(raw)
  except (TypeError, ValueError):
    return 0
