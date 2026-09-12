"""Hypermile Hill Climb: IMU-pitch grade hold and crest ease.

v1 signal is `carControl.orientationNED[1]` (same pitch `get_coast_accel`
already reads). No OSM / maps-elevation lookahead in this module.

Active only when Hypermile is On *and* the Hill Climb sub-toggle is On
(`NAPHypermileHillClimb`, default On). Hidden/inert when Hypermile is Off.

Does not raise HUD MAX / vCruise. Lead-approach and MPC hard brake still
win via the planner `min()`. Flat road leaves Accel 1 / map_track alone.
"""
from __future__ import annotations

import math

from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.mapd.constants import TRACK_DEADBAND_MS, TRACK_TAPER_MS

PARAM_HILL_CLIMB = "NAPHypermileHillClimb"

# Earth g. Same coefficient VirtualDAS / the Pre-AP plant use for grade load.
# Accel 1 Early is clamped at 0.30 m/s²; a 2° (~3.5%) grade already needs
# ~0.34 m/s² just to hold, so uncompensated Accel 1 sags on a real climb.
GRAVITY_MS2 = ACCELERATION_DUE_TO_GRAVITY

# Clearly uphill / downhill. ~1° is road crown + IMU bias; 2° is a real grade
# (≈3.5%) Accel 1 cannot hold. 4° (existing full-loop uphill fixture) is
# well above this.
PITCH_CLIMB_RAD = math.radians(2.0)
PITCH_DOWN_RAD = -PITCH_CLIMB_RAD
# Start crest ease as pitch falls through this after a climb (~1.7%).
PITCH_CREST_RAD = math.radians(1.0)

# Extra +a from grade, on top of flat Accel 1–10. Capped at Normal/Accel-5
# comfort (0.80) so a mountain grade cannot punch. Cruise get_max_accel and
# lead-close still clip after this.
CLIMB_EXTRA_MAX_MS2 = 0.80

# Crest / downhill ease. Hypermile Early map brake is 0.55; Late is 1.20
# (the bite that makes people sick). Stay well under Early.
CREST_EASE_MS2 = 0.15
DOWNHILL_EASE_MS2 = 0.25


def hill_climb_applies(hypermile_on: bool, hill_climb_on: bool) -> bool:
  return bool(hypermile_on) and bool(hill_climb_on)


def read_hypermile_hill_climb(params) -> bool:
  """Default On. Inert unless Hypermile is On. Missing / test double → On."""
  try:
    return bool(params.get_bool(PARAM_HILL_CLIMB))
  except Exception:
    return True


def grade_load_ms2(pitch_rad: float) -> float:
  """+a (m/s²) needed to hold speed against gravity. Positive = uphill."""
  return GRAVITY_MS2 * math.sin(float(pitch_rad))


def climb_authority_ms2(flat_a: float, pitch_rad: float) -> float:
  """Flat map-track climb plus grade load, extra capped.

  Used when clearly uphill and ego is under HUD MAX. Does not raise MAX.
  """
  extra = min(CLIMB_EXTRA_MAX_MS2, max(0.0, grade_load_ms2(pitch_rad)))
  return float(flat_a) + extra


def downhill_ease_ms2(pitch_rad: float) -> float:
  """Light |regen| (positive m/s²) on a downhill / crest. Never Late-hard."""
  if float(pitch_rad) > 0.0:
    return CREST_EASE_MS2
  span = max(1e-6, abs(PITCH_DOWN_RAD))
  scale = min(1.0, abs(float(pitch_rad)) / (2.0 * span))
  return CREST_EASE_MS2 + (DOWNHILL_EASE_MS2 - CREST_EASE_MS2) * scale


def _near_max(v_ego_ms: float, v_cruise_ms: float, in_deadband: bool) -> bool:
  if in_deadband:
    return True
  return float(v_ego_ms) + TRACK_TAPER_MS >= float(v_cruise_ms)


def _flattening_crest(pitch_rad: float, prev_pitch_rad: float) -> bool:
  """Pitch falling out of a climb — the crest, without map lookahead."""
  return (
    float(prev_pitch_rad) >= PITCH_CREST_RAD
    and float(pitch_rad) < float(prev_pitch_rad)
    and float(pitch_rad) < PITCH_CLIMB_RAD
  )


def apply_hill_climb(
  *,
  pitch_rad: float,
  prev_pitch_rad: float,
  v_ego_ms: float,
  v_cruise_ms: float,
  a_cmd: float,
  in_deadband: bool,
  hypermile_on: bool,
  hill_climb_on: bool,
) -> float:
  """Adjust map-track / hold a for grade. Never writes v_cruise / MAX.

  Caller still `min()`s with lead-approach and clips to cruise / lead-close.
  Negative a_cmd on an uphill (map brake / MPC / lead) is left alone.
  """
  if not hill_climb_applies(hypermile_on, hill_climb_on):
    return float(a_cmd)

  pitch = float(pitch_rad)
  cmd = float(a_cmd)
  under_max = (not in_deadband) and (float(v_cruise_ms) - float(v_ego_ms) > TRACK_DEADBAND_MS)
  near_max = _near_max(v_ego_ms, v_cruise_ms, in_deadband)

  # Map / MPC / lead already braking on an uphill or flat — do not fight.
  if cmd < 0.0 and pitch >= 0.0:
    return cmd

  if pitch >= PITCH_CLIMB_RAD:
    extra = min(CLIMB_EXTRA_MAX_MS2, max(0.0, grade_load_ms2(pitch)))
    if under_max:
      # Raise climb authority so Accel 1 leftover still walks toward MAX.
      if cmd >= 0.0:
        return cmd + extra
      return cmd
    # At / above MAX: hold against gravity only. No extra climb past MAX.
    return max(cmd, extra) if cmd >= 0.0 else cmd

  if pitch <= PITCH_DOWN_RAD:
    ease = downhill_ease_ms2(pitch)
    grade = abs(grade_load_ms2(pitch))
    if under_max and cmd > 0.0:
      # Gravity already pulls toward MAX; do not punch +a downhill.
      return max(0.0, cmd - min(CLIMB_EXTRA_MAX_MS2, grade))
    if near_max:
      return min(cmd, -ease)
    return cmd

  if _flattening_crest(pitch, prev_pitch_rad) and near_max:
    # Early light regen over the top — not a Late 1.20 dump.
    return min(cmd, -CREST_EASE_MS2)

  return cmd
