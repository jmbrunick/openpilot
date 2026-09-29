"""NAP Lateral Control settings content (turn geometry correction)."""
from openpilot.selfdrive.controls.lib.lat_turn_geometry import (
  PARAM_REF_OFFSET, PARAM_TURN_GEOMETRY, REF_OFFSET_DEFAULT_M,
)

NAP_LAT_TURN_GEOM = PARAM_TURN_GEOMETRY
NAP_LAT_REF_OFFSET = PARAM_REF_OFFSET
LAT_REF_OFFSET_PRESETS = [0.0, 0.2, 0.35, 0.5, 0.75, 1.0]
LAT_REF_OFFSET_LABELS = ["0", "0.2", "0.35", "0.5", "0.75", "1.0"]
LAT_REF_OFFSET_DEFAULT = REF_OFFSET_DEFAULT_M
LAT_TURN_GEOM_DESCRIPTION = (
  "Default On. Pre-AP. Aims the steering at the right point of the path "
  + "in turns: uses the measured low-speed steering lag and steers from "
  + "the rear axle, so turns start later and stop clipping corners. "
  + "Highway is unchanged. Also turns off the old roundabout outer bias. "
  + "Off = previous behavior."
)
NAP_ROUNDABOUT_ASSIST = "NAPRoundaboutAssist"   # roundabout_guide.PARAM_ROUNDABOUT_ASSIST
ROUNDABOUT_ASSIST_DESCRIPTION = (
  "Default Off. Pre-AP, experimental. Near a roundabout that is in the map "
  + "data, blends the steering toward the circle using GPS: keeps the right "
  + "entry until ~9 m out, then turns left smoothly and follows the lane "
  + "around. GPS is ~1-3 m off, so the camera still limits it at lane lines "
  + "and curbs, and it backs off when GPS looks poor. Wheel torque takes over "
  + "as usual. Needs Settings > NAP > Map Speed Limit > Refresh maps (or map "
  + "pack v4) so the ring is mapped. Off = model only."
)
LAT_REF_OFFSET_DESCRIPTION = (
  "Rear reference offset (m). How much later turn-in is anchored in space. "
  + "0.35 matches the Model S camera-to-axle difference. Higher = later turn-in."
)


def lat_ref_offset_value(params) -> float:
  try:
    raw = params.get(NAP_LAT_REF_OFFSET, return_default=True)
    return LAT_REF_OFFSET_DEFAULT if raw is None else float(raw)
  except (TypeError, ValueError, AttributeError):
    return LAT_REF_OFFSET_DEFAULT


def lat_ref_offset_index(params) -> int:
  v = lat_ref_offset_value(params)
  return min(range(len(LAT_REF_OFFSET_PRESETS)), key=lambda i: abs(LAT_REF_OFFSET_PRESETS[i] - v))
