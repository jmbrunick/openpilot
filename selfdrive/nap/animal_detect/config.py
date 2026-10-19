"""Animal/person detector constants. Log only: nothing here alerts or brakes."""
PARAM_ENABLE = "NAPAnimalDetect"

# Model classes, in output order.
CLASSES = ("person", "deer", "largeAnimal", "smallAnimal")
ANIMAL_CLASSES = frozenset({"deer", "largeAnimal", "smallAnimal"})
NUM_CLASSES = len(CLASSES)

# One model input tile, rows x cols. Road tiles are cut at native resolution.
TILE_H, TILE_W = 320, 640
# Rows of the road tile above / below the horizon row.
TILE_ABOVE_HORIZON = 110
# Wide camera: whole frame scaled to TILE_W columns, TILE_H rows around its horizon.
WIDE_SCALE_W = TILE_W

RATE_HZ = 5.0
# Little cores only. Never 4 (controlsd/selfdrived) or 6 (card).
CORES = (0, 1, 2, 3)
NICE = 19

# Scores. Detections under DET_CONF are dropped before tracking.
DET_CONF = 0.30
NMS_IOU = 0.45
MAX_DETECTIONS = 12

# Radar match gates (scope §3).
MATCH_BEARING_DEG = 1.5
MATCH_RANGE_FRAC = 0.25
# Camera-only track: confirmed after this many hits in a row (one miss allowed).
CAMERA_CONFIRM_FRAMES = 3
TRACK_MAX_MISSES = 1
TRACK_MATCH_IOU = 0.2

# Radar sits ~1.52 m ahead of the camera (radard RADAR_TO_CAMERA).
RADAR_TO_CAMERA = 1.52

# Back off when modeld drops frames, so the driving model always wins the GPU.
DROP_BACKOFF_S = 1.0
DROP_BACKOFF_MAX_S = 10.0
# Back off after a model error. Errors never escape the loop.
ERROR_BACKOFF_S = 2.0
ERROR_BACKOFF_MAX_S = 30.0
