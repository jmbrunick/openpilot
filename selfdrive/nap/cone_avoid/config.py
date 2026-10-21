"""Tunables. Max shift and min clearance are params so they can be raised later."""
from dataclasses import dataclass

PARAM_MODE = "NAPConeAvoid"               # "0" Off, "1" Log-only (default), "2" Active
PARAM_MAX_SHIFT = "NAPConeAvoidMaxShiftM"  # default 1.0
PARAM_MIN_CLEAR = "NAPConeAvoidMinClearM"  # default 1.0

MODE_OFF, MODE_LOG, MODE_ACTIVE = 0, 1, 2

# Hard bounds on what the params may request (a typo can't ask for 5 m).
MAX_SHIFT_BOUNDS = (0.0, 2.0)
MIN_CLEAR_BOUNDS = (0.5, 2.0)


@dataclass
class ConeAvoidConfig:
  max_shift_m: float = 1.0       # |offset| cap
  min_clear_m: float = 1.0       # target gap, cone to car edge
  hard_clear_m: float = 0.5      # below this even after shifting: take over, no steer
  car_half_m: float = 0.95       # Model S half width incl. mirrors margin
  rate_mps: float = 0.25         # offset rate limit (out and back)
  low_speed_mps: float = 6.7     # below ~15 mph ...
  low_speed_cap_m: float = 0.6   # ... cap the shift lower
  horizon_s: float = 4.0         # look-ahead window in time
  horizon_min_m: float = 15.0
  horizon_max_m: float = 70.0
  behind_m: float = -2.0         # keep a cone until it is beside/behind the car
  lane_lat_m: float = 3.0        # cones farther than this off the path are ignored
  act_conf: float = 0.5          # Active mode steers only on cones at/above this
  stale_s: float = 0.3           # cone data older than this is ignored
  fade_in_s: float = 1.5         # after a wheel push
  k_cap: float = 0.02            # |delta curvature| cap, 1/m
  k_rate: float = 0.02           # |delta curvature| rate, 1/m per s

  def clamp(self) -> "ConeAvoidConfig":
    self.max_shift_m = min(max(self.max_shift_m, MAX_SHIFT_BOUNDS[0]), MAX_SHIFT_BOUNDS[1])
    self.min_clear_m = min(max(self.min_clear_m, MIN_CLEAR_BOUNDS[0]), MIN_CLEAR_BOUNDS[1])
    return self
