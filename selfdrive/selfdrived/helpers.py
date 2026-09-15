from __future__ import annotations

import math
from enum import StrEnum, auto
from typing import TYPE_CHECKING

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, DT_CTRL
from opendbc.car.lateral import ISO_LATERAL_ACCEL
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX

if TYPE_CHECKING:
  from cereal import car, messaging
  from openpilot.selfdrive.locationd.helpers import Pose

MIN_EXCESSIVE_ACTUATION_COUNT = int(0.25 / DT_CTRL)
MIN_LATERAL_ENGAGE_BUFFER = int(1 / DT_CTRL)

PREAP_FINGERPRINT = "TESLA_MODEL_S_PREAP"

# Stock comma: 2× ISO 11270 (6.0 m/s², ~0.61 g) and 2× planner accel clip
# (4.0 / −7.0 m/s²). That is meant to catch a true emergency actuation, but
# on Pre-AP Model S it false-trips a faster-but-legal ramp or firm decel
# ("excessive" / emergency-maneuver knockoff).
EXCESSIVE_LAT_ACCEL = ISO_LATERAL_ACCEL * 2.0          # 6.0 m/s²
EXCESSIVE_LONG_ACCEL_POS = ACCEL_MAX * 2.0             # 4.0 m/s²
EXCESSIVE_LONG_ACCEL_NEG = ACCEL_MIN * 2.0             # -7.0 m/s²

# Pre-AP / NAP: raise a fair amount (3×) so ~0.5–0.75 g corners stay in.
# 9.0 m/s² (~0.92 g) / 6.0 / −10.5 still trip a real emergency swerve or
# panic stop. Do not disable this path; locationd / camera soft-disables
# are unchanged.
PREAP_EXCESSIVE_LAT_ACCEL = ISO_LATERAL_ACCEL * 3.0    # 9.0 m/s²
PREAP_EXCESSIVE_LONG_ACCEL_POS = ACCEL_MAX * 3.0       # 6.0 m/s²
PREAP_EXCESSIVE_LONG_ACCEL_NEG = ACCEL_MIN * 3.0       # -10.5 m/s²


def excessive_actuation_limits(fingerprint: str = "") -> tuple[float, float, float]:
  """Return (lat, long_pos, long_neg) thresholds for this car."""
  if fingerprint == PREAP_FINGERPRINT:
    return (PREAP_EXCESSIVE_LAT_ACCEL,
            PREAP_EXCESSIVE_LONG_ACCEL_POS,
            PREAP_EXCESSIVE_LONG_ACCEL_NEG)
  return (EXCESSIVE_LAT_ACCEL, EXCESSIVE_LONG_ACCEL_POS, EXCESSIVE_LONG_ACCEL_NEG)


class ExcessiveActuationType(StrEnum):
  LONGITUDINAL = auto()
  LATERAL = auto()


class ExcessiveActuationCheck:
  def __init__(self, fingerprint: str = ""):
    self._excessive_counter = 0
    self._engaged_counter = 0
    self._lat_limit, self._long_pos, self._long_neg = excessive_actuation_limits(fingerprint)

  def update(self, sm: messaging.SubMaster, CS: car.CarState, calibrated_pose: Pose) -> ExcessiveActuationType | None:
    # CS.aEgo can be noisy to bumps in the road, transitioning from standstill, losing traction, etc.
    # longitudinal
    accel_calibrated = calibrated_pose.acceleration.x
    excessive_long_actuation = sm['carControl'].longActive and (accel_calibrated > self._long_pos or accel_calibrated < self._long_neg)

    # lateral
    yaw_rate = calibrated_pose.angular_velocity.yaw
    roll = sm['liveParameters'].roll
    roll_compensated_lateral_accel = (CS.vEgo * yaw_rate) - (math.sin(roll) * ACCELERATION_DUE_TO_GRAVITY)

    # Prevent false positives after overriding
    excessive_lat_actuation = False
    self._engaged_counter = self._engaged_counter + 1 if sm['carControl'].latActive and not CS.steeringPressed else 0
    if self._engaged_counter > MIN_LATERAL_ENGAGE_BUFFER:
      if abs(roll_compensated_lateral_accel) > self._lat_limit:
        excessive_lat_actuation = True

    # livePose acceleration can be noisy due to bad mounting or aliased livePose measurements
    livepose_valid = abs(CS.aEgo - accel_calibrated) < 2
    self._excessive_counter = self._excessive_counter + 1 if livepose_valid and (excessive_long_actuation or excessive_lat_actuation) else 0

    excessive_type = None
    if self._excessive_counter > MIN_EXCESSIVE_ACTUATION_COUNT:
      if excessive_long_actuation:
        excessive_type = ExcessiveActuationType.LONGITUDINAL
      else:
        excessive_type = ExcessiveActuationType.LATERAL

    return excessive_type


def _gear_is_drive(gear) -> bool:
  name = getattr(gear, "name", gear)
  return name == "drive" or gear == "drive"


def preap_leave_drive_clears_mismatch(*, fingerprint: str, gear, gear_prev) -> bool:
  """True on the Drive → R/P/other falling edge (Pre-AP only).

  Clear the 3X/OP controls-mismatch counters on leave-Drive, same
  cleanup spirit as stalk cancel. Do not clear on Drive entry: later
  engagement in Drive uses the normal live panda/selfdrived checks.
  """
  if fingerprint != PREAP_FINGERPRINT:
    return False
  return _gear_is_drive(gear_prev) and (not _gear_is_drive(gear))
