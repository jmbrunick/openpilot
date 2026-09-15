from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.locationd.helpers import Measurement, Pose
from openpilot.selfdrive.selfdrived.helpers import (
  EXCESSIVE_LAT_ACCEL,
  EXCESSIVE_LONG_ACCEL_NEG,
  EXCESSIVE_LONG_ACCEL_POS,
  MIN_EXCESSIVE_ACTUATION_COUNT,
  MIN_LATERAL_ENGAGE_BUFFER,
  PREAP_EXCESSIVE_LAT_ACCEL,
  PREAP_EXCESSIVE_LONG_ACCEL_NEG,
  PREAP_EXCESSIVE_LONG_ACCEL_POS,
  PREAP_FINGERPRINT,
  ExcessiveActuationCheck,
  ExcessiveActuationType,
  excessive_actuation_limits,
)

# Faster-but-legal ramp: ~0.71 g. Stock 2× ISO (6.0) false-trips this;
# Pre-AP 3× ISO (9.0) must not.
FAST_NORMAL_CORNER_MPS2 = 7.0
# Panic swerve / emergency: ~1.12 g. Both stock and Pre-AP still trip.
EXTREME_SWERVE_MPS2 = 11.0
# Firm but not panic long decel: between stock −7.0 and Pre-AP −10.5.
FAST_NORMAL_DECEL_MPS2 = -8.0
EXTREME_DECEL_MPS2 = -12.0

V_EGO = 25.0  # m/s, ~56 mph


def _pose(*, ax=0.0, yaw_rate=0.0) -> Pose:
  z = np.zeros(3)
  acc = Measurement(np.array([ax, 0.0, 0.0]), z)
  ang = Measurement(np.array([0.0, 0.0, yaw_rate]), z)
  return Pose(Measurement(z, z), Measurement(z, z), acc, ang)


def _sm(*, lat_active=True, long_active=False, roll=0.0):
  return {
    "carControl": SimpleNamespace(latActive=lat_active, longActive=long_active),
    "liveParameters": SimpleNamespace(roll=roll),
  }


def _cs(*, v_ego=V_EGO, a_ego=0.0, steering_pressed=False):
  return SimpleNamespace(vEgo=v_ego, aEgo=a_ego, steeringPressed=steering_pressed)


def _run(check, *, frames, lat_mps2=0.0, ax=0.0, lat_active=True, long_active=False,
         steering_pressed=False):
  yaw = 0.0 if V_EGO == 0 else lat_mps2 / V_EGO
  sm = _sm(lat_active=lat_active, long_active=long_active)
  cs = _cs(a_ego=ax, steering_pressed=steering_pressed)
  pose = _pose(ax=ax, yaw_rate=yaw)
  last = None
  for _ in range(frames):
    last = check.update(sm, cs, pose)
  return last


def _lat_frames_to_trip():
  return MIN_LATERAL_ENGAGE_BUFFER + MIN_EXCESSIVE_ACTUATION_COUNT + 2


def _long_frames_to_trip():
  return MIN_EXCESSIVE_ACTUATION_COUNT + 2


def test_threshold_constants_document_old_vs_new():
  assert EXCESSIVE_LAT_ACCEL == 6.0
  assert PREAP_EXCESSIVE_LAT_ACCEL == 9.0
  assert EXCESSIVE_LONG_ACCEL_POS == 4.0
  assert PREAP_EXCESSIVE_LONG_ACCEL_POS == 6.0
  assert EXCESSIVE_LONG_ACCEL_NEG == -7.0
  assert PREAP_EXCESSIVE_LONG_ACCEL_NEG == -10.5
  stock = excessive_actuation_limits("comma")
  preap = excessive_actuation_limits(PREAP_FINGERPRINT)
  assert stock == (6.0, 4.0, -7.0)
  assert preap == (9.0, 6.0, -10.5)
  # Pre-AP is a fair amount looser (50% on the ISO multiple: 2× → 3×).
  assert PREAP_EXCESSIVE_LAT_ACCEL / EXCESSIVE_LAT_ACCEL == 1.5
  assert FAST_NORMAL_CORNER_MPS2 > EXCESSIVE_LAT_ACCEL
  assert FAST_NORMAL_CORNER_MPS2 < PREAP_EXCESSIVE_LAT_ACCEL
  assert EXTREME_SWERVE_MPS2 > PREAP_EXCESSIVE_LAT_ACCEL
  assert FAST_NORMAL_DECEL_MPS2 < EXCESSIVE_LONG_ACCEL_NEG
  assert FAST_NORMAL_DECEL_MPS2 > PREAP_EXCESSIVE_LONG_ACCEL_NEG
  assert EXTREME_DECEL_MPS2 < PREAP_EXCESSIVE_LONG_ACCEL_NEG


def test_preap_fast_normal_corner_does_not_trip():
  check = ExcessiveActuationCheck(fingerprint=PREAP_FINGERPRINT)
  assert _run(check, frames=_lat_frames_to_trip(), lat_mps2=FAST_NORMAL_CORNER_MPS2) is None


def test_stock_fast_normal_corner_still_trips():
  check = ExcessiveActuationCheck(fingerprint="honda")
  assert _run(check, frames=_lat_frames_to_trip(),
              lat_mps2=FAST_NORMAL_CORNER_MPS2) == ExcessiveActuationType.LATERAL


def test_preap_extreme_swerve_still_trips():
  check = ExcessiveActuationCheck(fingerprint=PREAP_FINGERPRINT)
  assert _run(check, frames=_lat_frames_to_trip(),
              lat_mps2=EXTREME_SWERVE_MPS2) == ExcessiveActuationType.LATERAL


def test_preap_fast_normal_decel_does_not_trip():
  check = ExcessiveActuationCheck(fingerprint=PREAP_FINGERPRINT)
  assert _run(check, frames=_long_frames_to_trip(), ax=FAST_NORMAL_DECEL_MPS2,
              lat_active=False, long_active=True) is None


def test_stock_fast_normal_decel_still_trips():
  check = ExcessiveActuationCheck()
  assert _run(check, frames=_long_frames_to_trip(), ax=FAST_NORMAL_DECEL_MPS2,
              lat_active=False, long_active=True) == ExcessiveActuationType.LONGITUDINAL


def test_preap_extreme_decel_still_trips():
  check = ExcessiveActuationCheck(fingerprint=PREAP_FINGERPRINT)
  assert _run(check, frames=_long_frames_to_trip(), ax=EXTREME_DECEL_MPS2,
              lat_active=False, long_active=True) == ExcessiveActuationType.LONGITUDINAL


def test_preap_path_is_not_disabled():
  """Raising the limit is not the same as turning the check off."""
  check = ExcessiveActuationCheck(fingerprint=PREAP_FINGERPRINT)
  assert check._lat_limit < 20.0
  assert check._long_neg > -20.0
  assert _run(check, frames=_lat_frames_to_trip(),
              lat_mps2=EXTREME_SWERVE_MPS2) is not None
