"""Pre-AP driver-brake switch, separate from brakePressed.

brakePressed stays false so a brake press does not take the generic
disengage path and drop lateral. driverBrakeApplied (with the CS.brake
float as a fallback) is the digital switch. Longitudinal pauses; lateral
stays. A stalk pull resumes long.
"""
from openpilot.common.realtime import DT_CTRL

PREAP_FINGERPRINT = "TESLA_MODEL_S_PREAP"
# A normal pedal RELEASE lands on the next 50 Hz command frame. Hold past
# this before calling it a regression.
BRAKE_LONG_OVERLAP_S = 0.1


def preap_pedal_long(CP) -> bool:
  """Pedal-mode Pre-AP: openpilot long, software cruise, lateral stays on brake."""
  return (
    getattr(CP, "brand", None) == "tesla"
    and getattr(CP, "carFingerprint", None) == PREAP_FINGERPRINT
    and bool(getattr(CP, "openpilotLongitudinalControl", False))
    and not bool(getattr(CP, "pcmCruise", True))
  )


def driver_brake_applied(CS) -> bool:
  """True brake-switch state. Ignores the always-false Pre-AP brakePressed."""
  if bool(getattr(CS, "driverBrakeApplied", False)):
    return True
  try:
    return float(getattr(CS, "brake", 0.0) or 0.0) >= 0.5
  except (TypeError, ValueError):
    return False


def preap_longitudinal_active(*, enabled: bool, longitudinal_override: bool,
                              openpilot_longitudinal: bool, preap_pedal: bool,
                              enable_long_control: bool, driver_brake: bool) -> bool:
  """carControl.longActive.

  Stock formula for every other car. On Pre-AP pedal long, also false while
  the driver brake is down or long is paused (enableLongControl false), so
  the log does not show an active decel the pedal is not executing.
  """
  active = bool(enabled) and not bool(longitudinal_override) and bool(openpilot_longitudinal)
  if preap_pedal:
    active = active and bool(enable_long_control) and not bool(driver_brake)
  return active


def published_long_accel(*, long_active: bool, loc_accel: float, preap_pedal: bool) -> float:
  """Accel stored on carControl. A Pre-AP long pause logs 0, not the plan."""
  if preap_pedal and not bool(long_active):
    return 0.0
  return float(loc_accel)


def brake_signal_disables(*, preap_pedal: bool, driver_brake: bool, prev_driver_brake: bool,
                          brake_pressed: bool, prev_brake_pressed: bool,
                          regen_braking: bool, prev_regen_braking: bool,
                          standstill: bool) -> bool:
  """Rising brake, or brake while moving, or the same for regen.

  Pre-AP pedal mode reads driverBrakeApplied. Other cars keep brakePressed.
  The caller still maps a Pre-AP hit to a long-only override, not a session cancel.
  """
  if preap_pedal:
    brake = bool(driver_brake)
    prev_brake = bool(prev_driver_brake)
  else:
    brake = bool(brake_pressed)
    prev_brake = bool(prev_brake_pressed)
  moving_or_rising = (not standstill)
  return (
    (brake and (not prev_brake or moving_or_rising))
    or (bool(regen_braking) and (not bool(prev_regen_braking) or moving_or_rising))
  )


class BrakeLongOverlap:
  """Latches once the brake switch and pedal-long authority overlap too long."""

  def __init__(self, limit_s: float = BRAKE_LONG_OVERLAP_S):
    self.limit_s = float(limit_s)
    self.held_s = 0.0
    self._latched = False

  def reset(self) -> None:
    self.held_s = 0.0
    self._latched = False

  def update(self, *, driver_brake: bool, pedal_long_active: bool,
             dt: float = DT_CTRL) -> tuple[bool, bool]:
    """Returns (fault, rising). Fault means the overlap has lasted more than limit_s."""
    if bool(driver_brake) and bool(pedal_long_active):
      self.held_s += float(dt)
    else:
      self.reset()
    fault = self.held_s > self.limit_s
    rising = fault and not self._latched
    if fault:
      self._latched = True
    return fault, rising
