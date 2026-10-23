"""Closed-loop accel-error trim for lead approaches (Pre-AP).

Oct 23 16:52 CR 31: while closing on a semi the car carried +0.2 to
+0.4 m/s² more accel than the plan commanded (grade / pedal map). The
VirtualDAS inner integrator is too slow to remove that inside an approach,
so the gap closed on a -0.2 m/s² command.

This trim compares measured accel with the command the car should be
producing by now (delayed and lagged like the actuator), and integrates the
excess into a small, bounded, rate-limited negative offset on the lead
command. It only runs while a lead is being approached under long control.
Otherwise it decays to zero. It never adds accel.
"""
from collections import deque
import math

TRIM_MIN_MS2 = -0.50  # most the trim may take off the command
TRIM_KI = 0.50  # 1/s on the excess accel
TRIM_DEADBAND_MS2 = 0.08  # pedal / estimator noise
TRIM_RATE_MS3 = 0.25  # output slew
TRIM_DECAY_TAU_S = 1.5  # back to 0 when not approaching
TRIM_LEAK_TAU_S = 5.0  # slow leak while approaching, so noise cannot ratchet it down
ACT_DELAY_S = 0.40  # command to accel delay
ACT_TAU_S = 0.25  # actuator lag
MEAS_TAU_S = 0.50  # smooths aEgo and the expected accel alike
MIN_V_MS = 5.0
CLOSE_LO_MS = 0.2  # closing speed weight 0 -> 1 between these
CLOSE_HI_MS = 0.8
MAX_HEADWAY_S = 3.0  # an approach this far out is not trimmed
CMD_MAX_MS2 = 0.10  # only while coasting or braking toward the lead


def _smooth01(x: float) -> float:
  if x <= 0.0:
    return 0.0
  if x >= 1.0:
    return 1.0
  return x * x * (3.0 - 2.0 * x)


def _lp(prev: float, x: float, dt: float, tau: float) -> float:
  return prev + (x - prev) * min(1.0, dt / max(tau, 1e-3))


class ApproachAccelTrim:
  def __init__(self, dt: float):
    self.dt = float(dt)
    n = max(1, int(round(ACT_DELAY_S / self.dt)))
    self._hist: deque[float] = deque([0.0] * n, maxlen=n)
    self._a_act = 0.0
    self._a_exp = 0.0
    self._a_meas = 0.0
    self._integ = 0.0
    self.trim = 0.0
    self.error = 0.0
    self.active_w = 0.0
    self._init = False

  def reset(self, a_ego: float = 0.0) -> None:
    self._hist = deque([float(a_ego)] * self._hist.maxlen, maxlen=self._hist.maxlen)
    self._a_act = self._a_exp = self._a_meas = float(a_ego)
    self._integ = 0.0
    self.trim = 0.0
    self.error = 0.0
    self.active_w = 0.0
    self._init = True

  def update(self, *, a_ego: float, a_cmd: float, v_ego: float, v_lead: float,
             gap: float | None, long_on: bool, a_floor: float = -10.0) -> float:
    """`a_cmd` is the lead command before this trim. Returns the trim (<= 0)."""
    dt = self.dt
    a_ego = float(a_ego)
    if not all(math.isfinite(x) for x in (a_ego, float(a_cmd), float(v_ego), float(v_lead))):
      self.reset(0.0)
      return 0.0
    if not self._init or not long_on:
      # Driver or no long control: no memory of the old command.
      self.reset(a_ego)
      return 0.0

    # Expected accel: the command actually sent (trim included), delayed and lagged.
    delayed = self._hist[0]
    self._hist.append(float(a_cmd) + self.trim)
    self._a_act = _lp(self._a_act, delayed, dt, ACT_TAU_S)
    self._a_exp = _lp(self._a_exp, self._a_act, dt, MEAS_TAU_S)
    self._a_meas = _lp(self._a_meas, a_ego, dt, MEAS_TAU_S)
    self.error = self._a_meas - self._a_exp

    w = 0.0
    if gap is not None and float(v_ego) >= MIN_V_MS and float(a_cmd) <= CMD_MAX_MS2:
      v_close = float(v_ego) - float(v_lead)
      w_close = _smooth01((v_close - CLOSE_LO_MS) / (CLOSE_HI_MS - CLOSE_LO_MS))
      hw = float(gap) / max(float(v_ego), 1.0)
      w_near = _smooth01((MAX_HEADWAY_S - hw) / 0.5)
      w = w_close * w_near
    self.active_w = w

    if w > 0.0:
      e = self.error
      if e > TRIM_DEADBAND_MS2:
        # Anti-windup: no deeper once the trimmed command sits on the floor.
        if float(a_cmd) + self._integ > float(a_floor) + 0.05:
          self._integ -= TRIM_KI * w * (e - TRIM_DEADBAND_MS2) * dt
      elif e < -TRIM_DEADBAND_MS2:
        # The car is braking harder than asked: give the trim back.
        self._integ += TRIM_KI * (-e - TRIM_DEADBAND_MS2) * dt
      # Leak: a trim must be re-earned, a reflected random walk is not.
      self._integ *= 1.0 - min(1.0, dt / TRIM_LEAK_TAU_S)
      # Partial weight relaxes toward zero in proportion.
      self._integ *= 1.0 - (1.0 - w) * min(1.0, dt / TRIM_DECAY_TAU_S)
    else:
      self._integ *= 1.0 - min(1.0, dt / TRIM_DECAY_TAU_S)
    self._integ = min(0.0, max(TRIM_MIN_MS2, self._integ))

    step = TRIM_RATE_MS3 * dt
    self.trim = min(0.0, max(TRIM_MIN_MS2, self.trim + max(-step, min(step, self._integ - self.trim))))
    return self.trim
