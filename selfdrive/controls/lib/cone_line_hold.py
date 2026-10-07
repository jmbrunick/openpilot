"""Hold the driver's line on a soft-handoff re-take near a cone line.

Acts only after the driver has already pushed away from the cones. While
lateral is yielded this does not steer and does not change the yield.
On the re-take the curvature target is the driver's line (their offset
from the model path) instead of the model path that bends back toward
the cones. The offset eases off after the line has been gone for a few
seconds, or if the driver steers back toward it.

Never commands a path closer to the cone line than the line the driver
was holding. Longitudinal is not an input or an output.

Thresholds:
  push          >= 0.55 Nm away from the cones for 2.0 s (a 0.40 s gap resets)
  model pull    path is >= 0.12 m toward the cone line at the lookahead
  lookahead     vEgo * 1.6 s, clamped to 12–40 m, frozen when the line is captured
  min offset    0.15 m; shifts larger than 1.5 m are clipped
  cone clear    line inactive for 2.5 s, then the offset decays over 2.5 s
  steer back    >= 0.55 Nm toward the cones for 0.30 s starts that same decay
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.radar_path_gate import path_y_at_x

PARAM_CONE_LINE_HOLD = "NAPConeLineHold"
PARAM_CONE_LINE_LOG = "NAPConeLineLog"

PUSH_ARM_S = 2.0
PUSH_NM = 0.55
PUSH_GAP_S = 0.40
PULL_Y_M = 0.12
PULL_K = 0.0005
MIN_OFFSET_M = 0.15
MAX_OFFSET_M = 1.5
MAX_CURVATURE_DELTA = 0.02
CLEAR_S = 2.5
RELEASE_S = 2.5
STEER_BACK_NM = 0.55
STEER_BACK_S = 0.30
X_LA_S = 1.6
X_LA_MIN_M = 12.0
X_LA_MAX_M = 40.0
K_FILTER_S = 0.30


def lookahead_m(v_ego: float) -> float:
  try:
    v = float(v_ego)
  except (TypeError, ValueError):
    v = 0.0
  if not math.isfinite(v):
    v = 0.0
  return min(X_LA_MAX_M, max(X_LA_MIN_M, abs(v) * X_LA_S))


def _smoothstep(t: float) -> float:
  t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
  return t * t * (3.0 - 2.0 * t)


def _finite(value, default: float = 0.0) -> float:
  try:
    out = float(value)
  except (TypeError, ValueError):
    return default
  return out if math.isfinite(out) else default


@dataclass(frozen=True)
class ConeHoldOutput:
  active: bool
  offset_m: float  # applied path shift, +left; 0 when not steering it
  curvature: float


class ConeLineHold:
  def __init__(self) -> None:
    self._reset()

  def _reset(self) -> None:
    self._push_s = 0.0
    self._push_gap_s = 0.0
    self._armed = False
    self._saw_yield = False
    self._k_driver = 0.0
    self._x_la = X_LA_MIN_M
    self._side = 0
    self._gain = 0.0
    self._clear_s = 0.0
    self._release_s = 0.0
    self._releasing = False
    self._steer_back_s = 0.0
    self._resume_push_s = 0.0
    self.busy = False

  def reset(self) -> None:
    self._reset()

  def update(self, *, enabled: bool, engaged: bool, cone, torque_nm: float,
             measured_k: float, model_k: float, path_x, path_y, v_ego: float,
             yielded: bool, lat_active: bool, dt: float) -> ConeHoldOutput:
    model_k = _finite(model_k)
    if not enabled or not engaged:
      self._reset()
      return ConeHoldOutput(False, 0.0, model_k)

    dt = _finite(dt)
    if dt < 0.0:
      dt = 0.0
    dt = min(dt, 0.2)
    torque = _finite(torque_nm)
    measured = _finite(measured_k)
    side = int(getattr(cone, "side", 0) or 0) if cone is not None else 0
    cone_active = bool(getattr(cone, "active", False)) and side in (-1, 1)

    x_now = lookahead_m(v_ego)
    y_model = path_y_at_x(path_x, path_y, self._x_la if self._armed else x_now)
    pulls = self._pulls_toward(y_model, model_k, side if cone_active else 0)
    pushing = self._pushing_away(torque, side) if cone_active and pulls else False

    if yielded and (pushing or self._armed):
      self._saw_yield = True

    if pushing:
      self._push_gap_s = 0.0
      self._push_s += dt
    elif not self._armed:
      self._push_gap_s += dt
      if self._push_gap_s > PUSH_GAP_S:
        self._push_s = 0.0

    if cone_active and not self._armed and self._push_s + 1e-12 >= PUSH_ARM_S and self._saw_yield:
      away = self._away(measured, y_model, x_now, side)
      if away >= MIN_OFFSET_M:
        self._armed = True
        self._side = side
        self._x_la = x_now
        self._k_driver = measured
        self._gain = 1.0
        self._releasing = False
        self._release_s = 0.0
        self._clear_s = 0.0

    if self._armed and pushing and cone_active and side == self._side:
      self._follow_further_away(measured, y_model, dt)

    line_here = cone_active and side == self._side
    if self._armed and line_here:
      self._clear_s = 0.0
    elif self._armed:
      self._clear_s += dt

    self._update_release(torque, line_here, pushing, dt)

    self.busy = self._armed
    apply = (
      self._armed
      and self._saw_yield
      and not yielded
      and lat_active
      and self._gain > 1e-3
    )
    if not apply:
      if self._armed and self._gain <= 1e-3:
        self._reset()
      return ConeHoldOutput(False, 0.0, model_k)

    y_model_cmd = path_y_at_x(path_x, path_y, self._x_la)
    if y_model_cmd is None:
      return ConeHoldOutput(False, 0.0, model_k)
    y_driver = 0.5 * self._k_driver * self._x_la * self._x_la
    delta = self._capped_delta(y_driver, y_model_cmd)
    # Lerp from the model path (gain 0) to the driver's line (gain 1).
    # While the line is still there and the driver has not steered back,
    # do not slide past their line toward the cones.
    y_hold = y_model_cmd + delta
    y_cmd = y_model_cmd + self._gain * delta
    if line_here and not self._releasing and self._side * (y_cmd - y_hold) > 0.02:
      y_cmd = y_hold
    shift = y_cmd - y_model_cmd
    delta_k = 2.0 * shift / (self._x_la * self._x_la)
    if delta_k > MAX_CURVATURE_DELTA:
      delta_k = MAX_CURVATURE_DELTA
    elif delta_k < -MAX_CURVATURE_DELTA:
      delta_k = -MAX_CURVATURE_DELTA
    return ConeHoldOutput(True, shift, model_k + delta_k)

  def _pulls_toward(self, y_model, model_k: float, side: int) -> bool:
    if side not in (-1, 1) or y_model is None:
      return False
    return side * y_model > PULL_Y_M or side * model_k > PULL_K

  def _pushing_away(self, torque: float, side: int) -> bool:
    # steeringTorque is +left. Away from a right-side line (side -1) is +torque.
    return side in (-1, 1) and torque * side < 0.0 and abs(torque) >= PUSH_NM

  def _toward_cones(self, torque: float) -> bool:
    return self._side in (-1, 1) and torque * self._side > 0.0 and abs(torque) >= STEER_BACK_NM

  def _away(self, measured_k: float, y_model, x_la: float, side: int) -> float:
    if y_model is None or side not in (-1, 1):
      return 0.0
    y_driver = 0.5 * measured_k * x_la * x_la
    return (y_driver - y_model) * (-side)

  def _follow_further_away(self, measured: float, y_model, dt: float) -> None:
    """Keep the captured line if the driver moves further from the cones."""
    if y_model is None:
      return
    proposed = measured
    if self._k_driver != 0.0 or self._push_s > 0.0:
      alpha = dt / (K_FILTER_S + dt) if dt > 0.0 else 1.0
      proposed = self._k_driver + alpha * (measured - self._k_driver)
    if self._away(proposed, y_model, self._x_la, self._side) + 1e-6 >= self._away(
        self._k_driver, y_model, self._x_la, self._side):
      self._k_driver = proposed

  def _capped_delta(self, y_driver: float, y_model: float) -> float:
    delta = y_driver - y_model
    away_sign = -self._side
    away = delta * away_sign
    if away > MAX_OFFSET_M:
      return away_sign * MAX_OFFSET_M
    if away < 0.0:
      # Driver line is already toward the cones relative to the model.
      # Do not add a shift that closes the gap.
      return 0.0
    return delta

  def _update_release(self, torque: float, line_here: bool, pushing: bool, dt: float) -> None:
    if not self._armed:
      return
    if self._toward_cones(torque):
      self._steer_back_s += dt
    else:
      self._steer_back_s = 0.0
    if self._steer_back_s + 1e-12 >= STEER_BACK_S or self._clear_s + 1e-12 >= CLEAR_S:
      self._releasing = True
    if pushing and self._releasing:
      self._resume_push_s += dt
      if self._resume_push_s + 1e-12 >= STEER_BACK_S:
        self._releasing = False
        self._release_s = 0.0
        self._gain = 1.0
        self._steer_back_s = 0.0
        self._clear_s = 0.0
        self._resume_push_s = 0.0
    else:
      self._resume_push_s = 0.0
    if not self._releasing:
      if line_here:
        self._gain = 1.0
      return
    self._release_s += dt
    self._gain = 1.0 - _smoothstep(self._release_s / RELEASE_S)
    if self._gain <= 1e-3:
      self._gain = 0.0
