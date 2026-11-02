"""controlsd hook: offset -> delta curvature, with every safety gate.

controlsd calls ConeAvoid.apply(...) once per loop. In Off and Log-only the
model curvature is returned unchanged; Log-only still computes and logs the
offset. Panda and the car interface are untouched: the existing curvature,
angle and torque limits downstream still govern.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from openpilot.selfdrive.nap.cone_avoid.config import (
  MODE_ACTIVE, MODE_LOG, MODE_OFF, PARAM_MAX_SHIFT, PARAM_MIN_CLEAR, PARAM_MODE, ConeAvoidConfig,
)
from openpilot.selfdrive.nap.cone_avoid.detector import fuse, obs_from_messages
from openpilot.selfdrive.nap.cone_avoid.planner import ConeAvoidPlanner, PlanOut

TORQUE_PUSH_NM = 0.55     # same push threshold the cone-line hold uses


def offset_to_curvature(offset_m: float, v_ego: float, k_cap: float) -> float:
  """Delta curvature that reaches offset_m at the look-ahead: 2*off/x^2."""
  x = min(max(2.5 * max(v_ego, 0.0), 10.0), 40.0)
  dk = 2.0 * offset_m / (x * x)
  return min(max(dk, -k_cap), k_cap)


@dataclass
class HookOut:
  mode: int = MODE_LOG
  offset_m: float = 0.0       # planned (log) offset, +right
  applied_m: float = 0.0      # offset actually applied (0 unless Active)
  dk: float = 0.0
  clear_m: float = 99.0
  take_over: bool = False
  low_conf: bool = False
  gated: str = ""


class ConeAvoid:
  def __init__(self, params=None, cfg: ConeAvoidConfig | None = None) -> None:
    self.params = params
    self.cfg = cfg or ConeAvoidConfig()
    self.planner = ConeAvoidPlanner(self.cfg)
    self.mode = MODE_LOG
    self.authority = 1.0
    self.dk = 0.0
    self.out = HookOut()
    self._frame = -1

  def _read_params(self) -> None:
    self._frame += 1
    if self.params is None or self._frame % 100:
      return
    try:
      raw = self.params.get(PARAM_MODE)
      self.mode = int(raw) if raw not in (None, b"", "") else MODE_LOG
      if self.mode not in (MODE_OFF, MODE_LOG, MODE_ACTIVE):
        self.mode = MODE_LOG
      for key, attr in ((PARAM_MAX_SHIFT, "max_shift_m"), (PARAM_MIN_CLEAR, "min_clear_m")):
        v = self.params.get(key)
        if v not in (None, b"", ""):
          f = float(v)
          if math.isfinite(f):
            setattr(self.cfg, attr, f)
      self.cfg.clamp()
    except Exception:
      self.mode = MODE_LOG

  def step(self, model_k: float, *, cones, v_ego: float, lat_active: bool, blinker: bool,
           lane_change: bool, steering_pressed: bool, torque_nm: float, dt: float) -> float:
    """Pure core (no messaging), used by apply() and the tests."""
    cfg = self.cfg
    plan: PlanOut = self.planner.update(cones, v_ego, dt)
    out = HookOut(mode=self.mode, offset_m=plan.offset_m, clear_m=plan.clear_m,
                  take_over=plan.take_over, low_conf=plan.low_conf)
    pushed = steering_pressed or abs(torque_nm) >= TORQUE_PUSH_NM
    if pushed:
      self.authority = 0.0                                  # cancel at once
    else:
      self.authority = min(1.0, self.authority + dt / cfg.fade_in_s)  # fade back in
    gate = ""
    if self.mode != MODE_ACTIVE:
      gate = "log_only" if self.mode == MODE_LOG else "off"
    elif not lat_active:
      gate = "lat_off"
    elif blinker or lane_change:
      gate = "blinker"
    elif pushed:
      gate = "driver"
    want = 0.0 if gate else plan.offset_m * self.authority
    out.gated = gate
    if gate:
      # Log-only records the curvature it would have added; nothing is applied.
      out.dk = offset_to_curvature(plan.offset_m, v_ego, cfg.k_cap) if gate == "log_only" else 0.0
      self.dk = 0.0
      self.out = out
      return float(model_k)
    dk_t = offset_to_curvature(want, v_ego, cfg.k_cap)
    step = cfg.k_rate * dt
    self.dk += min(max(dk_t - self.dk, -step), step)
    out.applied_m, out.dk = want, self.dk
    self.out = out
    return float(model_k) + self.dk

  def apply(self, model_k: float, CS, sm, *, lat_active: bool, lane_change: bool, dt: float) -> float:
    """The controlsd entry point. Never raises; any error returns model_k."""
    try:
      self._read_params()
      if self.mode == MODE_OFF:
        self.planner.reset()
        self.out = HookOut(mode=MODE_OFF, gated="off")
        return float(model_k)
      now = sm.logMonoTime["carState"]
      cone = vis = None
      ca = va = 1e9
      if sm.seen.get("coneLineNAP", False):
        cone, ca = sm["coneLineNAP"], (now - sm.logMonoTime["coneLineNAP"]) * 1e-9
      if sm.seen.get("pathObstacleVisionNAP", False):
        vis, va = sm["pathObstacleVisionNAP"], (now - sm.logMonoTime["pathObstacleVisionNAP"]) * 1e-9
      cones = fuse(obs_from_messages(cone, vis, ca, va), self.cfg.stale_s)
      return self.step(model_k, cones=cones, v_ego=float(CS.vEgo), lat_active=lat_active,
                       blinker=bool(CS.leftBlinker or CS.rightBlinker), lane_change=lane_change,
                       steering_pressed=bool(CS.steeringPressed), torque_nm=float(CS.steeringTorque), dt=dt)
    except Exception:
      self.out = HookOut(mode=self.mode, gated="error")
      return float(model_k)
