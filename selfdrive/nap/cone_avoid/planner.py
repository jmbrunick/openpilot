"""Lateral offset from individual cones. Pure, no messaging.

Clearance is computed per cone (every cone in the path horizon), never from a
fitted line, so a single cone 0.6 m into the lane drives the shift by itself.
The offset is rate-limited both out and back to the row line.
Sign: offset m, +right. Cone y path-relative, +right.
"""
from __future__ import annotations

from dataclasses import dataclass

from openpilot.selfdrive.nap.cone_avoid.config import ConeAvoidConfig
from openpilot.selfdrive.nap.cone_avoid.detector import Cone


@dataclass
class PlanOut:
  target_m: float = 0.0       # desired offset before rate limit (after cap / take-over)
  offset_m: float = 0.0       # rate-limited offset
  request_m: float = 0.0      # uncapped offset the cones ask for
  clear_m: float = 99.0       # smallest cone-to-car-edge gap at offset_m target
  n_cones: int = 0
  low_conf: bool = False      # only low-confidence (radar ghost) cones drove the request
  take_over: bool = False
  corridor: bool = False


def horizon_m(v_ego: float, cfg: ConeAvoidConfig) -> float:
  return min(max(v_ego * cfg.horizon_s, cfg.horizon_min_m), cfg.horizon_max_m)


def _request(cones: list[Cone], cfg: ConeAvoidConfig) -> tuple[float, bool, bool]:
  """Uncapped offset and whether it is feasible / a corridor."""
  need = cfg.min_clear_m + cfg.car_half_m
  lo, hi = -1e9, 1e9          # offset must be >= lo (left cones) and <= hi (right cones)
  near_l = near_r = None
  for c in cones:
    if c.y <= 0.0:            # left of (or dead on) the path: pass it on the right
      lo = max(lo, c.y + need)
      near_l = c.y if near_l is None else max(near_l, c.y)
    else:
      hi = min(hi, c.y - need)
      near_r = c.y if near_r is None else min(near_r, c.y)
  corridor = near_l is not None and near_r is not None
  if corridor:
    mid = 0.5 * (near_l + near_r)
    return (min(max(mid, lo), hi) if lo <= hi else mid), lo <= hi, True
  if near_l is not None:
    return max(0.0, lo), True, False
  if near_r is not None:
    return min(0.0, hi), True, False
  return 0.0, True, False


def _clear(cones: list[Cone], off: float, cfg: ConeAvoidConfig) -> float:
  return min((abs(c.y - off) - cfg.car_half_m for c in cones), default=99.0)


class ConeAvoidPlanner:
  def __init__(self, cfg: ConeAvoidConfig | None = None) -> None:
    self.cfg = cfg or ConeAvoidConfig()
    self.offset = 0.0

  def reset(self) -> None:
    self.offset = 0.0

  def update(self, cones: list[Cone], v_ego: float, dt: float, *, steer_on_low_conf: bool = False) -> PlanOut:
    cfg = self.cfg
    hz = horizon_m(v_ego, cfg)
    near = [c for c in cones if cfg.behind_m <= c.x <= hz and abs(c.y) <= cfg.lane_lat_m]
    trusted = [c for c in near if not c.low_conf and c.conf >= cfg.act_conf]
    use = near if steer_on_low_conf else trusted
    out = PlanOut(n_cones=len(near), low_conf=bool(near) and not trusted)
    if not use and near:
      # Log what the ghost would ask for, but never act on it.
      out.request_m = _request(near, cfg)[0]
    req, feasible, corridor = _request(use, cfg)
    if use:
      out.request_m = req
    out.corridor = corridor
    cap = cfg.max_shift_m
    if v_ego < cfg.low_speed_mps:
      cap = min(cap, cfg.low_speed_cap_m)
    target = min(max(req, -cap), cap)
    clear_after = _clear(use, target, cfg)
    over_cap = abs(req) > cap + 1e-6
    if use and (not feasible or over_cap or clear_after < cfg.min_clear_m - 1e-6):
      out.take_over = True
    if use and clear_after < cfg.hard_clear_m:
      target = 0.0  # not enough room even shifted: alert, don't steer somewhere worse
    out.target_m = target
    out.clear_m = _clear(use, target, cfg) if use else _clear(near, target, cfg)
    step = cfg.rate_mps * dt
    self.offset += min(max(target - self.offset, -step), step)
    out.offset_m = self.offset
    return out
