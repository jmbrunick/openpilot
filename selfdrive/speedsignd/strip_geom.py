"""Calibration-placed proposer strips (pure, no messaging).

The proposer input is a fixed 352x128 (2.75:1), trained on the 964x350
lower-right strip at a 2.74x downscale. Two areas keep that scale:

  main   964x350, the legacy strip, shifted vertically so its horizon line
         follows liveCalibration (clamped to +-HORIZON_MAX_DY px).
  upper  482x175 directly above main (top ~y350 at nominal calibration),
         placed at the right road edge projected from modelV2.roadEdges[1].
         This covers urban signs (bottom 2.1 m) that leave the main strip
         top at ~42 m, before they reach a readable size.

Missing / bad inputs (no calibration, not the 1928x1208 AR/OX road camera,
no confident right edge) give the legacy STRIP_RECT and no upper strip.
"""
from __future__ import annotations

import math

import numpy as np

FRAME_W, FRAME_H = 1928, 1208
FOCAL = 2648.0               # AR0231 / OX03C10 road camera
MAIN = (964, 525, 964, 350)  # == two_stage.STRIP_RECT
NOMINAL_HORIZON_Y = 604.0    # principal point row; MAIN was tuned around it
HORIZON_MAX_DY = 120
UPPER_W, UPPER_H = 482, 175
EDGE_STD_MAX = 1.0           # m; roadEdgeStds above this -> no placement
EDGE_X_M = 40.0              # project the right edge at this range
EDGE_MARGIN_M = 1.0          # signs stand ~1 m+ outside the edge; start the band 1 m inside


def _even(v: float) -> int:
  i = int(round(v))
  return i - (i % 2)


def project(rpy, x_m: float, y_m: float, z_m: float = 0.0) -> tuple[float, float] | None:
  """Calibrated-frame point (x fwd, y right, z down) -> road camera pixel."""
  try:
    from openpilot.common.transformations.orientation import rot_from_euler
    from openpilot.common.transformations.camera import view_frame_from_device_frame
    p_dev = rot_from_euler([float(v) for v in rpy]) @ np.array([x_m, y_m, z_m], dtype=float)
    p_view = view_frame_from_device_frame @ p_dev
  except Exception:
    return None
  if p_view[2] <= 0.1:
    return None
  return FRAME_W / 2 + FOCAL * p_view[0] / p_view[2], FRAME_H / 2 + FOCAL * p_view[1] / p_view[2]


def horizon_dy(rpy) -> int | None:
  if rpy is None or len(rpy) != 3 or not all(math.isfinite(float(v)) for v in rpy):
    return None
  uv = project(rpy, 1000.0, 0.0, 0.0)
  if uv is None:
    return None
  dy = uv[1] - NOMINAL_HORIZON_Y
  return int(max(-HORIZON_MAX_DY, min(HORIZON_MAX_DY, dy)))


def right_edge_u(rpy, edge_x, edge_y, edge_std: float | None) -> float | None:
  """Pixel column of (right road edge - margin) at EDGE_X_M, or None."""
  if rpy is None or edge_std is None or not math.isfinite(edge_std) or edge_std > EDGE_STD_MAX:
    return None
  try:
    xs = np.asarray(edge_x, dtype=float)
    ys = np.asarray(edge_y, dtype=float)
    if xs.size < 2 or xs[-1] < EDGE_X_M:
      return None
    y = float(np.interp(EDGE_X_M, xs, ys))
  except Exception:
    return None
  if not 0.5 <= y <= 15.0:     # a right edge must be to the right
    return None
  uv = project(rpy, EDGE_X_M, y - EDGE_MARGIN_M, 0.0)
  return None if uv is None else float(uv[0])


def strips(frame_w: int, frame_h: int, *, rpy=None, edge=None):
  """-> (main_rect, upper_rect_or_None). edge = (x list, y list, std)."""
  if int(frame_w) != FRAME_W or int(frame_h) != FRAME_H:
    return MAIN, None
  dy = horizon_dy(rpy)
  if dy is None:
    return MAIN, None
  mx, my, mw, mh = MAIN
  my = my + (dy - dy % 2)
  main = (mx, my, mw, mh)
  upper = None
  if edge is not None:
    u = right_edge_u(rpy, edge[0], edge[1], edge[2])
    if u is not None:
      x0 = _even(min(max(u, mx), FRAME_W - UPPER_W))
      y0 = my - UPPER_H
      if y0 >= 0:
        upper = (x0, y0, UPPER_W, UPPER_H)
  return main, upper


def edge_from_model(md):
  """(x, y, std) of modelV2.roadEdges[1], or None."""
  try:
    e = md.roadEdges[1]
    std = float(md.roadEdgeStds[1])
    return list(e.x), list(e.y), std
  except Exception:
    return None
