"""Compact raw-model summary for the roundabout assist's swaglog line (no capnp; plain JSON text)."""
from __future__ import annotations

import json

LOG_RAW_X = (5.0, 10.0, 15.0, 20.0, 30.0)


def _interp_xy(xs, ys, x: float) -> float | None:
  try:
    n = min(len(xs), len(ys))
  except TypeError:
    return None
  if n < 2:
    return None
  xs = [float(xs[i]) for i in range(n)]
  ys = [float(ys[i]) for i in range(n)]
  if not (xs[0] <= x <= xs[-1]):
    return None
  for i in range(1, n):
    if xs[i] >= x:
      a, b = xs[i - 1], xs[i]
      f = 0.0 if b <= a else (x - a) / (b - a)
      return ys[i - 1] + f * (ys[i] - ys[i - 1])
  return None


def _row(xs, ys) -> list:
  """y (m, device frame, + right) at LOG_RAW_X, None where the line does not reach."""
  out = []
  for x in LOG_RAW_X:
    y = _interp_xy(xs, ys, x)
    out.append(None if y is None else round(y, 2))
  return out


def raw_ring_summary(g, t: float, v_ego: float, model_k: float, out: float, model_v2,
                     driver) -> str:
  """One compact JSON line: ring pose / state plus the model's raw lane lines, road edges and path at 5/10/15/20/30 m."""
  d = g.debug
  rec: dict = {"t": round(t, 2), "v": round(v_ego, 2), "ph": g.phase, "lat": int(g.latched), "rel": g.release,
               "r": round(d.get("r", 0.0), 1), "rref": None if g.r_ref is None else round(g.r_ref, 2),
               "de": None if g.d_edge is None else round(g.d_edge, 1), "trav": round(g.travel, 0),
               "bias": round(d.get("bias", 0.0), 1), "bacc": round(d.get("bacc", 0.0), 0), "w": round(d.get("w", 0.0), 2),
               "k": round(model_k, 4), "out": round(out, 4)}
  if driver is not None:
    rec["stalk"] = [driver.stalk_dir, int(driver.stalk_held), int(driver.pressed), round(driver.torque, 1)]
  try:
    pos = model_v2.position
    rec["path"] = _row(pos.x, pos.y)
    lines, probs = model_v2.laneLines, model_v2.laneLineProbs
    rec["ll"] = [_row(lines[i].x, lines[i].y) for i in range(min(len(lines), 4))]
    rec["llp"] = [round(float(p), 2) for p in list(probs)[:4]]
    edges, stds = model_v2.roadEdges, model_v2.roadEdgeStds
    rec["re"] = [_row(edges[i].x, edges[i].y) for i in range(min(len(edges), 2))]
    rec["res"] = [round(float(p), 2) for p in list(stds)[:2]]
  except Exception:
    rec["model"] = "unavailable"
  return json.dumps(rec, separators=(",", ":"))
