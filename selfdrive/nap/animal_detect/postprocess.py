"""YOLOX output decode. The exported graph already decodes boxes:
out[b, n] = (cx, cy, w, h, objectness, class scores...) in tile pixels."""
from __future__ import annotations

import numpy as np

from openpilot.selfdrive.nap.animal_detect.config import CLASSES, DET_CONF, MAX_DETECTIONS, NMS_IOU


def iou(a, b) -> float:
  ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
  iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
  inter = ix * iy
  den = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
  return float(inter / den) if den > 0 else 0.0


def nms(boxes: np.ndarray, scores: np.ndarray, thr: float) -> list[int]:
  order = list(np.argsort(-scores))
  keep: list[int] = []
  while order:
    i = int(order.pop(0))
    keep.append(i)
    order = [j for j in order if iou(boxes[i], boxes[j]) < thr]
  return keep


def decode(out: np.ndarray, conf: float = DET_CONF, nms_iou: float = NMS_IOU,
           max_det: int = MAX_DETECTIONS) -> list[list[tuple[str, float, tuple[float, float, float, float]]]]:
  """Per tile: [(class name, score, (x0, y0, x1, y1)), ...] in tile pixels, best first."""
  out = np.asarray(out, dtype=np.float32)
  if out.ndim == 2:
    out = out[None]
  if out.ndim != 3 or out.shape[2] != 5 + len(CLASSES):
    raise ValueError(f"unexpected model output shape {out.shape}")
  res = []
  for t in out:
    t = t[np.isfinite(t).all(axis=1)]
    cls = np.argmax(t[:, 5:], axis=1)
    score = t[:, 4] * t[np.arange(len(t)), 5 + cls]
    m = score >= conf
    t, cls, score = t[m], cls[m], score[m]
    if len(t) == 0:
      res.append([])
      continue
    boxes = np.stack([t[:, 0] - t[:, 2] / 2, t[:, 1] - t[:, 3] / 2, t[:, 0] + t[:, 2] / 2, t[:, 1] + t[:, 3] / 2], axis=1)
    keep = nms(boxes, score, nms_iou)[:max_det]
    res.append([(CLASSES[int(cls[k])], float(score[k]), tuple(float(v) for v in boxes[k])) for k in keep])
  return res
