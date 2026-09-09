"""YOLOv8 ONNX decode for the US MUTCD speed-sign detector.

Expects an Ultralytics detect export:
  input  `images`  float32 [1,3,H,W] RGB 0..1 (letterboxed)
  output `output0` float32 [1,4+nc,N]  (or [1,N,4+nc]) = cx,cy,w,h + class scores
"""
from __future__ import annotations

import re

import numpy as np

from openpilot.selfdrive.speedsignd.detect_types import MUTCD_MPH, SpeedSign

# Crop reader must beat this to override the YOLO class (real Highway Gothic
# usually agrees; synthetic/block digits often need the override).
CROP_OVERRIDE_CONF = 0.55
from openpilot.selfdrive.speedsignd.weights_manifest import (
  YOLO_CLASS_NAMES,
  YOLO_IMGSZ,
  YOLO_IOU,
  YOLO_MAX_DET,
  YOLO_MIN_CONF,
)

_SPEED_RE = re.compile(r"^speedLimit(\d+)$")


def class_to_mph(name: str) -> int | None:
  m = _SPEED_RE.match(str(name))
  if not m:
    return None
  value = int(m.group(1))
  return value if value in MUTCD_MPH else None


def letterbox_rgb(rgb: np.ndarray, size: int = YOLO_IMGSZ) -> tuple[np.ndarray, float, int, int]:
  """Resize with aspect ratio, pad to size×size. Returns image, scale, pad_x, pad_y."""
  if rgb.ndim != 3 or rgb.shape[2] != 3:
    raise ValueError("letterbox_rgb expects HxWx3")
  h, w = rgb.shape[:2]
  scale = min(size / float(h), size / float(w))
  nh = max(1, int(round(h * scale)))
  nw = max(1, int(round(w * scale)))
  resized = _resize_rgb(rgb, nh, nw)
  canvas = np.full((size, size, 3), 114, dtype=np.uint8)
  top = (size - nh) // 2
  left = (size - nw) // 2
  canvas[top:top + nh, left:left + nw] = resized
  return canvas, scale, left, top


def _resize_rgb(img: np.ndarray, h: int, w: int) -> np.ndarray:
  if img.shape[0] == h and img.shape[1] == w:
    return img
  ys = np.linspace(0, img.shape[0] - 1, h).astype(np.int32)
  xs = np.linspace(0, img.shape[1] - 1, w).astype(np.int32)
  return img[ys][:, xs]


def nms_xyxy(boxes: np.ndarray, scores: np.ndarray, iou_thr: float = YOLO_IOU, max_det: int = YOLO_MAX_DET) -> list[int]:
  """Greedy IoU NMS. boxes are [N,4] xyxy."""
  if len(boxes) == 0:
    return []
  order = scores.argsort()[::-1]
  keep: list[int] = []
  while order.size and len(keep) < max_det:
    i = int(order[0])
    keep.append(i)
    if order.size == 1:
      break
    rest = order[1:]
    xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
    yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
    xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
    yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
    inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
    area_i = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
    area_r = (boxes[rest, 2] - boxes[rest, 0]) * (boxes[rest, 3] - boxes[rest, 1])
    iou = inter / np.maximum(1e-6, area_i + area_r - inter)
    order = rest[iou <= iou_thr]
  return keep


def _as_cn(raw: np.ndarray) -> np.ndarray:
  """Normalize YOLO output to [4+nc, N]."""
  arr = np.asarray(raw, dtype=np.float32)
  if arr.ndim == 3:
    arr = arr[0]
  if arr.ndim != 2:
    return np.zeros((0, 0), np.float32)
  channels = 4 + len(YOLO_CLASS_NAMES)
  if arr.shape[0] == channels:
    return arr
  if arr.shape[1] == channels:
    return arr.T
  # Unknown nc: prefer the shorter axis as channels (4+nc << anchors).
  if arr.shape[0] <= arr.shape[1] and arr.shape[0] >= 6:
    return arr
  if arr.shape[1] >= 6:
    return arr.T
  return arr


def decode_yolov8(
  raw,
  *,
  scale: float,
  pad_x: int,
  pad_y: int,
  src_hw: tuple[int, int],
  names: tuple[str, ...] = YOLO_CLASS_NAMES,
  min_conf: float = YOLO_MIN_CONF,
  iou: float = YOLO_IOU,
  max_det: int = YOLO_MAX_DET,
) -> list[SpeedSign]:
  pred = _as_cn(raw)
  if pred.size == 0 or pred.shape[0] < 5:
    return []
  boxes_xywh = pred[:4].T
  scores = pred[4:].T
  if scores.size == 0:
    return []
  cls = scores.argmax(axis=1)
  conf = scores.max(axis=1)
  mask = conf >= min_conf
  if not np.any(mask):
    return []
  boxes_xywh = boxes_xywh[mask]
  conf = conf[mask]
  cls = cls[mask]

  cx, cy, bw, bh = boxes_xywh[:, 0], boxes_xywh[:, 1], boxes_xywh[:, 2], boxes_xywh[:, 3]
  xyxy = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], axis=1)
  keep = nms_xyxy(xyxy, conf, iou_thr=iou, max_det=max(max_det * 3, 8))

  src_h, src_w = src_hw
  inv = 1.0 / scale if scale > 1e-6 else 1.0
  found: list[SpeedSign] = []
  for i in keep:
    name = names[int(cls[i])] if 0 <= int(cls[i]) < len(names) else ""
    mph = class_to_mph(name)
    if mph is None:
      continue
    x1 = (xyxy[i, 0] - pad_x) * inv
    y1 = (xyxy[i, 1] - pad_y) * inv
    x2 = (xyxy[i, 2] - pad_x) * inv
    y2 = (xyxy[i, 3] - pad_y) * inv
    x = int(round(max(0.0, min(x1, x2))))
    y = int(round(max(0.0, min(y1, y2))))
    w = int(round(max(1.0, abs(x2 - x1))))
    h = int(round(max(1.0, abs(y2 - y1))))
    if x >= src_w or y >= src_h:
      continue
    w = min(w, src_w - x)
    h = min(h, src_h - y)
    found.append(SpeedSign(mph=mph, conf=float(conf[i]), bbox=(x, y, w, h)))
    if len(found) >= max_det:
      break
  found.sort(key=lambda s: s.conf, reverse=True)
  return found


def refine_mph(sign: SpeedSign, y: np.ndarray | None, read_mph) -> SpeedSign:
  """Optional digit read on the YOLO crop. Does not invent a detection."""
  if y is None or y.ndim != 2:
    return sign
  x, yy, w, h = sign.bbox
  if w < 12 or h < 12:
    return sign
  inset = max(3, int(min(w, h) * 0.06))
  x0 = max(0, x + inset)
  y0 = max(0, yy + inset)
  x1 = min(y.shape[1], x + w - inset)
  y1 = min(y.shape[0], yy + h - inset)
  if x1 - x0 < 12 or y1 - y0 < 12:
    return sign
  crop = y[y0:y1, x0:x1]
  if crop.size == 0:
    return sign
  mph, conf = read_mph(crop)
  if mph is None or mph not in MUTCD_MPH:
    return sign
  if mph == sign.mph:
    return SpeedSign(mph=mph, conf=float(min(1.0, max(sign.conf, conf))), bbox=sign.bbox)
  if conf >= CROP_OVERRIDE_CONF:
    return SpeedSign(mph=int(mph), conf=float(conf), bbox=sign.bbox)
  return sign
