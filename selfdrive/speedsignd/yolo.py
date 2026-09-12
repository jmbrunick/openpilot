"""YOLOv8 ONNX decode for the US MUTCD speed-sign detector.

Expects an Ultralytics detect export:
  input  `images`  float32 [1,3,H,W] RGB 0..1 (letterboxed)
  output `output0` float32 [1,4+nc,N]  (or [1,N,4+nc]) = cx,cy,w,h + class scores
"""
from __future__ import annotations

import re

import numpy as np

from openpilot.selfdrive.speedsignd.detect_types import MUTCD_MPH, SpeedSign
from openpilot.selfdrive.speedsignd.nv12 import detect_crop_rect

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


def road_detect_crop_rect(h: int, w: int) -> tuple[int, int, int, int]:
  """Right-biased square of the short side. Keeps optical center + right shoulder.

  A 1928×1208 ROAD frame letterboxed to 320 is scale 0.166 — a clear 24×30 in
  R2-1 at ~60 ft is ~15 px, below YOLOv8s-320. Short-side square is 0.265.
  US MUTCD plates live on the right; the left third is oncoming / unused.
  """
  return detect_crop_rect(h, w)


def road_detect_crop(rgb: np.ndarray) -> tuple[np.ndarray, tuple[int, int, int, int]]:
  """Right-biased square of the short side. Keeps optical center + right shoulder."""
  if rgb.ndim != 3 or rgb.shape[2] != 3:
    raise ValueError("road_detect_crop expects HxWx3")
  h, w = rgb.shape[:2]
  x, y, side, _ = road_detect_crop_rect(h, w)
  return rgb[y:y + side, x:x + side], (x, y, side, side)


def _resize_rgb(img: np.ndarray, h: int, w: int) -> np.ndarray:
  """Bilinear resize. Nearest index-sampling destroyed distant R2-1s at 6×."""
  if img.shape[0] == h and img.shape[1] == w:
    return img
  try:
    import cv2
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
  except Exception:
    return _resize_rgb_bilinear(img, h, w)


def _resize_rgb_bilinear(img: np.ndarray, h: int, w: int) -> np.ndarray:
  src_h, src_w = img.shape[:2]
  if src_h < 1 or src_w < 1:
    return np.zeros((h, w, img.shape[2]), dtype=img.dtype)
  ys = (np.arange(h, dtype=np.float32) + 0.5) * (src_h / float(h)) - 0.5
  xs = (np.arange(w, dtype=np.float32) + 0.5) * (src_w / float(w)) - 0.5
  ys = np.clip(ys, 0.0, src_h - 1.0)
  xs = np.clip(xs, 0.0, src_w - 1.0)
  y0 = np.floor(ys).astype(np.int32)
  x0 = np.floor(xs).astype(np.int32)
  y1 = np.minimum(y0 + 1, src_h - 1)
  x1 = np.minimum(x0 + 1, src_w - 1)
  wy = (ys - y0).astype(np.float32)[:, None, None]
  wx = (xs - x0).astype(np.float32)[None, :, None]
  img_f = img.astype(np.float32)
  i00 = img_f[y0][:, x0]
  i01 = img_f[y0][:, x1]
  i10 = img_f[y1][:, x0]
  i11 = img_f[y1][:, x1]
  out = (i00 * (1.0 - wx) + i01 * wx) * (1.0 - wy) + (i10 * (1.0 - wx) + i11 * wx) * wy
  return np.clip(np.rint(out), 0, 255).astype(np.uint8)


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
  """Normalize YOLO output to [4+nc, N]. Squeeze tinygrad singleton dims."""
  arr = np.squeeze(np.asarray(raw, dtype=np.float32))
  if arr.ndim == 3 and arr.shape[0] == 1:
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


def yolo_peak(raw, names: tuple[str, ...] = YOLO_CLASS_NAMES) -> tuple[tuple, float, str, int]:
  """Pre-threshold peak: out_shape, max_conf, top class name, n_over min_conf."""
  arr = np.asarray(raw)
  shape = tuple(int(v) for v in arr.shape)
  pred = _as_cn(raw)
  if pred.size == 0 or pred.shape[0] < 5:
    return shape, 0.0, "", 0
  scores = pred[4:]
  if scores.size == 0:
    return shape, 0.0, "", 0
  conf = scores.max(axis=0) if scores.ndim == 2 else scores.reshape(-1)
  cls = scores.argmax(axis=0) if scores.ndim == 2 else np.zeros(conf.shape, np.int32)
  if conf.size == 0:
    return shape, 0.0, "", 0
  i = int(conf.argmax())
  max_conf = float(conf[i])
  top = int(cls[i]) if cls.size else 0
  name = names[top] if 0 <= top < len(names) else str(top)
  n_over = int(np.sum(conf >= YOLO_MIN_CONF))
  return shape, max_conf, name, n_over


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
