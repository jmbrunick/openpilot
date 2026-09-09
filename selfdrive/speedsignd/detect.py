"""MUTCD R2-1 detector: YOLO ONNX on the ROAD camera, numpy fallback for tests.

Stock modelV2 has no speedSign head. On-device detection is a compact YOLOv8
ONNX under /data (see weights_manifest). The numpy template matcher is only
for unit tests / missing weights — it does not see real roadside signs.

MUTCD R2-1 is a white rectangle (about 24x30 in, aspect ~0.8) with black
legend: SPEED LIMIT over a 1–3 digit mph value.
"""
from __future__ import annotations

import os

import numpy as np

from openpilot.selfdrive.speedsignd.detect_types import MUTCD_MPH, SpeedSign
from openpilot.selfdrive.speedsignd.nv12 import rgb_from_y, y_plane_from_nv12
from openpilot.selfdrive.speedsignd.paths import default_onnx_path
from openpilot.selfdrive.speedsignd.weights_manifest import (
  YOLO_CLASS_NAMES,
  YOLO_IMGSZ,
  YOLO_IOU,
  YOLO_MAX_DET,
  YOLO_MIN_CONF,
)
from openpilot.selfdrive.speedsignd.yolo import decode_yolov8, letterbox_rgb, refine_mph

MIN_CONF = 0.42
MAX_DET = 3
DIGIT_H, DIGIT_W = 24, 16
# Downsample the ROAD frame so CC stays cheap on the 3X.
MAX_DETECT_WIDTH = 320


def _resize(img: np.ndarray, h: int, w: int) -> np.ndarray:
  if img.shape[0] == h and img.shape[1] == w:
    return img
  ys = np.linspace(0, img.shape[0] - 1, h).astype(np.int32)
  xs = np.linspace(0, img.shape[1] - 1, w).astype(np.int32)
  return img[ys][:, xs]


def _ncc(a: np.ndarray, b: np.ndarray) -> float:
  x = a.astype(np.float32).ravel()
  y = b.astype(np.float32).ravel()
  x -= x.mean()
  y -= y.mean()
  den = float(np.linalg.norm(x) * np.linalg.norm(y))
  if den < 1e-6:
    return 0.0
  return float(np.dot(x, y) / den)


def _paint_rect(t: np.ndarray, r0: int, r1: int, c0: int, c1: int, v: int = 1) -> None:
  t[r0:r1, c0:c1] = v


def _digit_templates() -> dict[int, np.ndarray]:
  """Block Highway-Gothic-ish 16x24 ink templates (1 = ink)."""
  out: dict[int, np.ndarray] = {}
  h, w = DIGIT_H, DIGIT_W
  thick = 3

  def blank() -> np.ndarray:
    return np.zeros((h, w), np.uint8)

  # 0: closed oval
  t = blank()
  _paint_rect(t, 1, 1 + thick, 3, w - 3)
  _paint_rect(t, h - 1 - thick, h - 1, 3, w - 3)
  _paint_rect(t, 1, h - 1, 2, 2 + thick)
  _paint_rect(t, 1, h - 1, w - 2 - thick, w - 2)
  out[0] = t

  # 1: right stem + top serif
  t = blank()
  _paint_rect(t, 1, h - 1, w // 2, w // 2 + thick)
  _paint_rect(t, 1, 1 + thick, w // 2 - 4, w // 2 + thick)
  _paint_rect(t, h - 1 - thick, h - 1, w // 2 - 4, w // 2 + 5)
  out[1] = t

  # 2
  t = blank()
  _paint_rect(t, 1, 1 + thick, 2, w - 2)
  _paint_rect(t, 1, h // 2, w - 2 - thick, w - 2)
  _paint_rect(t, h // 2 - 1, h // 2 - 1 + thick, 2, w - 2)
  _paint_rect(t, h // 2, h - 1, 2, 2 + thick)
  _paint_rect(t, h - 1 - thick, h - 1, 2, w - 2)
  out[2] = t

  # 3
  t = blank()
  _paint_rect(t, 1, 1 + thick, 2, w - 2)
  _paint_rect(t, h // 2 - 1, h // 2 - 1 + thick, 3, w - 3)
  _paint_rect(t, h - 1 - thick, h - 1, 2, w - 2)
  _paint_rect(t, 1, h - 1, w - 2 - thick, w - 2)
  out[3] = t

  # 4: open top, crossbar, right stem
  t = blank()
  _paint_rect(t, 1, h // 2 + 1, 2, 2 + thick)
  _paint_rect(t, h // 2 - 1, h // 2 - 1 + thick, 2, w - 2)
  _paint_rect(t, 1, h - 1, w - 5, w - 2)
  out[4] = t

  # 5
  t = blank()
  _paint_rect(t, 1, 1 + thick, 2, w - 2)
  _paint_rect(t, 1, h // 2, 2, 2 + thick)
  _paint_rect(t, h // 2 - 1, h // 2 - 1 + thick, 2, w - 2)
  _paint_rect(t, h // 2, h - 1, w - 2 - thick, w - 2)
  _paint_rect(t, h - 1 - thick, h - 1, 2, w - 2)
  out[5] = t

  # 6
  t = blank()
  _paint_rect(t, 1, 1 + thick, 2, w - 3)
  _paint_rect(t, 1, h - 1, 2, 2 + thick)
  _paint_rect(t, h // 2 - 1, h // 2 - 1 + thick, 2, w - 2)
  _paint_rect(t, h // 2, h - 1, w - 2 - thick, w - 2)
  _paint_rect(t, h - 1 - thick, h - 1, 2, w - 2)
  out[6] = t

  # 7
  t = blank()
  _paint_rect(t, 1, 1 + thick, 2, w - 2)
  _paint_rect(t, 1, h - 1, w - 2 - thick, w - 2)
  out[7] = t

  # 8
  t = blank()
  _paint_rect(t, 1, 1 + thick, 3, w - 3)
  _paint_rect(t, h // 2 - 1, h // 2 - 1 + thick, 3, w - 3)
  _paint_rect(t, h - 1 - thick, h - 1, 3, w - 3)
  _paint_rect(t, 1, h - 1, 2, 2 + thick)
  _paint_rect(t, 1, h - 1, w - 2 - thick, w - 2)
  out[8] = t

  # 9
  t = blank()
  _paint_rect(t, 1, 1 + thick, 2, w - 2)
  _paint_rect(t, 1, h // 2 + 2, 2, 2 + thick)
  _paint_rect(t, 1, h // 2 + 2, w - 2 - thick, w - 2)
  _paint_rect(t, h // 2 - 1, h // 2 - 1 + thick, 2, w - 2)
  _paint_rect(t, h // 2, h - 1, w - 2 - thick, w - 2)
  _paint_rect(t, h - 1 - thick, h - 1, 2, w - 3)
  out[9] = t
  return out


DIGIT_TEMPLATES = _digit_templates()


def paint_digit(canvas: np.ndarray, digit: int, r0: int, c0: int, scale: int = 2, ink: int = 20) -> None:
  """Blit a template onto a uint8 Y image (for tests and ONNX fixtures)."""
  t = DIGIT_TEMPLATES[int(digit)]
  h, w = t.shape
  block = np.repeat(np.repeat(t, scale, axis=0), scale, axis=1)
  r1 = r0 + block.shape[0]
  c1 = c0 + block.shape[1]
  if r0 < 0 or c0 < 0 or r1 > canvas.shape[0] or c1 > canvas.shape[1]:
    return
  canvas[r0:r1, c0:c1] = np.where(block > 0, ink, canvas[r0:r1, c0:c1])


def paint_mutcd_r2_1(canvas: np.ndarray, mph: int, x: int, y: int, w: int, h: int) -> None:
  """Draw a white R2-1-style plate with SPEED LIMIT bars and gothic-ish digits."""
  canvas[y:y + h, x:x + w] = 235
  canvas[y:y + 3, x:x + w] = 15
  canvas[y + h - 3:y + h, x:x + w] = 15
  canvas[y:y + h, x:x + 3] = 15
  canvas[y:y + h, x + w - 3:x + w] = 15
  # "SPEED LIMIT" stand-in: two dark word bars in the upper third.
  bar_h = max(3, h // 18)
  canvas[y + h // 8:y + h // 8 + bar_h, x + w // 6:x + w - w // 6] = 25
  canvas[y + h // 5:y + h // 5 + bar_h, x + w // 5:x + w - w // 5] = 25
  digits = [int(ch) for ch in str(int(mph))]
  n = len(digits)
  inner_w = max(8, w - 12)
  inner_h = max(8, h // 2 - 10)
  scale = max(1, min(inner_h // DIGIT_H, (inner_w - (n - 1) * 2) // max(1, n * DIGIT_W)))
  digit_w = DIGIT_W * scale
  digit_h = DIGIT_H * scale
  gap = max(2, scale)
  total_w = n * digit_w + (n - 1) * gap
  c0 = x + (w - total_w) // 2
  r0 = y + h // 2 + h // 16
  if r0 + digit_h > y + h - 4:
    r0 = y + h - 4 - digit_h
  for i, d in enumerate(digits):
    paint_digit(canvas, d, r0, c0 + i * (digit_w + gap), scale=scale, ink=18)


def _components(mask: np.ndarray) -> list[tuple[int, int, int, int, int]]:
  """Two-pass connected components → (x, y, w, h, area)."""
  h, w = mask.shape
  labels = np.zeros((h, w), np.int32)
  parent = [0]

  def find(a: int) -> int:
    while parent[a] != a:
      parent[a] = parent[parent[a]]
      a = parent[a]
    return a

  def union(a: int, b: int) -> None:
    ra, rb = find(a), find(b)
    if ra != rb:
      parent[rb] = ra

  n = 0
  for r in range(h):
    row = mask[r]
    lab_row = labels[r]
    lab_up = labels[r - 1] if r else None
    for c in range(w):
      if not row[c]:
        continue
      left = lab_row[c - 1] if c else 0
      up = lab_up[c] if lab_up is not None else 0
      if left and up:
        lab_row[c] = left
        union(int(left), int(up))
      elif left:
        lab_row[c] = left
      elif up:
        lab_row[c] = up
      else:
        n += 1
        parent.append(n)
        lab_row[c] = n

  remap: dict[int, int] = {}
  next_id = 1
  stats: dict[int, list[int]] = {}
  for r in range(h):
    lab_row = labels[r]
    for c in range(w):
      lab = int(lab_row[c])
      if lab == 0:
        continue
      root = find(lab)
      i = remap.get(root)
      if i is None:
        remap[root] = next_id
        i = next_id
        next_id += 1
        stats[i] = [c, r, c, r, 0]
      s = stats[i]
      if c < s[0]:
        s[0] = c
      if r < s[1]:
        s[1] = r
      if c > s[2]:
        s[2] = c
      if r > s[3]:
        s[3] = r
      s[4] += 1
  boxes = []
  for x0, y0, x1, y1, area in stats.values():
    boxes.append((x0, y0, x1 - x0 + 1, y1 - y0 + 1, area))
  return boxes


def _digit_score(patch: np.ndarray) -> tuple[int, float]:
  ink = (patch < 90).astype(np.uint8)
  if ink.mean() < 0.05 or ink.mean() > 0.85:
    return 0, 0.0
  templ = _resize(ink, DIGIT_H, DIGIT_W)
  best_d, best_s = 0, -1.0
  for d, t in DIGIT_TEMPLATES.items():
    s = _ncc(templ, t)
    if s > best_s:
      best_d, best_s = d, s
  return best_d, float(best_s)


def _split_digit_boxes(ink: np.ndarray) -> list[tuple[int, int, int, int]]:
  if ink.size == 0:
    return []
  col = ink.mean(axis=0)
  if col.size >= 3:
    col = np.convolve(col, np.ones(3, dtype=np.float32) / 3.0, mode="same")
  thresh = max(0.025, float(col.max()) * 0.12)
  runs = []
  in_run = False
  start = 0
  for i, v in enumerate(col):
    if v >= thresh and not in_run:
      in_run = True
      start = i
    elif v < thresh and in_run:
      in_run = False
      runs.append((start, i))
  if in_run:
    runs.append((start, len(col)))

  raw: list[tuple[int, int, int, int]] = []
  for x0, x1 in runs:
    if x1 - x0 < 2:
      continue
    strip = ink[:, x0:x1]
    rows = np.where(strip.mean(axis=1) > 0.05)[0]
    if len(rows) < 4:
      continue
    y0, y1 = int(rows[0]), int(rows[-1]) + 1
    raw.append((x0, y0, x1 - x0, y1 - y0))
  if not raw:
    return []

  # Open 4 / thin 7: merge adjacent narrow stems. Do not glue a thin stem onto a wide neighbor (4+5).
  typical = max(b[2] for b in raw)
  thin = max(4, int(typical * 0.45))
  merged: list[tuple[int, int, int, int]] = [raw[0]]
  for b in raw[1:]:
    prev = merged[-1]
    gap = b[0] - (prev[0] + prev[2])
    if prev[2] <= thin and b[2] <= thin and gap <= max(8, typical // 2):
      x0 = prev[0]
      y0 = min(prev[1], b[1])
      x1 = b[0] + b[2]
      y1 = max(prev[1] + prev[3], b[1] + b[3])
      merged[-1] = (x0, y0, x1 - x0, y1 - y0)
    else:
      merged.append(b)
  return merged[:3]


def _read_mph(crop: np.ndarray) -> tuple[int | None, float]:
  h, w = crop.shape
  if h < 12 or w < 12:
    return None, 0.0
  lower = crop[int(h * 0.42):, int(w * 0.08):int(w * 0.92)]
  if lower.size == 0:
    return None, 0.0
  ink = (lower < 90).astype(np.uint8)
  boxes = _split_digit_boxes(ink)
  if not boxes:
    return None, 0.0
  digits = []
  scores = []
  for x, y, bw, bh in boxes:
    pad = 1
    y0 = max(0, y - pad)
    x0 = max(0, x - pad)
    patch = lower[y0:min(lower.shape[0], y + bh + pad), x0:min(lower.shape[1], x + bw + pad)]
    d, s = _digit_score(patch)
    digits.append(d)
    scores.append(s)
  if not scores or min(scores) < 0.25:
    return None, 0.0
  value = 0
  for d in digits:
    value = value * 10 + d
  if value not in MUTCD_MPH:
    return None, 0.0
  conf = float(sum(scores) / len(scores))
  # Extra boost when the upper third looks like stacked word bars (SPEED LIMIT).
  upper = crop[:int(h * 0.40), int(w * 0.10):int(w * 0.90)]
  if upper.size and upper.std() > 18.0:
    conf = min(1.0, conf + 0.08)
  return value, conf


def _sign_candidates(y: np.ndarray) -> list[tuple[int, int, int, int]]:
  h, w = y.shape
  bright = y > 175
  boxes = _components(bright)
  out = []
  frame_area = h * w
  for x, yy, bw, bh, area in boxes:
    if bh < 18 or bw < 12:
      continue
    if area < 80:
      continue
    aspect = bw / float(bh)
    if aspect < 0.45 or aspect > 1.15:
      continue
    fill = area / float(bw * bh)
    if fill < 0.35:
      continue
    if bw * bh > 0.35 * frame_area:
      continue
    crop = y[yy:yy + bh, x:x + bw]
    if crop.std() < 22.0:
      continue
    # White plate with dark ink: mean should stay high.
    if crop.mean() < 140:
      continue
    out.append((x, yy, bw, bh))
  return out


def detect_mutcd_speed_signs(y: np.ndarray, min_conf: float = MIN_CONF) -> list[SpeedSign]:
  """Return MUTCD numeric speed signs on a uint8 Y-plane image."""
  if y is None or y.ndim != 2 or y.shape[0] < 32 or y.shape[1] < 32:
    return []
  src_h, src_w = y.shape
  work = y
  scale = 1.0
  if src_w > MAX_DETECT_WIDTH:
    scale = MAX_DETECT_WIDTH / float(src_w)
    nh = max(32, int(src_h * scale))
    work = _resize(y, nh, MAX_DETECT_WIDTH)

  found: list[SpeedSign] = []
  for x, yy, bw, bh in _sign_candidates(work):
    mph, conf = _read_mph(work[yy:yy + bh, x:x + bw])
    if mph is None or conf < min_conf:
      continue
    if scale != 1.0:
      inv = 1.0 / scale
      x, yy, bw, bh = int(x * inv), int(yy * inv), int(bw * inv), int(bh * inv)
    found.append(SpeedSign(mph=mph, conf=float(conf), bbox=(x, yy, bw, bh)))
    if len(found) >= MAX_DET:
      break
  found.sort(key=lambda s: s.conf, reverse=True)
  return found


class OnnxSpeedSignDetector:
  """Compact ONNX under /data. Missing/unloadable → None.

  Supported models:
    1. Ultralytics YOLOv8 detect (preferred):
         input  float32 [1,3,H,W] RGB 0..1 (letterboxed)
         output float32 [1,4+nc,N] or [1,N,4+nc]
    2. Legacy custom:
         input  `image`  float32 [1,1,H,W] (Y 0..1) or [1,3,H,W] (RGB 0..1)
         output `dets`   float32 [N,6] = x, y, w, h, mph, conf
         A [1,2] (mph, conf) output is also accepted.
  """

  def __init__(self, path: str, session=None):
    self.path = path
    self.session = session

  @classmethod
  def try_load(cls, path: str | None = None) -> OnnxSpeedSignDetector | None:
    path = path or default_onnx_path()
    if not path or not os.path.isfile(path):
      return None
    session = _onnx_session(path)
    if session is None:
      return None
    return cls(path, session)

  def detect(self, y: np.ndarray, min_conf: float | None = None, rgb: np.ndarray | None = None) -> list[SpeedSign]:
    if self.session is None:
      return []
    if rgb is None:
      if y is None or y.ndim != 2:
        return []
      rgb = rgb_from_y(y)
    if rgb.ndim != 3 or rgb.shape[-1] != 3:
      return []
    src_h, src_w = rgb.shape[:2]
    inp = _onnx_input_name(self.session)
    shape = _onnx_input_shape(self.session)
    yolo = _is_yolo_input(shape)
    try:
      if yolo:
        size = YOLO_IMGSZ
        if shape is not None and len(shape) == 4:
          h = int(shape[2]) if shape[2] not in (None, 0, -1) else YOLO_IMGSZ
          w = int(shape[3]) if shape[3] not in (None, 0, -1) else YOLO_IMGSZ
          size = h if h == w else YOLO_IMGSZ
        boxed, scale, pad_x, pad_y = letterbox_rgb(rgb, size)
        blob = boxed.transpose(2, 0, 1)[None, ...].astype(np.float32) / 255.0
        raw = self.session.run(None, {inp: blob})[0]
        thr = YOLO_MIN_CONF if min_conf is None else min_conf
        hits = decode_yolov8(
          raw, scale=scale, pad_x=pad_x, pad_y=pad_y, src_hw=(src_h, src_w),
          names=YOLO_CLASS_NAMES, min_conf=thr, iou=YOLO_IOU, max_det=YOLO_MAX_DET,
        )
        luma = y if y is not None and getattr(y, "ndim", 0) == 2 else rgb[:, :, 1]
        return [refine_mph(s, luma, _read_mph) for s in hits]
      arr = (y if y is not None else rgb[:, :, 1]).astype(np.float32) / 255.0
      if shape is not None and len(shape) == 4:
        _n, c, h, w = [int(v) if v not in (None, 0, -1) else None for v in shape]
        hh = h or 256
        ww = w or 256
        resized = _resize(arr, hh, ww)
        if c == 3:
          blob = np.stack([resized, resized, resized], axis=0)[None, ...]
        else:
          blob = resized[None, None, ...]
      else:
        blob = arr[None, None, ...]
      raw = self.session.run(None, {inp: blob.astype(np.float32)})[0]
    except Exception:
      return []
    thr = MIN_CONF if min_conf is None else min_conf
    return _parse_onnx_dets(raw, (src_h, src_w), thr)


def _onnx_session(path: str):
  try:
    import onnxruntime as ort
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = 1
    return ort.InferenceSession(path, opts, providers=["CPUExecutionProvider"])
  except Exception:
    pass
  try:
    from tinygrad import Tensor
    from tinygrad.nn.onnx import OnnxRunner
    return _TinyOrtSession(OnnxRunner(path), Tensor)
  except Exception:
    return None


class _TinyOrtSession:
  """onnxruntime-shaped wrapper around tinygrad OnnxRunner (3X has tinygrad)."""

  def __init__(self, runner, tensor_cls):
    self.runner = runner
    self._tensor = tensor_cls
    names = list(getattr(runner, "graph_inputs", {}) or {"images": None})
    self._inputs = [type("I", (), {"name": names[0] if names else "images", "shape": [1, 3, YOLO_IMGSZ, YOLO_IMGSZ]})()]

  def get_inputs(self):
    return self._inputs

  def run(self, _outs, feed: dict):
    tensors = {k: self._tensor(v) for k, v in feed.items()}
    out = self.runner(tensors)
    if isinstance(out, dict):
      val = next(iter(out.values()))
    else:
      val = out
    arr = val.numpy() if hasattr(val, "numpy") else np.asarray(val)
    return [arr]


def _is_yolo_input(shape) -> bool:
  if shape is None or len(shape) != 4:
    return False
  c = shape[1]
  return c in (3, "3")


def _onnx_input_name(session) -> str:
  try:
    return session.get_inputs()[0].name
  except Exception:
    return "image"


def _onnx_input_shape(session):
  try:
    return session.get_inputs()[0].shape
  except Exception:
    return None


def _parse_onnx_dets(raw, frame_hw: tuple[int, int], min_conf: float) -> list[SpeedSign]:
  arr = np.asarray(raw, dtype=np.float32)
  signs: list[SpeedSign] = []
  if arr.ndim == 1 and arr.size >= 2:
    arr = arr.reshape(1, -1)
  if arr.ndim == 3:
    arr = arr.reshape(-1, arr.shape[-1])
  if arr.ndim != 2:
    return []
  fh, fw = frame_hw
  for row in arr:
    if row.size == 2:
      mph, conf = float(row[0]), float(row[1])
      x, yy, bw, bh = 0, 0, fw, fh
    elif row.size >= 6:
      x, yy, bw, bh, mph, conf = (float(v) for v in row[:6])
    else:
      continue
    if conf < min_conf:
      continue
    ival = int(round(mph))
    if ival not in MUTCD_MPH:
      continue
    signs.append(SpeedSign(mph=ival, conf=float(conf), bbox=(int(x), int(yy), int(bw), int(bh))))
  signs.sort(key=lambda s: s.conf, reverse=True)
  return signs[:MAX_DET]


class SpeedSignDetector:
  """YOLO ONNX if weights exist on /data. Numpy matcher is tests/dev only."""

  def __init__(self, onnx: OnnxSpeedSignDetector | None = None, onnx_path: str | None = None):
    if onnx is not None:
      self.onnx = onnx
    else:
      self.onnx = OnnxSpeedSignDetector.try_load(onnx_path)

  def detect(self, y: np.ndarray, min_conf: float | None = None, rgb: np.ndarray | None = None) -> list[SpeedSign]:
    if self.onnx is not None:
      thr = YOLO_MIN_CONF if min_conf is None else min_conf
      return self.onnx.detect(y, min_conf=thr, rgb=rgb)
    thr = MIN_CONF if min_conf is None else min_conf
    return detect_mutcd_speed_signs(y, min_conf=thr)
