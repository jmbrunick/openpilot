"""Two-stage speed-sign reader: cheap strip proposer, IoU tracker, one sharp read.

Stage 1 (find): a 1-class "sign" YOLOv8 (width 0.125) looks at the lower-right
strip x 964-1928, y 525-875 of the 1928x1208 ROAD frame, resized to 352x128.
~0.3 GFLOP, ~110 ms on one 3X core, so 2 Hz is ~22 % of a core. Skipped below
2 m/s (GPS speed / modelV2 velocity; no carState subscriber).

Stage 2 (read): proposals are linked into tracks (IoU or center distance).
A track must be seen twice, unless one proposal is at/above
TRACK_SOLO_PROPOSAL_CONF. The read crop is the track box AFTER this frame's
proposals are applied (the measured box, not the extrapolated one). When the
track is near its largest (box height >= 60 px, near the right edge, growth
has stopped, or the track was lost) we copy one square native crop around it
(3x the box, clamped 240-480 px), letterbox it to 320 and run the v3 detector
+ digit check. A hit may be read once more (READS_AFTER_MPH) so a second HUD
vote can land; a neg_yellow / neg_other read is suppressed and never read again.
While any 1-hit track is alive the proposer runs at PROPOSER_BURST_HZ, then
returns to PROPOSER_HZ. Proposer time is not part of the reader duty budget.

Weights: Release speed-sign-proposer-v1 / sign_proposer.onnx (Install weights).
When the proposer file is missing, speedsignd keeps the legacy fixed tiles.
"""
from __future__ import annotations

import json
import math
import os
import re
import shutil
import time
from dataclasses import dataclass, field

import numpy as np

STRIP_RECT = (964, 525, 964, 350)  # x, y, w, h in the 1928x1208 frame
PROPOSER_IN_W = 352
PROPOSER_IN_H = 128
PROPOSER_CONF = 0.15
PROPOSER_IOU = 0.5
PROPOSER_HZ = 2.0
# While a 1-hit track is alive. 5 Hz * ~110 ms ≈ 55% of core 2, and only
# until the second hit (or the track expires). Set equal to PROPOSER_HZ to
# disable the burst.
PROPOSER_BURST_HZ = 5.0
PROPOSER_MIN_SPEED_MS = 2.0

TRACK_IOU = 0.3
TRACK_MAX_GAP_S = 1.6
TRACK_MIN_HITS = 2
# One proposal at/above this may be read without a second sighting.
# Above 1.0 disables the bypass (seen-twice only). This is the proposer
# score, not YOLO_MIN_CONF.
TRACK_SOLO_PROPOSAL_CONF = 0.85
CAPTURE_MIN_H = 60
CAPTURE_EDGE_X = 1928 - 150
CAPTURE_MIN_SIDE = 240
CAPTURE_MAX_SIDE = 480
CAPTURE_SCALE = 3.0
MAX_READS_PER_TRACK = 3
# A track not re-measured for this long is retired (never cropped stale).
LOST_S = 0.6
# Speed-scaled read size: 60 px at/below READ_V_SLOW, 40 px at/above READ_V_FAST
# (55 mph). The reader letterboxes to 320 either way; at 55 mph a 60 px box is
# only ~1.2 s from the car and urban signs leave the strip top first (~42 m).
READ_PX_SLOW = 60.0
READ_PX_FAST = 40.0
READ_V_SLOW = 15.6   # 35 mph
READ_V_FAST = 24.6   # 55 mph
# Early read (predicted exit) needs at least this box height.
EARLY_MIN_PX = 24.0
# Speed-scaled scan rate: about one proposer frame per SCAN_SPACING_M, clamped.
SCAN_SPACING_M = 12.0
SCAN_HZ_MIN = 0.5
SCAN_HZ_MAX = 2.5
# v2.1: never scan slower than this below SCAN_FLOOR_V (55 mph). At 27 mph
# v/12 m gave ~1 Hz, so a sign got one or two proposer looks (Oct 19 replay).
SCAN_HZ_FLOOR = 2.0
SCAN_FLOOR_V = 24.6
# v2.1: the upper (right road edge) strip runs on every UPPER_EVERY-th tick.
UPPER_EVERY = 4
# v2.1: proposer duty cap, all passes (main + upper + burst) in a trailing
# window. 25% of a core = #354's 2 Hz x ~124 ms on-car pass.
PROPOSER_DUTY_WINDOW_S = 10.0
PROPOSER_DUTY_BUDGET = 0.25
PROPOSER_PASS_S_DEFAULT = 0.124
# v2.3: a 1-hit track's confirm pass may use this much extra duty in the
# window. At the 2 Hz floor the 25% cap is already full (20 x 124 ms / 10 s),
# so the 5 Hz confirm burst never ran and a slow ~40 px sign seen once was
# lost (Oct 19 #15). Bursts are short; the long-run average stays ~25%.
PROPOSER_BURST_HEADROOM = 0.05
# Only at low speed (#15 was 10-15 m/s). Above this the sign closes fast
# enough that the 2 Hz floor already gets two hits, and extra passes only
# shift read timing.
PROPOSER_BURST_MAX_V = 15.5


def burst_headroom(tracker, t: float, v_ego: float | None) -> float:
  if v_ego is None or v_ego > PROPOSER_BURST_MAX_V:
    return 0.0
  return PROPOSER_BURST_HEADROOM if tracker.wants_fast_proposer(t) else 0.0
# v2.1: live confirmed track -> up to EXEMPT_READS_PER_TRACK reads that skip
# the reader pacing (4x infer + 25%/10 s), spaced EXEMPT_GAP_X x infer. The
# spans are still recorded, so the next steady read waits until the 10 s
# window is back under 25% (charged afterwards).
EXEMPT_READS_PER_TRACK = 3
EXEMPT_GAP_X = 1.0
# v2.1: a read on a box under REREAD_MAX_PX that comes back below REREAD_CONF
# (or empty) is retried once the box has grown by REREAD_GROW.
REREAD_MAX_PX = 56.0
REREAD_CONF = 0.85
REREAD_GROW = 1.25
# v2.2: planned reads. Range from the proposer box: d ~= FOCAL_PX * H / h.
# After TRACK_MIN_HITS, range + closing speed (GPS / modelV2, else the fit)
# give the time the sign reaches NAPSignReadDistM. No read before that; then a
# burst of BURST_READS full-res reads on consecutive scheduled frames, placed
# so it ends at least one frame before the predicted strip / frame exit. The
# value needs 2 agreeing reads, or 1 read >= STRONG_CONF on a box >= STRONG_PX.
# One fallback read only if the whole burst was empty and the sign is in view.
FOCAL_PX = 2648.0
SIGN_H_M = 0.76
READ_DIST_PARAM = "NAPSignReadDistM"
READ_DIST_DEFAULT_M = 40.0
# Per-track read distance = max(NAPSignReadDistM, exit distance + margin).
# Exit distance: where the sign (mount height / lateral offset recovered from
# its image position and range) leaves the strip top or the right edge.
# Margin: EXIT_MARGIN_FRAMES scheduled frames at the current speed.
EXIT_MARGIN_FRAMES = 1.5
CAM_CX = 1928 / 2.0
CAM_CY = 1208 / 2.0
READ_DIST_MIN_M = 15.0
READ_DIST_MAX_M = 60.0
BURST_READS = 3
FALLBACK_READS = 1
# v2.3 decision (votes are the main reader's CLASS mph, never the OCR's):
#  - pair: two reads of the track in one burst, same class mph, each at
#    >= PAIR_CONF, neither an OCR clash -> decide.
#  - single: class and OCR agree, conf >= STRONG_CONF on a box >= STRONG_PX,
#    or conf >= SOLO_HI_CONF on a box >= SOLO_HI_PX.
#  - an OCR clash (OCR above its floor naming another mph) votes for nothing.
#  - EXTRA_READS more burst reads when the votes name different mph.
STRONG_PX = 56.0
STRONG_CONF = 0.85
SOLO_HI_CONF = 0.88
SOLO_HI_PX = 35.0
PAIR_CONF = 0.60
EXTRA_READS = 1
CLASH_VOTES = False
SOLO_CLASS_ONLY = False
# Class-only single (OCR silent): off unless SOLO_CLASS_CONF > 0.
SOLO_CLASS_CONF = 0.0
SOLO_CLASS_PX = 0.0
# One more burst read when no read so far had OCR agreeing and nothing decided.
EXTRA_ON_SILENT = 0
READ_S_DEFAULT = 0.8
EXIT_HORIZON_S = 4.0
# Sharp reads allowed after the first mph, so a second HUD vote can land
# on the same sign. 0 restores "done on the first mph".
READS_AFTER_MPH = 1
NEG_NAMES = ("neg_yellow", "neg_other")
NEG_SUPPRESS_CONF = 0.40


def resize_area(img: np.ndarray, h: int, w: int) -> np.ndarray:
  if img.shape[0] == h and img.shape[1] == w:
    return img
  try:
    import cv2
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
  except Exception:
    from openpilot.selfdrive.speedsignd.nv12 import _resize_gray_bilinear
    return _resize_gray_bilinear(img, h, w)


def strip_rgb_from_crop(crop) -> np.ndarray:
  """NV12 strip crop -> 128x352 RGB uint8 (area resize, BT.601)."""
  from openpilot.selfdrive.speedsignd.nv12 import _bt601_yuv_to_rgb
  y_s = resize_area(crop.y, PROPOSER_IN_H, PROPOSER_IN_W)
  uv = crop.uv
  if uv is None or uv.size == 0 or uv.shape[1] < 2:
    return np.stack([y_s, y_s, y_s], axis=-1)
  u = resize_area(np.ascontiguousarray(uv[:, 0::2]), PROPOSER_IN_H, PROPOSER_IN_W)
  v = resize_area(np.ascontiguousarray(uv[:, 1::2]), PROPOSER_IN_H, PROPOSER_IN_W)
  return _bt601_yuv_to_rgb(y_s, u, v)


def _nms(boxes: np.ndarray, scores: np.ndarray, iou: float) -> list[int]:
  order = list(np.argsort(-scores))
  keep: list[int] = []
  while order:
    i = order.pop(0)
    keep.append(int(i))
    order = [j for j in order if box_iou(boxes[i], boxes[j]) < iou]
  return keep


def decode_proposals(raw: np.ndarray, crop_xywh, conf: float = PROPOSER_CONF,
                     in_w: int = PROPOSER_IN_W, in_h: int = PROPOSER_IN_H) -> list[tuple[float, float, float, float, float]]:
  """[1,5,N] (cx,cy,w,h,score) at model scale -> frame (x, y, w, h, score), top-left xywh."""
  out = np.asarray(raw)
  if out.ndim == 3:
    out = out[0]
  if out.shape[0] != 5 and out.shape[-1] == 5:
    out = out.T
  sc = out[4]
  idx = np.where(sc >= conf)[0]
  if idx.size == 0:
    return []
  cx0, cy0, cw, ch = (float(v) for v in crop_xywh)
  sx = cw / float(in_w)
  sy = ch / float(in_h)
  b = out[:4, idx].T
  boxes = np.stack([
    (b[:, 0] - b[:, 2] / 2) * sx + cx0,
    (b[:, 1] - b[:, 3] / 2) * sy + cy0,
    b[:, 2] * sx,
    b[:, 3] * sy,
  ], axis=1)
  s = sc[idx].astype(np.float32)
  keep = _nms(boxes, s, PROPOSER_IOU)[:8]
  return [(float(boxes[i, 0]), float(boxes[i, 1]), float(boxes[i, 2]), float(boxes[i, 3]), float(s[i])) for i in keep]


class Proposer:
  """onnxruntime CPU, one thread. None-safe: .ok is False when weights/runtime missing."""

  def __init__(self, path: str):
    self.path = path
    self.session = None
    self.error = ""
    self.load()

  def load(self) -> bool:
    if not os.path.isfile(self.path):
      self.error = "missing"
      return False
    try:
      import onnxruntime as ort
      so = ort.SessionOptions()
      so.intra_op_num_threads = 1
      so.inter_op_num_threads = 1
      so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
      self.session = ort.InferenceSession(self.path, so, providers=["CPUExecutionProvider"])
      self._inp = self.session.get_inputs()[0].name
      self.error = ""
      return True
    except Exception as e:
      self.session = None
      self.error = f"{type(e).__name__}: {e}"[:120]
      return False

  @property
  def ok(self) -> bool:
    return self.session is not None

  def propose(self, crop) -> list[tuple[float, float, float, float, float]]:
    if self.session is None or crop is None:
      return []
    rgb = strip_rgb_from_crop(crop)
    blob = rgb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    raw = self.session.run(None, {self._inp: blob})[0]
    return decode_proposals(raw, crop.crop)


def box_iou(a, b) -> float:
  ax0, ay0, aw, ah = a[0], a[1], a[2], a[3]
  bx0, by0, bw, bh = b[0], b[1], b[2], b[3]
  ix = max(0.0, min(ax0 + aw, bx0 + bw) - max(ax0, bx0))
  iy = max(0.0, min(ay0 + ah, by0 + bh) - max(ay0, by0))
  inter = ix * iy
  union = aw * ah + bw * bh - inter
  return float(inter / union) if union > 0 else 0.0


def _near(a, b) -> bool:
  acx, acy = a[0] + a[2] / 2, a[1] + a[3] / 2
  bcx, bcy = b[0] + b[2] / 2, b[1] + b[3] / 2
  tol = max(a[2], a[3], b[2], b[3]) + 40
  # Signs drift outward (right/up) and grow; allow a larger forward step.
  # At 2 Hz a near sign can move ~300 px right per sample; never far left.
  return -tol < (bcx - acx) < tol * 3.0 and abs(acy - bcy) < tol


@dataclass
class Track:
  tid: int
  box: tuple[float, float, float, float]
  score: float
  first_t: float
  last_t: float
  hits: int = 1
  peak_h: float = 0.0
  reads: int = 0
  got_mph: bool = False
  neg: bool = False
  done: bool = False
  reread_h: float = 0.0
  planned: bool = False
  burst_n: int = 0
  fallback_n: int = 0
  votes: list = field(default_factory=list)  # (mph, conf, h, sign)
  decided_mph: int = 0
  burst_start_t: float = 0.0
  read_dist_m: float = 0.0
  history: list = field(default_factory=list)

  def predicted(self, t: float) -> tuple[float, float, float, float]:
    """Constant growth / drift from the last two boxes.

    Not the read crop. Extrapolating past the last hit is what put the
    sharp read beside the sign. crop_box() is the measurement.
    """
    if len(self.history) < 2:
      return self.box
    (t0, b0), (t1, b1) = self.history[-2], self.history[-1]
    dt = max(1e-3, t1 - t0)
    k = min(2.0, (t - t1) / dt)
    return tuple(b1[i] + (b1[i] - b0[i]) * k for i in range(4))  # type: ignore[return-value]

  def read_box(self, t: float, v_ego: float | None = None) -> tuple[float, float, float, float]:
    """Box to centre a planned read on at time t.

    Measured this frame -> the measurement. Otherwise the centre drifts per
    the last two boxes and the size follows range: h(t) = h1 * d1 / d(t),
    d(t) = d1 - v*(t - t1) (v from GPS / modelV2; growth fit without it).
    """
    if abs(t - self.last_t) <= 1e-6 or len(self.history) < 2:
      return self.box
    (t0, b0), (t1, b1) = self.history[-2], self.history[-1]
    dt = max(1e-3, t1 - t0)
    k = min(2.0, (t - t1) / dt)
    cx = b1[0] + b1[2] / 2 + ((b1[0] + b1[2] / 2) - (b0[0] + b0[2] / 2)) * k
    cy = b1[1] + b1[3] / 2 + ((b1[1] + b1[3] / 2) - (b0[1] + b0[3] / 2)) * k
    d1 = FOCAL_PX * SIGN_H_M / max(1.0, b1[3])
    if v_ego is not None and float(v_ego) > 0.5:
      dtt = max(3.0, d1 - float(v_ego) * (t - t1))
      g = d1 / dtt
    else:
      g = max(0.5, 1.0 + (b1[3] / max(1.0, b0[3]) - 1.0) * k)
    w, h = b1[2] * g, b1[3] * g
    return (cx - w / 2, cy - h / 2, w, h)

  def crop_box(self) -> tuple[float, float, float, float]:
    """Last measured box, in frame pixels. The sharp-read crop uses this."""
    return self.box


class SignTracker:
  def __init__(self, target_m: float | None = None):
    self.tracks: list[Track] = []
    self._next = 1
    # v2.2: None keeps the v2.1 px triggers; a distance enables planned bursts.
    self.target_m: float | None = target_m      # NAPSignReadDistM (floor); None = v2.1 px triggers
    self.read_s = READ_S_DEFAULT

  @property
  def planning(self) -> bool:
    return self.target_m is not None

  def target_for(self, tr: Track, d_now: float, closing: float, v_ego: float | None, strip=None) -> float:
    """max(NAPSignReadDistM, exit distance + margin of 1.5 scheduled frames)."""
    d_exit = exit_distance_m(tr.box, d_now, strip)
    margin = closing * self.read_spacing_s(v_ego) * EXIT_MARGIN_FRAMES
    return max(float(self.target_m or READ_DIST_DEFAULT_M), d_exit + margin)

  def read_spacing_s(self, v_ego: float | None = None) -> float:
    """Start-to-start of consecutive burst reads (read + one proposer pass)."""
    return max(1.0 / scan_hz_for_speed(v_ego), float(self.read_s) + PROPOSER_PASS_S_DEFAULT)

  def plan(self, tr: Track, v_ego: float | None, strip=None) -> tuple[float, float] | None:
    """(burst start time, predicted exit time or inf) for a confirmed track."""
    if not self.planning or tr.hits < TRACK_MIN_HITS:
      return None
    fit = fit_range(tr.history, v_ego)
    if fit is None:
      return None
    t1, d1, closing = fit
    tr.read_dist_m = self.target_for(tr, d1, closing, v_ego, strip)
    t_target = t1 + (d1 - tr.read_dist_m) / closing
    sp = self.read_spacing_s(v_ego)
    # Burst ENDS at the read distance: the closest point still inside the
    # strip / frame with margin (largest box, least blur). The
    # NAPSignReadDistM floor keeps it from going nearer than that.
    start = t_target - sp * (BURST_READS - 1)
    # Geometric exit time too (the read distance already clears it by the margin).
    d_exit = exit_distance_m(tr.box, d1, strip)
    t_exit_geo = t1 + (d1 - d_exit) / closing
    period = 1.0 / scan_hz_for_speed(v_ego)
    start = min(start, t_exit_geo - period - self.read_s - sp * (BURST_READS - 1))
    t_exit = predicted_exit_t(tr, strip)
    if math.isfinite(t_exit):
      # Last read must start one frame + one read before the exit.
      period = 1.0 / scan_hz_for_speed(v_ego)
      latest = t_exit - period - self.read_s - sp * (BURST_READS - 1)
      start = min(start, latest)
    return start, t_exit

  def next_plan_t(self, t: float, v_ego: float | None, strip=None) -> float | None:
    best = None
    for tr in self.tracks:
      if not tr.planned or tr.burst_n > 0 or tr.done or tr.neg:
        continue
      pl = self.plan(tr, v_ego, strip)
      if pl is not None and pl[0] > t and (best is None or pl[0] < best):
        best = pl[0]
    return best

  def update(self, props, t: float) -> list[Track]:
    used = set()
    for tr in self.tracks:
      # A finished planned track keeps absorbing its sign so it is not
      # re-born as a new track and read again.
      if tr.done and not tr.planned:
        continue
      best, bi = 0.0, -1
      for i, p in enumerate(props):
        if i in used:
          continue
        v = box_iou(tr.box, p)
        if v < TRACK_IOU and _near(tr.box, p):
          v = max(v, TRACK_IOU)
        if v > best:
          best, bi = v, i
      if bi >= 0 and best >= TRACK_IOU:
        p = props[bi]
        used.add(bi)
        tr.box = (p[0], p[1], p[2], p[3])
        tr.score = max(tr.score, p[4])
        tr.hits += 1
        tr.last_t = t
        tr.peak_h = max(tr.peak_h, p[3])
        tr.history = (tr.history + [(t, tr.box)])[-4:]
    for i, p in enumerate(props):
      if i in used:
        continue
      tr = Track(tid=self._next, box=(p[0], p[1], p[2], p[3]), score=p[4], first_t=t, last_t=t, peak_h=p[3])
      tr.history = [(t, tr.box)]
      self._next += 1
      self.tracks.append(tr)
    self.tracks = [tr for tr in self.tracks if t - tr.last_t <= TRACK_MAX_GAP_S * 2 and not (tr.done and t - tr.last_t > 1.0)]
    return self.tracks

  def _readable(self, tr: Track) -> bool:
    if tr.hits >= TRACK_MIN_HITS:
      return True
    return tr.hits >= 1 and tr.score >= TRACK_SOLO_PROPOSAL_CONF

  def capture_candidate(self, t: float, *, v_ego: float | None = None, next_t: float | None = None,
                        strip=None) -> Track | None:
    """Track due for its sharp read now, or None. Biggest measured box first.

    The crop is always the box measured THIS frame (tr.last_t == t). A track
    that was not re-measured this frame is never read: its stored box is where
    the sign was, not where it is (Oct 19 "lost" reads cropped empty road).
    A track lost longer than LOST_S is retired.

    Timing (all optional; None keeps the fixed 60 px / right-edge triggers):
      v_ego   speed (GPS, then modelV2) -> speed-scaled read size, ~40 px at >= 55 mph
      next_t  time of the next proposer frame -> read now if Track.predicted()
              says the box leaves the strip / frame before then
      strip   (x, y, w, h) of the scanned area, used with next_t
    predicted() only times the read. It is never cropped.
    """
    need_px = read_px_for_speed(v_ego)
    due = []
    for tr in self.tracks:
      if (self.planning and tr.planned and not tr.done and not tr.neg and 0 < tr.burst_n < burst_len(tr.votes)
          and 1e-6 < t - tr.last_t <= LOST_S):
        due.append(tr)                  # burst frame the proposer missed: predicted crop
        continue
      if self.planning and not tr.done and not tr.neg and abs(t - tr.last_t) <= 1e-6:
        if tr.hits >= TRACK_MIN_HITS:
          tr.planned = True
        if tr.planned:
          if self._planned_due(tr, t, v_ego, next_t, strip):
            due.append(tr)
          continue
      if tr.planned:
        if not tr.done and t - tr.last_t > LOST_S:
          tr.done = True
        continue
      if tr.done or tr.neg or tr.reads >= MAX_READS_PER_TRACK or not self._readable(tr):
        continue
      if abs(t - tr.last_t) > 1e-6:
        if t - tr.last_t > LOST_S:
          tr.done = True
        continue
      x, _y, w, h = tr.box
      early = next_t is not None and h >= EARLY_MIN_PX and exits_before(tr, next_t, strip)
      if tr.reads > 0 and tr.reread_h > 0.0:
        # Small low-confidence read: wait for growth (or an exit / edge).
        again = h >= tr.reread_h or x + w >= CAPTURE_EDGE_X or early
      else:
        again = tr.reads > 0
      if h >= need_px or x + w >= CAPTURE_EDGE_X or early or again:
        due.append(tr)
    if not due:
      return None
    return max(due, key=lambda tr: tr.box[3])

  def _planned_due(self, tr: Track, t: float, v_ego, next_t, strip) -> bool:
    if tr.decided_mph:
      return False
    if 0 < tr.burst_n < burst_len(tr.votes):
      return True                       # consecutive scheduled frames
    if tr.burst_n >= BURST_READS:
      empty = not any(v[0] for v in tr.votes)
      return empty and tr.fallback_n < FALLBACK_READS
    x, _y, w, h = tr.box
    if x + w >= CAPTURE_EDGE_X or (next_t is not None and exits_before(tr, next_t, strip)):
      return True                       # last frame before exit
    pl = self.plan(tr, v_ego, strip)
    if pl is None:
      return h >= read_px_for_speed(v_ego)
    half = 0.5 / scan_hz_for_speed(v_ego)
    return t + half >= pl[0]            # frame nearest the planned start

  def in_burst(self, t: float) -> bool:
    """A planned burst has reads left on a live track (no gap between them)."""
    for tr in self.tracks:
      if tr.planned and not tr.done and not tr.neg and 0 < tr.burst_n and t - tr.last_t <= LOST_S:
        if tr.burst_n < burst_len(tr.votes) or (not any(v[0] for v in tr.votes) and tr.fallback_n < FALLBACK_READS):
          return True
    return False

  def budget_exempt(self, t: float) -> bool:
    """A confirmed (seen-twice) track is live and still owed a read.

    While true, the reader's cpu-load / cpu-budget rest is skipped so the
    1-3 s a sign is visible is not spent resting. Bounded by
    MAX_READS_PER_TRACK per sign; the modeld-lag pause still applies.
    """
    for tr in self.tracks:
      if tr.done or tr.neg or tr.hits < TRACK_MIN_HITS:
        continue
      if tr.planned:
        # Planned burst / fallback reads skip the pacing; the plan is the limit.
        if not tr.decided_mph and tr.reads < BURST_READS + EXTRA_READS + FALLBACK_READS and t - tr.last_t <= LOST_S:
          return True
        continue
      if tr.reads >= min(MAX_READS_PER_TRACK, EXEMPT_READS_PER_TRACK):
        continue
      if t - tr.last_t <= LOST_S:
        return True
    return False

  def wants_fast_proposer(self, t: float) -> bool:
    """True while a not-yet-confirmed track is still inside the gap window."""
    for tr in self.tracks:
      if tr.done or tr.neg or tr.hits >= TRACK_MIN_HITS:
        continue
      if t - tr.last_t <= TRACK_MAX_GAP_S:
        return True
    return False

  def proposer_period_s(self, t: float, v_ego: float | None = None, strip=None) -> float:
    hz = scan_hz_for_speed(v_ego)
    if self.wants_fast_proposer(t):
      hz = max(hz, PROPOSER_BURST_HZ)
    if hz <= 0.0:
      hz = PROPOSER_HZ
    per = 1.0 / float(hz)
    nxt = self.next_plan_t(t, v_ego, strip) if self.planning else None
    if nxt is not None and t < nxt < t + per:
      per = max(1.0 / PROPOSER_BURST_HZ, nxt - t)   # land a frame on the plan
    return per

  def mark_planned_read(self, tr: Track, signs, read_h: float | None, neg: bool = False, t: float = 0.0) -> list:
    """Record a planned read; return the signs released to the HUD (often [])."""
    tr.reads += 1
    if tr.burst_n < burst_len(tr.votes):
      if tr.burst_n == 0:
        tr.burst_start_t = float(t)
      tr.burst_n += 1
    else:
      tr.fallback_n += 1
    tr.votes.append(read_vote(signs, float(read_h or 0.0)))
    if neg:
      tr.neg = True
      tr.done = True
      return []
    out = burst_decision(tr.votes)
    if out:
      tr.decided_mph = int(out[0].mph)
      tr.got_mph = True
      tr.done = True
      return out
    if tr.burst_n >= burst_len(tr.votes):
      empty = not any(v[0] for v in tr.votes)
      if not empty or tr.fallback_n >= FALLBACK_READS:
        tr.done = True
    return []

  def mark_read(self, tr: Track, *, got_mph: bool, neg: bool,
                read_h: float | None = None, top_conf: float | None = None) -> None:
    tr.reads += 1
    small = read_h is not None and float(read_h) < REREAD_MAX_PX
    if small and float(top_conf or 0.0) < REREAD_CONF:
      tr.reread_h = float(read_h) * REREAD_GROW  # type: ignore[arg-type]
    else:
      tr.reread_h = 0.0
    if got_mph:
      tr.got_mph = True
    if neg:
      tr.neg = True
      tr.done = True
    elif tr.reads >= MAX_READS_PER_TRACK:
      tr.done = True
    elif tr.got_mph and tr.reads >= 1 + READS_AFTER_MPH:
      tr.done = True


def select_read_target(tracker: SignTracker, props, t: float, *, allow_read: bool,
                       v_ego: float | None = None, next_t: float | None = None, strip=None,
                       allow_exempt: bool = False) -> Track | None:
  """Apply this frame's proposals, then pick the sharp-read track.

  Call this before cropping. The returned track's crop_box() is the
  measurement from `props`, not the box predicted from older hits.
  `allow_read` False still updates the tracker (so the seen-twice gate
  and the burst cadence advance) and returns None.
  """
  tracker.update(props, t)
  if not allow_read and not (allow_exempt and tracker.budget_exempt(t)):
    return None
  return tracker.capture_candidate(t, v_ego=v_ego, next_t=next_t, strip=strip)


def frame_bank_rect(frame_w: int = 1928, frame_h: int = 1208, rect=None) -> tuple[int, int, int, int]:
  """Strip expanded by half the max read crop, even-aligned, clamped.

  The worker proposes on STRIP_RECT, then subcrops the sharp read from
  this bank of the same frame. No second proposer pass.
  """
  x, y, w, h = STRIP_RECT if rect is None else rect
  pad = CAPTURE_MAX_SIDE // 2
  x0 = int(x) - pad
  y0 = int(y) - pad
  x1 = int(x) + int(w) + pad
  y1 = int(y) + int(h) + pad
  x0 -= x0 % 2
  y0 -= y0 % 2
  if x1 % 2:
    x1 += 1
  if y1 % 2:
    y1 += 1
  fw, fh = int(frame_w), int(frame_h)
  x0 = max(0, min(x0, max(0, fw - 2)))
  y0 = max(0, min(y0, max(0, fh - 2)))
  x1 = min(fw, max(x0 + 2, x1))
  y1 = min(fh, max(y0 + 2, y1))
  bw = x1 - x0
  bh = y1 - y0
  bw -= bw % 2
  bh -= bh % 2
  return x0, y0, max(2, bw), max(2, bh)


def capture_rect(box, frame_w: int = 1928, frame_h: int = 1208) -> tuple[int, int, int, int]:
  x, y, w, h = (float(v) for v in box)
  side = int(min(CAPTURE_MAX_SIDE, max(CAPTURE_MIN_SIDE, CAPTURE_SCALE * max(w, h))))
  side = min(side, frame_w, frame_h)
  side -= side % 2
  cx, cy = x + w / 2, y + h / 2
  x0 = int(min(max(0, cx - side / 2), frame_w - side))
  y0 = int(min(max(0, cy - side / 2), frame_h - side))
  x0 -= x0 % 2
  y0 -= y0 % 2
  return x0, y0, side, side


def neg_read(diag: dict | None) -> bool:
  if not diag:
    return False
  name = str(diag.get("peak_name") or "")
  try:
    conf = float(diag.get("peak_conf") or 0.0)
  except Exception:
    conf = 0.0
  return name in NEG_NAMES and conf >= NEG_SUPPRESS_CONF


def speed_from_sm(sm) -> float | None:
  """Ego speed without carState: GPS speed first, then modelV2 velocity."""
  for svc in ("gpsLocationExternal", "gpsLocation"):
    try:
      g = sm[svc]
      if bool(getattr(g, "hasFix", False)) or int(getattr(g, "flags", 0) or 0) & 1:
        return float(g.speed)
    except Exception:
      pass
  try:
    vx = sm["modelV2"].velocity.x
    if len(vx):
      return float(vx[0])
  except Exception:
    pass
  return None


def proposals_allowed(speed_ms: float | None) -> bool:
  """Skip the proposer when stopped. Unknown speed is not 'stopped'."""
  return speed_ms is None or speed_ms >= PROPOSER_MIN_SPEED_MS


# ---------------------------------------------------------------- feedback
FEEDBACK_DIRNAME = "speedsign_feedback"
PENDING_JSON = "pending.json"
PENDING_JPG = "pending.jpg"
FEEDBACK_JSONL = "feedback.jsonl"
CONFIRMED_JSONL = "confirmed.jsonl"
FEEDBACK_MAX_BYTES = 50 * 1024 * 1024


def feedback_dir() -> str:
  override = os.environ.get("NAP_SPEED_SIGN_FEEDBACK")
  if override:
    return override
  from openpilot.selfdrive.speedsignd.paths import nap_data_dir
  return os.path.join(nap_data_dir(), FEEDBACK_DIRNAME)


def crop_jpeg_bytes(crop) -> bytes | None:
  try:
    import cv2
    from openpilot.selfdrive.speedsignd.nv12 import _rgb_from_resized_nv12
    rgb = _rgb_from_resized_nv12(crop.y, crop.uv)
    ok, enc = cv2.imencode(".jpg", rgb[:, :, ::-1], [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    return bytes(enc) if ok else None
  except Exception:
    return None


def write_pending(meta: dict, jpg: bytes | None, d: str | None = None) -> bool:
  """speedsignd: the read now on the plate (overwritten each new plate). Atomic."""
  d = d or feedback_dir()
  try:
    os.makedirs(d, exist_ok=True)
    if jpg:
      with open(os.path.join(d, PENDING_JPG + ".tmp"), "wb") as f:
        f.write(jpg)
      os.replace(os.path.join(d, PENDING_JPG + ".tmp"), os.path.join(d, PENDING_JPG))
    m = dict(meta)
    m["has_jpg"] = bool(jpg)
    with open(os.path.join(d, PENDING_JSON + ".tmp"), "w") as f:
      json.dump(m, f)
    os.replace(os.path.join(d, PENDING_JSON + ".tmp"), os.path.join(d, PENDING_JSON))
    return True
  except Exception:
    return False


def _dir_bytes(d: str) -> int:
  n = 0
  try:
    for e in os.scandir(d):
      if e.is_file():
        n += e.stat().st_size
  except Exception:
    pass
  return n


REALDATA_PREFIX = "speedsign_"
_SEG_RE = re.compile(r"^[0-9a-f]{8}--[0-9a-f]{10}--\d+$")
CURRENT_SEG_MAX_AGE_S = 300.0


def current_segment_dir(root: str | None = None, now: float | None = None) -> str | None:
  """Newest loggerd segment folder under realdata, if written in the last 5 min."""
  if root is None:
    try:
      from openpilot.system.hardware.hw import Paths
      root = Paths.log_root()
    except Exception:
      return None
  now = time.time() if now is None else now  # noqa: TID251  compare with file mtimes
  best, best_m = None, -1.0
  try:
    with os.scandir(root) as it:
      for e in it:
        if not (_SEG_RE.match(e.name) and e.is_dir(follow_symlinks=False)):
          continue
        m = e.stat(follow_symlinks=False).st_mtime
        if m > best_m:
          best, best_m = e.path, m
  except OSError:
    return None
  if best is None or now - best_m > CURRENT_SEG_MAX_AGE_S:
    return None
  return best


def copy_to_realdata(rec: dict, line: str, d: str, root: str | None = None, now: float | None = None) -> str | None:
  """Mirror one feedback row (+ crop) into the current route segment folder.

  athena uploadFilesToUrls only reaches files under realdata, so harvest can
  pull speedsign_feedback.jsonl / speedsign_<stamp>_<yes|no>_<mph>.jpg per
  segment. The uploader never auto-sends these names (not qlog/qcamera).
  """
  seg = current_segment_dir(root, now=now)
  if seg is None:
    return None
  try:
    with open(os.path.join(seg, REALDATA_PREFIX + FEEDBACK_JSONL), "a") as f:
      f.write(line)
    jpg = rec.get("jpg")
    if jpg and os.path.isfile(os.path.join(d, jpg)):
      shutil.copyfile(os.path.join(d, jpg), os.path.join(seg, REALDATA_PREFIX + jpg))
    return os.path.basename(seg)
  except OSError:
    return None


def record_answer(displayed_mph: int, yes: bool, d: str | None = None, now: float | None = None) -> dict | None:
  """UI Yes/No: append feedback.jsonl (+confirmed.jsonl on Yes), keep the crop JPEG.

  Uses the pending read only when its mph matches the plate the driver saw.
  No upload, no cruise / HUD / map-speed change.
  """
  d = d or feedback_dir()
  now = time.time() if now is None else now  # noqa: TID251  wall clock for feedback rows
  try:
    os.makedirs(d, exist_ok=True)
    meta: dict = {}
    try:
      with open(os.path.join(d, PENDING_JSON)) as f:
        meta = json.load(f)
    except Exception:
      meta = {}
    if int(meta.get("mph", -1)) != int(displayed_mph):
      meta = {"mph": int(displayed_mph), "stale_pending": True}
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
    rec = {
      "t": round(now, 3),
      "answer": "yes" if yes else "no",
      "displayed_mph": int(displayed_mph),
      "class": meta.get("class"),
      "conf": meta.get("conf"),
      "refine_mph": meta.get("refine_mph"),
      "refine_conf": meta.get("refine_conf"),
      "track_id": meta.get("track_id"),
      "read_t": meta.get("t"),
      "lat": meta.get("lat"),
      "lon": meta.get("lon"),
      "bearing": meta.get("bearing"),
      "gps_ok": meta.get("gps_ok"),
      "crop": meta.get("crop"),
      "stale_pending": bool(meta.get("stale_pending", False)),
    }
    src = os.path.join(d, PENDING_JPG)
    if meta.get("has_jpg") and not rec["stale_pending"] and os.path.isfile(src) and _dir_bytes(d) < FEEDBACK_MAX_BYTES:
      name = f"{stamp}_{rec['answer']}_{int(displayed_mph)}.jpg"
      shutil.copyfile(src, os.path.join(d, name))
      rec["jpg"] = name
    line = json.dumps(rec, separators=(",", ":")) + "\n"
    with open(os.path.join(d, FEEDBACK_JSONL), "a") as f:
      f.write(line)
    if yes and not rec["stale_pending"]:
      with open(os.path.join(d, CONFIRMED_JSONL), "a") as f:
        f.write(line)
    rec["realdata"] = copy_to_realdata(rec, line, d, now=now)
    return rec
  except Exception:
    return None


def read_px_for_speed(v_ego: float | None) -> float:
  """Box height (px) that triggers the sharp read. None -> CAPTURE_MIN_H."""
  if v_ego is None:
    return float(CAPTURE_MIN_H)
  v = float(v_ego)
  if v <= READ_V_SLOW:
    return min(float(CAPTURE_MIN_H), READ_PX_SLOW)
  if v >= READ_V_FAST:
    return min(float(CAPTURE_MIN_H), READ_PX_FAST)
  f = (v - READ_V_SLOW) / (READ_V_FAST - READ_V_SLOW)
  return min(float(CAPTURE_MIN_H), READ_PX_SLOW + f * (READ_PX_FAST - READ_PX_SLOW))


def scan_hz_for_speed(v_ego: float | None) -> float:
  """~1 proposer frame per SCAN_SPACING_M, clamped. None -> PROPOSER_HZ."""
  if v_ego is None:
    return float(PROPOSER_HZ)
  v = float(v_ego)
  lo = SCAN_HZ_FLOOR if v < SCAN_FLOOR_V else SCAN_HZ_MIN
  return min(SCAN_HZ_MAX, max(lo, v / SCAN_SPACING_M))


def range_m(h_px: float) -> float:
  return FOCAL_PX * SIGN_H_M / max(1.0, float(h_px))


def fit_range(history, v_ego: float | None):
  """(t_last, range at t_last, closing m/s) from box heights; None if unusable.

  Closing speed is the vehicle speed when known, else the least-squares slope.
  The range at t_last is the mean of every sample carried forward at that speed.
  """
  pts = [(float(t), range_m(b[3])) for t, b in history]
  if len(pts) < 2:
    return None
  t1 = pts[-1][0]
  closing = None
  if v_ego is not None and float(v_ego) > 0.5:
    closing = float(v_ego)
  else:
    n = len(pts)
    mt = sum(p[0] for p in pts) / n
    md = sum(p[1] for p in pts) / n
    den = sum((p[0] - mt) ** 2 for p in pts)
    if den > 1e-6:
      slope = sum((p[0] - mt) * (p[1] - md) for p in pts) / den
      if -slope > 0.5:
        closing = -slope
  if closing is None:
    return None
  d1 = sum(d - closing * (t1 - t) for t, d in pts) / len(pts)
  return t1, d1, closing


def predicted_exit_t(tr: Track, strip=None, frame_w: int = 1928, horizon_s: float = EXIT_HORIZON_S) -> float:
  """Linear (unclamped) extrapolation of the last two boxes -> first time out."""
  if len(tr.history) < 2:
    return math.inf
  (t0, b0), (t1, b1) = tr.history[-2], tr.history[-1]
  dt = max(1e-3, t1 - t0)
  sx, sy, sw, sh = strip if strip is not None else STRIP_RECT
  right = min(float(frame_w), float(sx + sw))
  step = 0.05
  k = step
  while k <= horizon_s:
    f = k / dt
    px, py, pw, ph = (b1[i] + (b1[i] - b0[i]) * f for i in range(4))
    if py < sy or px + pw > right or py + ph > sy + sh:
      return t1 + k
    k += step
  return math.inf


def read_vote(signs, h: float) -> tuple:
  """(class mph or 0, class conf, box px, sign, kind) for one burst read.

  kind: "agree" (OCR names the class mph), "class" (OCR silent / below its
  floor), "clash" (OCR above its floor names another mph), "none".
  The vote is always the main reader's class mph, never the OCR's.
  """
  from openpilot.selfdrive.speedsignd.detect_types import MUTCD_MPH
  from openpilot.selfdrive.speedsignd.yolo import REFINE_OVERRIDE_CONF
  top = max(signs or [], key=lambda s: float(getattr(s, "class_conf", 0.0) or s.conf), default=None)
  if top is None:
    return (0, 0.0, float(h), None, "none")
  cls = getattr(top, "class_mph", None)
  if cls is None:
    cls, cc = int(top.mph), float(top.conf)
  else:
    cls, cc = int(cls), float(getattr(top, "class_conf", 0.0) or top.conf)
  ref = getattr(top, "refine_mph", None)
  rc = float(getattr(top, "refine_conf", 0.0) or 0.0)
  kind = "class"
  if ref is not None:
    try:
      ri = int(ref)
      if ri in MUTCD_MPH and rc >= REFINE_OVERRIDE_CONF:
        kind = "agree" if ri == cls else "clash"
    except (TypeError, ValueError):
      pass
  if cls not in MUTCD_MPH:
    return (0, cc, float(h), top, "none")
  if kind == "clash" and not CLASH_VOTES:
    return (0, cc, float(h), top, "clash")
  from dataclasses import replace as _replace
  try:
    sign = _replace(top, mph=cls, conf=cc)
  except Exception:
    sign = top
  return (cls, cc, float(h), sign, kind)


def burst_len(votes) -> int:
  """BURST_READS, plus EXTRA_READS once the votes name different mph."""
  named = {v[0] for v in votes if v[0]}
  n = BURST_READS + (EXTRA_READS if len(named) >= 2 else 0)
  if EXTRA_ON_SILENT and len(votes) >= BURST_READS and named and not burst_decision(votes) \
     and not any(len(v) >= 5 and v[4] == "agree" for v in votes):
    n += EXTRA_ON_SILENT
  return n


def _vote(v):
  return v if len(v) >= 5 else (v[0], v[1], v[2], v[3], "agree")


def burst_decision(votes) -> list:
  """Signs to release (as a proposal): one strong agreeing read, or a pair."""
  vs = [_vote(v) for v in votes]
  for mph, conf, h, sign, kind in vs:
    if not mph or sign is None:
      continue
    if kind == "agree" or (SOLO_CLASS_ONLY and kind == "class"):
      if (conf >= STRONG_CONF and h >= STRONG_PX) or (conf >= SOLO_HI_CONF and h >= SOLO_HI_PX):
        return [sign]
    if kind == "class" and SOLO_CLASS_CONF > 0.0 and conf >= SOLO_CLASS_CONF and h >= SOLO_CLASS_PX:
      return [sign]
  by: dict[int, list] = {}
  for mph, conf, _h, sign, kind in vs:
    if mph and sign is not None and conf >= PAIR_CONF and kind in ("agree", "class"):
      by.setdefault(mph, []).append(sign)
  agreed = [v for v in by.values() if len(v) >= 2]
  if len(agreed) == 1:
    return agreed[0][:2]
  return []


def exit_distance_m(box, d_now: float, strip=None, frame_w: int = 1928) -> float:
  """Range at which a sign at `box` (now at d_now) leaves the strip top or right edge.

  Back-projects the box to a mount height Y (above the optical axis) and a
  lateral offset X at d_now, then finds where its top edge reaches the strip
  top and its right edge reaches the strip / frame right. Signs only grow up
  and right as they approach, so those are the exits. 0 when it never exits.
  """
  x, y, w, h = (float(v) for v in box)
  sx, sy, sw, _sh = strip if strip is not None else STRIP_RECT
  right = min(float(frame_w), float(sx + sw))
  d = max(1.0, float(d_now))
  top_m = (CAM_CY - y) * d / FOCAL_PX               # sign top above the axis (m)
  right_m = (x + w - CAM_CX) * d / FOCAL_PX         # sign right edge, right of axis (m)
  out = 0.0
  if top_m > 0.0 and CAM_CY - sy > 1.0:
    out = max(out, FOCAL_PX * top_m / (CAM_CY - float(sy)))
  if right_m > 0.0 and right - CAM_CX > 1.0:
    out = max(out, FOCAL_PX * right_m / (right - CAM_CX))
  return out


def clamp_read_dist(v) -> float:
  try:
    d = float(v)
  except Exception:
    return READ_DIST_DEFAULT_M
  if not math.isfinite(d):
    return READ_DIST_DEFAULT_M
  return min(READ_DIST_MAX_M, max(READ_DIST_MIN_M, d))


def reader_next_after(*, end: float, infer_s: float, steady: float, duty_t: float,
                      exempt: bool, empty: bool) -> float:
  """Earliest next sharp-read start after a read that ended at `end`.

  exempt (a live confirmed track still owed a read): end + EXEMPT_GAP_X x
  infer, or `end` after an empty read. Not paced, not duty-gated; the read
  is in the duty spans, so `steady` reads after the track wait it off.
  empty, not exempt: no pacing gap, still inside the 25%/10 s cap (duty_t).
  Otherwise `steady` (schedule_after_read).
  """
  if exempt:
    return float(end) if empty else float(end) + EXEMPT_GAP_X * max(0.0, float(infer_s))
  if empty:
    return max(float(end), float(duty_t))
  return float(steady)


def top_conf(signs) -> float:
  try:
    return max((float(s.conf) for s in signs), default=0.0)
  except Exception:
    return 0.0


class ProposerDuty:
  """Proposer passes in a trailing window, capped at PROPOSER_DUTY_BUDGET.

  Cost per pass is the measured pass time (on car ~124 ms); before the first
  measurement PROPOSER_PASS_S_DEFAULT.
  """

  def __init__(self, window_s: float = PROPOSER_DUTY_WINDOW_S, budget: float = PROPOSER_DUTY_BUDGET):
    self.window_s = float(window_s)
    self.budget = float(budget)
    self.spans: list[tuple[float, float]] = []  # (start, seconds)
    self.pass_s = PROPOSER_PASS_S_DEFAULT

  def _prune(self, t: float) -> None:
    self.spans = [(a, d) for a, d in self.spans if t - a < self.window_s]

  def used(self, t: float) -> float:
    self._prune(t)
    return sum(d for _a, d in self.spans)

  def fits(self, t: float, passes: int = 1, headroom: float = 0.0) -> bool:
    return self.used(t) + passes * self.pass_s <= self.window_s * (self.budget + headroom) + 1e-6

  def next_fit(self, t: float, passes: int = 1) -> float:
    """Earliest time >= t at which `passes` more passes fit."""
    if self.fits(t, passes):
      return float(t)
    for a, _d in sorted(self.spans):
      tt = a + self.window_s
      if tt > t and self.fits(tt, passes):
        return tt
    return float(t) + self.window_s

  def add(self, t: float, passes: int, seconds: float | None = None) -> None:
    n = max(1, int(passes))
    if seconds is not None and seconds > 0.0:
      self.pass_s = 0.8 * self.pass_s + 0.2 * (float(seconds) / n)
    self.spans.append((float(t), n * self.pass_s))
    self._prune(t)


def exits_before(tr: Track, next_t: float, strip=None, frame_w: int = 1928) -> bool:
  """True if Track.predicted(next_t) leaves the scanned area or the frame.

  Uses constant growth / drift from the last two measured boxes. Needs two
  boxes; otherwise False (wait for the normal triggers).
  """
  if len(tr.history) < 2:
    return False
  px, py, pw, ph = tr.predicted(next_t)
  sx, sy, sw, sh = strip if strip is not None else STRIP_RECT
  right = min(float(frame_w), float(sx + sw))
  return py < sy or px + pw > right or py + ph > sy + sh


def union_rect(a, b):
  """Smallest rect covering a and b (b may be None)."""
  if b is None:
    return a
  x0, y0 = min(a[0], b[0]), min(a[1], b[1])
  x1, y1 = max(a[0] + a[2], b[0] + b[2]), max(a[1] + a[3], b[1] + b[3])
  return (x0, y0, x1 - x0, y1 - y0)
