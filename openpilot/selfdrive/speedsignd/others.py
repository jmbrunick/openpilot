"""Other-sign reads on the shared find -> track -> read pipeline (NAPSignOthers).

Display / log only. The speed-sign proposer, SignTracker and the sharp-read
crop (one copy out of the shared VisionIPC frame) are reused; this module
only adds a per-family reader on the same crop and per-family confirmation.

  families  stop, yield, do_not_enter, school, railroad, curve
  readers   FamilyReader interface. The shipped OnnxFamilyReader loads an
            optional classifier (paths.default_other_signs_path()). No file,
            no runtime, or a bad file -> .ok False and every read is a no-op.
            No weights are shipped (see PLAN section 9: Apache detector, own
            data + BDD100K; no AGPL or non-commercial weights).

Nothing here publishes or feeds longitudinal or lateral control. A stop
confirmation is cross-checked against the driving model's own stop plan
(modelV2 velocity going to ~0) and only reported.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

FAMILIES = ("stop", "yield", "do_not_enter", "school", "railroad", "curve")
# Classifier output order (index -> family/detail). "none" = not one of ours.
OTHER_CLASS_NAMES = (
  "none", "stop", "yield", "do_not_enter", "school", "railroad", "curve_left", "curve_right",
)
READ_MIN_CONF = 0.50
SOLO_STOP_CONF = 0.85
WARN_HUD_CONF = 0.70
STOP_PLAN_V = 1.0       # m/s: model plan minimum speed counted as a stop
STOP_PLAN_MIN_EGO = 3.0  # m/s: only meaningful while moving


@dataclass(frozen=True)
class FamilyRead:
  family: str
  conf: float
  detail: str = ""


class FamilyReader(Protocol):
  ok: bool

  def read(self, crop) -> FamilyRead | None: ...


def _rgb_square(crop, size: int) -> np.ndarray | None:
  """NV12 crop -> size x size RGB float [0,1] (nearest resize, BT.601)."""
  y = getattr(crop, "y", None)
  if y is None or y.size == 0:
    return None
  h, w = y.shape[:2]
  ys = (np.arange(size) * h // size).clip(0, h - 1)
  xs = (np.arange(size) * w // size).clip(0, w - 1)
  yy = y[ys][:, xs].astype(np.float32)
  uv = getattr(crop, "uv", None)
  if uv is not None and uv.size and uv.shape[1] >= 2:
    uh, uw = uv.shape[0], uv.shape[1] // 2
    uys = (np.arange(size) * uh // size).clip(0, uh - 1)
    uxs = (np.arange(size) * uw // size).clip(0, uw - 1)
    u = uv[:, 0::2][uys][:, uxs].astype(np.float32) - 128.0
    v = uv[:, 1::2][uys][:, uxs].astype(np.float32) - 128.0
  else:
    u = v = np.zeros_like(yy)
  r = yy + 1.402 * v
  g = yy - 0.344 * u - 0.714 * v
  b = yy + 1.772 * u
  return (np.stack([r, g, b], 0).clip(0, 255) / 255.0)[None].astype(np.float32)


class OnnxFamilyReader:
  """Optional ONNX classifier, onnxruntime CPU, one thread. No-op when absent."""

  def __init__(self, path: str, names=OTHER_CLASS_NAMES):
    self.path = path
    self.names = tuple(names)
    self.session = None
    self.size = 64
    self.error = ""
    self.load()

  @property
  def ok(self) -> bool:
    return self.session is not None

  def load(self) -> bool:
    if not self.path or not os.path.isfile(self.path):
      self.error = "missing"
      return False
    try:
      import onnxruntime as ort
      so = ort.SessionOptions()
      so.intra_op_num_threads = 1
      so.inter_op_num_threads = 1
      sess = ort.InferenceSession(self.path, so, providers=["CPUExecutionProvider"])
      inp = sess.get_inputs()[0]
      shp = list(inp.shape)
      if len(shp) == 4 and isinstance(shp[2], int) and shp[2] > 0:
        self.size = int(shp[2])
      meta = sess.get_modelmeta().custom_metadata_map or {}
      if "names" in meta:
        self.names = tuple(n.strip() for n in meta["names"].split(","))
      self.session, self._inp, self.error = sess, inp.name, ""
      return True
    except Exception as e:
      self.session = None
      self.error = f"{type(e).__name__}: {e}"[:120]
      return False

  def read(self, crop) -> FamilyRead | None:
    if self.session is None or crop is None:
      return None
    blob = _rgb_square(crop, self.size)
    if blob is None:
      return None
    try:
      out = np.asarray(self.session.run(None, {self._inp: blob})[0]).reshape(-1)
    except Exception:
      return None
    if out.size != len(self.names):
      return None
    if out.min() < 0.0 or out.max() > 1.0 or abs(float(out.sum()) - 1.0) > 1e-2:
      e = np.exp(out - out.max())
      out = e / e.sum()
    i = int(np.argmax(out))
    return parse_class(self.names[i], float(out[i]))


def parse_class(name: str, conf: float) -> FamilyRead | None:
  if name.startswith("curve"):
    return FamilyRead("curve", conf, name.split("_", 1)[1] if "_" in name else "")
  if name in FAMILIES:
    return FamilyRead(name, conf)
  return None


def model_stop_plan(md, v_ego: float | None) -> bool:
  """Driving model's own plan slows to ~0 within its horizon (cross-check only)."""
  try:
    vx = [float(v) for v in md.velocity.x]
  except Exception:
    return False
  if not vx or v_ego is None or float(v_ego) < STOP_PLAN_MIN_EGO:
    return False
  return min(vx) < STOP_PLAN_V


@dataclass
class TrackVotes:
  reads: list = field(default_factory=list)  # FamilyRead

  def top(self) -> tuple[str, str, int, float]:
    """(family, detail, agreeing reads, best conf) over reads >= READ_MIN_CONF."""
    good = [r for r in self.reads if r.conf >= READ_MIN_CONF]
    if not good:
      return "", "", 0, 0.0
    fams: dict[tuple[str, str], list[float]] = {}
    for r in good:
      fams.setdefault((r.family, r.detail), []).append(r.conf)
    (fam, det), confs = max(fams.items(), key=lambda kv: (len(kv[1]), max(kv[1])))
    return fam, det, len(confs), max(confs)


def decide(family: str, n: int, conf: float, stop_plan: bool) -> tuple[bool, bool]:
  """Per-family (hud, confirmed). PLAN.md section 8 rules; no map nodes yet."""
  if not family or n <= 0:
    return False, False
  if family == "stop":
    ok = n >= 2 or (conf >= SOLO_STOP_CONF and stop_plan)
    return ok, ok
  if family in ("yield", "do_not_enter"):
    ok = n >= 2
    return ok, ok
  # school, railroad, curve: HUD on one confident read, log on two.
  return (n >= 2 or conf >= WARN_HUD_CONF), n >= 2


@dataclass
class OthersState:
  enabled: bool = False
  reader_missing: bool = True
  family: str = ""
  conf: float = 0.0
  track_id: int = 0
  hud: bool = False
  confirmed: bool = False
  reads: int = 0
  model_stop_plan: bool = False
  detail: str = ""


class OtherSigns:
  """Per-track family votes on the shared tracker's track ids."""

  def __init__(self, reader: FamilyReader | None = None):
    self.reader = reader
    self.enabled = False
    self.votes: dict[int, TrackVotes] = {}
    self.state = OthersState()

  @property
  def active(self) -> bool:
    return bool(self.enabled and self.reader is not None and self.reader.ok)

  def set_enabled(self, on: bool) -> None:
    self.enabled = bool(on)
    if not on:
      self.votes.clear()
    self.state.enabled = self.enabled
    self.state.reader_missing = not (self.reader is not None and self.reader.ok)

  def on_read(self, crop, track_id: int | None, *, stop_plan: bool) -> OthersState:
    """Classify the sharp-read crop of a track. No-op unless active."""
    self.state.enabled = self.enabled
    self.state.reader_missing = not (self.reader is not None and self.reader.ok)
    if not self.active or crop is None or track_id is None:
      return self.state
    r = self.reader.read(crop)  # type: ignore[union-attr]
    if r is None:
      return self.state
    tv = self.votes.setdefault(int(track_id), TrackVotes())
    tv.reads.append(r)
    fam, det, n, conf = tv.top()
    hud, confirmed = decide(fam, n, conf, stop_plan)
    if fam:
      self.state = OthersState(
        enabled=True, reader_missing=False, family=fam, conf=round(conf, 3), track_id=int(track_id),
        hud=hud, confirmed=confirmed, reads=n, model_stop_plan=bool(stop_plan), detail=det,
      )
    return self.state

  def prune(self, live_ids) -> None:
    keep = {int(i) for i in live_ids}
    for k in list(self.votes):
      if k not in keep:
        del self.votes[k]


def fill_message(msg, st: OthersState) -> None:
  """Copy state into cereal OtherSignsNAP (display / log only)."""
  msg.enabled = bool(st.enabled)
  msg.readerMissing = bool(st.reader_missing)
  msg.family = st.family
  msg.conf = float(st.conf)
  msg.trackId = int(st.track_id)
  msg.hud = bool(st.hud)
  msg.confirmed = bool(st.confirmed)
  msg.reads = int(min(255, st.reads))
  msg.modelStopPlan = bool(st.model_stop_plan)
  msg.detail = st.detail
