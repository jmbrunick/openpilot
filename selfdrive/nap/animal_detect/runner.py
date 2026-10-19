"""Model runner. On the car: the tinygrad pickle that SConscript compiles from
models/animal_yolox_nano.onnx (same ONNX -> compile3 -> pickle path as the DM
model, so it runs on the Adreno GPU like modeld). Anything else is a test stub."""
from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np

from openpilot.selfdrive.nap.animal_detect.config import TILE_H, TILE_W

MODEL_DIR = Path(__file__).resolve().parent / "models"
ONNX_PATH = MODEL_DIR / "animal_yolox_nano.onnx"
PKL_PATH = MODEL_DIR / "animal_yolox_nano_tinygrad.pkl"
BATCH = 3   # two road tiles + one wide tile; the pickle is compiled for this shape
INPUT_NAME = "input"


class ModelMissing(RuntimeError):
  pass


class TinygradRunner:
  def __init__(self, pkl_path: Path = PKL_PATH):
    if not os.path.exists(pkl_path):
      raise ModelMissing(str(pkl_path))
    import pickle
    from tinygrad.tensor import Tensor
    self._Tensor = Tensor
    with open(pkl_path, "rb") as f:
      self._run = pickle.load(f)
    self._buf = np.zeros((BATCH, 3, TILE_H, TILE_W), dtype=np.float32)

  def __call__(self, tiles: list[np.ndarray]) -> tuple[np.ndarray, float]:
    """tiles: up to BATCH (TILE_H, TILE_W, 3) BGR uint8. Returns (output, ms)."""
    n = min(len(tiles), BATCH)
    self._buf[:] = 114.0
    for i in range(n):
      self._buf[i] = tiles[i].transpose(2, 0, 1)
    t0 = time.perf_counter()
    out = self._run(**{INPUT_NAME: self._Tensor(self._buf, device="NPY")}).numpy()
    ms = (time.perf_counter() - t0) * 1e3
    return out.reshape(BATCH, -1, out.shape[-1])[:n], ms
