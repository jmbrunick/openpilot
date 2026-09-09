"""Copy a compact ONNX detector onto /data. No osm.org, no sqlite."""
from __future__ import annotations

import os
import shutil

from openpilot.selfdrive.speedsignd.paths import default_onnx_path


def install_weights(src: str, dest: str | None = None) -> str:
  if not os.path.isfile(src):
    raise FileNotFoundError(f"ONNX not found: {src}")
  dest = dest or default_onnx_path()
  parent = os.path.dirname(dest)
  if parent:
    os.makedirs(parent, exist_ok=True)
  shutil.copy2(src, dest)
  return dest
