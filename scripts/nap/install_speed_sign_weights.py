#!/usr/bin/env python3
"""Install a compact speed-sign ONNX onto /data.

Does not download maps, does not talk to osm.org, and does not write sqlite.

On the comma 3X:

  python -m scripts.nap.install_speed_sign_weights /path/to/speed_sign.onnx

Destination (override with --out or NAP_SPEED_SIGN_ONNX):

  /data/media/0/nap/speed_sign.onnx

The logger runs without this file (built-in MUTCD numpy detector). A small
ONNX (a few MB) can replace that backend. Do not commit weights to git.
"""
from __future__ import annotations

import argparse
import os

from openpilot.selfdrive.speedsignd.install import install_weights
from openpilot.selfdrive.speedsignd.paths import nap_data_dir


def main(argv: list[str] | None = None) -> int:
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument("onnx", help="path to a small speed_sign.onnx")
  p.add_argument("--out", default=None, help="destination (default /data/media/0/nap/speed_sign.onnx)")
  args = p.parse_args(argv)
  dest = install_weights(args.onnx, args.out)
  print(f"installed {os.path.getsize(dest)} bytes -> {dest}")
  print(f"nap data dir: {nap_data_dir()}")
  print("enable: Settings → NAP → Speed Sign Logger, or Params put NAPSpeedSignLog 1")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
