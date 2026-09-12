#!/usr/bin/env python3
"""Export the US MUTCD YOLOv8 ONNX on a PC (not on the 3X).

Needs: pip install ultralytics onnx

  python -m scripts.nap.export_speed_sign_onnx --out ./speed_sign.onnx
  python -m scripts.nap.install_speed_sign_weights ./speed_sign.onnx

Then either scp the ONNX to the device or attach it to GitHub Release
speed-sign-onnx-v1 as speed_sign.onnx (same pattern as the OSM map pack).
"""
from __future__ import annotations

import argparse
import os

from openpilot.selfdrive.speedsignd.export import export_and_report
from openpilot.selfdrive.speedsignd.weights_manifest import ASSET_SHA256, YOLO_IMGSZ


def main(argv: list[str] | None = None) -> int:
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument("--out", default="speed_sign.onnx", help="output ONNX path")
  args = p.parse_args(argv)
  path, digest, matches = export_and_report(args.out)
  print(f"exported {os.path.getsize(path)} bytes -> {path}")
  print(f"sha256 {digest}")
  if matches:
    print(f"matches bundled ASSET_SHA256 ({ASSET_SHA256[:12]}…)")
  else:
    print(f"note: digest differs from bundled ASSET_SHA256 ({ASSET_SHA256})")
    print("update weights_manifest.ASSET_SHA256 if you intend to republish")
  print(f"imgsz {YOLO_IMGSZ}. next: python -m scripts.nap.install_speed_sign_weights {path}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
