"""Where speedsignd fetches the compact US MUTCD speed-sign ONNX.

Weights stay off git (the file is tens of MB, not hundreds). Clone/flash stays
small; `python -m scripts.nap.install_speed_sign_weights` pulls the ONNX onto
`/data/media/0/nap/speed_sign.onnx`.

Source model (MIT): YOLOv8s fine-tuned on LISA + US Roboflow traffic-sign
photos (cvtechniques/JC-Traffic-Sign-Detection). Export is 320² RGB, output
`[1, 25, 2100]` (xywh + 21 classes). Do not commit the ONNX.
"""
from __future__ import annotations

# Host + tag for the pre-exported ONNX. Bump when republishing.
GITHUB_REPO = "jmbrunick/openpilot"
RELEASE_TAG = "speed-sign-onnx-v1"
ASSET_NAME = "speed_sign.onnx"
RELEASE_REVISION = "1"

# HuggingFace checkpoint used by scripts.nap.export_speed_sign_onnx.
# Justin can re-export if the Release asset is missing.
HF_REPO = "cvtechniques/JC-Traffic-Sign-Detection"
HF_PT_PATH = "trainv8/weights/best.pt"
HF_PT_SHA256 = "74089754755d904d724e07ec9d74488427decc11c1c8af323e2b4524664575ae"

# SHA-256 of the exported ONNX (opset 12, 320², simplified).
ASSET_SHA256 = "6ed5f87f3ad2f94891ff242d351abd5262a1b8c2ec860fc5e29cf157fd39a130"
ASSET_BYTES = 44651386

USER_AGENT = "NotAutopilot-speedsignd/1.0 (https://github.com/jmbrunick/openpilot)"

# Ultralytics YOLOv8 export at imgsz=320. The JC checkpoint was trained at 640;
# we do not raise on-device imgsz (CPU). road_detect_crop + bilinear letterbox
# recover scale vs the old nearest-neighbor full-frame 1928→320 path.
YOLO_IMGSZ = 320
YOLO_INPUT_NAME = "images"
YOLO_OUTPUT_LAYOUT = (1, 25, 2100)  # 4 + 21 classes, 2100 anchors

# Class order from speed_sign.onnx metadata `names` (same as JC best.pt).
# 0 doNotEnter … 7 speedLimit30 … 11 speedLimit50 … 13 speedLimit60 …
# 19 stop 20 yield. Isolated MUTCD SVG 30/50/60 at 320 peak ~0.93 on the
# matching class — not an off-by-one. stop has ~18× the samples of
# speedLimit55; the head was trained at imgsz=640.
YOLO_CLASS_NAMES: tuple[str, ...] = (
  "doNotEnter",
  "noLeftTurn",
  "noRightTurn",
  "pedestrianCrossing",
  "speedLimit15",
  "speedLimit20",
  "speedLimit25",
  "speedLimit30",
  "speedLimit35",
  "speedLimit40",
  "speedLimit45",
  "speedLimit50",
  "speedLimit55",
  "speedLimit60",
  "speedLimit65",
  "speedLimit70",
  "speedLimit75",
  "speedLimit80",
  "speedLimit85",
  "stop",
  "yield",
)

# Empty-road max is ~0. Do not lower: on-car clear R2-1s peak at 0.00–0.12, so a
# HUD floor of 0.10 would false-trigger and 0.25 still would not fire. Model limit.
YOLO_MIN_CONF = 0.40
YOLO_IOU = 0.45
YOLO_MAX_DET = 3

# JSONL still needs two agreeing frames. HUD lights on the first in-threshold
# hit — a 1 Hz + skip-on-overrun pair often cannot land while a real R2-1 is
# in view (highway dwell ~1–3 s; a 1.5 s infer spaces hits by ~3 s).
DEBOUNCE_HITS = 2
DEBOUNCE_WINDOW_S = 4.0


def release_asset_url(repo: str = GITHUB_REPO, tag: str = RELEASE_TAG, asset: str = ASSET_NAME) -> str:
  return f"https://github.com/{repo}/releases/download/{tag}/{asset}"


def hf_pt_url(repo: str = HF_REPO, path: str = HF_PT_PATH) -> str:
  return f"https://huggingface.co/{repo}/resolve/main/{path}"
