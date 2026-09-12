"""YOLOv8 ONNX interface: class map, letterbox, decode, no numpy fallback."""
from __future__ import annotations

import numpy as np
import pytest

from openpilot.selfdrive.speedsignd.detect import OnnxSpeedSignDetector, SpeedSign, SpeedSignDetector, paint_mutcd_r2_1
from openpilot.selfdrive.speedsignd.tests.test_detect import _scene
from openpilot.selfdrive.speedsignd.weights_manifest import YOLO_CLASS_NAMES, YOLO_IMGSZ, YOLO_MIN_CONF
from openpilot.selfdrive.speedsignd.nv12 import Nv12DetectCrop
from openpilot.selfdrive.speedsignd.yolo import (
  PEAK_NAME_MIN,
  POSTED_LOG_MPH,
  _resize_rgb,
  class_to_mph,
  decode_yolov8,
  letterbox_rgb,
  nms_xyxy,
  refine_mph,
  road_detect_crop,
  road_detect_crop_rect,
  speed_limit_class_indices,
  yolo_peak,
)


def test_class_to_mph_reads_mutcd_r2_1_only():
  assert class_to_mph("speedLimit55") == 55
  assert class_to_mph("speedLimit60") == 60
  assert class_to_mph("speedLimit25") == 25
  assert class_to_mph("stop") is None
  assert class_to_mph("yield") is None
  assert class_to_mph("speedLimit47") is None
  assert class_to_mph("doNotEnter") is None


def test_yolo_class_list_covers_highway_speeds():
  names = set(YOLO_CLASS_NAMES)
  for mph in (15, 25, 30, 45, 55, 60, 70, 80):
    assert f"speedLimit{mph}" in names
  assert YOLO_CLASS_NAMES[12] == "speedLimit55"
  assert YOLO_CLASS_NAMES[13] == "speedLimit60"
  # JC best.pt names: 0 doNotEnter … 19 stop 20 yield. Not frequency order.
  assert YOLO_CLASS_NAMES[0] == "doNotEnter"
  assert YOLO_CLASS_NAMES[19] == "stop"
  assert YOLO_CLASS_NAMES[20] == "yield"
  assert len(YOLO_CLASS_NAMES) == 21
  sl = speed_limit_class_indices()
  assert sl[0] == 4 and YOLO_CLASS_NAMES[4] == "speedLimit15"
  assert sl[-1] == 18 and YOLO_CLASS_NAMES[18] == "speedLimit85"
  assert 19 not in sl and 20 not in sl
  # speed_sign.onnx metadata names — Justin's posted 60/50/30, not 65.
  assert YOLO_CLASS_NAMES[7] == "speedLimit30"
  assert YOLO_CLASS_NAMES[11] == "speedLimit50"
  assert YOLO_CLASS_NAMES[13] == "speedLimit60"
  assert YOLO_CLASS_NAMES[14] == "speedLimit65"
  assert POSTED_LOG_MPH == (30, 50, 60)
  assert PEAK_NAME_MIN == 0.05


def test_letterbox_keeps_aspect_and_pads():
  rgb = np.zeros((120, 320, 3), np.uint8)
  rgb[:, :] = (10, 20, 30)
  boxed, scale, pad_x, pad_y = letterbox_rgb(rgb, 320)
  assert boxed.shape == (320, 320, 3)
  assert scale == pytest.approx(1.0)
  assert pad_x == 0
  assert pad_y == 100
  assert boxed[pad_y, 10, 0] == 10


def test_resize_rgb_is_bilinear_not_nearest():
  """Nearest index-sample of 2×2→1×1 picks a corner; bilinear averages."""
  img = np.zeros((2, 2, 3), np.uint8)
  img[0, 0] = 0
  img[0, 1] = 100
  img[1, 0] = 0
  img[1, 1] = 100
  out = _resize_rgb(img, 1, 1)
  assert 40 <= int(out[0, 0, 0]) <= 60


def test_road_detect_crop_keeps_center_and_right():
  rgb = np.zeros((1208, 1928, 3), np.uint8)
  crop, box = road_detect_crop(rgb)
  x, y, w, h = box
  assert crop.shape == (1208, 1208, 3)
  assert w == h == 1208
  assert x == 720
  assert x <= 964 < x + w  # 3X ROAD optical center stays in-frame
  # Scale vs full-frame letterbox: 320/1208 vs 320/1928.
  assert (YOLO_IMGSZ / float(h)) > (YOLO_IMGSZ / 1928.0) * 1.5
  assert road_detect_crop_rect(1208, 1928) == (720, 0, 1208, 1208)


def test_yolo_peak_reports_below_threshold_class():
  raw = np.zeros((1, 25, 4), np.float32)
  raw[0, 4 + 19, 0] = 0.33  # stop
  peak = yolo_peak(raw)
  assert peak.shape == (1, 25, 4)
  assert peak.conf == pytest.approx(0.33, abs=1e-5)
  assert peak.name == "stop"
  assert peak.n_over == 0
  assert peak.sl_conf == pytest.approx(0.0, abs=1e-5)
  assert peak.n_over_sl == 0


def test_yolo_peak_splits_stop_from_speed_limit_and_logs_top3():
  """Justin's STOP-works / R2-1-miss shape: global peak is stop, sl_peak is weak."""
  raw = np.zeros((1, 25, 4), np.float32)
  raw[0, 4 + 19, 0] = 0.91  # stop
  raw[0, 4 + 20, 0] = 0.22  # yield
  raw[0, 4 + 12, 1] = 0.12  # speedLimit55 on another anchor
  raw[0, 4 + 6, 1] = 0.08   # speedLimit25
  peak = yolo_peak(raw)
  assert peak.name == "stop"
  assert peak.conf == pytest.approx(0.91, abs=1e-5)
  assert peak.n_over == 1
  assert peak.sl_name == "speedLimit55"
  assert peak.sl_conf == pytest.approx(0.12, abs=1e-5)
  assert peak.n_over_sl == 0
  assert [n for n, _c in peak.top3] == ["stop", "yield", "speedLimit55"]
  assert peak.top3[0][1] == pytest.approx(0.91, abs=1e-5)
  assert peak.top3[2][1] == pytest.approx(0.12, abs=1e-5)
  assert dict(peak.posted)[30] == pytest.approx(0.0, abs=1e-5)
  assert dict(peak.posted)[50] == pytest.approx(0.0, abs=1e-5)
  assert dict(peak.posted)[60] == pytest.approx(0.0, abs=1e-5)


def test_yolo_peak_blank_name_on_argmax_noise():
  """0.00/speedLimit65 is not a 65 read — no 65 was posted. Argmax of ~0."""
  raw = np.zeros((1, 25, 4), np.float32)
  raw[0, 4 + 14, 0] = 0.004  # speedLimit65 noise winner
  raw[0, 4 + 13, 0] = 0.003  # speedLimit60
  raw[0, 4 + 11, 0] = 0.002  # speedLimit50
  raw[0, 4 + 7, 0] = 0.001   # speedLimit30
  peak = yolo_peak(raw)
  assert peak.conf == pytest.approx(0.004, abs=1e-5)
  assert peak.name == ""
  assert peak.sl_name == ""
  assert peak.n_over == 0
  assert peak.top3 == ()
  assert dict(peak.posted) == {30: pytest.approx(0.001, abs=1e-5),
                               50: pytest.approx(0.002, abs=1e-5),
                               60: pytest.approx(0.003, abs=1e-5)}


def test_nms_keeps_highest_score():
  boxes = np.array([
    [0, 0, 10, 10],
    [1, 1, 11, 11],
    [50, 50, 60, 60],
  ], dtype=np.float32)
  scores = np.array([0.4, 0.9, 0.8], dtype=np.float32)
  keep = nms_xyxy(boxes, scores, iou_thr=0.3, max_det=3)
  assert keep[0] == 1
  assert 2 in keep
  assert 0 not in keep


def test_decode_yolov8_posted_30_50_60():
  """Class map for Justin's route speeds — same indices as the ONNX metadata."""
  for mph, idx in ((30, 7), (50, 11), (60, 13)):
    raw = np.zeros((1, 25, 4), np.float32)
    raw[0, :4, 0] = [160, 120, 80, 100]
    raw[0, 4 + idx, 0] = 0.88
    hits = decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4)
    assert len(hits) == 1 and hits[0].mph == mph


def test_decode_yolov8_speed_limit_55():
  raw = np.zeros((1, 25, 8), np.float32)
  raw[0, 0, 0] = 160.0
  raw[0, 1, 0] = 120.0
  raw[0, 2, 0] = 80.0
  raw[0, 3, 0] = 100.0
  raw[0, 4 + 12, 0] = 0.91  # speedLimit55
  hits = decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4)
  assert len(hits) == 1
  assert hits[0].mph == 55
  assert hits[0].conf == pytest.approx(0.91, abs=1e-5)
  x, y, w, h = hits[0].bbox
  assert x == 120 and y == 70
  assert w == 80 and h == 100


def test_decode_yolov8_accepts_extra_singleton_dim():
  """tinygrad OnnxRunner has returned [1,1,25,N]; old _as_cn dropped that as empty."""
  raw = np.zeros((1, 1, 25, 8), np.float32)
  raw[0, 0, 0, 0] = 160.0
  raw[0, 0, 1, 0] = 120.0
  raw[0, 0, 2, 0] = 80.0
  raw[0, 0, 3, 0] = 100.0
  raw[0, 0, 4 + 12, 0] = 0.91
  hits = decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4)
  assert len(hits) == 1 and hits[0].mph == 55


def test_decode_yolov8_ignores_stop_and_low_conf():
  raw = np.zeros((1, 25, 4), np.float32)
  raw[0, :4, 0] = [80, 80, 40, 40]
  raw[0, 4 + 19, 0] = 0.99  # stop
  raw[0, :4, 1] = [200, 80, 40, 40]
  raw[0, 4 + 13, 1] = 0.2  # speedLimit60 below threshold
  assert decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4) == []


def test_decode_yolov8_keeps_speed_limit_under_stronger_stop_on_same_anchor():
  """Argmax-all-classes used to drop this R2-1 because stop won the anchor."""
  raw = np.zeros((1, 25, 2), np.float32)
  raw[0, :4, 0] = [160, 120, 80, 100]
  raw[0, 4 + 19, 0] = 0.91  # stop
  raw[0, 4 + 12, 0] = 0.50  # speedLimit55
  hits = decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4)
  assert len(hits) == 1
  assert hits[0].mph == 55
  assert hits[0].conf == pytest.approx(0.50, abs=1e-5)


class _YoloSess:
  def get_inputs(self):
    return [type("I", (), {"name": "images", "shape": [1, 3, 320, 320]})()]

  def run(self, _out, feed):
    blob = next(iter(feed.values()))
    assert blob.shape == (1, 3, 320, 320)
    raw = np.zeros((1, 25, 4), np.float32)
    raw[0, 0, 0] = 200.0
    raw[0, 1, 0] = 80.0
    raw[0, 2, 0] = 60.0
    raw[0, 3, 0] = 80.0
    raw[0, 4 + 13, 0] = 0.88  # speedLimit60
    return [raw]


def test_onnx_yolo_session_returns_60():
  det = OnnxSpeedSignDetector("/tmp/fake.onnx", _YoloSess())
  y = np.zeros((320, 320), np.uint8)
  hits = det.detect(y, min_conf=0.4)
  assert hits and hits[0].mph == 60
  assert hits[0].conf == pytest.approx(0.88, abs=1e-5)


def test_yolo_crop_offsets_bbox_to_full_road_frame():
  det = OnnxSpeedSignDetector("/tmp/fake.onnx", _YoloSess(), backend="tinygrad")
  rgb = np.zeros((1208, 1928, 3), np.uint8)
  hits = det.detect(None, min_conf=0.4, rgb=rgb)
  assert hits and hits[0].mph == 60
  x, _y, _w, _h = hits[0].bbox
  assert x >= 720
  d = det.diag_dict()
  assert d["frame_w"] == 1928 and d["frame_h"] == 1208
  assert d["crop"] == (720, 0, 1208, 1208)
  assert d["backend"] == "tinygrad"


def test_onnx_records_peak_when_decode_empty():
  class NearMiss(_YoloSess):
    def run(self, _out, _feed):
      raw = np.zeros((1, 25, 4), np.float32)
      raw[0, :4, 0] = [80, 80, 40, 40]
      raw[0, 4 + 19, 0] = 0.33  # stop, below 0.40
      return [raw]

  det = OnnxSpeedSignDetector("/tmp/fake.onnx", NearMiss(), backend="tinygrad")
  y = np.zeros((320, 320), np.uint8)
  assert det.detect(y, min_conf=0.4) == []
  d = det.diag_dict()
  assert d["backend"] == "tinygrad"
  assert d["frame_w"] == 320 and d["frame_h"] == 320
  assert d["letterbox"] == 320
  assert d["peak_name"] == "stop"
  assert d["peak_conf"] == pytest.approx(0.33, abs=1e-5)
  assert d["n_over"] == 0
  assert d["sl_peak_conf"] == pytest.approx(0.0, abs=1e-5)
  assert d["n_over_sl"] == 0
  assert d["top3"][0][0] == "stop"
  assert d["error"] == ""


def test_onnx_detect_surfaces_session_error():
  class Boom(_YoloSess):
    def run(self, _out, _feed):
      raise RuntimeError("tinygrad layout")

  det = OnnxSpeedSignDetector("/tmp/fake.onnx", Boom(), backend="tinygrad")
  assert det.detect(np.zeros((64, 64), np.uint8)) == []
  assert "tinygrad layout" in det.diag_dict()["error"]


def test_loaded_onnx_does_not_fall_back_to_numpy(tmp_path):
  class EmptyYolo(_YoloSess):
    def run(self, _out, _feed):
      return [np.zeros((1, 25, 4), np.float32)]

  y = _scene()
  paint_mutcd_r2_1(y, 45, x=200, y=30, w=90, h=112)
  det = SpeedSignDetector(onnx=OnnxSpeedSignDetector(str(tmp_path / "x.onnx"), EmptyYolo()))
  assert det.onnx is not None
  assert det.detect(y) == []


def test_refine_overrides_wrong_yolo_class_on_clear_crop():
  from openpilot.selfdrive.speedsignd.detect import _read_mph
  y = _scene()
  paint_mutcd_r2_1(y, 55, x=200, y=30, w=90, h=112)
  wrong = SpeedSign(mph=65, conf=0.57, bbox=(200, 30, 90, 112))
  out = refine_mph(wrong, y, _read_mph)
  assert out.mph == 55


def test_yolo_min_conf_constant():
  assert YOLO_MIN_CONF == 0.40


def test_onnx_nv12_crop_path_offsets_and_chroma():
  """On-car detect uses the crop copy, not a full-frame RGB convert."""
  det = OnnxSpeedSignDetector("/tmp/fake.onnx", _YoloSess(), backend="tinygrad")
  y = np.full((1208, 1208), 88, np.uint8)
  u = np.full((604, 604), 128, np.uint8)
  v = np.full((604, 604), 128, np.uint8)
  uv = np.empty((604, 1208), np.uint8)
  uv[:, 0::2] = u
  uv[:, 1::2] = v
  nv12 = Nv12DetectCrop(y=y, uv=uv, frame_w=1928, frame_h=1208, crop=(720, 0, 1208, 1208))
  hits = det.detect(None, min_conf=0.4, nv12=nv12)
  assert hits and hits[0].mph == 60
  assert hits[0].bbox[0] >= 720
  d = det.diag_dict()
  assert d["backend"] == "tinygrad"
  assert d["frame_w"] == 1928 and d["frame_h"] == 1208
  assert d["crop"] == (720, 0, 1208, 1208)
  assert d["letterbox"] == 320
  assert d["chroma"] == 1
  assert d["luma_mean"] == pytest.approx(88.0, abs=1.0)
  assert d["prep_ms"] >= 0.0
  assert d["sess_ms"] >= 0.0


@pytest.mark.skipif(
  not __import__("os").path.isfile("/tmp/speedsign/best.onnx"),
  reason="exported ONNX not in this environment",
)
def test_real_onnx_empty_road_is_quiet():
  ort = pytest.importorskip("onnxruntime")
  from openpilot.selfdrive.speedsignd.detect import OnnxSpeedSignDetector
  sess = ort.InferenceSession("/tmp/speedsign/best.onnx", providers=["CPUExecutionProvider"])
  det = OnnxSpeedSignDetector("/tmp/speedsign/best.onnx", sess)
  rgb = np.zeros((240, 320, 3), np.uint8)
  rgb[:80] = (140, 170, 200)
  rgb[80:] = (45, 45, 40)
  assert det.detect(None, min_conf=0.40, rgb=rgb) == []
