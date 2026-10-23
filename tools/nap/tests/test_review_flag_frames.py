import json
import os
import shutil
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import review_flag_frames as rff


def test_resolve_uses_coarse_offset_and_rolls_segments(tmp_path):
  r = "000001bb--32fe2f4345"
  for seg in (10, 11):
    d = tmp_path / r
    d.mkdir(exist_ok=True)
    (d / f"{seg}.qcamera.ts").write_bytes(b"")
  flag = {"route": r, "segment": 11, "seg_offset_s": 19.1}
  assert rff.resolve(str(tmp_path), flag, None, "qcamera")[:2] == (11, 19.1)
  flag = {"route": r, "segment": 11, "seg_offset_s": -5.0}
  seg, off, cam = rff.resolve(str(tmp_path), flag, None, "qcamera")
  assert seg == 10 and off == 55.0 and cam.endswith("10.qcamera.ts")
  assert rff.resolve(str(tmp_path), {"route": None, "segment": None}, None, "qcamera") is None
  assert rff.resolve(str(tmp_path), {"route": r, "segment": 3, "seg_offset_s": 1.0}, None, "qcamera") is None


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg missing")
def test_end_to_end_cuts_frames_and_crops(tmp_path):
  r = "000001bb--32fe2f4345"
  d = tmp_path / "routes" / f"{r}--11"
  d.mkdir(parents=True)
  subprocess.run(["ffmpeg", "-loglevel", "error", "-f", "lavfi", "-i", "testsrc=size=526x330:rate=20", "-t", "20",
                  str(d / "qcamera.ts")], check=True)
  flags = tmp_path / "review_flags.jsonl"
  flags.write_text(json.dumps({"kind": "both", "reason": "both_maps_30_to_50", "mph": 50, "route": r, "segment": 11,
                               "seg_offset_s": 10.0, "lat": 45.3, "lon": -95.6}) + "\n")
  out = tmp_path / "review"
  assert rff.main([str(flags), "--routes-dir", str(tmp_path / "routes"), "--out", str(out), "--camera", "qcamera",
                   "--before", "2", "--after", "2", "--fps", "2"]) == 0
  sub = out / f"000_{r}--11_50"
  frames = sorted(p for p in os.listdir(sub) if p.startswith("frame_"))
  crops = sorted(p for p in os.listdir(sub) if p.startswith("crop_"))
  assert len(frames) >= 7 and len(crops) == len(frames)
  meta = json.loads((sub / "meta.json").read_text())
  assert meta["camera_offset_s"] == 10.0 and meta["reason"] == "both_maps_30_to_50"
  assert "sign_present" in (out / "index.csv").read_text().splitlines()[0]
