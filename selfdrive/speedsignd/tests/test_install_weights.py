"""Install-path helper for optional ONNX weights on /data."""
from pathlib import Path

from openpilot.selfdrive.speedsignd.install import install_weights
from scripts.nap.install_speed_sign_weights import main


def test_install_copies_onnx_to_dest(tmp_path):
  src = tmp_path / "speed_sign.onnx"
  src.write_bytes(b"tiny-onnx")
  dest = tmp_path / "nap" / "speed_sign.onnx"
  out = install_weights(str(src), str(dest))
  assert Path(out).read_bytes() == b"tiny-onnx"


def test_cli_install(tmp_path):
  src = tmp_path / "m.onnx"
  src.write_bytes(b"abc")
  dest = tmp_path / "out.onnx"
  assert main([str(src), "--out", str(dest)]) == 0
  assert dest.read_bytes() == b"abc"
