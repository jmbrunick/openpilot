"""Install-path helper: local copy and checksummed fetch onto /data."""
from pathlib import Path

import pytest

from openpilot.selfdrive.speedsignd.install import fetch_and_install, install_weights, sha256_file
from openpilot.selfdrive.speedsignd.weights_manifest import ASSET_SHA256, release_asset_url
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


def test_fetch_verifies_checksum(tmp_path):
  src = tmp_path / "speed_sign.onnx"
  src.write_bytes(b"compact-yolo")
  digest = sha256_file(str(src))
  dest = tmp_path / "nap" / "speed_sign.onnx"
  out = fetch_and_install(dest=str(dest), url=src.resolve().as_uri(), sha256=digest)
  assert Path(out).read_bytes() == b"compact-yolo"


def test_fetch_rejects_bad_checksum(tmp_path):
  src = tmp_path / "speed_sign.onnx"
  src.write_bytes(b"nope")
  dest = tmp_path / "nap" / "speed_sign.onnx"
  with pytest.raises(RuntimeError, match="SHA-256"):
    fetch_and_install(dest=str(dest), url=src.resolve().as_uri(), sha256="0" * 64)


def test_cli_download_file_url(tmp_path):
  src = tmp_path / "pub.onnx"
  src.write_bytes(b"from-url")
  dest = tmp_path / "installed.onnx"
  digest = sha256_file(str(src))
  assert main(["--url", src.resolve().as_uri(), "--out", str(dest), "--sha256", digest]) == 0
  assert dest.read_bytes() == b"from-url"


def test_manifest_points_at_github_release():
  url = release_asset_url()
  assert "speed-sign-onnx-v1" in url
  assert url.endswith("speed_sign.onnx")
  assert len(ASSET_SHA256) == 64
