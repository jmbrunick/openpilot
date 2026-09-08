import hashlib
import os
import shutil
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.fetch_maps import (
  cleanup_incomplete,
  fetch_and_install,
  install_from_file,
  installed_db_summary,
  main as fetch_maps_main,
  required_free_bytes,
  staging_dir,
)
from openpilot.selfdrive.mapd.maps_manifest import (
  ASSET_SHA256,
  ASSET_SQLITE_BYTES,
  ASSET_ZST_BYTES,
  FREE_MARGIN_BYTES,
  MIN_FREE_BYTES,
  MIN_FREE_MIB,
  SQLITE_SHA256,
)
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB


def _tiny_sqlite(path: str) -> None:
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1, "Main", "primary", 35 * CV.MPH_TO_MS,
    [(37.0, -122.0), (37.001, -122.0)],
  )
  con.execute("INSERT OR REPLACE INTO meta(key, value) VALUES ('way_count', '1')")
  con.commit()
  con.close()


def _sha256(path: str) -> str:
  h = hashlib.sha256()
  with open(path, "rb") as f:
    h.update(f.read())
  return h.hexdigest()


def _make_zst(tmp_path, name="speed_limits.sqlite.zst"):
  zstd = pytest.importorskip("zstandard")
  src = str(tmp_path / "speed_limits.sqlite")
  _tiny_sqlite(src)
  zst = str(tmp_path / name)
  cctx = zstd.ZstdCompressor()
  with open(src, "rb") as inf, open(zst, "wb") as outf:
    cctx.copy_stream(inf, outf)
  return src, zst


def _track_hashed_paths(monkeypatch) -> list[str]:
  import openpilot.selfdrive.mapd.fetch_maps as fm
  hashed: list[str] = []
  orig = fm._sha256_file

  def wrap(path: str) -> str:
    hashed.append(os.path.abspath(path))
    return orig(path)

  monkeypatch.setattr(fm, "_sha256_file", wrap)
  return hashed


def test_us_pack_free_space_threshold():
  assert MIN_FREE_MIB == 800
  assert MIN_FREE_BYTES == 800 * 1024 * 1024
  assert MIN_FREE_BYTES == 838860800
  assert required_free_bytes(us_pack=True) == MIN_FREE_BYTES
  assert ASSET_ZST_BYTES + ASSET_SQLITE_BYTES + FREE_MARGIN_BYTES == MIN_FREE_BYTES


def test_fetch_installs_sqlite(tmp_path):
  src = str(tmp_path / "speed_limits.sqlite")
  _tiny_sqlite(src)

  dest = str(tmp_path / "installed" / "speed_limits.sqlite")
  fetch_and_install(dest=dest, url=Path(src).as_uri(), sha256="")

  db = OsmSpeedLimitDB(dest)
  assert db.open()
  m = db.lookup(37.0004, -122.0, bearing_deg=0.0)
  assert m is not None and m.way_id == 1
  db.close()
  summary = installed_db_summary(dest)
  assert "Installed" in summary
  assert os.path.getsize(dest) > 1024
  stage = staging_dir(dest)
  assert not os.path.isdir(stage) or not os.listdir(stage)


def test_stages_beside_dest_not_tmp(tmp_path, monkeypatch):
  src = str(tmp_path / "speed_limits.sqlite")
  _tiny_sqlite(src)
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  recorded: list[str] = []

  import openpilot.selfdrive.mapd.fetch_maps as fm
  orig = fm.download_url

  def wrap(url, dest_path, progress=True):
    recorded.append(dest_path)
    return orig(url, dest_path, progress)

  monkeypatch.setattr(fm, "download_url", wrap)
  fetch_and_install(dest=dest, url=Path(src).as_uri(), sha256="")

  assert recorded
  packed = recorded[0]
  dest_dir = os.path.dirname(os.path.abspath(dest))
  assert os.path.dirname(packed) == os.path.join(dest_dir, ".download")
  assert os.path.basename(os.path.dirname(packed)) == ".download"
  # Staging lives beside dest, not a system tempfile.mkdtemp() tree.
  assert packed.startswith(dest_dir + os.sep)


def test_insufficient_space_fails_before_download(tmp_path, monkeypatch):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  downloaded = {"n": 0}

  import openpilot.selfdrive.mapd.fetch_maps as fm

  monkeypatch.setattr(fm.shutil, "disk_usage", lambda p: SimpleNamespace(free=100 * 1024 * 1024, used=0, total=0))

  def boom(url, dest_path, progress=True):
    downloaded["n"] += 1
    raise AssertionError("download should not start when space check fails")

  monkeypatch.setattr(fm, "download_url", boom)

  with pytest.raises(RuntimeError, match="Not enough free space"):
    fetch_and_install(
      dest=dest,
      url="https://github.com/jmbrunick/openpilot/releases/download/osm-us-speed-limits-v1/speed_limits_us.sqlite.zst",
      sha256=ASSET_SHA256,
    )
  assert downloaded["n"] == 0
  assert not os.path.isfile(dest)


def test_cleanup_on_failure_removes_partials_and_staging(tmp_path, monkeypatch):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  leftover_partial = dest + ".partial"
  stage = staging_dir(dest)
  os.makedirs(stage, exist_ok=True)
  leftover_zst = os.path.join(stage, "speed_limits_us.sqlite.zst")
  with open(leftover_partial, "wb") as f:
    f.write(b"old-partial")
  with open(leftover_zst, "wb") as f:
    f.write(b"old-zst")
  # Previous good sqlite must survive a failed fetch.
  _tiny_sqlite(dest)

  import openpilot.selfdrive.mapd.fetch_maps as fm

  def fail_download(url, dest_path, progress=True):
    os.makedirs(os.path.dirname(dest_path), exist_ok=True)
    with open(dest_path, "wb") as f:
      f.write(b"incomplete-zst")
    raise RuntimeError("simulated download failure")

  monkeypatch.setattr(fm, "download_url", fail_download)

  with pytest.raises(RuntimeError, match="simulated download failure"):
    fetch_and_install(dest=dest, url=Path(tmp_path / "x.sqlite").as_uri(), sha256="")

  assert os.path.isfile(dest)
  assert not os.path.isfile(leftover_partial)
  assert not os.path.isfile(leftover_zst)
  assert not os.path.isdir(stage) or not os.listdir(stage)


def test_sha256_is_of_downloaded_zst_not_sqlite(tmp_path):
  src, zst = _make_zst(tmp_path)
  zst_hash = _sha256(zst)
  sqlite_hash = _sha256(src)
  assert zst_hash != sqlite_hash

  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  with pytest.raises(RuntimeError, match="SHA-256 mismatch"):
    fetch_and_install(dest=dest, url=Path(zst).as_uri(), sha256=sqlite_hash)
  assert not os.path.isfile(dest)
  assert not os.path.isfile(dest + ".partial")

  fetch_and_install(dest=dest, url=Path(zst).as_uri(), sha256=zst_hash)
  assert os.path.isfile(dest)
  assert _sha256(dest) == sqlite_hash
  db = OsmSpeedLimitDB(dest)
  assert db.open()
  db.close()
  stage = staging_dir(dest)
  assert not os.path.isdir(stage) or not os.listdir(stage)


def test_install_from_file_does_not_hash_sqlite_against_zst(tmp_path, monkeypatch):
  """Device bug: after decompress, hashed dest sqlite vs ASSET_SHA256 (zst).

  Justin 3X: got 264e2f… (sqlite at /data/media/0/osm/speed_limits.sqlite)
  expected d45edd… (zst). ASSET_SHA256 must never be compared to dest.
  """
  src, zst = _make_zst(tmp_path)
  zst_hash = _sha256(zst)
  sqlite_hash = _sha256(src)
  assert zst_hash != sqlite_hash
  assert SQLITE_SHA256 == ""

  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  hashed = _track_hashed_paths(monkeypatch)
  install_from_file(zst, dest, sha256=zst_hash)
  assert os.path.isfile(dest)
  assert _sha256(dest) == sqlite_hash
  dest_abs = os.path.abspath(dest)
  assert dest_abs not in hashed
  assert all(p.endswith(".zst") for p in hashed)
  assert not any(p == dest_abs or p.endswith(".partial") for p in hashed)

  # Re-running with the zst hash on an already-installed sqlite must not fail
  # the way the old install_from_file did (hash dest vs ASSET_SHA256).
  hashed.clear()
  sqlite_copy = str(tmp_path / "already_installed.sqlite")
  shutil.copyfile(dest, sqlite_copy)
  dest2 = str(tmp_path / "osm2" / "speed_limits.sqlite")
  install_from_file(sqlite_copy, dest2, sha256=zst_hash)
  assert os.path.isfile(dest2)
  assert hashed == []


def test_fetch_and_install_never_hashes_dest_sqlite(tmp_path, monkeypatch):
  src, zst = _make_zst(tmp_path)
  zst_hash = _sha256(zst)
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  hashed = _track_hashed_paths(monkeypatch)
  fetch_and_install(dest=dest, url=Path(zst).as_uri(), sha256=zst_hash)
  dest_abs = os.path.abspath(dest)
  assert dest_abs not in hashed
  assert all(".zst" in os.path.basename(p) for p in hashed)
  assert _sha256(dest) == _sha256(src)
  assert _sha256(dest) != zst_hash


def test_empty_sha256_skips_verification(tmp_path, monkeypatch):
  src, zst = _make_zst(tmp_path)
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  hashed = _track_hashed_paths(monkeypatch)
  install_from_file(zst, dest, sha256="")
  assert hashed == []
  dest2 = str(tmp_path / "osm2" / "speed_limits.sqlite")
  src2 = str(tmp_path / "plain.sqlite")
  _tiny_sqlite(src2)
  fetch_and_install(dest=dest2, url=Path(src2).as_uri(), sha256="")
  assert hashed == []
  dest3 = str(tmp_path / "osm3" / "speed_limits.sqlite")
  rc = fetch_maps_main(["--url", Path(src2).as_uri(), "--out", dest3, "--sha256", ""])
  assert rc == 0
  assert os.path.isfile(dest3)


def test_invalid_sqlite_does_not_clobber_dest(tmp_path):
  zstd = pytest.importorskip("zstandard")
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_sqlite(dest)
  prev = _sha256(dest)
  zst = str(tmp_path / "junk.sqlite.zst")
  cctx = zstd.ZstdCompressor()
  with open(zst, "wb") as outf:
    outf.write(cctx.compress(b"not a sqlite"))
  with pytest.raises(RuntimeError, match="not a NAP OSM sqlite"):
    fetch_and_install(dest=dest, url=Path(zst).as_uri(), sha256=_sha256(zst))
  assert os.path.isfile(dest)
  assert _sha256(dest) == prev
  assert not os.path.isfile(dest + ".partial")


def test_fetch_maps_does_not_use_tempfile():
  import inspect
  import openpilot.selfdrive.mapd.fetch_maps as fm
  assert "tempfile" not in inspect.getsource(fm)


def test_cleanup_incomplete_helper(tmp_path):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_sqlite(dest)
  with open(dest + ".partial", "wb") as f:
    f.write(b"x")
  stage = staging_dir(dest)
  os.makedirs(stage, exist_ok=True)
  with open(os.path.join(stage, "foo.zst"), "wb") as f:
    f.write(b"y")
  cleanup_incomplete(dest)
  assert os.path.isfile(dest)
  assert not os.path.isfile(dest + ".partial")
  assert not os.path.isdir(stage) or not os.listdir(stage)
