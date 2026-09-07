"""Download a prebuilt NAP OSM speed-limit sqlite from a GitHub Release."""
from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import tempfile
import urllib.error
import urllib.request

from openpilot.selfdrive.mapd.db_paths import default_db_path
from openpilot.selfdrive.mapd.maps_manifest import (
  ASSET_NAME,
  ASSET_SHA256,
  ATTRIBUTION,
  GITHUB_REPO,
  LICENSE,
  LICENSE_URL,
  RELEASE_TAG,
  USER_AGENT,
  release_asset_url,
)
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB

CHUNK = 256 * 1024


def _p(msg: str) -> None:
  print(msg, flush=True)


def installed_db_summary(path: str | None = None) -> str:
  path = path or default_db_path()
  if not path or not os.path.isfile(path):
    return "Not installed"
  try:
    sz = os.path.getsize(path)
  except OSError:
    return "Not installed"
  if sz < 1024:
    return "Not installed"
  mb = sz / (1024 * 1024)
  ways = OsmSpeedLimitDB.way_count(path)
  if ways is None:
    return f"Installed ({mb:.0f} MB)"
  return f"Installed ({mb:.0f} MB, {ways:,} ways)"


def _sha256_file(path: str) -> str:
  h = hashlib.sha256()
  with open(path, "rb") as f:
    while True:
      chunk = f.read(1024 * 1024)
      if not chunk:
        break
      h.update(chunk)
  return h.hexdigest()


def _progress(prefix: str, done: int, total: int) -> None:
  if total > 0:
    pct = min(100.0, 100.0 * done / total)
    _p(f"{prefix} {done / 1e6:.1f}/{total / 1e6:.1f} MB ({pct:.0f}%)")
  else:
    _p(f"{prefix} {done / 1e6:.1f} MB")


def download_url(url: str, dest: str, progress: bool = True) -> int:
  req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
  try:
    resp_cm = urllib.request.urlopen(req, timeout=60)
  except urllib.error.HTTPError as e:
    raise RuntimeError(
      f"Download failed HTTP {e.code} for {url}. " +
      "Publish the OSM sqlite as a GitHub Release asset (see docs-nap/map-speed.md)."
    ) from e
  except urllib.error.URLError as e:
    raise RuntimeError(f"Download failed: {e}. Need Wi-Fi to GitHub.") from e

  with resp_cm as resp:
    total = int(resp.headers.get("Content-Length") or 0)
    os.makedirs(os.path.dirname(dest) or ".", exist_ok=True)
    tmp = dest + ".partial"
    n = 0
    last = 0
    with open(tmp, "wb") as f:
      while True:
        chunk = resp.read(CHUNK)
        if not chunk:
          break
        f.write(chunk)
        n += len(chunk)
        if progress and (n - last >= 5 * 1024 * 1024 or (total and n == total)):
          _progress("Downloading", n, total)
          last = n
    os.replace(tmp, dest)
    return n


def _decompress_zst(src: str, dest: str) -> None:
  try:
    import zstandard as zstd
  except ImportError as e:
    zstd_bin = shutil.which("zstd")
    if not zstd_bin:
      raise RuntimeError("Need the zstandard Python package (or zstd CLI) to decompress maps.") from e
    import subprocess
    subprocess.check_call([zstd_bin, "-d", "-f", src, "-o", dest])
    return
  dctx = zstd.ZstdDecompressor()
  tmp = dest + ".partial"
  with open(src, "rb") as inf, open(tmp, "wb") as outf:
    dctx.copy_stream(inf, outf)
  os.replace(tmp, dest)


def install_from_file(src: str, dest: str, sha256: str = "") -> str:
  """Copy/decompress src onto dest. src may be .zst or a raw sqlite."""
  os.makedirs(os.path.dirname(dest) or ".", exist_ok=True)
  lower = src.lower()
  if lower.endswith(".zst"):
    _p("Decompressing zstd…")
    _decompress_zst(src, dest)
  else:
    tmp = dest + ".partial"
    shutil.copyfile(src, tmp)
    os.replace(tmp, dest)
  if sha256:
    got = _sha256_file(dest)
    if got.lower() != sha256.lower():
      os.remove(dest)
      raise RuntimeError(f"SHA-256 mismatch for {dest}: got {got}, expected {sha256}")
  db = OsmSpeedLimitDB(dest)
  ok = db.open()
  db.close()
  if not ok:
    raise RuntimeError(f"Downloaded file is not a NAP OSM sqlite: {dest}")
  return dest


def fetch_and_install(
  dest: str | None = None,
  url: str | None = None,
  sha256: str = ASSET_SHA256,
) -> str:
  dest = dest or default_db_path()
  url = url or os.environ.get("NAP_MAP_RELEASE_URL") or release_asset_url()
  _p(f"OpenStreetMap speed limits ({LICENSE}). {ATTRIBUTION}")
  _p(LICENSE_URL)
  _p(f"Source: {url}")
  _p(f"Install path: {dest}")
  work = tempfile.mkdtemp(prefix="nap-osm-")
  try:
    ext = ".sqlite.zst" if url.lower().endswith(".zst") else os.path.splitext(url)[1] or ".bin"
    packed = os.path.join(work, "download" + ext)
    n = download_url(url, packed)
    _p(f"Downloaded {n / 1e6:.1f} MB")
    install_from_file(packed, dest, sha256=sha256)
  finally:
    shutil.rmtree(work, ignore_errors=True)
  _p(installed_db_summary(dest))
  _p("mapd reloads this file within ~15s onroad. No reboot required.")
  return dest


def main(argv: list[str] | None = None) -> int:
  p = argparse.ArgumentParser(description="Fetch prebuilt US OSM speed-limit DB (ODbL) onto the device")
  p.add_argument("--url", default=None, help="Override release asset URL")
  p.add_argument("--tag", default=RELEASE_TAG)
  p.add_argument("--repo", default=GITHUB_REPO)
  p.add_argument("--asset", default=ASSET_NAME)
  p.add_argument("--out", default=None, help="sqlite destination (default: /data/media/0/osm/speed_limits.sqlite)")
  p.add_argument("--sha256", default=ASSET_SHA256)
  p.add_argument("--status", action="store_true", help="Print installed DB summary and exit")
  args = p.parse_args(argv)

  if args.status:
    _p(installed_db_summary(args.out))
    return 0

  url = args.url or release_asset_url(args.repo, args.tag, args.asset)
  try:
    fetch_and_install(dest=args.out, url=url, sha256=args.sha256)
  except Exception as e:
    _p(f"ERROR: {e}")
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
