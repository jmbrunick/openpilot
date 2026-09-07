"""Download a prebuilt NAP OSM speed-limit sqlite from a GitHub Release.

Stages on the destination filesystem (not /tmp). US pack peak is the zst plus
the sqlite.partial before the zst is deleted: ~204 MiB + ~516 MiB.
"""
from __future__ import annotations

import argparse
import errno
import hashlib
import os
import shutil
import urllib.error
import urllib.request

from openpilot.selfdrive.mapd.db_paths import default_db_path
from openpilot.selfdrive.mapd.maps_manifest import (
  ASSET_NAME,
  ASSET_SHA256,
  ASSET_SQLITE_BYTES,
  ASSET_ZST_BYTES,
  ATTRIBUTION,
  FREE_MARGIN_BYTES,
  GITHUB_REPO,
  LICENSE,
  LICENSE_URL,
  MIN_FREE_BYTES,
  MIN_FREE_MIB,
  RELEASE_TAG,
  USER_AGENT,
  release_asset_url,
)
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB

CHUNK = 256 * 1024
STAGING_DIRNAME = ".download"


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


def staging_dir(dest: str) -> str:
  return os.path.join(os.path.dirname(os.path.abspath(dest)) or ".", STAGING_DIRNAME)


def cleanup_incomplete(dest: str) -> None:
  """Remove leftover partials/staging from a failed or interrupted fetch."""
  victims = [dest + ".partial"]
  stage = staging_dir(dest)
  if os.path.isdir(stage):
    try:
      names = os.listdir(stage)
    except OSError:
      names = []
    for name in names:
      victims.append(os.path.join(stage, name))
  for p in victims:
    try:
      if os.path.isfile(p) or os.path.islink(p):
        os.remove(p)
    except OSError:
      pass
  try:
    if os.path.isdir(stage) and not os.listdir(stage):
      os.rmdir(stage)
  except OSError:
    pass


def _space_error(dest: str, free: int, need: int) -> RuntimeError:
  dest_dir = os.path.dirname(os.path.abspath(dest)) or "."
  osm = dest_dir
  return RuntimeError(
    f"Not enough free space on {dest_dir}. " +
    f"Need {need / (1024 * 1024):.0f} MiB, have {free / (1024 * 1024):.0f} MiB. " +
    f"US pack is ~{ASSET_ZST_BYTES / (1024 * 1024):.0f} MiB zst + " +
    f"~{ASSET_SQLITE_BYTES / (1024 * 1024):.0f} MiB sqlite " +
    f"(peak both on this filesystem; +{FREE_MARGIN_BYTES / (1024 * 1024):.0f} MiB margin, " +
    f"threshold {MIN_FREE_MIB} MiB / {MIN_FREE_BYTES} bytes). " +
    "Clean leftovers then routes/videos if still short:\n" +
    f"  rm -f {osm}/*.partial {osm}/{STAGING_DIRNAME}/*\n" +
    f"  df -h {dest_dir}\n" +
    "Smaller region (PC, then scp): python scripts/nap/download_osm_speed_limits.py " +
    "--lat <lat> --lon <lon> --radius-km 30"
  )


def required_free_bytes(zst_bytes: int | None = None, *, us_pack: bool = True) -> int:
  """Bytes that must be free on the dest filesystem before a fetch.

  US pack: max(zst + sqlite + margin, MIN_FREE_BYTES). MIN_FREE_BYTES is
  800 MiB = 800 * 1024 * 1024 = 838860800 (204 + 516 + 80 MiB).
  Local/test installs (file://, empty sha256) only need a small headroom.
  """
  if not us_pack:
    extra = int(zst_bytes or 0)
    return max(extra + 8 * 1024 * 1024, 8 * 1024 * 1024)
  zst = ASSET_ZST_BYTES if not zst_bytes or zst_bytes <= 0 else int(zst_bytes)
  need = zst + ASSET_SQLITE_BYTES + FREE_MARGIN_BYTES
  return max(need, MIN_FREE_BYTES)


def assert_free_space(dest: str, zst_bytes: int | None = None, *, us_pack: bool = True) -> int:
  dest_dir = os.path.dirname(os.path.abspath(dest)) or "."
  os.makedirs(dest_dir, exist_ok=True)
  free = int(shutil.disk_usage(dest_dir).free)
  need = required_free_bytes(zst_bytes, us_pack=us_pack)
  _p(f"Free on {dest_dir}: {free / (1024 * 1024):.0f} MiB (need {need / (1024 * 1024):.0f} MiB)")
  if free < need:
    raise _space_error(dest, free, need)
  return free


def _raise_if_enospc(exc: OSError, dest: str) -> None:
  if exc.errno == errno.ENOSPC:
    dest_dir = os.path.dirname(os.path.abspath(dest)) or "."
    free = int(shutil.disk_usage(dest_dir).free)
    raise _space_error(dest, free, required_free_bytes()) from exc
  raise exc


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
    try:
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
    except OSError as e:
      try:
        if os.path.isfile(tmp):
          os.remove(tmp)
      except OSError:
        pass
      _raise_if_enospc(e, dest)
    return n


def _decompress_zst(src: str, dest_partial: str) -> None:
  """Stream-decompress src → dest_partial. Delete src after the stream succeeds
  so peak disk is zst+partial, then partial only. Caller replaces onto dest."""
  os.makedirs(os.path.dirname(dest_partial) or ".", exist_ok=True)
  try:
    try:
      import zstandard as zstd
    except ImportError as e:
      zstd_bin = shutil.which("zstd")
      if not zstd_bin:
        raise RuntimeError("Need the zstandard Python package (or zstd CLI) to decompress maps.") from e
      import subprocess
      try:
        subprocess.check_call([zstd_bin, "-d", "-f", src, "-o", dest_partial])
      except subprocess.CalledProcessError as cpe:
        raise RuntimeError(f"zstd decompress failed (exit {cpe.returncode})") from cpe
    else:
      dctx = zstd.ZstdDecompressor()
      with open(src, "rb") as inf, open(dest_partial, "wb") as outf:
        dctx.copy_stream(inf, outf)
    try:
      os.remove(src)
    except OSError:
      pass
  except OSError as e:
    try:
      if os.path.isfile(dest_partial):
        os.remove(dest_partial)
    except OSError:
      pass
    _raise_if_enospc(e, dest_partial)


def _sqlite_ok(path: str) -> bool:
  db = OsmSpeedLimitDB(path)
  ok = db.open()
  db.close()
  return ok


def _verify_sha256(path: str, sha256: str) -> None:
  got = _sha256_file(path)
  if got.lower() != sha256.lower():
    raise RuntimeError(f"SHA-256 mismatch for {path} (zst/asset, before decompress): got {got}, expected {sha256}")


def fetch_and_install(
  dest: str | None = None,
  url: str | None = None,
  sha256: str = ASSET_SHA256,
) -> str:
  dest = os.path.abspath(dest or default_db_path())
  url = url or os.environ.get("NAP_MAP_RELEASE_URL") or release_asset_url()
  _p(f"OpenStreetMap speed limits ({LICENSE}). {ATTRIBUTION}")
  _p(LICENSE_URL)
  _p(f"Source: {url}")
  _p(f"Install path: {dest}")

  dest_dir = os.path.dirname(dest) or "."
  os.makedirs(dest_dir, exist_ok=True)
  cleanup_incomplete(dest)
  # Default Settings path (HTTPS + ASSET_SHA256) needs the US-pack budget.
  # file:// / empty sha256 (tests, Overpass copies) skip the 800 MiB check.
  us_pack = not url.startswith("file:") and bool(sha256)
  assert_free_space(dest, us_pack=us_pack)

  stage = staging_dir(dest)
  os.makedirs(stage, exist_ok=True)
  packed_name = os.path.basename(url.split("?", 1)[0]) or ASSET_NAME
  if not packed_name or packed_name in (".", "/"):
    packed_name = ASSET_NAME
  packed = os.path.join(stage, packed_name)

  try:
    n = download_url(url, packed)
    _p(f"Downloaded {n / 1e6:.1f} MB → {packed}")
    if us_pack:
      # zst is now on dest fs; still need room for sqlite.partial + margin
      dest_dir = os.path.dirname(os.path.abspath(dest)) or "."
      free = int(shutil.disk_usage(dest_dir).free)
      need_decomp = ASSET_SQLITE_BYTES + FREE_MARGIN_BYTES
      _p(f"Free after zst: {free / (1024 * 1024):.0f} MiB (need {need_decomp / (1024 * 1024):.0f} MiB to decompress)")
      if free < need_decomp:
        raise _space_error(dest, free, need_decomp)
    if sha256:
      _p("Verifying SHA-256 of downloaded asset…")
      _verify_sha256(packed, sha256)
    partial = dest + ".partial"
    if packed.lower().endswith(".zst"):
      _p("Decompressing zstd…")
      _decompress_zst(packed, partial)
    else:
      shutil.copyfile(packed, partial)
      try:
        os.remove(packed)
      except OSError:
        pass
    if not _sqlite_ok(partial):
      raise RuntimeError(f"Downloaded file is not a NAP OSM sqlite: {partial}")
    os.replace(partial, dest)
  except Exception:
    cleanup_incomplete(dest)
    raise

  cleanup_incomplete(dest)
  if not _sqlite_ok(dest):
    raise RuntimeError(f"Downloaded file is not a NAP OSM sqlite: {dest}")

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
