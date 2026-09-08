"""Where comma 3X fetches the prebuilt US OSM speed-limit DB.

The sqlite is not in git (too large). Clone/flash stays small; after install,
Settings → NAP → Download US Maps (or `python -m scripts.nap.fetch_osm_maps`)
pulls a GitHub Release asset onto `/data/media/0/osm/speed_limits.sqlite`.

Refresh maps fetches this module's `MAPS_INDEX_URL` (small JSON, not Overpass
and not the sqlite) and downloads the asset only when the published revision
or zst SHA-256 differs from what is installed.

OpenStreetMap data is ODbL: © OpenStreetMap contributors.
https://www.openstreetmap.org/copyright

Size basis (Taginfo US, 2026-09): ~3.40 million ways with a maxspeed tag.
Measured osm-us-speed-limits-v1: ~204 MiB zst → ~516 MiB sqlite. Fetch requires
800 MiB free on the dest filesystem (zst + sqlite + 80 MiB margin).
"""
from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass

ATTRIBUTION = "© OpenStreetMap contributors"
LICENSE = "ODbL"
LICENSE_URL = "https://www.openstreetmap.org/copyright"

# Host + tag for the prebuilt DB. Bump the tag when republishing a newer extract.
GITHUB_REPO = "jmbrunick/openpilot"
RELEASE_TAG = "osm-us-speed-limits-v1"
ASSET_NAME = "speed_limits_us.sqlite.zst"
# First-install revision; must match committed maps-index.json.
RELEASE_REVISION = "1"

# Live index the 3X fetches for Refresh maps (small JSON, not the sqlite).
# Pointed at raw nap-dev so a county refresh is a JSON bump + new Release
# asset — no software reflash. Override with NAP_MAPS_INDEX_URL.
MAPS_INDEX_REF = "nap-dev"
MAPS_INDEX_FILENAME = "maps-index.json"
MAPS_INDEX_URL = (
  f"https://raw.githubusercontent.com/{GITHUB_REPO}/{MAPS_INDEX_REF}"
  f"/selfdrive/mapd/{MAPS_INDEX_FILENAME}"
)

# Measured osm-us-speed-limits-v1 (~204 MiB zst → ~516 MiB sqlite).
# Peak on dest fs is zst + sqlite.partial before the zst is deleted.
ASSET_ZST_BYTES = 204 * 1024 * 1024
ASSET_SQLITE_BYTES = 516 * 1024 * 1024
FREE_MARGIN_BYTES = 80 * 1024 * 1024
MIN_FREE_MIB = 800
MIN_FREE_BYTES = MIN_FREE_MIB * 1024 * 1024  # 204+516+80 MiB

# SHA-256 of the Release **zst** asset (not the decompressed sqlite).
# Fetch verifies this on the downloaded .zst BEFORE decompress.
ASSET_SHA256 = "d45eddf120f2aa733548b8317e6542ed22880025d33b5f606136567a0feaa26e"
# Optional SHA-256 of the decompressed sqlite. Empty = do not hash dest.
SQLITE_SHA256 = ""

USER_AGENT = "NotAutopilot-mapd/1.0 (OSM ODbL; https://github.com/jmbrunick/openpilot)"

PARAM_REVISION = "NAPMapSpeedDbRevision"
PARAM_SHA256 = "NAPMapSpeedDbSha256"
REVISION_SIDECAR = "maps-revision.json"


def release_asset_url(repo: str = GITHUB_REPO, tag: str = RELEASE_TAG, asset: str = ASSET_NAME) -> str:
  return f"https://github.com/{repo}/releases/download/{tag}/{asset}"


@dataclass(frozen=True)
class MapsIndex:
  """Published US pack metadata. The 3X downloads this JSON, not Overpass."""
  revision: str
  asset_url: str
  asset_name: str
  sha256: str
  bytes: int | None = None
  notes: str = ""


def bundled_maps_index() -> MapsIndex:
  """First-install constants. Kept working even if the live index is unreachable."""
  return MapsIndex(
    revision=RELEASE_REVISION,
    asset_url=release_asset_url(),
    asset_name=ASSET_NAME,
    sha256=ASSET_SHA256,
    bytes=ASSET_ZST_BYTES,
    notes=f"US OSM maxspeed ways ({RELEASE_TAG}).",
  )


def committed_maps_index_path() -> str:
  return os.path.join(os.path.dirname(os.path.abspath(__file__)), MAPS_INDEX_FILENAME)


def _field(data: dict, *keys: str, default: str | None = None) -> str | None:
  for key in keys:
    if key in data and data[key] is not None:
      value = data[key]
      if isinstance(value, str):
        value = value.strip()
        if value:
          return value
      elif isinstance(value, (int, float)) and not isinstance(value, bool):
        return str(value)
  return default


def parse_maps_index(raw: dict | str | bytes) -> MapsIndex:
  """Parse a published maps-index JSON object.

  Required: revision (or version/semver), asset url, sha256 of the zst.
  Optional: asset name, notes, bytes.
  """
  if isinstance(raw, (bytes, bytearray)):
    raw = raw.decode("utf-8")
  if isinstance(raw, str):
    raw = raw.strip()
    if not raw:
      raise ValueError("maps index is empty")
    data = json.loads(raw)
  elif isinstance(raw, dict):
    data = raw
  else:
    raise ValueError(f"maps index must be JSON object or string, not {type(raw).__name__}")
  if not isinstance(data, dict):
    raise ValueError("maps index JSON must be an object")

  revision = _field(data, "revision", "version", "semver")
  asset_url = _field(data, "asset_url", "url")
  sha256 = _field(data, "sha256", "sha256_zst")
  if not revision:
    raise ValueError("maps index missing revision (or version/semver)")
  if not asset_url:
    raise ValueError("maps index missing asset_url")
  if not sha256:
    raise ValueError("maps index missing sha256 of the zst")
  sha256 = sha256.lower()
  if not re.fullmatch(r"[0-9a-f]{64}", sha256):
    raise ValueError(f"maps index sha256 must be 64 hex chars, got {sha256!r}")

  asset_name = _field(data, "asset_name", "name") or os.path.basename(asset_url.split("?", 1)[0]) or ASSET_NAME
  notes = _field(data, "notes") or ""
  size_raw = data.get("bytes", data.get("size"))
  size: int | None
  if size_raw is None or size_raw == "":
    size = None
  else:
    try:
      size = int(size_raw)
    except (TypeError, ValueError) as e:
      raise ValueError(f"maps index bytes must be an integer, got {size_raw!r}") from e
    if size < 0:
      raise ValueError("maps index bytes must be >= 0")

  return MapsIndex(
    revision=revision,
    asset_url=asset_url,
    asset_name=asset_name,
    sha256=sha256,
    bytes=size,
    notes=notes,
  )


def load_committed_maps_index(path: str | None = None) -> MapsIndex:
  path = path or committed_maps_index_path()
  with open(path, encoding="utf-8") as f:
    return parse_maps_index(f.read())


def _revision_parts(rev: str) -> tuple[int, ...]:
  parts: list[int] = []
  for token in re.split(r"[^\d]+", (rev or "").strip()):
    if token:
      parts.append(int(token))
  return tuple(parts) if parts else (0,)


def revision_is_newer(remote: str, installed: str) -> bool:
  """True when remote revision should replace installed (numeric/semver compare)."""
  remote = (remote or "").strip()
  installed = (installed or "").strip()
  if not remote:
    return False
  if not installed:
    return True
  if remote == installed:
    return False
  remote_parts = _revision_parts(remote)
  installed_parts = _revision_parts(installed)
  n = max(len(remote_parts), len(installed_parts))
  remote_parts = remote_parts + (0,) * (n - len(remote_parts))
  installed_parts = installed_parts + (0,) * (n - len(installed_parts))
  if remote_parts != installed_parts:
    return remote_parts > installed_parts
  return remote != installed


def maps_update_decision(
  has_sqlite: bool,
  installed_revision: str,
  installed_sha256: str,
  remote: MapsIndex,
  *,
  bundled: MapsIndex | None = None,
) -> str:
  """Return 'install', 'update', or 'up_to_date'.

  SHA-256 of the published zst is the file identity. A newer revision or a
  different SHA means download. Missing sqlite means first install. A legacy
  sqlite with no recorded revision is treated as the bundled first-install
  pack when the remote index still matches those constants.
  """
  bundled = bundled or bundled_maps_index()
  if not has_sqlite:
    return "install"

  remote_sha = (remote.sha256 or "").strip().lower()
  inst_sha = (installed_sha256 or "").strip().lower()
  inst_rev = (installed_revision or "").strip()

  if inst_sha and remote_sha and inst_sha == remote_sha:
    return "up_to_date"
  if inst_sha and remote_sha and inst_sha != remote_sha:
    return "update"
  if inst_rev:
    return "update" if revision_is_newer(remote.revision, inst_rev) else "up_to_date"

  # Legacy install (sqlite present, no revision/hash recorded).
  if remote_sha == (bundled.sha256 or "").strip().lower() and not revision_is_newer(remote.revision, bundled.revision):
    return "up_to_date"
  return "update"
