"""Where comma 3X fetches the prebuilt US OSM speed-limit DB.

The sqlite is not in git (too large). Clone/flash stays small; after install,
Settings → NAP → Download US Maps (or `python -m scripts.nap.fetch_osm_maps`)
pulls a GitHub Release asset onto `/data/media/0/osm/speed_limits.sqlite`.

OpenStreetMap data is ODbL: © OpenStreetMap contributors.
https://www.openstreetmap.org/copyright

Size basis (Taginfo US, 2026-09): ~3.40 million ways with a maxspeed tag.
A speed-limits-only sqlite with simplified polylines is expected ~0.8–1.5 GB
on disk and ~200–500 MB as zstd — under GitHub's release-asset cap, so one
US-wide file rather than per-state packs. If a future extract exceeds ~1.5 GB
zst, split to state packs with the same fetch script (`--region`).
"""
from __future__ import annotations

ATTRIBUTION = "© OpenStreetMap contributors"
LICENSE = "ODbL"
LICENSE_URL = "https://www.openstreetmap.org/copyright"

# Host + tag for the prebuilt DB. Bump the tag when republishing a newer extract.
GITHUB_REPO = "jmbrunick/openpilot"
RELEASE_TAG = "osm-us-speed-limits-v1"
ASSET_NAME = "speed_limits_us.sqlite.zst"

# Taginfo north-america:us key=maxspeed ways (2026-09-06 extract).
US_MAXSPEED_WAYS_TAGINFO = 3403120
APPROX_SQLITE_GB = 1.2
APPROX_ZST_MB = 350

# Set after the first published asset so the device can verify the download.
# Empty means "accept whatever sqlite the release currently serves".
ASSET_SHA256 = "d45eddf120f2aa733548b8317e6542ed22880025d33b5f606136567a0feaa26e"

USER_AGENT = "NotAutopilot-mapd/1.0 (OSM ODbL; https://github.com/jmbrunick/openpilot)"


def release_asset_url(repo: str = GITHUB_REPO, tag: str = RELEASE_TAG, asset: str = ASSET_NAME) -> str:
  return f"https://github.com/{repo}/releases/download/{tag}/{asset}"
