"""Where comma 3X fetches the prebuilt US OSM speed-limit DB.

The sqlite is not in git (too large). Clone/flash stays small; after install,
Settings → NAP → Download US Maps (or `python -m scripts.nap.fetch_osm_maps`)
pulls a GitHub Release asset onto `/data/media/0/osm/speed_limits.sqlite`.

OpenStreetMap data is ODbL: © OpenStreetMap contributors.
https://www.openstreetmap.org/copyright

Size basis (Taginfo US, 2026-09): ~3.40 million ways with a maxspeed tag.
Measured osm-us-speed-limits-v1: ~204 MiB zst → ~516 MiB sqlite. Fetch requires
800 MiB free on the dest filesystem (zst + sqlite + 80 MiB margin).
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

# Measured osm-us-speed-limits-v1 (Justin 3X download: 203.8 MB zst, ~516 MB sqlite).
# Peak on dest fs is zst + sqlite.partial before the zst is deleted.
ASSET_ZST_BYTES = 204 * 1024 * 1024       # 214015488
ASSET_SQLITE_BYTES = 516 * 1024 * 1024    # 541065216
FREE_MARGIN_BYTES = 80 * 1024 * 1024      # 83886080
MIN_FREE_MIB = 800
MIN_FREE_BYTES = MIN_FREE_MIB * 1024 * 1024  # 838860800; 204+516+80 MiB

APPROX_SQLITE_GB = ASSET_SQLITE_BYTES / (1024 ** 3)
APPROX_ZST_MB = ASSET_ZST_BYTES / (1024 * 1024)

# SHA-256 of the Release **zst** asset (not the decompressed sqlite).
ASSET_SHA256 = "d45eddf120f2aa733548b8317e6542ed22880025d33b5f606136567a0feaa26e"

USER_AGENT = "NotAutopilot-mapd/1.0 (OSM ODbL; https://github.com/jmbrunick/openpilot)"


def release_asset_url(repo: str = GITHUB_REPO, tag: str = RELEASE_TAG, asset: str = ASSET_NAME) -> str:
  return f"https://github.com/{repo}/releases/download/{tag}/{asset}"
