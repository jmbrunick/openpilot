import os

from openpilot.selfdrive.mapd.constants import DB_FILENAME


def osm_dir() -> str:
  # Avoid hardware/cereal so mapd unit tests and fetch_maps stay importable off-device.
  if os.path.isdir("/data/media/0"):
    return "/data/media/0/osm"
  return os.path.join(os.path.expanduser("~"), ".comma", "media", "0", "osm")


def default_db_path() -> str:
  override = os.environ.get("NAP_MAP_DB")
  if override:
    return override
  return os.path.join(osm_dir(), DB_FILENAME)
