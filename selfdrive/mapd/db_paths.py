import os

from openpilot.selfdrive.mapd.constants import DB_FILENAME
from openpilot.system.hardware.hw import Paths
from openpilot.system.hardware import PC


def osm_dir() -> str:
  if PC:
    return os.path.join(Paths.comma_home(), "media", "0", "osm")
  return "/data/media/0/osm"


def default_db_path() -> str:
  override = os.environ.get("NAP_MAP_DB")
  if override:
    return override
  return os.path.join(osm_dir(), DB_FILENAME)
