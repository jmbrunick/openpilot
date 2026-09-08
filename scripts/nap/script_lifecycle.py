"""Which NAP script-runner jobs reboot the device on Exit.

Refresh maps and Download US Maps only write sqlite. Pedal / EPAS / other
Panda tools keep HARDWARE.reboot() so manager can resume.
"""
from __future__ import annotations

_NO_REBOOT_MODULES = frozenset({
  "scripts.nap.fetch_osm_maps",
  "scripts.nap.refresh_osm_maps",
})


def script_reboots_on_exit(module: str) -> bool:
  return module not in _NO_REBOOT_MODULES
