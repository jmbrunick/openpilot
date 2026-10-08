"""Comma hotspot subnet for the Pre-AP Tesla browser.

The helpers live in wifi_manager.py. Importing that module also imports
NetworkManager and cereal. When those are not built (a host without capnp),
execute just the subnet functions from the same source so the assertions
still cover the real hotspot block.
"""
import ast
from pathlib import Path
from typing import Any


def _load():
  try:
    from openpilot.system.ui.lib.wifi_manager import (
      TETHERING_IP_ADDRESS,
      TETHERING_PREFIX,
      ipv4_shared_address,
      tethering_ipv4_setting,
    )
    return TETHERING_IP_ADDRESS, TETHERING_PREFIX, ipv4_shared_address, tethering_ipv4_setting
  except Exception:
    src_path = Path(__file__).resolve().parents[1] / "wifi_manager.py"
    tree = ast.parse(src_path.read_text(encoding="utf-8"))
    keep = {
      "TETHERING_IP_ADDRESS",
      "TETHERING_PREFIX",
      "_DBUS_SIG_CHARS",
      "_dbus_variant",
      "tethering_ipv4_setting",
      "ipv4_shared_address",
    }

    def _name(node):
      if isinstance(node, ast.FunctionDef):
        return node.name
      if isinstance(node, ast.Assign):
        target = node.targets[0]
        return target.id if isinstance(target, ast.Name) else None
      if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
        return node.target.id
      return None

    body = [node for node in tree.body if _name(node) in keep]
    module = ast.Module(body=body, type_ignores=[])
    ast.fix_missing_locations(module)
    ns = {"Any": Any}
    exec(compile(module, str(src_path), "exec"), ns)
    return (
      ns["TETHERING_IP_ADDRESS"],
      ns["TETHERING_PREFIX"],
      ns["ipv4_shared_address"],
      ns["tethering_ipv4_setting"],
    )


TETHERING_IP_ADDRESS, TETHERING_PREFIX, ipv4_shared_address, tethering_ipv4_setting = _load()


def test_hotspot_is_tesla_browser_subnet():
  assert TETHERING_IP_ADDRESS == "100.99.9.1"
  assert TETHERING_PREFIX == 24
  block = tethering_ipv4_setting()
  assert block["method"] == ("s", "shared")
  assert block["gateway"] == ("s", "100.99.9.1")
  assert block["never-default"] == ("b", True)
  assert ipv4_shared_address(block) == "100.99.9.1"


def test_old_hotspot_address_is_detected():
  legacy = {
    "method": ("s", "shared"),
    "address-data": ("aa{sv}", [[
      ("address", ("s", "192.168.43.1")),
      ("prefix", ("u", 24)),
    ]]),
    "gateway": ("s", "192.168.43.1"),
  }
  assert ipv4_shared_address(legacy) == "192.168.43.1"
  assert ipv4_shared_address(legacy) != TETHERING_IP_ADDRESS

  nm_default = {"method": ("s", "shared"), "gateway": ("s", "10.42.0.1")}
  assert ipv4_shared_address(nm_default) == "10.42.0.1"

  # Unpacked dict form, no signature tuples.
  plain = {"address-data": [{"address": "100.99.9.1", "prefix": 24}]}
  assert ipv4_shared_address(plain) == "100.99.9.1"


def test_client_profile_shape_is_not_the_hotspot_block():
  """Infrastructure Wi-Fi stays method=auto. Only the shared block is rewritten."""
  hotspot = tethering_ipv4_setting()
  assert hotspot["method"][1] == "shared"
  assert "dns-priority" not in hotspot
