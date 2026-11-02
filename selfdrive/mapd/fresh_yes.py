"""nap-dev's FreshYes module (PR #396). On nap-alpha FreshYes lives in short_segment.py (PR #395); re-exported here so both
branches share one implementation."""
from openpilot.selfdrive.mapd.short_segment import YES_FRESH_S, YES_PASSED_M, FreshYes

__all__ = ["YES_FRESH_S", "YES_PASSED_M", "FreshYes"]
