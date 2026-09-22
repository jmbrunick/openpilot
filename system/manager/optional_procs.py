"""Optional manager processes that must not block engage.

Companion Dash is not a driving process. If it crashes, fails to bind
:7070, or fails to preimport, selfdrived must not raise processNotRunning
(NO_ENTRY + SOFT_DISABLE).
"""
from __future__ import annotations

from collections.abc import Iterable
from typing import Any

OPTIONAL_PROCESS_NAMES = frozenset({"nap_dash"})


def missing_required_processes(processes: Iterable[Any], optional_names: set[str] | frozenset[str] | None = None) -> list[str]:
  """Names that should trigger EventName.processNotRunning."""
  skip = OPTIONAL_PROCESS_NAMES if optional_names is None else optional_names
  return [p.name for p in processes if not p.running and p.shouldBeRunning and p.name not in skip]


def nap_dash_enabled(started: bool, params: Any, CP: Any) -> bool:
  """Companion Dash. Default on. Missing/corrupt param must not disable or block engage."""
  try:
    if params is None or params.get("NAPDashEnabled") is None:
      return True
    return bool(params.get_bool("NAPDashEnabled"))
  except Exception:
    return True
