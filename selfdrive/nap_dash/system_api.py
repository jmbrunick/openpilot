"""Read/write comma Software params for MCU/phone. No SSH. No engage."""
from __future__ import annotations

import re
import subprocess

BRANCH_OK = re.compile(r"^[A-Za-z0-9._/-]+$")
BLOCKED_ACTIONS = {
    "engage",
    "disengage",
    "uninstall",
    "reset_calibration",
    "reset_lateral",
    "reset_longitudinal",
    "force_onroad",
}
SOFTWARE_ACTIONS = {"set_branch", "fetch", "set_offline"}
UPDATED_PATTERNS = (
    "openpilot.system.updated.updated",
    "system.updated.updated",
)


class SoftwareError(ValueError):
    pass


def _as_str(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    return str(value)


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        folded = value.strip().lower()
        if folded in ("1", "true", "yes", "on"):
            return True
        if folded in ("0", "false", "no", "off", ""):
            return False
    raise SoftwareError(f"invalid bool: {value!r}")


def sanitize_branch(raw: str | None) -> str:
    branch = (raw or "").strip()
    if not branch or len(branch) > 80:
        raise SoftwareError("invalid branch")
    if ".." in branch or branch.startswith("/") or branch.startswith("-"):
        raise SoftwareError("invalid branch")
    key = branch.lower().replace("-", "_")
    if key in BLOCKED_ACTIONS or "engage" in key:
        raise SoftwareError("action not allowed")
    if not BRANCH_OK.match(branch):
        raise SoftwareError("invalid branch")
    return branch


def read_software(params) -> dict:
    branches_raw = _as_str(params.get("UpdaterAvailableBranches"))
    branches = [part for part in branches_raw.split(",") if part]
    offline = bool(params.get_bool("DisableUpdates"))
    return {
        "git_branch": _as_str(params.get("GitBranch")),
        "git_commit": _as_str(params.get("GitCommit")),
        "version": _as_str(params.get("Version")),
        "target_branch": _as_str(params.get("UpdaterTargetBranch")),
        "available_branches": branches,
        "updater_state": _as_str(params.get("UpdaterState")) or "idle",
        "fetch_available": bool(params.get_bool("UpdaterFetchAvailable")),
        "update_available": bool(params.get_bool("UpdateAvailable")),
        "disable_updates": offline,
        "offline": offline,
    }


def ping_updated(signal: str = "USR1") -> None:
    """Same wake-up the comma Software UI uses. Fail-soft if updated is idle."""
    for pattern in UPDATED_PATTERNS:
        try:
            subprocess.run(
                ["pkill", f"-SIG{signal}", "-f", pattern],
                check=False,
                timeout=2,
            )
        except Exception:
            pass


def _put(params, key: str, value: str) -> None:
    try:
        params.put(key, value, block=True)
    except TypeError:
        params.put(key, value)


def _put_bool(params, key: str, value: bool) -> None:
    try:
        params.put_bool(key, value)
    except Exception:
        _put(params, key, "1" if value else "0")


def handle_software(payload: dict, params, pinger=ping_updated) -> dict:
    action = str(payload.get("action") or "").strip().lower().replace("-", "_")
    if not action:
        raise SoftwareError("missing action")
    if action in BLOCKED_ACTIONS or "engage" in action:
        raise SoftwareError("action not allowed")
    if action not in SOFTWARE_ACTIONS:
        raise SoftwareError(f"software action not allowed: {action}")
    if action == "set_branch":
        branch = sanitize_branch(str(payload.get("branch") or payload.get("name") or ""))
        _put(params, "UpdaterTargetBranch", branch)
        pinger("USR1")
    elif action == "set_offline":
        raw = payload.get("offline")
        if raw is None:
            raw = payload.get("value")
        offline = True if raw is None else _as_bool(raw)
        _put_bool(params, "DisableUpdates", offline)
    else:
        pinger("USR1")
    return read_software(params)
