"""Helpers for the split params key headers (common/params_keys.h + common/nap_params_keys.h)."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
KEY_HEADERS = ("common/params_keys.h", "common/nap_params_keys.h")


def params_keys_text(root: Path | str | None = None) -> str:
  """Concatenated text of every params key header (missing files skipped)."""
  base = Path(root) if root is not None else ROOT
  parts = []
  for rel in KEY_HEADERS:
    p = base / rel
    if p.is_file():
      parts.append(p.read_text(encoding="utf-8"))
  return "\n".join(parts)
