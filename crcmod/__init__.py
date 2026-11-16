"""Minimal pure-Python stand-in for the `crcmod` package (mkCrcFun only).

AGNOS 19.x no longer ships crcmod-plus in the device venv (upstream openpilot
replaced it with opendbc.car.crc). This repo's opendbc submodule still does
`import crcmod` (Tesla Pre-AP teslacan, Hyundai, Nissan), so this module keeps
those imports working on AGNOS 19.x. It lives at the repo root, which is on
PYTHONPATH on device. Behaviour matches crcmod.mkCrcFun, including its
`initCrc` convention (the given value is pre-XORed with `xorOut`).
"""

__all__ = ["mkCrcFun"]


def _reflect(value: int, width: int) -> int:
  out = 0
  for _ in range(width):
    out = (out << 1) | (value & 1)
    value >>= 1
  return out


def _make_table(poly: int, width: int, rev: bool) -> list[int]:
  mask = (1 << width) - 1
  table = []
  if rev:
    rpoly = _reflect(poly & mask, width)
    for i in range(256):
      crc = i
      for _ in range(8):
        crc = (crc >> 1) ^ rpoly if crc & 1 else crc >> 1
      table.append(crc & mask)
  else:
    top = 1 << (width - 1)
    for i in range(256):
      crc = i << (width - 8)
      for _ in range(8):
        crc = ((crc << 1) ^ poly) if crc & top else (crc << 1)
      table.append(crc & mask)
  return table


def mkCrcFun(poly: int, initCrc: int = ~0, rev: bool = True, xorOut: int = 0):
  width = poly.bit_length() - 1
  if width not in (8, 16, 24, 32, 64):
    raise ValueError("The degree of the polynomial must be 8, 16, 24, 32 or 64")
  mask = (1 << width) - 1
  xor_out = xorOut & mask
  init_reg = (initCrc & mask) ^ xor_out
  table = _make_table(poly, width, rev)
  shift = width - 8

  if rev:
    def crcfun(data: bytes, crc: int | None = None) -> int:
      reg = init_reg if crc is None else (crc & mask) ^ xor_out
      for b in bytes(data):
        reg = table[(reg ^ b) & 0xFF] ^ (reg >> 8)
      return reg ^ xor_out
  else:
    def crcfun(data: bytes, crc: int | None = None) -> int:
      reg = init_reg if crc is None else (crc & mask) ^ xor_out
      for b in bytes(data):
        reg = (table[((reg >> shift) ^ b) & 0xFF] ^ (reg << 8)) & mask
      return reg ^ xor_out

  return crcfun
