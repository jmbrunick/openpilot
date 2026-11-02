from openpilot.selfdrive.selfdrived.helpers import (
  PREAP_FINGERPRINT,
  _gear_is_drive,
  preap_not_in_drive_clears_mismatch,
)


def _out(*, fingerprint=PREAP_FINGERPRINT, gear):
  return preap_not_in_drive_clears_mismatch(fingerprint=fingerprint, gear=gear)


def test_gear_is_drive_accepts_name_and_string():
  assert _gear_is_drive("drive")
  assert _gear_is_drive(type("G", (), {"name": "drive"})())
  assert not _gear_is_drive("reverse")
  assert not _gear_is_drive("park")
  assert not _gear_is_drive(None)


def test_holds_clear_while_in_reverse():
  assert _out(gear="reverse")


def test_holds_clear_while_in_park():
  assert _out(gear="park")


def test_holds_clear_while_in_neutral():
  assert _out(gear="neutral")


def test_does_not_clear_in_drive():
  assert not _out(gear="drive")


def test_non_preap_never_clears():
  assert not _out(fingerprint="honda", gear="reverse")
  assert not _out(fingerprint="honda", gear="drive")
