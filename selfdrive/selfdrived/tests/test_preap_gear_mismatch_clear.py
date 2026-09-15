from openpilot.selfdrive.selfdrived.helpers import (
  PREAP_FINGERPRINT,
  _gear_is_drive,
  preap_leave_drive_clears_mismatch,
)


def _leave(*, fingerprint=PREAP_FINGERPRINT, gear, gear_prev):
  return preap_leave_drive_clears_mismatch(
    fingerprint=fingerprint, gear=gear, gear_prev=gear_prev)


def test_gear_is_drive_accepts_name_and_string():
  assert _gear_is_drive("drive")
  assert _gear_is_drive(type("G", (), {"name": "drive"})())
  assert not _gear_is_drive("reverse")
  assert not _gear_is_drive("park")
  assert not _gear_is_drive(None)


def test_clears_on_leave_drive_to_reverse():
  assert _leave(gear="reverse", gear_prev="drive")


def test_clears_on_leave_drive_to_park():
  assert _leave(gear="park", gear_prev="drive")


def test_clears_on_leave_drive_to_neutral():
  assert _leave(gear="neutral", gear_prev="drive")


def test_does_not_clear_on_drive_entry():
  assert not _leave(gear="drive", gear_prev="reverse")
  assert not _leave(gear="drive", gear_prev="park")
  assert not _leave(gear="drive", gear_prev="unknown")


def test_does_not_clear_while_staying_in_drive():
  assert not _leave(gear="drive", gear_prev="drive")


def test_does_not_clear_while_staying_out_of_drive():
  assert not _leave(gear="reverse", gear_prev="reverse")
  assert not _leave(gear="park", gear_prev="reverse")


def test_non_preap_never_clears():
  assert not _leave(fingerprint="honda", gear="reverse", gear_prev="drive")
  assert not _leave(fingerprint="honda", gear="drive", gear_prev="reverse")
