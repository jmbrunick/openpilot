from openpilot.selfdrive.selfdrived.helpers import (
  PREAP_FINGERPRINT,
  PreapGearOutMismatchClear,
  _gear_is_drive,
)


def _clear():
  return PreapGearOutMismatchClear()


def test_gear_is_drive_accepts_name_and_string():
  assert _gear_is_drive("drive")
  assert _gear_is_drive(type("G", (), {"name": "drive"})())
  assert not _gear_is_drive("reverse")
  assert not _gear_is_drive("park")
  assert not _gear_is_drive(None)


def test_oneshot_on_drive_after_engaged_reverse():
  c = _clear()
  assert not c.update(fingerprint=PREAP_FINGERPRINT, session_up=True,
                      gear="reverse", gear_prev="drive")
  assert c.update(fingerprint=PREAP_FINGERPRINT, session_up=False,
                  gear="drive", gear_prev="reverse")


def test_oneshot_on_drive_after_engaged_park():
  c = _clear()
  assert not c.update(fingerprint=PREAP_FINGERPRINT, session_up=True,
                      gear="park", gear_prev="drive")
  assert c.update(fingerprint=PREAP_FINGERPRINT, session_up=False,
                  gear="drive", gear_prev="park")


def test_oneshot_fires_only_once():
  c = _clear()
  c.update(fingerprint=PREAP_FINGERPRINT, session_up=True,
           gear="reverse", gear_prev="drive")
  assert c.update(fingerprint=PREAP_FINGERPRINT, session_up=False,
                  gear="drive", gear_prev="reverse")
  # Later Drive frames, including a new engage, must not keep clearing.
  assert not c.update(fingerprint=PREAP_FINGERPRINT, session_up=True,
                      gear="drive", gear_prev="drive")
  assert not c.update(fingerprint=PREAP_FINGERPRINT, session_up=True,
                      gear="drive", gear_prev="drive")


def test_park_to_drive_without_session_does_not_clear():
  c = _clear()
  assert not c.update(fingerprint=PREAP_FINGERPRINT, session_up=False,
                      gear="park", gear_prev="drive")
  assert not c.update(fingerprint=PREAP_FINGERPRINT, session_up=False,
                      gear="drive", gear_prev="park")


def test_door_in_drive_does_not_arm():
  c = _clear()
  assert not c.update(fingerprint=PREAP_FINGERPRINT, session_up=True,
                      gear="drive", gear_prev="drive")
  assert not c.update(fingerprint=PREAP_FINGERPRINT, session_up=False,
                      gear="drive", gear_prev="drive")


def test_reverse_while_already_down_does_not_arm():
  c = _clear()
  assert not c.update(fingerprint=PREAP_FINGERPRINT, session_up=False,
                      gear="reverse", gear_prev="drive")
  assert not c.update(fingerprint=PREAP_FINGERPRINT, session_up=False,
                      gear="drive", gear_prev="reverse")


def test_non_preap_never_clears():
  c = _clear()
  assert not c.update(fingerprint="honda", session_up=True,
                      gear="reverse", gear_prev="drive")
  assert not c.update(fingerprint="honda", session_up=False,
                      gear="drive", gear_prev="reverse")


def test_startup_unknown_to_drive_does_not_clear():
  c = _clear()
  assert not c.update(fingerprint=PREAP_FINGERPRINT, session_up=False,
                      gear="drive", gear_prev="unknown")


def test_stay_in_reverse_is_not_a_clear():
  c = _clear()
  c.update(fingerprint=PREAP_FINGERPRINT, session_up=True,
           gear="reverse", gear_prev="drive")
  assert not c.update(fingerprint=PREAP_FINGERPRINT, session_up=False,
                      gear="reverse", gear_prev="reverse")
