"""Pre-AP reverse is a quiet USER_DISABLE, not TAKE CONTROL IMMEDIATELY.

EventName.reverseGear still drops OP (USER_DISABLE) and blocks re-entry
(NO_ENTRY). Stock ImmediateDisableAlert / warningImmediate is Pre-AP-only
muted so a parking-lot R does not sound like an emergency. FCW/AEB and the
#163 leave-Drive mismatch clear are unchanged.
"""
from cereal import car, log
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
from openpilot.selfdrive.selfdrived.events import (
  ET,
  EVENTS,
  Events,
  ImmediateDisableAlert,
  Priority,
  pcm_disable_alert,
  reverse_gear_disable_alert,
)
from openpilot.selfdrive.selfdrived.helpers import PREAP_FINGERPRINT, preap_not_in_drive_clears_mismatch
from openpilot.selfdrive.selfdrived.state import StateMachine

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
EventName = log.OnroadEvent.EventName
State = log.SelfdriveState.OpenpilotState
VisualAlert = car.CarControl.HUDControl.VisualAlert


def _cp(*, fingerprint=PREAP_FINGERPRINT, brand="tesla"):
  cp = car.CarParams.new_message()
  cp.brand = brand
  cp.carFingerprint = fingerprint
  return cp


def _cs():
  return car.CarState.new_message()


def _args(cp=None):
  return (cp or _cp(), _cs(), None, False, 100, log.LongitudinalPersonality.standard)


def _alert(cp=None):
  return reverse_gear_disable_alert(*_args(cp))


def test_reverse_gear_still_user_disables_and_blocks_entry():
  types = EVENTS[EventName.reverseGear]
  assert ET.USER_DISABLE in types
  assert ET.NO_ENTRY in types
  assert ET.PERMANENT in types
  assert types[ET.USER_DISABLE] is reverse_gear_disable_alert


def test_preap_reverse_disable_is_silent():
  alert = _alert()
  assert alert.alert_text_1 == ""
  assert alert.alert_text_2 == ""
  assert alert.alert_size == AlertSize.none
  assert alert.alert_status == AlertStatus.normal
  assert alert.visual_alert == VisualAlert.none
  assert alert.audible_alert == AudibleAlert.none
  assert alert.priority == Priority.HIGH
  assert alert.duration == int(1.0 / DT_CTRL)


def test_stock_reverse_disable_is_still_immediate_take_control():
  alert = _alert(_cp(fingerprint="HONDA_CIVIC_2022", brand="honda"))
  assert isinstance(alert, ImmediateDisableAlert)
  assert alert.alert_text_1 == "TAKE CONTROL IMMEDIATELY"
  assert alert.alert_text_2 == "Reverse Gear"
  assert alert.audible_alert == AudibleAlert.warningImmediate
  assert alert.visual_alert == VisualAlert.steerRequired
  assert alert.priority == Priority.HIGHEST


def test_reverse_user_disable_drops_enabled_state():
  ev = Events()
  ev.add(EventName.reverseGear)
  sm = StateMachine()
  sm.state = State.enabled
  enabled, active = sm.update(ev)
  assert sm.state == State.disabled
  assert not enabled
  assert not active
  assert ET.USER_DISABLE in sm.current_alert_types
  assert ET.IMMEDIATE_DISABLE not in sm.current_alert_types


def test_reverse_no_entry_blocks_enable():
  ev = Events()
  ev.add(EventName.reverseGear)
  ev.add(EventName.pcmEnable)
  sm = StateMachine()
  sm.state = State.disabled
  enabled, _ = sm.update(ev)
  assert sm.state == State.disabled
  assert not enabled
  assert ET.NO_ENTRY in sm.current_alert_types


def test_quiet_reverse_outranks_pcm_disable_takeover_chime():
  """Same-frame cruise-off pcmDisable must not become TAKE CONTROL or a chime."""
  cp = _cp()
  args = _args(cp)
  reverse = reverse_gear_disable_alert(*args)
  reverse.alert_type = "reverseGear/userDisable"
  reverse.event_type = ET.USER_DISABLE
  pcm = pcm_disable_alert(*args)
  pcm.alert_type = "pcmDisable/userDisable"
  pcm.event_type = ET.USER_DISABLE

  am = AlertManager()
  am.add_many(0, [reverse, pcm])
  am.process_alerts(0, set())
  shown = am.current_alert
  assert shown.audible_alert == AudibleAlert.none
  assert shown.visual_alert == VisualAlert.none
  assert shown.alert_text_1 != "TAKE CONTROL IMMEDIATELY"

  # pcmDisable lasts 0.8s; quiet reverse lasts 1s so the chime cannot surface.
  pcm_frames = int(0.8 / DT_CTRL)
  for frame in range(1, pcm_frames + 1):
    am.process_alerts(frame, set())
    shown = am.current_alert
    assert shown.audible_alert != AudibleAlert.warningImmediate
    assert shown.alert_text_1 != "TAKE CONTROL IMMEDIATELY"
    if frame <= int(1.0 / DT_CTRL):
      assert shown.audible_alert != AudibleAlert.disengage

  ev = Events()
  ev.add(EventName.reverseGear)
  ev.add(EventName.pcmDisable)
  alerts = ev.create_alerts([ET.USER_DISABLE], list(args))
  by_type = {a.alert_type: a for a in alerts}
  assert "reverseGear/userDisable" in by_type
  quiet = by_type["reverseGear/userDisable"]
  assert quiet.audible_alert == AudibleAlert.none
  assert quiet.alert_size == AlertSize.none


def test_fcw_aeb_alerts_still_critical():
  """Quiet reverse must not weaken collision alerts."""
  aeb = EVENTS[EventName.stockAeb][ET.PERMANENT]
  assert aeb.alert_status == AlertStatus.critical
  assert aeb.priority == Priority.HIGHEST
  fcw = EVENTS[EventName.fcw][ET.PERMANENT]
  assert fcw.alert_status == AlertStatus.critical
  assert fcw.priority == Priority.HIGHEST
  assert fcw.visual_alert == VisualAlert.fcw


def test_leave_drive_mismatch_clear_still_holds_in_reverse():
  assert preap_not_in_drive_clears_mismatch(fingerprint=PREAP_FINGERPRINT, gear="reverse")
  assert not preap_not_in_drive_clears_mismatch(fingerprint=PREAP_FINGERPRINT, gear="drive")
