"""Pre-AP brake pauses long only.

longActive and the logged accel drop while the driver brake is down or
long is paused. brakePressed stays false so the press does not cancel
lateral. A brake that outlives the pedal RELEASE by more than 0.1 s is
an error event with no alert and no chime.
"""
from pathlib import Path
from types import SimpleNamespace

from cereal import car, log
from opendbc.car.tesla.preap.constants import PEDAL_LONG_K_BP, PEDAL_LONG_KI_V, PEDAL_LONG_KP_V

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.preap_driver_brake import (
  BRAKE_LONG_OVERLAP_S,
  BrakeLongOverlap,
  brake_signal_disables,
  driver_brake_applied,
  preap_longitudinal_active,
  preap_pedal_long,
  published_long_accel,
)
from openpilot.selfdrive.selfdrived.events import ET, EVENTS, Events
from openpilot.selfdrive.selfdrived.state import ACTIVE_STATES, StateMachine

EventName = log.OnroadEvent.EventName
State = log.SelfdriveState.OpenpilotState
REPO = Path(__file__).resolve().parents[3]


def _cp(**kwargs):
  cp = SimpleNamespace(
    brand="tesla",
    carFingerprint="TESLA_MODEL_S_PREAP",
    openpilotLongitudinalControl=True,
    pcmCruise=False,
  )
  for key, value in kwargs.items():
    setattr(cp, key, value)
  return cp


def _active(**kwargs):
  params = dict(
    enabled=True,
    longitudinal_override=False,
    openpilot_longitudinal=True,
    preap_pedal=True,
    enable_long_control=True,
    driver_brake=False,
  )
  params.update(kwargs)
  return preap_longitudinal_active(**params)


def _disables(**kwargs):
  params = dict(
    preap_pedal=True,
    driver_brake=False,
    prev_driver_brake=False,
    brake_pressed=False,
    prev_brake_pressed=False,
    regen_braking=False,
    prev_regen_braking=False,
    standstill=False,
  )
  params.update(kwargs)
  return brake_signal_disables(**params)


def test_preap_pedal_long_is_the_software_cruise_fingerprint():
  assert preap_pedal_long(_cp())
  assert not preap_pedal_long(_cp(pcmCruise=True, openpilotLongitudinalControl=False))
  assert not preap_pedal_long(_cp(brand="toyota", carFingerprint="TOYOTA_RAV4"))


def test_driver_brake_applied_ignores_brake_pressed():
  assert driver_brake_applied(SimpleNamespace(driverBrakeApplied=True, brakePressed=False, brake=0.0))
  assert not driver_brake_applied(SimpleNamespace(driverBrakeApplied=False, brakePressed=True, brake=0.0))
  assert driver_brake_applied(SimpleNamespace(brakePressed=False, brake=1.0))
  assert not driver_brake_applied(SimpleNamespace(brakePressed=False, brake=0.0))


def test_long_active_is_false_while_braking_or_paused_and_only_on_preap():
  assert not _active(driver_brake=True)
  assert not _active(enable_long_control=False)
  assert _active()
  assert not _active(longitudinal_override=True)
  assert not _active(enabled=False)
  # Stock cars still publish long while brakePressed-style flags are set.
  # Pre-AP disengage is not this gate.
  assert _active(preap_pedal=False, driver_brake=True, enable_long_control=False)


def test_paused_long_logs_zero_accel_not_the_phantom_decel():
  assert published_long_accel(long_active=False, loc_accel=-0.38, preap_pedal=True) == 0.0
  assert published_long_accel(long_active=True, loc_accel=-0.38, preap_pedal=True) == -0.38
  assert published_long_accel(long_active=False, loc_accel=-0.38, preap_pedal=False) == -0.38

  params = car.CarParams.new_message()
  params.brand = "tesla"
  params.carFingerprint = "TESLA_MODEL_S_PREAP"
  params.openpilotLongitudinalControl = True
  params.pcmCruise = False
  params.longitudinalTuning.kpBP = PEDAL_LONG_K_BP
  params.longitudinalTuning.kpV = PEDAL_LONG_KP_V
  params.longitudinalTuning.kiBP = PEDAL_LONG_K_BP
  params.longitudinalTuning.kiV = PEDAL_LONG_KI_V
  params.vEgoStarting = 0.1
  loc = LongControl(params)
  loc.last_output_accel = -0.38
  loc.long_control_state = LongCtrlState.pid
  cs = car.CarState.new_message()
  cs.vEgo = 12.0
  cs.brakePressed = False
  out = loc.update(False, cs, -0.38, False, (-1.5, 2.0))
  assert float(out) == 0.0
  assert loc.long_control_state == LongCtrlState.off
  assert published_long_accel(long_active=False, loc_accel=out, preap_pedal=True) == 0.0


def test_preap_brake_check_reads_the_switch_and_does_not_kill_lateral():
  assert _disables(driver_brake=True, brake_pressed=False)
  # The always-false brakePressed bit is not the Pre-AP source, even if set.
  assert not _disables(driver_brake=False, brake_pressed=True, prev_brake_pressed=False)
  assert _disables(preap_pedal=False, driver_brake=False, brake_pressed=True)
  # Held brake at standstill is not a new edge. Rising still counts.
  assert not _disables(driver_brake=True, prev_driver_brake=True, standstill=True)
  assert _disables(driver_brake=True, prev_driver_brake=False, standstill=True)

  assert set(EVENTS[EventName.gasPressedOverride]) == {ET.OVERRIDE_LONGITUDINAL}
  assert ET.USER_DISABLE not in EVENTS[EventName.gasPressedOverride]
  assert ET.USER_DISABLE in EVENTS[EventName.pedalPressed]
  alert = EVENTS[EventName.gasPressedOverride][ET.OVERRIDE_LONGITUDINAL]
  assert alert.alert_text_1 == "" and alert.alert_text_2 == ""
  assert alert.visual_alert == car.CarControl.HUDControl.VisualAlert.none
  assert alert.audible_alert == car.CarControl.HUDControl.AudibleAlert.none

  machine = StateMachine()
  machine.state = State.enabled
  events = Events()
  events.add(EventName.gasPressedOverride)
  enabled, active = machine.update(events)
  assert enabled and active
  assert machine.state == State.overriding
  assert machine.state in ACTIVE_STATES

  src = (REPO / "selfdrive/selfdrived/selfdrived.py").read_text()
  branch = src.split("if preap_steering_only_brake:", 1)[1].split("else:", 1)[0]
  assert "gasPressedOverride" in branch
  assert "pedalPressed" not in branch
  assert "brake_pressed=driver_brake_applied(CS)" in src
  assert "Long paused" not in src


def test_brake_long_overlap_errors_only_after_a_tenth_and_has_no_alert():
  assert BRAKE_LONG_OVERLAP_S == 0.1
  monitor = BrakeLongOverlap()
  fault = rising = False
  for _ in range(int(BRAKE_LONG_OVERLAP_S / DT_CTRL)):
    fault, rising = monitor.update(driver_brake=True, pedal_long_active=True, dt=DT_CTRL)
  assert not fault and not rising
  fault, rising = monitor.update(driver_brake=True, pedal_long_active=True, dt=DT_CTRL)
  assert fault and rising
  fault, rising = monitor.update(driver_brake=True, pedal_long_active=True, dt=DT_CTRL)
  assert fault and not rising
  fault, rising = monitor.update(driver_brake=True, pedal_long_active=False, dt=DT_CTRL)
  assert not fault and not rising

  # The 14:34 release is one control frame, then pedal long is down.
  monitor = BrakeLongOverlap()
  fault, _ = monitor.update(driver_brake=True, pedal_long_active=True, dt=DT_CTRL)
  assert not fault
  fault, _ = monitor.update(driver_brake=True, pedal_long_active=False, dt=DT_CTRL)
  assert not fault

  assert EVENTS[EventName.preapBrakeLongActive] == {}
  events = Events()
  events.add(EventName.preapBrakeLongActive)
  assert events.create_alerts(
    [ET.WARNING, ET.PERMANENT, ET.USER_DISABLE, ET.SOFT_DISABLE, ET.IMMEDIATE_DISABLE],
  ) == []
  msg = events.to_msg()
  assert len(msg) == 1
  assert msg[0].name == EventName.preapBrakeLongActive
  for flag in (
    "warning", "permanent", "enable", "noEntry", "userDisable",
    "softDisable", "immediateDisable", "overrideLongitudinal", "overrideLateral",
  ):
    assert not getattr(msg[0], flag)
  assert "preapBrakeLongActive" in (REPO / "selfdrive/selfdrived/selfdrived.py").read_text()
  overlay = (REPO / "selfdrive/car/tesla/preap_blinker_lat_pause.py").read_text()
  assert "ret.driverBrakeApplied = bool(applied)" in overlay
  assert "ret.brakePressed" not in overlay


def test_controlsd_wires_long_active_and_the_logged_accel():
  src = (REPO / "selfdrive/controls/controlsd.py").read_text()
  assert "preap_longitudinal_active(" in src
  assert "published_long_accel(" in src
  assert "driver_brake=driver_brake_applied(CS)" in src
  assert "enable_long_control=bool(getattr(CS, \"enableLongControl\", False))" in src
