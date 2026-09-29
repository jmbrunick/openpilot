"""Armed lane change prompt, with a "Lane lines unclear" state.

DesireHelper keeps a confirmed lane change armed (and retries the target
lock) when the lane line to cross is not confident. Once the retry runs out
it flags that in modelV2.meta.laneChangeSignalsRemaining (an otherwise
unused countdown; no capnp change) and this prompt tells the driver why
nothing is happening. Installed over the stock prompts by selfdrived.
"""
from cereal import car, log
import cereal.messaging as messaging
from openpilot.selfdrive.controls.lib.lane_change_target import LANE_LINES_UNCLEAR_SIGNAL
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, Alert, AlertCallbackType, Priority
from openpilot.system.hardware import HARDWARE

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
EventName = log.OnroadEvent.EventName


def pre_lane_change_alert(left: bool, mici: bool) -> AlertCallbackType:
  side = "Left" if left else "Right"

  def alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
    try:
      unclear = sm['modelV2'].meta.laneChangeSignalsRemaining == LANE_LINES_UNCLEAR_SIGNAL
    except Exception:
      unclear = False
    if unclear:
      if mici:
        return Alert("Lane lines unclear", "Steer to Change Lane When Clear", AlertStatus.userPrompt, AlertSize.mid,
                     Priority.LOW, VisualAlert.none, AudibleAlert.none, .1)
      return Alert("Lane lines unclear", "", AlertStatus.userPrompt, AlertSize.small,
                   Priority.LOW, VisualAlert.none, AudibleAlert.none, .1)
    if mici:
      return Alert(f"Steer {side}", "Confirm Lane Change", AlertStatus.normal, AlertSize.mid,
                   Priority.LOW, VisualAlert.none, AudibleAlert.none, .1)
    return Alert(f"Steer {side} to Start Lane Change Once Safe", "", AlertStatus.normal, AlertSize.small,
                 Priority.LOW, VisualAlert.none, AudibleAlert.none, .1)
  return alert


def install_lane_change_alerts(events=EVENTS, mici: bool | None = None) -> None:
  if mici is None:
    mici = HARDWARE.get_device_type() == 'mici'
  events[EventName.preLaneChangeLeft] = {ET.WARNING: pre_lane_change_alert(True, mici)}
  events[EventName.preLaneChangeRight] = {ET.WARNING: pre_lane_change_alert(False, mici)}
