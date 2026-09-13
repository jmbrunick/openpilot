#!/usr/bin/env python3
import math
from numbers import Number

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_CTRL, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import BlinkerLateralHold, lat_active_with_blinker_pause
from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  PARAM_DRIVER_LAT_HANDOFF, DriverLateralHandoff, apply_lat_authority,
  cs_hands_on_level, cs_real_brake_pressed, handoff_enabled,
  handoff_new_desired_curvature, lat_active_after_handoff,
  pin_desired_curvature_to_measured)
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


class Controls:
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    self.CI = interfaces[self.CP.carFingerprint](self.CP)

    self.sm = messaging.SubMaster(['liveDelay', 'liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'lateralManeuverPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance'], poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'])

    self.steer_limited_by_safety = False
    self.curvature = 0.0
    self.desired_curvature = 0.0
    self.blinker_lat_hold = BlinkerLateralHold()
    # Default On (NAPDriverLatHandoff=1) for Pre-AP. Re-read each cycle so
    # Settings → NAP can turn it Off immediately if gravel/wind misbehave.
    self.lat_handoff = DriverLateralHandoff(
      enabled=handoff_enabled(
        fingerprint=self.CP.carFingerprint,
        param_on=bool(self.params.get_bool(PARAM_DRIVER_LAT_HANDOFF))))
    self._lat_handoff = self.lat_handoff.update(
      engaged=False, lat_would_be_active=False,
      steering_torque=0.0, steering_rate_deg=0.0)

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI, DT_CTRL)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI, DT_CTRL)

  def update(self):
    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, 0.3) or CS.standstill
    # Driver-turn lamps latch through flash gaps. Soft-lat Off still pauses
    # lat on that latch; Soft-lat On keeps latActive and lets handoff
    # inhibit re-enable. ALC keep-alive flashes must not latch a driver
    # turn: gate while laneChangeState != off, and on a stalk tip
    # (LEFT/RIGHT then IDLE within 0.40s) so latActive stays up for
    # DesireHelper. A held stalk past that window is a turn at any speed
    # when ALC is not latched. During ALC / leftover keep-alive,
    # same-direction stalk must stay on >1.0s to steal ALC as a turn.
    alc_active = model_v2.meta.laneChangeState != LaneChangeState.off
    # Soft yield frees the EPS the same way blinker pause does (latActive
    # false → DAS_steeringControlType=0). Do not keep latActive and track
    # measured angle — that is follow-the-rim holding, not a free wheel.
    # Handoff still sees the pre-yield bit so it does not reset itself.
    # Longitudinal / enabled are untouched. ALC is not this path.
    # Soft-lat On: lamp latch must not clear latActive. Soft-lat Off:
    # keep today's blinker lat-pause so a held turn still frees the wheel.
    self.lat_handoff.enabled = handoff_enabled(
      fingerprint=self.CP.carFingerprint,
      param_on=bool(self.params.get_bool(PARAM_DRIVER_LAT_HANDOFF)))
    lat_would_be_active = lat_active_with_blinker_pause(
      active=self.sm['selfdriveState'].active,
      steer_fault_temporary=CS.steerFaultTemporary,
      steer_fault_permanent=CS.steerFaultPermanent,
      standstill=standstill,
      steer_at_standstill=self.CP.steerAtStandstill,
      left_blinker=CS.leftBlinker,
      right_blinker=CS.rightBlinker,
      steering_pressed=CS.steeringPressed,
      steering_disengage=bool(getattr(CS, 'steeringDisengage', False)),
      engaged=bool(self.sm['selfdriveState'].enabled),
      hold=self.blinker_lat_hold,
      alc_active=alc_active,
      v_ego=CS.vEgo,
      stalk_state=getattr(CS, 'turnSignalStalkState', 0),
      soft_lat_on=self.lat_handoff.enabled,
    )
    CC.longActive = CC.enabled and not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and self.CP.openpilotLongitudinalControl
    # turn_active is the latched driver-turn blinker (not ALC, not the
    # post-turn hand-on hold). Soft-lat uses it as re-enable inhibit +
    # falling-edge enter-yield. Soft-lat Off ignores it (identity).
    self._lat_handoff = self.lat_handoff.update(
      engaged=bool(CC.enabled),
      lat_would_be_active=bool(lat_would_be_active),
      steering_torque=float(CS.steeringTorque),
      steering_rate_deg=float(CS.steeringRateDeg),
      alc_active=alc_active,
      blinker_paused=bool(self.blinker_lat_hold.turn_active),
      hands_on_level=cs_hands_on_level(CS),
      tracking_error=float(self.desired_curvature - self.curvature),
      brake_applied=cs_real_brake_pressed(CS),
      a_ego=float(CS.aEgo),
      v_ego=float(CS.vEgo),
    )
    CC.latActive = lat_active_after_handoff(
      lat_would_be_active, self._lat_handoff.yielded)

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Keep the indicator flashing while ALC is armed or in progress. Pre-AP
    # carcontroller TXes DAS_bodyControls from CC.leftBlinker / rightBlinker.
    CC.leftBlinker, CC.rightBlinker = DesireHelper.lane_change_keep_blinker(
      model_v2.meta.laneChangeState, model_v2.meta.laneChangeDirection)

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    actuators.accel = float(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, pid_accel_limits))

    # Steering PID loop and lateral MPC
    # Reset desired curvature to current to avoid violating the limits on engage.
    # Yield (and soft-lat Off blinker pause) clear latActive (EPS free).
    # Pin to the wheel while lat is down so resume clips from there.
    # Soft-lat On does not drop lat on lamp latch; after a yielded turn
    # the falling blinker enters yield and the normal 0.15 s confirm +
    # 1 s blend owns take-back. Hands still on stay yielded.
    if self.sm.valid['lateralManeuverPlan']:
      model_or_plan_curvature = self.sm['lateralManeuverPlan'].desiredCurvature
    else:
      model_or_plan_curvature = model_v2.action.desiredCurvature
    new_desired_curvature = handoff_new_desired_curvature(
      yielded=bool(self._lat_handoff.yielded),
      lat_active=bool(CC.latActive),
      model_curvature=model_or_plan_curvature,
      measured_curvature=self.curvature,
    )
    if pin_desired_curvature_to_measured(
        self._lat_handoff.yielded, bool(CC.latActive)):
      self.desired_curvature = self.curvature
      curvature_limited = False
    else:
      self.desired_curvature, curvature_limited = clip_curvature(
        CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    lat_delay = self.sm["liveDelay"].lateralDelay + LAT_SMOOTH_SECONDS

    actuators.curvature = self.desired_curvature
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_safety, self.desired_curvature,
                                                       curvature_limited, lat_delay)
    # Blend only while lat is actually requesting and authority < 1.
    # While yielded, latActive is false — do not follow-measured-angle.
    # Do not write blended values back into desired_curvature.
    if CC.latActive and self._lat_handoff.authority < 1.0:
      steer, steeringAngleDeg, blended_curvature = apply_lat_authority(
        self._lat_handoff.authority, steer, steeringAngleDeg, CS.steeringAngleDeg,
        self.desired_curvature, self.curvature)
      actuators.torque = float(steer)
      actuators.steeringAngleDeg = float(steeringAngleDeg)
      actuators.curvature = float(blended_curvature)
    else:
      actuators.torque = float(steer)
      actuators.steeringAngleDeg = float(steeringAngleDeg)
    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish(self, CC, lac_log):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    CC.currentCurvature = self.curvature
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    # Soft-lat emergency hard-brake: request the normal cancel path (stock
    # CC spoof / session teardown). Distinct from silent long pause.
    emergency_cancel = bool(
      self.lat_handoff.enabled and self._lat_handoff.emergency_cancel)
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (
      not CC.enabled or not self.CP.pcmCruise or emergency_cancel)
    CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and not self.sm['longitudinalPlan'].shouldStop

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    if self.sm['selfdriveState'].active:
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_safety = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_safety = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    if self.lat_handoff.enabled:
      cs.latAuthority = float(self._lat_handoff.authority)
      cs.latHandoffPaused = bool(self._lat_handoff.ui_paused)
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool((self.sm['driverMonitoringState'].alertLevel == log.DriverMonitoringState.AlertLevel.three) or
                         (self.sm['selfdriveState'].state == State.softDisabling))

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
