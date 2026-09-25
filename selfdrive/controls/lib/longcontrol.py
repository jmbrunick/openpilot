import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.controls.lib.lead_approach import (
  LeadResidualWindow,
  guard_follow_actuator_regen,
)
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState
PREAP_FINGERPRINT = "TESLA_MODEL_S_PREAP"


def _is_tesla_preap_long(CP) -> bool:
  return (getattr(CP, "brand", None) == "tesla"
          and getattr(CP, "carFingerprint", None) == PREAP_FINGERPRINT
          and bool(getattr(CP, "openpilotLongitudinalControl", False))
          and not bool(getattr(CP, "pcmCruise", True)))


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    # ~0.5 s of lead v_rel and measured aEgo. One 100 Hz radar step
    # must not read as a #222 residual brake.
    self._lead_residual = LeadResidualWindow()

  def reset(self):
    self.pid.reset()
    self._lead_residual.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits,
             lead_v_rel=None, lead_d_rel=None, lead_slack=None, lead_fcw=False,
             lead_v_ego=None, lead_v_cruise=None, lead_a_lead=None):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    # Pre-AP follow comfort: coast / MILD must not dump firm regen.
    # Mid-gap slow close hard-caps to slight lift. Stopping / starting /
    # off keep stock authority. Planner ≤ −0.5 outside that settle,
    # FCW, and a near bumper still reach the plant.
    if (self.long_control_state == LongCtrlState.pid
        and _is_tesla_preap_long(self.CP)):
      if lead_v_rel is None:
        self._lead_residual.reset()
        prev_v_rel, residual_dt, residual_a = (
          None, DT_CTRL, float(getattr(CS, "aEgo", 0.0)),
        )
      else:
        prev_v_rel, residual_dt, residual_a = self._lead_residual.update(
          float(lead_v_rel), float(getattr(CS, "aEgo", 0.0)), DT_CTRL,
        )
      self.last_output_accel = float(guard_follow_actuator_regen(
        self.last_output_accel, a_target,
        v_rel=lead_v_rel, d_rel=lead_d_rel, slack=lead_slack, fcw=lead_fcw,
        v_ego=lead_v_ego, v_cruise=lead_v_cruise, a_lead=lead_a_lead,
        prev_v_rel=prev_v_rel, a_ego=residual_a,
        dt=residual_dt, should_stop=bool(should_stop),
      ))
    return self.last_output_accel
