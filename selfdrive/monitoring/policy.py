from collections import defaultdict
from math import atan2, radians
import math
import random
import numpy as np

from cereal import car, log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_DMON
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.stat_live import RunningStatFilter
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.selfdrive.monitoring.dm_toggles import (
  DEFAULT_FALSE_ALERT_IGNORE, DEFAULT_SIMULATE_LOOKING,
  exclusive_dm_toggle_states, read_exclusive_dm_toggles,
)

try:
  from openpilot.common.params import Params
except Exception:  # params_pyx not built in some unit-test hosts
  Params = None

AlertLevel = log.DriverMonitoringState.AlertLevel
MonitoringPolicy = log.DriverMonitoringState.MonitoringPolicy

def to_percent(v):
  return int(min(max(v * 100., 0.), 100.))

# ******************************************************************************************
#  NOTE: To fork maintainers.
#  Disabling or nerfing safety features will get you and your users banned from our servers.
#  We recommend that you do not change these numbers from the defaults.
# ******************************************************************************************

# NAP hidden DM toggles (triple-tap Settings → NAP). Shared cadence:
# after drain starts, wait past 1.0 s, then fire uniform in (1.0 s, 3.0 s].
# Hold until gradual recovery returns awareness to 1.0. After a full
# reset the same rule applies to the next countdown.
# Simulate Look On = pre-FAI full looking-path wipe on that cadence
# (face + filter + driver_distracted + pose/eye/phone type bits). That
# is the original "do not nag" recovery for no-face, uncertain, phone,
# pose, and eye false nags. False Alert Ignore On (Sim Look Off) =
# phone-only soft-clear; pose and eye still drain. Mutually exclusive
# — only one may be On (both Off is allowed). nap-dev defaults:
# Simulate Look On / FAI Off. Stale both-On → Simulate Look On /
# FAI Off. Hands-on ≥ 2 / stalk / door / reverse unchanged.
LOOK_SIM_COUNTDOWN_MIN_S = 1.0
LOOK_SIM_RANDOM_WINDOW_S = 2.0
LOOK_SIM_FIRE_MAX_S = LOOK_SIM_COUNTDOWN_MIN_S + LOOK_SIM_RANDOM_WINDOW_S  # 3.0
# Always hold at least this long so recovery is not a single tick.
LOOK_SIM_HOLD_MIN_S = 0.5
# Covers wheeltouch recovery from empty awareness (~11 s) plus margin.
LOOK_SIM_HOLD_MAX_S = 12.0
LOOK_SIM_MODE_PHONE = "phone"
LOOK_SIM_MODE_GLANCE = "glance"
# Stock looking-path: filter.x below this + face + low pose std.
VISION_LOOKING_FILTER_X = 0.37
VISION_RECOVERY_FACTOR_MAX = 5.0
VISION_RECOVERY_FACTOR_MIN = 1.25
# Pause looking-away / distraction / eyes-off-road alerts below this speed
# (same stock standstill exemption, plus creeping). Tesla CS.standstill is
# only true when fully stopped; 1 mph still nags at a light. Justin: "below
# two miles an hour, or maybe even below one" — gate is 2 mph (strictly
# below). Toggles stay as-is; this is not Simulate Look / FAI Off.
# controlsd lat standstill (~0.3 m/s) is too low for creep. FCW/AEB and
# hard cancels are unchanged.
DM_LOOKAWAY_GATE_MPH = 2.0
DM_LOOKAWAY_GATE_MS = DM_LOOKAWAY_GATE_MPH * CV.MPH_TO_MS


def _param_bool(name: str, default: bool) -> bool:
  if Params is None:
    return default
  try:
    return bool(Params().get_bool(name))
  except Exception:
    return default


def vision_looking_path(face_detected, low_std, distraction_filter_x) -> bool:
  """True when stock DM treats the driver as looking / attentive."""
  return bool(face_detected and low_std and distraction_filter_x < VISION_LOOKING_FILTER_X)


def lookaway_alerts_paused(standstill, car_speed) -> bool:
  """True when looking-away / distraction alerts must not fire.

  Stock already pauses at CS.standstill before the green prompt. Creeping
  below DM_LOOKAWAY_GATE_MPH is not standstill on Pre-AP, so OR that in.
  Independent of Simulate Look / FAI.
  """
  return bool(standstill or abs(float(car_speed)) < DM_LOOKAWAY_GATE_MS)


def looking_recovery_time_s(awareness, alert_3_timeout,
                            rmax=VISION_RECOVERY_FACTOR_MAX,
                            rmin=VISION_RECOVERY_FACTOR_MIN) -> float:
  """Seconds of stock looking-path recovery to return awareness to 1.0.

  Integrates da/dt = ((Rmax-Rmin)*(1-a)+Rmin)/T3 until a=1.
  """
  u0 = max(0.0, 1.0 - float(awareness))
  if u0 <= 1e-12 or alert_3_timeout <= 0:
    return 0.0
  k = (rmax - rmin) / alert_3_timeout
  c = rmin / alert_3_timeout
  return math.log((k * u0 + c) / c) / k


class DRIVER_MONITOR_SETTINGS:
  def __init__(self):
    # https://eur-lex.europa.eu/legal-content/EN/TXT/PDF/?uri=CELEX:42018X1947&rid=2
    self._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT = 15.
    self._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT = 24.
    self._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT = 30.
    # https://cdn.euroncap.com/cars/assets/euro_ncap_protocol_safe_driving_driver_engagement_v11_a30e874152.pdf
    self._VISION_POLICY_ALERT_1_TIMEOUT = 3.
    self._VISION_POLICY_ALERT_2_TIMEOUT = 5.
    self._VISION_POLICY_ALERT_3_TIMEOUT = 11.

    self._TIMEOUT_RECOVERY_FACTOR_MAX = 5.
    self._TIMEOUT_RECOVERY_FACTOR_MIN = 1.25

    self._MAX_TERMINAL_ALERTS = 3  # not allowed to engage after 3 terminal alerts
    self._MAX_TERMINAL_DURATION = int(30 / DT_DMON)  # not allowed to engage after 30s of terminal alerts

    self._FACE_THRESHOLD = 0.7
    self._EYE_THRESHOLD = 0.65
    self._SG_THRESHOLD = 0.9
    self._BLINK_THRESHOLD = 0.865
    self._PHONE_THRESH = 0.5
    self._POSE_PITCH_THRESHOLD = 0.3133
    self._POSE_PITCH_THRESHOLD_SLACK = 0.3237
    self._POSE_PITCH_THRESHOLD_STRICT = self._POSE_PITCH_THRESHOLD
    self._POSE_YAW_THRESHOLD = 0.4020
    self._POSE_YAW_THRESHOLD_SLACK = 0.5042
    self._POSE_YAW_THRESHOLD_STRICT = self._POSE_YAW_THRESHOLD
    self._POSE_YAW_MIN_STEER_DEG = 30
    self._POSE_YAW_STEER_FACTOR = 0.15
    self._POSE_YAW_STEER_MAX_OFFSET = 0.3927
    self._PITCH_NATURAL_OFFSET = 0.011 # initial value before offset is learned
    self._PITCH_NATURAL_THRESHOLD = 0.449
    self._YAW_NATURAL_OFFSET = 0.075 # initial value before offset is learned
    self._PITCH_NATURAL_VAR = 3*0.01
    self._YAW_NATURAL_VAR = 3*0.05
    self._PITCH_MAX_OFFSET = 0.124
    self._PITCH_MIN_OFFSET = -0.0881
    self._YAW_MAX_OFFSET = 0.289
    self._YAW_MIN_OFFSET = -0.0246

    self._DCAM_UNCERTAIN_ALERT_THRESHOLD = 0.1
    self._DCAM_UNCERTAIN_ALERT_COUNT = int(60  / DT_DMON)
    self._DCAM_UNCERTAIN_RESET_COUNT = int(2  / DT_DMON)
    self._HI_STD_THRESHOLD = 0.3
    self._HI_STD_FALLBACK_TIME = int(10  / DT_DMON)  # fall back to wheel touch if model is uncertain for 10s
    self._DISTRACTED_FILTER_TS = 0.25  # 0.6Hz

    self._POSE_CALIB_MIN_SPEED = 13  # 30 mph
    self._POSE_OFFSET_MIN_COUNT = int(60 / DT_DMON)  # valid data counts before calibration completes, 1min cumulative
    self._POSE_OFFSET_MAX_COUNT = int(360 / DT_DMON)  # stop deweighting new data after 6 min, aka "short term memory"
    self._WHEELPOS_CALIB_MIN_SPEED = 11
    self._WHEELPOS_THRESHOLD = 0.5
    self._WHEELPOS_FILTER_MIN_COUNT = int(15 / DT_DMON) # allow 15 seconds to converge wheel side
    self._WHEELPOS_DATA_AVG = 0.03
    self._WHEELPOS_DATA_VAR = 3*5.5e-5
    self._WHEELPOS_MAX_COUNT = -1

class DriverPose:
  def __init__(self, settings):
    pitch_filter_raw_priors = (settings._PITCH_NATURAL_OFFSET, settings._PITCH_NATURAL_VAR, 2)
    yaw_filter_raw_priors = (settings._YAW_NATURAL_OFFSET, settings._YAW_NATURAL_VAR, 2)
    self.yaw = 0.
    self.pitch = 0.
    self.pitch_offsetter = RunningStatFilter(raw_priors=pitch_filter_raw_priors, max_trackable=settings._POSE_OFFSET_MAX_COUNT)
    self.yaw_offsetter = RunningStatFilter(raw_priors=yaw_filter_raw_priors, max_trackable=settings._POSE_OFFSET_MAX_COUNT)
    self.calibrated = False
    self.low_std = True
    self.cfactor_pitch = 1.
    self.cfactor_yaw = 1.
    self.steer_yaw_offset = 0.

class DriverBlink:
  def __init__(self):
    self.left = 0.
    self.right = 0.

# model output refers to center of undistorted+leveled image
ref_undistorted_cam = DEVICE_CAMERAS[("tici", "ar0231")].dcam
dcam_undistorted_FL = 598.0
dcam_undistorted_W, dcam_undistorted_H = (ref_undistorted_cam.width, ref_undistorted_cam.height)

def face_orientation_from_model(orient_model, pos_model, rpy_calib):
  pitch_model = orient_model[0]
  yaw_model = orient_model[1]

  face_pixel_position = ((pos_model[0]+0.5)*dcam_undistorted_W, (pos_model[1]+0.5)*dcam_undistorted_H)
  yaw_focal_angle = atan2(face_pixel_position[0] - dcam_undistorted_W//2, dcam_undistorted_FL)
  pitch_focal_angle = atan2(face_pixel_position[1] - dcam_undistorted_H//2, dcam_undistorted_FL)

  pitch = pitch_model + pitch_focal_angle
  yaw = -yaw_model + yaw_focal_angle

  pitch -= rpy_calib[1]
  yaw -= rpy_calib[2]
  return pitch, yaw


class DriverMonitoring:
  def __init__(self, rhd_saved=False, settings=None, always_on=False):
    # init policy settings
    self.settings = settings if settings is not None else DRIVER_MONITOR_SETTINGS()

    # init driver status
    wheelpos_filter_raw_priors = (self.settings._WHEELPOS_DATA_AVG, self.settings._WHEELPOS_DATA_VAR, 2)
    self.wheelpos_offsetter = RunningStatFilter(raw_priors=wheelpos_filter_raw_priors, max_trackable=self.settings._WHEELPOS_MAX_COUNT)
    self.pose = DriverPose(settings=self.settings)
    self.blink = DriverBlink()
    self.phone_prob = 0.

    self.alert_level = AlertLevel.none
    self.always_on = always_on
    self.distracted_types = defaultdict(bool)
    self.driver_distracted = False
    self.driver_distraction_filter = FirstOrderFilter(0., self.settings._DISTRACTED_FILTER_TS, DT_DMON)
    self.wheel_on_right = False
    self.wheel_on_right_last = None
    self.wheel_on_right_default = rhd_saved
    self.face_detected = False
    self.terminal_alert_cnt = 0
    self.terminal_time = 0
    self.step_change = 0.
    self.active_policy = MonitoringPolicy.vision
    self.driver_interacting = False
    self.is_model_uncertain = False
    self.hi_stds = 0
    self.model_std_max = 0.
    self.threshold_alert_1 = 0.
    self.threshold_alert_2 = 0.
    self.dcam_uncertain_cnt = 0
    self.dcam_reset_cnt = 0
    self.too_distracted = _param_bool("DriverTooDistracted", False)
    # nap-dev: Simulate Look On / FAI Off. Both-On resolves to Sim On.
    if Params is not None:
      try:
        sim, fai = read_exclusive_dm_toggles(Params())
      except Exception:
        sim, fai = DEFAULT_SIMULATE_LOOKING, DEFAULT_FALSE_ALERT_IGNORE
    else:
      sim, fai = DEFAULT_SIMULATE_LOOKING, DEFAULT_FALSE_ALERT_IGNORE
    self.nap_dm_simulate_looking = sim
    self.nap_dm_false_alert_ignore = fai
    self._rng = random.Random()
    self._look_sim_countdown_s = 0.0
    self._look_sim_holding = False
    self._look_sim_hold_s = 0.0
    self._look_sim_hold_start_awareness = 1.0
    self._look_sim_mode = None
    self.car_speed = 0.0
    self._redraw_look_sim_interval()

    self._reset_awareness()
    self._set_policy(MonitoringPolicy.vision)

  def _redraw_look_sim_interval(self):
    """Fire time in (1.0 s, 3.0 s] after drain start: 1 s + uniform (0, 2 s]."""
    # random() is [0, 1); (1-u)*window is (0, 2].
    delay = (1.0 - self._rng.random()) * LOOK_SIM_RANDOM_WINDOW_S
    self._look_sim_fire_s = LOOK_SIM_COUNTDOWN_MIN_S + delay

  def _look_sim_active(self) -> bool:
    return bool(self.nap_dm_simulate_looking or self.nap_dm_false_alert_ignore)

  def _enforce_exclusive_dm_toggles(self):
    """Runtime guard: both-On → Simulate Look wins; stop the other path."""
    sim, fai = exclusive_dm_toggle_states(
      self.nap_dm_simulate_looking, self.nap_dm_false_alert_ignore)
    self.nap_dm_simulate_looking = sim
    self.nap_dm_false_alert_ignore = fai
    self._stop_inactive_dm_path()

  def _stop_inactive_dm_path(self):
    """Abort full-wipe hold if Sim Off; end phone soft-clear if FAI Off."""
    if not self._look_sim_holding:
      return
    if self._look_sim_mode == LOOK_SIM_MODE_GLANCE and not self.nap_dm_simulate_looking:
      self._abort_look_sim_hold()
    elif self._look_sim_mode == LOOK_SIM_MODE_PHONE and not self.nap_dm_false_alert_ignore:
      self._end_look_sim_hold(redraw=True)

  def set_nap_dm_toggles(self, *, simulate_looking=None, false_alert_ignore=None):
    """Apply a live toggle flip (or a resolved pair) and stop the other path.

    Turning Simulate Look On clears FAI and ends a phone hold. Turning FAI
    On clears Simulate Look and aborts an in-flight glance hold. Passing
    both True still resolves to Simulate Look On / FAI Off.
    """
    if simulate_looking is not None and false_alert_ignore is not None:
      self.nap_dm_simulate_looking, self.nap_dm_false_alert_ignore = (
        exclusive_dm_toggle_states(simulate_looking, false_alert_ignore))
      self._stop_inactive_dm_path()
      return
    if simulate_looking is not None:
      self.nap_dm_simulate_looking = bool(simulate_looking)
      if self.nap_dm_simulate_looking:
        self.nap_dm_false_alert_ignore = False
    if false_alert_ignore is not None:
      self.nap_dm_false_alert_ignore = bool(false_alert_ignore)
      if self.nap_dm_false_alert_ignore:
        self.nap_dm_simulate_looking = False
    self._enforce_exclusive_dm_toggles()

  def _pose_or_eye_alarming(self) -> bool:
    return bool(self.distracted_types['pose'] or self.distracted_types['eye'])

  def _phone_soft_clear_eligible(self) -> bool:
    """False device only: phone bit, face + low std, pose/eye not alarming."""
    return (self.nap_dm_false_alert_ignore
            and bool(self.distracted_types['phone'])
            and not self._pose_or_eye_alarming()
            and self.face_detected
            and self.pose.low_std)

  def _simulate_look_wipe_eligible(self) -> bool:
    """Simulate Look On: any soft drain is eligible for the full wipe."""
    return bool(self.nap_dm_simulate_looking)

  def _apply_phone_soft_clear(self):
    """Ignore phoneProb / phone distraction only. Pose and eye stay live."""
    self.distracted_types['phone'] = False
    self.driver_distracted = (any(self.distracted_types.values())
                              and self.face_detected and self.pose.low_std)
    if not self.driver_distracted:
      self.driver_distraction_filter.x = 0.0

  def _apply_simulated_looking(self) -> bool:
    """Pre-FAI full looking-path wipe. Recovers no-face / uncertain / phone /
    pose / eye nags. Does not mute hard cancels (those are outside DM)."""
    self.face_detected = True
    self.pose.low_std = True
    self.is_model_uncertain = False
    self.driver_distracted = False
    self.driver_distraction_filter.x = 0.0
    self.distracted_types['pose'] = False
    self.distracted_types['eye'] = False
    self.distracted_types['phone'] = False
    return True

  def _abort_look_sim_hold(self):
    """Stop a hold without consuming the fire window — pose/eye must drain."""
    self._look_sim_holding = False
    self._look_sim_hold_s = 0.0
    self._look_sim_mode = None

  def _end_look_sim_hold(self, redraw=True):
    self._look_sim_holding = False
    self._look_sim_hold_s = 0.0
    self._look_sim_countdown_s = 0.0
    self._look_sim_mode = None
    if redraw:
      self._redraw_look_sim_interval()

  def _look_sim_hold_done(self) -> bool:
    if not self._look_sim_holding:
      return False
    if self._look_sim_hold_s + 1e-9 >= LOOK_SIM_HOLD_MAX_S:
      return True
    return (self.awareness >= 1.0 - 1e-9 and
            self._look_sim_hold_s + 1e-9 >= LOOK_SIM_HOLD_MIN_S)

  def _start_look_sim_hold(self, mode):
    self._look_sim_holding = True
    self._look_sim_mode = mode
    self._look_sim_hold_s = DT_DMON
    self._look_sim_hold_start_awareness = self.awareness

  def _maybe_simulate_looking(self, op_engaged, allow_look_sim):
    """Shared 1–3 s cadence for False Alert Ignore and Simulate Look."""
    self._enforce_exclusive_dm_toggles()
    if not (allow_look_sim and self._look_sim_active() and op_engaged):
      if not op_engaged:
        self._end_look_sim_hold(redraw=False)
      return

    if self._look_sim_holding:
      if self._look_sim_mode == LOOK_SIM_MODE_PHONE:
        # FAI phone-only: pose/eye must still drain.
        if self._pose_or_eye_alarming():
          self._abort_look_sim_hold()
          # A phone hold snaps the distraction filter to 0. Restore it so a
          # new pose/eye alarm is not treated as looking for ~0.25 s.
          if self.driver_distracted:
            self.driver_distraction_filter.x = max(
              self.driver_distraction_filter.x, 0.64)
          return
        if not self.nap_dm_false_alert_ignore:
          self._end_look_sim_hold(redraw=True)
          return
        self._apply_phone_soft_clear()
      else:
        # Simulate Look: keep the full wipe even if pose/eye/phone return.
        if not self.nap_dm_simulate_looking:
          self._abort_look_sim_hold()
          return
        self._apply_simulated_looking()
      self._look_sim_hold_s += DT_DMON
      self._look_sim_countdown_s = 0.0
      # Stock looking recovery requires awareness > 0 (red does not climb).
      if self.awareness <= 0. and self._look_sim_hold_s + 1e-9 >= LOOK_SIM_HOLD_MIN_S:
        self._reset_awareness()
      if self._look_sim_hold_done():
        self._end_look_sim_hold(redraw=True)
      return

    if self.awareness < 1.0:
      self._look_sim_countdown_s += DT_DMON
    else:
      self._look_sim_countdown_s = 0.0
      return
    # Past 1.0 s of drain, then the drawn time in (1.0, 3.0].
    if (self._look_sim_countdown_s > LOOK_SIM_COUNTDOWN_MIN_S and
        self._look_sim_countdown_s + 1e-9 >= self._look_sim_fire_s):
      if self._phone_soft_clear_eligible():
        self._start_look_sim_hold(LOOK_SIM_MODE_PHONE)
        self._apply_phone_soft_clear()
      elif self._simulate_look_wipe_eligible():
        self._start_look_sim_hold(LOOK_SIM_MODE_GLANCE)
        self._apply_simulated_looking()

  def _reset_awareness(self):
    self.awareness = 1.
    self.last_vision_awareness = 1.
    self.last_wheeltouch_awareness = 1.
    self._look_sim_countdown_s = 0.0

  def _set_policy(self, target_policy):
    if self.active_policy == MonitoringPolicy.vision and self.awareness <= self.threshold_alert_2:
      if target_policy == MonitoringPolicy.vision:
        self.step_change = DT_DMON / self.settings._VISION_POLICY_ALERT_3_TIMEOUT
      else:
        self.step_change = 0.
      return  # no exploit after orange alert
    elif self.awareness <= 0.:
      return

    if target_policy == MonitoringPolicy.vision:
      # when falling back from passive mode to active mode, reset awareness to avoid false alert
      if self.active_policy != MonitoringPolicy.vision:
        self.last_wheeltouch_awareness = self.awareness
        self.awareness = self.last_vision_awareness

      self.threshold_alert_1 = 1. - self.settings._VISION_POLICY_ALERT_1_TIMEOUT / self.settings._VISION_POLICY_ALERT_3_TIMEOUT
      self.threshold_alert_2 = 1. - self.settings._VISION_POLICY_ALERT_2_TIMEOUT / self.settings._VISION_POLICY_ALERT_3_TIMEOUT
      self.step_change = DT_DMON / self.settings._VISION_POLICY_ALERT_3_TIMEOUT
      self.active_policy = MonitoringPolicy.vision
    else:
      if self.active_policy == MonitoringPolicy.vision:
        self.last_vision_awareness = self.awareness
        self.awareness = self.last_wheeltouch_awareness

      self.threshold_alert_1 = 1. - self.settings._WHEELTOUCH_POLICY_ALERT_1_TIMEOUT / self.settings._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT
      self.threshold_alert_2 = 1. - self.settings._WHEELTOUCH_POLICY_ALERT_2_TIMEOUT / self.settings._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT
      self.step_change = DT_DMON / self.settings._WHEELTOUCH_POLICY_ALERT_3_TIMEOUT
      self.active_policy = MonitoringPolicy.wheeltouch

  def _set_pose_strictness(self, brake_disengage_prob, car_speed):
    bp = brake_disengage_prob
    k1 = max(-0.00156*((car_speed-16)**2)+0.6, 0.2)
    bp_normal = max(min(bp / k1, 0.5),0)
    self.pose.cfactor_pitch = np.interp(bp_normal, [0, 0.5],
                                           [self.settings._POSE_PITCH_THRESHOLD_SLACK,
                                            self.settings._POSE_PITCH_THRESHOLD_STRICT]) / self.settings._POSE_PITCH_THRESHOLD
    self.pose.cfactor_yaw = np.interp(bp_normal, [0, 0.5],
                                           [self.settings._POSE_YAW_THRESHOLD_SLACK,
                                            self.settings._POSE_YAW_THRESHOLD_STRICT]) / self.settings._POSE_YAW_THRESHOLD

  def _get_distracted_types(self):
    self.distracted_types = defaultdict(bool)

    if not self.pose.calibrated:
      pitch_error = self.pose.pitch - self.settings._PITCH_NATURAL_OFFSET
      yaw_error = self.pose.yaw - self.settings._YAW_NATURAL_OFFSET
    else:
      pitch_error = self.pose.pitch - min(max(self.pose.pitch_offsetter.filtered_stat.mean(),
                                                       self.settings._PITCH_MIN_OFFSET), self.settings._PITCH_MAX_OFFSET)
      yaw_error = self.pose.yaw - min(max(self.pose.yaw_offsetter.filtered_stat.mean(),
                                                    self.settings._YAW_MIN_OFFSET), self.settings._YAW_MAX_OFFSET)
    pitch_error = 0 if pitch_error > 0 else abs(pitch_error) # no positive pitch limit

    if yaw_error * self.pose.steer_yaw_offset > 0: # unidirectional
      yaw_error = max(abs(yaw_error) - min(abs(self.pose.steer_yaw_offset), self.settings._POSE_YAW_STEER_MAX_OFFSET), 0.)
    else:
      yaw_error = abs(yaw_error)

    pitch_threshold = self.settings._POSE_PITCH_THRESHOLD * self.pose.cfactor_pitch if self.pose.calibrated else self.settings._PITCH_NATURAL_THRESHOLD
    yaw_threshold = self.settings._POSE_YAW_THRESHOLD * self.pose.cfactor_yaw

    self.distracted_types['pose'] = bool((pitch_error > pitch_threshold) or (yaw_error > yaw_threshold))
    self.distracted_types['eye'] = bool((self.blink.left + self.blink.right)*0.5 > self.settings._BLINK_THRESHOLD)
    self.distracted_types['phone'] = bool(self.phone_prob > self.settings._PHONE_THRESH)

  def _update_states(self, driver_state, cal_rpy, car_speed, op_engaged, standstill, demo_mode=False, steering_angle_deg=0.):
    self.car_speed = float(car_speed)
    rhd_pred = driver_state.wheelOnRightProb
    # calibrates only when there's movement and either face detected
    if car_speed > self.settings._WHEELPOS_CALIB_MIN_SPEED and (driver_state.leftDriverData.faceProb > self.settings._FACE_THRESHOLD or
                                          driver_state.rightDriverData.faceProb > self.settings._FACE_THRESHOLD):
      self.wheelpos_offsetter.push_and_update(rhd_pred)

    wheelpos_calibrated = self.wheelpos_offsetter.filtered_stat.n >= self.settings._WHEELPOS_FILTER_MIN_COUNT

    if wheelpos_calibrated or demo_mode:
      self.wheel_on_right = self.wheelpos_offsetter.filtered_stat.M > self.settings._WHEELPOS_THRESHOLD
    else:
      self.wheel_on_right = self.wheel_on_right_default # use default/saved if calibration is unfinished
    # make sure no switching when engaged
    if op_engaged and self.wheel_on_right_last is not None and self.wheel_on_right_last != self.wheel_on_right and not demo_mode:
      self.wheel_on_right = self.wheel_on_right_last
    driver_data = driver_state.rightDriverData if self.wheel_on_right else driver_state.leftDriverData
    if not all(len(x) > 0 for x in (driver_data.faceOrientation, driver_data.facePosition,
                                    driver_data.faceOrientationStd, driver_data.facePositionStd)):
      return

    self.face_detected = driver_data.faceProb > self.settings._FACE_THRESHOLD
    self.pose.pitch, self.pose.yaw = face_orientation_from_model(driver_data.faceOrientation, driver_data.facePosition, cal_rpy)
    steer_d = max(abs(steering_angle_deg) - self.settings._POSE_YAW_MIN_STEER_DEG, 0.)
    self.pose.steer_yaw_offset = radians(steer_d) * -np.sign(steering_angle_deg) * self.settings._POSE_YAW_STEER_FACTOR
    if self.wheel_on_right:
      self.pose.yaw *= -1
      self.pose.steer_yaw_offset *= -1
    self.wheel_on_right_last = self.wheel_on_right
    self.model_std_max = max(driver_data.faceOrientationStd[0], driver_data.faceOrientationStd[1])
    self.pose.low_std = self.model_std_max < self.settings._HI_STD_THRESHOLD
    self.blink.left = driver_data.leftBlinkProb * (driver_data.leftEyeProb > self.settings._EYE_THRESHOLD) \
                      * (driver_data.sunglassesProb < self.settings._SG_THRESHOLD)
    self.blink.right = driver_data.rightBlinkProb * (driver_data.rightEyeProb > self.settings._EYE_THRESHOLD) \
                      * (driver_data.sunglassesProb < self.settings._SG_THRESHOLD)
    self.phone_prob = driver_data.phoneProb

    self._get_distracted_types()
    self.driver_distracted = any(self.distracted_types.values()) and driver_data.faceProb > self.settings._FACE_THRESHOLD and self.pose.low_std
    self.driver_distraction_filter.update(self.driver_distracted)

    # only update offsetter when driver is actively driving the car above a certain speed
    if self.face_detected and car_speed > self.settings._POSE_CALIB_MIN_SPEED and self.pose.low_std and (not op_engaged or not self.driver_distracted):
      self.pose.pitch_offsetter.push_and_update(self.pose.pitch)
      self.pose.yaw_offsetter.push_and_update(self.pose.yaw)

    self.pose.calibrated = self.pose.pitch_offsetter.filtered_stat.n >= self.settings._POSE_OFFSET_MIN_COUNT and \
                           self.pose.yaw_offsetter.filtered_stat.n >= self.settings._POSE_OFFSET_MIN_COUNT

    if self.face_detected and not self.driver_distracted:
      dcam_uncertain = self.model_std_max > self.settings._DCAM_UNCERTAIN_ALERT_THRESHOLD
      if dcam_uncertain and not standstill:
        self.dcam_uncertain_cnt += 1
        self.dcam_reset_cnt = 0
      else:
        self.dcam_reset_cnt += 1
        if self.dcam_reset_cnt > self.settings._DCAM_UNCERTAIN_RESET_COUNT:
          self.dcam_uncertain_cnt = 0

    self.is_model_uncertain = self.hi_stds >= self.settings._HI_STD_FALLBACK_TIME
    self._set_policy(MonitoringPolicy.vision if self.face_detected and not self.is_model_uncertain else MonitoringPolicy.wheeltouch)
    if self.face_detected and not self.pose.low_std and not self.driver_distracted:
      self.hi_stds += 1
    elif self.face_detected and self.pose.low_std:
      self.hi_stds = 0

  def _update_events(self, driver_engaged, op_engaged, standstill, wrong_gear, allow_look_sim=True):
    self.alert_level = AlertLevel.none
    self.driver_interacting = driver_engaged
    self._maybe_simulate_looking(op_engaged, allow_look_sim)

    if self.terminal_alert_cnt >= self.settings._MAX_TERMINAL_ALERTS or \
       self.terminal_time >= self.settings._MAX_TERMINAL_DURATION:
      self.too_distracted = True

    always_on_valid = self.always_on and not wrong_gear
    if (self.driver_interacting and self.awareness > 0 and self.active_policy == MonitoringPolicy.wheeltouch) or \
       (not always_on_valid and not op_engaged) or \
       (always_on_valid and not op_engaged and self.awareness <= 0):
      # always reset on disengage with normal mode; disengage resets only on red if always on
      self._reset_awareness()
      return

    awareness_prev = self.awareness
    _reaching_alert_1 = self.awareness - self.step_change <= self.threshold_alert_1
    _reaching_alert_3 = self.awareness - self.step_change <= 0
    # Stock pauses before green at CS.standstill. Also pause while creeping
    # below DM_LOOKAWAY_GATE_MPH (Sim Look / FAI stay as toggled).
    standstill_exemption = lookaway_alerts_paused(standstill, self.car_speed) and _reaching_alert_1
    always_on_exemption = always_on_valid and not op_engaged and _reaching_alert_3

    looking = vision_looking_path(self.face_detected, self.pose.low_std,
                                  self.driver_distraction_filter.x)
    if self.awareness > 0 and (looking or standstill_exemption):
      if self.driver_interacting:
        self._reset_awareness()
        return
      # only restore awareness when paying attention and alert is not red
      self.awareness = min(self.awareness + ((self.settings._TIMEOUT_RECOVERY_FACTOR_MAX-self.settings._TIMEOUT_RECOVERY_FACTOR_MIN)*
                                             (1.-self.awareness)+self.settings._TIMEOUT_RECOVERY_FACTOR_MIN)*self.step_change, 1.)
      if self.awareness == 1.:
        self.last_wheeltouch_awareness = min(self.last_wheeltouch_awareness + self.step_change, 1.)
      # don't display alert banner when awareness is recovering and has cleared orange
      if self.awareness > self.threshold_alert_2:
        return

    certainly_distracted = self.driver_distraction_filter.x > 0.63 and self.driver_distracted and self.face_detected
    maybe_distracted = self.is_model_uncertain or not self.face_detected

    if certainly_distracted or maybe_distracted:
      # should always be counting if distracted unless at standstill and reaching green
      # also will not be reaching 0 if DM is active when not engaged
      if not (standstill_exemption or always_on_exemption):
        self.awareness = max(self.awareness - self.step_change, -0.1)

    if self.awareness <= 0.:
      # terminal alert: disengagement required
      self.alert_level = AlertLevel.three
      self.terminal_time += 1
      if awareness_prev > 0.:
        self.terminal_alert_cnt += 1
    elif self.awareness <= self.threshold_alert_2:
      self.alert_level = AlertLevel.two
    elif self.awareness <= self.threshold_alert_1:
      self.alert_level = AlertLevel.one

  def get_state_packet(self, valid=True):
    # build driverMonitoringState packet
    import cereal.messaging as messaging
    dat = messaging.new_message('driverMonitoringState', valid=valid)
    dm = dat.driverMonitoringState

    dm.lockout = self.too_distracted
    dm.alertCountLockoutPercent = to_percent(self.terminal_alert_cnt / self.settings._MAX_TERMINAL_ALERTS)
    dm.alertTimeLockoutPercent = to_percent(self.terminal_time / self.settings._MAX_TERMINAL_DURATION)
    dm.alwaysOn = self.always_on
    dm.alwaysOnLockout = self.always_on and self.awareness <= self.threshold_alert_2
    dm.alertLevel = self.alert_level
    dm.activePolicy = self.active_policy
    dm.isRHD = self.wheel_on_right
    dm.rhdCalibration.calibratedPercent = to_percent(self.wheelpos_offsetter.filtered_stat.n / self.settings._WHEELPOS_FILTER_MIN_COUNT)
    dm.rhdCalibration.offset = self.wheelpos_offsetter.filtered_stat.M

    dm.visionPolicyState.awarenessPercent = to_percent(self.last_vision_awareness if self.active_policy != MonitoringPolicy.vision else self.awareness)
    dm.visionPolicyState.awarenessStep = self.step_change if self.active_policy == MonitoringPolicy.vision else 0.
    dm.visionPolicyState.isDistracted = self.driver_distracted
    dm.visionPolicyState.distractedTypes.pose = self.distracted_types['pose']
    dm.visionPolicyState.distractedTypes.eye = self.distracted_types['eye']
    dm.visionPolicyState.distractedTypes.phone = self.distracted_types['phone']
    dm.visionPolicyState.faceDetected = self.face_detected
    dm.visionPolicyState.pose.pitch = self.pose.pitch
    dm.visionPolicyState.pose.yaw = self.pose.yaw
    dm.visionPolicyState.pose.calibrated = self.pose.calibrated
    dm.visionPolicyState.pose.pitchCalib.calibratedPercent = to_percent(self.pose.pitch_offsetter.filtered_stat.n / self.settings._POSE_OFFSET_MIN_COUNT)
    dm.visionPolicyState.pose.pitchCalib.offset = self.pose.pitch_offsetter.filtered_stat.M
    dm.visionPolicyState.pose.yawCalib.calibratedPercent = to_percent(self.pose.yaw_offsetter.filtered_stat.n / self.settings._POSE_OFFSET_MIN_COUNT)
    dm.visionPolicyState.pose.yawCalib.offset = self.pose.yaw_offsetter.filtered_stat.M
    dm.visionPolicyState.pose.uncertainty = self.model_std_max
    dm.visionPolicyState.wheeltouchFallbackPercent = to_percent(self.hi_stds / self.settings._HI_STD_FALLBACK_TIME)
    dm.visionPolicyState.uncertainOffroadAlertPercent = to_percent(self.dcam_uncertain_cnt / self.settings._DCAM_UNCERTAIN_ALERT_COUNT)

    dm.wheeltouchPolicyState.awarenessPercent = to_percent(self.last_wheeltouch_awareness if self.active_policy == MonitoringPolicy.vision else self.awareness)
    dm.wheeltouchPolicyState.awarenessStep = 0. if self.active_policy == MonitoringPolicy.vision else self.step_change
    dm.wheeltouchPolicyState.driverInteracting = self.driver_interacting
    return dat

  def run_step(self, sm, demo=False):
    if demo:
      car_speed = 30
      enabled = True
      wrong_gear = False
      standstill = False
      driver_engaged = False
      brake_disengage_prob = 1.0
      steering_angle_deg = 0.0
      rpyCalib = [0., 0., 0.]
    else:
      car_speed = sm['carState'].vEgo
      enabled = sm['selfdriveState'].enabled
      wrong_gear = sm['carState'].gearShifter not in (car.CarState.GearShifter.drive, car.CarState.GearShifter.low)
      standstill = sm['carState'].standstill
      driver_engaged = sm['carState'].steeringPressed or sm['carState'].gasPressed
      brake_disengage_prob = sm['modelV2'].meta.disengagePredictions.brakeDisengageProbs[0] # brake disengage prob in next 2s
      steering_angle_deg = sm['carState'].steeringAngleDeg
      rpyCalib = sm['liveCalibration'].rpyCalib

    self._set_pose_strictness(
      brake_disengage_prob=brake_disengage_prob,
      car_speed=car_speed,
    )

    # Parse data from dmonitoringmodeld
    self._update_states(
      driver_state=sm['driverStateV2'],
      cal_rpy=rpyCalib,
      car_speed=car_speed,
      op_engaged=enabled,
      standstill=standstill,
      demo_mode=demo,
      steering_angle_deg=steering_angle_deg,
    )

    self._update_events(
      driver_engaged=driver_engaged,
      op_engaged=enabled,
      standstill=standstill,
      wrong_gear=wrong_gear,
      allow_look_sim=not demo,
    )
