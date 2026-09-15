#!/usr/bin/env python3
import os
import time
import threading

import cereal.messaging as messaging

from cereal import car, log

from opendbc.car.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog, ForwardingHandler

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData, CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.fw_versions import ObdCallback
from opendbc.car.car_helpers import get_car, interfaces
from opendbc.car.interfaces import CarInterfaceBase, RadarInterfaceBase
from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET, VCruiseHelper
from openpilot.selfdrive.mapd.map_speed_policy import (
  MapCruiseHold, apply_map_speed_kph, decide_map_cruise, effective_map_limit_ms,
  map_slew_a_ms2, read_map_speed_params, should_write_preap_pedal, slew_map_speed_ms,
)
from openpilot.selfdrive.controls.lib.curve_max_hold import CurveMaxHold
from openpilot.selfdrive.controls.lib.hypermile import (
  FollowStalkGesture, button_event_closer, button_event_released,
  map_target_offset_kph, persist_follow_distance, read_hypermile_params,
  read_hypermile_step_down,
)
from openpilot.selfdrive.car.tesla.preap_force_offroad_handoff import (
  install_force_offroad_handoff,
  update_force_offroad_handoff,
)

REPLAY = "REPLAY" in os.environ

EventName = log.OnroadEvent.EventName

# forward
carlog.addHandler(ForwardingHandler(cloudlog))


def obd_callback(params: Params) -> ObdCallback:
  def set_obd_multiplexing(obd_multiplexing: bool):
    if params.get_bool("ObdMultiplexingEnabled") != obd_multiplexing:
      cloudlog.warning(f"Setting OBD multiplexing to {obd_multiplexing}")
      params.remove("ObdMultiplexingChanged")
      params.put_bool("ObdMultiplexingEnabled", obd_multiplexing, block=True)
      params.get_bool("ObdMultiplexingChanged", block=True)
      cloudlog.warning("OBD multiplexing set successfully")
  return set_obd_multiplexing


def can_comm_callbacks(logcan: messaging.SubSocket, sendcan: messaging.PubSocket) -> tuple[CanRecvCallable, CanSendCallable]:
  def can_recv(wait_for_one: bool = False) -> list[list[CanData]]:
    """
    wait_for_one: wait the normal logcan socket timeout for a CAN packet, may return empty list if nothing comes

    Returns: CAN packets comprised of CanData objects for easy access
    """
    ret = []
    for can in messaging.drain_sock(logcan, wait_for_one=wait_for_one):
      ret.append([CanData(msg.address, msg.dat, msg.src) for msg in can.can])
    return ret

  def can_send(msgs: list[CanData]) -> None:
    sendcan.send(can_list_to_can_capnp(msgs, msgtype='sendcan'))

  return can_recv, can_send


class Car:
  CI: CarInterfaceBase
  RI: RadarInterfaceBase
  CP: car.CarParams

  def __init__(self, CI=None, RI=None) -> None:
    self.can_sock = messaging.sub_sock('can', timeout=20)
    self.sm = messaging.SubMaster(['pandaStates', 'carControl', 'onroadEvents', 'liveMapDataNAP', 'radarState'])
    self.pm = messaging.PubMaster(['sendcan', 'carState', 'carParams', 'carOutput', 'liveTracks'])

    self.can_rcv_cum_timeout_counter = 0

    self.CC_prev = car.CarControl.new_message()
    self.CS_prev = car.CarState.new_message()
    self.initialized_prev = False

    self.last_actuators_output = structs.CarControl.Actuators()

    self.params = Params()

    self.can_callbacks = can_comm_callbacks(self.can_sock, self.pm.sock['sendcan'])

    is_release = self.params.get_bool("IsReleaseBranch")

    if CI is None:
      # wait for one pandaState and one CAN packet
      print("Waiting for CAN messages...")
      while True:
        can = messaging.recv_one_retry(self.can_sock)
        if len(can.can) > 0:
          break

      alpha_long_allowed = self.params.get_bool("AlphaLongitudinalEnabled")

      cached_params = None
      cached_params_raw = self.params.get("CarParamsCache")
      if cached_params_raw is not None:
        with car.CarParams.from_bytes(cached_params_raw) as _cached_params:
          cached_params = _cached_params

      self.CI = get_car(*self.can_callbacks, obd_callback(self.params), alpha_long_allowed, is_release, cached_params)
      self.RI = interfaces[self.CI.CP.carFingerprint].RadarInterface(self.CI.CP)
      self.CP = self.CI.CP

      # continue onto next fingerprinting step in pandad
      self.params.put_bool("FirmwareQueryDone", True, block=True)
    else:
      self.CI, self.CP = CI, CI.CP
      self.RI = RI

    self.CP.alternativeExperience = 0
    openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")
    controller_available = self.CI.CC is not None and openpilot_enabled_toggle and not self.CP.dashcamOnly
    # Fingerprint before passive may overwrite safetyConfigs to noOutput.
    tesla_preap = str(getattr(self.CP, "carFingerprint", "") or "") == "TESLA_MODEL_S_PREAP"
    tesla_preap = tesla_preap or any(
      cfg.safetyModel == car.CarParams.SafetyModel.teslaPreap for cfg in self.CP.safetyConfigs
    )
    self._tesla_preap = tesla_preap
    self.CP.passive = not controller_available or self.CP.dashcamOnly
    if self.CP.passive:
      safety_config = structs.CarParams.SafetyConfig()
      safety_config.safetyModel = structs.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    if self.CP.secOcRequired:
      # Copy user key if available
      try:
        with open("/cache/params/SecOCKey") as f:
          user_key = f.readline().strip()
          if len(user_key) == 32:
            self.params.put("SecOCKey", user_key, block=True)
      except Exception:
        pass

      secoc_key = self.params.get("SecOCKey")
      if secoc_key is not None:
        saved_secoc_key = bytes.fromhex(secoc_key.strip())
        if len(saved_secoc_key) == 16:
          self.CP.secOcKeyAvailable = True
          self.CI.CS.secoc_key = saved_secoc_key
          if controller_available:
            self.CI.CC.secoc_key = saved_secoc_key
        else:
          cloudlog.warning("Saved SecOC key is invalid")

    # Write previous route's CarParams
    prev_cp = self.params.get("CarParamsPersistent")
    if prev_cp is not None:
      self.params.put("CarParamsPrevRoute", prev_cp, block=True)

    # Write CarParams for controls and radard
    cp_bytes = self.CP.to_bytes()
    self.params.put("CarParams", cp_bytes, block=True)
    self.params.put("CarParamsCache", cp_bytes)
    self.params.put("CarParamsPersistent", cp_bytes)

    self.v_cruise_helper = VCruiseHelper(self.CP)
    self._map_hold = MapCruiseHold()
    self._curve_max = CurveMaxHold()
    self._map_slew_ms: float | None = None
    self._last_pedal_kph: float | None = None
    self._follow_stalk_mono: float = 0.0
    self._follow_gesture = FollowStalkGesture()
    self._refresh_map_speed_params()

    self.is_metric = self.params.get_bool("IsMetric")
    self.experimental_mode = self.params.get_bool("ExperimentalMode")

    # card is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)

    self._can_packets: list[CanData] = []
    self.radar_donor_vin = None
    tesla_preap = getattr(self, "_tesla_preap", False)
    if tesla_preap:
      from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import install_blinker_lat_pause
      from openpilot.selfdrive.car.tesla.preap_body_controls import install_body_controls_test
      from opendbc.car.tesla.preap.nap_conf import nap_conf
      from opendbc.car.tesla.preap.radar_donor_vin import RadarDonorVinCommissioner

      install_blinker_lat_pause()
      install_body_controls_test()
      install_force_offroad_handoff()

      def store_donor_vin(vin: str) -> None:
        nap_conf.radar_donor_vin = vin
        self.params.put("NAPRadarVinReadStatus", f"saved {vin}")
        cloudlog.info("preap radar donor vin stored")

      self._nap_conf = nap_conf
      self.radar_donor_vin = RadarDonorVinCommissioner(store_donor_vin)

  def state_update(self) -> tuple[car.CarState, structs.RadarDataT | None]:
    """carState update loop, driven by can"""

    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    can_list = can_capnp_to_list(can_strs)
    self._can_packets = [CanData(addr, dat, src) for _ts, frames in can_list for addr, dat, src in frames]

    # Update carState from CAN
    CS = self.CI.update(can_list)

    # Pre-AP Force Offroad stock-CC handoff (or immediate ready if not
    # software-long). Must run while onroad so CANCEL/SET can still TX.
    update_force_offroad_handoff(
      getattr(self.CI, "CS", None) if getattr(self, "_tesla_preap", False) else None,
      CS,
    )

    # Update radar tracks from CAN
    RD: structs.RadarDataT | None = self.RI.update(can_list)

    self.sm.update(0)

    can_rcv_valid = len(can_strs) > 0

    # Check for CAN timeout
    if not can_rcv_valid:
      self.can_rcv_cum_timeout_counter += 1

    if can_rcv_valid and REPLAY:
      self.can_log_mono_time = messaging.log_from_bytes(can_strs[0]).logMonoTime

    try:
      preap_software_cruise = (
        self.CP.brand == "tesla"
        and self.CP.carFingerprint == "TESLA_MODEL_S_PREAP"
        and self.CP.openpilotLongitudinalControl
        and not self.CP.pcmCruise
      )

      if not preap_software_cruise:
        self.v_cruise_helper.update_v_cruise(CS, self.sm['carControl'].enabled, self.is_metric)
        if self.sm['carControl'].enabled and not self.CC_prev.enabled:
          # Use CarState w/ buttons from the step selfdrived enables on
          self.v_cruise_helper.initialize_v_cruise(self.CS_prev, self.experimental_mode)
      else:
        # Pre-AP pedal mode owns set-speed via pedal_speed_kph. Overlay OSM
        # onto vCruise/MAX for the OP session (cruiseEnabled), including a
        # brake long pause so sticky MAX can rebase. The first
        # pull is lateral-only and must not seed or arm a sticky hold.
        raw_kph = float(CS.cruiseState.speed * CV.MS_TO_KPH)
        raw_kph, follow_stalk = self._maybe_follow_stalk(CS, raw_kph)
        long_active = bool(getattr(CS, 'pedalLongActive', False))
        long_active_prev = bool(getattr(self.CS_prev, 'pedalLongActive', False))
        engage_rising = long_active and not long_active_prev
        session_enabled = bool(getattr(CS.cruiseState, 'enabled', False))
        resume_held, take_speed_now = self._preap_set_events()
        # Overlay held can be None (maps off never stalk-latched). The FSM
        # still has the brake-pause MAX — adopt it before pause substitution
        # so we never treat ego as held, and so session_engaged stays up.
        self._adopt_preap_fsm_held_max()
        # Software long (enableLongControl), not pedal authority. Pause and
        # the authority-acquisition gap both have pedalLongActive False;
        # only a true long pause publishes cruiseState.speed as ego.
        soft_long = bool(getattr(CS, 'enableLongControl', False))
        # Pause publishes cruiseState.speed as ego. Keep the held MAX instead.
        if not soft_long and self._map_hold.held_max_kph is not None:
          raw_kph = float(self._map_hold.held_max_kph)
        traveled_kph = float(CS.vEgo) * CV.MS_TO_KPH
        has_held = self._map_hold.held_max_kph is not None
        session_engaged = bool(session_enabled) and (
          soft_long or has_held or resume_held or take_speed_now
        )
        stalk_pressed = (not follow_stalk) and self._preap_stalk_set_pressed(CS)
        posted_kph = None
        map_kph = None
        map_valid = bool(self.sm.valid.get('liveMapDataNAP', False) and self.sm['liveMapDataNAP'].speedLimitValid)
        md = self.sm['liveMapDataNAP'] if map_valid else None
        raw_posted_kph = None
        if md is not None and md.speedLimit > 0:
          raw_posted_kph = float(md.speedLimit) * CV.MS_TO_KPH
        # Offset from raw OSM posted (mph). Hypermile eco / Step Down apply
        # only with a known map posted — never invent a drop when maps are
        # off, unmatched, or speedLimit unknown (same as sticky MAX).
        maps_posted = raw_posted_kph is not None
        map_offset_kph = self._live_map_offset_kph(raw_posted_kph, maps_posted=maps_posted)
        if raw_posted_kph is not None:
          posted_kph = raw_posted_kph + map_offset_kph
        # Snapshot pre-curve MAX before decide so OSM flicker cannot wipe sticky.
        last_hud_kph = float(self.v_cruise_helper.v_cruise_kph)
        steer_deg = float(getattr(CS, 'steeringAngleDeg', 0.0) or 0.0)
        policy_posted_kph, _ = self._curve_max.begin_cycle(
          self._map_hold,
          last_hud_kph=last_hud_kph,
          posted_kph=posted_kph,
          v_ego_ms=float(CS.vEgo),
          angle_steers_deg=steer_deg,
          steer_ratio=float(getattr(self.CP, 'steerRatio', 0.0) or 0.0),
          wheelbase=float(getattr(self.CP, 'wheelbase', 0.0) or 0.0),
          engaged=session_engaged,
          take_speed_now=take_speed_now,
          dt=DT_CTRL,
        )
        dec = decide_map_cruise(
          self._map_hold,
          engaged=session_engaged,
          mode=self._map_speed_mode,
          raw_kph=raw_kph,
          posted_kph=policy_posted_kph,
          engage_rising=engage_rising,
          now=time.monotonic(),
          stalk_pressed=stalk_pressed,
          take_speed_now=take_speed_now,
          resume_held=resume_held,
          traveled_kph=traveled_kph,
          long_active=soft_long,
        )
        # Stalk +/- while maps off updates overlay held; keep the FSM latch
        # aligned so the next brake pause resumes that MAX.
        self._publish_preap_held_max(long_active=soft_long)
        if map_valid and md is not None and not dec.sticky:
          # Any posted decrease: kin+110 m ease, never assign the new limit in one shot.
          lim = effective_map_limit_ms(
            float(md.speedLimit),
            float(md.nextSpeedLimit),
            float(md.nextSpeedLimitDistance),
            float(CS.vEgo),
            self._map_speed_lookahead,
            self._map_speed_accel,
            sticky=False,
          )
          if lim is not None and lim > 0:
            if self._map_slew_ms is None:
              prev_kph = float(self.v_cruise_helper.v_cruise_kph)
              prev_ms = prev_kph * CV.KPH_TO_MS
              if 0.0 < prev_kph < V_CRUISE_UNSET and prev_ms > lim + 0.3:
                self._map_slew_ms = prev_ms
              else:
                self._map_slew_ms = lim
            a = map_slew_a_ms2(
              self._map_slew_ms, lim, self._map_speed_lookahead, self._map_speed_accel,
            )
            self._map_slew_ms = slew_map_speed_ms(self._map_slew_ms, lim, DT_CTRL, a)
            map_kph = self._map_slew_ms * CV.MS_TO_KPH
          else:
            self._map_slew_ms = None
        elif not map_valid:
          self._map_slew_ms = None
        # seed_kph is a one-shot write (engage, posted raise, stalk step).
        # Sticky hold uses driver_kph / follow_override — do not write pedal
        # every frame or CI.update's stalk step is undone.
        if dec.seed_kph is not None:
          preap_v_cruise_kph = dec.seed_kph
          self._map_slew_ms = dec.seed_kph * CV.KPH_TO_MS
        else:
          preap_v_cruise_kph = apply_map_speed_kph(
            dec.driver_kph,
            map_kph,
            mode=self._map_speed_mode,
            offset_kph=map_offset_kph,
            engaged=session_engaged,
            op_long_software_cruise=True,
            driver_override=dec.follow_override,
          )
        # Temporary curve cap may lower HUD MAX. Restore seed puts pre-curve
        # MAX back after the bend. Do not let that cap rebase sticky / held.
        curve_out = self._curve_max.finish(
          hud_kph=preap_v_cruise_kph,
          hold=self._map_hold,
          posted_kph=posted_kph,
          v_ego_ms=float(CS.vEgo),
          angle_steers_deg=steer_deg,
          steer_ratio=float(getattr(self.CP, 'steerRatio', 0.0) or 0.0),
          wheelbase=float(getattr(self.CP, 'wheelbase', 0.0) or 0.0),
          engaged=session_engaged,
          stalk_pressed=stalk_pressed,
          take_speed_now=take_speed_now,
          dt=DT_CTRL,
        )
        preap_v_cruise_kph = float(curve_out.hud_kph)
        restore_seed_kph = curve_out.restore_seed_kph
        seed_kph = dec.seed_kph if restore_seed_kph is None else float(restore_seed_kph)
        if restore_seed_kph is not None:
          self._map_slew_ms = float(restore_seed_kph) * CV.KPH_TO_MS
        # Write engage/posted/stalk seed, or when HUD MAX rose. Never write
        # the same sticky MAX every frame. Pause still writes a rebase /
        # resume seed onto pedal_speed so one SET keeps the held MAX.
        # Curve restore is a one-shot seed so MAX returns to the pre-bend set.
        write_max = long_active or soft_long or resume_held or take_speed_now or (
          session_engaged and seed_kph is not None
        )
        if write_max and should_write_preap_pedal(seed_kph, preap_v_cruise_kph, self._last_pedal_kph):
          self._write_preap_pedal_speed(CS, preap_v_cruise_kph)
          self._last_pedal_kph = float(preap_v_cruise_kph)
        elif not session_enabled:
          self._last_pedal_kph = None
        self.v_cruise_helper.v_cruise_kph_last = self.v_cruise_helper.v_cruise_kph
        self.v_cruise_helper.v_cruise_kph = preap_v_cruise_kph
        self.v_cruise_helper.v_cruise_cluster_kph = preap_v_cruise_kph
    except Exception:
      # Fail-safe: never crash card due cruise-target selection logic.
      cloudlog.exception("Pre-AP software cruise target update failed, falling back to VCruiseHelper default")
      self.v_cruise_helper.update_v_cruise(CS, self.sm['carControl'].enabled, self.is_metric)
      if self.sm['carControl'].enabled and not self.CC_prev.enabled:
        self.v_cruise_helper.initialize_v_cruise(self.CS_prev, self.experimental_mode)

    # TODO: mirror the carState.cruiseState struct?
    CS.vCruise = float(self.v_cruise_helper.v_cruise_kph)
    CS.vCruiseCluster = float(self.v_cruise_helper.v_cruise_cluster_kph)

    return CS, RD

  def _preap_engagement(self):
    inner = getattr(self.CI, 'CS', None)
    return getattr(inner, 'engagement', None) if inner is not None else None

  def _preap_set_events(self) -> tuple[bool, bool]:
    """Consume one-shot SET flags from the Pre-AP engagement patch.

    resume_held: one SET after a brake long pause.
    take_speed_now: double SET / initial engage (forget sticky).
    """
    eng = self._preap_engagement()
    resume = bool(getattr(eng, '_nap_set_resume_long', False))
    take = bool(getattr(eng, '_nap_set_take_speed_now', False))
    if eng is not None:
      eng._nap_set_resume_long = False
      eng._nap_set_take_speed_now = False
    return resume, take

  def _adopt_preap_fsm_held_max(self) -> None:
    """If overlay never latched MAX, take the brake/turn FSM hold.

    Maps off does not invent posted. A stalk-set pedal MAX can exist on
    the FSM latch while MapCruiseHold.held_max_kph is still None. Adopting
    it before pause substitution prevents writing ego into held.
    """
    if self._map_hold.held_max_kph is not None:
      return
    eng = self._preap_engagement()
    held = getattr(eng, '_nap_held_max_kph', None) if eng is not None else None
    if held is not None and 0.0 < float(held) < V_CRUISE_UNSET:
      self._map_hold.held_max_kph = float(held)

  def _publish_preap_held_max(self, *, long_active: bool) -> None:
    """Keep the FSM latch in sync with overlay held while software long is on."""
    if not long_active or self._map_hold.held_max_kph is None:
      return
    eng = self._preap_engagement()
    if eng is not None:
      eng._nap_held_max_kph = float(self._map_hold.held_max_kph)

  def _preap_cruise_detent(self) -> int | None:
    """Raw SpdCtrlLvr_Stat / CruiseButtons. Distinguishes tip vs 2nd detent."""
    inner = getattr(self.CI, "CS", None)
    if inner is None:
      return None
    if hasattr(inner, "cruise_buttons"):
      try:
        return int(inner.cruise_buttons)
      except (TypeError, ValueError):
        pass
    msg = getattr(inner, "msg_stw_actn_req", None)
    if isinstance(msg, dict):
      try:
        return int(msg.get("SpdCtrlLvr_Stat") or 0)
      except (TypeError, ValueError):
        return None
    return None

  def _maybe_follow_stalk(self, CS, raw_kph: float) -> tuple[float, bool]:
    """Route Pre-AP stalk tip to stock Follow Distance 1–7 when a lead is present.

    Same whether Hypermile is On or Off. A first-detent tip undoes that
    frame's 1 mph MAX and writes NAPFollowDistance only after the lever
    returns to IDLE without a 2nd detent. A full press (2nd detent /
    5 mph) keeps MAX +5/−5 and does not remap Follow, even if the
    lever passed through first detent. No lead: leave stalk as MAX
    adjust. During a long pause, cruiseState.speed is ego — detent
    still distinguishes tip vs hold; buttonEvents alone do not.
    """
    has_lead = bool(
      self.sm.valid.get("radarState", False)
      and getattr(self.sm["radarState"].leadOne, "status", False)
    )
    soft_long = bool(getattr(CS, "enableLongControl", False))
    prev_raw = None
    try:
      prev_raw = float(self.CS_prev.cruiseState.speed) * CV.MS_TO_KPH
    except Exception:
      prev_raw = None
    events = getattr(CS, "buttonEvents", None)
    commit, closer, undo = self._follow_gesture.update(
      has_lead=has_lead,
      detent=self._preap_cruise_detent(),
      button_closer=button_event_closer(events),
      button_released=button_event_released(events),
      raw_kph=raw_kph if soft_long else None,
      prev_raw_kph=prev_raw if soft_long else None,
    )
    routed = commit or self._follow_gesture.is_pending or undo is not None
    if commit and closer is not None and (time.monotonic() - self._follow_stalk_mono) >= 0.25:
      persist_follow_distance(self.params, bool(closer))
      self._follow_stalk_mono = time.monotonic()
    if undo is not None and soft_long:
      self._write_preap_pedal_speed(CS, undo)
      self._last_pedal_kph = float(undo)
      return float(undo), True
    return raw_kph, routed

  @staticmethod
  def _preap_stalk_set_pressed(CS) -> bool:
    """True on stalk +/-. Extra signal; pedal_speed 1/5 mph steps also count."""
    for be in getattr(CS, 'buttonEvents', None) or []:
      try:
        typ = getattr(be, 'type', None)
        name = str(getattr(typ, 'name', typ)).lower().replace('_', '')
        if 'accelcruise' in name or 'decelcruise' in name or name in ('accel', 'decel'):
          return True
      except Exception:
        continue
    return False

  def _write_preap_pedal_speed(self, CS, kph: float) -> None:
    """Write HUD MAX into pre-AP pedal_speed so long / stalk +/- track that set."""
    kph = float(kph)
    try:
      CS.cruiseState.speed = kph * CV.KPH_TO_MS
    except Exception:
      pass
    inner = getattr(self.CI, 'CS', None)
    if inner is None:
      return
    if hasattr(inner, 'pedal_speed_kph'):
      inner.pedal_speed_kph = kph
    eng = getattr(inner, 'engagement', None)
    if eng is not None and hasattr(eng, 'pedal_speed_kph'):
      eng.pedal_speed_kph = kph
      eng._nap_held_max_kph = kph

  def state_publish(self, CS: car.CarState, RD: structs.RadarDataT | None):
    """carState and carParams publish loop"""

    # carParams - logged every 50 seconds (> 1 per segment)
    if self.sm.frame % int(50. / DT_CTRL) == 0:
      cp_send = messaging.new_message('carParams')
      cp_send.valid = True
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # publish new carOutput
    co_send = messaging.new_message('carOutput')
    co_send.valid = self.sm.all_checks(['carControl'])
    co_send.carOutput.actuatorsOutput = self.last_actuators_output
    self.pm.send('carOutput', co_send)

    # kick off controlsd step while we actuate the latest carControl packet
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.canErrorCounter = self.can_rcv_cum_timeout_counter
    cs_send.carState.cumLagMs = -self.rk.remaining * 1000.
    self.pm.send('carState', cs_send)

    if RD is not None:
      tracks_msg = messaging.new_message('liveTracks')
      tracks_msg.valid = not any(RD.errors.to_dict().values())
      tracks_msg.liveTracks = RD
      self.pm.send('liveTracks', tracks_msg)

  def controls_update(self, CS: car.CarState, CC: car.CarControl):
    """control update loop, driven by carControl"""

    if not self.initialized_prev:
      # Initialize CarInterface, once controls are ready
      # TODO: this can make us miss at least a few cycles when doing an ECU knockout
      self.CI.init(self.CP, *self.can_callbacks)
      # signal pandad to switch to car safety mode
      self.params.put_bool("ControlsReady", True)

    if self.sm.all_alive(['carControl']):
      # send car controls over can
      now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
      self.last_actuators_output, can_sends = self.CI.apply(CC, now_nanos)
      can_sends = list(can_sends)
      if self.radar_donor_vin is not None:
        controls_allowed = False
        if self.sm.valid['pandaStates']:
          controls_allowed = any(ps.controlsAllowed for ps in self.sm['pandaStates'])
        force_read = self.params.get_bool("NAPRadarReadVin")
        if force_read and not self._nap_conf.radar_enabled:
          self.params.put("NAPRadarVinReadStatus", "enable radar first")
          self.params.put_bool("NAPRadarReadVin", False)
        else:
          can_sends.extend(self.radar_donor_vin.update(
            self._can_packets,
            time.monotonic(),
            radar_enabled=self._nap_conf.radar_enabled,
            stored_vin=self._nap_conf.radar_donor_vin,
            controls_allowed=controls_allowed,
            enabled=bool(CC.enabled),
            force_read=force_read,
          ))
          if force_read and self.radar_donor_vin.read_finished:
            if not self.radar_donor_vin.reader.vin and self.radar_donor_vin.reader.failure:
              self.params.put(
                "NAPRadarVinReadStatus",
                self.radar_donor_vin.reader.failure.name.lower().replace("_", " "),
              )
            self.params.put_bool("NAPRadarReadVin", False)
      if getattr(self, "_tesla_preap", False):
        can_sends = self._collar_hold_sends(can_sends)
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))

      self.CC_prev = CC

  def _collar_hold_sends(self, can_sends):
    from openpilot.selfdrive.car.tesla.preap_body_controls import collar_hold_sends
    inner = getattr(self.CI, "CS", None)
    tesla_can = getattr(getattr(self.CI, "CC", None), "tesla_can", None)
    return collar_hold_sends(can_sends, inner, tesla_can, 0)

  def _publish_collar_hold(self):
    """Always last-mile 0x45. Do not wait for CI.apply / carControl / engage.

    NAP Int extra-forward is the same stock-cc overlay inside apply. Parked,
    apply is skipped or sendcan is gated on carControl alive — collar must
    still TX. Panda teslaPreap already allows 0x45 while disengaged.
    """
    from openpilot.selfdrive.car.tesla.preap_body_controls import write_collar_heartbeat
    try:
      sends = self._collar_hold_sends([])
      if sends:
        self.pm.send('sendcan', can_list_to_can_capnp(sends, msgtype='sendcan', valid=True))
    except Exception as e:
      write_collar_heartbeat(f"exception:{type(e).__name__}")

  def step(self):
    CS, RD = self.state_update()

    self.state_publish(CS, RD)

    initialized = (not any(e.name == EventName.selfdriveInitializing for e in self.sm['onroadEvents']) and
                   self.sm.seen['onroadEvents'])
    if not self.CP.passive and initialized:
      self.controls_update(CS, self.sm['carControl'])
    if getattr(self, "_tesla_preap", False):
      # Not elif: initialized+apply can still skip sendcan (carControl dead).
      # Collar3 uses this same 0x45 path every tick like Int extra-forward.
      self._publish_collar_hold()

    self.initialized_prev = initialized
    self.CS_prev = CS

  def _refresh_map_speed_params(self):
    mode, offset, lookahead, accel = read_map_speed_params(self.params)
    hm_on = read_hypermile_params(self.params)
    self._map_speed_mode = mode
    # User / settings offset only. Hypermile eco is computed live from posted.
    self._map_speed_user_offset_kph = offset
    self._map_hypermile_on = hm_on
    self._map_step_down_on = read_hypermile_step_down(self.params)
    self._map_speed_lookahead = lookahead
    self._map_speed_accel = accel

  def _live_map_offset_kph(self, raw_posted_kph: float | None,
                           maps_posted: bool | None = None) -> float:
    return map_target_offset_kph(
      self._map_speed_user_offset_kph,
      hypermile_on=self._map_hypermile_on,
      step_down_on=self._map_step_down_on,
      posted_kph=raw_posted_kph,
      maps_posted=maps_posted,
    )

  def params_thread(self, evt):
    while not evt.is_set():
      self.is_metric = self.params.get_bool("IsMetric")
      self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl
      self._refresh_map_speed_params()
      time.sleep(0.1)

  def card_thread(self):
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e, ))
    try:
      t.start()
      while True:
        self.step()
        self.rk.monitor_time()
    finally:
      e.set()
      t.join()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  car = Car()
  car.card_thread()


if __name__ == "__main__":
  main()
