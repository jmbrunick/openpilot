"""Pre-AP Bosch radar SensorDirty: degrade, do not soft-disable.

I-40 gorge construction zone, Oct 4 2026 03:03:59 CT (route
1c95345a3286a5db|00000156--eb2236cd87): the radar set RADC_SensorDirty in its own
SguInfo frame (0x301 bit 44) while CAN was valid and leadOne radar=1 stayed live.
opendbc maps the flag to radarUnavailableTemporary -> radarTempUnavailable
(SOFT_DISABLE + NO_ENTRY): disengaged, 3 min 50 s of re-engage lockout.

Pinned here: NAPRadarIgnoreSensorDirty (default on) masks only that flag while live
measured tracks exist, tags radarPreferReason, keeps the radar lead, warns through the
existing radarPreferFallback event (never a disable), and escalates to the old
soft-disable only when the flag persists SENSOR_DIRTY_PERSIST_S with no live track.
Param off = old behavior. The radarstat line carries the radar status bits and a track
summary.
"""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from cereal import messaging
from openpilot.selfdrive.controls.lib import radar_sensor_dirty as sd
from openpilot.selfdrive.controls.lib import radar_status_log as rsl
from openpilot.selfdrive.controls.lib.radar_sensor_dirty import (
  SENSOR_DIRTY_LIVE_RECOVER_S,
  SENSOR_DIRTY_PERSIST_S,
  SENSOR_DIRTY_REASON,
  SensorDirtyPolicy,
  reason_token,
  sensor_dirty_degraded,
  sensor_dirty_ignore_enabled,
)
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA, RadarD
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, EventName

DT = 0.05
REPO = Path(__file__).resolve().parents[3]


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get_bool(self, key):
    return bool(self.values[key])


# ---------------------------------------------------------------- policy


def _run_policy(policy, seconds, flag=True, live=True, ignore=True):
  mask = None
  for _ in range(int(round(seconds / DT))):
    mask = policy.update(flag, live, DT, ignore=ignore)
  return mask


def test_constants_are_pinned():
  assert SENSOR_DIRTY_PERSIST_S == 10.0
  assert SENSOR_DIRTY_LIVE_RECOVER_S == 1.0
  assert SENSOR_DIRTY_REASON == "sensorDirty"
  assert sd.SENSOR_DIRTY_PARAM == "NAPRadarIgnoreSensorDirty"


def test_param_defaults_on_and_off_means_old_behavior():
  assert sensor_dirty_ignore_enabled(None) is True
  assert sensor_dirty_ignore_enabled(FakeParams({"NAPRadarIgnoreSensorDirty": True})) is True
  assert sensor_dirty_ignore_enabled(FakeParams({"NAPRadarIgnoreSensorDirty": False})) is False

  class Broken:
    def get_bool(self, key):
      raise KeyError(key)

  # An unreadable param is the new default, never a silent switch to the old soft-disable.
  assert sensor_dirty_ignore_enabled(Broken()) is True


def test_live_tracks_keep_the_flag_masked_indefinitely():
  policy = SensorDirtyPolicy()
  for _ in range(int(120 / DT)):
    assert policy.update(True, True, DT)
    assert policy.degraded and not policy.escalated
  assert policy.no_track_s == 0.0


def test_no_flag_never_masks_and_resets():
  policy = SensorDirtyPolicy()
  _run_policy(policy, 5.0, flag=True, live=False)
  assert policy.no_track_s > 0.0
  assert policy.update(False, False, DT) is False
  assert not policy.degraded and not policy.escalated and policy.no_track_s == 0.0


def test_flag_without_live_tracks_escalates_after_persist_window():
  policy = SensorDirtyPolicy()
  frames = int(round(SENSOR_DIRTY_PERSIST_S / DT))
  for i in range(frames - 1):
    assert policy.update(True, False, DT) is True, i
  assert not policy.escalated
  # The frame that reaches the persistence window hands the flag back unmasked.
  assert policy.update(True, False, DT) is False
  assert policy.escalated and not policy.degraded
  assert policy.update(True, False, DT) is False


def test_brief_track_blip_does_not_restart_the_no_track_clock():
  policy = SensorDirtyPolicy()
  _run_policy(policy, 6.0, live=False)
  before = policy.no_track_s
  _run_policy(policy, SENSOR_DIRTY_LIVE_RECOVER_S - 0.2, live=True)
  assert policy.no_track_s == pytest.approx(before)
  _run_policy(policy, 1.0, live=False)
  _run_policy(policy, SENSOR_DIRTY_LIVE_RECOVER_S - 0.5, live=True)
  assert policy.no_track_s == pytest.approx(before + 1.0)
  _run_policy(policy, 3.5, live=False)
  assert policy.escalated


def test_tracks_back_restart_the_clock_and_lift_the_escalation():
  policy = SensorDirtyPolicy()
  _run_policy(policy, SENSOR_DIRTY_PERSIST_S + 1.0, live=False)
  assert policy.escalated
  _run_policy(policy, SENSOR_DIRTY_LIVE_RECOVER_S + 0.2, live=True)
  assert policy.no_track_s == 0.0
  assert policy.degraded and not policy.escalated
  # And it needs a fresh full window to escalate again.
  _run_policy(policy, SENSOR_DIRTY_PERSIST_S - 1.0, live=False)
  assert policy.degraded and not policy.escalated


def test_ignore_off_never_masks_and_never_degrades():
  policy = SensorDirtyPolicy()
  assert _run_policy(policy, 30.0, flag=True, live=True, ignore=False) is False
  assert not policy.degraded and not policy.escalated


def test_reason_token_and_parse():
  assert reason_token("", True) == "sensorDirty"
  assert reason_token("dropout", True) == "sensorDirty+dropout"
  assert reason_token("dropout", False) == "dropout"
  assert reason_token("", False) == ""
  assert sensor_dirty_degraded("sensorDirty")
  assert sensor_dirty_degraded("sensorDirty+dropout")
  assert not sensor_dirty_degraded("dropout")
  assert not sensor_dirty_degraded("")
  # Substring is not a match, and junk never raises.
  assert not sensor_dirty_degraded("notsensorDirtyAtAll")
  assert not sensor_dirty_degraded(None)
  assert not sensor_dirty_degraded(object())


# ---------------------------------------------------------------- radard


class DirtyScenario:
  """radard fed by liveTracks messages that carry radarErrors."""

  def __init__(self, ignore: bool | None = True, v_ego: float = 17.6):
    services = ["modelV2", "carState", "liveTracks"]
    self.sm = messaging.SubMaster(services, ignore_alive=services, ignore_avg_freq=services)
    self.radar = RadarD()
    self.radar.rain_gate.set_enabled_override(True)
    self.radar.set_sensor_dirty_ignore_override(ignore)
    self.v_ego = v_ego
    self.t = 1.0
    self.frame = 0

  def step(self, dirty=False, points=((7, 25.0, 0.0, -1.0),), can_error=False, radar_fault=False, measured=True,
           send_tracks=True):
    self.t += DT
    model = messaging.new_message("modelV2")
    model.logMonoTime = int(self.t * 1e9)
    model.modelV2.velocity.x = [self.v_ego]
    leads = model.modelV2.init("leadsV3", 2)
    for lead in leads:
      lead.prob = 0.9
      lead.x = [25.0 + RADAR_TO_CAMERA]
      lead.xStd = [3.0]
      lead.y = [0.0]
      lead.yStd = [1.0]
      lead.v = [self.v_ego - 1.0]
      lead.vStd = [2.0]
      lead.a = [0.0]
    msgs = [model]
    if self.frame == 0:
      car_state = messaging.new_message("carState")
      car_state.logMonoTime = int(self.t * 1e9)
      car_state.carState.vEgo = self.v_ego
      msgs.append(car_state)
    live = messaging.new_message("liveTracks")
    if not send_tracks:
      self.sm.update_msgs(self.t, [m.as_reader() for m in msgs])
      self.radar.update(self.sm, self.sm["liveTracks"])
      self.frame += 1
      return self.radar.radar_state
    live.logMonoTime = int(self.t * 1e9)
    live.liveTracks.errors.radarUnavailableTemporary = dirty
    live.liveTracks.errors.canError = can_error
    live.liveTracks.errors.radarFault = radar_fault
    pts = live.liveTracks.init("points", len(points))
    for point, (track_id, d_rel, y_rel, v_rel) in zip(pts, points, strict=True):
      point.trackId, point.dRel, point.yRel, point.vRel, point.measured = track_id, d_rel, y_rel, v_rel, measured
    msgs.append(live)
    self.sm.update_msgs(self.t, [m.as_reader() for m in msgs])
    self.radar.update(self.sm, self.sm["liveTracks"])
    self.frame += 1
    return self.radar.radar_state

  def run(self, seconds, **kwargs):
    state = None
    for _ in range(int(round(seconds / DT))):
      state = self.step(**kwargs)
    return state


def test_radard_masks_sensor_dirty_while_tracks_are_live_and_keeps_the_radar_lead():
  sc = DirtyScenario(ignore=True)
  state = sc.run(30.0, dirty=True)
  assert not state.radarErrors.radarUnavailableTemporary
  assert not state.radarErrors.canError and not state.radarErrors.radarFault
  assert sensor_dirty_degraded(state.radarPreferReason)
  # Reliability saw the masked errors: the only reason is the token, no "fault" riding along.
  assert state.radarPreferReason == "sensorDirty"
  assert state.leadOne.status and state.leadOne.radar
  # selfdrived maps radarPreferFallback to a WARNING (events.py words it "Radar Sensor Dirty").
  assert state.radarPreferFallback


def test_radard_param_off_keeps_the_old_soft_disable_flag():
  sc = DirtyScenario(ignore=False)
  state = sc.run(5.0, dirty=True)
  assert state.radarErrors.radarUnavailableTemporary
  assert not sensor_dirty_degraded(state.radarPreferReason)
  # The old path: reliability trips on the raw flag ("fault"), exactly as before.
  assert state.radarPreferReason == "fault"


def test_radard_default_param_is_read_from_params_and_defaults_on(monkeypatch):
  sc = DirtyScenario(ignore=None)
  sc.radar.rain_gate._params = FakeParams({"NAPRadarIgnoreSensorDirty": True})
  state = sc.run(2.0, dirty=True)
  assert not state.radarErrors.radarUnavailableTemporary
  sc = DirtyScenario(ignore=None)
  sc.radar.rain_gate._params = FakeParams({"NAPRadarIgnoreSensorDirty": False})
  state = sc.run(2.0, dirty=True)
  assert state.radarErrors.radarUnavailableTemporary


def test_radard_escalates_to_the_old_soft_disable_when_flag_persists_without_tracks():
  sc = DirtyScenario(ignore=True)
  state = sc.run(SENSOR_DIRTY_PERSIST_S - 1.0, dirty=True, points=())
  assert not state.radarErrors.radarUnavailableTemporary
  assert sensor_dirty_degraded(state.radarPreferReason)
  state = sc.run(1.5, dirty=True, points=())
  assert state.radarErrors.radarUnavailableTemporary
  assert not sensor_dirty_degraded(state.radarPreferReason)


def test_radard_radar_going_silent_counts_as_no_live_tracks():
  sc = DirtyScenario(ignore=True)
  state = sc.run(3.0, dirty=True)
  assert not state.radarErrors.radarUnavailableTemporary
  # liveTracks stops (the last message, with the flag, stays in the SubMaster); 0.5 s timeout, then the window.
  state = sc.run(SENSOR_DIRTY_PERSIST_S + 2.0, dirty=True, send_tracks=False)
  assert state.radarErrors.radarUnavailableTemporary
  assert not sensor_dirty_degraded(state.radarPreferReason)


def test_radard_unmeasured_tracks_do_not_count_as_live():
  sc = DirtyScenario(ignore=True)
  state = sc.run(SENSOR_DIRTY_PERSIST_S + 1.0, dirty=True, measured=False)
  assert state.radarErrors.radarUnavailableTemporary


def test_radard_tracks_return_after_escalation_unlocks_reengage():
  sc = DirtyScenario(ignore=True)
  state = sc.run(SENSOR_DIRTY_PERSIST_S + 1.0, dirty=True, points=())
  assert state.radarErrors.radarUnavailableTemporary
  state = sc.run(SENSOR_DIRTY_LIVE_RECOVER_S + 0.5, dirty=True)
  assert not state.radarErrors.radarUnavailableTemporary
  assert sensor_dirty_degraded(state.radarPreferReason)


def test_radard_never_masks_other_radar_errors():
  sc = DirtyScenario(ignore=True)
  state = sc.run(2.0, dirty=True, can_error=True)
  assert state.radarErrors.canError
  sc = DirtyScenario(ignore=True)
  state = sc.run(2.0, dirty=True, radar_fault=True)
  assert state.radarErrors.radarFault
  assert not state.radarErrors.radarUnavailableTemporary


def test_radard_clean_radar_has_no_sensor_dirty_trace():
  sc = DirtyScenario(ignore=True)
  state = sc.run(5.0, dirty=False)
  assert not sensor_dirty_degraded(state.radarPreferReason)
  assert not state.radarErrors.radarUnavailableTemporary
  assert not state.radarPreferFallback
  assert state.leadOne.radar


def test_radard_flag_clearing_drops_the_degraded_tag():
  sc = DirtyScenario(ignore=True)
  assert sensor_dirty_degraded(sc.run(5.0, dirty=True).radarPreferReason)
  assert not sensor_dirty_degraded(sc.run(1.0, dirty=False).radarPreferReason)


# ---------------------------------------------------------------- alert / event wiring


class _Sm(dict):
  pass


def _alert(event_type, reason):
  sm = _Sm(radarState=SimpleNamespace(radarPreferReason=reason))
  return EVENTS[EventName.radarPreferFallback][event_type](None, None, sm, False, 0, None)


@pytest.mark.parametrize("event_type", [ET.PERMANENT, ET.WARNING])
def test_alert_text_switches_on_the_sensor_dirty_token(event_type):
  dirty = _alert(event_type, "sensorDirty")
  assert dirty.alert_text_1 == "Radar Sensor Dirty"
  assert "still active" in dirty.alert_text_2
  assert _alert(event_type, "sensorDirty+dropout").alert_text_1 == "Radar Sensor Dirty"
  old = _alert(event_type, "fault")
  assert old.alert_text_1 == "Radar Unreliable"
  assert old.alert_text_2 == "Using camera lead"
  assert _alert(event_type, "").alert_text_1 == "Radar Unreliable"


def test_degraded_alert_is_a_warning_not_a_disable_or_no_entry():
  events = EVENTS[EventName.radarPreferFallback]
  assert ET.SOFT_DISABLE not in events and ET.NO_ENTRY not in events
  assert ET.IMMEDIATE_DISABLE not in events and ET.USER_DISABLE not in events
  # The hard events are untouched.
  assert ET.SOFT_DISABLE in EVENTS[EventName.radarTempUnavailable]
  assert ET.NO_ENTRY in EVENTS[EventName.radarTempUnavailable]


def test_selfdrived_wiring_is_unchanged_and_degrade_rides_the_fallback_event():
  text = (REPO / "selfdrive/selfdrived/selfdrived.py").read_text()
  hard = "elif self.sm['radarState'].radarErrors.radarUnavailableTemporary:\n      self.events.add(EventName.radarTempUnavailable)"
  fallback = 'elif getattr(self.sm[\'radarState\'], "radarPreferFallback", False):'
  assert hard in text and fallback in text
  # canError, then the hard SensorDirty soft-disable, then any other radar error, then the warning-only fallback.
  assert text.index("radarErrors.canError:") < text.index(hard) < text.index("any(self.sm['radarState'].radarErrors.to_dict().values())") < text.index(fallback)


def test_param_key_defaults_on():
  keys = (REPO / "common/params_keys.h").read_text()
  assert '{"NAPRadarIgnoreSensorDirty", {PERSISTENT, BOOL, "1"}}' in keys
  assert '{"NAPRadarIgnoreHwFail", {PERSISTENT, BOOL}}' in keys


# ---------------------------------------------------------------- radarstat line


def _sgu(dirty=0, hw=0, sgu=1):
  v = (dirty << 44) | (hw << 45) | (sgu << 46)
  return v.to_bytes(8, "little")


def _pkt(addr, dat, src=1):
  return SimpleNamespace(address=addr, dat=dat, src=src)


def _pt(d, y, v, measured=True):
  return SimpleNamespace(dRel=d, yRel=y, vRel=v, measured=measured)


def test_status_log_bit_positions_match_the_ui_decoder():
  from openpilot.selfdrive.ui.radar import bosch_status as ui
  assert rsl.SGU_DIRTY_BIT == ui.SGU_DIRTY_BIT == 44
  assert rsl.SGU_HW_FAIL_BIT == ui.SGU_HW_FAIL_BIT == 45
  assert rsl.SGU_FAIL_BIT == ui.SGU_FAIL_BIT == 46
  assert rsl.ADDR_SGU == ui.ADDR_SGU == 0x301
  assert rsl.ADDR_ALERT == ui.ADDR_ALERT == 0x501
  assert rsl.RADAR_BUS == ui.RADAR_BUS == 1
  assert len(rsl.ALERT_NAMES) == 62
  for bit, name in ui.ALERT_BITS:
    assert rsl.ALERT_NAMES[bit].lower() == name.lower() or name.lower() in rsl.ALERT_NAMES[bit].lower()


def test_status_log_alert_names_cover_blinded_and_radome_heater():
  assert rsl.ALERT_NAMES[6] == "sensorBlinded"
  assert rsl.ALERT_NAMES[51] == "radomeHtrInop"
  assert rsl.alert_names((1 << 6) | (1 << 51) | (1 << 3)) == ["adjustmentNotDone", "sensorBlinded", "radomeHtrInop"]
  assert rsl.alert_names(0) == []


def test_status_log_decodes_the_oct4_sgu_frame():
  # Raw TeslaRadarSguInfo bytes at 03:05:25.033 CT: SensorDirty=1, HWFail=0, SGUFail=1.
  assert rsl.decode_sgu(bytes.fromhex("5824ffa2cd59")) == {"dirty": 1, "hw": 0, "sgu": 1}
  assert rsl.decode_sgu(_sgu(dirty=1, hw=0, sgu=1)) == {"dirty": 1, "hw": 0, "sgu": 1}
  assert rsl.decode_sgu(_sgu(dirty=0, hw=1, sgu=0)) == {"dirty": 0, "hw": 1, "sgu": 0}


def test_status_log_track_summary_counts_the_barrier_signature():
  v = 17.6
  pts = [
    _pt(25.0, 0.2, -1.0),          # moving lead
    _pt(30.0, 1.9, -v),            # stationary near the road edge
    _pt(41.0, -2.4, -v + 0.5),     # stationary near the road edge
    _pt(35.0, 7.5, -v),            # stationary, far lateral
    _pt(90.0, 1.0, -v),            # stationary, beyond the near-lateral range
    _pt(45.0, 0.0, -2.0, measured=False),
  ]
  s = rsl.track_summary(pts, v)
  assert (s["n"], s["nm"], s["ns"], s["nl"]) == (6, 5, 4, 2)
  assert s["ysm"] == 1.0
  assert s["d0"] == 25.0
  empty = rsl.track_summary([], v)
  assert empty == {"n": 0, "nm": 0, "ns": 0, "nl": 0, "ysm": None, "d0": None}


def test_status_log_emits_two_hz_with_bits_and_summary():
  log = rsl.RadarStatusLogger()
  alert = (1 << 6) | (1 << 51)
  packets = [_pkt(0x301, _sgu(dirty=1)), _pkt(0x501, alert.to_bytes(8, "little")), _pkt(0x301, b"\x00" * 8, src=0)]
  points = [_pt(25.0, 0.2, -1.0), _pt(30.0, 1.9, -17.6)]
  line = log.update(packets, points, 17.6, now=100.0)
  assert line.startswith("radarstat ")
  rec = json.loads(line.removeprefix("radarstat "))
  assert rec["dirty"] == 1 and rec["hw"] == 0 and rec["sgu"] == 1
  assert rec["al"] == f"{alert:016x}"
  assert rec["aln"] == ["sensorBlinded", "radomeHtrInop"]
  assert (rec["n"], rec["nm"], rec["ns"], rec["nl"]) == (2, 2, 1, 1)
  assert rec["sa"] == 0.0 and rec["aa"] == 0.0 and rec["v"] == 17.6
  # 2 Hz: nothing inside the period, one line after it; bits are remembered between frames.
  assert log.update([], points, 17.6, now=100.3) is None
  later = json.loads(log.update([], points, 17.6, now=100.6).removeprefix("radarstat "))
  assert later["dirty"] == 1 and later["aln"] == ["sensorBlinded", "radomeHtrInop"]
  assert later["sa"] == pytest.approx(0.6) and later["aa"] == pytest.approx(0.6)


def test_status_log_marks_unseen_frames_and_skips_cycles_without_radar_data():
  log = rsl.RadarStatusLogger()
  assert log.update([], None, 10.0, now=1.0) is None
  rec = json.loads(log.update([], [], 10.0, now=2.0).removeprefix("radarstat "))
  assert rec["sa"] is None and rec["al"] is None and rec["aa"] is None and rec["aln"] == []
  assert "dirty" not in rec


def test_status_log_ignores_non_radar_bus_and_never_raises():
  log = rsl.RadarStatusLogger()
  rec = json.loads(log.update([_pkt(0x301, _sgu(dirty=1), src=0), _pkt(0x501, b"\xff" * 8, src=2)], [], 5.0, now=1.0)
                   .removeprefix("radarstat "))
  assert "dirty" not in rec and rec["al"] is None
  try:
    out = log.update([object()], [], 5.0, now=9.0)
  except Exception as exc:
    raise AssertionError(f"status log must swallow its own errors: {exc!r}") from exc
  assert out is None


def test_card_wires_the_status_log_for_preap_only():
  text = (REPO / "selfdrive/car/card.py").read_text()
  assert "self._radar_stat = RadarStatusLogger() if tesla_preap else None" in text
  assert 'radar_stat.update(self._can_packets, getattr(RD, "points", None), CS.vEgo)' in text
  assert "cloudlog.error(stat_line)" in text
