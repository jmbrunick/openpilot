"""Rain-gate parser / radar-hold policy. No cereal — dry fusion stays in test_radard."""
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.rain_radar_hold import (
  ACQUIRE_ON,
  ACQUIRE_SENSITIVITY_SCALE,
  RAIN_CUT_IN_GAP_M,
  RAIN_WIPE_HOLD_S,
  RainRadarGate,
  closest_inlane_radar,
  evaluate_rain_follow_gate,
  pick_rain_radar_track,
  radar_hold_kinematics_ok,
  read_wiper_rain_status,
  read_wiper_speed,
  status_has_acq,
  status_has_rain,
  status_has_wipe,
  status_is_wet,
  status_rain_usable,
  status_score_at_acquire,
  wiper_is_auto,
)


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get(self, key, return_default=False):
    return self.values.get(key)


def nap_wiper_status(*, rain=1, wipe=1, acq=1, score=5.03, sens=2, err=False):
  """Same field names Auto Wipers writes to NAPWiperRainStatus."""
  rain_bits = (
    "rain=err" if err else
    f"hold=1 ema=4.83 score={score:.2f} bokeh=4.48 speckle=0.010 "
    f"sens={int(sens)} acq={int(acq)} rpt=1 heavy=0"
  )
  return (
    f"nap wiper auto setting=3 on=1 gear=drive gear_src=cs raw=4 drive=1 moving=1 "
    f"rain={int(rain)} wipe={int(wipe)} "
    f"sens={int(sens)} acq={int(acq)} rpt=1 park=0 v0=0 "
    f"collar=1 wash=0 installed=1 {rain_bits}"
  )


def track(identifier: int, d_rel: float, y_rel: float = 0.0):
  return SimpleNamespace(identifier=identifier, dRel=d_rel, yRel=y_rel)


def _gate(speed, status, now=10.0, last_wipe_t=None, last_wet_t=None):
  return evaluate_rain_follow_gate(speed, status, now, last_wipe_t, last_wet_t)


def test_status_fields_prefer_main_line_rain_over_err_bits():
  line = nap_wiper_status(rain=1, err=True)
  assert status_has_rain(line)
  assert status_rain_usable(line)
  assert status_is_wet(line)
  assert "rain=err" in line


def test_status_missing_and_err_only_are_not_usable():
  assert not status_rain_usable(None)
  assert not status_rain_usable("")
  assert not status_rain_usable("rain=err helper=0")
  assert status_rain_usable("nap wiper auto rain=0 wipe=0 acq=0")
  assert not status_has_rain("nap wiper auto rain=0 wipe=0 acq=0")
  assert not status_is_wet("nap wiper auto rain=0 wipe=0 acq=0")


def test_acq_and_score_strengthen_match_auto_wipers():
  assert status_has_acq(nap_wiper_status(rain=0, acq=1))
  assert not status_has_acq(nap_wiper_status(rain=0, acq=0, score=1.0))
  mid_acquire = ACQUIRE_ON * ACQUIRE_SENSITIVITY_SCALE[2]
  assert status_score_at_acquire(nap_wiper_status(rain=0, acq=0, score=mid_acquire, sens=2))
  assert not status_score_at_acquire(nap_wiper_status(rain=0, acq=0, score=mid_acquire - 0.1, sens=2))
  drier_acquire = ACQUIRE_ON * ACQUIRE_SENSITIVITY_SCALE[0]
  assert status_score_at_acquire(nap_wiper_status(rain=0, acq=0, score=drier_acquire, sens=0))


def test_rain_one_gates_regardless_of_wiper_speed():
  """Speed==3 is Auto stalk only. rain=1 is the wet signal at any setpoint."""
  line = nap_wiper_status(rain=1, acq=0, score=0.0)
  for speed in (0, 1, 2, 3):
    gate, _, _ = _gate(speed, line)
    assert gate, f"rain=1 must gate at Speed={speed}"


def test_speed_three_is_not_raining():
  """Auto selected + dry status is not a rain gate."""
  assert wiper_is_auto(3)
  dry = nap_wiper_status(rain=0, wipe=0, acq=0, score=0.2)
  gate, _, _ = _gate(3, dry)
  assert not gate
  params = FakeParams({
    "NAPWiperSpeed": 3,
    "NAPWiperRainStatus": dry,
  })
  assert not RainRadarGate(params=params).update()


def test_auto_plus_missing_status_is_not_raining():
  """Never Speed==3 ⇒ raining when the wet line is absent."""
  gate, wipe_t, wet_t = _gate(3, None, now=20.0)
  assert not gate
  assert wipe_t is None
  assert wet_t is None
  gate, _, _ = _gate(3, "collar=1 wash=0", now=20.0)
  assert not gate


def test_dry_status_does_not_use_wipe_or_collar():
  line = nap_wiper_status(rain=0, wipe=1, acq=0, score=0.5)
  assert status_has_wipe(line)
  assert "collar=1" in line
  gate, wipe_t, _ = _gate(3, line, now=10.0)
  assert not gate
  assert wipe_t == 10.0


def test_acq_or_score_can_strengthen_when_rain_zero():
  acq_line = nap_wiper_status(rain=0, wipe=0, acq=1, score=1.0)
  gate, _, _ = _gate(0, acq_line)
  assert gate

  score_line = nap_wiper_status(rain=0, wipe=0, acq=0, score=5.0, sens=2)
  gate, _, _ = _gate(2, score_line)
  assert gate


def test_missing_status_falls_back_to_recent_wipe_or_wet_score():
  gate, wipe_t, _ = _gate(0, None, now=20.0, last_wipe_t=None)
  assert not gate
  assert wipe_t is None

  gate, wipe_t, _ = _gate(3, None, now=20.0, last_wipe_t=20.0 - RAIN_WIPE_HOLD_S)
  assert gate
  assert wipe_t == 20.0 - RAIN_WIPE_HOLD_S

  gate, _, _ = _gate(3, None, now=20.0, last_wipe_t=20.0 - RAIN_WIPE_HOLD_S - 0.01)
  assert not gate

  gate, _, wet_t = _gate(0, None, now=20.0, last_wet_t=19.0)
  assert gate
  assert wet_t == 19.0


def test_err_only_status_uses_recent_wipe_or_embedded_wet_score():
  gate, _, _ = _gate(3, "rain=err helper=0", now=5.0, last_wipe_t=4.0)
  assert gate
  gate, _, _ = _gate(3, "rain=err helper=0", now=5.0, last_wipe_t=None)
  assert not gate
  gate, _, _ = _gate(0, "rain=err acq=1 score=5.03 sens=2", now=5.0)
  assert gate


def test_rain_gate_reads_live_status_not_speed_setpoint():
  params = FakeParams({
    "NAPWiperSpeed": 0,
    "NAPWiperRainStatus": nap_wiper_status(rain=1),
  })
  gate = RainRadarGate(params=params, now_fn=lambda: 1.0)
  assert gate.update()
  assert read_wiper_speed(params) == 0

  params.values["NAPWiperSpeed"] = 3
  params.values["NAPWiperRainStatus"] = nap_wiper_status(rain=0, wipe=0, acq=0, score=0.2)
  assert not gate.update()
  assert "rain=0" in (read_wiper_rain_status(params) or "")


def test_rain_gate_override_skips_params():
  gate = RainRadarGate(params=FakeParams({"NAPWiperSpeed": 0}))
  gate.set_override(True)
  assert gate.update()
  gate.set_override(False)
  assert not gate.update()


def test_pick_holds_incumbent_through_vision_mismatch():
  tracks = {806: track(806, 93.8), 900: track(900, 90.0, y_rel=3.2)}
  associated = None  # vision jumped −32 m; association rejected
  chosen = pick_rain_radar_track(associated, tracks, incumbent_id=806)
  assert chosen is tracks[806]


def test_pick_switches_to_much_closer_inlane_cut_in():
  tracks = {806: track(806, 93.8), 12: track(12, 93.8 - RAIN_CUT_IN_GAP_M)}
  chosen = pick_rain_radar_track(None, tracks, incumbent_id=806)
  assert chosen is tracks[12]


def test_pick_prefers_inlane_radar_over_unassociated_vision():
  tracks = {806: track(806, 93.8)}
  chosen = pick_rain_radar_track(None, tracks, incumbent_id=None)
  assert chosen is tracks[806]
  assert closest_inlane_radar(tracks) is tracks[806]


def test_pick_drops_incumbent_that_left_the_path():
  tracks = {806: track(806, 93.8, y_rel=6.0)}
  assert not radar_hold_kinematics_ok(tracks[806])
  chosen = pick_rain_radar_track(None, tracks, incumbent_id=806)
  assert chosen is None
