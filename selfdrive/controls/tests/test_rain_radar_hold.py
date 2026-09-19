"""Rain-sensing mode gate / radar-hold policy. No cereal — dry fusion stays in test_radard."""
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.rain_radar_hold import (
  RAIN_CUT_IN_GAP_M,
  RainRadarGate,
  closest_inlane_radar,
  pick_rain_radar_track,
  radar_hold_kinematics_ok,
  rain_sensing_on,
  read_wiper_speed,
  wiper_is_auto,
)


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get(self, key, return_default=False):
    return self.values.get(key)


def nap_wiper_status(*, rain=1, wipe=1, acq=1, score=5.03):
  """Status line Auto Wipers writes. Mode gate must not require these fields."""
  return (
    f"nap wiper auto setting=3 rain={int(rain)} wipe={int(wipe)} "
    f"acq={int(acq)} score={score:.2f} collar=1"
  )


def track(identifier: int, d_rel: float, y_rel: float = 0.0, v_rel: float = 0.0):
  return SimpleNamespace(identifier=identifier, dRel=d_rel, yRel=y_rel, vRel=v_rel)


def test_auto_is_rain_sensing_mode_not_weather():
  """Speed==3 means Auto / rain-sensing On. It is not proof that it is raining."""
  assert wiper_is_auto(3)
  assert rain_sensing_on(3)
  for speed in (0, 1, 2):
    assert not wiper_is_auto(speed)
    assert not rain_sensing_on(speed)


def test_auto_on_radar_prefer_without_rain_one():
  """Do not require rain=1 / acq= / score. Auto On is enough."""
  params = FakeParams({
    "NAPWiperSpeed": 3,
    "NAPWiperRainStatus": nap_wiper_status(rain=0, wipe=0, acq=0, score=0.2),
  })
  assert RainRadarGate(params=params).update()


def test_auto_on_with_missing_status_still_radar_prefer():
  params = FakeParams({"NAPWiperSpeed": 3})
  assert RainRadarGate(params=params).update()


def test_manual_wiper_modes_keep_dry_fusion_even_if_status_says_rain():
  """Off / Int / On are not rain-sensing. Wet status must not flip the gate."""
  wet = nap_wiper_status(rain=1, wipe=1, acq=1, score=9.0)
  for speed in (0, 1, 2):
    params = FakeParams({"NAPWiperSpeed": speed, "NAPWiperRainStatus": wet})
    assert not RainRadarGate(params=params).update()


def test_auto_not_inferred_from_wipe_or_collar():
  params = FakeParams({
    "NAPWiperSpeed": 1,
    "NAPWiperRainStatus": nap_wiper_status(rain=1, wipe=1) + " collar=1",
  })
  assert not RainRadarGate(params=params).update()
  assert read_wiper_speed(params) == 1


def test_rain_gate_reads_live_speed_param():
  params = FakeParams({"NAPWiperSpeed": 3})
  gate = RainRadarGate(params=params)
  assert gate.update()
  params.values["NAPWiperSpeed"] = 0
  assert not gate.update()


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


def test_pick_drops_left_roadside_sign_old_yrel_window():
  """Old rain in-lane was 2.5 m / incumbent 4.0 m — a left sign at 3.2 m held."""
  tracks = {44: track(44, 48.0, y_rel=3.2, v_rel=-20.0)}
  assert not radar_hold_kinematics_ok(tracks[44], v_ego=20.0)
  chosen = pick_rain_radar_track(None, tracks, incumbent_id=44, v_ego=20.0)
  assert chosen is None
  chosen = pick_rain_radar_track(tracks[44], tracks, incumbent_id=None, v_ego=20.0)
  assert chosen is None


def test_pick_rejects_oncoming_opposing_lane():
  v_ego = 20.0
  tracks = {9: track(9, 55.0, y_rel=-1.6, v_rel=-(v_ego + 18.0))}
  assert not radar_hold_kinematics_ok(tracks[9], v_ego=v_ego)
  chosen = pick_rain_radar_track(None, tracks, incumbent_id=9, v_ego=v_ego)
  assert chosen is None
  chosen = pick_rain_radar_track(tracks[9], tracks, incumbent_id=None, v_ego=v_ego)
  assert chosen is None


def test_pick_still_holds_in_path_radar_through_mismatch():
  tracks = {806: track(806, 93.8, y_rel=0.4, v_rel=-1.0)}
  chosen = pick_rain_radar_track(None, tracks, incumbent_id=806, v_ego=20.0)
  assert chosen is tracks[806]


def test_pick_does_not_let_rain_override_path_or_oncoming_rejects():
  v_ego = 20.0
  sign = track(3, 40.0, y_rel=3.5, v_rel=-v_ego)
  oncoming = track(4, 42.0, y_rel=-1.4, v_rel=-(v_ego + 16.0))
  good = track(5, 45.0, y_rel=0.2, v_rel=-0.5)
  assert pick_rain_radar_track(sign, {3: sign}, None, v_ego) is None
  assert pick_rain_radar_track(oncoming, {4: oncoming}, None, v_ego) is None
  assert pick_rain_radar_track(None, {5: good}, None, v_ego) is good
