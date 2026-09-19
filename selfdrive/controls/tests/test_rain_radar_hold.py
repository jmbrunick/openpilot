"""Rain-sensing mode gate / radar-hold policy. No cereal — dry fusion stays in test_radard."""
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.radar_path_gate import (
  CURVE_OUTSIDE_PATH_X,
  CURVE_OUTSIDE_PATH_Y,
  CURVE_OUTSIDE_TWO_LANES_M,
  CURVE_OUTSIDE_X_M,
  ATLANTIC_LEFT_PATH_X,
  ATLANTIC_LEFT_PATH_Y,
  LEFT_TURN_PATH_X,
  LEFT_TURN_PATH_Y,
  LEFT_TURN_X_M,
  ONCOMING_VLEAD_MS,
  RADAR_TO_CAMERA_M,
  ROUNDABOUT_EXIT_PATH_X,
  ROUNDABOUT_EXIT_PATH_Y,
  ROUNDABOUT_EXIT_X_M,
  path_y_at_x,
)
from openpilot.selfdrive.controls.lib.rain_radar_hold import (
  RAIN_CUT_IN_GAP_M,
  RAIN_FAR_HOLD_DREL_M,
  RAIN_INLANE_YREL_M,
  RainRadarGate,
  closest_inlane_radar,
  pick_rain_radar_track,
  radar_hold_kinematics_ok,
  rain_far_hold_ok,
  rain_stat_hold_ok,
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


def test_pick_does_not_fov_cut_in_without_association():
  """Closer in-lane radar without a path association must not steal the hold."""
  tracks = {806: track(806, 93.8), 12: track(12, 93.8 - RAIN_CUT_IN_GAP_M)}
  chosen = pick_rain_radar_track(None, tracks, incumbent_id=806)
  assert chosen is tracks[806]


def test_pick_switches_to_associated_path_cut_in():
  tracks = {806: track(806, 93.8), 12: track(12, 93.8 - RAIN_CUT_IN_GAP_M)}
  chosen = pick_rain_radar_track(tracks[12], tracks, incumbent_id=806)
  assert chosen is tracks[12]


def test_pick_does_not_acquire_unassociated_inlane_radar():
  """Rain-hold is not a wide-FOV prefer. No association → no new latch."""
  tracks = {806: track(806, 93.8)}
  chosen = pick_rain_radar_track(None, tracks, incumbent_id=None)
  assert chosen is None
  assert closest_inlane_radar(tracks) is tracks[806]


def test_pick_drops_incumbent_that_left_the_path():
  tracks = {806: track(806, 93.8, y_rel=6.0)}
  assert not radar_hold_kinematics_ok(tracks[806])
  chosen = pick_rain_radar_track(None, tracks, incumbent_id=806)
  assert chosen is None


def test_e1_hold_drops_incumbent_at_yrel_2_8_stationary():
  """Dig: drop incumbent at yRel=2.8 STAT (old 4.0 window held it)."""
  v_ego = 16.0  # ~36 mph, ep B band
  t = track(318, 40.8, y_rel=2.8, v_rel=-v_ego)
  assert not radar_hold_kinematics_ok(t, v_ego=v_ego)
  assert pick_rain_radar_track(None, {318: t}, 318, v_ego) is None


def test_e1_does_not_pick_opposing_vlead_minus_8():
  """Dig: must not pick opposing vLead=−8."""
  v_ego = 16.0
  t = track(101, 50.0, y_rel=1.2, v_rel=-(v_ego + 8.0))  # vLead = −8
  assert t.vRel + v_ego == -8.0
  assert not radar_hold_kinematics_ok(t, v_ego=v_ego)
  assert pick_rain_radar_track(t, {101: t}, None, v_ego) is None
  assert pick_rain_radar_track(None, {101: t}, 101, v_ego) is None


def test_e1_ep_b_does_not_acquire_low_prob_stationary_at_yrel_0_86():
  """20:49:10 track 321: dRel=99.3, yRel=+0.86, vLead≈0, mp=0.007."""
  v_ego = 16.05
  t = track(321, 99.3, y_rel=0.86, v_rel=-v_ego)
  assert pick_rain_radar_track(None, {321: t}, None, v_ego) is None


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
  assert pick_rain_radar_track(None, {5: good}, None, v_ego) is None
  assert pick_rain_radar_track(good, {5: good}, None, v_ego) is good


def test_pick_does_not_latch_roundabout_entrant_off_exit_path():
  """~20:54 CT: rain must not hold the entering vehicle on a right-exit path."""
  v_ego = 12.0
  entrant = track(22, ROUNDABOUT_EXIT_X_M - RADAR_TO_CAMERA_M, y_rel=0.0, v_rel=-2.0)
  path = (ROUNDABOUT_EXIT_PATH_X, ROUNDABOUT_EXIT_PATH_Y)
  assert not radar_hold_kinematics_ok(entrant, v_ego=v_ego, path_x=path[0], path_y=path[1])
  assert pick_rain_radar_track(entrant, {22: entrant}, None, v_ego, path[0], path[1]) is None
  assert pick_rain_radar_track(None, {22: entrant}, 22, v_ego, path[0], path[1]) is None


def test_tip_still_has_path_gate_2_0_yrel_and_oncoming_reject():
  """Justin flash check: path gate + 2.5→2.0 yRel + vLead < 0 reject."""
  assert RAIN_INLANE_YREL_M == 2.0
  assert ONCOMING_VLEAD_MS < 0.0


def test_ep2059_inlane_gate_is_2_0_not_2_5():
  """Justin: RAIN_INLANE_YREL_M 2.5 → 2.0. Rejects 2.23 and 2.48 latches."""
  assert RAIN_INLANE_YREL_M == 2.0
  v_ego = 20.0
  assert not radar_hold_kinematics_ok(track(771, 45.0, 2.23, -(v_ego + 22.0)),
                                      RAIN_INLANE_YREL_M, v_ego)
  assert not radar_hold_kinematics_ok(track(802, 40.0, 2.48, -(v_ego + 18.7)),
                                      RAIN_INLANE_YREL_M, v_ego)


def test_ep2059_near_edge_oncoming_not_latched():
  """20:59:33 track 771 / 21:00:12 track 802 — inside old 2.5 m, oncoming."""
  v_ego = 20.0  # ~45 mph arterial
  # 771: yRel +2.23→+3.98, vLead −22 m/s (−49 mph)
  t771 = track(771, 45.0, y_rel=2.23, v_rel=-(v_ego + 22.0))
  t771_walk = track(771, 20.0, y_rel=3.98, v_rel=-(v_ego + 22.0))
  # 802: yRel +2.23→+2.48 (0.02 m INSIDE old 2.5), vLead −18.7 m/s
  t802 = track(802, 40.0, y_rel=2.23, v_rel=-(v_ego + 18.7))
  t802_edge = track(802, 28.0, y_rel=2.48, v_rel=-(v_ego + 18.7))
  for t in (t771, t771_walk, t802, t802_edge):
    assert not radar_hold_kinematics_ok(t, v_ego=v_ego)
    assert pick_rain_radar_track(t, {t.identifier: t}, None, v_ego) is None
    assert pick_rain_radar_track(None, {t.identifier: t}, t.identifier, v_ego) is None


def test_straight_opposing_passers_stay_ignored():
  """~20:58 CT: several opposing-lane cars pass toward ego — no regen.

  They never associate as leadOne. Path-gating rain-hold must not FOV-pick
  them or invent a new oncoming-brake. Stay ignored, same as stock.
  """
  v_ego = 20.0
  passers = [
    track(1, 80.0, y_rel=3.1, v_rel=-(v_ego + 22.0)),
    track(2, 50.0, y_rel=2.9, v_rel=-(v_ego + 18.0)),
    track(3, 28.0, y_rel=3.4, v_rel=-(v_ego + 20.0)),
  ]
  tracks = {t.identifier: t for t in passers}
  assert pick_rain_radar_track(None, tracks, None, v_ego) is None
  for t in passers:
    assert pick_rain_radar_track(None, tracks, t.identifier, v_ego) is None
    assert pick_rain_radar_track(t, tracks, None, v_ego) is None


def test_straight_opposing_does_not_steal_in_path_lead():
  """20:58-class passers must not displace a real path-associated lead."""
  v_ego = 20.0
  lead = track(11, 40.0, y_rel=0.3, v_rel=-0.4)
  passer = track(2, 50.0, y_rel=2.9, v_rel=-(v_ego + 18.0))
  tracks = {11: lead, 2: passer}
  assert pick_rain_radar_track(lead, tracks, 11, v_ego) is lead
  assert pick_rain_radar_track(None, tracks, 11, v_ego) is lead


def test_ep2059_mid_lane_opposing_never_acquired():
  """21:02 / 21:03: opposing outside 2.5 m — leadOne stayed None. Keep that."""
  v_ego = 20.0
  mid = track(900, 50.0, y_rel=3.2, v_rel=-(v_ego + 20.0))
  assert pick_rain_radar_track(None, {900: mid}, None, v_ego) is None
  assert pick_rain_radar_track(None, {900: mid}, 900, v_ego) is None
  assert pick_rain_radar_track(mid, {900: mid}, None, v_ego) is None


def test_2102_farther_over_semi_stays_clean():
  """Justin 21:02: 4–5 opposing including a semi farther over — no regen.

  Near-edge 2.23 m (20:59/21:00) is outside the 2.0 m perimeter.
  Farther-over ~3.0–3.5 m must stay ignored; do not nuke all opposing.
  """
  v_ego = 20.0
  near_edge = track(802, 40.0, y_rel=2.23, v_rel=-(v_ego + 18.8))
  pack = [
    track(1, 70.0, y_rel=3.0, v_rel=-(v_ego + 20.0)),
    track(2, 55.0, y_rel=3.2, v_rel=-(v_ego + 18.0)),
    track(3, 40.0, y_rel=3.5, v_rel=-(v_ego + 22.0)),  # semi farther over
    track(4, 28.0, y_rel=2.9, v_rel=-(v_ego + 19.0)),
    track(5, 20.0, y_rel=3.3, v_rel=-(v_ego + 21.0)),
  ]
  tracks = {t.identifier: t for t in pack}
  assert not radar_hold_kinematics_ok(near_edge, v_ego=v_ego)
  assert pick_rain_radar_track(None, tracks, None, v_ego) is None
  for t in pack:
    assert pick_rain_radar_track(None, tracks, t.identifier, v_ego) is None
    assert pick_rain_radar_track(t, {t.identifier: t}, None, v_ego) is None
  lead = track(11, 38.0, y_rel=0.3, v_rel=-0.4)
  tracks[11] = lead
  assert pick_rain_radar_track(lead, tracks, 11, v_ego) is lead


def test_2103_mid_lane_oncoming_truck_stays_clean():
  """~21:03 CT: oncoming truck in the middle of the opposing lane — no regen.

  Pass with 21:02. Fail cases stay near-edge / off-path (20:59–21:00).
  """
  v_ego = 20.0
  truck = track(910, 45.0, y_rel=3.7, v_rel=-(v_ego + 22.0))
  assert not radar_hold_kinematics_ok(truck, v_ego=v_ego)
  assert pick_rain_radar_track(None, {910: truck}, None, v_ego) is None
  assert pick_rain_radar_track(None, {910: truck}, 910, v_ego) is None
  assert pick_rain_radar_track(truck, {910: truck}, None, v_ego) is None
  lead = track(11, 40.0, y_rel=0.2, v_rel=-0.4)
  assert pick_rain_radar_track(lead, {11: lead, 910: truck}, 11, v_ego) is lead


def test_ep2059_far_stat_phantom_not_acquired():
  """20:59:40 Dip 2: tids 782/784/786, vLead≈0, dRel 84–123 m, mp≤0.014."""
  v_ego = 20.0
  for tid, d_rel, y_rel in ((782, 123.0, 0.86), (784, 100.0, 1.4), (786, 84.0, 2.11)):
    far = track(tid, d_rel, y_rel=y_rel, v_rel=-v_ego)
    assert pick_rain_radar_track(None, {tid: far}, None, v_ego, vision_prob=0.01) is None
    assert pick_rain_radar_track(None, {tid: far}, tid, v_ego, vision_prob=0.01) is None


def test_far_low_prob_incumbent_not_held_for_regen():
  """20:59 second slam / 20:49:10 track 321: far + mp≪0.15 must not stay leadOne."""
  v_ego = 16.0
  phantom = track(321, 99.3, y_rel=0.86, v_rel=-v_ego)
  assert phantom.dRel > RAIN_FAR_HOLD_DREL_M
  assert not rain_far_hold_ok(phantom, None, vision_prob=0.007)
  assert pick_rain_radar_track(None, {321: phantom}, 321, v_ego, vision_prob=0.007) is None


def test_ep2106_unassociated_stat_low_mp_not_latched():
  """EP_2106: 872/905/925/940 — STAT, mp≪0.15, rain-hold. Semi never leadOne."""
  v_ego = 20.0
  cases = (
    (872, 109.0, 1.98),   # LEFT sign; oncoming semi was NOT leadOne
    (905, 107.0, 1.23),   # "ahead" then sweeps; yRel inside 2.0
    (906, 100.0, 2.11),
    (907, 117.0, 2.48),
    (913, 117.0, 1.73),
    (925, 112.0, -0.15),  # Atlantic gas-station
    (940, 151.0, 0.73),   # near stop; yRel inside 2.0
    (943, 82.0, 2.11),
  )
  for tid, d_rel, y_rel in cases:
    t = track(tid, d_rel, y_rel=y_rel, v_rel=-v_ego)
    assert not rain_stat_hold_ok(t, None, vision_prob=0.01, v_ego=v_ego)
    assert pick_rain_radar_track(None, {tid: t}, tid, v_ego, vision_prob=0.01) is None
    assert pick_rain_radar_track(None, {tid: t}, None, v_ego, vision_prob=0.01) is None


def test_on_path_stationary_associated_still_held():
  """Do not blanket-reject vLead≈0. Stopped in-path lead stays readable."""
  v_ego = 12.0
  stopped = track(11, 28.0, y_rel=0.2, v_rel=-v_ego)
  assert rain_stat_hold_ok(stopped, stopped, vision_prob=0.80, v_ego=v_ego)
  assert pick_rain_radar_track(stopped, {11: stopped}, None, v_ego,
                               vision_prob=0.80) is stopped


def test_far_associated_rain_lead_still_held():
  """Original #201: 93.8 m path lead + confident vision flap still holds."""
  t = track(806, 93.8, y_rel=0.0, v_rel=0.0)
  assert rain_far_hold_ok(t, t, vision_prob=0.978)
  assert pick_rain_radar_track(None, {806: t}, 806, vision_prob=0.978) is t
  # Live association at 100 m (on-path STAT / distant car) is not "absurd".
  far = track(11, 100.0, y_rel=0.2, v_rel=-0.3)
  assert pick_rain_radar_track(far, {11: far}, None, vision_prob=0.70) is far


def test_oncoming_rejected_even_when_yrel_is_inside_gate():
  """Size / near-center radar return must not override vLead oncoming reject."""
  v_ego = 20.0
  semi = track(802, 35.0, y_rel=1.0, v_rel=-(v_ego + 18.8))
  assert not radar_hold_kinematics_ok(semi, v_ego=v_ego)
  assert pick_rain_radar_track(semi, {802: semi}, None, v_ego) is None


def test_2100_close_semi_not_kept_as_rain_incumbent():
  """~21:00 CT: large/close opposing semi. Rain must drop a live incumbent."""
  v_ego = 20.0
  close = track(802, 28.0, y_rel=2.23, v_rel=-(v_ego + 18.8))
  tighter = track(803, 25.0, y_rel=1.0, v_rel=-(v_ego + 18.8))
  for t in (close, tighter):
    assert pick_rain_radar_track(None, {t.identifier: t}, t.identifier, v_ego) is None
    assert pick_rain_radar_track(t, {t.identifier: t}, t.identifier, v_ego) is None


def test_associated_lead_at_1_7m_still_held():
  """Don't over-tighten path-valid associations (1.5 would have dropped this)."""
  v_ego = 20.0
  t = track(11, 40.0, y_rel=1.7, v_rel=-0.5)
  assert radar_hold_kinematics_ok(t, v_ego=v_ego)
  assert pick_rain_radar_track(t, {11: t}, None, v_ego) is t


def test_2108_gas_station_not_held_while_closing():
  """21:08: rain must not keep ego-forward gas-station furniture as dRel closes."""
  v_ego = 12.5
  path = (ATLANTIC_LEFT_PATH_X, ATLANTIC_LEFT_PATH_Y)
  for tid, d_rel, y_rel in ((925, 40.0, 0.0), (932, 22.0, 0.15), (940, 9.0, -0.2)):
    t = track(tid, d_rel, y_rel=y_rel, v_rel=-v_ego)
    assert not radar_hold_kinematics_ok(t, v_ego=v_ego, path_x=path[0], path_y=path[1])
    assert pick_rain_radar_track(None, {tid: t}, tid, v_ego, path[0], path[1]) is None
    assert pick_rain_radar_track(t, {tid: t}, None, v_ego, path[0], path[1]) is None


def test_left_turn_rain_does_not_latch_ego_forward_or_oncoming():
  """21:06–21:07: rain must not hold yRel≈0 furniture or an off-path semi."""
  v_ego = 12.0
  d_rel = LEFT_TURN_X_M - RADAR_TO_CAMERA_M
  path = (LEFT_TURN_PATH_X, LEFT_TURN_PATH_Y)
  ahead = track(905, d_rel, y_rel=0.0, v_rel=-v_ego)
  left_sign = track(872, d_rel, y_rel=2.0, v_rel=-v_ego)
  semi = track(880, d_rel, y_rel=0.4, v_rel=-(v_ego + 16.0))
  for t in (ahead, left_sign, semi):
    assert not radar_hold_kinematics_ok(t, v_ego=v_ego, path_x=path[0], path_y=path[1])
    assert pick_rain_radar_track(t, {t.identifier: t}, None, v_ego, path[0], path[1]) is None
    assert pick_rain_radar_track(None, {t.identifier: t}, t.identifier, v_ego,
                                 path[0], path[1]) is None


def test_pick_does_not_latch_outside_curve_sign():
  """~20:55 CT: rain must not hold a two-lane-outside static / bogus-oncoming sign."""
  v_ego = 16.0
  path_at = path_y_at_x(CURVE_OUTSIDE_PATH_X, CURVE_OUTSIDE_PATH_Y, CURVE_OUTSIDE_X_M)
  y_rel = -(path_at - CURVE_OUTSIDE_TWO_LANES_M)
  d_rel = CURVE_OUTSIDE_X_M - RADAR_TO_CAMERA_M
  path = (CURVE_OUTSIDE_PATH_X, CURVE_OUTSIDE_PATH_Y)
  stationary = track(33, d_rel, y_rel=y_rel, v_rel=-v_ego)
  bogus = track(34, d_rel, y_rel=y_rel, v_rel=-(v_ego + 18.0))
  for phantom in (stationary, bogus):
    assert not radar_hold_kinematics_ok(phantom, v_ego=v_ego, path_x=path[0], path_y=path[1])
    assert pick_rain_radar_track(phantom, {phantom.identifier: phantom}, None,
                                 v_ego, path[0], path[1]) is None
    assert pick_rain_radar_track(None, {phantom.identifier: phantom}, phantom.identifier,
                                 v_ego, path[0], path[1]) is None
