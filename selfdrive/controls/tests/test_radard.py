import pytest

from cereal import messaging
from openpilot.selfdrive.controls.lib.radar_path_gate import (
  CURVE_OUTSIDE_PATH_X,
  CURVE_OUTSIDE_PATH_Y,
  CURVE_OUTSIDE_TWO_LANES_M,
  CURVE_OUTSIDE_X_M,
  RADAR_TO_CAMERA_M,
  ROUNDABOUT_EXIT_PATH_X,
  ROUNDABOUT_EXIT_PATH_Y,
  ROUNDABOUT_EXIT_X_M,
  path_y_at_x,
)
from openpilot.selfdrive.controls.radard import KalmanParams, RADAR_TO_CAMERA, RadarD, Track


MAX_RADAR_MEASUREMENT_AGE = 0.5


class RadarScenario:
  def __init__(self, v_ego: float = 20.0):
    services = ["modelV2", "carState", "liveTracks"]
    self.sm = messaging.SubMaster(services, ignore_alive=services, ignore_avg_freq=services)
    self.radar = RadarD()
    self.v_ego = v_ego
    self.frame = 0

  def step(self, time_s: float, vision_d_rel: float, radar_points: list[tuple[int, float, float, float]] | None = None,
           vision_v: float | None = None, vision_prob: float | None = None,
           vision_y: float | None = None,
           path_x: list[float] | None = None, path_y: list[float] | None = None):
    messages = [self._model_message(time_s, vision_d_rel, vision_v, vision_prob,
                                    vision_y, path_x, path_y)]
    if self.frame == 0:
      messages.append(self._car_state_message(time_s))
    if radar_points is not None:
      messages.append(self._radar_message(time_s, radar_points))

    self.sm.update_msgs(time_s, [message.as_reader() for message in messages])
    self.radar.update(self.sm, self.sm["liveTracks"])
    self.frame += 1
    return self.radar.radar_state.leadOne

  def set_rain_hold(self, raining: bool | None):
    self.radar.rain_gate.set_override(raining)

  def _model_message(self, time_s: float, vision_d_rel: float, vision_v: float | None,
                     vision_prob: float | None, vision_y: float | None = None,
                     path_x: list[float] | None = None, path_y: list[float] | None = None):
    message = messaging.new_message("modelV2")
    message.logMonoTime = int(time_s * 1e9)
    message.modelV2.velocity.x = [self.v_ego]
    if path_x is not None and path_y is not None:
      message.modelV2.position.x = list(path_x)
      message.modelV2.position.y = list(path_y)
    leads = message.modelV2.init("leadsV3", 2)
    for lead in leads:
      lead.prob = 0.9 if vision_prob is None else vision_prob
      lead.x = [vision_d_rel + RADAR_TO_CAMERA]
      lead.xStd = [3.0]
      lead.y = [0.0 if vision_y is None else vision_y]
      lead.yStd = [1.0]
      lead.v = [self.v_ego if vision_v is None else vision_v]
      lead.vStd = [2.0]
      lead.a = [0.0]
    return message

  def _car_state_message(self, time_s: float):
    message = messaging.new_message("carState")
    message.logMonoTime = int(time_s * 1e9)
    message.carState.vEgo = self.v_ego
    return message

  @staticmethod
  def _radar_message(time_s: float, radar_points: list[tuple[int, float, float, float]]):
    message = messaging.new_message("liveTracks")
    message.logMonoTime = int(time_s * 1e9)
    points = message.liveTracks.init("points", len(radar_points))
    for point, (track_id, d_rel, y_rel, v_rel) in zip(points, radar_points, strict=True):
      point.trackId = track_id
      point.dRel = d_rel
      point.yRel = y_rel
      point.vRel = v_rel
      point.measured = True
    return message


def test_radar_silence_falls_back_to_current_vision_lead():
  scenario = RadarScenario()
  lead = scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  assert lead.radar

  model_dt = 0.05
  expiration_frame = int(MAX_RADAR_MEASUREMENT_AGE / model_dt) + 1
  for frame in range(1, expiration_frame):
    lead = scenario.step(1.0 + frame * model_dt, vision_d_rel=36.0)
  assert lead.radar

  lead = scenario.step(1.0 + expiration_frame * model_dt, vision_d_rel=36.0)

  assert not lead.radar
  assert lead.dRel == pytest.approx(36.0)


def test_association_rejects_distance_outlier():
  scenario = RadarScenario()
  lead = scenario.step(1.0, vision_d_rel=100.0, radar_points=[(7, 120.0, 0.0, 0.0)])

  assert not lead.radar
  assert lead.dRel == pytest.approx(100.0)


def test_association_rejects_velocity_outlier():
  scenario = RadarScenario()
  lead = scenario.step(1.0, vision_d_rel=50.0, radar_points=[(7, 50.0, 0.0, 15.0)])

  assert not lead.radar
  assert lead.vLead == pytest.approx(20.0)


def test_association_requires_minimum_score():
  scenario = RadarScenario()
  # Each residual remains inside its independent 3-sigma gate, while their
  # combined likelihood is below the score floor.
  lead = scenario.step(1.0, vision_d_rel=50.0, radar_points=[(7, 58.0, 2.9, 5.5)])

  assert not lead.radar


def test_association_retains_incumbent_until_challenger_wins():
  scenario = RadarScenario()
  two_tracks = [(81, 70.0, 0.0, 0.0), (82, 72.0, 0.0, 0.0)]
  vision_distances = [70.8, 71.2, 70.8, 71.2, 70.8, 71.2, 72.0, 70.8]

  selected_ids = [
    scenario.step(1.0 + frame * 0.1, vision_d_rel=vision_d_rel, radar_points=two_tracks).radarTrackId
    for frame, vision_d_rel in enumerate(vision_distances)
  ]
  selected_ids.append(scenario.step(1.8, vision_d_rel=70.0, radar_points=[two_tracks[0]]).radarTrackId)

  assert selected_ids == [81, 81, 81, 81, 81, 81, 82, 82, 81]


def test_unmeasured_track_does_not_update_kalman_state():
  track = Track(identifier=7, v_lead=20.0, kalman_params=KalmanParams(0.1))
  track.update(d_rel=30.0, y_rel=0.0, v_rel=0.0, v_lead=20.0, measured=True)
  track.update(d_rel=30.0, y_rel=0.0, v_rel=1.0, v_lead=21.0, measured=True)
  measured_state = (track.vLeadK, track.aLeadK)

  track.update(d_rel=30.0, y_rel=0.0, v_rel=10.0, v_lead=30.0, measured=False)

  assert (track.vLeadK, track.aLeadK) == pytest.approx(measured_state)


def test_kalman_uses_observed_radar_interval():
  scenario = RadarScenario()
  scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])

  scenario.step(1.08, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])

  assert scenario.radar.tracks[7].K_A[0][1] == pytest.approx(0.08)


# Evening rain dig (1c95345a3286a5db|000000df--467073c363, 19:39:05 CT):
# radar track 806 at 93.8 m, vision jumps to 61.6 m with modelProb 0.978.
DIG_RADAR_ID = 806
DIG_RADAR_DREL = 93.8
DIG_VISION_WRONG_DREL = 61.6
DIG_VISION_PROB = 0.978


def _dig_radar_point(d_rel: float = DIG_RADAR_DREL):
  return (DIG_RADAR_ID, d_rel, 0.0, 0.0)


def test_dry_vision_confident_wrong_range_still_drops_to_vision():
  """Off rain gate: existing fusion. High modelProb does not keep a mismatched radar track."""
  scenario = RadarScenario()
  scenario.set_rain_hold(False)
  lead = scenario.step(1.0, vision_d_rel=DIG_RADAR_DREL, radar_points=[_dig_radar_point()],
                       vision_prob=DIG_VISION_PROB)
  assert lead.radar
  assert lead.radarTrackId == DIG_RADAR_ID

  lead = scenario.step(1.1, vision_d_rel=DIG_VISION_WRONG_DREL, radar_points=[_dig_radar_point()],
                       vision_prob=DIG_VISION_PROB)
  assert not lead.radar
  assert lead.dRel == pytest.approx(DIG_VISION_WRONG_DREL)
  assert lead.modelProb == pytest.approx(DIG_VISION_PROB)


def test_rain_holds_radar_through_vision_confident_wrong_range():
  """On rain gate: keep track 806 through the dig's −32 m vision jump at modelProb 0.978."""
  scenario = RadarScenario()
  scenario.set_rain_hold(True)
  lead = scenario.step(1.0, vision_d_rel=DIG_RADAR_DREL, radar_points=[_dig_radar_point()],
                       vision_prob=DIG_VISION_PROB)
  assert lead.radar
  assert lead.radarTrackId == DIG_RADAR_ID

  for frame, vision_d_rel in enumerate((61.6, 89.0, 65.5, 68.3, 49.4), start=1):
    lead = scenario.step(1.0 + frame * 0.05, vision_d_rel=vision_d_rel,
                         radar_points=[_dig_radar_point()], vision_prob=DIG_VISION_PROB)
    assert lead.radar, f"frame {frame} dropped radar at vision {vision_d_rel}"
    assert lead.radarTrackId == DIG_RADAR_ID
    assert lead.dRel == pytest.approx(DIG_RADAR_DREL)


def test_rain_raises_vision_only_modelprob_bar():
  scenario = RadarScenario()
  scenario.set_rain_hold(True)
  lead = scenario.step(1.0, vision_d_rel=40.0, vision_prob=0.60)
  assert not lead.status

  lead = scenario.step(1.05, vision_d_rel=40.0, vision_prob=0.91)
  assert lead.status
  assert not lead.radar
  assert lead.dRel == pytest.approx(40.0)


def test_dry_vision_only_modelprob_bar_unchanged():
  scenario = RadarScenario()
  scenario.set_rain_hold(False)
  lead = scenario.step(1.0, vision_d_rel=40.0, vision_prob=0.60)
  assert lead.status
  assert not lead.radar
  assert lead.dRel == pytest.approx(40.0)


def test_rain_hold_survives_many_mismatch_frames_while_radar_lives():
  scenario = RadarScenario()
  scenario.set_rain_hold(True)
  scenario.step(1.0, vision_d_rel=DIG_RADAR_DREL, radar_points=[_dig_radar_point()],
                vision_prob=DIG_VISION_PROB)
  for frame in range(1, 21):
    lead = scenario.step(1.0 + frame * 0.05, vision_d_rel=DIG_VISION_WRONG_DREL,
                         radar_points=[_dig_radar_point()], vision_prob=DIG_VISION_PROB)
    assert lead.radarTrackId == DIG_RADAR_ID
    assert lead.dRel == pytest.approx(DIG_RADAR_DREL)


def test_rain_holds_cached_radar_then_allows_vision_after_track_lost():
  from openpilot.selfdrive.controls.lib.rain_radar_hold import RAIN_RADAR_LOST_HOLD_FRAMES

  scenario = RadarScenario()
  scenario.set_rain_hold(True)
  lead = scenario.step(1.0, vision_d_rel=30.0, radar_points=[(7, 30.0, 0.0, 0.0)])
  assert lead.radarTrackId == 7

  for frame in range(1, RAIN_RADAR_LOST_HOLD_FRAMES + 1):
    lead = scenario.step(1.0 + frame * 0.05, vision_d_rel=36.0, radar_points=[], vision_prob=0.95)
    assert lead.radar, f"lost-hold dropped at frame {frame}"
    assert lead.radarTrackId == 7
    assert lead.dRel == pytest.approx(30.0)

  lead = scenario.step(1.0 + (RAIN_RADAR_LOST_HOLD_FRAMES + 1) * 0.05, vision_d_rel=36.0,
                       radar_points=[], vision_prob=0.95)
  assert not lead.radar
  assert lead.dRel == pytest.approx(36.0)


# Left roadside sign / opposing traffic — Fri 2026-09-18 ~20:49 CT intersection regen.
LEFT_SIGN_YREL = 3.2
ONCOMING_VREL = -38.0  # v_ego 20 → vLead −18
IN_PATH = (11, 40.0, 0.3, -0.4)


def test_dry_off_path_lateral_does_not_become_radar_lead():
  """yRel 2.8 is inside the old 3-sigma vision gate; path gate must still reject."""
  scenario = RadarScenario()
  lead = scenario.step(1.0, vision_d_rel=40.0, radar_points=[(3, 40.0, 2.8, 0.0)])
  assert not lead.radar


def test_rain_does_not_hold_left_roadside_sign():
  scenario = RadarScenario()
  scenario.set_rain_hold(True)
  # No confident vision lead — rain used to pick closest |yRel|≤2.5 / hold ≤4.0.
  lead = scenario.step(1.0, vision_d_rel=48.0, radar_points=[(44, 48.0, LEFT_SIGN_YREL, -20.0)],
                       vision_prob=0.40)
  assert not lead.status or not lead.radar
  lead = scenario.step(1.1, vision_d_rel=48.0, radar_points=[(44, 48.0, LEFT_SIGN_YREL, -20.0)],
                       vision_prob=0.40)
  assert not lead.radar


def test_rain_does_not_latch_oncoming_as_follow_lead():
  scenario = RadarScenario()
  scenario.set_rain_hold(True)
  pts = [(9, 55.0, -1.5, ONCOMING_VREL)]
  lead = scenario.step(1.0, vision_d_rel=55.0, radar_points=pts, vision_prob=0.40,
                       vision_v=-18.0)
  assert not lead.radar
  assert not lead.status
  # Live reject must not keep a phantom cache either.
  lead = scenario.step(1.1, vision_d_rel=55.0, radar_points=pts, vision_prob=0.40,
                       vision_v=-18.0)
  assert not lead.radar
  assert not lead.status


def test_dry_oncoming_does_not_associate_as_radar_lead():
  scenario = RadarScenario()
  lead = scenario.step(1.0, vision_d_rel=50.0, radar_points=[(9, 50.0, -1.2, ONCOMING_VREL)],
                       vision_v=-18.0)
  assert not lead.radar
  assert not lead.status


def test_straight_opposing_passers_stay_ignored_dry_and_rain():
  """~20:58 CT: multiple opposing-lane vehicles, no leadOne / no regen.

  Path-gating must not FOV-pick them. An in-path lead still associates
  while they pass.
  """
  v_ego = 20.0
  passers = [
    (1, 80.0, 3.1, -(v_ego + 22.0)),
    (2, 50.0, 2.9, -(v_ego + 18.0)),
    (3, 28.0, 3.4, -(v_ego + 20.0)),
  ]
  for raining in (False, True):
    scenario = RadarScenario(v_ego=v_ego)
    scenario.set_rain_hold(raining)
    lead = scenario.step(1.0, vision_d_rel=70.0, radar_points=passers,
                         vision_prob=0.40, vision_v=-(v_ego + 20.0))
    assert not lead.radar, f"rain={raining} latched a 20:58 passer"
    assert not lead.status, f"rain={raining} published opposing as leadOne"

  scenario = RadarScenario(v_ego=v_ego)
  scenario.set_rain_hold(True)
  lead = scenario.step(1.0, vision_d_rel=40.0, radar_points=[IN_PATH, *passers])
  assert lead.radar
  assert lead.radarTrackId == 11


def test_in_path_lead_still_acquired_dry_and_rain():
  for raining in (False, True):
    scenario = RadarScenario()
    scenario.set_rain_hold(raining)
    lead = scenario.step(1.0, vision_d_rel=40.0, radar_points=[IN_PATH])
    assert lead.radar, f"rain={raining} missed in-path lead"
    assert lead.radarTrackId == 11
    assert lead.dRel == pytest.approx(40.0)


def test_rain_hold_does_not_override_path_or_oncoming_rejects():
  scenario = RadarScenario()
  scenario.set_rain_hold(True)
  # Acquire a real in-path lead first (so incumbent / cache exist).
  lead = scenario.step(1.0, vision_d_rel=40.0, radar_points=[IN_PATH])
  assert lead.radarTrackId == 11

  # Same ID walks off path to a left sign — rain must drop, not hold.
  lead = scenario.step(1.1, vision_d_rel=40.0, radar_points=[(11, 40.0, LEFT_SIGN_YREL, -20.0)],
                       vision_prob=0.40)
  assert not lead.radar

  # Fresh oncoming track must not replace it.
  lead = scenario.step(1.2, vision_d_rel=40.0, radar_points=[(9, 42.0, -1.4, ONCOMING_VREL)],
                       vision_prob=0.40, vision_v=-16.0)
  assert not lead.radar
  assert not lead.status


def test_rain_still_holds_true_in_path_through_vision_jump():
  scenario = RadarScenario()
  scenario.set_rain_hold(True)
  lead = scenario.step(1.0, vision_d_rel=40.0, radar_points=[IN_PATH])
  assert lead.radarTrackId == 11
  lead = scenario.step(1.05, vision_d_rel=18.0, radar_points=[IN_PATH], vision_prob=0.97)
  assert lead.radar
  assert lead.radarTrackId == 11
  assert lead.dRel == pytest.approx(40.0)


def test_rain_does_not_acquire_unassociated_radar():
  """#201 FOV prefer: closest in-lane radar with no path association must not latch."""
  scenario = RadarScenario()
  scenario.set_rain_hold(True)
  lead = scenario.step(1.0, vision_d_rel=80.0, radar_points=[(44, 40.0, 1.2, -20.0)],
                       vision_prob=0.40)
  assert not lead.radar
  assert not lead.status


def test_rain_does_not_swap_path_lead_for_left_sign():
  scenario = RadarScenario()
  scenario.set_rain_hold(True)
  both = [IN_PATH, (44, 36.0, LEFT_SIGN_YREL, -20.0)]
  lead = scenario.step(1.0, vision_d_rel=40.0, radar_points=both)
  assert lead.radarTrackId == 11
  lead = scenario.step(1.1, vision_d_rel=18.0, radar_points=both, vision_prob=0.97)
  assert lead.radar
  assert lead.radarTrackId == 11


def test_model_path_rejects_off_path_even_when_yrel_is_small():
  """OP path curves right; a yRel≈0 roadside object is off the driving path."""
  path_x = [0.0, 20.0, 40.0, 80.0]
  path_y = [0.0, -1.5, -3.0, -4.0]
  scenario = RadarScenario()
  scenario.set_rain_hold(True)
  lead = scenario.step(1.0, vision_d_rel=38.48, radar_points=[(3, 38.48, 0.0, 0.0)],
                       path_x=path_x, path_y=path_y)
  assert not lead.radar

  # Same curve: radar on the path (device y = −3 → yRel = +3) still associates.
  lead = scenario.step(1.1, vision_d_rel=38.48, radar_points=[(11, 38.48, 3.0, 0.0)],
                       vision_y=-3.0, path_x=path_x, path_y=path_y)
  assert lead.radar
  assert lead.radarTrackId == 11


def test_roundabout_right_exit_does_not_follow_entering_vehicle():
  """~20:54 CT: path turns right to leave; entering / cross traffic is off-path.

  Entrant is not oncoming (circulating, vRel≈−2). yRel≈0 looks like an
  in-path lead in a wide FOV. Dry and rain must not make it leadOne.
  A vehicle already on the exit path still associates.
  """
  path_x = list(ROUNDABOUT_EXIT_PATH_X)
  path_y = list(ROUNDABOUT_EXIT_PATH_Y)
  d_rel = ROUNDABOUT_EXIT_X_M - RADAR_TO_CAMERA_M
  entrant = (22, d_rel, 0.0, -2.0)
  on_exit = (11, d_rel, 4.0, -1.0)

  for raining in (False, True):
    scenario = RadarScenario(v_ego=12.0)
    scenario.set_rain_hold(raining)
    lead = scenario.step(1.0, vision_d_rel=d_rel, radar_points=[entrant],
                         vision_prob=0.92, vision_y=0.0,
                         path_x=path_x, path_y=path_y)
    assert not lead.radar, f"rain={raining} latched roundabout entrant"
    assert not lead.status, f"rain={raining} vision-only followed entrant"

  scenario = RadarScenario(v_ego=12.0)
  scenario.set_rain_hold(True)
  lead = scenario.step(1.0, vision_d_rel=d_rel, radar_points=[on_exit, entrant],
                       vision_y=-4.0, path_x=path_x, path_y=path_y)
  assert lead.radar
  assert lead.radarTrackId == 11
  # Vision flaps toward the entrant — rain must keep the exit-path association.
  lead = scenario.step(1.1, vision_d_rel=d_rel, radar_points=[on_exit, entrant],
                       vision_prob=0.97, vision_y=0.0, path_x=path_x, path_y=path_y)
  assert lead.radar
  assert lead.radarTrackId == 11


def test_e1_episodes_do_not_latch_off_path_or_oncoming():
  """Pinned e1 kinematics (1c95345a3286a5db|000000e1--b993674371).

  Rain Auto On. Stock #201 published these as leadOne with mp≪0.15.
  """
  cases = (
    # CT          id   dRel   yRel   vRel          vision_prob
    ("20:46:45", 224, 50.0,  3.4,  -16.0,         0.02),   # Ep A LEFT STAT sweep
    ("20:49:10", 321, 99.3,  0.86, -16.05,        0.007),  # Ep B far STAT
    ("20:49:11", 318, 40.8,  3.36, -16.0,         0.008),  # Ep B LEFT OFFPATH
    ("20:51:52", 471, 45.0,  2.5,  -20.6,         0.05),   # Ep C LEFT STAT
    ("20:53:45", 573, 40.0,  3.73, -12.0,         0.04),   # Ep D roundabout LEFT
    ("20:54:53", 641, 50.0,  3.9,  -24.6,         0.022),  # Ep E curve-outside STAT
    ("20:45:47", 101, 55.0,  2.0,  -(16.0 + 5.5), 0.10),   # oncoming vLead≈−5.5
    ("20:59:33", 771, 45.0,  2.23, -(16.0 + 21.9), 0.05),  # EP_2059 near-edge
    ("21:00:12", 802, 40.0,  2.23, -(16.0 + 18.8), 0.04),  # EP_2059 semi
    ("20:59:40", 840, 100.0, 0.80, -16.0,          0.01),  # far STAT phantom
  )
  for label, tid, d_rel, y_rel, v_rel, mp in cases:
    scenario = RadarScenario(v_ego=16.0)
    scenario.set_rain_hold(True)
    lead = scenario.step(1.0, vision_d_rel=d_rel, radar_points=[(tid, d_rel, y_rel, v_rel)],
                         vision_prob=mp)
    assert not lead.radar, f"{label} track {tid} latched as radar leadOne"


def test_curve_outside_sign_does_not_become_oncoming_lead():
  """~20:55 CT: sign on the outside of a left curve (~two lanes off path).

  Static (vLead≈0) and bogus oncoming Doppler must not become leadOne.
  A lead on the curve still associates.
  """
  path_x = list(CURVE_OUTSIDE_PATH_X)
  path_y = list(CURVE_OUTSIDE_PATH_Y)
  path_at = path_y_at_x(path_x, path_y, CURVE_OUTSIDE_X_M)
  d_rel = CURVE_OUTSIDE_X_M - RADAR_TO_CAMERA_M
  y_rel = -(path_at - CURVE_OUTSIDE_TWO_LANES_M)
  v_ego = 16.0
  stationary = (33, d_rel, y_rel, -v_ego)
  bogus = (34, d_rel, y_rel, -(v_ego + 18.0))
  on_curve = (11, d_rel, -path_at, -1.0)

  for raining, pts, vision_v in (
    (False, [stationary], 0.0),
    (True, [stationary], 0.0),
    (False, [bogus], -18.0),
    (True, [bogus], -18.0),
  ):
    scenario = RadarScenario(v_ego=v_ego)
    scenario.set_rain_hold(raining)
    lead = scenario.step(1.0, vision_d_rel=d_rel, radar_points=pts,
                         vision_prob=0.92, vision_v=vision_v, vision_y=-y_rel,
                         path_x=path_x, path_y=path_y)
    assert not lead.radar, f"rain={raining} latched outside-curve sign"
    assert not lead.status, f"rain={raining} followed outside-curve sign"

  scenario = RadarScenario(v_ego=v_ego)
  scenario.set_rain_hold(True)
  lead = scenario.step(1.0, vision_d_rel=d_rel, radar_points=[on_curve, stationary],
                       vision_y=path_at, path_x=path_x, path_y=path_y)
  assert lead.radar
  assert lead.radarTrackId == 11


def test_ep2059_near_edge_oncoming_semi_does_not_become_lead():
  """20:59/21:00: yRel +2.23 inside old 2.5 m + oncoming vLead → no leadOne.

  21:02/21:03 mid-lane opposing (|yRel| > 2.5) stays leadOne=None.
  Oncoming reject is follow-lead only — it does not invent a new brake.
  """
  v_ego = 20.0
  near = (802, 40.0, 2.23, -(v_ego + 18.8))
  mid = (900, 50.0, 3.2, -(v_ego + 20.0))
  for raining, pts in ((True, [near]), (True, [mid]), (False, [near])):
    scenario = RadarScenario(v_ego=v_ego)
    scenario.set_rain_hold(raining)
    lead = scenario.step(1.0, vision_d_rel=pts[0][1], radar_points=pts,
                         vision_prob=0.40, vision_v=-(v_ego + 18.8))
    assert not lead.radar, f"rain={raining} pts={pts[0][0]} latched oncoming"
    assert not lead.status, f"rain={raining} pts={pts[0][0]} published leadOne"


def test_left_turn_straight_ahead_sign_off_path_not_lead():
  """~21:07 CT: sign dead ahead in radar (yRel≈0) but model path is turning left."""
  path_x = [0.0, 15.0, 30.0, 50.0]
  path_y = [0.0, 1.8, 4.0, 6.5]
  v_ego = 12.0
  sign = (55, 28.48, 0.0, -v_ego)
  for raining in (False, True):
    scenario = RadarScenario(v_ego=v_ego)
    scenario.set_rain_hold(raining)
    lead = scenario.step(1.0, vision_d_rel=28.48, radar_points=[sign],
                         vision_prob=0.40, path_x=path_x, path_y=path_y)
    assert not lead.radar, f"rain={raining} latched off-path ahead sign"
    assert not lead.status, f"rain={raining} followed off-path ahead sign"
