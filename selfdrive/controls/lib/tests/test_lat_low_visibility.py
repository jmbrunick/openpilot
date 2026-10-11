"""Low-visibility lateral back-off: confidence hysteresis, ramp, exposure, sun."""

import math
from pathlib import Path

from cereal import log
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.lat_low_visibility import (
  AUTHORITY_DOWN_PER_S,
  AUTHORITY_FLOOR,
  CAMERA_ONLY_AUTHORITY,
  PROLONGED_S,
  authority_target,
  AUTHORITY_UP_PER_S,
  CURV_JUMP,
  CURV_WINDOW_S,
  ENTRY_MIN_V_MS,
  EXPOSURE_COLLAPSE_RATIO,
  LANE_CONF_ENTER,
  LANE_CONF_EXIT,
  MODEL_ENTER_S,
  MODEL_EXIT_S,
  PARAM_LOW_VIS_BACKOFF,
  PATH_Y_STD_ENTER_M,
  PATH_Y_STD_EXIT_M,
  STRAIGHT_HOLD_S,
  TURN_CURVATURE,
  LowVisibility,
  edge_confidence,
  edges_both_unusable,
  entry_speed_ok,
  fade_curvature,
  in_turn,
  model_is_clear,
  model_is_poor,
  path_y_std_at,
  side_confidences,
  side_confidences_exit,
  solar_position,
  sun_ahead_from_fix,
  sun_low_and_ahead,
)

DT = 0.05  # one model frame


def _step(lv, *, left=0.95, right=0.95, ystd=0.4, seconds=DT, integ=700,
          sun=False, enabled=True, dt=DT, edges=None,
          v_ego=15.0, blinker=False, yielded=False,
          measured_k=0.0, model_k=0.0, chatter=None, near_roundabout=False):
  n = max(1, int(round(seconds / dt)))
  out = None
  if edges is None:
    edges = [0.12, 0.12]
  for i in range(n):
    mk = model_k
    if chatter is not None:
      amp = float(chatter)
      mk = amp if (i % 2 == 0) else -amp
    out = lv.update(
      enabled=enabled,
      lane_probs=[0.5, left, right, 0.5],
      edge_stds=edges,
      path_t=[0.0, 1.5, 3.0, 6.0],
      path_y_std=[0.2, 0.3, ystd, ystd],
      integ_lines=integ,
      sun_ahead=sun,
      dt=dt,
      v_ego=v_ego,
      blinker=blinker,
      yielded=yielded,
      measured_curvature=measured_k,
      model_curvature=mk,
      near_roundabout=near_roundabout,
    )
  return out


# Blind road edges (std large enough that confidence is under 0.30).
NO_EDGES = [8.0, 8.0]


def _mph(mph: float) -> float:
  return mph * CV.MPH_TO_MS


def test_side_confidence_and_path_std_match_the_incident_numbers():
  # Right lane 0.12 is below 0.30; left 0.56 is not. Either side is enough.
  left, right = side_confidences([0.4, 0.56, 0.12, 0.3], [0.1, 0.1])
  assert left > LANE_CONF_ENTER
  assert right < LANE_CONF_ENTER
  y = path_y_std_at([0.0, 2.0, 4.0], [0.4, 4.1, 5.0], 3.0)
  assert abs(y - (4.1 + 0.5 * (5.0 - 4.1))) < 1e-6
  # Lane lines do not make the path poor. Wide plan + no edges + jump does.
  assert not model_is_poor(4.1, False, True)
  assert not model_is_poor(4.1, True, False)
  assert not model_is_poor(0.4, True, True)
  assert model_is_poor(4.1, True, True)
  assert model_is_clear(0.4, True)
  assert not model_is_clear(0.4, False)
  assert not model_is_clear(5.0, True)  # wide plan holds, whatever the lanes say
  assert not model_is_clear(None, True)
  assert edges_both_unusable([8.0, 8.0])
  assert not edges_both_unusable([0.1, 8.0])  # one curb is usable
  assert not edges_both_unusable([0.1])  # a missing edge is not blind
  exit_left, exit_right = side_confidences_exit([0.4, 0.90, 0.85, 0.3], [8.0, 8.0])
  assert exit_left >= LANE_CONF_EXIT and exit_right >= LANE_CONF_EXIT
  # The old worse-of helper still describes a wide edge. It is not the trigger.
  enter_left, enter_right = side_confidences([0.4, 0.90, 0.85, 0.3], [8.0, 8.0])
  assert enter_left < LANE_CONF_ENTER and enter_right < LANE_CONF_ENTER
  assert edge_confidence(0.1) > 0.8
  assert edge_confidence(1.5) < LANE_CONF_ENTER


def test_hysteresis_ignores_one_frame_and_the_middle_band():
  lv = LowVisibility()
  out = _step(lv, left=0.0, right=0.0, ystd=0.4, seconds=3.0, edges=NO_EDGES)
  assert not out.active and out.authority == 1.0

  # One uncertain frame must not latch. Lane lines are irrelevant.
  out = _step(lv, left=0.95, right=0.95, ystd=4.1, seconds=DT, edges=NO_EDGES, chatter=CURV_JUMP)
  assert not out.active

  out = _step(lv, left=0.0, right=0.0, ystd=4.1, seconds=MODEL_ENTER_S + DT,
              edges=NO_EDGES, chatter=CURV_JUMP)
  assert out.active and out.alert and out.model_poor
  assert out.authority < 1.0

  # Plan std between the bars, curvature still jumping: stay latched.
  held = _step(lv, left=0.90, right=0.90, ystd=1.5, seconds=2.0, edges=NO_EDGES, chatter=CURV_JUMP)
  assert held.active

  # Settled path, but desired curvature is still jumping: do not release.
  jumping = _step(lv, left=0.0, right=0.0, ystd=0.4, seconds=MODEL_EXIT_S + 0.3,
                  edges=NO_EDGES, chatter=CURV_JUMP)
  assert jumping.active

  # Calm curvature and a settled path. Short of the exit hold, stay latched.
  almost = _step(lv, left=0.0, right=0.0, ystd=0.4, seconds=MODEL_EXIT_S * 0.5,
                 edges=NO_EDGES, model_k=0.0)
  assert almost.active
  cleared = _step(lv, left=0.0, right=0.0, ystd=0.4, seconds=CURV_WINDOW_S + MODEL_EXIT_S,
                  edges=NO_EDGES, model_k=0.0)
  assert not cleared.active and not cleared.alert


def test_backoff_ramps_down_and_recovers_without_a_step():
  lv = LowVisibility()
  _step(lv, left=0.0, right=0.0, ystd=4.1, seconds=MODEL_ENTER_S + 0.05,
        dt=0.01, edges=NO_EDGES, chatter=CURV_JUMP)
  assert lv.active
  start = lv.authority
  mid = _step(lv, left=0.0, right=0.0, ystd=4.1, seconds=0.50, dt=0.01,
              edges=NO_EDGES, chatter=CURV_JUMP)
  assert mid.authority < start
  assert abs((start - mid.authority) - 0.50 * AUTHORITY_DOWN_PER_S) < 0.02
  floor = _step(lv, left=0.0, right=0.0, ystd=4.1, seconds=1.0, dt=0.01,
                edges=NO_EDGES, chatter=CURV_JUMP)
  # Never zero: 4.1 m path std bottoms out at the floor, and the lateral
  # target still follows the path (not the wheel, not a held heading).
  assert abs(floor.authority - AUTHORITY_FLOOR) < 1e-9
  assert fade_curvature(0.02, 0.0, floor.authority) > 0.0
  assert fade_curvature(0.02, 0.0, 1.0) == 0.02

  _step(lv, left=0.0, right=0.0, ystd=0.4, seconds=CURV_WINDOW_S + MODEL_EXIT_S, dt=0.01,
        edges=NO_EDGES, model_k=0.0)
  assert not lv.active
  rising = lv.authority
  later = _step(lv, left=0.0, right=0.0, ystd=0.4, seconds=0.50, dt=0.01,
                edges=NO_EDGES, model_k=0.0)
  assert later.authority > rising
  assert abs(later.authority - min(1.0, rising + 0.50 * AUTHORITY_UP_PER_S)) < 0.02


def test_paved_drive_does_not_back_off_and_toggle_off_is_identity():
  lv = LowVisibility()
  out = _step(lv, left=0.97, right=0.92, ystd=0.4, seconds=30.0)
  assert not out.active and out.authority == 1.0 and not out.alert

  lv2 = LowVisibility()
  out = _step(lv2, left=0.10, right=0.10, ystd=5.0, seconds=2.0, enabled=False)
  assert out.authority == 1.0 and not out.alert and not out.active


def test_three_incident_episodes_each_latch():
  """Three uncertain-path windows. Each has to enter on its own."""
  lv = LowVisibility()
  _step(lv, left=0.0, right=0.0, ystd=0.4, seconds=2.0, edges=NO_EDGES)
  episodes = []
  for _ in range(3):
    _step(lv, left=0.0, right=0.0, ystd=0.4, seconds=CURV_WINDOW_S + MODEL_EXIT_S + 0.3,
          edges=NO_EDGES, model_k=0.0)
    assert not lv.active
    out = _step(lv, left=0.95, right=0.95, ystd=4.1, seconds=MODEL_ENTER_S + 0.2,
                edges=NO_EDGES, chatter=CURV_JUMP)
    episodes.append(out)
    assert out.active and out.model_poor and out.authority < 1.0
  assert len(episodes) == 3
  # One calm frame, then a short gap under the exit hold, does not drop it.
  _step(lv, left=0.0, right=0.0, ystd=0.4, seconds=DT, edges=NO_EDGES, model_k=0.0)
  assert lv.active
  _step(lv, left=0.0, right=0.0, ystd=4.1, seconds=0.2, edges=NO_EDGES, chatter=CURV_JUMP)
  _step(lv, left=0.0, right=0.0, ystd=0.4, seconds=MODEL_EXIT_S * 0.4, edges=NO_EDGES, model_k=0.0)
  assert lv.active


def test_exposure_collapse_and_single_frame_and_sun_heads_up():
  lv = LowVisibility()
  # Bright baseline (731 lines), then the incident drop to 153.
  _step(lv, integ=731, seconds=2.0)
  assert not lv.active
  one = _step(lv, integ=153, seconds=DT)
  assert not one.camera_blind and not one.active

  # A plain exposure drop with a confident path and no low sun is a
  # shade-to-sun change, not glare: it never latches.
  plain_drop = _step(lv, integ=153, seconds=0.8)
  assert not plain_drop.camera_blind and not plain_drop.active

  lv = LowVisibility()
  _step(lv, integ=731, seconds=2.0)
  dropped = _step(lv, integ=153, seconds=0.8, sun=True)
  assert dropped.camera_blind and dropped.active and dropped.alert
  assert AUTHORITY_FLOOR <= dropped.authority < 1.0
  # 153/731 is well under the 0.45 collapse ratio.
  assert 153 / 731 < EXPOSURE_COLLAPSE_RATIO

  # Recover.
  back = _step(lv, integ=700, seconds=2.0, sun=True)
  assert not back.camera_blind and not back.active

  # Milder drop does not trip without the sun, and does with the sun ahead.
  plain = LowVisibility()
  _step(plain, integ=731, seconds=2.0)
  mild = int(731 * 0.60)  # between 0.45 and 0.65
  out = _step(plain, integ=mild, seconds=1.0, sun=False)
  assert not out.camera_blind

  early = LowVisibility()
  _step(early, integ=731, seconds=2.0)
  out = _step(early, integ=mild, seconds=1.0, sun=True)
  assert out.camera_blind and out.alert


def test_sun_gate_and_morning_geometry():
  assert sun_low_and_ahead(elevation_deg=0.6, heading_deg=90.0, azimuth_deg=98.0)
  assert not sun_low_and_ahead(elevation_deg=40.0, heading_deg=90.0, azimuth_deg=98.0)
  assert not sun_low_and_ahead(elevation_deg=1.0, heading_deg=90.0, azimuth_deg=140.0)

  # Oct 7 2026 12:33 UTC, ~45.3N 93.2W, heading due east: sun is low and
  # a few degrees south of east (right of ahead). Qualitative, not a survey.
  import datetime
  ts = datetime.datetime(2026, 10, 7, 12, 33, tzinfo=datetime.timezone.utc).timestamp()
  elev, az = solar_position(45.3, -93.2, ts)
  assert -1.0 < elev < 8.0
  assert sun_low_and_ahead(elevation_deg=elev, heading_deg=90.0, azimuth_deg=az)
  # Summer noon is high and not "ahead" of an east heading.
  noon = datetime.datetime(2026, 6, 21, 18, 0, tzinfo=datetime.timezone.utc).timestamp()
  elev_n, az_n = solar_position(45.3, -93.2, noon)
  assert elev_n > 50.0
  assert not sun_low_and_ahead(elevation_deg=elev_n, heading_deg=90.0, azimuth_deg=az_n)

  ms = int(ts * 1000)
  assert sun_ahead_from_fix(
    latitude=45.3, longitude=-93.2, unix_timestamp_millis=ms,
    bearing_deg=90.0, horizontal_accuracy_m=5.0) is True
  assert sun_ahead_from_fix(
    latitude=45.3, longitude=-93.2, unix_timestamp_millis=ms,
    bearing_deg=90.0, horizontal_accuracy_m=80.0) is None


def test_alert_text_and_toggle_are_wired():
  # Schema only: importing events pulls msgq, which unit tests don't need.
  assert log.OnroadEvent.EventName.lowVisibility is not None
  root = Path(__file__).resolve().parents[4]
  events = (root / "selfdrive/selfdrived/events.py").read_text()
  block = events.split("_low_visibility = getattr(EventName, \"lowVisibility\"", 1)[1]
  block = block.split("if HARDWARE.get_device_type()", 1)[0]
  assert '"Low visibility"' in block
  assert '"Take over if needed"' in block
  assert "ET.WARNING" in block
  assert "ET.SOFT_DISABLE" not in block
  assert "ET.IMMEDIATE_DISABLE" not in block
  assert "ET.USER_DISABLE" not in block
  keys = (root / "common/params_keys.h").read_text()
  line = next(ln for ln in keys.splitlines() if f'"{PARAM_LOW_VIS_BACKOFF}"' in ln)
  assert "BOOL" in line and '"1"' in line
  content = (root / "selfdrive/ui/layouts/settings/nap_content.py").read_text()
  assert "LOW_VIS_BACKOFF_DESCRIPTION" in content
  assert "Low visibility" in content
  assert "Longitudinal stays on" in content
  for rel in (
    "selfdrive/ui/layouts/settings/driving_mannerisms.py",
    "selfdrive/ui/mici/layouts/settings/driving_mannerisms.py",
    "selfdrive/controls/controlsd.py",
    "selfdrive/selfdrived/selfdrived.py",
  ):
    assert "NAP_LOW_VIS_BACKOFF" in (root / rel).read_text() or "lowVisibility" in (root / rel).read_text()
  controls = (root / "selfdrive/controls/controlsd.py").read_text()
  assert "fade_curvature" in controls
  # The back-off must not rewrite the longitudinal accel command.
  accel = controls.split("actuators.accel = published_long_accel(", 1)[1].split("Steering PID", 1)[0]
  assert "low_vis" not in accel
  assert "NAPLatRefOffset" not in controls.split("fade_curvature", 1)[1][:400]
  updater = controls.split("def _update_low_visibility", 1)[1].split("def _cone_curvature", 1)[0]
  # Reuse the carState socket and the curvatures controlsd already reads.
  assert "SubMaster" not in updater
  assert "carState" in updater
  assert "v_ego" in updater and "blinker" in updater and "yielded" in updater
  assert "measured_curvature" in updater and "model_curvature" in updater
  assert "lateralManeuverPlan" in updater
  assert "desiredCurvature" in updater
  # Roundabout gate reuses the yield context. No new carState socket.
  assert "roundabout_yield_context" in updater
  assert "liveMapDataNAP" in updater
  assert "near_roundabout" in updater
  # Yielded steering must not become a disengage from this path.
  assert "steeringDisengage" not in updater
  assert "CC.enabled" not in updater
  assert "CC.latActive" not in updater


def test_exit_follows_path_confidence_not_lane_lines():
  """A wide plan holds the latch. A settled calm path releases with no paint."""
  lv = LowVisibility()
  entered = _step(lv, left=0.95, right=0.95, ystd=4.1, seconds=MODEL_ENTER_S + DT,
                  edges=NO_EDGES, chatter=CURV_JUMP)
  assert entered.active and entered.model_poor

  # Lane lines back, path still wide, curvature still jumping: stay.
  held = _step(lv, left=0.92, right=0.85, ystd=2.5, seconds=MODEL_EXIT_S + 0.4,
               edges=NO_EDGES, chatter=CURV_JUMP)
  assert held.active

  # No lane lines, path settled, curvature calm: release.
  cleared = _step(lv, left=0.0, right=0.0, ystd=0.4,
                  seconds=CURV_WINDOW_S + MODEL_EXIT_S, edges=NO_EDGES, model_k=0.0)
  assert not cleared.active and not cleared.alert and not cleared.model_poor


def test_turn_does_not_enter_and_straight_at_speed_does():
  """A turn is not low visibility. The same unsure path on a straight is."""
  assert TURN_CURVATURE == 0.01
  assert entry_speed_ok(_mph(25.0))
  assert not entry_speed_ok(_mph(20.0))
  assert not entry_speed_ok(_mph(19.0))
  assert in_turn(0.048)
  assert not in_turn(0.0)
  assert not in_turn(0.004)
  # A jumping desired curvature is not the car turning.
  assert not in_turn(0.0)
  assert abs(ENTRY_MIN_V_MS - _mph(20.0)) < 1e-9

  turn = LowVisibility()
  out = _step(turn, left=0.01, right=0.01, ystd=3.0, seconds=3.0,
              v_ego=_mph(25.0), measured_k=0.048, chatter=CURV_JUMP, edges=NO_EDGES)
  assert not out.active and not out.alert and out.authority == 1.0

  straight = LowVisibility()
  early = _step(straight, left=0.01, right=0.01, ystd=3.0, seconds=MODEL_ENTER_S - DT,
                v_ego=_mph(25.0), measured_k=0.0, chatter=CURV_JUMP, edges=NO_EDGES)
  assert not early.active
  entered = _step(straight, left=0.01, right=0.01, ystd=3.0, seconds=3 * DT,
                  v_ego=_mph(25.0), chatter=CURV_JUMP, edges=NO_EDGES)
  assert entered.active and entered.model_poor and entered.alert


def test_confident_path_releases_quickly_and_a_turn_holds():
  """Once the plan settles and desired curvature calms, release. A turn holds."""
  assert PATH_Y_STD_EXIT_M == 1.2
  assert not model_is_clear(PATH_Y_STD_EXIT_M, True)
  assert model_is_clear(0.8, True)
  assert not model_is_poor(0.8, True, True)

  lv = LowVisibility()
  entered = _step(lv, left=0.0, right=0.0, ystd=PATH_Y_STD_ENTER_M + 1.0,
                  seconds=MODEL_ENTER_S + DT, edges=NO_EDGES, chatter=CURV_JUMP, v_ego=_mph(30.0))
  assert entered.active

  # Path settled but desired curvature still jumping: stay.
  almost = _step(lv, left=0.90, right=0.05, ystd=0.8, seconds=MODEL_EXIT_S,
                 edges=NO_EDGES, v_ego=_mph(30.0), chatter=CURV_JUMP)
  assert almost.active
  cleared = _step(lv, left=0.0, right=0.05, ystd=0.8,
                  seconds=CURV_WINDOW_S + MODEL_EXIT_S,
                  edges=NO_EDGES, v_ego=_mph(30.0), model_k=0.0)
  assert not cleared.active and not cleared.alert

  held_turn = LowVisibility()
  _step(held_turn, left=0.0, right=0.0, ystd=3.0, seconds=MODEL_ENTER_S + DT,
        edges=NO_EDGES, chatter=CURV_JUMP)
  assert held_turn.active
  # Already active as the car turns: stay up even though the plan looks calm.
  still = _step(held_turn, left=0.95, right=0.95, ystd=0.4, seconds=2.0,
                edges=NO_EDGES, measured_k=0.059, model_k=0.0)
  assert still.active and still.model_poor

  # Wide plan keeps the latch after the turn, even with lane lines painted back on.
  blind = LowVisibility()
  _step(blind, left=0.0, right=0.0, ystd=3.0, seconds=MODEL_ENTER_S + DT,
        edges=NO_EDGES, chatter=CURV_JUMP)
  stayed = _step(blind, left=0.90, right=0.90, ystd=3.0, seconds=2.0,
                 edges=NO_EDGES, chatter=CURV_JUMP)
  assert stayed.active and stayed.alert


def test_latched_alert_stays_up_through_a_turn_then_releases():
  lv = LowVisibility()
  _step(lv, left=0.0, right=0.0, ystd=3.0, seconds=MODEL_ENTER_S + DT,
        v_ego=_mph(28.0), edges=NO_EDGES, chatter=CURV_JUMP)
  assert lv.active
  mid = _step(lv, left=0.95, right=0.95, ystd=0.4, seconds=2.0,
              v_ego=_mph(22.0), measured_k=0.04, model_k=0.0, edges=NO_EDGES)
  assert mid.active
  short = _step(lv, left=0.0, right=0.0, ystd=0.8, seconds=MODEL_EXIT_S * 0.5,
                v_ego=_mph(30.0), measured_k=0.0, model_k=0.0, edges=NO_EDGES)
  assert short.active
  done = _step(lv, left=0.0, right=0.0, ystd=0.8, seconds=CURV_WINDOW_S + MODEL_EXIT_S,
               v_ego=_mph(30.0), edges=NO_EDGES, model_k=0.0)
  assert not done.active


def test_uncertain_path_one_second_after_a_turn_still_enters():
  lv = LowVisibility()
  turning = _step(lv, left=0.0, right=0.0, ystd=3.0, seconds=2.0,
                  v_ego=_mph(25.0), measured_k=0.04, chatter=CURV_JUMP, edges=NO_EDGES)
  assert not turning.active
  arming = _step(lv, left=0.0, right=0.0, ystd=3.0, seconds=STRAIGHT_HOLD_S - 0.10,
                 v_ego=_mph(25.0), measured_k=0.0, chatter=CURV_JUMP, edges=NO_EDGES)
  assert not arming.active
  entered = _step(lv, left=0.0, right=0.0, ystd=3.0,
                  seconds=0.10 + MODEL_ENTER_S + DT,
                  v_ego=_mph(25.0), chatter=CURV_JUMP, edges=NO_EDGES)
  assert entered.active and entered.model_poor


def test_blinker_yield_and_low_speed_hold_the_timer_at_zero():
  poor = dict(left=0.0, right=0.0, ystd=4.0, edges=NO_EDGES, chatter=CURV_JUMP)
  for gate in (
    dict(blinker=True, v_ego=_mph(30.0)),
    dict(yielded=True, v_ego=_mph(30.0)),
    dict(v_ego=_mph(17.0)),
    dict(v_ego=_mph(20.0)),
  ):
    lv = LowVisibility()
    out = _step(lv, seconds=2.0, **poor, **gate)
    assert not out.active and out.authority == 1.0
  # Crossing back above 20 mph with the wheel straight starts a fresh 0.40 s.
  lv = LowVisibility()
  _step(lv, seconds=1.0, **poor, v_ego=_mph(18.0))
  assert not lv.active
  entered = _step(lv, seconds=MODEL_ENTER_S + DT, **poor, v_ego=_mph(21.0))
  assert entered.active


def test_camera_collapse_is_unchanged_in_a_turn():
  """Sun glare does not consult the turn gate."""
  lv = LowVisibility()
  _step(lv, integ=731, seconds=2.0, measured_k=0.05, model_k=0.05,
        v_ego=_mph(12.0), blinker=True, left=0.95, right=0.95, ystd=0.4)
  assert not lv.active
  one = _step(lv, integ=153, seconds=DT, measured_k=0.05, model_k=0.05, v_ego=_mph(12.0))
  assert not one.camera_blind and not one.active
  dropped = _step(lv, integ=153, seconds=0.8, measured_k=0.05, model_k=0.05,
                  v_ego=_mph(12.0), blinker=True, sun=True)
  assert dropped.camera_blind and dropped.active and dropped.alert
  assert 153 / 731 < EXPOSURE_COLLAPSE_RATIO
  back = _step(lv, integ=700, seconds=2.0, measured_k=0.05, model_k=0.05, v_ego=_mph(12.0), sun=True)
  assert not back.camera_blind and not back.active


def _frames(seconds, dt=DT):
  return max(1, int(round(seconds / dt)))


def _scene(lv, *, left, right, ystd, v_mph, measured_k, model_k,
           blinker=False, yielded=False, edges=None, integ=2016, dt=DT):
  if edges is None:
    edges = [8.0, 8.0]
  return lv.update(
    enabled=True,
    lane_probs=[0.5, left, right, 0.5],
    edge_stds=edges,
    path_t=[0.0, 1.5, 3.0, 6.0],
    path_y_std=[0.2, 0.4, ystd, ystd],
    integ_lines=integ,
    sun_ahead=False,
    dt=dt,
    v_ego=_mph(v_mph),
    blinker=blinker,
    yielded=yielded,
    measured_curvature=measured_k,
    model_curvature=model_k,
  )


def test_route174_200426_blinker_turn_does_not_trigger():
  """20:04:26 right turn, blinker on, 18–23 mph, lanes ~0.03, 6.6 s."""
  lv = LowVisibility()
  dt = DT
  n = _frames(6.6, dt)
  for i in range(n):
    mph = 18.0 + (23.0 - 18.0) * (i / max(1, n - 1))
    out = _scene(lv, left=0.03, right=0.03, ystd=3.0, v_mph=mph,
                 measured_k=0.048, model_k=0.032, blinker=True, dt=dt)
    assert not out.active and not out.alert and not out.model_poor


def test_route174_200616_yielded_s_bend_does_not_trigger():
  """20:06:16 S-bend at 17 mph. Low-vis rose after a same-direction yield."""
  lv = LowVisibility()
  dt = DT
  curvatures = (-0.053, -0.036, -0.042)
  steered = (-0.050, -0.051, -0.041)
  n = _frames(13.6, dt)
  for i in range(n):
    k = curvatures[i % 3]
    out = _scene(lv, left=0.05, right=0.08, ystd=3.0, v_mph=17.0,
                 measured_k=steered[i % 3], model_k=k, yielded=True, dt=dt)
    assert not out.active and not out.alert


def test_route174_201044_lot_turns_do_not_trigger():
  """20:10:44 final turns into the lot, 3–17 mph, ended parked. Was 96.4 s."""
  lv = LowVisibility()
  dt = 0.10
  n = _frames(96.4, dt)
  for i in range(n):
    mph = 3.0 + (17.0 - 3.0) * (0.5 + 0.5 * math.sin(i / 40.0))
    if i < n // 2:
      model_k, measured_k = 0.097, 0.088
    else:
      model_k, measured_k = -0.050, -0.051
    lane = 0.01 + 0.07 * (i % 5) / 4.0
    out = _scene(lv, left=lane, right=lane, ystd=3.5, v_mph=mph,
                 measured_k=measured_k, model_k=model_k, dt=dt)
    assert not out.active and not out.alert and out.authority == 1.0


def test_route174_200934_confident_straight_does_not_hold():
  """20:09:34 turn from a stop, then a straight with only the left line.

  Missing the right line is not low visibility. A confident path stays
  quiet, and an unsure path releases once it settles, inside 8 s of the
  straight instead of the old 25.5 s fade.
  """
  lv = LowVisibility()
  # Turn from a stop. Model +0.078 vs steered +0.059. Lines gone.
  out = _step(lv, left=0.08, right=0.08, ystd=3.0, seconds=8.6,
              v_ego=_mph(12.0), measured_k=0.059, model_k=0.078, edges=NO_EDGES)
  assert not out.active and out.authority == 1.0

  # Straight at 30 mph, left line 0.9, right line gone, path settled.
  quiet = _step(lv, left=0.90, right=0.15, ystd=0.8, seconds=15.0,
                v_ego=_mph(30.0), measured_k=0.0008, model_k=-0.004, edges=NO_EDGES)
  assert not quiet.active and quiet.authority == 1.0

  # If the plan really is lost after the turn, it can arm, then lets go
  # once the path is confident again. Far under the old 25.5 s.
  unsure = LowVisibility()
  _step(unsure, left=0.90, right=0.15, ystd=3.0, seconds=2.0,
        measured_k=0.04, model_k=0.05, v_ego=_mph(15.0))  # still in the turn
  assert not unsure.active
  entered = _step(unsure, left=0.90, right=0.05, ystd=3.0,
                  seconds=STRAIGHT_HOLD_S + MODEL_ENTER_S + 0.2,
                  v_ego=_mph(30.0), chatter=CURV_JUMP, edges=NO_EDGES)
  assert entered.active
  released = _step(unsure, left=0.90, right=0.05, ystd=0.8,
                   seconds=CURV_WINDOW_S + MODEL_EXIT_S + 0.15,
                   v_ego=_mph(30.0), model_k=0.0, edges=NO_EDGES)
  assert not released.active
  assert STRAIGHT_HOLD_S + MODEL_ENTER_S + CURV_WINDOW_S + MODEL_EXIT_S < 8.0


def test_route172_1556_unmarked_stays_quiet_unless_the_path_is_unsure():
  """15:56:24, ~22 mph, both lane lines unseen about 88% of 150 s.

  drivingModelData in the qlog has laneLineMeta and desired curvature,
  not position.yStd or roadEdgeStds. Nothing recorded shows the model
  was unsure where to go, so missing lines alone do not latch.
  """
  lv = LowVisibility()
  out = _step(lv, left=0.05, right=0.04, ystd=0.6, seconds=5.0,
              v_ego=_mph(22.0), measured_k=0.002, model_k=0.002, edges=NO_EDGES)
  assert not out.active and not out.alert and out.authority == 1.0


def test_unmarked_confident_road_stays_quiet():
  """Gravel or a city street with no paint, and a model that knows the path."""
  gravel = LowVisibility()
  out = _step(gravel, left=0.02, right=0.03, ystd=0.45, seconds=5.0,
              edges=NO_EDGES, v_ego=_mph(28.0), model_k=0.0015, measured_k=0.001)
  assert not out.active and out.authority == 1.0

  city = LowVisibility()
  out = _step(city, left=0.04, right=0.05, ystd=0.55, seconds=5.0,
              edges=[0.12, 0.18], v_ego=_mph(25.0), model_k=-0.002)
  assert not out.active

  # Wide plan alone, or a small wobble, is not enough. One curb blocks it.
  wide = LowVisibility()
  out = _step(wide, left=0.0, right=0.0, ystd=4.0, seconds=2.0,
              edges=NO_EDGES, v_ego=_mph(30.0), model_k=0.001)
  assert not out.active
  wobble = LowVisibility()
  out = _step(wobble, left=0.0, right=0.0, ystd=4.0, seconds=2.0,
              edges=NO_EDGES, chatter=0.002, v_ego=_mph(30.0))
  assert not out.active
  curb = LowVisibility()
  out = _step(curb, left=0.0, right=0.0, ystd=4.0, seconds=2.0,
              edges=[0.10, 8.0], chatter=CURV_JUMP, v_ego=_mph(30.0))
  assert not out.active


def test_uncertain_path_with_no_edges_triggers():
  """High path std, jumping desired curvature, and no road edge. Lanes irrelevant."""
  lv = LowVisibility()
  early = _step(lv, left=0.95, right=0.92, ystd=3.2, seconds=0.25,
                edges=NO_EDGES, chatter=CURV_JUMP, v_ego=_mph(25.0))
  assert not early.active
  entered = _step(lv, left=0.0, right=0.0, ystd=3.2, seconds=0.45,
                  edges=NO_EDGES, chatter=CURV_JUMP, v_ego=_mph(25.0))
  assert entered.active and entered.model_poor and entered.alert


def test_roundabout_lost_lines_keep_steering_and_a_confident_path_releases():
  """Lines vanish mid-circle. Do not start. A latch from before can release on the ring."""
  # Within 40 m, still straight, path looks lost: the roundabout gate blocks
  # what would otherwise arm. Then the circle itself, lines gone, stays quiet
  # so authority (and steering) stays full.
  blocked = LowVisibility()
  approach = _step(blocked, left=0.85, right=0.80, ystd=3.5, seconds=2.0,
                   v_ego=_mph(25.0), measured_k=0.002, chatter=CURV_JUMP,
                   edges=NO_EDGES, near_roundabout=True)
  assert not approach.active and approach.authority == 1.0
  circle = _step(blocked, left=0.02, right=0.01, ystd=4.0, seconds=6.0,
                 v_ego=_mph(22.0), measured_k=0.05, chatter=CURV_JUMP,
                 edges=NO_EDGES, near_roundabout=True)
  assert not circle.active and not circle.alert and circle.authority == 1.0

  # Same approach with no roundabout does arm, so the gate is what held.
  open_road = LowVisibility()
  armed = _step(open_road, left=0.02, right=0.01, ystd=3.5, seconds=MODEL_ENTER_S + 0.2,
                v_ego=_mph(25.0), measured_k=0.002, chatter=CURV_JUMP,
                edges=NO_EDGES, near_roundabout=False)
  assert armed.active

  # Already active, then the circle, path confident: release without
  # waiting to finish the turn. Steering comes back on the ring.
  latched = LowVisibility()
  _step(latched, left=0.0, right=0.0, ystd=3.5, seconds=MODEL_ENTER_S + DT,
        v_ego=_mph(28.0), chatter=CURV_JUMP, edges=NO_EDGES)
  assert latched.active
  released = _step(latched, left=0.0, right=0.0, ystd=0.5,
                   seconds=CURV_WINDOW_S + MODEL_EXIT_S + 0.15,
                   v_ego=_mph(20.5), measured_k=0.05, model_k=0.04,
                   edges=NO_EDGES, near_roundabout=True)
  assert not released.active and not released.alert


# Route 0000017b--8ced80769d (Oct 10, 2026, ~4:19 PM CT, clear day): road
# camera integLines at 4 Hz, 100–150 s into the route. Shade (≈466) into sun
# (≈45). The path was confident (lateral std at 3 s ≈ 0.16 m) and the sun was
# 23° up. The old build latched camera-blind here and zeroed lateral for 7+ min.
ROUTE_17B_INTEG_4HZ = [
  161, 162, 169, 177, 171, 154, 147, 142, 134, 124, 120, 122, 119, 111, 112, 113, 110, 110, 116, 122,
  122, 122, 130, 138, 142, 149, 152, 156, 160, 169, 174, 180, 186, 175, 164, 160, 147, 147, 150, 155,
  152, 164, 171, 170, 170, 193, 215, 214, 213, 238, 268, 302, 344, 365, 401, 446, 427, 386, 445, 466,
  465, 460, 452, 440, 430, 418, 397, 376, 362, 341, 324, 319, 322, 319, 316, 325, 328, 328, 327, 322,
  290, 268, 240, 217, 212, 211, 208, 199, 192, 184, 174, 170, 172, 177, 177, 167, 157, 144, 133, 121,
  121, 124, 125, 120, 133, 138, 136, 134, 129, 125, 124, 127, 126, 127, 128, 125, 124, 116, 117, 112,
  102, 92, 99, 106, 105, 101, 108, 122, 135, 145, 139, 142, 153, 157, 165, 172, 170, 165, 168, 176,
  185, 182, 177, 188, 191, 190, 190, 187, 178, 172, 167, 149, 140, 135, 136, 133, 131, 125, 127, 127,
  127, 129, 129, 128, 129, 125, 123, 122, 120, 116, 113, 112, 107, 101, 96, 88, 82, 80, 78, 74,
  69, 65, 60, 55, 52, 49, 49, 48, 47, 45, 47, 47, 49, 50, 50, 50, 50, 50, 50, 46,
]
CONFIDENT_17B = dict(ystd=0.16, edges=[0.3, 0.3], v_ego=_mph(45.0))


def _play_17b(lv, sun_fn):
  outs = []
  for i, integ in enumerate(ROUTE_17B_INTEG_4HZ):
    outs.append(_step(lv, integ=integ, seconds=0.25, sun=sun_fn(i), **CONFIDENT_17B))
  return outs


def test_route17b_shade_to_sun_never_latches_or_reduces_lateral():
  lv = LowVisibility()
  outs = _play_17b(lv, lambda i: False)
  assert not any(o.active or o.alert or o.camera_blind for o in outs)
  assert all(o.authority == 1.0 for o in outs)


def test_route17b_forced_latch_releases_within_3s_and_never_zeroes_lateral():
  """Even if glare were corroborated (sun low ahead) at the shade→sun drop,
  a confident path releases within the 3 s backstop and lateral stays on."""
  lv = LowVisibility()
  drop = 80  # ≈ where 466 falls toward 210
  outs = _play_17b(lv, lambda i: i < drop + 4)
  latched = [i for i, o in enumerate(outs) if o.camera_blind]
  assert latched, "the corroborated drop should latch"
  first_unsun = drop + 4
  released = next(i for i in range(first_unsun, len(outs)) if not outs[i].active)
  assert (released - first_unsun) * 0.25 <= 3.0 + 0.25
  assert min(o.authority for o in outs) >= CAMERA_ONLY_AUTHORITY - 1e-9
  assert not any(o.active for o in outs[released:])


def test_route17b_no_ratio_releases_in_about_one_second():
  """Baseline under 280 (normal daylight) means clear: release after ~1 s."""
  lv = LowVisibility()
  _step(lv, integ=466, seconds=13.0, **CONFIDENT_17B)
  _step(lv, integ=150, seconds=0.5, sun=True, **CONFIDENT_17B)
  assert lv.active
  t = 0.0
  # Mid-band (ratio 0.45–0.70 would hold); once the bright sample ages out
  # the baseline drops under 280 and there is no ratio.
  while lv.active and t < 20.0:
    _step(lv, integ=45, seconds=DT, **dict(CONFIDENT_17B, ystd=1.5))
    t += DT
  assert not lv.active
  assert t <= 13.0


def test_model_uncertainty_corroborates_glare_without_sun():
  lv = LowVisibility()
  _step(lv, integ=731, seconds=2.0, ystd=1.5)
  out = _step(lv, integ=153, seconds=0.8, ystd=1.5)
  assert out.camera_blind and out.active
  assert out.authority >= AUTHORITY_FLOOR


def test_authority_tracks_uncertainty_and_never_zeroes():
  assert authority_target(False, False, False, 5.0) == 1.0
  assert authority_target(True, False, True, 0.2) == CAMERA_ONLY_AUTHORITY
  mid = authority_target(True, True, False, 2.6)
  assert AUTHORITY_FLOOR < mid < 1.0
  assert authority_target(True, True, False, 9.0) == AUTHORITY_FLOOR
  assert authority_target(True, True, True, float("nan")) >= AUTHORITY_FLOOR


def test_prolonged_degrade_raises_a_distinct_louder_alert():
  lv = LowVisibility()
  poor = dict(left=0.0, right=0.0, ystd=4.1, edges=NO_EDGES, chatter=CURV_JUMP, dt=0.05)
  _step(lv, seconds=PROLONGED_S - 1.0, **poor)
  assert lv.active and not lv.prolonged
  _step(lv, seconds=1.5, **poor)
  assert lv.prolonged
  assert log.OnroadEvent.EventName.lowVisibilityProlonged is not None
  root = Path(__file__).resolve().parents[4]
  events = (root / "selfdrive/selfdrived/events.py").read_text()
  block = events.split('_low_visibility_prolonged = getattr(EventName, "lowVisibilityProlonged"', 1)[1][:600]
  assert "ET.WARNING" in block and "AudibleAlert.none" not in block
  assert "ET.SOFT_DISABLE" not in block and "ET.IMMEDIATE_DISABLE" not in block
  selfdrived = (root / "selfdrive/selfdrived/selfdrived.py").read_text()
  assert "lowVisibilityProlonged" in selfdrived and "LOW_VIS_PROLONGED_S" in selfdrived
  # No new carState subscriber.
  assert selfdrived.count("sub_sock('carState'") <= 1
