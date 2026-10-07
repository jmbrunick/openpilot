"""Low-visibility lateral back-off: confidence hysteresis, ramp, exposure, sun."""

from pathlib import Path

from cereal import log
from openpilot.selfdrive.controls.lib.lat_low_visibility import (
  AUTHORITY_DOWN_PER_S,
  AUTHORITY_UP_PER_S,
  EXPOSURE_COLLAPSE_RATIO,
  LANE_CONF_ENTER,
  LANE_CONF_EXIT,
  MODEL_ENTER_S,
  MODEL_EXIT_S,
  PARAM_LOW_VIS_BACKOFF,
  PATH_Y_STD_ENTER_M,
  PATH_Y_STD_EXIT_M,
  LowVisibility,
  edge_confidence,
  fade_curvature,
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
          sun=False, enabled=True, dt=DT, edges=None):
  n = max(1, int(round(seconds / dt)))
  out = None
  if edges is None:
    edges = [0.12, 0.12]
  for _ in range(n):
    out = lv.update(
      enabled=enabled,
      lane_probs=[0.5, left, right, 0.5],
      edge_stds=edges,
      path_t=[0.0, 1.5, 3.0, 6.0],
      path_y_std=[0.2, 0.3, ystd, ystd],
      integ_lines=integ,
      sun_ahead=sun,
      dt=dt,
    )
  return out


def test_side_confidence_and_path_std_match_the_incident_numbers():
  # Right lane 0.12 is below 0.30; left 0.56 is not. Either side is enough.
  left, right = side_confidences([0.4, 0.56, 0.12, 0.3], [0.1, 0.1])
  assert left > LANE_CONF_ENTER
  assert right < LANE_CONF_ENTER
  y = path_y_std_at([0.0, 2.0, 4.0], [0.4, 4.1, 5.0], 3.0)
  assert abs(y - (4.1 + 0.5 * (5.0 - 4.1))) < 1e-6
  assert model_is_poor(left, right, 4.1)
  assert not model_is_poor(0.95, 0.92, 0.4)
  assert model_is_clear(0.95, 0.92, 0.4)
  assert not model_is_clear(0.40, 0.90, 0.4)  # between the bars
  # Path std cannot hold a clear exit once both sides are at the bar.
  assert model_is_clear(0.90, 0.90, 5.0)
  exit_left, exit_right = side_confidences_exit([0.4, 0.90, 0.85, 0.3], [8.0, 8.0])
  assert exit_left >= LANE_CONF_EXIT and exit_right >= LANE_CONF_EXIT
  # Entry still uses the worse signal, so a wide edge keeps the side poor.
  enter_left, enter_right = side_confidences([0.4, 0.90, 0.85, 0.3], [8.0, 8.0])
  assert enter_left < LANE_CONF_ENTER and enter_right < LANE_CONF_ENTER
  assert edge_confidence(0.1) > 0.8
  assert edge_confidence(1.5) < LANE_CONF_ENTER


def test_hysteresis_ignores_one_frame_and_the_middle_band():
  lv = LowVisibility()
  out = _step(lv, left=0.97, right=0.92, ystd=0.4, seconds=3.0)
  assert not out.active and out.authority == 1.0

  # One bad model frame must not latch.
  out = _step(lv, left=0.56, right=0.08, ystd=4.1, seconds=DT)
  assert not out.active

  # Held poor: right 0.06–0.29, left 0.56, y std 4.1 m.
  out = _step(lv, left=0.56, right=0.18, ystd=4.1, seconds=MODEL_ENTER_S + DT)
  assert out.active and out.alert and out.model_poor
  assert out.authority < 1.0

  # Middle band: the better of lane and edge on the weak side is 0.40.
  # A sharp road edge would count as seen; a wide one must not release.
  held = _step(lv, left=0.90, right=0.40, ystd=1.5, seconds=2.0, edges=[8.0, 8.0])
  assert held.active

  # Exit needs the high bar for MODEL_EXIT_S. Short of that, stay latched.
  almost = _step(lv, left=0.95, right=0.95, ystd=0.4, seconds=MODEL_EXIT_S * 0.5)
  assert almost.active
  cleared = _step(lv, left=0.95, right=0.95, ystd=0.4, seconds=MODEL_EXIT_S)
  assert not cleared.active and not cleared.alert


def test_backoff_ramps_down_and_recovers_without_a_step():
  lv = LowVisibility()
  # Blind edges, so the weak lane stays the better signal and the latch holds.
  blind = [8.0, 8.0]
  _step(lv, left=0.56, right=0.10, ystd=4.1, seconds=MODEL_ENTER_S, dt=0.01, edges=blind)
  assert lv.active
  start = lv.authority
  mid = _step(lv, left=0.56, right=0.10, ystd=4.1, seconds=0.50, dt=0.01, edges=blind)
  assert mid.authority < start
  assert abs((start - mid.authority) - 0.50 * AUTHORITY_DOWN_PER_S) < 0.02
  floor = _step(lv, left=0.56, right=0.10, ystd=4.1, seconds=1.0, dt=0.01, edges=blind)
  assert floor.authority == 0.0
  # Still engaged-shaped: authority 0 fades curvature fully onto the wheel.
  assert fade_curvature(0.02, 0.0, floor.authority) == 0.0
  assert fade_curvature(0.02, 0.0, 1.0) == 0.02

  _step(lv, left=0.95, right=0.95, ystd=0.4, seconds=MODEL_EXIT_S, dt=0.01)
  assert not lv.active
  rising = lv.authority
  later = _step(lv, left=0.95, right=0.95, ystd=0.4, seconds=0.50, dt=0.01)
  assert later.authority > rising
  assert abs((later.authority - rising) - 0.50 * AUTHORITY_UP_PER_S) < 0.02


def test_paved_drive_does_not_back_off_and_toggle_off_is_identity():
  lv = LowVisibility()
  out = _step(lv, left=0.97, right=0.92, ystd=0.4, seconds=30.0)
  assert not out.active and out.authority == 1.0 and not out.alert

  lv2 = LowVisibility()
  out = _step(lv2, left=0.10, right=0.10, ystd=5.0, seconds=2.0, enabled=False)
  assert out.authority == 1.0 and not out.alert and not out.active


def test_three_incident_episodes_each_latch():
  """Synthetic modelV2-rate sequence: three right-pull windows, then clear."""
  lv = LowVisibility()
  _step(lv, left=0.97, right=0.92, ystd=0.4, seconds=2.0)
  episodes = []
  for _ in range(3):
    # Full clear so each episode must enter on its own.
    _step(lv, left=0.97, right=0.92, ystd=0.4, seconds=MODEL_EXIT_S + 0.3)
    assert not lv.active
    out = _step(lv, left=0.56, right=0.12, ystd=4.1, seconds=MODEL_ENTER_S + 0.2)
    episodes.append(out)
    assert out.active and out.model_poor and out.authority < 1.0
  assert len(episodes) == 3
  # A one-frame healthy spike between bad frames must not drop the latch,
  # and a short healthy gap shorter than the exit hold must not either.
  _step(lv, left=0.97, right=0.92, ystd=0.4, seconds=DT)
  assert lv.active
  _step(lv, left=0.56, right=0.20, ystd=4.1, seconds=0.2)
  _step(lv, left=0.97, right=0.92, ystd=0.4, seconds=MODEL_EXIT_S * 0.4)
  assert lv.active


def test_exposure_collapse_and_single_frame_and_sun_heads_up():
  lv = LowVisibility()
  # Bright baseline (731 lines), then the incident drop to 153.
  _step(lv, integ=731, seconds=2.0)
  assert not lv.active
  one = _step(lv, integ=153, seconds=DT)
  assert not one.camera_blind and not one.active

  dropped = _step(lv, integ=153, seconds=0.8)
  assert dropped.camera_blind and dropped.active and dropped.alert
  assert dropped.authority < 1.0
  # 153/731 is well under the 0.45 collapse ratio.
  assert 153 / 731 < EXPOSURE_COLLAPSE_RATIO

  # Recover.
  back = _step(lv, integ=700, seconds=2.0)
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
  accel = controls.split("actuators.accel = float(self.LoC.update(", 1)[1].split("Steering PID", 1)[0]
  assert "low_vis" not in accel
  assert "NAPLatRefOffset" not in controls.split("fade_curvature", 1)[1][:400]


def test_exit_uses_the_better_side_and_ignores_path_std():
  """Lanes 0.8–0.95 clear the latch even when edges and path std stay wide."""
  lv = LowVisibility()
  entered = _step(lv, left=0.56, right=0.12, ystd=4.1, seconds=MODEL_ENTER_S + DT)
  assert entered.active and entered.model_poor

  def _held(*, left, right, ystd, edges, seconds):
    n = max(1, int(round(seconds / DT)))
    out = None
    for _ in range(n):
      out = lv.update(
        enabled=True,
        lane_probs=[0.5, left, right, 0.5],
        edge_stds=edges,
        path_t=[0.0, 1.5, 3.0, 6.0],
        path_y_std=[0.2, 0.3, ystd, ystd],
        integ_lines=700,
        sun_ahead=False,
        dt=DT,
      )
    return out

  # A side still between the bars stays latched, even with a tight path.
  held = _held(left=0.90, right=0.40, ystd=0.3, edges=[8.0, 8.0], seconds=MODEL_EXIT_S + 0.4)
  assert held.active

  # Healthy lane lines, useless road edges, path std still 4 m: exit.
  cleared = _held(left=0.92, right=0.85, ystd=4.0, edges=[8.0, 8.0], seconds=MODEL_EXIT_S)
  assert not cleared.active and not cleared.alert and not cleared.model_poor
