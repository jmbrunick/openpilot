"""City vs highway Follow Distance blend."""
import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.follow_distance import (
  FOLLOW_CITY_ENTER_MPH,
  FOLLOW_DEFAULT,
  FOLLOW_HWY_ENTER_MPH,
  FOLLOW_HWY_INTENT_EGO_EXIT_MPH,
  FOLLOW_HWY_INTENT_EGO_MPH,
  FOLLOW_HWY_INTENT_EXIT_MPH,
  FOLLOW_HWY_INTENT_MPH,
  FOLLOW_OPEN_SLOW_FRAC,
  FOLLOW_SPLIT_MPH,
  FOLLOW_T_SLEW_CREEP_PER_S,
  FOLLOW_T_SLEW_OPEN_PER_S,
  FollowDistanceBlend,
  PARAM_FOLLOW,
  PARAM_FOLLOW_CITY,
  PARAM_FOLLOW_HWY,
  PARAM_FOLLOW_MIGRATED,
  follow_band_is_highway,
  follow_ego_highway_intent,
  follow_highway_intent,
  follow_open_a_ms2,
  migrate_follow_distance_params,
)
from openpilot.selfdrive.controls.lib.lead_approach import nap_t_follow


class FakeParams:
  def __init__(self, follow=FOLLOW_DEFAULT, city=None, hwy=None, migrated=False):
    self._ints = {PARAM_FOLLOW: int(follow)}
    if city is not None:
      self._ints[PARAM_FOLLOW_CITY] = int(city)
    if hwy is not None:
      self._ints[PARAM_FOLLOW_HWY] = int(hwy)
    self._bools = {PARAM_FOLLOW_MIGRATED: bool(migrated)}

  def get(self, key, return_default=False):
    if key in self._ints:
      return self._ints[key]
    if return_default:
      return FOLLOW_DEFAULT
    return None

  def put(self, key, value):
    self._ints[key] = int(value)

  def put_bool(self, key, value):
    self._bools[key] = bool(value)

  def get_bool(self, key):
    return bool(self._bools.get(key, False))


def test_migrate_copies_legacy_follow_distance_once():
  params = FakeParams(follow=2)
  city, hwy = migrate_follow_distance_params(params)
  assert city == 2 and hwy == 2
  assert params.get(PARAM_FOLLOW_CITY) == 2
  assert params.get(PARAM_FOLLOW_HWY) == 2
  assert params.get_bool(PARAM_FOLLOW_MIGRATED)
  params.put(PARAM_FOLLOW_CITY, 6)
  city, hwy = migrate_follow_distance_params(params)
  assert city == 6 and hwy == 2


def test_follow_band_hysteresis_around_50():
  v50 = FOLLOW_SPLIT_MPH * CV.MPH_TO_MS
  v48 = FOLLOW_CITY_ENTER_MPH * CV.MPH_TO_MS
  v52 = FOLLOW_HWY_ENTER_MPH * CV.MPH_TO_MS
  assert follow_band_is_highway(v50) is True
  assert follow_band_is_highway(v48 - 0.1) is False
  # Hold highway down through 49 mph; drop only below 48.
  v49 = 49.0 * CV.MPH_TO_MS
  assert follow_band_is_highway(v49, prev_highway=True) is True
  assert follow_band_is_highway(v48 - 0.05, prev_highway=True) is False
  # Hold city up through 51; rise only above 52.
  v51 = 51.0 * CV.MPH_TO_MS
  assert follow_band_is_highway(v51, prev_highway=False) is False
  assert follow_band_is_highway(v52 + 0.05, prev_highway=False) is True


def test_highway_intent_starts_hwy_fd_from_30_not_48_52():
  """Lead + long + MAX > 50: hwy FD once ego > ~30. MAX ≤ 50 stays city."""
  v30 = FOLLOW_HWY_INTENT_EGO_MPH * CV.MPH_TO_MS
  v28 = FOLLOW_HWY_INTENT_EGO_EXIT_MPH * CV.MPH_TO_MS
  v35 = 35.0 * CV.MPH_TO_MS
  v29 = 29.0 * CV.MPH_TO_MS
  v55 = 55.0 * CV.MPH_TO_MS
  v_max65 = 65.0 * CV.MPH_TO_MS
  v_max50 = FOLLOW_HWY_INTENT_MPH * CV.MPH_TO_MS
  v_max45 = 45.0 * CV.MPH_TO_MS
  kw = dict(engaged=True, has_lead=True)

  assert abs(FOLLOW_HWY_INTENT_MPH - 50.0) < 1e-9
  assert abs(FOLLOW_HWY_INTENT_EXIT_MPH - 48.0) < 1e-9
  assert abs(FOLLOW_HWY_INTENT_EGO_MPH - 30.0) < 1e-9
  assert abs(FOLLOW_HWY_INTENT_EGO_EXIT_MPH - 28.0) < 1e-9
  assert FOLLOW_HWY_INTENT_EGO_EXIT_MPH < FOLLOW_HWY_INTENT_EGO_MPH < FOLLOW_CITY_ENTER_MPH
  assert follow_highway_intent(v_max65) is True
  assert follow_highway_intent(v_max50) is False
  assert follow_highway_intent(v_max45) is False
  assert follow_highway_intent(49.0 * CV.MPH_TO_MS, prev_intent=True) is True
  assert follow_highway_intent(47.5 * CV.MPH_TO_MS, prev_intent=True) is False

  # Climbing out of town toward MAX 65: hwy from ~30, not 48/52.
  assert follow_band_is_highway(v35, v_cruise=v_max65, **kw) is True
  assert follow_band_is_highway(v30 + 0.05, v_cruise=v_max65, **kw) is True
  assert follow_band_is_highway(v29, v_cruise=v_max65, **kw) is False
  # Chatter at 30: hold hwy down through 29; drop only below 28.
  assert follow_ego_highway_intent(v29, prev_highway=True) is True
  assert follow_ego_highway_intent(v28 - 0.05, prev_highway=True) is False
  # MAX ≤ 50: city even at 55 (blend back toward city FD).
  assert follow_band_is_highway(v55, v_cruise=v_max45, **kw) is False
  assert follow_band_is_highway(v55, v_cruise=v_max50, **kw) is False
  # Unknown MAX keeps 48/52 (no silent city flip at 60).
  assert follow_band_is_highway(v55, engaged=True, has_lead=True) is True
  assert follow_band_is_highway(v35, engaged=True, has_lead=True) is False
  # No lead / not engaged: still 48/52 even with MAX 65.
  assert follow_band_is_highway(v35, v_cruise=v_max65, engaged=True, has_lead=False) is False
  assert follow_band_is_highway(v55, v_cruise=v_max65, engaged=False, has_lead=True) is True


def test_blend_creeps_to_hwy_from_30_when_max_is_highway_intent():
  b = FollowDistanceBlend()
  b.city, b.hwy = 6, 2
  b.highway = False
  b.t_follow = nap_t_follow(6)
  v_ego = 35.0 * CV.MPH_TO_MS
  v_max = 65.0 * CV.MPH_TO_MS
  dt = 0.05
  t_hwy = nap_t_follow(2)
  t, dist, extra = b.update(
    v_ego, dt, engaged=True, has_lead=True, v_lead=v_ego, v_cruise=v_max,
  )
  assert dist == 2
  assert extra is None
  assert t < nap_t_follow(6)
  times = [t]
  for _ in range(int(20.0 / dt)):
    t, dist, extra = b.update(
      v_ego, dt, engaged=True, has_lead=True, v_lead=v_ego, v_cruise=v_max,
    )
    times.append(t)
    assert extra is None
    if t <= t_hwy + 1e-6:
      break
  assert dist == 2
  assert times[-1] == pytest.approx(t_hwy, abs=1e-6)
  steps = [s - t for s, t in zip(times, times[1:], strict=False)]
  assert max(steps) <= FOLLOW_T_SLEW_CREEP_PER_S * dt + 1e-9


def test_blend_returns_to_city_when_max_drops_below_50():
  b = FollowDistanceBlend()
  b.city, b.hwy = 6, 2
  b.highway = True
  b.intent = True
  b.t_follow = nap_t_follow(2)
  v_ego = 55.0 * CV.MPH_TO_MS
  t_city = nap_t_follow(6)
  t, dist, extra = b.update(
    v_ego, 0.05, engaged=True, has_lead=True, v_lead=v_ego,
    v_cruise=45.0 * CV.MPH_TO_MS,
  )
  assert dist == 6
  assert t > nap_t_follow(2)
  assert extra is not None and extra < 0.0
  # Keep slewing toward city; do not snap.
  t2, dist2, _ = b.update(
    v_ego, 0.05, engaged=True, has_lead=True, v_lead=v_ego,
    v_cruise=45.0 * CV.MPH_TO_MS,
  )
  assert dist2 == 6
  assert t2 > t
  assert t2 < t_city


def test_blend_opens_gradually_to_city_then_holds():
  b = FollowDistanceBlend()
  b.city, b.hwy = 6, 2
  b.highway = True
  b.t_follow = nap_t_follow(2)
  v_lead = 45.0 * CV.MPH_TO_MS
  v_ego = v_lead
  dt = 0.05
  t_city = nap_t_follow(6)
  extras = []
  times = [b.t_follow]
  for _ in range(int(12.0 / dt)):
    t, dist, extra = b.update(v_ego, dt, engaged=True, has_lead=True, v_lead=v_lead)
    times.append(t)
    extras.append(extra)
    if extra is not None:
      v_ego = max(0.0, v_ego + extra * dt)
    if t >= t_city - 1e-6:
      break
  assert dist == 6
  assert times[0] == pytest.approx(nap_t_follow(2))
  assert times[-1] == pytest.approx(t_city, abs=1e-6)
  assert any(a is not None and a < 0.0 for a in extras)
  assert max(t - s for s, t in zip(times, times[1:], strict=False)) <= FOLLOW_T_SLEW_OPEN_PER_S * dt + 1e-9
  t, dist, extra = b.update(v_lead, dt, engaged=True, has_lead=True, v_lead=v_lead)
  assert t == pytest.approx(t_city)
  assert extra is None


def test_blend_creeps_toward_hwy_without_a_step():
  b = FollowDistanceBlend()
  b.city, b.hwy = 6, 2
  b.highway = False
  b.t_follow = nap_t_follow(6)
  v_ego = 60.0 * CV.MPH_TO_MS
  v_lead = v_ego
  dt = 0.05
  t_hwy = nap_t_follow(2)
  times = [b.t_follow]
  for _ in range(int(20.0 / dt)):
    t, dist, extra = b.update(v_ego, dt, engaged=True, has_lead=True, v_lead=v_lead)
    times.append(t)
    assert extra is None
    if t <= t_hwy + 1e-6:
      break
  assert dist == 2
  assert times[-1] == pytest.approx(t_hwy, abs=1e-6)
  steps = [s - t for s, t in zip(times, times[1:], strict=False)]
  assert max(steps) <= FOLLOW_T_SLEW_CREEP_PER_S * dt + 1e-9
  assert min(steps) >= 0.0 or times[-1] == pytest.approx(t_hwy)


def test_no_lead_or_not_engaged_snaps_to_band():
  b = FollowDistanceBlend()
  b.city, b.hwy = 7, 1
  v_city = 40.0 * CV.MPH_TO_MS
  t, dist, extra = b.update(v_city, 0.05, engaged=True, has_lead=False)
  assert dist == 7
  assert t == pytest.approx(nap_t_follow(7))
  assert extra is None
  v_hwy = 65.0 * CV.MPH_TO_MS
  t, dist, extra = b.update(v_hwy, 0.05, engaged=False, has_lead=True, v_lead=v_hwy)
  assert dist == 1
  assert t == pytest.approx(nap_t_follow(1))


def test_persist_steps_city_or_hwy_band_by_speed():
  from openpilot.selfdrive.controls.lib.follow_stalk import persist_follow_distance

  params = FakeParams(follow=4, city=4, hwy=4, migrated=True)
  persist_follow_distance(params, closer=True, v_ego=40.0 * CV.MPH_TO_MS)
  assert params.get(PARAM_FOLLOW_CITY) == 3
  assert params.get(PARAM_FOLLOW_HWY) == 4
  assert params.get(PARAM_FOLLOW) == 3
  persist_follow_distance(params, closer=False, v_ego=65.0 * CV.MPH_TO_MS)
  assert params.get(PARAM_FOLLOW_CITY) == 3
  assert params.get(PARAM_FOLLOW_HWY) == 5
  assert params.get(PARAM_FOLLOW) == 5
  # Highway intent at 35 mph with MAX 65 + lead writes hwy, not city.
  persist_follow_distance(
    params, closer=False, v_ego=35.0 * CV.MPH_TO_MS,
    v_cruise=65.0 * CV.MPH_TO_MS, has_lead=True, engaged=True,
  )
  assert params.get(PARAM_FOLLOW_CITY) == 3
  assert params.get(PARAM_FOLLOW_HWY) == 6


def test_invalid_setpoints_do_not_blend():
  params = FakeParams(follow=0, city=0, hwy=8, migrated=True)
  b = FollowDistanceBlend()
  b.read_setpoints(params)
  t, dist, extra = b.update(25.0, 0.05, engaged=True, has_lead=True, v_lead=25.0)
  assert dist is None
  assert t is None
  assert extra is None


def test_open_a_is_slightly_slower_than_lead():
  v_lead = 20.0
  v_ego = v_lead * (1.0 + FOLLOW_OPEN_SLOW_FRAC)
  a = follow_open_a_ms2(v_ego, v_lead)
  assert a is not None and a < 0.0
  assert follow_open_a_ms2(v_lead * (1.0 - FOLLOW_OPEN_SLOW_FRAC), v_lead) is None


def test_planner_and_ui_wire_city_hwy_follow():
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  planner = (root / "selfdrive/controls/lib/longitudinal_planner.py").read_text()
  tici = (root / "selfdrive/ui/layouts/settings/driving_mannerisms.py").read_text()
  mici = (root / "selfdrive/ui/mici/layouts/settings/driving_mannerisms.py").read_text()
  keys = (root / "common/params_keys.h").read_text()
  content = (root / "selfdrive/ui/layouts/settings/nap_content.py").read_text()
  nap_params = (root / "opendbc_repo/opendbc/car/tesla/preap/nap_params.py").read_text()
  assert "FollowDistanceBlend" in planner
  assert "v_cruise=v_hud_ms" in planner
  assert "NAPFollowDistanceCity" in keys
  assert "NAPFollowDistanceHwy" in keys
  assert "FOLLOW_DISTANCE_CITY" in nap_params
  assert "City Follow Distance" in tici
  assert "Highway Follow Distance" in tici
  assert "city follow distance" in mici
  assert "highway follow distance" in mici
  assert "Not Hypermile" in content
  assert "hypermile" not in planner.lower()
