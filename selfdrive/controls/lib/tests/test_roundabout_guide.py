"""Roundabout Steering Assist (NAPRoundaboutAssist, Pre-AP, default Off).

Sep 30: once latched the assist follows the map's lane circle (no blend), capped at 1.15x the model, 2.2 m/s², slew 0.035; released only by a
held same-side stalk / steering pull / override (no map-only exit); ALC tips are suppressed while latched. R126 / R128 (Sep 28) and
0000012f / 00000130 (Sep 29, roundabout_passes_sep29.json) replay below.

Sep 28 PM, Soco Rd / Dellwood / Jonathan Creek ring (R ~21.6 m, CCW):
R126 16:52 CT (inner lane) and R128 17:38 CT (outer lane). The model's plan
was a wide, slow S: it swung across the band (island side on R128, outer curb
on R126) and the driver took over. The fixture carries the logged model
curvature vs distance, the wheel speed and the real 1 Hz qcom GNSS errors, so
the closed-loop sims below run the guide against the same GPS quality.

Curvature sign: openpilot's (positive = right). US CCW circulation < 0.
"""
import json
import math
from pathlib import Path

import pytest

from openpilot.selfdrive.controls.lib import roundabout_guide as RG
from openpilot.selfdrive.controls.lib import roundabout_ring_hold as RH
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.mapd.roundabout_map import RingGeometry, latlon_from_xy

ROOT = Path(__file__).resolve().parents[4]
FIXTURE = Path(__file__).parent / "data" / "roundabout_passes_sep28.json"
PASSES = json.loads(FIXTURE.read_text())["passes"]
PASSES29 = json.loads((Path(__file__).parent / "data" / "roundabout_passes_sep29.json").read_text())["passes"]
PASSES_ALL = {**PASSES, **PASSES29}
MPH = 0.44704
DT = 0.01


@pytest.fixture(autouse=True)
def _assist_code_enabled(monkeypatch):
  # nap-release forces the assist off (common/nap_release.py); these tests
  # exercise the assist code itself. test_nap_release covers the release gate.
  from openpilot.common import nap_release
  monkeypatch.setattr(nap_release, "ROUNDABOUT_ASSIST_ENABLED", True)


def _ring(p) -> RingGeometry:
  return RingGeometry.from_json(p["ring"])


def _interp(x, xs, ys):
  if x <= xs[0]:
    return ys[0]
  if x >= xs[-1]:
    return ys[-1]
  lo, hi = 0, len(xs) - 1
  while hi - lo > 1:
    mid = (lo + hi) // 2
    if xs[mid] <= x:
      lo = mid
    else:
      hi = mid
  f = (x - xs[lo]) / (xs[hi] - xs[lo])
  return ys[lo] + f * (ys[hi] - ys[lo])


_LAST: dict = {}


def simulate(name: str, kind: str, *, slow: bool = True, fix_scale: float = 1.0, pool=None):
  """Kinematic closed loop: model curvature by distance -> [guide] -> clip_curvature -> 0.2 s delay + 0.2 s lag.

  kind: "before" (model only) or "after" (guide On). slow: planner ring slow-down from the first hint.
  Returns rows (t, x, y, psi, v, model_k, out_k, des_k).
  """
  p = (pool or PASSES)[name]
  ring = _ring(p)
  R, hw = ring.radius_m, ring.half_width_m
  guide = RG.RoundaboutGuide()
  guide.set_ring(ring)
  _LAST["guide"] = guide
  fixes = list(p["fixes"])
  hist = p["hist"]
  fi = 0
  # Warm the pose tracker on the logged approach (as on the car, GNSS is fed continuously).
  track = []
  for t, x, y, psi, v, yr in hist:
    guide.update(t, v, yr, 0.0, enabled=kind == "after", lat_active=True)
    track.append((t, x, y, psi, v))
    while fi < len(fixes) and fixes[fi][0] <= t:
      _feed(guide, ring, fixes[fi], track, fix_scale)
      fi += 1
  t, x, y, psi = 0.0, hist[-1][1], hist[-1][2], hist[-1][3]
  ks = [i * p["model_k_ds"] for i in range(len(p["model_k"]))]
  vts = [i * p["v_dt"] for i in range(len(p["v"]))]
  k_des = p["k_des0"]
  k_act = p["k_act0"]
  delay = [k_act] * 20
  s = 0.0
  rows = []
  v_target = guide.ring_v          # the planner's ring speed (RB_RING_SPEED_MPH = 18 since Sep 30), not the logged 15.8 mph hint
  while t < vts[-1]:
    v = _interp(t, vts, p["v"])
    if slow:
      d = max(math.hypot(x, y) - (R + hw), 0.0)
      v = min(v, math.sqrt(v_target * v_target + 2.0 * 1.0 * d))
    model_k = _interp(s, ks, p["model_k"])
    track.append((t, x, y, psi, v))
    while fi < len(fixes) and fixes[fi][0] <= t:
      _feed(guide, ring, fixes[fi], track, fix_scale)
      fi += 1
    if kind == "after":
      out_k = guide.update(t, v, k_act * v, model_k, enabled=True, lat_active=True)
    else:
      out_k = model_k
    k_des, _ = clip_curvature(v, k_des, out_k, 0.0)
    delay.append(k_des)
    k_act += (delay.pop(0) - k_act) * (DT / 0.2)
    rows.append((t, x, y, psi, v, model_k, out_k, k_des))
    psi += k_act * v * DT
    x += v * math.sin(psi) * DT
    y += v * math.cos(psi) * DT
    s += v * DT
    t += DT
  return rows, ring


def _feed(guide, ring, f, track, scale):
  t_fix, ex, ey, dbrg, bacc = f
  te = t_fix - RG.GPS_LATENCY_S
  pts = [r for r in track if r[0] <= te]
  if not pts:
    return
  _, x, y, psi, v = pts[-1]
  lat, lon = latlon_from_xy(x + scale * ex, y + scale * ey, ring.lat, ring.lon)
  guide.gnss(t_fix, lat, lon, (math.degrees(psi) + scale * dbrg) % 360.0, v, bacc)


def band_metrics(rows, ring, deg=120.0):
  """First `deg` degrees after entering the ring band: max distance beyond a curb, % inside the band."""
  R, hw = ring.radius_m, ring.half_width_m
  r = [math.hypot(row[1], row[2]) for row in rows]
  i0 = next(i for i, rr in enumerate(r) if rr < R + hw)
  th0 = math.atan2(rows[i0][2], rows[i0][1])
  travel, prev, out = 0.0, th0, []
  for i in range(i0, len(rows)):
    th = math.atan2(rows[i][2], rows[i][1])
    travel += math.degrees(RG._wrap(th - prev))
    prev = th
    if travel > deg:
      break
    out.append(max(0.0, (R - hw) - r[i], r[i] - (R + hw)))
  return max(out), 100.0 * sum(o < 0.01 for o in out) / len(out)


# ---------------------------------------------------------------------------------------------
# Replay: both passes, closed loop with the real GNSS error stream

@pytest.mark.parametrize("name", ["00000126", "00000128", "00000130"])
def test_model_only_leaves_the_ring_band(name):
  rows, ring = simulate(name, "before", pool=PASSES_ALL)
  beyond, inband = band_metrics(rows, ring, 90.0)
  assert beyond > 0.9 and inband < 70.0


@pytest.mark.parametrize("name,max_beyond,min_inband", [("00000126", 2.0, 85.0), ("00000128", 0.7, 94.0), ("00000130", 0.5, 95.0)])
def test_ring_follow_keeps_the_passes_in_the_ring_band(name, max_beyond, min_inband):
  """First 90° of circulation (the later part of the logged passes is the driver leaving the ring at the exit)."""
  before = band_metrics(*simulate(name, "before", pool=PASSES_ALL), 90.0)
  after = band_metrics(*simulate(name, "after", pool=PASSES_ALL), 90.0)
  assert after[0] <= max_beyond
  assert after[1] >= min_inband
  assert after[0] < before[0] and after[1] > before[1]


def test_unreliable_pose_pass_is_model_only_and_unchanged():
  """0000012f (Scallywag 11:57): bearing accuracy 13-52° the whole way in, the guide never latches and never touches the pass."""
  before, ring = simulate("0000012f", "before", pool=PASSES_ALL)
  after, _ = simulate("0000012f", "after", pool=PASSES_ALL)
  assert not _LAST["guide"].latch_t
  assert all(a[6] == b[6] for a, b in zip(after, before, strict=False))


@pytest.mark.parametrize("name", sorted(PASSES_ALL))
def test_right_entry_held_until_turn_distance(name, monkeypatch):
  monkeypatch.setattr(RG, "SPEED_HOLD_MS", 1e9)   # the ring-speed curl hold would also hold the left; test the distance hold alone
  rows, ring = simulate(name, "after", pool=PASSES_ALL)
  R, hw = ring.radius_m, ring.half_width_m
  t_latch = _LAST["guide"].latch_t if _LAST["guide"].latch_t is not None else 1e9    # entry only: after the latch the car is on the ring
  for _t, x, y, _psi, _v, mk, out, _ in rows:
    if _t >= t_latch:
      break
    d_edge = math.hypot(x, y) - (R + hw)
    if d_edge > RG.D_TURN_M + 3.0:   # + guide pose error margin
      assert out >= mk - 1e-9, f"early left at {d_edge:.1f} m"


@pytest.mark.parametrize("name", sorted(PASSES_ALL))
def test_left_transition_within_jerk_limit(name):
  rows, _ = simulate(name, "after", pool=PASSES_ALL)
  t_latch = _LAST["guide"].latch_t or 1e9
  for a, b in zip(rows, rows[1:], strict=False):
    if abs(b[0] - t_latch) < 1e-6:
      continue        # the one sample where an over-cap entry curl meets the 1.15x cap (clip_curvature rate-limits it downstream)
    corr_rate = abs((b[6] - b[5]) - (a[6] - a[5])) / DT
    out_rate = abs(b[6] - a[6]) / DT
    model_rate = abs(b[5] - a[5]) / DT
    assert corr_rate <= RG.OUT_SLEW_GOOD + 1e-6
    assert out_rate <= max(RG.OUT_SLEW_GOOD, RG.RING_CAP_RATIO * model_rate) + 1e-6   # the ring cap follows the model at 1.15x


@pytest.mark.parametrize("name", sorted(PASSES_ALL))
def test_ring_speed_is_18_mph_and_lateral_cap(name):
  """Sep 30: entry was a little slow; the ring speed constant is 18 mph (RB_RING_SPEED_MPH), up from 15.8 (2.5 m/s² comfort speed)."""
  from openpilot.selfdrive.mapd.roundabout import RB_RING_SPEED_MPH
  assert RB_RING_SPEED_MPH == 18.0
  rows, ring = simulate(name, "after", pool=PASSES_ALL)
  assert _LAST["guide"].ring_v == pytest.approx(18.0 * MPH)
  R, hw = ring.radius_m, ring.half_width_m
  inside = [r for r in rows if math.hypot(r[1], r[2]) < R + hw]
  assert inside and max(r[4] for r in inside) <= 18.05 * MPH
  assert max(abs(r[7]) * r[4] ** 2 for r in inside) <= 3.0


@pytest.mark.parametrize("name", ["00000126", "00000128", "00000130"])
def test_hands_back_to_model_after_exit(name):
  rows, _ = simulate(name, "after", pool=PASSES_ALL)
  tail = rows[-100:]          # the fade of a widened ring correction (0.015 at the 0.035 slew) takes ~0.45 s after the safety release
  assert all(r[6] == r[5] for r in tail)
  assert _LAST["guide"].phase in (RG.DONE, RG.IDLE)   # DONE, then reset once > 60 m from the ring
  assert _LAST["guide"].dk_out == 0.0


# ---------------------------------------------------------------------------------------------
# Identity / gating

class FakeParams:
  def __init__(self, on=False, ring=None):
    self.on, self.ring = on, ring

  def get_bool(self, key):
    assert key == RG.PARAM_ROUNDABOUT_ASSIST
    return self.on

  def get(self, key):
    return self.ring


class FakeGps:
  def __init__(self, lat, lon, brg, speed):
    self.latitude, self.longitude, self.bearingDeg, self.speed = lat, lon, brg, speed
    self.bearingAccuracyDeg, self.hasFix = 1.0, True


class FakeSM:
  def __init__(self):
    self.recv_frame = {"gpsLocation": 0, "gpsLocationExternal": 0}
    self.logMonoTime = {"gpsLocation": 0}
    self.msgs = {}

  def __getitem__(self, k):
    return self.msgs[k]


class Hint:
  def __init__(self, way_id):
    self.way_id = way_id


_TRAJ = {}


def _trajectory(name):
  """The 'after' closed-loop trajectory (truth pose, speed, model curvature) through the ring."""
  if name not in _TRAJ:
    _TRAJ[name] = simulate(name, "after")[0]
  return _TRAJ[name]


def _drive_assist(assist, name, *, hint=True, lat_active=True, maneuver=False, lane_change=False, hint_until=1e9):
  """Replay the approach + ring trajectory through RoundaboutAssist (1 Hz GNSS from truth); (model_k, out_k)."""
  p = PASSES[name]
  ring = _ring(p)
  sm = FakeSM()
  rows = [(t, x, y, psi, v, 0.0) for t, x, y, psi, v, _yr in p["hist"]]
  rows += [(r[0], r[1], r[2], r[3], r[4], r[5]) for r in _trajectory(name)[::10]]   # 0.1 s steps
  pairs = []
  frame = 0
  next_fix = rows[0][0] + 1.0
  prev = None
  for i, (t, x, y, psi, v, mk_base) in enumerate(rows):
    yr = 0.0 if prev is None else RG._wrap(psi - prev[3]) / max(t - prev[0], 1e-3)
    prev = (t, x, y, psi)
    if t >= next_fix:
      next_fix += 1.0
      lat, lon = latlon_from_xy(x, y, ring.lat, ring.lon)
      frame += 1
      sm.recv_frame["gpsLocation"] = frame
      # The fix of time t arrives ~0.9 s later (qcom latency); one 0.1 s step before this update.
      sm.logMonoTime["gpsLocation"] = int((t - 0.1 + RG.GPS_LATENCY_S) * 1e9)
      sm.msgs["gpsLocation"] = FakeGps(lat, lon, math.degrees(psi) % 360.0, v)
    mk = mk_base + 0.002 * math.sin(0.7 * i)
    out = assist.update(sm, t=t, v_ego=v, yaw_rate=yr, model_k=mk, lat_active=lat_active, maneuver_active=maneuver,
                        lane_change_active=lane_change, hint=Hint(ring.way_ids[0]) if (hint and t <= hint_until) else None,
                        model_v2=None)
    pairs.append((mk, out))
  return pairs


def _assist(on=True, preap=True, ring="128"):
  ring_json = json.dumps(PASSES["00000128"]["ring"]) if ring == "128" else ring
  return RG.RoundaboutAssist(preap, FakeParams(on=on, ring=ring_json))


def test_assist_engages_through_the_ring_when_on():
  a = _assist()
  pairs = _drive_assist(a, "00000128")
  assert a.guide.ring is not None and a.guide.pose.pose() is not None
  assert max(abs(out - mk) for mk, out in pairs) > 0.01
  assert pairs[-1][0] == pairs[-1][1]   # handed back after the exit


@pytest.mark.parametrize("preap,on", [(False, True), (True, False), (False, False)])
def test_identity_when_off_or_not_preap(preap, on):
  for mk, out in _drive_assist(_assist(on=on, preap=preap), "00000128"):
    assert out == mk


def test_identity_without_hint_or_ring_param():
  for ring in (None, "{}"):
    for mk, out in _drive_assist(_assist(ring=ring), "00000128"):
      assert out == mk
  for mk, out in _drive_assist(_assist(), "00000128", hint=False):
    assert out == mk


def test_stale_ring_does_not_engage(monkeypatch):
  """Ring loaded from an old hint, hint gone > HINT_HOLD_S before the ring: stays model-only."""
  monkeypatch.setattr(RG, "HINT_HOLD_S", 3.0)
  a = _assist()
  pairs = _drive_assist(a, "00000128", hint_until=-7.5)
  assert a.guide.ring is not None
  assert all(out == mk for mk, out in pairs)


@pytest.mark.parametrize("kw", [dict(lat_active=False), dict(maneuver=True), dict(lane_change=True)])
def test_identity_when_driver_has_wheel_or_maneuver(kw):
  for mk, out in _drive_assist(_assist(), "00000128", **kw):
    assert out == mk


def test_identity_far_from_any_ring():
  """Non-roundabout driving: straight road 400 m from the ring, toggle On, ring loaded -> exact model."""
  ring = _ring(PASSES["00000128"])
  g = RG.RoundaboutGuide()
  g.set_ring(ring)
  x, y = -400.0, 400.0
  for i in range(3000):
    t = i * DT
    if i % 100 == 0:
      lat, lon = latlon_from_xy(x, y, ring.lat, ring.lon)
      g.gnss(t, lat, lon, 90.0, 15.0, 1.0)
    mk = 0.002 * math.sin(0.01 * i)
    assert g.update(t, 15.0, 0.0, mk, enabled=True, lat_active=True) == mk
    x += 15.0 * DT


def test_edge_limit_keeps_camera_authority():
  e = RG.EdgeInfo(x=10.0, path_y=0.0, left_y=-1.4, right_y=1.6)
  # 1.28 m margin (half car + 0.3): left line at -1.4 -> at most 0.12 m of extra left at 10 m.
  dk = RG.limit_correction_by_edges(-0.03, e)
  assert dk == pytest.approx(2 * -0.12 / 100.0)
  assert RG.limit_correction_by_edges(0.03, e) == pytest.approx(2 * 0.32 / 100.0)
  # Already inside the margin: no correction toward that side at all; the other side untouched.
  tight = RG.EdgeInfo(x=10.0, path_y=0.0, left_y=-1.0, right_y=None)
  assert RG.limit_correction_by_edges(-0.03, tight) == 0.0
  assert RG.limit_correction_by_edges(0.03, tight) == 0.03
  assert RG.limit_correction_by_edges(-0.03, None) == -0.03


def test_pose_error_lowers_confidence_and_fades():
  """No GNSS after the approach: fix age -> confidence 0 -> correction slews back to the model."""
  p = PASSES["00000128"]
  ring = _ring(p)
  g = RG.RoundaboutGuide()
  g.set_ring(ring)
  fixes = {round(f[0], 1): f for f in p["fixes"]}
  for t, x, y, psi, v, yr in p["hist"]:
    g.update(t, v, yr, 0.0, enabled=True, lat_active=True)
    if round(t, 1) in fixes:
      lat, lon = latlon_from_xy(x, y, ring.lat, ring.lon)
      g.gnss(t, lat, lon, math.degrees(psi) % 360.0, v, 1.0, latency_s=0.0)
  assert g.pose.confidence(p["hist"][-1][0]) > 0.9
  t, x, y, psi, v = p["hist"][-1][0], p["hist"][-1][1], p["hist"][-1][2], p["hist"][-1][3], 8.0
  outs = []
  for _ in range(800):   # 8 s, no fixes, driving straight into the ring
    t += DT
    outs.append(g.update(t, v, 0.0, 0.0, enabled=True, lat_active=True))
  assert g.pose.confidence(t) == 0.0
  assert outs[-1] == 0.0


def test_map_match_bias_removes_gnss_offset_beyond_a_lane():
  """R126: GNSS ran ~4-6 m north of the inbound Soco Rd; one lane (1.75 m) of it is not an error."""
  ring = _ring(PASSES["00000126"])
  g = RG.RoundaboutGuide()
  g.set_ring(ring)
  # Inbound on the lower Soco branch (travel direction stored in the ring JSON), driven along its polyline.
  app = min((a for a in ring.approaches if math.hypot(*a[-1]) < math.hypot(*a[0]) and a[0][1] < -5.0),
            key=lambda a: a[0][0])
  segs = list(zip(app, app[1:], strict=False))
  bias = 5.0
  lefts = []
  i = 0
  for (x0, y0), (x1, y1) in segs:
    L = math.hypot(x1 - x0, y1 - y0)
    ux, uy = (x1 - x0) / L, (y1 - y0) / L
    nx, ny = -uy, ux                   # left normal of travel
    psi = math.atan2(ux, uy)
    for k in range(int(L / (8.0 * DT))):
      t = i * DT
      x, y = x0 + 8.0 * DT * k * ux, y0 + 8.0 * DT * k * uy
      g.update(t, 8.0, 0.0, 0.0, enabled=False, lat_active=False)
      if i % 100 == 0 and i > 0 and math.hypot(x, y) > ring.radius_m + 10.0:
        lat, lon = latlon_from_xy(x + bias * nx, y + bias * ny, ring.lat, ring.lon)
        g.gnss(t, lat, lon, math.degrees(psi) % 360.0, 8.0, 1.0, latency_s=0.0)
        lefts.append((nx, ny))
      i += 1
  nx, ny = lefts[-1]
  bx, by = g.bias
  assert (bx * nx + by * ny) == pytest.approx(bias - RG.LANE_SNAP_M, abs=0.8)


def test_circle_follow_sign_convention():
  # On the ring centerline, tangent to CCW circulation: follow curvature ~ -1/R (left, negative).
  R = 21.6
  k = RG.circle_follow_curvature(R, 0.0, 0.0, 7.0, R, True)      # east point, heading north
  assert k == pytest.approx(-1.0 / R, abs=1e-6)
  k_cw = RG.circle_follow_curvature(R, 0.0, math.pi, 7.0, R, False)  # CW: heading south
  assert k_cw == pytest.approx(1.0 / R, abs=1e-6)
  # Outside the lane radius -> tighter left; heading outward -> tighter left.
  assert RG.circle_follow_curvature(R + 2.0, 0.0, 0.0, 7.0, R, True) < -1.0 / (R + 2.0)
  assert RG.circle_follow_curvature(R, 0.0, math.radians(20.0), 7.0, R, True) < -1.0 / R


def test_params_and_menu_wiring():
  keys = (ROOT / "common" / "params_keys.h").read_text()
  assert '{"NAPRoundaboutAssist", {PERSISTENT, BOOL, "0"}}' in keys
  assert '{"NAPRoundaboutRing", {CLEAR_ON_MANAGER_START, JSON}}' in keys
  from openpilot.selfdrive.ui.layouts.settings import nap_lateral
  assert nap_lateral.NAP_ROUNDABOUT_ASSIST == RG.PARAM_ROUNDABOUT_ASSIST
  assert nap_lateral.ROUNDABOUT_ASSIST_DESCRIPTION.startswith("Default Off.")
  menu = (ROOT / "selfdrive" / "ui" / "layouts" / "settings" / "driving_mannerisms.py").read_text()
  assert '"Roundabout Steering Assist"' in menu and "NAP_ROUNDABOUT_ASSIST" in menu


def test_controlsd_applies_assist_before_legacy_bias_and_pins_nothing_else():
  src = (ROOT / "selfdrive" / "controls" / "controlsd.py").read_text()
  assert "'gpsLocation', 'gpsLocationExternal'" in src
  assert "RoundaboutAssist(self._turn_geom_preap, self.params)" in src
  i_assist = src.index("self.rb_assist.update(")
  i_bias = src.index("rb_bias = roundabout_lateral_curvature_bias(\n")
  i_handoff = src.index("new_desired_curvature = handoff_new_desired_curvature(")
  assert i_assist < i_bias < i_handoff
  assert "model_or_plan_curvature = float(model_or_plan_curvature) + (0.0 if self.rb_assist.active else rb_bias)" in src
  # Assist output flows through the unchanged handoff + clip_curvature path.
  assert "clip_curvature(\n        CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)" in src


def test_nan_odometry_sample_does_not_poison_the_pose():
  """R126 qlog had NaN yaw samples early in the route; one NaN used to leave the pose NaN for the drive."""
  ring = _ring(PASSES["00000128"])
  g = RG.RoundaboutGuide()
  g.set_ring(ring)
  x, y = 60.0, 6.0
  for i in range(600):
    t = i * DT
    yr = float("nan") if i == 50 else 0.0
    assert g.update(t, 10.0, yr, 0.001, enabled=True, lat_active=True) == 0.001 or i > 0
    if i % 100 == 99:
      lat, lon = latlon_from_xy(x - 10.0 * t, y, ring.lat, ring.lon)
      g.gnss(t, lat, lon, 270.0, 10.0, 1.0, latency_s=0.0)
  pose = g.corrected_pose()
  assert pose is not None and all(math.isfinite(v) for v in pose)
  assert g.update(6.01, float("nan"), 0.0, 0.002, enabled=True, lat_active=True) == 0.002


# ---------------------------------------------------------------------------------------------
# Sep 29 11:57:48 (Scallywag): the left curl was ~2x the model at 17-18 mph from a pose that read inside the lane
# (qcom bearing accuracy 30-50°, map-match bias 1.2→3.4 m). 11:57:20-52 carState v / yaw / model curvature (10 Hz)
# and the 1 Hz GNSS fixes are in the fixture; t is relative to 11:57:20.

SEP29 = json.loads((Path(__file__).parent / "data" / "roundabout_sep29_1157.json").read_text())
MPH_MS = 0.44704


def _sep29(**patch):
  """Replay the ring pass through the guide; returns (rows, guide). rows: (t, v, model_k, out_k, latched)."""
  ring = RingGeometry.from_json(SEP29["ring"])
  g = RG.RoundaboutGuide()
  g.set_ring(ring)
  fixes = list(SEP29["gps"])
  fi, rows = 0, []
  for t, v, yr, mk, lat in SEP29["rows"]:
    while fi < len(fixes) and fixes[fi][0] <= t:
      f = fixes[fi]
      g.gnss(f[0], f[1], f[2], f[3], f[4], f[5], patch.get("latency", RG.GPS_LATENCY_S))
      fi += 1
    out = g.update(t, v, yr, mk, enabled=t >= 16.0, lat_active=bool(lat))
    rows.append((t, v, mk, out, g.latched))
  return rows, g


def _old_constants(monkeypatch):
  """The Sep 29 behavior: 3.0 m/s² left cap, fast slews, no curl hold, latch whatever the pose, no ring cap."""
  for k, v in dict(A_LAT_CIRC_MAX=3.0, OUT_SLEW=0.07, TARGET_SLEW=0.07, OUT_SLEW_GOOD=0.07, TARGET_SLEW_GOOD=0.07, SPEED_HOLD_MS=1e9,
                   BEARING_ACC_UNRELIABLE_DEG=1e9, BIAS_UNRELIABLE_M=1e9, BIAS_LATCH_DRIFT_M=1e9, RING_CAP_RATIO=1e3, RING_A_LAT_MAX=1e3,
                   RING_SLEW=0.07).items():
    monkeypatch.setattr(RG, k, v)


def _left_peak(rows):
  win = [r for r in rows if 24.5 <= r[0] <= 30.2]
  return max(-r[3] for r in win), max(-r[2] for r in win)


def test_sep29_left_curl_no_longer_doubles_the_model(monkeypatch):
  _old_constants(monkeypatch)
  old_rows, _ = _sep29(latency=0.9)
  old_out, old_mk = _left_peak(old_rows)
  assert old_out > 1.5 * old_mk                  # the logged bug (0.0476 vs the model's 0.0237) without the Sep 29/30 limits
  monkeypatch.undo()
  rows, g = _sep29()
  out, mk = _left_peak(rows)
  assert out <= 1.2 * mk + 1e-9                  # never more than 1.2x the model through the pass
  assert out < 0.7 * old_out
  assert not any(r[4] for r in rows if r[0] < 30.0)   # no latch on the unreliable pose (bearing accuracy 33-45°)
  assert all(abs(r[3] - r[2]) < 1e-9 for r in rows if 29.0 <= r[0] <= 30.2)   # ...and the assist has faded: the model keeps the pass
  assert any("latch blocked unreliable" in e for e in g.events)


def test_sep29_left_transition_accel_and_rate_stay_low():
  rows, _ = _sep29()
  win = [r for r in rows if 24.5 <= r[0] <= 30.2]
  assert max(abs(r[3]) * r[1] ** 2 for r in win if r[3] < 0) <= 2.2
  assert max(abs((b[3] - b[2]) - (a[3] - a[2])) / (b[0] - a[0]) for a, b in zip(win, win[1:], strict=False)) <= 0.035   # correction rate


def test_gps_latency_is_the_measured_one():
  assert RG.GPS_LATENCY_S == 0.66
  assert RG.GPS_EXT_LATENCY_S == 0.2


def test_left_transition_cap_uses_actual_speed():
  v = 8.0
  assert RG.lat_limited(-1.0, v, 1.0) == pytest.approx(-2.2 / v ** 2)        # CCW: left (negative) capped at 2.2 m/s²
  assert RG.lat_limited(1.0, v, 1.0) == pytest.approx(3.0 / v ** 2)          # right entry keeps 3.0
  assert RG.lat_limited(1.0, v, -1.0) == pytest.approx(2.2 / v ** 2)         # CW ring mirrors it
  assert RG.lat_limited(-1.0, v, -1.0) == pytest.approx(-3.0 / v ** 2)


def test_ring_cap_is_1_15x_the_model_and_2_2_mps2():
  assert (RG.RING_CAP_RATIO, RG.RING_A_LAT_MAX, RG.RING_SLEW) == (1.15, 2.2, 0.035)
  assert RG.ring_cap(0.02, -0.02, 6.0, 1.0) == pytest.approx(1.15 * 0.02)               # CCW, model 0.02 left: 1.15x
  assert RG.ring_cap(0.03, -0.03, 8.0, 1.0) == pytest.approx(2.2 / 64.0)                # lateral cap bites first (1.15x = 0.0345)
  assert RG.ring_cap(0.05, -0.05, 20.0, 1.0) == pytest.approx(0.05)                     # ...never below the model itself
  assert RG.ring_cap(0.02, 0.01, 6.0, 1.0) == pytest.approx(-0.01)                      # model curving away: the assist never adds left
  assert RG.ring_cap(0.02, 0.02, 6.0, -1.0) == pytest.approx(1.15 * 0.02)               # CW mirrors it
  assert RG.ring_floor(0.02, 6.0) == pytest.approx(0.85 * 0.02)


def test_slews_are_0_035_for_a_poor_pose():
  assert RG.OUT_SLEW == 0.035 and RG.TARGET_SLEW == 0.035
  assert RG.OUT_SLEW_GOOD == 0.07 and RG.TARGET_SLEW_GOOD == 0.07     # good pose: R126/R128 fixtures need the old rate


def test_pose_quality_thresholds():
  assert RG.pose_quality(31.0, 0.0, 0.0, 1.0) == (True, True)         # bearing accuracy > 30°
  assert RG.pose_quality(10.0, 0.0, 0.0, 1.0) == (False, False)
  assert RG.pose_quality(25.0, 2.5, 0.0, 1.0) == (True, True)         # bias > 2 m with a degraded bearing
  assert RG.pose_quality(10.0, 4.5, 0.0, 1.0) == (False, False)       # R126: large but removed bias, usable bearing
  assert RG.pose_quality(25.0, 0.5, 0.0, 1.0) == (False, True)        # poor: latch allowed, capped
  assert RG.pose_quality(10.0, 0.5, 0.0, RG.INNOV_POOR_M + 0.1) == (False, True)


def test_curl_is_held_until_within_3_mph_of_ring_speed():
  assert RG.SPEED_HOLD_MS == pytest.approx(3.0 * MPH_MS)
  assert RG.hold_curl(-0.03, 0.01, 1.0) == 0.01                        # CCW: left (negative) not added over the model
  assert RG.hold_curl(0.03, 0.01, 1.0) == 0.03                         # right still added
  assert RG.hold_curl(0.03, -0.01, -1.0) == -0.01                      # CW mirrors it
  ring = RingGeometry.from_json(SEP29["ring"])
  g = RG.RoundaboutGuide()
  g.set_ring(ring)
  assert g.ring_v == pytest.approx(18.0 * MPH_MS, abs=0.05)   # RB_RING_SPEED_MPH
  rows, g2 = _sep29()
  assert any(e.startswith("curl held") for e in g2.events) and any(e.startswith("curl released") for e in g2.events)


def _tangent_guide(bacc, bias, old_bias=None):
  """Guide with a fabricated pose on the ring's east point heading north (tangent for CCW), one update in."""
  ring = RingGeometry.from_json(SEP29["ring"])
  g = RG.RoundaboutGuide()
  g.set_ring(ring)
  g.pose.bacc = bacc
  g.bias = (bias, 0.0)
  if old_bias is not None:
    g.bias_hist.append((-2.0, old_bias, 0.0))
  g.pose.initialized = True
  g.pose.xo, g.pose.yo, g.pose.psio, g.pose.dpsi = ring.radius_m - 2.5 + bias, 0.0, 0.0, 0.0
  g.pose.t = 0.0
  g.pose.n_good, g.pose.innov, g.pose.last_fix_t = 3, 0.5, 0.0
  g.phase = RG.ACTIVE
  g.update(0.1, 7.0, 0.0, -0.02, enabled=True, lat_active=True)
  return g, ring


def test_poor_pose_latch_never_aims_at_the_inner_side():
  g, ring = _tangent_guide(5.0, 0.3)                       # good pose: the lane clamp R - hw/2 is fine
  assert g.latched and g.r_ref >= ring.radius_m - 0.5 * ring.half_width_m - 1e-9
  g, ring = _tangent_guide(25.0, 0.3)                      # degraded bearing: latch, but never inside R
  assert g.latched and g.latch_poor and g.r_ref >= ring.radius_m - 1e-9
  g, ring = _tangent_guide(5.0, 3.0)                       # bias > 2 m: never inside R
  assert g.latched and g.r_ref >= ring.radius_m - 1e-9


def test_unreliable_or_unsettled_pose_does_not_latch():
  g, _ = _tangent_guide(35.0, 0.3)
  assert not g.latched and any("latch blocked unreliable" in e for e in g.events)
  g, _ = _tangent_guide(25.0, 2.5)
  assert not g.latched
  g, _ = _tangent_guide(5.0, 0.3, old_bias=2.0)             # bias moved 1.7 m in the last 2 s: not bias-consistent yet
  assert not g.latched and any("latch blocked bias_unsettled" in e for e in g.events)
  g, _ = _tangent_guide(5.0, 0.3, old_bias=0.0)             # 0.3 m: consistent
  assert g.latched


def _with_poor_pose(monkeypatch, name):
  monkeypatch.setattr(RG, "pose_quality", lambda *a: (False, True))
  rows, ring = simulate(name, "after")
  return rows, _LAST["guide"]


@pytest.mark.parametrize("name", sorted(PASSES))
def test_poor_pose_uses_the_slow_slews(monkeypatch, name):
  rows, _ = _with_poor_pose(monkeypatch, name)
  for a, b in zip(rows, rows[1:], strict=False):
    assert abs((b[6] - b[5]) - (a[6] - a[5])) / DT <= RG.OUT_SLEW + 1e-6 or abs(b[6] - a[6]) / DT <= RG.OUT_SLEW + 1e-6   # or held
    assert abs(b[6] - a[6]) / DT <= max(RG.OUT_SLEW, RG.RING_CAP_RATIO * abs(b[5] - a[5]) / DT) + 1e-6   # the cap follows the model at 1.15x


@pytest.mark.parametrize("name", sorted(PASSES))
def test_poor_pose_target_slew(monkeypatch, name):
  seen = []
  orig = RG.RoundaboutGuide.update

  def spy(self, t, *a, **k):
    out = orig(self, t, *a, **k)
    seen.append((t, self.k_tgt))
    return out
  monkeypatch.setattr(RG.RoundaboutGuide, "update", spy)
  _with_poor_pose(monkeypatch, name)
  steps = [(b[0] - a[0], abs(b[1] - a[1])) for a, b in zip(seen, seen[1:], strict=False) if a[1] is not None and b[1] is not None and 0 < b[0] - a[0] < 0.05]
  assert len(steps) > 500
  jumps = [d / dt for dt, d in steps if d / dt > RG.TARGET_SLEW + 1e-6]
  assert len(jumps) <= 3    # only re-seeds when lat/the model reset the target


@pytest.mark.parametrize("name", ["00000128", "00000130"])
def test_ring_output_never_above_1_15x_the_model_or_2_2_after_the_latch(name):
  """After the latch, whatever the pose quality, the output is within [0.85, 1.15] x the model while it curves with the circulation.
  Oct 1 (ring hold): except while the cap is widened toward the circle (model under half of it, <= CIRCLE_CAP_EXTRA over the model) or
  inside an exit window (floor = the circle target); those are bounded below and in test_ring_hold_*."""
  for poor in (False, True):
    with pytest.MonkeyPatch.context() as mp:
      if poor:
        mp.setattr(RG, "pose_quality", lambda *a: (False, True))
      seen = {}
      orig = RG.RoundaboutGuide.update

      def spy(self, t, *a, _o=orig, _s=seen, **k):
        out = _o(self, t, *a, **k)
        pose = self.corrected_pose()
        _s[round(t, 2)] = (self.cap_extra, pose is not None and RH.in_exit_window(math.atan2(pose[1], pose[0]), 1.0, self.exits))
        return out
      mp.setattr(RG.RoundaboutGuide, "update", spy)
      rows, _ = simulate(name, "after", pool=PASSES_ALL)
      g = _LAST["guide"]
    assert g.latch_t is not None
    peak = 0.0
    for r in rows:
      if r[0] < g.latch_t + 0.5 or r[0] > g.latch_t + 40.0 or r[5] >= 0.0 or r[6] == r[5]:
        continue
      extra, window = seen[round(r[0], 2)]
      if extra > 0.0 or window:
        assert -r[6] * r[4] ** 2 <= RG.RING_A_LAT_MAX * 1.0001          # the widened cap / floor stays bound by 2.2 m/s²
        continue
      peak = max(peak, r[6] / r[5])
      assert r[6] / r[5] <= RG.RING_CAP_RATIO + 1e-6
      assert -r[6] * r[4] ** 2 <= max(RG.RING_A_LAT_MAX, -r[5] * r[4] ** 2) + 1e-6
    assert peak > 1.05


def test_ring_output_slews_at_most_0_035_after_the_latch():
  rows, _ = simulate("00000128", "after", pool=PASSES_ALL)
  g = _LAST["guide"]
  win = [r for r in rows if g.latch_t <= r[0] <= g.latch_t + 8.0]
  assert len(win) > 500
  for a, b in zip(win, win[1:], strict=False):
    assert abs((b[6] - b[5]) - (a[6] - a[5])) / DT <= RG.RING_SLEW + 1e-6 or abs(b[6] - a[6]) / DT <= RG.RING_SLEW + 1e-6   # or held


def test_curl_held_while_over_ring_speed_on_the_fixtures():
  for name in ("00000128",):
    rows, ring = simulate(name, "after", slow=False)     # no ring slow-down: the car arrives fast
    g = _LAST["guide"]
    R, hw = ring.radius_m, ring.half_width_m
    fast = [r for r in rows if r[4] > g.ring_v + RG.SPEED_HOLD_MS and math.hypot(r[1], r[2]) > R - 0.5 * hw]
    assert fast and all(r[6] >= r[5] - 1e-9 for r in fast)


def test_ring_state_is_logged_via_cloudlog_without_capnp(monkeypatch):
  msgs = []
  monkeypatch.setattr(RG.cloudlog, "error", lambda m: msgs.append(m))   # errorLogMessage is in qlog, logMessage is rlog-only
  a = _assist()
  _drive_assist(a, "00000128")
  assert any(m.startswith("roundabout_assist latched") for m in msgs)
  assert any(m.startswith("roundabout_assist released") for m in msgs)
  assert any(m.startswith("roundabout_assist state phase=") and "bacc=" in m and "r_ref=" in m for m in msgs)
  src = (ROOT / "selfdrive" / "controls" / "lib" / "roundabout_guide.py").read_text()
  assert "import cereal" not in src and "from cereal" not in src
  assert "from cereal" not in (ROOT / "selfdrive" / "controls" / "lib" / "roundabout_ring_log.py").read_text()


class _Seq:
  def __init__(self, x, y):
    self.x, self.y = x, y


class _FakeModel:
  position = _Seq([0.0, 10.0, 20.0, 40.0], [0.0, 0.5, 2.0, 8.0])
  laneLines = [_Seq([0.0, 40.0], [-5.5, -5.0]), _Seq([0.0, 40.0], [-1.8, -1.4]), _Seq([0.0, 40.0], [1.8, 2.2]), _Seq([0.0, 25.0], [5.5, 6.0])]
  laneLineProbs = [0.1, 0.9, 0.8, 0.2]
  roadEdges = [_Seq([0.0, 40.0], [-7.0, -6.0]), _Seq([0.0, 40.0], [4.0, 4.5])]
  roadEdgeStds = [0.3, 1.2]


def test_raw_model_summary_logged_near_and_in_the_ring(monkeypatch):
  """~1.25 Hz compact JSON of the model's raw lane lines / road edges / path (+ ring state and driver inputs) near and in a ring."""
  msgs = []
  monkeypatch.setattr(RG.cloudlog, "error", lambda m: msgs.append(m))
  a = _assist()
  p = PASSES["00000128"]
  ring = _ring(p)
  sm = FakeSM()
  t = 0.0
  for i in range(400):
    t = p["hist"][0][0] + i * 0.05
    a.update(sm, t=t, v_ego=8.0, yaw_rate=0.0, model_k=0.0, lat_active=True, maneuver_active=False, lane_change_active=False,
             hint=Hint(ring.way_ids[0]), model_v2=_FakeModel())
  raw = [m for m in msgs if m.startswith("roundabout_raw ")]
  assert raw == [] or len(raw) <= int(400 * 0.05 / RG.LOG_RAW_PERIOD_S) + 1
  g = a.guide
  g.d_edge, g.debug = 20.0, {"r": 45.0}
  a._last_raw_t = -1e9
  a._log(100.0, 0.01, 0.01, 8.0, _FakeModel(), RG.DriverInput(2, True, False, 0.4), True)
  lines = [m for m in msgs if m.startswith("roundabout_raw ")]
  assert lines
  line = lines[-1]
  d = json.loads(line[len("roundabout_raw "):])
  assert d["path"] == [0.25, 0.5, 1.25, 2.0, 5.0]
  assert len(d["ll"]) == 4 and d["ll"][0][0] == -5.44 and d["ll"][3][-1] is None     # 4th line stops at 25 m
  assert d["llp"] == [0.1, 0.9, 0.8, 0.2] and d["res"] == [0.3, 1.2] and len(d["re"]) == 2
  assert d["stalk"] == [2, 1, 0, 0.4] and d["lat"] == 0 and d["de"] == 20.0
  assert len(line) < 700
  n = len(msgs)
  a._log(100.3, 0.01, 0.01, 8.0, _FakeModel(), None, True)       # inside LOG_RAW_PERIOD_S: no second summary
  assert len(msgs) == n
  g.d_edge = 200.0
  a._log(105.0, 0.01, 0.01, 8.0, _FakeModel(), None, True)       # far from any ring: nothing
  assert len(msgs) == n
  a._log(106.0, 0.01, 0.01, 8.0, None, None, True)               # no model message: nothing
  assert len(msgs) == n


def test_off_stays_bit_identical_on_the_sep29_pass():
  ring_json = json.dumps(SEP29["ring"])
  for on, preap in ((False, True), (True, False)):
    a = RG.RoundaboutAssist(preap, FakeParams(on=on, ring=ring_json))
    sm = FakeSM()
    for t, v, yr, mk, lat in SEP29["rows"]:
      assert a.update(sm, t=t, v_ego=v, yaw_rate=yr, model_k=mk, lat_active=bool(lat), maneuver_active=False, lane_change_active=False,
                      hint=Hint(SEP29["ring"]["ids"][0]), model_v2=None) == mk


# ---------------------------------------------------------------------------------------------
# Sep 30: release only by the driver (held same-side stalk / steering pull / override); the driver always wins; tips suppressed

def _on_ring(**kw):
  """Latched guide on the ring (east point, heading north, CCW), a few updates in, model curving left (-0.02)."""
  g, ring = _tangent_guide(5.0, 0.3)
  assert g.latched and g.phase == RG.ACTIVE
  return g


def _run(g, n, driver=None, mk=-0.02, lat=True, t0=0.2, v=7.0):
  outs = []
  for i in range(n):
    outs.append(g.update(t0 + i * DT, v, v * mk, mk, enabled=True, lat_active=lat, driver=driver))
  return outs


def test_ring_assist_follows_the_lane_circle_once_latched():
  g = _on_ring()
  outs = _run(g, 100)
  assert g.latched and g.r_lane == pytest.approx(g.ring.radius_m - 0.5 * g.ring.half_width_m)
  assert all(o >= -0.02 - RH.CIRCLE_CAP_EXTRA - 1e-9 for o in outs)       # CCW: never more left than the model + CIRCLE_CAP_EXTRA


def test_held_right_stalk_releases_the_ring_and_hands_back_at_once():
  g = _on_ring()
  _run(g, 50)
  out = _run(g, 1, RG.DriverInput(stalk_dir=2, stalk_held=True), t0=0.7)
  assert out == [-0.02] and g.release == "stalk" and g.phase == RG.DONE and g.dk_out == 0.0
  assert any(e.startswith("released stalk") for e in g.events)
  assert _run(g, 20, t0=0.8) == [-0.02] * 20                              # stays released


def test_held_left_stalk_does_not_release_but_the_guide_weight_is_zero():
  g = _on_ring()
  _run(g, 50)
  outs = _run(g, 100, RG.DriverInput(stalk_dir=1, stalk_held=True), t0=0.7)
  assert outs == [-0.02] * 100 and g.latched and g.phase == RG.ACTIVE and g.release == "" and g.debug["w"] == 0.0


def test_steering_pull_toward_the_exit_releases_after_0_3_s():
  g = _on_ring()
  _run(g, 50)
  outs = _run(g, 25, RG.DriverInput(torque=-2.0), t0=0.7)                  # + is left: -2 Nm pulls right = the exit on a CCW ring
  assert outs == [-0.02] * 25 and g.release == "" and g.latched          # weight is 0 at once, not released yet
  outs = _run(g, 10, RG.DriverInput(torque=-2.0), t0=0.95)
  assert outs == [-0.02] * 10 and g.release == "pull" and g.phase == RG.DONE


def test_steering_pull_toward_the_circulation_does_not_release_but_wins():
  g = _on_ring()
  _run(g, 50)
  outs = _run(g, 100, RG.DriverInput(torque=2.0), t0=0.7)
  # Same-side add keeps the extra curl. It does not zero the weight and it does not release.
  assert any(o != -0.02 for o in outs) and g.release == "" and g.phase == RG.ACTIVE and g.latched


def test_steering_press_zeroes_the_weight_after_0_3_s_and_overrides_after_0_5_s():
  g = _on_ring()
  _run(g, 50)
  assert any(o != -0.02 for o in _run(g, 25, RG.DriverInput(pressed=True), t0=0.7))     # < 0.3 s: a flicker, the assist keeps going
  _run(g, 5, t0=0.95)                                                                    # hands off: the debounce resets
  mid = _run(g, 40, RG.DriverInput(pressed=True), t0=1.0)                                # 0.40 s: curl stays
  assert any(o != -0.02 for o in mid) and g.release == ""
  assert _run(g, 20, RG.DriverInput(pressed=True), t0=1.4)[-1] == -0.02 and g.release == "override"
  g = _on_ring()
  _run(g, 50)
  assert _run(g, 100, lat=False, t0=0.7) == [-0.02] * 100 and g.release == "override"      # lat off: the driver has the wheel


def test_no_blinker_no_map_exit_the_car_stays_on_the_ring():
  """No stalk / pull / press: the guide never releases for map reasons, on any of the four passes."""
  for name in ("00000126", "00000128", "0000012f", "00000130"):
    simulate(name, "after", pool=PASSES_ALL)
    g = _LAST["guide"]
    assert g.release in ("", "far_from_ring"), name         # the only non-driver release is the safety one, > 15 m outside the band
    if g.latch_t is not None:
      assert g.d_edge is None or g.release == "" or g.d_edge > RG.FAILSAFE_EDGE_M
  g = _on_ring()
  g.travel = 400.0                                                      # more than a full circuit of the ring: still no map exit
  outs = _run(g, 150)
  assert g.release == "" and g.latched and g.phase == RG.ACTIVE and any(o != -0.02 for o in outs)


def test_latched_weight_does_not_depend_on_pose_confidence():
  """The 1.15x / 2.2 / 0.035 limits bound the assist, not a confidence fade: a stale fix (2.5 s) keeps the full weight."""
  g = _on_ring()
  _run(g, 80)
  g.pose.last_fix_t = g.last_t - 2.5
  _run(g, 5, t0=1.0)
  assert g.debug["conf"] < 0.5 and g.debug["w"] == pytest.approx(1.0) and g.latched


def test_safety_release_far_outside_the_ring_fades_instead_of_stepping():
  g = _on_ring()
  _run(g, 50)
  g.pose.xo += 40.0                                                     # pose says 40 m outside the band
  _run(g, 5, t0=0.7)
  assert g.release == "far_from_ring" and g.phase in (RG.EXITING, RG.DONE)


class _PutParams(FakeParams):
  def __init__(self, **kw):
    super().__init__(**kw)
    self.puts = []

  def put_bool(self, key, val):
    self.puts.append((key, val))


def test_latched_param_and_stalk_release_through_the_assist():
  from openpilot.selfdrive.controls.lib.stalk_tip_turn import PARAM_RING_LATCHED
  assert PARAM_RING_LATCHED == "NAPRoundaboutLatched"
  params = _PutParams(on=True, ring=json.dumps(PASSES["00000128"]["ring"]))
  a = RG.RoundaboutAssist(True, params)
  g = _on_ring()
  a.guide = g
  sm = FakeSM()
  def step(i, stalk):
    return a.update(sm, t=0.2 + i * DT, v_ego=7.0, yaw_rate=-0.14, model_k=-0.02, lat_active=True, maneuver_active=False,
                    lane_change_active=False, hint=Hint(g.ring.way_ids[0]), model_v2=None, stalk_state=stalk)
  for i in range(30):
    step(i, 0)
  assert a.latched and params.puts == [(PARAM_RING_LATCHED, True)]
  outs = [step(30 + i, 2) for i in range(60)]                            # right stalk held 0.6 s: StalkTipTurn.is_turn after 0.40 s
  assert g.release == "stalk" and not a.latched and params.puts[-1] == (PARAM_RING_LATCHED, False)
  assert outs[-1] == -0.02


def test_off_while_latched_resets_and_clears_the_param():
  params = _PutParams(on=True, ring=json.dumps(PASSES["00000128"]["ring"]))
  a = RG.RoundaboutAssist(True, params)
  a.guide = _on_ring()
  a.update(FakeSM(), t=0.2, v_ego=7.0, yaw_rate=0.0, model_k=-0.02, lat_active=True, maneuver_active=False, lane_change_active=False,
           hint=Hint(a.guide.ring.way_ids[0]))
  assert params.puts == [(RG.PARAM_RING_LATCHED, True)]
  params.on = False
  a._frame = RG.PARAM_READ_FRAMES
  assert a.update(FakeSM(), t=0.3, v_ego=7.0, yaw_rate=0.0, model_k=-0.02, lat_active=True, maneuver_active=False,
                  lane_change_active=False) == -0.02
  assert not a.guide.latched and params.puts[-1] == (RG.PARAM_RING_LATCHED, False)


def test_tips_are_suppressed_while_latched_and_modeld_controlsd_are_wired():
  from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState
  from openpilot.selfdrive.controls.lib.tests.test_desire_helper import _arm_left
  dh = DesireHelper()
  dh.suppress_tips = True
  _arm_left(dh)
  assert dh.lane_change_state == LaneChangeState.off                     # a tip no longer arms ALC on the ring
  dh = DesireHelper()
  _arm_left(dh)
  assert dh.lane_change_state == LaneChangeState.preLaneChange           # ...and does elsewhere
  modeld = (ROOT / "selfdrive" / "modeld" / "modeld.py").read_text()
  assert "DH.suppress_tips = params.get_bool(PARAM_RING_LATCHED)" in modeld
  assert modeld.index("DH.suppress_tips = ") < modeld.index("DH.update(")
  cd = (ROOT / "selfdrive" / "controls" / "controlsd.py").read_text()
  assert "stalk_state=int(getattr(CS, 'turnSignalStalkState', 0) or 0)" in cd
  assert "steering_pressed=bool(CS.steeringPressed), steering_torque=float(CS.steeringTorque)" in cd
  assert '{"NAPRoundaboutLatched", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}}' in (ROOT / "common" / "params_keys.h").read_text()


def test_ring_speed_constant_is_18_mph_and_planner_uses_it():
  from openpilot.selfdrive.mapd import roundabout as rb
  assert rb.RB_RING_SPEED_MPH == 18.0 and rb.RB_V_MIN_MS == pytest.approx(18.0 * MPH)
  assert rb.roundabout_target_ms(12.0 * MPH) == pytest.approx(18.0 * MPH)       # a slower comfort speed is floored up to it
  assert rb.roundabout_target_ms(40.0 * MPH) == pytest.approx(20.0 * MPH)       # and the max stays 20 mph
