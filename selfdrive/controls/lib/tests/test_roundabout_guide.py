"""Roundabout Steering Assist (NAPRoundaboutAssist, Pre-AP, default Off).

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
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.mapd.roundabout_map import RingGeometry, latlon_from_xy

ROOT = Path(__file__).resolve().parents[4]
FIXTURE = Path(__file__).parent / "data" / "roundabout_passes_sep28.json"
PASSES = json.loads(FIXTURE.read_text())["passes"]
MPH = 0.44704
DT = 0.01


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


def simulate(name: str, kind: str, *, slow: bool = True, fix_scale: float = 1.0):
  """Kinematic closed loop: model curvature by distance -> [guide] -> clip_curvature -> 0.2 s delay + 0.2 s lag.

  kind: "before" (model only) or "after" (guide On). slow: planner ring slow-down from the first hint.
  Returns rows (t, x, y, psi, v, model_k, out_k, des_k).
  """
  p = PASSES[name]
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
  v_target = p["hint_speed_ms"]
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


def band_metrics(rows, ring):
  """First 120° after entering the ring band: max distance beyond a curb, % inside the band."""
  R, hw = ring.radius_m, ring.half_width_m
  r = [math.hypot(row[1], row[2]) for row in rows]
  i0 = next(i for i, rr in enumerate(r) if rr < R + hw)
  th0 = math.atan2(rows[i0][2], rows[i0][1])
  travel, prev, out = 0.0, th0, []
  for i in range(i0, len(rows)):
    th = math.atan2(rows[i][2], rows[i][1])
    travel += math.degrees(RG._wrap(th - prev))
    prev = th
    if travel > 120.0:
      break
    out.append(max(0.0, (R - hw) - r[i], r[i] - (R + hw)))
  return max(out), 100.0 * sum(o < 0.01 for o in out) / len(out)


# ---------------------------------------------------------------------------------------------
# Replay: both passes, closed loop with the real GNSS error stream

@pytest.mark.parametrize("name", sorted(PASSES))
def test_model_only_leaves_the_ring_band(name):
  rows, ring = simulate(name, "before")
  beyond, inband = band_metrics(rows, ring)
  assert beyond > 2.5 and inband < 70.0


@pytest.mark.parametrize("name,max_beyond,min_inband", [("00000126", 0.5, 95.0), ("00000128", 0.5, 95.0)])
def test_assist_keeps_both_passes_in_the_ring_band(name, max_beyond, min_inband):
  before = band_metrics(*simulate(name, "before"))
  after = band_metrics(*simulate(name, "after"))
  assert after[0] <= max_beyond
  assert after[1] >= min_inband
  assert after[0] < before[0] and after[1] > before[1]


@pytest.mark.parametrize("name", sorted(PASSES))
def test_right_entry_held_until_turn_distance(name):
  rows, ring = simulate(name, "after")
  R, hw = ring.radius_m, ring.half_width_m
  for _t, x, y, _psi, _v, mk, out, _ in rows:
    d_edge = math.hypot(x, y) - (R + hw)
    if d_edge > RG.D_TURN_M + 3.0:   # + guide pose error margin
      assert out >= mk - 1e-9, f"early left at {d_edge:.1f} m"


@pytest.mark.parametrize("name", sorted(PASSES))
def test_left_transition_within_jerk_limit(name):
  rows, _ = simulate(name, "after")
  for a, b in zip(rows, rows[1:], strict=False):
    corr_rate = abs((b[6] - b[5]) - (a[6] - a[5])) / DT
    out_rate = abs(b[6] - a[6]) / DT
    model_rate = abs(b[5] - a[5]) / DT
    assert corr_rate <= RG.OUT_SLEW + 1e-6
    assert out_rate <= max(RG.OUT_SLEW, model_rate) + 1e-6


@pytest.mark.parametrize("name", sorted(PASSES))
def test_ring_speed_about_15_mph_and_lateral(name):
  p = PASSES[name]
  assert 15.0 * MPH <= p["hint_speed_ms"] <= 16.5 * MPH
  rows, ring = simulate(name, "after")
  R, hw = ring.radius_m, ring.half_width_m
  inside = [r for r in rows if math.hypot(r[1], r[2]) < R + hw]
  assert inside and max(r[4] for r in inside) <= 16.5 * MPH
  assert max(abs(r[7]) * r[4] ** 2 for r in inside) <= 3.0


@pytest.mark.parametrize("name", sorted(PASSES))
def test_hands_back_to_model_after_exit(name):
  rows, _ = simulate(name, "after")
  tail = rows[-150:]
  assert all(r[6] == r[5] for r in tail)
  assert _LAST["guide"].phase == RG.DONE
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
