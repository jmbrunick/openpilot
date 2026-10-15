"""Curve follow replayed closed loop on Justin's logged model paths (Oct 4-10 2026).

Each fixture is a window of the 2 Hz `curvefollow` shadow records that carry
the full 33-point path (T, D, K, Y) plus his carState speed and accel at 10 Hz.
The path is held between records and shifted by the distance driven since.
The simulated car holds the fastest speed Justin drove 15 to 4 s before the
apex (as OP long would at that set speed) and curve follow is its only curve term (active mode), so the speed it
reaches at the apex is curve follow's own.
"""
from __future__ import annotations

import base64
import bisect
import gzip
import json
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.curve_follow import FREE_A_MS2, CurveFollow

HERE = Path(__file__).parent / "curve_follow_replay"
MPH = 2.23694
DT = 0.05
HOLD_S = 3.0


@pytest.fixture(autouse=True)
def _curve_follow_code_enabled(monkeypatch):
  """nap-release forces curve follow off; these tests exercise the code itself."""
  try:
    from openpilot.common import nap_release
  except ImportError:
    return
  monkeypatch.setattr(nap_release, "CURVE_FOLLOW_ENABLED", True, raising=False)


def _load(name: str) -> dict:
  raw = (HERE / f"{name}.json.gz.b64").read_text().replace("\n", "")
  return json.loads(gzip.decompress(base64.b64decode(raw)))


def _shift(rec, age, travel):
  T = np.asarray(rec["T"], float)
  D = np.asarray(rec["D"], float)
  K = np.asarray(rec["K"], float) * 1e-4
  Y = np.asarray(rec["Y"], float) if rec.get("Y") and len(rec["Y"]) == len(T) else None
  if travel >= D[-1] - 5.0:
    return None
  keep = (D - travel) > 0.0
  d = np.concatenate([[0.0], D[keep] - travel])
  k = np.concatenate([[np.interp(travel, D, K)], K[keep]])
  t = np.concatenate([[0.0], np.maximum(T[keep] - age, 0.01)])
  y = None if Y is None else np.concatenate([[np.interp(travel, D, Y)], Y[keep]]).tolist()
  return t.tolist(), d.tolist(), k.tolist(), y


def _replay(fx: dict, *, pre: float = 20.0, post: float = 6.0, driver: str = "cruise"):
  """Rows (t, v, a_cmd, a_cf, flags) from t=-pre to +post around the fixture's centre."""
  recs = fx["recs"]
  rt = [r["t"] for r in recs]
  jt = [j[0] for j in fx["justin"]]
  cf = CurveFollow()
  t = -pre
  # Set speed: the fastest Justin went 15 to 4 s before the centre.
  v_set = max(float(j[1]) for j in fx["justin"] if -15.0 <= j[0] <= -4.0)
  v = v_set
  rows = []
  last, travel = None, 0.0
  while t < post:
    j = bisect.bisect_right(rt, t) - 1
    path = None
    rec = None
    if j >= 0 and t - rt[j] <= HOLD_S:
      if last != j:
        last, travel = j, 0.0
      rec = recs[j]
      path = _shift(rec, t - rt[j], travel)
    if path is None:
      a_cf = cf.step(dt=DT, v_ego=v, ts=None, ds=None, ks=None, valid=False)
      flags = {}
    else:
      ts, ds, ks, ys = path
      flags = {"lc": rec["lc"] != "off", "rb": bool(rec["rb"])}
      a_cf = cf.step(dt=DT, v_ego=v, ts=ts, ds=ds, ks=ks, y_std=ys, lane_change=flags["lc"],
                     blinker=bool(rec["bl"]), frame_drop_pct=float(rec["df"] or 0.0), rb_active=flags["rb"])
    k = min(bisect.bisect_left(jt, t), len(jt) - 1)
    if driver == "cruise":
      a_drv = max(-0.3, min(0.8, 0.4 * (v_set - v)))
    else:
      a_drv = fx["justin"][k][2] + 0.5 * (fx["justin"][k][1] - v)
    a = min(a_drv, a_cf)
    rows.append((t, v, a, a_cf, flags, fx["justin"][k][1]))
    v = max(0.0, v + a * DT)
    travel += v * DT
    t += DT
  return rows


def _apex(rows):
  w = [r for r in rows if -1.0 <= r[0] <= 1.0]
  return min(r[1] for r in w) * MPH, min(r[5] for r in w) * MPH


def _brake_jerk(rows):
  worst = 0.0
  for r0, r1 in zip(rows, rows[1:], strict=False):
    if r0[2] < 0.0 and r1[2] < 0.0:
      worst = max(worst, (r0[2] - r1[2]) / DT)
  return worst


SWEEPERS = ["sweeper_1004_152424", "sweeper_1004_115709", "sweeper_1005_165317", "sweeper_1004_125119",
            "town_1010_164157", "town_1004_092417"]


@pytest.mark.parametrize("name", SWEEPERS)
def test_curves_a_little_slower_than_justin_never_far_under(name):
  rows = _replay(_load(name))
  v_apex, v_justin = _apex(rows)
  under = v_justin - v_apex
  assert 0.5 <= under <= 8.0, (name, round(v_apex, 1), round(v_justin, 1))
  assert min(r[2] for r in rows) >= -1.5 - 1e-6
  assert _brake_jerk(rows) <= 2.0 + 1e-6


def test_late_seen_corner_brakes_firm_but_never_a_wall():
  """Oct 10 5:40:45 PM: 50 mph into an R~40 m turn the model shows only ~3.5 s out.

  Justin braked at up to 2.6 m/s² from 5.6 s out. Curve follow cannot reach the
  table speed by the apex from a 3.5 s view at <= 1.5 m/s² (that would need ~2.8);
  it must still use the urgent allowance, jerk-limited, and never pass -1.5.
  """
  rows = _replay(_load("late_corner_1010_174045"))
  a_min = min(r[2] for r in rows)
  assert -1.5 - 1e-6 <= a_min <= -1.4
  assert _brake_jerk(rows) <= 2.0 + 1e-6
  v_apex, _ = _apex(rows)
  assert v_apex <= rows[0][1] * MPH - 3.0


@pytest.mark.parametrize("name", ["lane_change_1010_164908", "lane_change_2"])
def test_no_slowing_during_lane_changes(name):
  rows = _replay(_load(name), pre=4.0, post=8.0, driver="justin")
  for t, _v, _a, a_cf, flags, _vj in rows:
    if flags.get("lc"):
      assert a_cf >= 0.0, (name, t, a_cf)


def test_no_slowing_on_a_straight():
  rows = _replay(_load("straight_1"), pre=10.0, post=10.0, driver="justin")
  assert all(r[3] == FREE_A_MS2 or r[3] >= 0.0 for r in rows)


def test_roundabout_funnel_owns_entry_and_curve_follow_adds_no_braking():
  """With the map roundabout funnel live (rb), curve follow fades out and never brakes.

  On its own it could not do the entry: the model shows the ring ~2-3 s out, so
  it would arrive at ~27 mph (Justin: 17). The funnel (#290 yield, ring speed
  18 mph) and the MAX restore on exit (#289) stay in charge.
  """
  fx = _load("roundabout_1007_200617")
  rows = _replay(fx)
  rb_rows = [r for r in rows if r[4].get("rb")]
  assert rb_rows, "fixture must carry the roundabout flag"
  assert all(r[3] >= 0.0 for r in rb_rows)


def test_post_apex_accel_is_capped_like_justin():
  """After a bend that slowed the car peaks, +a is held to 0.5 m/s² for ~2 s."""
  rows = _replay(_load("sweeper_1004_152424"), post=8.0)
  after = [r for r in rows if 0.0 <= r[0] <= 6.0]
  capped = [r for r in after if 0.0 < r[3] <= 0.5 + 1e-6]
  assert capped, "the post-apex cap should bind after this bend"
