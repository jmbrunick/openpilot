"""Crossing path near a stopped car: no classification, no positive accel.

Nov 17 2026, route 00000008 seg 6, 13:14:49.7–13:14:55.5 CST, nap-alpha
ef0c30ca. A stopped car at a light came into radar at 130 m (track 839,
vLead ~0.1 m/s) while ego was at 16 m/s. Radar lateral-speed noise of
0.66–0.95 m/s at 117–130 m qualified it as a crosser at 13:14:50.53. The
law then saw 28 % of the closing speed and the brake eased from -1.44 to
-0.17 m/s² at 100 m. At 13:14:53.84 the "exit sweep" fired on yRel
wander and the handover drove the command to +1.08 m/s² with the car at
70→55 m. The model held 0.98–0.99 on it from about 103 m, but the vision
veto only applied inside 55 m. The driver disengaged at 46.8 m.

Rows are the published radarState leadOne, carState.vEgo, the model-path
lateral offset and controlsState.curvature at each planner frame, plus the
aTarget that ef0c30ca published. The replay is open-loop (logged vEgo).
"""
import pytest

from openpilot.selfdrive.controls.lib.crossing_vehicle import (
  CROSS_EXIT_VY_MS,
  CROSS_GUARD_NEED_MS2,
  CROSS_VY_MS,
  CROSS_VY_RANGE_REF_M,
  crossing_stop_guard_active,
  crossing_vy_required,
)
from openpilot.selfdrive.controls.lib.unified_lead import (
  UnifiedLeadController,
  approach_physics_need,
  unified_follow_desired,
)
from openpilot.selfdrive.controls.tests.test_crossing_vehicle import (
  DT,
  T_FOLLOW,
  V_CEIL,
  V_EGO,
  _e1_y,
  _vlat_at,
)

# Lead-free command in force the frame before the lead (published +0.30).
NOV17_EXISTING_A = 0.30
# Planner seed step for a first lead that wants a brake (SEED_BRAKE_STEP_MS2).
NOV17_SEED_STEP = 0.7
NOV17_V_CEILING = 17.88  # 40 mph map limit on 2nd Street
NOV17_TRACK = 839

# (vEgo, dRel, vLead, aLeadK, yRel, vLat, modelProb, path_lat, curvature, aTarget_ef0c30ca)
NOV17_ROWS = (
  (15.9, 130.1, 0.12, -0.05, -2.23, 0.0, 0.66, 1.89, 0.00015, -0.76),
  (15.92, 128.9, 0.15, -0.06, -2.11, 0.0, 0.65, 1.87, 0.00015, -0.84),
  (15.94, 128.9, 0.15, -0.06, -2.11, 0.0, 0.62, 1.91, 0.00012, -0.93),
  (15.96, 127.5, 0.13, -0.07, -1.86, 0.0, 0.6, 1.67, 0.00012, -1.04),
  (15.97, 126.4, 0.15, -0.07, -1.73, 0.0, 0.74, 1.54, 8e-05, -1.16),
  (15.96, 126.4, 0.15, -0.07, -1.73, 0.0, 0.74, 1.52, 9e-05, -1.24),
  (15.97, 125.2, 0.09, -0.09, -1.86, 0.0, 0.81, 1.7, 5e-05, -1.3),
  (15.97, 124.1, 0.1, -0.1, -1.73, 0.0, 0.73, 1.58, 2e-05, -1.34),
  (15.98, 124.1, 0.1, -0.1, -1.73, 0.0, 0.54, 1.64, 2e-05, -1.36),
  (15.98, 122.7, 0.04, -0.12, -1.86, 0.95, 0.56, 1.84, 2e-05, -1.38),
  (15.96, 122.7, 0.04, -0.12, -1.86, 0.95, 0.61, 1.85, 2e-05, -1.38),
  (15.95, 121.2, 0.03, -0.12, -1.73, 0.8, 0.68, 1.64, -1e-05, -1.41),
  (15.95, 120.1, 0.02, -0.12, -1.73, 0.71, 0.6, 1.61, -1e-05, -1.42),
  (15.94, 120.1, 0.02, -0.12, -1.73, 0.71, 0.63, 1.62, -5e-05, -1.42),
  (15.93, 118.9, 0.07, -0.13, -1.61, 0.66, 0.63, 1.49, -5e-05, -1.44),
  (15.88, 118.9, 0.07, -0.13, -1.61, 0.66, 0.59, 1.55, -5e-05, -1.39),
  (15.82, 117.4, 0.13, -0.13, -1.61, 0.5, 0.69, 1.57, -8e-05, -1.35),
  (15.77, 116.3, 0.07, -0.13, -1.61, 0.43, 0.68, 1.62, -8e-05, -1.29),
  (15.68, 116.3, 0.07, -0.13, -1.61, 0.43, 0.7, 1.61, -0.00011, -1.22),
  (15.61, 115.1, -0.01, -0.15, -1.36, 0.37, 0.64, 1.31, -0.00011, -1.15),
  (15.55, 114.0, -0.01, -0.15, -1.23, 0.37, 0.75, 1.21, -0.00011, -1.07),
  (15.49, 114.0, -0.01, -0.15, -1.23, 0.37, 0.75, 1.27, -0.00014, -0.99),
  (15.42, 112.6, -0.01, -0.17, -0.98, 0.45, 0.84, 1.01, -0.00014, -0.92),
  (15.35, 111.5, -0.01, -0.18, -1.11, 0.56, 0.82, 1.13, -0.00018, -0.85),
  (15.26, 111.5, -0.01, -0.18, -1.11, 0.56, 0.86, 1.07, -0.00018, -0.78),
  (15.2, 110.2, -0.05, -0.18, -0.98, 0.7, 0.79, 1.03, -0.00018, -0.71),
  (15.13, 109.2, 0.0, -0.17, -0.98, 0.74, 0.85, 1.03, -0.00018, -0.65),
  (15.05, 109.2, 0.0, -0.17, -0.98, 0.74, 0.84, 1.03, -0.00018, -0.59),
  (14.98, 107.9, -0.15, -0.21, -0.98, 0.77, 0.86, 1.07, -0.00018, -0.54),
  (14.93, 107.9, -0.15, -0.21, -0.98, 0.77, 0.86, 1.1, -0.00018, -0.49),
  (14.87, 106.5, -0.14, -0.23, -0.98, 0.74, 0.9, 1.06, -0.00022, -0.44),
  (14.81, 105.4, -0.07, -0.22, -1.11, 0.64, 0.81, 1.21, -0.00021, -0.4),
  (14.76, 105.4, -0.07, -0.22, -1.11, 0.64, 0.82, 1.19, -0.00021, -0.36),
  (14.69, 104.4, -0.12, -0.22, -1.11, 0.53, 0.82, 1.19, -0.0002, -0.32),
  (14.67, 103.1, -0.05, -0.21, -1.23, 0.39, 0.84, 1.31, -0.0002, -0.29),
  (14.64, 103.1, -0.05, -0.21, -1.23, 0.39, 0.99, 1.3, -0.0002, -0.25),
  (14.6, 101.9, -0.06, -0.19, -1.36, 0.15, 0.99, 1.4, -0.00019, -0.23),
  (14.58, 100.5, -0.01, -0.16, -1.36, 0.0, 0.99, 1.38, -0.00019, -0.2),
  (14.56, 100.5, -0.01, -0.16, -1.36, 0.0, 0.99, 1.37, -0.00019, -0.19),
  (14.54, 99.5, -0.07, -0.15, -1.23, -0.03, 0.99, 1.16, -0.00019, -0.19),
  (14.51, 99.5, -0.07, -0.15, -1.23, -0.03, 0.99, 1.11, -0.00023, -0.19),
  (14.51, 98.1, 0.01, -0.12, -1.11, 0.01, 0.99, 0.93, -0.00023, -0.19),
  (14.5, 97.3, 0.07, -0.07, -1.11, 0.08, 0.99, 1.07, -0.00023, -0.18),
  (14.5, 97.3, 0.07, -0.07, -1.11, 0.08, 0.98, 1.08, -0.00023, -0.18),
  (14.49, 96.2, 0.06, -0.04, -0.86, 0.2, 0.98, 0.8, -0.00024, -0.18),
  (14.48, 95.1, 0.05, -0.01, -1.23, 0.23, 0.98, 1.19, -0.00024, -0.17),
  (14.48, 95.1, 0.05, -0.01, -1.23, 0.23, 0.98, 1.16, -0.00025, -0.17),
  (14.47, 93.9, 0.04, 0.01, -1.11, 0.33, 0.98, 0.89, -0.00025, -0.17),
  (14.46, 92.6, 0.03, 0.02, -1.11, 0.43, 0.99, 0.93, -0.00026, -0.17),
  (14.47, 92.6, 0.03, 0.02, -1.11, 0.43, 0.99, 0.89, -0.00026, -0.16),
  (14.46, 91.5, 0.03, 0.02, -1.23, 0.46, 0.98, 0.92, -0.00026, -0.16),
  (14.46, 91.5, 0.03, 0.02, -1.23, 0.46, 0.97, 0.89, -0.00023, -0.16),
  (14.45, 90.5, 0.02, 0.03, -1.11, 0.46, 0.98, 0.76, -0.00023, -0.16),
  (14.45, 89.2, 0.08, 0.04, -0.98, 0.53, 0.98, 0.59, -0.0002, -0.17),
  (14.44, 89.2, 0.08, 0.04, -0.98, 0.53, 0.98, 0.56, -0.00016, -0.17),
  (14.45, 88.1, 0.07, 0.05, -0.98, 0.53, 0.99, 0.53, -0.00013, -0.17),
  (14.45, 87.1, 0.07, 0.06, -1.23, 0.32, 0.99, 0.83, -0.00013, -0.17),
  (14.44, 87.1, 0.07, 0.06, -1.23, 0.32, 0.99, 0.83, -0.0001, -0.17),
  (14.44, 86.1, 0.06, 0.07, -1.48, 0.11, 0.99, 1.12, -3e-05, -0.17),
  (14.44, 84.7, 0.07, 0.07, -1.73, -0.13, 0.98, 1.35, 1e-05, -0.18),
  (14.44, 84.7, 0.07, 0.07, -1.73, -0.13, 0.99, 1.29, 0.0, -0.18),
  (14.43, 83.7, 0.06, 0.06, -1.73, -0.26, 0.99, 1.23, 4e-05, -0.18),
  (14.43, 82.8, 0.12, 0.08, -1.98, -0.56, 0.99, 1.46, 8e-05, -0.18),
  (14.42, 82.8, 0.12, 0.08, -1.98, -0.56, 0.99, 1.51, 8e-05, -0.18),
  (14.42, 81.6, 0.11, 0.08, -1.98, -0.71, 0.99, 1.5, 8e-05, -0.18),
  (14.42, 81.6, 0.11, 0.08, -1.98, -0.71, 0.99, 1.54, 0.00012, -0.18),
  (14.41, 80.3, 0.1, 0.08, -1.98, -0.94, 0.99, 1.56, 0.00015, -0.19),
  (14.42, 80.3, 0.1, 0.08, -1.98, -0.94, 0.99, 1.51, 0.00015, -0.19),
  (14.42, 79.0, 0.1, 0.08, -2.11, -1.08, 0.99, 1.67, 0.00018, -0.19),
  (14.41, 77.8, 0.17, 0.1, -1.98, -1.1, 0.99, 1.5, 0.00022, -0.19),
  (14.4, 77.8, 0.17, 0.1, -1.98, -1.1, 0.99, 1.51, 0.00022, -0.19),
  (14.39, 76.8, 0.22, 0.12, -1.98, -1.14, 0.99, 1.54, 0.00022, -0.19),
  (14.39, 76.8, 0.22, 0.12, -1.98, -1.14, 0.99, 1.57, 0.00026, -0.19),
  (14.38, 75.4, 0.2, 0.12, -1.98, -1.09, 0.99, 1.55, 0.00026, -0.2),
  (14.38, 74.3, 0.25, 0.15, -2.11, -0.98, 0.99, 1.66, 0.00025, -0.2),
  (14.37, 74.3, 0.25, 0.15, -2.11, -0.98, 0.99, 1.7, 0.00029, -0.2),
  (14.36, 73.2, 0.24, 0.16, -1.61, -0.68, 0.99, 1.21, 0.00029, -0.2),
  (14.36, 73.2, 0.24, 0.16, -1.61, -0.68, 0.99, 1.17, 0.00029, -0.2),
  (14.36, 72.1, 0.24, 0.16, -1.61, -0.33, 0.99, 1.27, 0.00029, -0.21),
  (14.36, 72.1, 0.24, 0.16, -1.61, -0.33, 0.99, 1.37, 0.00032, -0.21),
  (14.35, 70.7, 0.29, 0.18, -1.61, -0.15, 0.99, 1.42, 0.00033, -0.21),
  (14.34, 69.7, 0.29, 0.19, -1.36, 0.12, 0.99, 1.12, 0.00033, -0.11),
  (14.33, 69.7, 0.29, 0.19, -1.36, 0.12, 0.99, 1.1, 0.00033, 0.01),
  (14.33, 68.6, 0.39, 0.22, -1.23, 0.25, 0.99, 1.06, 0.00033, 0.14),
  (14.32, 67.6, 0.46, 0.25, -1.11, 0.4, 0.99, 0.96, 0.00032, 0.25),
  (14.32, 67.6, 0.46, 0.25, -1.11, 0.4, 0.99, 1.02, 0.00033, 0.36),
  (14.31, 66.2, 0.44, 0.27, -1.23, 0.45, 0.99, 1.12, 0.00033, 0.45),
  (14.3, 66.2, 0.44, 0.27, -1.23, 0.45, 0.99, 1.11, 0.00032, 0.54),
  (14.3, 64.9, 0.49, 0.29, -0.98, 0.54, 0.99, 0.93, 0.00032, 0.62),
  (14.29, 64.9, 0.49, 0.29, -0.98, 0.54, 0.99, 0.92, 0.00032, 0.69),
  (14.29, 63.8, 0.54, 0.32, -0.86, 0.64, 0.99, 0.76, 0.00032, 0.75),
  (14.3, 62.6, 0.6, 0.34, -0.86, 0.7, 1.0, 0.74, 0.00032, 0.8),
  (14.31, 62.6, 0.6, 0.34, -0.86, 0.7, 0.99, 0.74, 0.00032, 0.86),
  (14.31, 61.4, 0.62, 0.36, -0.98, 0.63, 0.99, 0.89, 0.00033, 0.9),
  (14.33, 60.3, 0.69, 0.39, -1.11, 0.54, 0.99, 1.05, 0.00032, 0.94),
  (14.34, 60.3, 0.69, 0.39, -1.11, 0.54, 0.99, 1.05, 0.00032, 0.98),
  (14.36, 59.2, 0.72, 0.41, -1.11, 0.33, 1.0, 1.04, 0.00032, 1.01),
  (14.41, 59.2, 0.72, 0.41, -1.11, 0.33, 1.0, 1.05, 0.00032, 1.04),
  (14.48, 58.2, 0.79, 0.44, -1.11, 0.19, 1.0, 1.05, 0.00028, 1.07),
  (14.52, 56.9, 0.85, 0.47, -0.98, 0.06, 1.0, 0.99, 0.00028, 1.08),
  (14.59, 56.9, 0.85, 0.47, -0.98, 0.06, 1.0, 1.0, 0.00028, 1.08),
  (14.68, 55.9, 1.03, 0.53, -0.86, -0.06, 1.0, 0.87, 0.00025, 1.08),
  (14.74, 55.9, 1.03, 0.53, -0.86, -0.06, 1.0, 0.89, 0.00025, 1.07),
  (14.83, 54.7, 1.2, 0.61, -0.98, -0.18, 1.0, 1.01, 0.00021, 1.07),
  (14.9, 53.6, 1.27, 0.61, -0.98, -0.3, 1.0, 0.98, 0.00021, 0.95),
  (14.95, 53.6, 1.27, 0.61, -0.98, -0.3, 1.0, 1.03, 0.00021, 0.82),
  (15.03, 52.4, 1.2, 0.61, -0.98, -0.42, 1.0, 1.02, 0.00017, 0.7),
  (15.11, 52.4, 1.2, 0.61, -0.98, -0.42, 1.0, 1.06, 0.00017, 0.57),
  (15.19, 51.1, 1.42, 0.72, -0.86, -0.56, 1.0, 0.99, 0.00014, 0.45),
  (15.28, 51.1, 1.42, 0.72, -0.86, -0.56, 1.0, 0.98, 0.00014, 0.32),
  (15.39, 50.0, 1.61, 0.84, -0.73, -0.58, 1.0, 0.87, 0.0001, 0.2),
  (15.45, 48.9, 1.75, 0.95, -0.86, -0.61, 1.0, 0.97, 7e-05, 0.07),
  (15.54, 48.9, 1.75, 0.95, -0.86, -0.61, 1.0, 0.97, 7e-05, -0.05),
  (15.59, 47.6, 1.98, 0.95, -0.86, -0.58, 1.0, 0.99, 7e-05, -0.18),
  (15.64, 47.6, 1.98, 0.95, -0.86, -0.58, 1.0, 1.02, 4e-05, -0.3),
)


def _replay_nov17():
  ctrl = UnifiedLeadController()
  v0, d0, vl0, al0, y0, _vlat0, p0, pl0, k0, _ = NOV17_ROWS[0]
  wanted = unified_follow_desired(
    d0, v0, vl0, al0, T_FOLLOW, v_ceiling=NOV17_V_CEILING, y_rel=y0, curvature=k0,
    path_lat=pl0, model_prob=p0, radar=True,
  )
  # Same first-lead seed the planner applies (_apply_lead_follow).
  if wanted >= 0.0:
    seed = min(NOV17_EXISTING_A, wanted)
  else:
    seed = min(NOV17_EXISTING_A, max(wanted, min(NOV17_EXISTING_A, 0.0) - NOV17_SEED_STEP))
  out = []
  for v, d, vl, al, y, vlat, p, pl, k, _logged in NOV17_ROWS:
    a = ctrl.step(
      dt=DT, present=True, gap=d, v_ego=v, v_lead=vl, a_lead=al, t_follow=T_FOLLOW,
      lead_id=("radar", NOV17_TRACK), seed_a=seed, v_ceiling=NOV17_V_CEILING,
      y_rel=y, curvature=k, path_lat=pl, model_prob=p, radar=True, v_lat=vlat, long_on=True,
    )
    out.append((a, bool(ctrl.crossing_classified), float(ctrl._cross.resume_w)))
  return out


def test_nov17_fixture_is_the_bad_case():
  logged = [r[-1] for r in NOV17_ROWS]
  assert max(logged) >= 1.0
  assert NOV17_ROWS[0][1] >= 125.0 and NOV17_ROWS[0][2] < 0.5
  assert max(abs(r[5]) for r in NOV17_ROWS[:20]) >= CROSS_VY_MS


def test_nov17_stopped_car_at_130m_is_not_a_crosser_and_never_accelerates():
  out = _replay_nov17()
  assert not any(c for _, c, _ in out), "stopped car classified as a crosser"
  assert all(w == 0.0 for _, _, w in out)
  cmds = [a for a, _, _ in out]
  assert max(cmds) <= 0.0, max(cmds)
  # Braking builds and is held: by 1 s in, and firmer as the gap shrinks.
  assert cmds[20] <= -1.0, cmds[20]
  assert max(cmds[20:]) <= -1.0, max(cmds[20:])
  assert cmds[-1] <= -2.0, cmds[-1]


def test_crossing_vy_required_grows_past_the_reference_range():
  assert crossing_vy_required(10.0) == pytest.approx(CROSS_VY_MS)
  assert crossing_vy_required(CROSS_VY_RANGE_REF_M) == pytest.approx(CROSS_VY_MS)
  assert crossing_vy_required(2.0 * CROSS_VY_RANGE_REF_M) == pytest.approx(2.0 * CROSS_VY_MS)
  # Nov 17 noise at 117–130 m stays under the requirement.
  assert crossing_vy_required(117.0) > 0.95


def test_stop_guard_needs_a_real_stop_and_an_unverified_crosser():
  need = -1.5
  assert not crossing_stop_guard_active(-CROSS_GUARD_NEED_MS2 + 0.05, 0.99, True, 0.0, 1.0)
  assert crossing_stop_guard_active(need, 0.99, False, 0.0, 1.0)
  assert not crossing_stop_guard_active(need, 0.25, False, 0.0, 1.0)
  assert crossing_stop_guard_active(need, 0.25, True, 0.0, 1.0)
  assert crossing_stop_guard_active(need, 0.25, True, -1.0, 1.0)
  assert not crossing_stop_guard_active(need, 0.25, True, CROSS_EXIT_VY_MS + 0.1, 1.0)


def _e1_handover(v_lat_of, prob_of):
  ctrl = UnifiedLeadController()
  rows = []
  for i in range(100):
    t = i * DT
    gap = max(8.0, 80.0 - V_EGO * t)
    a = ctrl.step(
      dt=DT, present=True, gap=gap, v_ego=V_EGO, v_lead=0.2, a_lead=0.0, t_follow=T_FOLLOW,
      lead_id=2842, seed_a=0.0, v_ceiling=V_CEIL, y_rel=_e1_y(t), radar=True,
      model_prob=prob_of(t), v_lat=v_lat_of(t), long_on=True,
    )
    need = approach_physics_need(gap, V_EGO, ctrl.last_v_lead, T_FOLLOW)
    rows.append((t, a, bool(ctrl._cross.exit), need, float(ctrl.last_desired), bool(ctrl.crossing_stop_guard)))
  return rows


def test_unverified_exit_handover_does_not_accelerate_toward_a_stop():
  """Exit flagged from y wander alone, no ground lateral speed: no +a, no softer than the stop."""
  rows = _e1_handover(lambda t: _vlat_at(t, _e1_y) if t < 3.4 else 0.0, lambda t: 0.25)
  exits = [r for r in rows if r[2]]
  assert exits, "exit never flagged"
  t0 = exits[0][0]
  later = [r for r in rows if r[0] >= t0 + 0.1]
  assert later
  assert max(r[1] for r in later) <= 0.0
  for _t, _a, _e, need, desired, guard in later:
    if need <= -CROSS_GUARD_NEED_MS2:
      assert guard
      # The target is never softer than the stop; the command only slews toward it.
      assert desired <= need + 1e-6, (desired, need)
  for prev, cur in zip(later, later[1:], strict=False):
    assert cur[1] <= prev[1] + 1e-6, (prev[1], cur[1])


def test_model_lead_during_handover_returns_to_braking_at_once():
  rows = _e1_handover(lambda t: _vlat_at(t, _e1_y), lambda t: 0.25 if t < 4.1 else 0.95)
  after = [r[1] for r in rows if r[0] >= 4.1]
  assert max(after) <= 0.0
  assert after[3] < after[0] - 0.25


def test_model_lead_past_55m_clears_the_crossing_class():
  """A classified return the model then calls an in-path lead at 80–100 m is a stopped car."""
  def run(prob_after):
    ctrl = UnifiedLeadController()
    seen = []
    for i in range(50):
      t = i * DT
      gap = 100.0 - V_EGO * t
      ctrl.step(
        dt=DT, present=True, gap=gap, v_ego=V_EGO, v_lead=0.2, a_lead=0.0, t_follow=T_FOLLOW,
        lead_id=2842, seed_a=0.0, v_ceiling=V_CEIL, y_rel=_e1_y(t), radar=True,
        model_prob=0.25 if t < 1.2 else prob_after, v_lat=_vlat_at(t, _e1_y), long_on=True,
      )
      seen.append((t, gap, bool(ctrl.crossing_classified)))
    return seen

  crosser = run(0.25)
  assert any(c for t, _, c in crosser if t < 1.2), "sweep at 100 m never classified"
  assert all(c for t, _, c in crosser if 1.3 <= t <= 2.4)
  lead = run(0.95)
  late = [(gap, c) for t, gap, c in lead if 1.3 <= t <= 2.4]
  assert late and min(gap for gap, _ in late) > 55.0
  assert not any(c for _, c in late)
