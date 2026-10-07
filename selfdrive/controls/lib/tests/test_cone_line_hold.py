"""Soft-handoff offset hold near a cone line, and the Ridgewood-style re-take.

The detector staying log-only is the case with no driver push: curvature
is the model curvature exactly. Longitudinal is not in this module.
"""

from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.cone_line_hold import (
  CLEAR_S,
  PARAM_CONE_LINE_HOLD,
  PARAM_CONE_LINE_LOG,
  PUSH_ARM_S,
  RELEASE_S,
  ConeLineHold,
  lookahead_m,
)
from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  HANDS_OFF_CONFIRM_S,
  RELEASE_HOLD_S,
  DriverLateralHandoff,
  apply_lat_authority,
)
from openpilot.selfdrive.controls.lib.radar_path_gate import path_y_at_x

DT = 0.01
V = 15.0
# Model path bends right (toward the cones) by 0.55 m at the hold lookahead.
Y_PULL = -0.55


def _path(x_la):
  return [0.0, x_la, x_la * 2.0], [0.0, Y_PULL, Y_PULL * 2.0]


def _cone(active=True, side=-1):
  return SimpleNamespace(active=active, side=side)


def _model_k(x_la):
  return 2.0 * Y_PULL / (x_la * x_la)


def _step(hold, *, torque=0.0, measured_k=0.0, model_k=None, yielded=False,
          lat_active=True, enabled=True, engaged=True, cone=None, seconds=DT,
          v=V, path=None):
  x_la = lookahead_m(v)
  path_x, path_y = _path(x_la) if path is None else path
  if model_k is None:
    model_k = _model_k(x_la)
  if cone is None:
    cone = _cone()
  n = max(1, int(round(seconds / DT)))
  out = None
  for _ in range(n):
    out = hold.update(
      enabled=enabled, engaged=engaged, cone=cone, torque_nm=torque,
      measured_k=measured_k, model_k=model_k, path_x=path_x, path_y=path_y,
      v_ego=v, yielded=yielded, lat_active=lat_active, dt=DT,
    )
  return out


def test_log_only_line_does_not_change_curvature_without_a_push():
  hold = ConeLineHold()
  x_la = lookahead_m(V)
  model_k = _model_k(x_la)
  out = _step(hold, torque=0.0, seconds=4.0, lat_active=True, yielded=False)
  assert out.active is False
  assert out.offset_m == 0.0
  assert out.curvature == model_k
  assert not hold.busy


def test_push_toward_the_cones_does_not_arm():
  hold = ConeLineHold()
  # Cones on the right. Torque -1 Nm is toward them, not away.
  out = _step(hold, torque=-1.0, seconds=PUSH_ARM_S + 1.0, yielded=True, lat_active=False)
  assert out.active is False
  assert out.curvature == _model_k(lookahead_m(V))
  released = _step(hold, torque=0.0, seconds=HANDS_OFF_CONFIRM_S + 0.2, yielded=False, lat_active=True)
  assert released.active is False
  assert released.curvature == _model_k(lookahead_m(V))


def test_hold_toggle_off_is_identity():
  hold = ConeLineHold()
  out = _step(hold, torque=1.0, seconds=PUSH_ARM_S + 0.5, yielded=True, lat_active=False, enabled=False)
  assert out.curvature == _model_k(lookahead_m(V))
  assert out.active is False


def test_offset_holds_on_retake_and_releases_after_the_line_clears():
  hold = ConeLineHold()
  model_k = _model_k(lookahead_m(V))
  # Driver holds a straight line (measured curvature 0) while yielded.
  pushing = _step(hold, torque=1.0, seconds=PUSH_ARM_S + 0.3, yielded=True, lat_active=False)
  assert pushing.active is False
  assert pushing.curvature == model_k  # yielded: do not steer
  retake = _step(hold, torque=0.0, seconds=0.2, yielded=False, lat_active=True)
  assert retake.active is True
  assert retake.offset_m == pytest.approx(-Y_PULL, abs=0.03)
  assert retake.curvature == pytest.approx(0.0, abs=1e-6)
  assert retake.curvature > model_k  # less toward the cones than the model
  # Line still there: keep the driver's line.
  still = _step(hold, torque=0.0, seconds=3.0, yielded=False, lat_active=True)
  assert still.active is True
  assert still.curvature == pytest.approx(0.0, abs=1e-6)
  # Cleared, but the wait has not finished.
  waiting = _step(hold, torque=0.0, seconds=CLEAR_S * 0.5, yielded=False, lat_active=True,
                  cone=_cone(active=False, side=0))
  assert waiting.curvature == pytest.approx(0.0, abs=1e-4)
  # Through the clear wait and the decay, the command eases onto the model
  # and never goes past it toward the cones.
  mid = _step(hold, torque=0.0, seconds=CLEAR_S * 0.5 + RELEASE_S * 0.5,
              yielded=False, lat_active=True, cone=_cone(active=False, side=0))
  assert model_k <= mid.curvature <= 0.05
  done = _step(hold, torque=0.0, seconds=RELEASE_S, yielded=False, lat_active=True,
               cone=_cone(active=False, side=0))
  assert done.active is False
  assert done.curvature == pytest.approx(model_k, abs=1e-6)


def test_steer_back_releases_even_while_the_line_remains():
  hold = ConeLineHold()
  model_k = _model_k(lookahead_m(V))
  _step(hold, torque=1.0, seconds=PUSH_ARM_S + 0.3, yielded=True, lat_active=False)
  held = _step(hold, torque=0.0, seconds=0.2, yielded=False, lat_active=True)
  assert held.curvature == pytest.approx(0.0, abs=1e-6)
  _step(hold, torque=-0.8, seconds=0.4, yielded=False, lat_active=True)
  done = _step(hold, torque=0.0, seconds=RELEASE_S + 0.2, yielded=False, lat_active=True)
  assert done.active is False
  assert done.curvature == pytest.approx(model_k, abs=1e-4)


def test_ridgewood_retake_blends_toward_the_driver_line():
  """Route-shaped: cones on the right, model path back toward them, driver held left.

  Approximates 1c95345a3286a5db segs 66–68. The re-take blend must not aim
  at the model curvature. A full-control yield still clears latActive; this
  test does not change that state machine.
  """
  x_la = lookahead_m(V)
  path_x, path_y = _path(x_la)
  assert path_y_at_x(path_x, path_y, x_la) == pytest.approx(Y_PULL, abs=1e-6)
  model_k = _model_k(x_la)
  hold = ConeLineHold()
  handoff = DriverLateralHandoff()
  cone = _cone()

  def both(*, torque, hands, yielded_lat=None):
    ho = handoff.update(
      engaged=True, lat_would_be_active=True, steering_torque=torque,
      steering_rate_deg=0.0, hands_on_level=hands, dt=DT,
      model_curvature=model_k, measured_curvature=0.0,
    )
    lat_active = (not ho.yielded) if yielded_lat is None else yielded_lat
    co = hold.update(
      enabled=True, engaged=True, cone=cone, torque_nm=torque,
      measured_k=0.0, model_k=model_k, path_x=path_x, path_y=path_y,
      v_ego=V, yielded=ho.yielded, lat_active=lat_active and not ho.yielded, dt=DT,
    )
    return ho, co

  ho = co = None
  for _ in range(int((PUSH_ARM_S + 0.4) / DT)):
    ho, co = both(torque=1.0, hands=1)
  assert ho.yielded is True
  assert co.active is False
  assert co.curvature == model_k

  for _ in range(int((RELEASE_HOLD_S + 0.05) / DT)):
    ho, co = both(torque=0.0, hands=0)
  assert ho.yielded is False
  assert ho.blending is True
  assert co.active is True
  assert co.curvature == pytest.approx(0.0, abs=1e-6)
  # Authority blend from the wheel (0) toward the hold target stays off the model path.
  for authority in (0.0, 0.25, ho.authority, 1.0):
    _torque, _angle, blended = apply_lat_authority(authority, 0.0, 0.0, 0.0, co.curvature, 0.0)
    assert blended == pytest.approx(0.0, abs=1e-6)
    assert blended > model_k + 0.0005


def test_yield_machine_and_longitudinal_are_untouched():
  root = Path(__file__).resolve().parents[4]
  handoff = (root / "selfdrive/controls/lib/driver_lateral_handoff.py").read_text()
  assert "cone" not in handoff.lower()
  controls = (root / "selfdrive/controls/controlsd.py").read_text()
  assert "actuators.accel = float(self.LoC.update(" in controls
  assert "_cone_curvature" in controls
  # The curvature adjust sits after the longitudinal command is already written.
  assert controls.index("actuators.accel = float(self.LoC.update(") < controls.index(
    "model_or_plan_curvature = self._cone_curvature(")
  assert "steering_pressed=bool(CS.steeringPressed),\n      model_curvature=float(self._raw_model_curvature)," in controls
  cone = (root / "selfdrive/controls/lib/cone_line.py").read_text()
  hold = (root / "selfdrive/controls/lib/cone_line_hold.py").read_text()
  assert "actuators" not in cone
  assert "LoC" not in hold
  assert "leadOne" not in cone
  keys = (root / "common/params_keys.h").read_text()
  hold_line = next(ln for ln in keys.splitlines() if f'"{PARAM_CONE_LINE_HOLD}"' in ln)
  log_line = next(ln for ln in keys.splitlines() if f'"{PARAM_CONE_LINE_LOG}"' in ln)
  assert 'BOOL' in hold_line and '"1"' in hold_line
  assert 'BOOL' in log_line and '"1"' in log_line
  services = (root / "cereal/services.py").read_text()
  assert '"coneLineNAP": (True, 8., 2)' in services
  ui = (root / "selfdrive/ui/layouts/settings/nap_content.py").read_text()
  assert "Hold my line" in ui or "CONE_LINE_HOLD_DESCRIPTION" in ui
  assert "NAPConeLineHold" in ui
  manner = (root / "selfdrive/ui/layouts/settings/driving_mannerisms.py").read_text()
  assert "Hold my line near cones" in manner
  hidden = (root / "selfdrive/ui/layouts/settings/hidden_toggles.py").read_text()
  assert "Cone line log" in hidden
  radard = (root / "selfdrive/controls/radard.py").read_text()
  assert "_update_cone_line" in radard
  assert "self.radar_state.leadOne" in radard
