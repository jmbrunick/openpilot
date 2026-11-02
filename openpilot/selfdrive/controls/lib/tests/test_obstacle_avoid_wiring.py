"""controlsd wiring for the Oct 14 steer-zone avoid shift (source checks)."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]


def test_controlsd_applies_avoid_after_cone_and_before_low_vis():
  src = (ROOT / "selfdrive/controls/controlsd.py").read_text()
  i_cone = src.index("model_or_plan_curvature = self._cone_curvature(")
  i_avoid = src.index("model_or_plan_curvature = self._obstacle_avoid_curvature(")
  i_fade = src.index("model_or_plan_curvature = fade_curvature(")
  assert i_cone < i_avoid < i_fade
  assert "gas_pressed=bool(CS.gasPressed)" in src
  assert "steering_pressed=bool(CS.steeringPressed)" in src
  assert "cs.obstacleAvoidOffset" in src
  # No brake / longitudinal output from the steering feature.
  body = src[src.index("def _obstacle_avoid_curvature"):src.index("def state_control")]
  assert "accel" not in body and "LoC" not in body


def test_schema_claims_the_fields():
  log = (ROOT / "cereal/log.capnp").read_text()
  assert "obstacleAvoidOffset @72 :Float32;" in log
  custom = (ROOT / "cereal/custom.capnp").read_text()
  for f in ("rangeM @20", "lateral @21", "spanM @22", "vRel @23"):
    assert f in custom
