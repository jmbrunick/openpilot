"""Shared paths and the HistoricalMutation record for the Pre-AP mutation gate."""
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
LONGCONTROL_TEST_PATH = "selfdrive/controls/tests/test_tesla_preap_longcontrol.py"
FOLLOWING_TEST_PATH = "selfdrive/controls/tests/test_tesla_preap_following.py"
GAS_LIFT_TEST_PATH = "selfdrive/controls/tests/test_tesla_preap_gas_lift_handoff.py"
BRAKE_CANCEL_TEST_PATH = (
  "selfdrive/controls/tests/test_tesla_preap_brake_cancel_regen.py"
)
UNIFIED_TEST_PATH = "selfdrive/controls/tests/test_unified_lead.py"
UNIFIED_PLANNER_TEST_PATH = "selfdrive/controls/tests/test_unified_lead_planner.py"
CURVE_PREVIEW_TEST_PATH = "selfdrive/controls/tests/test_curve_preview.py"
CURVE_MAX_HOLD_TEST_PATH = "selfdrive/controls/lib/tests/test_curve_max_hold.py"
LEAD_LEAVING_TEST_PATH = "selfdrive/controls/tests/test_lead_leaving.py"
TURN_GEOM_TEST_PATH = "selfdrive/controls/lib/tests/test_lat_turn_geometry.py"
LC_REPLAY_TEST_PATH = "selfdrive/controls/lib/tests/test_lane_change_confirm_replay.py"
LC_NUDGE_TEST_PATH = "selfdrive/controls/lib/tests/test_lane_change_nudge.py"
OFFSET_TEST_PATH = "selfdrive/mapd/tests/test_map_speed_offset_slew.py"
ENGAGE_MAX_TEST_PATH = "selfdrive/car/tests/test_preap_engage_max_after_pause.py"
FRONTAGE_TEST_PATH = "selfdrive/mapd/tests/test_map_match_frontage.py"
LANE_CHANGE_TARGET_TEST_PATH = "selfdrive/controls/lib/tests/test_lane_change_target.py"
LC_TURN_TEST_PATH = "selfdrive/controls/lib/tests/test_lane_change_turn.py"
NOISE_GATE_TEST_NODE = (
  "opendbc_repo/opendbc/car/tesla/preap/tests/test_virtual_das.py::TestInnerPID::" +
  "test_sub_deadband_sign_changing_noise_does_not_accumulate_residual_authority"
)
INNER_DEADBAND_TEST_NODE = (
  "opendbc_repo/opendbc/car/tesla/preap/tests/test_virtual_das.py::TestInnerPID::" +
  "test_persistent_sub_deadband_error_earns_residual_authority"
)


@dataclass(frozen=True)
class HistoricalMutation:
  name: str
  source_path: str
  original: bytes
  replacement: bytes
  test_nodes: tuple[str, ...]

