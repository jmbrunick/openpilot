"""Turn-in delay trim mutation definitions (see tesla_preap_longitudinal_mutations.py).

NAPTurnInDelay (steps -2..+3, 30 ms each) trims the low-speed turn-in on top of
the turn geometry correction. Each entry breaks one rule of the trim (default
0 identical, bounded, low-speed band only, slewed, wired into modeld and the
Driving Mannerisms menu) and must fail the pinned tests with an AssertionError.
"""
from tesla_preap_mutation_common import (
  HistoricalMutation,
  TURN_GEOM_TEST_PATH,
)

TG = "selfdrive/controls/lib/lat_turn_geometry.py"
MODELD = "selfdrive/modeld/modeld.py"
MENU = "selfdrive/ui/layouts/settings/driving_mannerisms.py"
T = TURN_GEOM_TEST_PATH


def _m(name, path, original, replacement, *tests):
  return HistoricalMutation(
    name=name, source_path=path, original=original, replacement=replacement,
    test_nodes=tuple(f"{T}::{t}" for t in tests),
  )


MUTATIONS_E = (
  _m("turn-in-step-clamp-removed", TG,
     b"  return int(min(TURN_IN_STEP_MAX, max(TURN_IN_STEP_MIN, round(v))))\n",
     b"  return int(round(v))\n",
     "test_turn_in_step_is_clamped_and_junk_is_zero"),
  _m("turn-in-junk-step-not-zero", TG,
     b"    return 0\n  if not np.isfinite(v):\n    return 0\n",
     b"    return 0\n  if not np.isfinite(v):\n    return 1\n",
     "test_turn_in_step_is_clamped_and_junk_is_zero"),
  _m("turn-in-step-size-doubled", TG,
     b"TURN_IN_STEP_S = 0.030 ", b"TURN_IN_STEP_S = 0.060 ",
     "test_turn_in_param_defaults_to_zero_int", "test_turn_in_each_step_is_30ms_in_the_full_band"),
  _m("turn-in-max-step-raised", TG,
     b"TURN_IN_STEP_MAX = 3\n", b"TURN_IN_STEP_MAX = 5\n",
     "test_turn_in_param_defaults_to_zero_int", "test_turn_in_step_is_clamped_and_junk_is_zero"),
  _m("turn-in-min-lookahead-lowered", TG,
     b"TURN_IN_MIN_LOOKAHEAD_S = 0.10 ", b"TURN_IN_MIN_LOOKAHEAD_S = 0.01 ",
     "test_turn_in_param_defaults_to_zero_int", "test_turn_in_lookahead_is_bounded"),
  _m("turn-in-band-not-faded-by-10mph", TG,
     b"  return low_speed_reach_weight(v_ego) * speed_fade(v_ego)\n",
     b"  return speed_fade(v_ego)\n",
     "test_turn_in_trim_never_changes_10mph_and_up_or_below_1p5ms",
     "test_turn_in_band_fades_out_by_10mph_and_in_between_1p5_and_2p5"),
  _m("turn-in-band-active-at-standstill", TG,
     b"  return low_speed_reach_weight(v_ego) * speed_fade(v_ego)\n",
     b"  return low_speed_reach_weight(v_ego)\n",
     "test_turn_in_trim_never_changes_10mph_and_up_or_below_1p5ms",
     "test_turn_in_band_fades_out_by_10mph_and_in_between_1p5_and_2p5"),
  _m("turn-in-shift-sign-flipped", TG,
     b"  return clamp_turn_in_step(step) * TURN_IN_STEP_S * turn_in_weight(v_ego)\n",
     b"  return -clamp_turn_in_step(step) * TURN_IN_STEP_S * turn_in_weight(v_ego)\n",
     "test_turn_in_each_step_is_30ms_in_the_full_band"),
  _m("turn-in-step-not-clamped-in-shift", TG,
     b"  return clamp_turn_in_step(step) * TURN_IN_STEP_S * turn_in_weight(v_ego)\n",
     b"  return float(step) * TURN_IN_STEP_S * turn_in_weight(v_ego)\n",
     "test_turn_in_band_fades_out_by_10mph_and_in_between_1p5_and_2p5"),
  _m("turn-in-later-floor-removed", TG,
     b"    return max(min(L, TURN_IN_MIN_LOOKAHEAD_S), L - shift)\n",
     b"    return L - shift\n",
     "test_turn_in_lookahead_is_bounded"),
  _m("turn-in-earlier-above-stock", TG,
     b"  return min(max(L, float(stock_lookahead_s)), L - shift)\n",
     b"  return L - shift\n",
     "test_turn_in_lookahead_is_bounded"),
  _m("turn-in-sample-floor-not-trimmed", TG,
     b"    self.sample_floor_s = float(min(MIN_STABLE_DELAY, max(TURN_IN_MIN_LOOKAHEAD_S, trim_floor)))\n",
     b"    pass\n",
     "test_turn_in_moves_the_sample_floor_with_the_lookahead"),
  _m("turn-in-plan-sampling-floor-restored", TG,
     b"    t = max(float(action_t), floor, TURN_IN_MIN_LOOKAHEAD_S)\n",
     b"    t = max(float(action_t), floor, LOW_SPEED_SAMPLE_FLOOR_S)\n",
     "test_turn_in_moves_the_sample_floor_with_the_lookahead"),
  _m("turn-in-trim-not-slewed", TG,
     b"    self.trim_s += float(np.clip(shift_target - self.trim_s, -trim_step, trim_step))\n",
     b"    self.trim_s = shift_target\n",
     "test_turn_in_trim_is_slewed_not_stepped"),
  _m("turn-in-trim-kept-when-geometry-off", TG,
     b"  def reset(self) -> None:\n    self.trim_s = 0.0\n",
     b"  def reset(self) -> None:\n",
     "test_turn_in_trim_needs_turn_geometry_on"),
  _m("turn-in-default-step-one", "common/params_keys.h",
     b'{"NAPTurnInDelay", {PERSISTENT, INT, "0"}}', b'{"NAPTurnInDelay", {PERSISTENT, INT, "1"}}',
     "test_turn_in_param_defaults_to_zero_int"),
  _m("turn-in-modeld-drops-the-step", MODELD,
     b"None,\n          turn_in_step=turn_in_step)\n", b"None)\n",
     "test_turn_in_trim_is_wired_into_modeld_and_the_menu"),
  _m("turn-in-modeld-reads-wrong-param", MODELD,
     b"clamp_turn_in_step(params.get(PARAM_TURN_IN_DELAY, return_default=True))",
     b"clamp_turn_in_step(params.get(PARAM_REF_OFFSET, return_default=True))",
     "test_turn_in_trim_is_wired_into_modeld_and_the_menu"),
  _m("turn-in-menu-row-not-added", MENU,
     b"    self._all_items.append(self._turn_in_buttons)\n", b"    pass\n",
     "test_turn_in_trim_is_wired_into_modeld_and_the_menu"),
  _m("turn-in-menu-writes-the-button-index", MENU,
     b"int(TURN_IN_DELAY_STEPS[index])", b"int(index)",
     "test_turn_in_trim_is_wired_into_modeld_and_the_menu"),
)
