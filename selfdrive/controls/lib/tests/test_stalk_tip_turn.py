from openpilot.selfdrive.controls.lib.stalk_tip_turn import STALK_TIP_HOLD_S, StalkTipTurn

DT_CTRL = 0.01
DT_MDL = 0.05


def _hold(s, stalk, t, dt):
  n = int(round(t / dt))
  for _ in range(n):
    s.update(stalk, dt)


def test_tip_hold_window_is_documented_040s():
  assert STALK_TIP_HOLD_S == 0.40


def test_left_then_idle_within_window_is_tip():
  s = StalkTipTurn()
  _hold(s, 1, 0.20, DT_CTRL)
  assert s.is_pending
  assert not s.is_turn
  s.update(0, DT_CTRL)
  assert s.tip_event
  assert s.tip_direction == 1
  assert not s.is_turn
  assert not s.is_pending


def test_right_then_idle_within_window_is_tip():
  s = StalkTipTurn()
  s.update(2, DT_MDL)
  s.update(0, DT_MDL)
  assert s.tip_event
  assert s.tip_direction == 2


def test_held_past_window_is_turn_not_tip():
  s = StalkTipTurn()
  _hold(s, 1, STALK_TIP_HOLD_S, DT_CTRL)
  assert s.is_turn
  assert not s.is_pending
  s.update(0, DT_CTRL)
  assert not s.tip_event
  assert not s.is_turn


def test_boundary_just_under_window_is_still_tip():
  s = StalkTipTurn()
  # 0.35s < 0.40s
  _hold(s, 1, STALK_TIP_HOLD_S - 0.05, DT_CTRL)
  assert not s.is_turn
  s.update(0, DT_CTRL)
  assert s.tip_event


def test_sna_is_idle():
  s = StalkTipTurn()
  s.update(1, DT_CTRL)
  s.update(3, DT_CTRL)
  assert s.tip_event
  assert s.direction == 0


def test_model_rate_hold_crosses_window():
  s = StalkTipTurn()
  n = int(round(STALK_TIP_HOLD_S / DT_MDL))
  for _ in range(n):
    s.update(1, DT_MDL)
  assert s.is_turn


def test_direction_change_restarts_hold():
  s = StalkTipTurn()
  _hold(s, 1, 0.30, DT_CTRL)
  assert not s.is_turn
  s.update(2, DT_CTRL)
  assert s.right_press
  assert s.is_pending
  assert not s.is_turn
  assert s.held_s == DT_CTRL
