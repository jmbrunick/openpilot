"""Synthetic red/green sequences for the stoplight tracker and gate."""
from openpilot.selfdrive.stoplightd import tracker as tk
from openpilot.selfdrive.stoplightd.detect import GREEN, RED, Lamp
from openpilot.selfdrive.stoplightd.gate import StoppedInDrive, in_drive_from_events, speed_from_pose


def red(x=1000.0, y=300.0, size=24.0, chroma=90.0):
  return Lamp(RED, x, y, size, size, 120, 40, chroma)


def green(x=1000.0, y=355.0, size=24.0, chroma=90.0):
  return Lamp(GREEN, x, y, size, size, 120, 40, chroma)


def run(frames, dt=0.25, t0=10.0):
  tr = tk.StoplightTracker()
  return [tr.update(t0 + i * dt, f) for i, f in enumerate(frames)]


def test_red_then_green_dings_once():
  outs = run([[red()]] * 8 + [[green()]] * 8)
  assert sum(o.chime for o in outs) == 1
  assert outs[7].red
  assert outs[-1].green and not outs[-1].red


def test_red_needs_confirm_before_flag():
  outs = run([[red()]] * 3)
  assert not any(o.red for o in outs)


def test_green_without_red_is_quiet():
  outs = run([[]] * 4 + [[green()]] * 8)
  assert not any(o.chime for o in outs)


def test_green_that_was_already_lit_is_quiet():
  # Another head (or a sign) is green the whole time; our red just goes out.
  outs = run([[red(), green(x=1300)]] * 8 + [[green(x=1300)]] * 8)
  assert not any(o.chime for o in outs)


def test_red_out_without_green_is_quiet():
  # Right turn on red / brake lights going out.
  outs = run([[red()]] * 8 + [[]] * 8)
  assert not any(o.chime for o in outs)


def test_green_elsewhere_is_quiet():
  outs = run([[red()]] * 8 + [[green(x=1600, y=300)]] * 8)
  assert not any(o.chime for o in outs)


def test_green_above_red_is_quiet():
  outs = run([[red()]] * 8 + [[green(y=245)]] * 8)
  assert not any(o.chime for o in outs)


def test_horizontal_head_dings():
  outs = run([[red()]] * 8 + [[green(x=1055, y=300)]] * 8)
  assert sum(o.chime for o in outs) == 1


def test_billboard_like_shift_is_quiet():
  outs = run([[red(size=26)]] * 8 + [[green(x=1015, y=303, size=12)]] * 8)
  assert not any(o.chime for o in outs)


def test_late_green_is_quiet():
  # Red went out, nothing for 2 s, then a green: not the same change.
  outs = run([[red()]] * 8 + [[]] * 8 + [[green()]] * 6)
  assert not any(o.chime for o in outs)


def test_flicker_does_not_ding():
  # One dropped red frame and a one-frame green blip are not a change.
  frames = [[red()]] * 6 + [[]] + [[red()]] * 3 + [[green()]] + [[red()]] * 6
  outs = run(frames)
  assert not any(o.chime for o in outs)


def test_dull_lamps_are_ignored():
  outs = run([[red(chroma=40)]] * 8 + [[green(chroma=50)]] * 8)
  assert not any(o.red for o in outs)
  assert not any(o.chime for o in outs)


def test_reset_allows_next_stop():
  tr = tk.StoplightTracker()
  t = 0.0
  chimes = 0
  for _stop in range(2):
    for f in [[red()]] * 8 + [[green()]] * 6:
      chimes += tr.update(t, f).chime
      t += 0.25
    tr.reset()
  assert chimes == 2


def test_negative_times_work():
  outs = run([[red()]] * 8 + [[green()]] * 8, t0=-30.0)
  assert sum(o.chime for o in outs) == 1


def test_scale_for_small_sensor():
  s = 1344.0 / tk.REF_WIDTH
  tr = tk.StoplightTracker(scale=s)
  outs = []
  for i, f in enumerate([[red(x=700, y=210, size=17)]] * 8 + [[green(x=700, y=248, size=17)]] * 6):
    outs.append(tr.update(i * 0.25, f))
  assert sum(o.chime for o in outs) == 1


def test_gate_opens_after_hold_and_closes_on_move():
  g = StoppedInDrive()
  t = 0.0
  states = []
  for i in range(12):
    g.on_pose(t, 0.01)
    if i % 4 == 0:
      g.on_events(t, [])
    states.append(g.update(t))
    t += 0.25
  assert not states[0] and states[-1]
  g.on_pose(t, 1.5)
  assert not g.update(t)


def test_gate_creep_hysteresis():
  g = StoppedInDrive()
  for i in range(8):
    g.on_pose(i * 0.25, 0.0)
    g.on_events(i * 0.25, [])
    g.update(i * 0.25)
  assert g.open
  g.on_pose(2.0, 0.5)   # creeping in the queue, under MOVE_V
  assert g.update(2.0)


def test_gate_needs_drive():
  g = StoppedInDrive()
  for i in range(8):
    g.on_pose(i * 0.25, 0.0)
    g.on_events(i * 0.25, ["wrongGear"])
    assert not g.update(i * 0.25)
  assert not in_drive_from_events(["reverseGear"])
  assert in_drive_from_events(["gasPressedOverride"])


def test_gate_stale_inputs_close():
  g = StoppedInDrive()
  for i in range(8):
    g.on_pose(i * 0.25, 0.0)
    g.on_events(i * 0.25, [])
    g.update(i * 0.25)
  assert g.open
  assert not g.update(10.0)


def test_speed_from_pose():
  assert speed_from_pose(3.0, 4.0) == 5.0
  assert speed_from_pose(1.0, 0.0, valid=False) is None
  assert speed_from_pose(float("nan"), 0.0) is None
