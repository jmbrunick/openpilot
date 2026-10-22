from openpilot.selfdrive.speedsignd import strip_geom as G

RPY_LOW_HORIZON = [0.0007, -0.0757, -0.0195]   # Oct 22 000001bb liveCalibration


def test_default_lift_keeps_current_geometry():
  assert G.MAIN_LIFT_PX == 0
  main, _ = G.strips(1928, 1208, rpy=RPY_LOW_HORIZON)
  assert main == (964, 645, 964, 350)


def test_lift_moves_main_up_same_size(monkeypatch):
  monkeypatch.setattr(G, "MAIN_LIFT_PX", 120)
  main, _ = G.strips(1928, 1208, rpy=RPY_LOW_HORIZON)
  assert main == (964, 525, 964, 350)       # same area and scale -> same proposer cost
  monkeypatch.setattr(G, "MAIN_LIFT_PX", 5000)
  assert G.strips(1928, 1208, rpy=RPY_LOW_HORIZON)[0][1] == 0
