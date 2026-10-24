from openpilot.selfdrive.speedsignd.night_exposure import NightExposure, WINDOW_S


class P:
  def __init__(self, v): self.v = v
  def get_bool(self, k): return self.v


def test_off_writes_nothing(tmp_path):
  p = tmp_path / "r"
  ne = NightExposure(P(False), str(p))
  assert not ne.update(True, 10.0) and not p.exists()


def test_on_with_track_writes_deadline(tmp_path):
  p = tmp_path / "r"
  ne = NightExposure(P(True), str(p))
  assert not ne.update(False, 10.0)
  assert ne.update(True, 10.0)
  assert int(p.read_text()) == int((10.0 + WINDOW_S) * 1e9)
  assert not ne.update(True, 10.2)  # rate limited
  assert ne.update(True, 10.6)
