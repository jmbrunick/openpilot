from types import SimpleNamespace

from openpilot.selfdrive.nap.path_object.phantom import PhantomTracker

MD = SimpleNamespace(latitude=44.53, longitude=-94.99, bearingDeg=4.0, overheadAheadM=0.0, overheadKind="")


def _obs(rng, seen=False, lights=False, cls="obstacle", tid=7, day=False):
  return SimpleNamespace(trackId=tid, rangeM=rng, visionEvaluated=True, roadLevelSeen=seen, roadLights=lights,
                         objectClass=cls, daylight=day)


def _drive(seen_close=False, cls="obstacle", stop=False, close=True):
  """Approach at 20 m/s from 80 m, 20 Hz; the track drops at 30 m (or never gets close)."""
  pt, ev, bm = PhantomTracker(), [], []
  t, rng = 0.0, 80.0

  def emit(n, e):
    ev.append((n, e))

  pt.update(t, 20.0, True, _obs(rng, cls=cls), True, MD, emit, lambda: bm.append(1))
  drop_at = 30.0 if close else 70.0
  while t < 8.0:
    t += 0.05
    v = 0.0 if stop and t > 1.0 else 20.0
    rng -= v * 0.05
    o = _obs(rng, seen=seen_close and rng <= 60, cls=cls) if rng > drop_at else None
    pt.update(t, v, False, o, o is not None, MD, emit, lambda: bm.append(1))
  return ev, bm


def test_phantom_fires_with_bookmark():
  ev, bm = _drive()
  assert [e[0] for e in ev] == ["phantomObjectNAP"] and bm == [1]
  e = ev[0][1]
  assert e["lat"] == 44.53 and e["heading_deg"] == 4.0 and e["close_grabs"] > 0 and e["daylight"] is False
  assert e["ranges"][0] == (0.0, 80.0) and len(e["ranges"]) > 5


def test_camera_saw_it_no_phantom():
  assert _drive(seen_close=True) == ([], [])


def test_never_checked_close_no_phantom():
  assert _drive(close=False) == ([], [])


def test_stopped_short_no_phantom():
  assert _drive(stop=True) == ([], [])


def test_people_and_animals_never_phantom():
  for cls in ("human", "animal"):
    assert _drive(cls=cls) == ([], [])


def test_overhead_kind_tagged():
  pt = PhantomTracker()
  md = SimpleNamespace(latitude=1.0, longitude=2.0, bearingDeg=3.0, overheadAheadM=85.0, overheadKind="bridge")
  pt.update(0.0, 20.0, True, _obs(80.0), True, md)
  assert pt.ep.kind == "bridge"
