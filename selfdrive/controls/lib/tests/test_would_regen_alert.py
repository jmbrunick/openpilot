from types import SimpleNamespace

from openpilot.selfdrive.controls.lib import longitudinal_planner as lp


def test_would_regen_alert_rising_edge(monkeypatch):
  events = []
  import openpilot.common.swaglog as sw
  monkeypatch.setattr(sw.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  me = SimpleNamespace()
  obj = SimpleNamespace(range_m=42.0, lateral=0.1, object_class="obstacle")
  f = lp.LongitudinalPlanner._would_regen_alert_log
  for w in (False, True, True, False, True):
    f(me, w, obj, 18.0)
  assert [e[0] for e in events] == ["wouldRegenAlert", "wouldRegenAlert"]
  assert events[0][1]["range_m"] == 42.0 and events[0][1]["commanded"] is False
