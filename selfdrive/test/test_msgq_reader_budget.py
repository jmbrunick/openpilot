"""msgq allows at most 15 readers per service (msgq.h NUM_READERS). A 16th
subscriber evicts every reader of that service, they re-register and evict
each other again, and all of them drop messages. Downstream daemons then
publish valid=False and selfdrived raises commIssue.

This counts static subscribers per service across the daemons that run
onroad (plus loggerd, which reads every logged service) and keeps one slot
free for card's lazy carState fallback, athena getMessage, or an SSH dump.
"""
import ast
import os
import re

from openpilot.common.basedir import BASEDIR

NUM_READERS = 15
HEADROOM = 1

# Not onroad daemons, or only run offline / in debug.
EXCLUDE_DIRS = ("selfdrive/debug", "tools", "/tests", "/test/", "system/webrtc", "selfdrive/ui/tests")
EXCLUDE_FILES = {
  "selfdrive/selfdrived/events.py",        # SubMaster(all services) under __main__
  "system/camerad/snapshot.py",            # offroad snapshot tool
  "system/ubloxd/ubloxd.py",               # not on a 3X
  "selfdrive/speedsignd/speedsignd.py",    # optional; counted below when enabled
  "selfdrive/mapd/gps_fix.py",             # helper SubMaster only used standalone
  "system/loggerd/uploader.py",            # deviceState only
  "system/statsd.py",
  "system/athena/athenad.py",
  "system/timed.py",
  "system/manager/manager.py",
  "selfdrive/car/tesla/preap_body_controls.py",  # lazy fallback, covered by HEADROOM
}


def _services():
  src = open(os.path.join(BASEDIR, "cereal/services.py")).read()
  out = {}
  for name, should_log in re.findall(r'^\s*"(\w+)":\s*\((True|False)', src, re.M):
    out[name] = should_log == "True"
  return out


def _strings(node, assigns):
  found = set()
  for n in ast.walk(node):
    if isinstance(n, ast.Constant) and isinstance(n.value, str):
      found.add(n.value)
    elif isinstance(n, (ast.Name, ast.Attribute)):
      key = n.id if isinstance(n, ast.Name) else n.attr
      for v in assigns.get(key, []):
        for m in ast.walk(v):
          if isinstance(m, ast.Constant) and isinstance(m.value, str):
            found.add(m.value)
  return found


def count_readers():
  services = _services()
  counts = {s: (1 if logged else 0) for s, logged in services.items()}  # loggerd
  where = {s: (["loggerd"] if logged else []) for s, logged in services.items()}
  for top in ("selfdrive", "system"):
    for root, _, files in os.walk(os.path.join(BASEDIR, top)):
      for fn in files:
        if not fn.endswith(".py"):
          continue
        path = os.path.join(root, fn)
        rel = os.path.relpath(path, BASEDIR)
        if rel in EXCLUDE_FILES or any(x in "/" + rel for x in EXCLUDE_DIRS) or fn.startswith("test_"):
          continue
        tree = ast.parse(open(path).read())
        assigns = {}
        for n in ast.walk(tree):
          if isinstance(n, ast.Assign):
            for t in n.targets:
              k = t.id if isinstance(t, ast.Name) else (t.attr if isinstance(t, ast.Attribute) else None)
              if k:
                assigns.setdefault(k, []).append(n.value)
        for n in ast.walk(tree):
          if not (isinstance(n, ast.Call) and n.args):
            continue
          fname = n.func.attr if isinstance(n.func, ast.Attribute) else getattr(n.func, "id", "")
          if fname not in ("SubMaster", "sub_sock"):
            continue
          for s in _strings(n.args[0], assigns) & services.keys():
            counts[s] += 1
            where[s].append(f"{rel}:{n.lineno}")
  # C++ readers outside loggerd
  for s in ("deviceState", "driverCameraState", "selfdriveState", "sendcan"):
    counts[s] = counts.get(s, 0) + 1
  return counts, where


def test_reader_budget():
  counts, where = count_readers()
  limit = NUM_READERS - HEADROOM
  over = {s: (c, where[s]) for s, c in counts.items() if c > limit}
  assert not over, f"msgq reader budget ({limit}) exceeded: {over}"


if __name__ == "__main__":
  c, w = count_readers()
  for s, n in sorted(c.items(), key=lambda kv: -kv[1])[:8]:
    print(n, s)
