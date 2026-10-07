"""msgq allows NUM_READERS subscribers per service (msgq.h). One more evicts
every reader of that socket; they re-register and evict each other, and
downstream daemons publish valid=False. selfdrived then raises commIssue.

Count every steady process reader, including daemons that are not listed as
PythonProcess entries (athenad, uploader, statsd, manager) and every socket
a process opens (pandad has two deviceState readers). A get_gps_location_service
subscriber opens exactly one of gpsLocation / gpsLocationExternal, so it is
charged to each when checking that configuration. The free slot under
NUM_READERS is athena getMessage or an SSH dump, not a permanent reader.
"""
import ast
import os
import re

from openpilot.common.basedir import BASEDIR

HEADROOM = 1

# Not steady onroad processes. gps_fix creates a SubMaster only for the
# offline map refresh helper; mapd owns the live GPS sockets.
EXCLUDE_DIRS = ("selfdrive/debug", "tools", "scripts", "system/webrtc")
EXCLUDE_FILES = {
  "system/camerad/snapshot.py",
  "selfdrive/mapd/gps_fix.py",
}


def num_readers() -> int:
  found = []
  for rel in ("msgq/msgq.h", "msgq_repo/msgq/msgq.h"):
    path = os.path.join(BASEDIR, rel)
    if not os.path.exists(path):
      continue
    match = re.search(r"#define\s+NUM_READERS\s+(\d+)", open(path).read())
    if match:
      found.append(int(match.group(1)))
  assert found, "NUM_READERS missing from msgq.h"
  assert len(set(found)) == 1, found
  return found[0]


def _services():
  src = open(os.path.join(BASEDIR, "cereal/services.py")).read()
  out = {}
  for name, should_log in re.findall(r'^\s*"(\w+)":\s*\((True|False)', src, re.M):
    out[name] = should_log == "True"
  return out


def _call_name(func) -> str:
  if isinstance(func, ast.Attribute):
    return func.attr
  if isinstance(func, ast.Name):
    return func.id
  return ""


def _simple_name(node) -> str:
  if isinstance(node, ast.Name):
    return node.id
  if isinstance(node, ast.Attribute):
    return node.attr
  return ""


def _main_guard(node: ast.If) -> bool:
  test = node.test
  if not isinstance(test, ast.Compare) or len(test.ops) != 1 or len(test.comparators) != 1:
    return False
  if not isinstance(test.ops[0], (ast.Eq, ast.Is)):
    return False
  sides = (test.left, test.comparators[0])

  def _is_name(n):
    return isinstance(n, ast.Name) and n.id == "__name__"

  def _is_main(n):
    return isinstance(n, ast.Constant) and n.value == "__main__"

  return (_is_name(sides[0]) and _is_main(sides[1])) or (_is_name(sides[1]) and _is_main(sides[0]))


class _Readers(ast.NodeVisitor):
  """One subscriber call site is one reader, even inside the same process."""

  def __init__(self):
    self.scope = [{}]
    self.comp = []
    self.hits = []

  def visit_FunctionDef(self, node):
    self.scope.append({})
    self.generic_visit(node)
    self.scope.pop()

  visit_AsyncFunctionDef = visit_FunctionDef

  def visit_ClassDef(self, node):
    self.scope.append({})
    self.generic_visit(node)
    self.scope.pop()

  def visit_Lambda(self, node):
    self.scope.append({})
    self.generic_visit(node)
    self.scope.pop()

  def visit_If(self, node):
    if _main_guard(node):
      for stmt in node.orelse:
        self.visit(stmt)
      return
    self.generic_visit(node)

  def visit_Assign(self, node):
    for target in node.targets:
      name = _simple_name(target)
      if name:
        self.scope[-1].setdefault(name, []).append((node.lineno, node.value))
    self.generic_visit(node)

  def visit_AnnAssign(self, node):
    name = _simple_name(node.target)
    if name and node.value is not None:
      self.scope[-1].setdefault(name, []).append((node.lineno, node.value))
    self.generic_visit(node)

  def visit_For(self, node):
    name = _simple_name(node.target)
    if name:
      self.comp.append({name: list(self._resolve(node.iter, node.lineno))})
    for stmt in node.body:
      self.visit(stmt)
    if name:
      self.comp.pop()
    for stmt in node.orelse:
      self.visit(stmt)

  def _visit_comp(self, node, parts):
    pushed = 0
    for gen in node.generators:
      name = _simple_name(gen.target)
      if not name:
        continue
      self.comp.append({name: list(self._resolve(gen.iter, getattr(gen.iter, "lineno", node.lineno)))})
      pushed += 1
    for part in parts:
      self.visit(part)
    for _ in range(pushed):
      self.comp.pop()

  def visit_ListComp(self, node):
    self._visit_comp(node, [node.elt])

  def visit_SetComp(self, node):
    self._visit_comp(node, [node.elt])

  def visit_GeneratorExp(self, node):
    self._visit_comp(node, [node.elt])

  def visit_DictComp(self, node):
    self._visit_comp(node, [node.key, node.value])

  def visit_Call(self, node):
    if _call_name(node.func) in ("SubMaster", "sub_sock") and node.args:
      for kind, name in self._resolve(node.args[0], node.lineno):
        self.hits.append((kind, name, node.lineno))
    self.generic_visit(node)

  def _latest_assign(self, name, lineno):
    for scope in reversed(self.scope):
      entries = scope.get(name)
      if not entries:
        continue
      prior = [value for line, value in entries if line <= lineno]
      if prior:
        return prior[-1]
      return None
    return None

  def _resolve(self, node, lineno, seen=None):
    if node is None:
      return
    if seen is None:
      seen = set()
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
      yield ("service", node.value)
      return
    if isinstance(node, (ast.List, ast.Tuple, ast.Set)):
      for elt in node.elts:
        yield from self._resolve(elt, lineno, seen)
      return
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
      yield from self._resolve(node.left, lineno, seen)
      yield from self._resolve(node.right, lineno, seen)
      return
    if isinstance(node, ast.Call):
      if _call_name(node.func) == "get_gps_location_service":
        yield ("dynamic_gps", "")
      return
    if isinstance(node, (ast.Name, ast.Attribute)):
      key = _simple_name(node)
      for scope in reversed(self.comp):
        if key in scope:
          yield from scope[key]
          return
      mark = ("name", key)
      if mark in seen:
        return
      seen.add(mark)
      value = self._latest_assign(key, lineno)
      if value is not None:
        yield from self._resolve(value, lineno, seen)


def _skip(rel: str) -> bool:
  rel = rel.replace("\\", "/")
  if rel in EXCLUDE_FILES:
    return True
  parts = rel.split("/")
  if parts[-1].startswith("test_") or parts[-1] == "conftest.py":
    return True
  if any(rel.startswith(d + "/") or ("/" + d + "/") in ("/" + rel) for d in EXCLUDE_DIRS):
    return True
  if "/tests/" in ("/" + rel) or "/test/" in ("/" + rel):
    return True
  return False


def _cpp_readers(rel: str, text: str):
  hits = []
  for match in re.finditer(r"\bSubMaster\s+\w+\s*\(\s*\{([^}]*)\}", text, re.S):
    line = text.count("\n", 0, match.start()) + 1
    for name in re.findall(r'"(\w+)"', match.group(1)):
      hits.append((name, line))
  for match in re.finditer(r"\bSubSocket::create\s*\([^,]+,\s*\"(\w+)\"", text):
    line = text.count("\n", 0, match.start()) + 1
    hits.append((match.group(1), line))
  return [(name, f"{rel}:{line}") for name, line in hits]


def count_readers():
  services = _services()
  counts = {name: (1 if logged else 0) for name, logged in services.items()}
  where = {name: (["loggerd"] if logged else []) for name, logged in services.items()}
  dynamic_gps = []

  for top in ("selfdrive", "system"):
    for root, _, files in os.walk(os.path.join(BASEDIR, top)):
      for fn in files:
        path = os.path.join(root, fn)
        rel = os.path.relpath(path, BASEDIR)
        if _skip(rel):
          continue
        if fn.endswith(".py"):
          try:
            tree = ast.parse(open(path).read())
          except SyntaxError:
            continue
          visitor = _Readers()
          visitor.visit(tree)
          for kind, name, lineno in visitor.hits:
            loc = f"{rel}:{lineno}"
            if kind == "dynamic_gps":
              dynamic_gps.append(loc)
              continue
            if name not in services:
              continue
            counts[name] += 1
            where[name].append(loc)
        elif fn.endswith((".cc", ".h", ".hpp")):
          for name, loc in _cpp_readers(rel, open(path).read()):
            if name not in services:
              continue
            counts[name] += 1
            where[name].append(loc)

  # One process, one GPS socket. Check both device configurations.
  for name in ("gpsLocation", "gpsLocationExternal"):
    if name not in counts:
      continue
    counts[name] += len(dynamic_gps)
    where[name].extend(dynamic_gps)
  return counts, where


def test_reader_budget():
  counts, where = count_readers()
  limit = num_readers() - HEADROOM
  over = {name: (count, where[name]) for name, count in counts.items() if count > limit}
  assert not over, f"msgq reader budget ({limit}) exceeded: {over}"


def test_card_process_does_not_subscribe_carstate():
  """card publishes carState. preap body controls must reuse that, not subscribe."""
  _, where = count_readers()
  hits = [loc for loc in where.get("carState", []) if "card.py" in loc or "preap_body_controls.py" in loc]
  assert hits == [], hits


def test_pandad_has_two_devicestate_readers():
  _, where = count_readers()
  hits = [loc for loc in where["deviceState"] if "pandad.cc" in loc]
  assert len(hits) == 2, hits


def test_always_on_daemons_are_counted():
  _, where = count_readers()
  blob = "\n".join(where["deviceState"])
  for name in ("athenad.py", "uploader.py", "statsd.py", "manager.py"):
    assert name in blob, name


if __name__ == "__main__":
  readers, locations = count_readers()
  limit = num_readers() - HEADROOM
  print(f"NUM_READERS={num_readers()} limit={limit}")
  for name, count in sorted(readers.items(), key=lambda kv: -kv[1])[:12]:
    print(f"{count:3} {name}")
    if count >= limit - 1:
      for loc in locations[name]:
        print(f"    {loc}")
