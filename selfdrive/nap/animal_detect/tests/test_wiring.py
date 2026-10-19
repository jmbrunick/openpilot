"""Registration checks: default off, optional process, little cores, log only, license."""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "../../../.."))


def read(rel):
  return open(os.path.join(ROOT, rel)).read()


def test_param_default_off():
  assert re.search(r'\{"NAPAnimalDetect",\s*\{PERSISTENT,\s*BOOL,\s*"0"\}\}', read("common/nap_params_keys.h"))


def test_process_optional_and_gated():
  src = read("system/manager/process_config.py")
  line = [ln for ln in src.splitlines() if '"animaldetd"' in ln]
  assert len(line) == 1 and "NAPAnimalDetect" in line[0] and "optional=True" in line[0] and "only_onroad" in line[0]
  assert '"animaldetd"' in read("system/manager/optional_procs.py")


def test_service_registered():
  assert '"animalDetectNAP": (True, 5., 1)' in read("cereal/services.py")
  assert "animalDetectNAP @112 :Custom.AnimalDetectNAP;" in read("cereal/log.capnp")


def test_only_pathobstacled_reads_it():
  readers = []
  for top in ("selfdrive", "system"):
    for root, _dirs, files in os.walk(os.path.join(ROOT, top)):
      if "/tests" in root:
        continue
      for fn in files:
        if fn.endswith(".py") and "animalDetectNAP" in open(os.path.join(root, fn)).read():
          readers.append(os.path.relpath(os.path.join(root, fn), ROOT))
  outside = [r for r in readers if not r.startswith("selfdrive/nap/animal_detect/")]
  assert outside == ["selfdrive/pathobstacled/pathobstacled.py"], outside


def test_hook_does_not_touch_decisions():
  src = read("selfdrive/pathobstacled/pathobstacled.py")
  hook_lines = [ln for ln in src.splitlines() if "animal" in ln.lower() or "nnclass" in ln.lower()]
  assert 0 < len(hook_lines) <= 10, hook_lines
  for ln in hook_lines:
    low = ln.lower()
    assert "chime" not in low and "brake" not in low and "commit" not in low and "self.det" not in low, ln


def test_no_agpl_code_or_weights():
  mod = os.path.join(ROOT, "selfdrive/nap/animal_detect")
  for root, _dirs, files in os.walk(mod):
    for fn in files:
      if fn.endswith((".py", "SConscript")) and fn != os.path.basename(__file__):
        assert "ultralytics" not in open(os.path.join(root, fn)).read().lower(), fn
  card = open(os.path.join(mod, "models", "MODEL_CARD.md")).read()
  assert "Apache-2.0" in card and "YOLOX" in card and "No Ultralytics / AGPL" in card


def test_build_skips_missing_model():
  s = read("selfdrive/nap/animal_detect/SConscript")
  assert "if os.path.isfile(onnx):" in s
  i = s.index("if os.path.isfile(onnx):")
  assert s.index("env.Command(") > i
