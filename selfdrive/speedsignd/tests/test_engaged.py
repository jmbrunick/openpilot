"""Skip ONNX while OP is controlling; allow it while driving manually (moving OK)."""
from __future__ import annotations

from types import SimpleNamespace

from openpilot.selfdrive.speedsignd.detect import SpeedSign
from openpilot.selfdrive.speedsignd.hud import live_sign_view
from openpilot.selfdrive.speedsignd.jsonl import JsonlLogger
from openpilot.selfdrive.speedsignd.speedsignd import (
  SM_HZ,
  SPEEDSIGND_HZ,
  UNKNOWN_GRACE_S,
  InferSlot,
  detect_if_allowed,
  drain_vision_latest,
  engaged_from_sm,
  engagement_from_sm,
  engagement_log_fields,
  op_controlling,
  should_reset_detect_after_wait,
  should_run_onnx_detect,
  should_run_speed_sign_log,
)


class _RaiseIfDetect:
  def detect(self, y, min_conf=None, rgb=None):
    raise AssertionError("ONNX detect must not run while OP is controlling")


class _HitDetect:
  def __init__(self):
    self.calls = 0

  def detect(self, y, min_conf=None, rgb=None):
    self.calls += 1
    return [SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 8, 8))]


class _FakeParams:
  def __init__(self, enabled=False):
    self.enabled = enabled

  def get_bool(self, key):
    return self.enabled


def test_logger_on_still_starts_process_when_onroad():
  assert should_run_speed_sign_log(True, _FakeParams(True))
  assert not should_run_speed_sign_log(True, _FakeParams(False))


def test_no_onnx_while_op_controlling():
  assert not should_run_onnx_detect(True)
  assert should_run_onnx_detect(False)


def test_unknown_is_not_forever_controlling():
  """Soft fail-safe: cereal silence is not 'OP is driving'."""
  assert op_controlling() is False
  assert op_controlling(active=None, state=None, enabled=None) is False
  assert should_run_onnx_detect(op_controlling())


def test_active_and_controlling_states_are_the_gate():
  assert op_controlling(active=True) is True
  assert op_controlling(active=False) is False
  assert op_controlling(state="disabled") is False
  assert op_controlling(state="preEnabled", enabled=True, active=False) is False
  assert op_controlling(state="enabled", active=True) is True
  assert op_controlling(state="softDisabling") is True
  assert op_controlling(state="overriding") is True
  assert op_controlling(state=0) is False
  assert op_controlling(state=2) is True
  assert op_controlling(enabled=False) is False
  assert op_controlling(enabled=True) is True


def test_detect_if_allowed_skips_onnx_and_jsonl_when_controlling(tmp_path):
  log = JsonlLogger(str(tmp_path / "out.jsonl"))
  y = __import__("numpy").zeros((32, 32), __import__("numpy").uint8)
  signs, written = detect_if_allowed(
    y, 45.0, -95.0, 0.0, True, _RaiseIfDetect(), log, now=1.0, controlling=True,
  )
  assert signs == [] and written == []
  assert not (tmp_path / "out.jsonl").exists()


def test_detect_if_allowed_runs_when_disengaged_moving(tmp_path):
  """Product: log signs while driving manually. Moving / not parked is OK."""
  det = _HitDetect()
  log = JsonlLogger(str(tmp_path / "out.jsonl"))
  y = __import__("numpy").zeros((32, 32), __import__("numpy").uint8)
  signs, written = detect_if_allowed(
    y, 45.0, -95.0, 0.0, True, det, log, now=1.0, controlling=False,
  )
  assert det.calls == 1
  assert signs and signs[0].mph == 55
  assert written and written[0]["mph"] == 55


class _Sm:
  def __init__(self, recv: dict[str, int], services: dict[str, object], *,
               seen: dict[str, bool] | None = None,
               alive: dict[str, bool] | None = None,
               valid: dict[str, bool] | None = None):
    self.recv_frame = recv
    self._services = services
    self.seen = seen if seen is not None else {k: v > 0 for k, v in recv.items()}
    if alive is not None:
      self.alive = alive
    if valid is not None:
      self.valid = valid

  def __getitem__(self, name: str):
    return self._services[name]


def _ss(*, enabled=False, active=False, state="disabled"):
  return SimpleNamespace(enabled=enabled, active=active, state=state)


def test_unknown_cereal_startup_grace_then_allow_detect():
  sm = _Sm({}, {})
  start = 10.0
  during = engagement_from_sm(sm, now=start + 0.5, started_at=start)
  assert during.known is False
  assert during.controlling is False
  assert during.allow_detect is False
  assert during.detect_paused is False
  after = engagement_from_sm(sm, now=start + UNKNOWN_GRACE_S + 0.1, started_at=start)
  assert after.allow_detect is True
  assert after.detect_paused is False
  assert after.unknown_after_grace is True
  # now=None (tests) is post-grace allow, not forever-WAIT.
  relaxed = engagement_from_sm(sm)
  assert relaxed.allow_detect is True
  assert relaxed.detect_paused is False
  assert engaged_from_sm(sm) is False


def test_stale_alive_or_invalid_uses_last_payload():
  """1 Hz poll of 100 Hz selfdriveState makes alive timeout (100 ms) fire.

  Last parsed active/state still wins — do not stuck WAIT.
  """
  sm = _Sm(
    {"selfdriveState": 9},
    {"selfdriveState": _ss(enabled=False, active=False, state="disabled")},
    seen={"selfdriveState": True},
    alive={"selfdriveState": False},
  )
  sample = engagement_from_sm(sm)
  assert sample.known is True
  assert sample.controlling is False
  assert sample.allow_detect is True
  assert sample.detect_paused is False
  assert sample.alive is False

  sm = _Sm(
    {"selfdriveState": 9},
    {"selfdriveState": _ss(enabled=False, active=False, state="disabled")},
    seen={"selfdriveState": True},
    valid={"selfdriveState": False},
  )
  sample = engagement_from_sm(sm)
  assert sample.allow_detect is True
  assert sample.detect_paused is False


def test_first_recv_frame_zero_is_seen():
  """SubMaster writes recv_frame=0 on the first update that lands a packet."""
  sm = _Sm(
    {"selfdriveState": 0},
    {"selfdriveState": _ss(enabled=False, active=False, state="disabled")},
    seen={"selfdriveState": True},
    alive={"selfdriveState": True},
    valid={"selfdriveState": True},
  )
  sample = engagement_from_sm(sm)
  assert sample.known is True
  assert sample.allow_detect is True
  assert sample.detect_paused is False


def test_engagement_from_sm_active_controlling():
  sm = _Sm(
    {"selfdriveState": 3},
    {"selfdriveState": _ss(enabled=True, active=True, state="enabled")},
    seen={"selfdriveState": True},
  )
  sample = engagement_from_sm(sm)
  assert sample.controlling is True
  assert sample.allow_detect is False
  assert sample.detect_paused is True
  assert not should_run_onnx_detect(sample.controlling)

  sm = _Sm(
    {"selfdriveState": 3},
    {"selfdriveState": _ss(enabled=False, active=False, state="disabled")},
    seen={"selfdriveState": True},
  )
  sample = engagement_from_sm(sm)
  assert sample.controlling is False
  assert sample.allow_detect is True
  assert sample.detect_paused is False
  assert should_run_onnx_detect(sample.controlling)


def test_enabled_but_not_active_preenabled_allows_detect():
  sm = _Sm(
    {"selfdriveState": 4},
    {"selfdriveState": _ss(enabled=True, active=False, state="preEnabled")},
    seen={"selfdriveState": True},
  )
  sample = engagement_from_sm(sm)
  assert sample.controlling is False
  assert sample.allow_detect is True
  assert sample.detect_paused is False


def test_overriding_and_soft_disabling_pause_detect():
  for state in ("overriding", "softDisabling"):
    sm = _Sm(
      {"selfdriveState": 5},
      {"selfdriveState": _ss(enabled=True, active=True, state=state)},
      seen={"selfdriveState": True},
    )
    sample = engagement_from_sm(sm)
    assert sample.controlling is True
    assert sample.detect_paused is True
    assert sample.allow_detect is False


def test_stale_flags_still_pause_when_last_payload_is_active():
  sm = _Sm(
    {"selfdriveState": 9},
    {"selfdriveState": _ss(enabled=True, active=True, state="enabled")},
    seen={"selfdriveState": True},
    alive={"selfdriveState": False},
    valid={"selfdriveState": False},
  )
  sample = engagement_from_sm(sm)
  assert sample.controlling is True
  assert sample.allow_detect is False
  assert sample.detect_paused is True


def test_stock_cruise_or_moving_does_not_block_detect():
  """Manual driving (stock CC on, moving) still runs YOLO when OP is off."""
  cruise_on = SimpleNamespace(cruiseState=SimpleNamespace(enabled=True), vEgo=22.0)
  sm = _Sm(
    {"selfdriveState": 4, "carState": 2},
    {"selfdriveState": _ss(enabled=False, active=False, state="disabled"), "carState": cruise_on},
    seen={"selfdriveState": True, "carState": True},
  )
  sample = engagement_from_sm(sm)
  assert sample.controlling is False
  assert sample.allow_detect is True
  # carState-only (no selfdriveState yet) is unknown — not WAIT, detect after grace.
  sm = _Sm({"carState": 2}, {"carState": cruise_on}, seen={"carState": True})
  sample = engagement_from_sm(sm, now=5.0, started_at=0.0)
  assert sample.known is False
  assert sample.detect_paused is False
  assert sample.allow_detect is True


def test_wait_plate_only_when_controlling():
  paused = live_sign_view(enabled=True, msg_valid=True, mph=0, valid=False, detect_paused=True)
  assert paused.plate_text == "WAIT"
  unknown = live_sign_view(enabled=True, msg_valid=True, mph=0, valid=False, detect_paused=False)
  assert unknown.plate_text == ""
  assert not unknown.show_detect_paused


def test_engagement_log_fields_include_gate_signals():
  sm = _Sm(
    {"selfdriveState": 3},
    {"selfdriveState": _ss(enabled=True, active=True, state="enabled")},
    seen={"selfdriveState": True},
    alive={"selfdriveState": True},
    valid={"selfdriveState": True},
  )
  text = engagement_log_fields(engagement_from_sm(sm))
  assert "enabled=True" in text
  assert "active=True" in text
  assert "state=enabled" in text
  assert "alive=True" in text
  assert "valid=True" in text


def test_sm_polls_faster_than_detect():
  assert SM_HZ >= 10.0
  assert SM_HZ <= 20.0
  assert SPEEDSIGND_HZ == 1.0
  assert SM_HZ > SPEEDSIGND_HZ
  # Root cause of Justin's stuck WAIT: 100 Hz selfdriveState alive window is
  # 10/freq = 100 ms. A 1 Hz sm.update leaves alive stale every tick.
  selfdrive_freq = 100.0
  alive_s = 10.0 / selfdrive_freq
  assert alive_s == 0.1
  assert 1.0 / SPEEDSIGND_HZ > alive_s
  assert 1.0 / SM_HZ < alive_s


def test_infer_slot_abandons_in_flight_on_pause():
  """Engage must not join a 300–1500 ms YOLO; drop the result and keep the loop moving."""
  import time
  slot = InferSlot()
  started = __import__("threading").Event()
  release = __import__("threading").Event()

  def slow():
    started.set()
    release.wait(2.0)
    return [SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 1, 1))], [{"mph": 55}]

  t0 = time.monotonic()
  assert slot.start(slow)
  assert started.wait(1.0)
  assert slot.busy is True
  assert slot.start(slow) is False  # one at a time
  abandoned = slot.pause()
  assert abandoned is True
  assert time.monotonic() - t0 < 0.25  # did not wait for the infer
  release.set()
  time.sleep(0.05)
  assert slot.take() is None
  assert slot.busy is False
  # Still paused: refuse new work until resume.
  assert slot.start(slow) is False
  slot.resume()
  assert slot.start(lambda: ([], [])) is True
  time.sleep(0.05)
  out = slot.take()
  assert out is not None and out.signs == []


def test_infer_slot_leftover_after_resume_is_still_abandoned():
  """WAIT→manual must not adopt the abandoned infer (that applied overrun skip)."""
  import time
  slot = InferSlot()
  started = __import__("threading").Event()
  release = __import__("threading").Event()

  def slow():
    started.set()
    release.wait(2.0)
    return [SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 1, 1))], [{"mph": 55}]

  assert slot.start(slow)
  assert started.wait(1.0)
  assert slot.pause() is True
  slot.resume()
  release.set()
  time.sleep(0.08)
  assert slot.take() is None
  assert slot.busy is False
  assert slot.start(lambda: ([SpeedSign(mph=60, conf=0.8, bbox=(0, 0, 1, 1))], [])) is True
  for _ in range(50):
    out = slot.take()
    if out is not None:
      assert out.signs[0].mph == 60
      return
    time.sleep(0.01)
  raise AssertionError("post-WAIT infer never arrived")


def test_post_wait_manual_detect_resets_next_detect():
  """Coverage gap after #70: WAIT cleared but leftover next_detect starved YOLO."""
  assert should_reset_detect_after_wait(False, True) is True
  assert should_reset_detect_after_wait(True, True) is False
  assert should_reset_detect_after_wait(None, True) is False
  assert should_reset_detect_after_wait(False, False) is False


def test_drain_vision_latest_is_nonblocking():
  class _Client:
    def __init__(self):
      self.calls = []

    def recv(self, timeout_ms=0):
      self.calls.append(timeout_ms)
      return None

  client = _Client()
  drain_vision_latest(client)
  assert client.calls == [0]
  drain_vision_latest(None)


def test_infer_slot_keeps_result_when_not_paused():
  slot = InferSlot()
  assert slot.start(lambda: ([SpeedSign(mph=60, conf=0.8, bbox=(0, 0, 1, 1))], []))
  for _ in range(50):
    out = slot.take()
    if out is not None:
      assert out.signs[0].mph == 60
      return
    __import__("time").sleep(0.01)
  raise AssertionError("infer result never arrived")


def test_detect_gate_is_not_parked_or_force_offroad():
  from pathlib import Path
  src = Path(__file__).resolve().parents[1] / "speedsignd.py"
  text = src.read_text(encoding="utf-8")
  assert "NAPForceOffroad" not in text
  assert "standstill" not in text
  assert "carState" not in text
  assert "selfdriveState" in text
  assert "detect_paused" in text
  assert "SM_HZ" in text
  assert "UNKNOWN_GRACE_S" in text
  assert "engagement_log_fields" in text
  assert "InferSlot" in text
  assert "abandon" in text
  assert "should_reset_detect_after_wait" in text
  assert "drain_vision_latest" in text
  assert "_gen" in text
