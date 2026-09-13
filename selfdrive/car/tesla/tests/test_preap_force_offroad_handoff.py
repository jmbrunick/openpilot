"""Force Offroad stock-CC handoff + OP-long kill. Ordering matters."""
from types import SimpleNamespace

from openpilot.selfdrive.car.tesla.preap_force_offroad_handoff import (
  ARMING,
  CANCELING,
  CANCEL_DELAY_FRAMES,
  CC_ENGAGE_TIMEOUT_FRAMES,
  ENABLED_WAIT_S,
  HANDOFF_READY_PARAM,
  HANDOFF_TIMEOUT_S,
  IDLE,
  PARAM,
  READY,
  SPOOF_RETRY_S,
  STANDBY_WAIT_S,
  WAIT_ENABLED,
  ForceOffroadHandoff,
  HandoffCommand,
  StockCCKillOnLong,
  apply_controller_flags,
  di_enabled_ok,
  di_is_holding,
  di_needs_kill,
  di_ready_to_set,
  reset_handoff_for_tests,
  software_long_active,
)


def _fsm():
  return ForceOffroadHandoff()


def test_di_helpers_and_long_gate():
  assert di_is_holding("ENABLED")
  assert di_is_holding("STANDSTILL")
  assert di_is_holding("OVERRIDE")
  assert not di_is_holding("STANDBY")
  assert not di_is_holding("OFF")
  assert di_ready_to_set("STANDBY")
  assert not di_ready_to_set("ENABLED")
  assert di_enabled_ok("ENABLED")
  assert di_enabled_ok("STANDSTILL")
  assert di_needs_kill("STANDBY")
  assert di_needs_kill("ENABLED")
  assert not di_needs_kill("OFF")
  assert software_long_active(True)
  assert not software_long_active(False)


def test_unconfirmed_does_not_start_handoff():
  """Yes/No comes first. No cancel / drop / SET until confirmed."""
  h = _fsm()
  cmd = h.update(force_offroad=True, enable_long=True, di_state="ENABLED",
                 now=0.0, confirmed=False)
  assert cmd.state == IDLE
  assert not cmd.ready
  assert not cmd.cancel
  assert not cmd.engage
  assert not cmd.drop_long
  assert not cmd.arm


def test_not_long_force_offroad_is_immediate_ready():
  """Lat-only / stock-CC / not engaged: today's Force Offroad (no invented handoff)."""
  h = _fsm()
  cmd = h.update(force_offroad=True, enable_long=False, di_state="OFF", now=0.0)
  assert cmd.ready
  assert not cmd.cancel
  assert not cmd.engage
  assert not cmd.drop_long
  assert cmd.state == READY


def test_handoff_enabled_cancel_then_standby_then_drop_then_set():
  """Pedal-long + DI ENABLED: CANCEL while long still on, drop only at STANDBY, then SET."""
  h = _fsm()
  cmd = h.update(force_offroad=True, enable_long=True, di_state="ENABLED", now=0.0)
  assert cmd.state == CANCELING
  assert cmd.cancel
  assert not cmd.drop_long
  assert not cmd.ready
  assert not cmd.engage

  # Still ENABLED: do not drop long (stock CC still holding / about to drop).
  cmd = h.update(force_offroad=True, enable_long=True, di_state="ENABLED", now=0.10)
  assert cmd.state == CANCELING
  assert not cmd.drop_long
  assert not cmd.cancel  # cooldown; first CANCEL is in-flight

  cmd = h.update(force_offroad=True, enable_long=True, di_state="STANDBY", now=0.20)
  assert cmd.drop_long
  assert cmd.engage
  assert cmd.suppress_cancel
  assert not cmd.cancel
  assert cmd.state == WAIT_ENABLED
  assert h.dropped_long

  cmd = h.update(force_offroad=True, enable_long=False, di_state="STANDBY", now=0.30)
  assert cmd.suppress_cancel
  assert not cmd.drop_long
  assert not cmd.ready

  cmd = h.update(force_offroad=True, enable_long=False, di_state="ENABLED", now=0.40)
  assert cmd.ready
  assert cmd.state == READY
  assert cmd.suppress_cancel


def test_handoff_already_standby_drops_and_sets_same_tick():
  h = _fsm()
  cmd = h.update(force_offroad=True, enable_long=True, di_state="STANDBY", now=1.0)
  assert cmd.drop_long
  assert cmd.engage
  assert cmd.suppress_cancel
  assert not cmd.cancel
  assert not cmd.ready
  assert cmd.state == WAIT_ENABLED


def test_handoff_off_arms_then_sets():
  """Typical after engage-kill: DI is OFF. MAIN → STANDBY, then drop + SET."""
  h = _fsm()
  cmd = h.update(force_offroad=True, enable_long=True, di_state="OFF", now=0.0)
  assert cmd.state == ARMING
  assert cmd.arm
  assert not cmd.drop_long
  assert not cmd.engage

  cmd = h.update(force_offroad=True, enable_long=True, di_state="STANDBY", now=0.20)
  assert cmd.drop_long
  assert cmd.engage
  assert cmd.suppress_cancel

  cmd = h.update(force_offroad=True, enable_long=False, di_state="ENABLED", now=0.40)
  assert cmd.ready


def test_handoff_off_main_goes_enabled_drops_without_set():
  """Some DIs SET on MAIN. Already holding → drop OP, no SET fight."""
  h = _fsm()
  h.update(force_offroad=True, enable_long=True, di_state="OFF", now=0.0)
  cmd = h.update(force_offroad=True, enable_long=True, di_state="ENABLED", now=0.20)
  assert cmd.drop_long
  assert not cmd.engage
  assert cmd.ready
  assert cmd.suppress_cancel


def test_handoff_does_not_drop_long_before_standby():
  h = _fsm()
  for t in (0.0, 0.05, 0.10, 0.15):
    cmd = h.update(force_offroad=True, enable_long=True, di_state="ENABLED", now=t)
    assert not cmd.drop_long
    assert not cmd.engage
    assert not cmd.ready
    assert cmd.state == CANCELING


def test_toggle_off_resets_and_resumes_kill():
  h = _fsm()
  h.update(force_offroad=True, enable_long=True, di_state="ENABLED", now=0.0)
  assert h.state == CANCELING
  cmd = h.update(force_offroad=False, enable_long=True, di_state="ENABLED", now=0.1)
  assert h.state == IDLE
  assert not cmd.ready
  assert cmd.cancel  # kill-on-long resumes; they are fighting


def test_overall_timeout_writes_ready():
  h = _fsm()
  h.update(force_offroad=True, enable_long=True, di_state="ENABLED", now=0.0)
  cmd = h.update(force_offroad=True, enable_long=True, di_state="ENABLED", now=HANDOFF_TIMEOUT_S + 0.01)
  assert cmd.ready
  assert cmd.state == READY


def test_standby_phase_timeout_fallback():
  h = _fsm()
  h.update(force_offroad=True, enable_long=True, di_state="OFF", now=0.0)
  cmd = h.update(force_offroad=True, enable_long=True, di_state="OFF", now=STANDBY_WAIT_S + 0.01)
  assert cmd.ready


def test_enabled_wait_timeout_fallback():
  h = _fsm()
  h.update(force_offroad=True, enable_long=True, di_state="STANDBY", now=0.0)
  cmd = h.update(force_offroad=True, enable_long=False, di_state="STANDBY", now=ENABLED_WAIT_S + 0.01)
  assert cmd.ready


def test_timing_constants_cover_spoofer_budgets():
  assert CANCEL_DELAY_FRAMES == 10
  assert CC_ENGAGE_TIMEOUT_FRAMES == 50
  assert SPOOF_RETRY_S >= (CANCEL_DELAY_FRAMES + 10) / 100.0
  assert ENABLED_WAIT_S >= (CC_ENGAGE_TIMEOUT_FRAMES / 100.0)
  assert HANDOFF_TIMEOUT_S > ENABLED_WAIT_S + STANDBY_WAIT_S


def test_kill_on_long_rising_enabled_then_standby():
  k = StockCCKillOnLong()
  assert k.update(True, "ENABLED", 0.0)
  assert not k.update(True, "ENABLED", 0.10)  # cooldown
  assert k.update(True, "STANDBY", 0.30)
  assert not k.update(True, "OFF", 0.60)
  assert not k.active


def test_kill_on_long_standby_at_engage():
  k = StockCCKillOnLong()
  assert k.update(True, "STANDBY", 0.0)
  assert not k.update(False, "STANDBY", 0.3)


def test_kill_inactive_when_not_long():
  k = StockCCKillOnLong()
  assert not k.update(False, "ENABLED", 0.0)


def test_handoff_suppresses_kill_while_force_offroad():
  h = _fsm()
  # Rising long would kill STANDBY; Force Offroad needs that STANDBY to SET.
  cmd = h.update(force_offroad=True, enable_long=True, di_state="STANDBY", now=0.0)
  assert not cmd.cancel
  assert cmd.engage


def test_apply_controller_flags_cancel_does_not_beat_engage():
  cs = SimpleNamespace(preap_cc_cancel_needed=True, preap_cc_engage_needed=False,
                       preap_cc_arm_needed=False)
  apply_controller_flags(cs, HandoffCommand(engage=True, suppress_cancel=True))
  assert cs.preap_cc_cancel_needed is False
  assert cs.preap_cc_engage_needed is True


def test_drop_long_silent_keeps_cruise():
  from openpilot.selfdrive.car.tesla.preap_force_offroad_handoff import _drop_long_silent
  eng = SimpleNamespace(enableLongControl=True, cruiseEnabled=True, enableJustCC=False)
  inner = SimpleNamespace(engagement=eng, enableLongControl=True, cruiseEnabled=True,
                          enableJustCC=False)
  _drop_long_silent(inner)
  assert inner.enableLongControl is False
  assert inner.cruiseEnabled is True
  assert inner.enableJustCC is True
  assert eng.enableLongControl is False
  assert eng.enableJustCC is True


def test_update_hook_writes_ready_when_not_long(monkeypatch):
  reset_handoff_for_tests()
  written = {}

  class FakeParams:
    def get_bool(self, key):
      return key == PARAM

    def put_bool(self, key, val):
      written[key] = bool(val)

  from openpilot.selfdrive.car.tesla import preap_force_offroad_handoff as mod
  monkeypatch.setattr(mod, "_params", lambda: FakeParams())
  inner = SimpleNamespace(
    enableLongControl=False,
    di_cruise_state="OFF",
    engagement=SimpleNamespace(
      enableLongControl=False, cruiseEnabled=False, enableJustCC=False,
    ),
  )
  from openpilot.selfdrive.car.tesla.preap_force_offroad_handoff import update_force_offroad_handoff
  cmd = update_force_offroad_handoff(inner, None, now=1.0, force_offroad=True, confirmed=True)
  assert cmd.ready
  assert written.get(HANDOFF_READY_PARAM) is True
  reset_handoff_for_tests()


def test_update_hook_drops_long_only_after_standby(monkeypatch):
  reset_handoff_for_tests()
  written = {}

  class FakeParams:
    def get_bool(self, key):
      return False

    def put_bool(self, key, val):
      written[key] = bool(val)

  from openpilot.selfdrive.car.tesla import preap_force_offroad_handoff as mod
  monkeypatch.setattr(mod, "_params", lambda: FakeParams())
  from openpilot.selfdrive.car.tesla.preap_force_offroad_handoff import update_force_offroad_handoff
  eng = SimpleNamespace(enableLongControl=True, cruiseEnabled=True, enableJustCC=False)
  inner = SimpleNamespace(
    enableLongControl=True, di_cruise_state="ENABLED", engagement=eng,
    cruiseEnabled=True, enableJustCC=False,
  )
  public = SimpleNamespace(enableLongControl=True)
  cmd = update_force_offroad_handoff(inner, public, now=0.0, force_offroad=True, confirmed=True)
  assert cmd.cancel
  assert inner.enableLongControl is True
  assert public.enableLongControl is True
  assert written.get(HANDOFF_READY_PARAM) is not True

  inner.di_cruise_state = "STANDBY"
  cmd = update_force_offroad_handoff(inner, public, now=0.2, force_offroad=True, confirmed=True)
  assert cmd.drop_long
  assert inner.enableLongControl is False
  assert public.enableLongControl is False
  assert eng.enableLongControl is False
  reset_handoff_for_tests()


def test_card_and_hardwared_wire_handoff():
  from pathlib import Path
  root = Path(__file__).resolve().parents[4]
  card = (root / "selfdrive" / "car" / "card.py").read_text(encoding="utf-8")
  hw = (root / "system" / "hardware" / "hardwared.py").read_text(encoding="utf-8")
  assert "install_force_offroad_handoff" in card
  assert "update_force_offroad_handoff(" in card
  # Must run after CI.update (live DI + enableLongControl) and while onroad.
  assert card.index("CS = self.CI.update") < card.index("update_force_offroad_handoff(")
  assert "NAP_FORCE_OFFROAD_HANDOFF_READY_PARAM" in hw
  assert "NAP_FORCE_OFFROAD_CONFIRMED_PARAM" in hw
  assert "handoff_ready" in hw
  assert hw.index("not_force_offroad") < hw.index("ign_edge =")
  main = (root / "selfdrive" / "ui" / "layouts" / "main.py").read_text(encoding="utf-8")
  confirm = (root / "selfdrive" / "ui" / "onroad" / "force_offroad_confirm.py").read_text(encoding="utf-8")
  assert "maybe_show_force_offroad_confirm" in main
  assert "CONFIRM_PROMPT" in confirm
  assert "CONFIRM_YES" in confirm
  assert "CONFIRM_NO" in confirm
  assert "ForceOffroadConfirmDialog" in confirm


def test_stock_cc_arm_wrapper_sends_main_after_delay():
  import pytest
  pytest.importorskip("capnp")
  from openpilot.selfdrive.car.tesla.preap_force_offroad_handoff import (
    install_force_offroad_handoff,
  )
  from opendbc.car.tesla.preap.stock_cc_spoofer import StockCCSpoofer
  from opendbc.car.tesla.values import CruiseButtons

  class FakeCan:
    def __init__(self):
      self.calls: list[tuple] = []

    def create_action_request(self, *args):
      self.calls.append(args)
      return ("MAIN",)

  install_force_offroad_handoff()
  s = StockCCSpoofer()
  can = FakeCan()
  cs = SimpleNamespace(
    preap_cc_cancel_needed=False,
    preap_cc_engage_needed=False,
    preap_cc_arm_needed=True,
    di_cruise_state="OFF",
    msg_stw_actn_req={"MC_STW_ACTN_RQ": 3},
  )
  s.update(cs, 0, can, 2)
  assert cs.preap_cc_arm_needed is False
  assert getattr(s, "_nap_arm_pending", False)
  # Before delay: no MAIN
  for f in range(1, CANCEL_DELAY_FRAMES):
    s.update(cs, f, can, 2)
  assert can.calls == []
  # Delay + slot
  for f in range(CANCEL_DELAY_FRAMES, CANCEL_DELAY_FRAMES + 15):
    s.update(cs, f, can, 2)
  assert len(can.calls) >= 1
  assert can.calls[-1] == (CruiseButtons.MAIN, 2, 4, cs.msg_stw_actn_req)
