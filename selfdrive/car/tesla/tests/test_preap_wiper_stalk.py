"""NAP wiper Off/Auto + physical collar 0→1→0 shortcut."""
import time
from types import SimpleNamespace

from openpilot.selfdrive.car.tesla.preap_body_controls import (
  NAP_WIPER_HUD_PENDING,
  NAP_WIPER_SPEED,
  STALK_FLICK_WINDOW_S,
  STW_ACTN_RQ_ADDR,
  WIPER_HUD_AUTO,
  WIPER_HUD_DURATION_S,
  WIPER_HUD_OFF,
  WIPER_SETTING_AUTO,
  WIPER_SETTING_INTERMITTENT,
  WIPER_SETTING_OFF,
  WIPER_SETTING_ON,
  normalize_wiper_setting,
  poll_wiper_stalk_shortcut,
  read_wiper_setting,
  reset_auto_gates,
  reset_wiper_stalk_detector,
  toggle_wiper_auto,
  wiper_hud_text,
)


class _WiperParams:
  def __init__(self, speed=WIPER_SETTING_OFF):
    self.store = {NAP_WIPER_SPEED: int(speed)}

  def get(self, key, return_default=False):
    return self.store.get(key, 0 if return_default else None)

  def put(self, key, dat, block=False):
    self.store[key] = dat

  def put_bool(self, key, val):
    self.store[key] = bool(val)

  def get_bool(self, key):
    return bool(self.store.get(key))


def _collar_cs(posn: int, wash: int = 0):
  return SimpleNamespace(msg_stw_actn_req={
    "WprSw6Posn": posn, "WprWashSw_Psd": wash, "SpdCtrlLvr_Stat": 0,
  })


def test_normalize_wiper_setting_keeps_auto_coerces_legacy():
  assert normalize_wiper_setting(WIPER_SETTING_OFF) == WIPER_SETTING_OFF
  assert normalize_wiper_setting(WIPER_SETTING_AUTO) == WIPER_SETTING_AUTO
  assert normalize_wiper_setting(WIPER_SETTING_INTERMITTENT) == WIPER_SETTING_OFF
  assert normalize_wiper_setting(WIPER_SETTING_ON) == WIPER_SETTING_OFF
  assert normalize_wiper_setting(99) == WIPER_SETTING_OFF
  assert normalize_wiper_setting(None) == WIPER_SETTING_OFF
  assert wiper_hud_text(WIPER_SETTING_OFF) == WIPER_HUD_OFF
  assert wiper_hud_text(WIPER_SETTING_AUTO) == WIPER_HUD_AUTO
  assert WIPER_HUD_DURATION_S >= 2.0
  assert STALK_FLICK_WINDOW_S == 1.0


def test_read_wiper_setting_persists_legacy_int_as_off(monkeypatch):
  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  fake = _WiperParams(WIPER_SETTING_INTERMITTENT)
  monkeypatch.setattr(body, "_get_params", lambda: fake)
  reset_auto_gates()
  try:
    assert read_wiper_setting() == WIPER_SETTING_OFF
    assert fake.store[NAP_WIPER_SPEED] == WIPER_SETTING_OFF
    assert not fake.store.get(NAP_WIPER_HUD_PENDING)
  finally:
    reset_auto_gates()


def test_stalk_double_flick_toggles_off_auto_and_banner(monkeypatch):
  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  fake = _WiperParams(WIPER_SETTING_OFF)
  monkeypatch.setattr(body, "_get_params", lambda: fake)
  reset_auto_gates()
  t = 100.0
  try:
    assert poll_wiper_stalk_shortcut(_collar_cs(0), now=t) is None
    assert poll_wiper_stalk_shortcut(_collar_cs(1), now=t + 0.05) is None
    nxt = poll_wiper_stalk_shortcut(_collar_cs(0), now=t + 0.40)
    assert nxt == WIPER_SETTING_AUTO
    assert fake.store[NAP_WIPER_SPEED] == WIPER_SETTING_AUTO
    assert fake.store[NAP_WIPER_HUD_PENDING] is True
    assert wiper_hud_text() == WIPER_HUD_AUTO

    fake.store[NAP_WIPER_HUD_PENDING] = False
    t = 200.0
    assert poll_wiper_stalk_shortcut(_collar_cs(0), now=t) is None
    assert poll_wiper_stalk_shortcut(_collar_cs(1), now=t + 0.05) is None
    nxt = poll_wiper_stalk_shortcut(_collar_cs(0), now=t + 0.35)
    assert nxt == WIPER_SETTING_OFF
    assert fake.store[NAP_WIPER_SPEED] == WIPER_SETTING_OFF
    assert fake.store[NAP_WIPER_HUD_PENDING] is True
    assert wiper_hud_text() == WIPER_HUD_OFF
  finally:
    reset_auto_gates()


def test_stalk_slow_or_incomplete_flick_does_not_toggle(monkeypatch):
  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  fake = _WiperParams(WIPER_SETTING_OFF)
  monkeypatch.setattr(body, "_get_params", lambda: fake)
  reset_auto_gates()
  t = 100.0
  try:
    assert poll_wiper_stalk_shortcut(_collar_cs(0), now=t) is None
    assert poll_wiper_stalk_shortcut(_collar_cs(1), now=t + 0.05) is None
    assert poll_wiper_stalk_shortcut(_collar_cs(0), now=t + 1.20) is None
    assert fake.store[NAP_WIPER_SPEED] == WIPER_SETTING_OFF
    assert not fake.store.get(NAP_WIPER_HUD_PENDING)

    reset_wiper_stalk_detector()
    t = 300.0
    assert poll_wiper_stalk_shortcut(_collar_cs(0), now=t) is None
    assert poll_wiper_stalk_shortcut(_collar_cs(1), now=t + 0.05) is None
    assert poll_wiper_stalk_shortcut(_collar_cs(1), now=t + 0.50) is None
    assert fake.store[NAP_WIPER_SPEED] == WIPER_SETTING_OFF

    reset_wiper_stalk_detector()
    t = 400.0
    assert poll_wiper_stalk_shortcut(_collar_cs(0), now=t) is None
    assert poll_wiper_stalk_shortcut(_collar_cs(2), now=t + 0.05) is None
    assert poll_wiper_stalk_shortcut(_collar_cs(0), now=t + 0.20) is None
    assert fake.store[NAP_WIPER_SPEED] == WIPER_SETTING_OFF

    reset_wiper_stalk_detector()
    t = 500.0
    assert poll_wiper_stalk_shortcut(_collar_cs(0), now=t) is None
    assert poll_wiper_stalk_shortcut(_collar_cs(1, wash=2), now=t + 0.05) is None
    assert poll_wiper_stalk_shortcut(_collar_cs(0), now=t + 0.20) is None
    assert fake.store[NAP_WIPER_SPEED] == WIPER_SETTING_OFF
  finally:
    reset_auto_gates()


def test_stalk_flick_uses_physical_collar_not_auto_overlay(monkeypatch):
  """Auto INTERVAL1 overlay must not look like a driver flick."""
  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  fake = _WiperParams(WIPER_SETTING_AUTO)
  monkeypatch.setattr(body, "_get_params", lambda: fake)
  reset_auto_gates()
  t = 100.0
  try:
    # Live stalk stays Off while Auto overlays collar=1 on TX.
    assert poll_wiper_stalk_shortcut(_collar_cs(0), now=t) is None
    assert poll_wiper_stalk_shortcut(_collar_cs(0), now=t + 0.05) is None
    assert fake.store[NAP_WIPER_SPEED] == WIPER_SETTING_AUTO
    nxt = toggle_wiper_auto(announce=False)
    assert nxt == WIPER_SETTING_OFF
    assert not fake.store.get(NAP_WIPER_HUD_PENDING)
  finally:
    reset_auto_gates()


def test_stock_cc_update_sees_physical_flick(monkeypatch):
  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  fake_params = _WiperParams(WIPER_SETTING_OFF)
  monkeypatch.setattr(body, "_get_params", lambda: fake_params)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])

  class _Fake:
    def _send(self, CS, tesla_can, bus, button):
      return (STW_ACTN_RQ_ADDR, b"\x00", bus)

  reset_auto_gates()
  t = [1000.0]
  monkeypatch.setattr(time, "monotonic", lambda: t[0])
  try:
    body.stock_cc_update_with_overlay(_Fake(), _collar_cs(0), 10, None, 0)
    t[0] += 0.05
    body.stock_cc_update_with_overlay(_Fake(), _collar_cs(1), 11, None, 0)
    t[0] += 0.20
    body.stock_cc_update_with_overlay(_Fake(), _collar_cs(0), 12, None, 0)
    assert fake_params.store[NAP_WIPER_SPEED] == WIPER_SETTING_AUTO
    assert fake_params.store[NAP_WIPER_HUD_PENDING] is True
  finally:
    reset_auto_gates()
    monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", None)
