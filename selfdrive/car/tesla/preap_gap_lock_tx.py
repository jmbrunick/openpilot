"""Repeat CANCEL on the stock-CC spoofer while a gap-lock stalk hold is in progress.

Keeps stock cruise asleep during the 2 second engage hold, including while
longitudinal is still paused. An in-session CANCEL does not drop lateral
(panda: lever == CANCEL only when controls are not allowed). This does not
reset the spoofer's cancel frame.
"""
from opendbc.car.tesla.values import CruiseButtons

from openpilot.selfdrive.controls.lib.gap_lock import gap_lock_cancel_hold_tx

STW_ACTN_RQ_ADDR = 0x45

_installed = False
_ORIG_STOCK_CC_UPDATE = None


def stock_cc_update_with_gap_lock(self, CS, frame, tesla_can, can_bus_party):
  orig = _ORIG_STOCK_CC_UPDATE
  can_sends = orig(self, CS, frame, tesla_can, can_bus_party) if orig is not None else []
  if can_sends is None:
    can_sends = []
  had_stw = any(msg[0] == STW_ACTN_RQ_ADDR for msg in can_sends)
  hold = getattr(CS, "preap_cc_cancel_hold", False)
  if hold is True and gap_lock_cancel_hold_tx(had_stw, True, frame):
    sent = self._send(CS, tesla_can, can_bus_party, int(CruiseButtons.CANCEL))
    if sent is not None:
      can_sends.append(sent)
  return can_sends


def install_gap_lock_tx():
  """Wrap StockCCSpoofer.update. Safe to call more than once."""
  global _installed, _ORIG_STOCK_CC_UPDATE
  if _installed:
    return
  from opendbc.car.tesla.preap.stock_cc_spoofer import StockCCSpoofer
  _ORIG_STOCK_CC_UPDATE = StockCCSpoofer.update
  StockCCSpoofer.update = stock_cc_update_with_gap_lock
  _installed = True
