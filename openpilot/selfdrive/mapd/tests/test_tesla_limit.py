from types import SimpleNamespace

import pytest

from openpilot.selfdrive.mapd.tesla_limit import (
  HOLD_S, TeslaLimitFusion, base_mph, mpp_mph,
)


def test_base_mph_rounds_and_unknown():
  assert base_mph(0.0) == 0                 # 0 = unknown, never a limit
  assert base_mph(13.25) == 30              # 29.6 mph raw (0.25 m/s steps)
  assert base_mph(22.25) == 50              # 49.8
  assert base_mph(26.75) == 60              # 59.8
  assert base_mph(float("nan")) == 0
  assert base_mph(63.75) == 0               # 142 mph: out of range -> unknown


def test_mpp_units():
  assert mpp_mph(45) == 45
  assert mpp_mph(0) == 0
  assert mpp_mph(80, units_kph=True) == 50


def test_fusion_hold_and_conf():
  f = TeslaLimitFusion()
  t = 0.0
  while t < HOLD_S - 0.4:
    f.add_base(t, 50, 99)
    assert not f.state(t).valid           # not held long enough yet
    t += 0.4
  f.add_base(HOLD_S, 50, 99)
  st = f.state(HOLD_S)
  assert st.valid and st.base_mph == 50 and st.conf == 99
  # a new value must hold again; the old one stays published meanwhile
  f.add_base(HOLD_S + 0.4, 60, 99)
  assert f.state(HOLD_S + 0.4).base_mph == 50
  f.add_base(2 * HOLD_S + 0.5, 60, 99)
  assert f.state(2 * HOLD_S + 0.5).base_mph == 60
  # low confidence drops it to unknown
  f.add_base(10.0, 60, 80)
  assert not f.state(10.0).valid and f.state(10.0).base_mph == 0


def test_fusion_unknown_and_stale():
  f = TeslaLimitFusion()
  for i in range(10):
    f.add_base(i * 0.4, 30, 99)
  assert f.state(3.6).base_mph == 30
  f.add_base(4.0, 0, 99)                   # 0 = unknown: unpublished at once
  assert f.state(4.0).base_mph == 0
  for i in range(10):
    f.add_base(5 + i * 0.4, 30, 99)
  assert f.state(8.6).valid
  assert not f.state(8.6 + 2.5).valid       # no frames for > 2 s
  f.add_mpp(9.0, 55)
  assert f.state(9.5).mpp_mph == 55 and f.state(13.0).mpp_mph == 0


def _ev(t, frames):
  return SimpleNamespace(logMonoTime=int(t * 1e9), can=[SimpleNamespace(address=a, dat=d, src=s) for a, d, s in frames])


def test_can_reader_decodes_mux3_and_mpp():
  pytest.importorskip("opendbc.can.parser")
  from opendbc.can.packer import CANPacker
  from openpilot.selfdrive.mapd.tesla_limit import TeslaCanReader
  pk = CANPacker("tesla_preap")
  r = TeslaCanReader()
  evs = []
  for i in range(12):
    t = i * 0.4
    fr = [pk.make_can_msg("UI_driverAssistRoadSign", 0, {"UI_roadSign": 3, "UI_baseMapSpeedLimitMPS": 20.0,
                                                          "UI_splineLocConfidence": 99}),
          # another mux on the same address must be ignored (its byte 1 is not a speed)
          pk.make_can_msg("UI_driverAssistRoadSign", 0, {"UI_roadSign": 1, "UI_baseMapSpeedLimitMPS": 2.0}),
          # bus 2 copy must be ignored
          pk.make_can_msg("UI_driverAssistRoadSign", 2, {"UI_roadSign": 3, "UI_baseMapSpeedLimitMPS": 5.0}),
          pk.make_can_msg("UI_gpsVehicleSpeed", 0, {"UI_mppSpeedLimit": 40})]
    evs.append(_ev(t, fr))
  r.update(evs)
  st = r.fusion.state(4.4)
  assert st.valid and st.base_mph == 45 and st.mpp_mph == 40 and st.conf == 99
