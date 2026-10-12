"""Early yield on steeringPressed (no EPAS hands gate).

EPAS_handsOnLevel lags the torsion bar, so a firm push can reach hands >= 2
before the hands-gated soft yield fires. Same-sign torsion >= EARLY_YIELD_NM
while steeringPressed for EARLY_YIELD_FRAMES yields without the hands gate so
the wheel input lands on a yielded lateral (B), not a full one (A).

Tuned conservatively on Oct 1-3 qlogs (see driver_lateral_handoff.py): the
1 Nm / 50 ms variant false-yielded on road chatter.
"""
from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  EARLY_YIELD_FRAMES,
  EARLY_YIELD_NM,
  DriverLateralHandoff,
)


def _run(torques, *, pressed=True, hands=0, alc=False, enabled=True, v_ego=20.0):
  h = DriverLateralHandoff(enabled=enabled)
  out = None
  for tq in torques:
    out = h.update(engaged=True, lat_would_be_active=True, steering_torque=tq,
                   steering_rate_deg=0.0, hands_on_level=hands, alc_active=alc,
                   v_ego=v_ego, steering_pressed=pressed)
  return out


def test_sustained_same_sign_push_yields_without_hands():
  assert _run([EARLY_YIELD_NM + 0.5] * EARLY_YIELD_FRAMES).yielded
  assert _run([-(EARLY_YIELD_NM + 0.5)] * EARLY_YIELD_FRAMES).yielded
  assert _run([EARLY_YIELD_NM] * EARLY_YIELD_FRAMES).yielded  # boundary


def test_one_frame_short_does_not_yield():
  assert not _run([EARLY_YIELD_NM + 0.5] * (EARLY_YIELD_FRAMES - 1)).yielded


def test_below_threshold_does_not_yield():
  # 1.95 Nm is above the 1.4 Nm / 80 ms path, so it yields. Resting and
  # short firm-but-not-1.4 pushes do not.
  assert not _run([0.85] * 40).yielded
  assert not _run([1.2] * 8, pressed=False).yielded
  assert _run([1.2] * 15, pressed=False).yielded


def test_alternating_sign_chatter_does_not_yield():
  chatter = [3.0, -3.0] * 30
  assert not _run(chatter).yielded
  # a sign flip in the middle restarts the count
  assert not _run([3.0] * (EARLY_YIELD_FRAMES - 1) + [-3.0] + [3.0] * (EARLY_YIELD_FRAMES - 1)).yielded


def test_gap_restarts_the_count():
  assert not _run(([3.0] * (EARLY_YIELD_FRAMES - 1) + [0.0]) * 4).yielded


def test_firm_torque_yields_without_steering_pressed():
  """1.4 Nm / 80 ms does not wait for steeringPressed or a hands level."""
  assert _run([1.4] * 8, pressed=False).yielded
  assert _run([4.0] * 8, pressed=False).yielded
  assert not _run([1.2] * 8, pressed=False).yielded


def test_resting_band_never_yields():
  """Hands resting at 0.15–0.35 Nm never enter yield or a take-back taper.

  The check is every frame. A dropped 0.9 Nm threshold lets 0.25 Nm yield
  and then R1 give the wheel back, so the final frame is full lateral again.
  """
  for tq in (0.15, 0.25, 0.35, -0.30):
    for pressed, hands in ((False, 0), (True, 1)):
      h = DriverLateralHandoff(enabled=True)
      for _ in range(400):
        out = h.update(engaged=True, lat_would_be_active=True, steering_torque=tq,
                       steering_rate_deg=0.0, hands_on_level=hands, v_ego=20.0,
                       steering_pressed=pressed)
        assert not out.yielded and not out.blending and out.authority == 1.0


def test_not_in_tipped_lane_change():
  assert not _run([4.0] * 40, alc=True).yielded


def test_same_direction_hands_edge_yields_on_one_frame():
  """Helping torque + hands 3 yields before the 80 ms counter. Opposite does not."""
  h = DriverLateralHandoff(enabled=True)
  out = h.update(engaged=True, lat_would_be_active=True, steering_torque=-3.5,
                 steering_rate_deg=0.0, hands_on_level=3, v_ego=15.0,
                 steering_angle_deg=-30.0, commanded_angle_deg=-40.0,
                 steering_pressed=True)
  assert out.yielded and out.authority == 0.0

  h = DriverLateralHandoff(enabled=True)
  out = h.update(engaged=True, lat_would_be_active=True, steering_torque=3.5,
                 steering_rate_deg=0.0, hands_on_level=3, v_ego=15.0,
                 steering_angle_deg=-30.0, commanded_angle_deg=-40.0,
                 steering_pressed=True, undertrack=True)
  assert not out.yielded and out.authority == 1.0


def test_roundabout_any_input_yields_immediately():
  h = DriverLateralHandoff(enabled=True)
  out = h.update(engaged=True, lat_would_be_active=True, steering_torque=2.0,
                 steering_rate_deg=0.0, hands_on_level=2, v_ego=8.0,
                 steering_angle_deg=-20.0, commanded_angle_deg=-10.0,
                 roundabout_yield=True, steering_pressed=True)
  assert out.yielded
  h = DriverLateralHandoff(enabled=True)
  out = h.update(engaged=True, lat_would_be_active=True, steering_torque=1.2,
                 steering_rate_deg=0.0, hands_on_level=0, v_ego=8.0,
                 roundabout_yield=True, steering_pressed=True)
  assert out.yielded


def test_farm_trace_yields_on_the_final_helping_yank():
  torques = [0.70, -0.95, 0.47, -2.19, 0.86, -1.79, 0.15, -1.00, -3.29, -3.77]
  h = DriverLateralHandoff(enabled=True)
  out = None
  for i, tq in enumerate(torques):
    meas = -26.0 + (-44.0 + 26.0) * i / (len(torques) - 1)
    out = h.update(engaged=True, lat_would_be_active=True, steering_torque=tq,
                   steering_rate_deg=0.0, hands_on_level=3 if i == len(torques) - 1 else 0,
                   v_ego=8.0, steering_angle_deg=meas, commanded_angle_deg=meas + 8.0,
                   undertrack=True, steering_pressed=abs(tq) >= 1.0)
    if i < len(torques) - 1:
      assert not out.yielded
  assert out.yielded


def test_handoff_off_is_identity():
  out = _run([4.0] * 40, enabled=False)
  assert not out.yielded and out.authority == 1.0


def test_yield_frees_the_eps_then_blends_back_after_hands_off():
  from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
    BLEND_TIME_S,
    RELEASE_HOLD_S,
    lat_active_after_handoff,
  )
  h = DriverLateralHandoff()
  def step(tq, pressed, hands=0):
    return h.update(engaged=True, lat_would_be_active=True, steering_torque=tq,
                    steering_rate_deg=0.0, hands_on_level=hands, v_ego=20.0,
                    steering_pressed=pressed)
  for _ in range(EARLY_YIELD_FRAMES):
    out = step(3.0, True)
  assert out.yielded and not lat_active_after_handoff(True, out.yielded)
  for _ in range(int(RELEASE_HOLD_S / 0.01) + 3):
    out = step(0.0, False)
  assert out.blending and lat_active_after_handoff(True, out.yielded)
  for _ in range(int(BLEND_TIME_S / 0.01) + 3):
    out = step(0.0, False)
  assert not out.blending and out.authority == 1.0


def test_controlsd_and_card_pass_steering_pressed_to_the_handoff():
  from pathlib import Path
  repo = Path(__file__).resolve().parents[4]
  controlsd = (repo / "selfdrive/controls/controlsd.py").read_text()
  call_tail = (
    "      steering_pressed=bool(CS.steeringPressed),\n"
    "      model_curvature=float(self._raw_model_curvature),\n"
    "      measured_curvature=float(self.curvature),\n"
    "    )\n"
  )
  assert call_tail in controlsd
  assert "CC.latActive = lat_active_after_handoff(" in controlsd
  card = (repo / "selfdrive/car/tesla/preap_blinker_lat_pause.py").read_text()
  assert 'steering_pressed=bool(getattr(ret, "steeringPressed", False)),' in card
