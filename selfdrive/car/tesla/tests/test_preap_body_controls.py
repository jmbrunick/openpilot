"""0x45 stalk wiper / high-beam test. Off matches today's forwarded stalk."""
import time

import numpy as np

from openpilot.selfdrive.car.tesla.preap_body_controls import (
  BEAM_SETTING_HIGH,
  BEAM_SETTING_LOW,
  BEAM_SETTING_OFF,
  NAP_HIGH_LOW_BEAM,
  NAP_WIPER_SPEED,
  STW_ACTN_RQ_ADDR,
  STW_CANCEL_BURST_N,
  STW_HIBM_MASK,
  STW_HIGH_BEAM,
  STW_HIGH_BEAM_FLASH,
  STW_TURN_MASK,
  STW_WASHER_SPRAY,
  STW_WIPER_BEAM_BYTE,
  STW_WIPER_ON,
  STW_COLLAR_INTERVAL1,
  STW_WASH_MASK,
  apply_stw_collar,
  overlay_collar_on_can_msg,
  stw_collar_posn,
  stw_wash,
  WIPER_SETTING_AUTO,
  WIPER_SETTING_INTERMITTENT,
  WIPER_SETTING_OFF,
  WIPER_SETTING_ON,
  apply_stw_wiper_beam_nibbles,
  extra_stw_forward_needed,
  high_beam_test_requested,
  hibm_nibble,
  live_stw_counter,
  overlay_stw_wiper_beam,
  rain_wiper_needed,
  register_nap_body_params,
  replace_relayed_stw,
  reset_auto_gates,
  send_replaced_live_stw,
  set_auto_gates,
  set_cereal_gear,
  set_rain_wiper_needed,
  stalk_test_active,
  wiper_rest_tx_needed,
  wiper_test_requested,
)
from openpilot.selfdrive.car.tesla.preap_windshield_rain import (
  ACQUIRE_ON,
  BLOB_WET,
  BOKEH_ABSURD,
  BOKEH_ON,
  CLEAR_RELEASE_N,
  CLEAR_WAIT_S,
  CONNECT_RETRY_S,
  FROST_ON,
  HEAVY_ON,
  HOLD_OFF,
  HOLD_ON,
  ICE_ON,
  MIN_HOLD_N,
  REWIPE_ON,
  SCORE_ABSURD,
  SCORE_HZ,
  SCORE_INVALID,
  SCORE_OFF,
  SCORE_ON,
  SCORE_PERIOD_S,
  STALE_S,
  STREAM_FALLBACK_S,
  WIPE_CLEAR_N,
  WIPE_PULSE_S,
  WARMUP_N,
  Y_COPY_SIDE,
  WindshieldRain,
  reset_windshield_rain,
  windshield_frost_score,
  windshield_ice_score,
  windshield_looks_rainy,
  windshield_mist_score,
  windshield_obstruction_score,
  windshield_rain_score,
  y_plane_from_nv12,
)


def _rest() -> bytes:
  # Justin's parked capture, counter/checksum ignored for packing.
  return bytes.fromhex("00ff000000000000")


def _byte(dat: bytes) -> int:
  return dat[STW_WIPER_BEAM_BYTE]


def test_setting_maps_to_stalk_test_flags():
  assert not wiper_test_requested(WIPER_SETTING_OFF)
  assert wiper_test_requested(WIPER_SETTING_INTERMITTENT)
  assert wiper_test_requested(WIPER_SETTING_ON)
  assert not wiper_test_requested(WIPER_SETTING_AUTO)
  assert not wiper_test_requested(WIPER_SETTING_AUTO, False)
  assert wiper_test_requested(WIPER_SETTING_AUTO, True)
  assert not wiper_test_requested(WIPER_SETTING_OFF, True)
  assert wiper_test_requested(WIPER_SETTING_INTERMITTENT, False)
  assert wiper_test_requested(WIPER_SETTING_ON, False)
  assert not high_beam_test_requested(BEAM_SETTING_OFF)
  assert not high_beam_test_requested(BEAM_SETTING_LOW)
  assert high_beam_test_requested(BEAM_SETTING_HIGH)


def test_byte_packing_rest_wiper_high_both_never_spray():
  rest = _rest()
  assert apply_stw_wiper_beam_nibbles(rest, False, False) == rest
  assert _byte(apply_stw_wiper_beam_nibbles(rest, True, False)) == STW_WIPER_ON
  assert _byte(apply_stw_wiper_beam_nibbles(rest, False, True)) == STW_HIGH_BEAM
  assert _byte(apply_stw_wiper_beam_nibbles(rest, True, True)) == (STW_WIPER_ON | STW_HIGH_BEAM)

  packed = [
    apply_stw_wiper_beam_nibbles(rest, False, False),
    apply_stw_wiper_beam_nibbles(rest, True, False),
    apply_stw_wiper_beam_nibbles(rest, False, True),
    apply_stw_wiper_beam_nibbles(rest, True, True),
  ]
  assert all(_byte(dat) != STW_WASHER_SPRAY for dat in packed)
  assert all((_byte(dat) & 0x0F) != STW_HIGH_BEAM_FLASH for dat in packed)
  assert all(dat[:2] == rest[:2] and dat[3:] == rest[3:] for dat in packed)


def test_off_leaves_real_stalk_nibbles_alone():
  held_wiper = bytes.fromhex("00ff100000000000")
  held_high = bytes.fromhex("00ff040000000000")
  held_both = bytes.fromhex("00ff140000000000")
  spray = bytes.fromhex("00ff200000000000")
  assert apply_stw_wiper_beam_nibbles(held_wiper, False, False) == held_wiper
  assert apply_stw_wiper_beam_nibbles(held_high, False, False) == held_high
  assert apply_stw_wiper_beam_nibbles(held_both, False, False) == held_both
  # Off must not force 0 over a stalk the driver is holding, including spray.
  assert apply_stw_wiper_beam_nibbles(spray, False, False) == spray
  # On replaces spray with wiper 1 instead of sending 2.
  assert _byte(apply_stw_wiper_beam_nibbles(spray, True, False)) == STW_WIPER_ON
  # Auto dry / wipe-release cancel does clear nibble 1 to rest.
  assert apply_stw_wiper_beam_nibbles(held_wiper, False, False, clear_wiper=True) == _rest()
  assert _byte(apply_stw_wiper_beam_nibbles(held_both, False, True, clear_wiper=True)) == STW_HIGH_BEAM
  assert _byte(apply_stw_wiper_beam_nibbles(spray, False, False, clear_wiper=True)) != STW_WASHER_SPRAY


def test_overlay_off_is_identity_including_crc():
  rest = _rest()
  assert overlay_stw_wiper_beam(rest, False, False, crc_fn=lambda _: 0xAA) == rest


def test_overlay_resigns_crc_only_when_changed():
  rest = _rest()
  out = overlay_stw_wiper_beam(rest, True, False, crc_fn=lambda payload: sum(payload) & 0xFF)
  assert _byte(out) == STW_WIPER_ON
  assert out[:7] != rest[:7]
  assert out[7] == (sum(out[:7]) & 0xFF)
  held = overlay_stw_wiper_beam(rest, False, True, crc_fn=lambda payload: sum(payload) & 0xFF)
  assert hibm_nibble(held) == STW_HIGH_BEAM
  assert held[7] == (sum(held[:7]) & 0xFF)
  cleared = overlay_stw_wiper_beam(
    bytes.fromhex("00ff100000000000"), False, False, crc_fn=lambda payload: sum(payload) & 0xFF,
    clear_wiper=True)
  assert _byte(cleared) == 0
  assert cleared[7] == (sum(cleared[:7]) & 0xFF)
  assert overlay_stw_wiper_beam(rest, False, False, crc_fn=lambda _: 0xAA, clear_wiper=True) == rest


def test_high_keeps_sending_captured_00ff04_not_sna_or_rest():
  """Real stalk holds 00ff04. High must keep sending that, not a one-shot press."""
  rest = _rest()
  captured_high = bytes.fromhex("00ff04")
  assert rest[:3] == bytes.fromhex("00ff00")
  for _ in range(50):
    held = apply_stw_wiper_beam_nibbles(rest, False, True)
    assert held[:3] == captured_high
    assert hibm_nibble(held) == STW_HIGH_BEAM
    assert hibm_nibble(held) != 0
    assert hibm_nibble(held) != STW_HIGH_BEAM_FLASH
    assert (hibm_nibble(held) & STW_HIBM_MASK) != STW_HIBM_MASK  # not SNA
  off = apply_stw_wiper_beam_nibbles(rest, False, False)
  assert off == rest
  assert off[:3] == bytes.fromhex("00ff00")


def test_off_low_then_high_holds_4_again():
  rest = _rest()
  assert apply_stw_wiper_beam_nibbles(rest, False, False) == rest
  assert apply_stw_wiper_beam_nibbles(rest, False, high_beam_test_requested(BEAM_SETTING_LOW)) == rest
  held = apply_stw_wiper_beam_nibbles(rest, False, high_beam_test_requested(BEAM_SETTING_HIGH))
  assert hibm_nibble(held) == STW_HIGH_BEAM


def test_high_overlay_preserves_turn_indicator_bits():
  blinker = bytes.fromhex("00ff010000000000")
  held = apply_stw_wiper_beam_nibbles(blinker, False, True)
  assert _byte(held) & STW_TURN_MASK == 0x01
  assert hibm_nibble(held) == STW_HIGH_BEAM
  off = apply_stw_wiper_beam_nibbles(blinker, False, False)
  assert off == blinker


def test_wiper_overlay_stays_held_with_high_nibble_4():
  rest = _rest()
  held = apply_stw_wiper_beam_nibbles(rest, True, False)
  assert _byte(held) == STW_WIPER_ON
  assert apply_stw_wiper_beam_nibbles(rest, True, False) == held
  both = apply_stw_wiper_beam_nibbles(held, True, True)
  assert _byte(both) == (STW_WIPER_ON | STW_HIGH_BEAM)
  assert _byte(both) & 0xF0 == STW_WIPER_ON
  assert hibm_nibble(both) == STW_HIGH_BEAM
  still = apply_stw_wiper_beam_nibbles(held, True, True)
  assert still == both


def test_stalk_test_active_is_settings_only():
  assert not stalk_test_active(False, False)
  assert stalk_test_active(True, False)
  assert stalk_test_active(False, True)


def test_extra_forward_only_when_on_and_no_existing_0x45():
  existing = [(STW_ACTN_RQ_ADDR, b"\x00" * 8, 0)]
  assert extra_stw_forward_needed([], 10, False, False) is False
  assert extra_stw_forward_needed([], 11, True, False) is False
  assert extra_stw_forward_needed(existing, 10, True, False) is False
  assert extra_stw_forward_needed([], 10, True, False) is True
  assert extra_stw_forward_needed([], 20, False, True) is True
  # Held High must TX between 10 Hz slots so bus-0 IDLE cannot sit unopposed.
  assert extra_stw_forward_needed([], 11, False, True) is True
  assert extra_stw_forward_needed([], 11, True, False) is False
  assert extra_stw_forward_needed(existing, 11, False, True) is False
  # Camera Auto + rain holds INTERVAL1 at High's 10 ms last-win, not 10 Hz Int.
  auto_rain = wiper_test_requested(WIPER_SETTING_AUTO, True)
  auto_dry = wiper_test_requested(WIPER_SETTING_AUTO, False)
  assert extra_stw_forward_needed([], 10, auto_rain, False, collar_hold=True) is True
  assert extra_stw_forward_needed([], 11, auto_rain, False, collar_hold=True) is True
  assert extra_stw_forward_needed([], 11, auto_rain, False) is False  # Int-style without collar_hold
  assert extra_stw_forward_needed([], 10, auto_dry, False) is False
  assert extra_stw_forward_needed(existing, 10, auto_rain, False, collar_hold=True) is False
  assert extra_stw_forward_needed([], 10, auto_rain, False) is True
  # Auto dry still extra-forwards rest (cancel). Falling-edge cancel_now
  # does not wait for the 10 Hz slot. Off with no cancel leaves the stalk.
  assert extra_stw_forward_needed([], 10, auto_dry, False, wiper_cancel=True) is True
  assert extra_stw_forward_needed([], 11, auto_dry, False, wiper_cancel=True) is False
  assert extra_stw_forward_needed([], 11, auto_dry, False, wiper_cancel=True, cancel_now=True) is True
  assert extra_stw_forward_needed(existing, 10, auto_dry, False, wiper_cancel=True) is False
  assert extra_stw_forward_needed(existing, 11, auto_dry, False, wiper_cancel=True, cancel_now=True) is False


def test_settings_copy_describes_held_4_same_counter_replace():
  from openpilot.selfdrive.ui.layouts.settings.nap_content import HIGH_LOW_BEAM_DESCRIPTION
  text = HIGH_LOW_BEAM_DESCRIPTION.lower()
  assert "nibble 4" in text
  assert "hold" in text
  assert "same counter" in text
  assert "rest" in text or "idle" in text
  assert "second 0x45" in text
  assert "off/low" in text
  assert "not a one-shot tap" in text
  assert "00ff04" in text
  assert "flash" in text
  assert "pulse" not in text
  assert "sna" not in text


def test_settings_copy_describes_auto_rain_hold():
  from openpilot.selfdrive.ui.layouts.settings.nap_content import (
    WIPER_SPEED_DESCRIPTION, WIPER_SPEED_LABELS, WIPER_SPEED_VALUES,
  )
  assert WIPER_SPEED_VALUES == [0, 1, 2, 3]
  assert WIPER_SPEED_LABELS == ["Off", "Int", "On", "Auto"]
  text = WIPER_SPEED_DESCRIPTION.lower()
  assert "auto" in text
  assert "rain" in text or "windshield" in text
  assert "ice" in text or "frost" in text
  assert "camera" in text
  assert "nibble 1" in text
  assert "interval1" in text or "collar" in text
  assert "tipwipe" in text
  assert "hold" in text
  assert "spray" in text
  assert "das" in text
  assert "opt-in" in text or "not every drive" in text
  assert "int/on" in text
  assert "headlight" in text
  assert "drive" in text
  assert "reverse" in text
  assert "park" in text
  assert "rest" in text
  assert "cancel" in text
  assert "pulse" not in text
  assert "rainprob" not in text


def test_auto_rain_signal_sets_and_clears_hold():
  try:
    set_rain_wiper_needed(False)
    assert not rain_wiper_needed()
    assert not wiper_test_requested(WIPER_SETTING_AUTO, rain_wiper_needed())
    set_rain_wiper_needed(True)
    assert rain_wiper_needed()
    assert wiper_test_requested(WIPER_SETTING_AUTO, rain_wiper_needed())
    rest = _rest()
    held = apply_stw_wiper_beam_nibbles(rest, True, False)
    assert _byte(held) == STW_WIPER_ON
    assert _byte(held) != STW_WASHER_SPRAY
    set_rain_wiper_needed(False)
    assert not rain_wiper_needed()
    released = apply_stw_wiper_beam_nibbles(rest, False, False)
    assert released == rest
  finally:
    set_rain_wiper_needed(None)


def _dry_windshield(h=240, w=320, seed=0) -> np.ndarray:
  """Dry 3X ROAD stand-in: sky + distant grain up top; smoother near-glass below.

  Live dry garage scored blob≈7.5 on the old upper-mid crop. Rain now
  looks at the lower driver-side glass, so keep that region calm.
  """
  rng = np.random.RandomState(seed)
  y = np.full((h, w), 128, np.uint8)
  y[:int(h * 0.32)] = np.linspace(70, 150, int(h * 0.32), dtype=np.uint8)[:, None]
  mid = y[int(h * 0.32):int(h * 0.52)]
  mid[:] = np.clip(110 + rng.randint(-25, 26, mid.shape), 0, 255)
  near = y[int(h * 0.52):]
  # Calm near-glass. ±6 residual looked like frost (blob~3).
  near[:] = np.clip(120 + rng.randint(-2, 3, near.shape), 0, 255)
  return y


def _wet_windshield(h=240, w=320, n=25, seed=1) -> np.ndarray:
  """Same dry scene with soft near-field blobs (drops on the glass)."""
  y = _dry_windshield(h, w, seed=0)
  rng = np.random.RandomState(seed)
  r0, r1 = int(h * 0.50), int(h * 0.88)
  c0, c1 = int(w * 0.08), int(w * 0.48)
  for _ in range(n):
    rad = rng.randint(2, 8)
    cy = rng.randint(r0 + rad, r1 - rad)
    cx = rng.randint(c0 + rad, c1 - rad)
    yy, xx = np.ogrid[-rad:rad + 1, -rad:rad + 1]
    mask = yy * yy + xx * xx <= rad * rad
    dist = np.sqrt(yy * yy + xx * xx)
    bump = (1.0 - dist / max(rad, 1)) * rng.randint(50, 120)
    patch = y[cy - rad:cy + rad + 1, cx - rad:cx + rad + 1].astype(np.float32)
    patch[mask] += bump[mask]
    y[cy - rad:cy + rad + 1, cx - rad:cx + rad + 1] = np.clip(patch, 0, 255).astype(np.uint8)
  return y


def _dense_bead_windshield(h=240, w=320, n=1200, seed=12) -> np.ndarray:
  """Human-obvious: beads covering the whole pane (Justin's rest-in-rain photo).

  Packed droplets raise sharp-residual structure and can sat-highlight.
  That used to score 0 as foliage/lamps. Must HOLD at least as strongly
  as light soft-bokeh.
  """
  y = _dry_windshield(h, w, seed=0)
  rng = np.random.RandomState(seed)
  r0, r1 = int(h * 0.05), int(h * 0.90)
  c0, c1 = int(w * 0.05), int(w * 0.95)
  for _ in range(n):
    rad = rng.randint(2, 7)
    cy = rng.randint(r0 + rad, r1 - rad)
    cx = rng.randint(c0 + rad, c1 - rad)
    yy, xx = np.ogrid[-rad:rad + 1, -rad:rad + 1]
    mask = yy * yy + xx * xx <= rad * rad
    dist = np.sqrt(yy * yy + xx * xx)
    bump = (1.0 - dist / max(rad, 1)) * rng.randint(60, 160)
    patch = y[cy - rad:cy + rad + 1, cx - rad:cx + rad + 1].astype(np.float32)
    patch[mask] += bump[mask]
    y[cy - rad:cy + rad + 1, cx - rad:cx + rad + 1] = np.clip(patch, 0, 255).astype(np.uint8)
  return y


def _frost_windshield(h=240, w=320, seed=3) -> np.ndarray:
  """Dry scene with near-field crystal mottle on the glass."""
  y = _dry_windshield(h, w, seed=0).astype(np.float32)
  rng = np.random.RandomState(seed)
  noise = rng.randn(h, w)
  k = 3
  p = np.pad(noise, k, mode="edge")
  acc = np.zeros_like(noise)
  for i in range(2 * k + 1):
    for j in range(2 * k + 1):
      acc += p[i:i + h, j:j + w]
  acc /= float((2 * k + 1) ** 2)
  return np.clip(y * 0.85 + 20.0 + acc * 18.0, 0, 255).astype(np.uint8)


def _ice_sheet(h=240, w=320, seed=4) -> np.ndarray:
  """Milky ice sheet: contrast of the view through the glass is collapsed."""
  y = _dry_windshield(h, w, seed=0).astype(np.float32)
  rng = np.random.RandomState(seed)
  y = (y - y.mean()) * 0.35 + 140.0
  noise = rng.randn(h, w)
  k = 5
  p = np.pad(noise, k, mode="edge")
  acc = np.zeros_like(noise)
  for i in range(2 * k + 1):
    acc += p[i:i + h, k:k + w]
  acc /= float(2 * k + 1)
  return np.clip(y + acc * 8.0, 0, 255).astype(np.uint8)


def _bokeh_windshield(h=240, w=320, n=10, seed=5) -> np.ndarray:
  """Far-focus 3X ROAD: large soft circles of confusion on the glass.

  Matches Justin's live ROAD UI in rain — not sharp phone-bead close-ups.
  """
  y = _dry_windshield(h, w, seed=0).astype(np.float32)
  rng = np.random.RandomState(seed)
  yy, xx = np.mgrid[0:h, 0:w]
  for _ in range(n):
    cy = rng.uniform(h * 0.50, h * 0.86)
    cx = rng.uniform(w * 0.08, w * 0.48)
    sig = rng.uniform(min(h, w) * 0.055, min(h, w) * 0.14)
    amp = rng.uniform(35.0, 80.0) * rng.choice([1.0, 1.0, 0.85, -0.4])
    y += amp * np.exp(-((yy - cy) ** 2 + (xx - cx) ** 2) / (2.0 * sig * sig))
  return np.clip(y, 0, 255).astype(np.uint8)


def _overcast_windshield(h=240, w=320, seed=21) -> np.ndarray:
  """Dry overcast: sky wash + huge soft clouds. Low bokeh, not rain blobs."""
  y = _dry_windshield(h, w, seed=0).astype(np.float32)
  rng = np.random.RandomState(seed)
  yy, xx = np.mgrid[0:h, 0:w]
  for _ in range(3):
    cy = rng.uniform(h * 0.05, h * 0.35)
    cx = rng.uniform(w * 0.2, w * 0.8)
    sig = rng.uniform(70.0, 120.0)
    amp = rng.uniform(8.0, 16.0)
    y += amp * np.exp(-((yy - cy) ** 2 + (xx - cx) ** 2) / (2.0 * sig * sig))
  return np.clip(y, 0, 255).astype(np.uint8)


def _low_bokeh_overcast(h=240, w=320, seed=13) -> np.ndarray:
  """Weak false bokeh under BOKEH_ON — dry/overcast texture, not rain."""
  y = _dry_windshield(h, w, seed=0).astype(np.float32)
  rng = np.random.RandomState(seed)
  yy, xx = np.mgrid[0:h, 0:w]
  for _ in range(8):
    cy = rng.uniform(h * 0.18, h * 0.56)
    cx = rng.uniform(w * 0.15, w * 0.85)
    sig = rng.uniform(min(h, w) * 0.04, min(h, w) * 0.10)
    amp = rng.uniform(10.0, 22.0)
    y += amp * np.exp(-((yy - cy) ** 2 + (xx - cx) ** 2) / (2.0 * sig * sig))
  return np.clip(y, 0, 255).astype(np.uint8)


def _false_bokeh_overcast(h=240, w=320, seed=40) -> np.ndarray:
  """Old 1.5–2.1 bokeh band: dry/overcast scene texture, not real rain (~3+)."""
  y = _dry_windshield(h, w, seed=0).astype(np.float32)
  rng = np.random.RandomState(seed)
  yy, xx = np.mgrid[0:h, 0:w]
  for _ in range(8):
    cy = rng.uniform(h * 0.18, h * 0.56)
    cx = rng.uniform(w * 0.15, w * 0.85)
    sig = rng.uniform(min(h, w) * 0.05, min(h, w) * 0.12)
    amp = rng.uniform(20.0, 38.0) * rng.choice([1.0, 1.0, 0.85, -0.3])
    y += amp * np.exp(-((yy - cy) ** 2 + (xx - cx) ** 2) / (2.0 * sig * sig))
  return np.clip(y, 0, 255).astype(np.uint8)


def _heavy_bokeh_windshield(h=240, w=320, seed=6) -> np.ndarray:
  """Meaningful wet: overlapping milky drops on the driver-side near glass.

  Must not rely on distant driveway grain (that is what dry garage scored).
  """
  y = _dry_windshield(h, w, seed=0).astype(np.float32)
  rng = np.random.RandomState(seed)
  r0, r1 = int(h * 0.48), int(h * 0.90)
  c0, c1 = int(w * 0.04), int(w * 0.52)
  for _ in range(90):
    rad = rng.randint(5, 16)
    cy = rng.randint(r0 + rad, r1 - rad)
    cx = rng.randint(c0 + rad, c1 - rad)
    yy, xx = np.ogrid[-rad:rad + 1, -rad:rad + 1]
    mask = yy * yy + xx * xx <= rad * rad
    dist = np.sqrt(yy * yy + xx * xx)
    bump = (1.0 - dist / max(rad, 1)) * rng.randint(70, 150)
    patch = y[cy - rad:cy + rad + 1, cx - rad:cx + rad + 1]
    patch[mask] += bump[mask]
    y[cy - rad:cy + rad + 1, cx - rad:cx + rad + 1] = patch
  return np.clip(y, 0, 255).astype(np.uint8)


def _mist_windshield(h=240, w=320, seed=17) -> np.ndarray:
  """Highway light mist: veil + small streaks on near-glass; far road stays sharp."""
  y = _dry_windshield(h, w, seed=0).astype(np.float32)
  rng = np.random.RandomState(seed)
  r0, r1 = int(h * 0.56), int(h * 0.90)
  c0, c1 = int(w * 0.06), int(w * 0.50)
  patch = y[r0:r1, c0:c1]
  m = float(patch.mean())
  y[r0:r1, c0:c1] = (patch - m) * 0.32 + m + 10.0
  yy, xx = np.mgrid[0:h, 0:w]
  for _ in range(14):
    cy = rng.uniform(h * 0.58, h * 0.88)
    cx = rng.uniform(w * 0.08, w * 0.46)
    sigy = rng.uniform(h * 0.012, h * 0.04)
    sigx = rng.uniform(w * 0.006, w * 0.016)
    amp = rng.uniform(18.0, 36.0)
    y += amp * np.exp(-((yy - cy) ** 2) / (2.0 * sigy * sigy) - ((xx - cx) ** 2) / (2.0 * sigx * sigx))
  return np.clip(y, 0, 255).astype(np.uint8)


def _fine_mist_windshield(h=240, w=320, seed=19) -> np.ndarray:
  """Dense fine drizzle film on near-glass (Justin 13 mph photo). Far road stays."""
  y = _dry_windshield(h, w, seed=0).astype(np.float32)
  rng = np.random.RandomState(seed)
  r0, r1 = int(h * 0.48), int(h * 0.90)
  c0, c1 = int(w * 0.06), int(w * 0.50)
  patch = y[r0:r1, c0:c1]
  m = float(patch.mean())
  y[r0:r1, c0:c1] = (patch - m) * 0.40 + m + 6.0
  for _ in range(280):
    rad = rng.randint(1, 3)
    cy = rng.randint(r0 + rad, r1 - rad)
    cx = rng.randint(c0 + rad, c1 - rad)
    yy, xx = np.ogrid[-rad:rad + 1, -rad:rad + 1]
    mask = yy * yy + xx * xx <= rad * rad
    bump = rng.uniform(16.0, 40.0)
    sl = y[cy - rad:cy + rad + 1, cx - rad:cx + rad + 1]
    sl[mask] += bump
    y[cy - rad:cy + rad + 1, cx - rad:cx + rad + 1] = sl
  return np.clip(y, 0, 255).astype(np.uint8)


def _atmos_fog_windshield(h=240, w=320, seed=18) -> np.ndarray:
  """Distant atmospheric fog: whole-frame wash, no near-glass streaks."""
  y = _dry_windshield(h, w, seed=0).astype(np.float32)
  y = (y - y.mean()) * 0.22 + 145.0
  return np.clip(y, 0, 255).astype(np.uint8)


def test_windshield_camera_wet_holds_and_dry_releases():
  dry = _dry_windshield()
  wet = _heavy_bokeh_windshield()
  assert windshield_rain_score(dry) < SCORE_OFF
  assert windshield_rain_score(wet) >= SCORE_ON
  assert not windshield_looks_rainy(dry)
  assert windshield_looks_rainy(wet)
  assert not wiper_test_requested(WIPER_SETTING_AUTO, windshield_looks_rainy(dry))
  assert wiper_test_requested(WIPER_SETTING_AUTO, windshield_looks_rainy(wet))
  rest = _rest()
  held = apply_stw_wiper_beam_nibbles(rest, windshield_looks_rainy(wet), False)
  assert _byte(held) == STW_WIPER_ON
  assert _byte(held) != STW_WASHER_SPRAY
  released = apply_stw_wiper_beam_nibbles(rest, windshield_looks_rainy(dry), False)
  assert released == rest


def test_windshield_soft_bokeh_holds_and_dry_releases():
  """3X ROAD is far-focused: rain is large soft bokeh, not sharp beads."""
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import (
    _BOKEH_ROWS, _COLS, _band, _near_features,
  )

  dry = _dry_windshield()
  bokeh = _bokeh_windshield()
  _blob, speckle, _sparse, _sat, _structure, bokeh_e = _near_features(_band(bokeh, _BOKEH_ROWS, _COLS))
  # Soft circles of confusion: not the old sparse-speckle drop gate.
  assert speckle < 0.012
  assert bokeh_e >= BOKEH_ON
  assert windshield_rain_score(dry) < SCORE_OFF
  assert windshield_rain_score(bokeh) >= SCORE_ON
  assert not windshield_looks_rainy(dry)
  assert windshield_looks_rainy(bokeh)
  assert not wiper_test_requested(WIPER_SETTING_AUTO, windshield_looks_rainy(dry))
  assert wiper_test_requested(WIPER_SETTING_AUTO, windshield_looks_rainy(bokeh))
  rest = _rest()
  held = apply_stw_wiper_beam_nibbles(rest, windshield_looks_rainy(bokeh), False)
  assert _byte(held) == STW_WIPER_ON
  assert _byte(held) != STW_WASHER_SPRAY
  assert apply_stw_wiper_beam_nibbles(rest, windshield_looks_rainy(dry), False) == rest

  # Light sprinkle-level bokeh looks rainy but must not acquire.
  det = WindshieldRain()
  _skip_warmup(det)
  for _ in range(MIN_HOLD_N + 8):
    assert not det.update_from_y(bokeh)
  assert not det.hold


def test_heavy_soft_bokeh_over_driveway_holds():
  """Later live 3X ROAD UI: more rain, large overlapping milky blobs over cars."""
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import (
    _BOKEH_ROWS, _COLS, _band, _near_features,
  )

  dry = _dry_windshield()
  heavy = _heavy_bokeh_windshield()
  blob, speckle, _sparse, _sat, _structure, bokeh_e = _near_features(_band(heavy, _BOKEH_ROWS, _COLS))
  assert speckle < 0.35
  # Soft defocus is present; rain score may also come from the speckle path.
  assert bokeh_e >= 1.5
  assert blob >= BLOB_WET
  assert windshield_rain_score(dry) < SCORE_OFF
  assert windshield_rain_score(heavy) >= SCORE_ON
  assert windshield_looks_rainy(heavy)
  assert not windshield_looks_rainy(dry)
  rest = _rest()
  held = apply_stw_wiper_beam_nibbles(rest, windshield_looks_rainy(heavy), False)
  assert _byte(held) == STW_WIPER_ON
  assert _byte(held) != STW_WASHER_SPRAY
  det = WindshieldRain()
  saw = False
  for _ in range(16):
    if det.update_from_y(heavy):
      saw = True
      break
  assert saw
  _expire_wipe(det)
  assert not det.hold


def test_rain_score_is_monotonic_at_or_above_wetness():
  """Anything at or above the wetness floor wipes. More water must not score drier.

  Live miss: light soft-bokeh fired; heavy overlapping milky (high blob, bokeh
  under BOKEH_ON, ratio under the old 0.22 veto) scored 0 on the mid band.
  """
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import (
    _BOKEH_RATIO, _BOKEH_ROWS, _COLS, _STRUCTURE_RAIN,
    _band, _near_features, _rain_from_band,
  )

  dry = _dry_windshield()
  light = _bokeh_windshield()
  heavy = _heavy_bokeh_windshield()
  light_s = windshield_rain_score(light)
  heavy_s = windshield_rain_score(heavy)
  assert light_s >= SCORE_ON
  assert heavy_s >= SCORE_ON
  assert heavy_s >= light_s
  assert windshield_obstruction_score(heavy) >= windshield_obstruction_score(light)
  assert windshield_looks_rainy(light)
  assert windshield_looks_rainy(heavy)
  assert not windshield_looks_rainy(dry)

  blob, speckle, _sparse, _sat, structure, bokeh_e = _near_features(_band(heavy, _BOKEH_ROWS, _COLS))
  assert blob >= BLOB_WET
  assert structure < _STRUCTURE_RAIN
  # Meaningful wet must still score if the old ratio gate would have vetoed it.
  if bokeh_e < BOKEH_ON or bokeh_e / (blob + 0.2) < _BOKEH_RATIO:
    assert _rain_from_band(_band(heavy, _BOKEH_ROWS, _COLS)) >= SCORE_ON
  mid = _rain_from_band(_band(heavy, _BOKEH_ROWS, _COLS))
  assert mid >= SCORE_ON
  assert mid >= light_s
  # Dense wet blob must not be classified as foliage (structure stays low).
  assert blob > 8.0

  det = WindshieldRain()
  saw = False
  for _ in range(MIN_HOLD_N + 4):
    if det.update_from_y(heavy):
      saw = True
      break
  assert saw
  assert det.hold

  rest = _rest()
  assert _byte(apply_stw_wiper_beam_nibbles(rest, True, False)) == STW_WIPER_ON
  _expire_wipe(det)
  assert not det.hold
  assert apply_stw_wiper_beam_nibbles(rest, False, False) == rest


def test_valid_heavy_obstruction_is_not_forced_dry():
  """Old SCORE_ABSURD=12 zeroed real heavy scores. 15 is wet, 49165 is garbage."""
  assert SCORE_ABSURD < SCORE_INVALID
  latch = WindshieldRain()
  _skip_warmup(latch)
  for _ in range(MIN_HOLD_N - 1):
    assert not latch._update_score(15.0)
    assert not latch.hold
  assert latch._update_score(15.0)
  assert latch.hold
  assert latch.last_score == 15.0

  garbage = WindshieldRain()
  for _ in range(MIN_HOLD_N + 8):
    assert not garbage._update_score(49165.0)
    assert not garbage.hold
    assert garbage.last_score == 0.0


def test_dense_beads_covering_glass_hold_at_least_as_light_bokeh():
  """Packed full-pane beads must wipe. Structure/sat must not fail closed as foliage."""
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import (
    _BOKEH_ROWS, _NEAR_ROWS, _COLS, _STRUCTURE_RAIN, _SAT_MAX, _band, _near_features, _rain_from_band,
  )

  dry = _dry_windshield()
  light = _bokeh_windshield()
  dense = _dense_bead_windshield()
  light_s = windshield_rain_score(light)
  dense_s = windshield_rain_score(dense)
  assert light_s >= SCORE_ON
  assert dense_s >= SCORE_ON
  assert dense_s >= light_s
  assert windshield_looks_rainy(dense)
  assert not windshield_looks_rainy(dry)

  # This scene is why wipe sat at rest: both bands can exceed the old
  # structure/sat vetoes while bokeh is still rain-like.
  struct_hi = False
  for rows in (_NEAR_ROWS, _BOKEH_ROWS):
    blob, speckle, sparse, sat, structure, bokeh_e = _near_features(_band(dense, rows, _COLS))
    band = _rain_from_band(_band(dense, rows, _COLS))
    if structure > _STRUCTURE_RAIN or sat > _SAT_MAX:
      struct_hi = True
      assert bokeh_e >= BOKEH_ON or blob >= BLOB_WET or speckle >= 0.012
      assert band >= SCORE_ON
  assert struct_hi or dense_s >= light_s

  det = WindshieldRain()
  saw = False
  for _ in range(MIN_HOLD_N + 4):
    if det.update_from_y(dense):
      saw = True
      break
  assert saw
  assert det.hold
  rest = _rest()
  assert _byte(apply_stw_wiper_beam_nibbles(rest, True, False)) == STW_WIPER_ON
  _expire_wipe(det)
  assert not det.hold


def test_one_below_wet_idle_look_resets_consecutive_acquire():
  """Two consecutive clearly-wet idle looks acquire. One dry in between resets."""
  wet = HEAVY_ON + 0.8
  det = WindshieldRain()
  _skip_warmup(det)
  for _ in range(MIN_HOLD_N - 1):
    assert not det._update_score(wet)
    assert not det.hold
  assert det._hold_n == MIN_HOLD_N - 1
  assert not det._update_score(0.0)
  assert not det.hold
  assert det._hold_n == 0
  assert not det._update_score(wet)
  assert not det.hold
  assert det._update_score(wet)
  assert det.hold


def test_windshield_ice_and_frost_hold_like_rain():
  dry = _dry_windshield()
  frost = _frost_windshield()
  ice = _ice_sheet()
  assert windshield_obstruction_score(dry) < HOLD_ON
  assert windshield_frost_score(frost) >= FROST_ON
  assert windshield_ice_score(ice) >= ICE_ON
  assert windshield_looks_rainy(frost)
  assert windshield_looks_rainy(ice)
  assert not windshield_looks_rainy(dry)
  rest = _rest()
  assert _byte(apply_stw_wiper_beam_nibbles(rest, True, False)) == STW_WIPER_ON
  assert apply_stw_wiper_beam_nibbles(rest, False, False) == rest


def test_windshield_rejects_foliage_and_headlamps():
  h, w = 240, 320
  rng = np.random.RandomState(0)
  foliage = np.full((h, w), 80, np.uint8)
  foliage[int(h * 0.48):int(h * 0.90), :int(w * 0.50)] = rng.randint(
    40, 160, (int(h * 0.90) - int(h * 0.48), int(w * 0.50))).astype(np.uint8)
  lamps = np.full((h, w), 30, np.uint8)
  lamps[int(h * 0.55):int(h * 0.70), int(w * 0.10):int(w * 0.22)] = 250
  lamps[int(h * 0.55):int(h * 0.70), int(w * 0.70):int(w * 0.82)] = 250
  assert windshield_rain_score(foliage) < SCORE_ON
  assert windshield_rain_score(lamps) < SCORE_ON
  assert windshield_frost_score(foliage) < FROST_ON
  assert windshield_ice_score(foliage) < ICE_ON
  assert windshield_frost_score(lamps) < FROST_ON
  assert windshield_ice_score(lamps) < ICE_ON
  assert not windshield_looks_rainy(foliage)
  assert not windshield_looks_rainy(lamps)


def test_visionipc_retries_after_failure(monkeypatch):
  """One VisionIpc exception must not permanently dry Auto."""
  import time as time_mod

  now = {"t": 1000.0}
  monkeypatch.setattr(time_mod, "monotonic", lambda: now["t"])

  class _Boom:
    def is_connected(self):
      return True

    def recv(self, timeout_ms=0):
      raise RuntimeError("vipc down")

  class _Quiet:
    def is_connected(self):
      return True

    def recv(self, timeout_ms=0):
      return None

  det = WindshieldRain()
  det._client = _Boom()
  assert det._recv_y() is None
  assert det._failed
  assert det._client is None
  assert det.last_err == "RuntimeError"
  assert det._recv_y() is None
  assert det._failed

  now["t"] += CONNECT_RETRY_S + 0.05
  det._client = _Quiet()
  assert det._recv_y() is None
  assert not det._failed
  assert det.poll() is False


def test_visionipc_falls_back_to_wide_without_frames():
  det = WindshieldRain()
  det._stream_t0 = 5000.0
  assert det.stream == "ROAD"
  det._maybe_fallback_stream(5000.0 + STREAM_FALLBACK_S + 0.05)
  assert det.stream == "WIDE"
  assert det._client is None


def _nv12_buf(y):
  h, w = y.shape
  class _Buf:
    width = int(w)
    height = int(h)
    stride = int(w)
    uv_offset = int(w * h)
    data = np.concatenate([y.reshape(-1), np.full((w * h) // 2, 128, np.uint8)])
  return _Buf()


class _CountingVisionClient:
  def __init__(self, buf):
    self.buf = buf
    self.n_recv = 0

  def is_connected(self):
    return True

  def recv(self, timeout_ms=0):
    self.n_recv += 1
    wait = (timeout_ms or 0) / 1000.0
    time.sleep(0.02 if wait <= 0 else min(0.03, wait))
    return self.buf


def _skip_warmup(det) -> None:
  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain
  det._warm_n = int(rain.WARMUP_N)


def _acquire_score(det, score: float) -> None:
  """Stop on first HOLD. Skip helper warmup; one-sweep pulse is wall-clock."""
  _skip_warmup(det)
  for _ in range(MIN_HOLD_N + 4):
    det._update_score(score)
    if det.hold:
      break
  assert det.hold
  assert det._wipe_t0 > 0.0


def _acquire_y(det, y) -> None:
  _skip_warmup(det)
  for _ in range(MIN_HOLD_N + 4):
    det.update_from_y(y)
    if det.hold:
      break
  assert det.hold
  assert det._wipe_t0 > 0.0


def _expire_wipe(det) -> None:
  """End the one-sweep nibble-1 pulse the way poll() does on wall-clock."""
  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain

  assert det.hold
  det._wipe_t0 = time.monotonic() - float(rain.WIPE_PULSE_S) - 0.05
  det._apply_wipe_pulse()
  assert not det.hold
  assert det._wait_t0 > 0.0


def _expire_wait(det) -> None:
  """Skip CLEAR_WAIT_S so the next score is an assess, not blade-FOV."""
  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain

  assert det._wait_t0 > 0.0
  det._wait_t0 = time.monotonic() - float(rain.CLEAR_WAIT_S) - 0.05


def test_helper_holds_soft_bokeh_and_poll_does_not_recv(monkeypatch):
  """stock_cc / card is CTRL_HIGH: poll must not drain VisionIpc. Heavy bokeh HOLD."""
  import time as time_mod

  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain

  monkeypatch.setattr(rain, "SCORE_PERIOD_S", 0.05)
  monkeypatch.setattr(rain, "WIPE_PULSE_S", 60.0)
  bokeh = _heavy_bokeh_windshield()
  client = _CountingVisionClient(_nv12_buf(bokeh))
  det = WindshieldRain()
  det._client = client
  det.start_helper()
  try:
    deadline = time_mod.monotonic() + 2.0
    while time_mod.monotonic() < deadline and not det.hold:
      time_mod.sleep(0.02)
    assert det.helper_alive
    assert det.hold
    assert det.n_frames >= 1
    n_poll = det._poll_recv
    for _ in range(25):
      assert det.poll() is True
    assert det._poll_recv == n_poll
    assert det.hold
  finally:
    det.stop_helper()


def test_helper_dry_road_does_not_hold():
  import time as time_mod

  dry = _dry_windshield()
  client = _CountingVisionClient(_nv12_buf(dry))
  det = WindshieldRain()
  det._client = client
  det.start_helper()
  try:
    deadline = time_mod.monotonic() + 1.0
    while time_mod.monotonic() < deadline and det.n_frames < 4:
      time_mod.sleep(0.02)
    assert det.n_frames >= 1
    assert not det.hold
    assert det.poll() is False
    assert det._poll_recv == 0
  finally:
    det.stop_helper()


def test_stock_cc_update_primes_visionipc_helper(monkeypatch):
  """Auto stock_cc.update starts ROAD drain; poll does not recv on that thread."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body
  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain

  reset_windshield_rain()
  monkeypatch.setattr(body, "RAIN_HELPER_START_DELAY_S", 0.0)
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  monkeypatch.setattr(body, "requested_wiper_test", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  class _Fake:
    def _send(self, CS, tesla_can, bus, button):
      return (STW_ACTN_RQ_ADDR, b"\x00", bus)

  fake = _Fake()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0})
  reset_auto_gates()
  try:
    out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
    # Auto dry still extra-forwards rest (cancel latched Int). Helper must start.
    assert len(out) == 1
    assert out[0][0] == STW_ACTN_RQ_ADDR
    assert rain._detector is not None
    assert rain._detector._helper_started
    assert rain._detector.poll() is False
    assert rain._detector._poll_recv == 0
  finally:
    reset_windshield_rain()
    reset_auto_gates()


def test_auto_helper_does_not_start_during_engage_delay(monkeypatch):
  """Do not subscribe ROAD in the first RAIN_HELPER_START_DELAY_S of Auto."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body
  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain

  reset_windshield_rain()
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])

  class _Fake:
    def _send(self, CS, tesla_can, bus, button):
      return (STW_ACTN_RQ_ADDR, b"\x00", bus)

  reset_auto_gates()
  try:
    assert body.RAIN_HELPER_START_DELAY_S >= 2.0
    body.stock_cc_update_with_overlay(_Fake(), SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0}), 10, None, 0)
    assert rain._detector is None or not rain._detector._helper_started
  finally:
    reset_windshield_rain()
    reset_auto_gates()


def test_poll_never_recvs_on_card_thread():
  """Helper down used to recv+numpy on poll() — that lagged selfdrive on engage."""
  import inspect

  class _Boom:
    def is_connected(self):
      return True

    def recv(self, timeout_ms=0):
      raise RuntimeError("poll must not recv")

  det = WindshieldRain()
  det._client = _Boom()
  det.hold = True
  assert det.poll() is False
  assert det._poll_recv == 0
  src = inspect.getsource(WindshieldRain.poll)
  assert "_recv_y" not in src
  assert "update_from_y" not in src


def test_stock_cc_int_stops_rain_helper_and_drops_hold(monkeypatch):
  """Int/On/Off keep nibble 1 / stalk without ROAD. Switching off Auto must drop HOLD."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body
  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain

  reset_windshield_rain()
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])

  class _Fake:
    def _send(self, CS, tesla_can, bus, button):
      return (STW_ACTN_RQ_ADDR, b"\x00", bus)

  fake = _Fake()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0})
  reset_auto_gates()
  try:
    det = rain.ensure_windshield_rain_helper()
    det.hold = True
    assert det._helper_started
    monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
      WIPER_SETTING_INTERMITTENT if key == NAP_WIPER_SPEED else default
    ))
    out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
    assert not det._helper_started
    assert not det.hold
    assert det.poll() is False
    assert det._poll_recv == 0
    assert any(msg[0] == STW_ACTN_RQ_ADDR for msg in out)
  finally:
    reset_windshield_rain()
    reset_auto_gates()


def test_install_does_not_start_rain_helper():
  """card.__init__ must not subscribe ROAD — that hit camerad at engage."""
  import inspect
  from openpilot.selfdrive.car.tesla.preap_body_controls import install_body_controls_test
  src = inspect.getsource(install_body_controls_test)
  assert "ensure_windshield_rain_helper" not in src
  assert "start_helper" not in src


def test_dense_bead_sparse_below_8_is_rain():
  """Sparse≈6.85 mid-blob used to be a reject. Meaningful wet still rains."""
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import _SPARSE_MIN, _SPARSE_RAIN_MIN
  assert _SPARSE_RAIN_MIN <= 6.85 < _SPARSE_MIN
  heavy = _heavy_bokeh_windshield()
  assert windshield_rain_score(heavy) >= SCORE_ON
  assert windshield_looks_rainy(heavy)
  dense = _dense_bead_windshield()
  assert windshield_rain_score(dense) >= SCORE_ON
  assert windshield_looks_rainy(dense)


def test_windshield_latch_holds_then_releases():
  det = WindshieldRain()
  wet = _heavy_bokeh_windshield()
  dry = _dry_windshield()
  saw_wet = False
  for _ in range(16):
    if det.update_from_y(wet):
      saw_wet = True
      break
  assert saw_wet
  assert det.update_from_y(dry)
  assert det.hold
  _expire_wipe(det)
  assert not det.hold
  frost = _frost_windshield()
  assert windshield_looks_rainy(frost)
  assert windshield_obstruction_score(frost) < ACQUIRE_ON
  frost_det = WindshieldRain()
  for _ in range(MIN_HOLD_N + 8):
    assert not frost_det.update_from_y(frost)
  ice = _ice_sheet()
  assert windshield_looks_rainy(ice)
  assert windshield_obstruction_score(ice) < ACQUIRE_ON
  ice_det = WindshieldRain()
  for _ in range(MIN_HOLD_N + 8):
    assert not ice_det.update_from_y(ice)


def test_hold_rides_brief_wipe_clear_then_releases_on_sustained_dry():
  """One-sweep pulse: dry ROAD during WIPE_PULSE_S does not abort the wipe."""
  det = WindshieldRain()
  wet = _heavy_bokeh_windshield()
  dry = _dry_windshield()
  _acquire_y(det, wet)
  assert det.hold
  assert det._wipe_t0 > 0.0
  assert det.update_from_y(dry)
  assert det.hold
  _expire_wipe(det)
  assert not det.hold
  assert apply_stw_wiper_beam_nibbles(_rest(), False, False) == _rest()


def test_sustained_dry_releases_when_wipe_pulse_ends():
  """HOLD is the one-sweep pulse. poll() / pulse elapsed drops it; dry then stays off."""
  assert MIN_HOLD_N <= CLEAR_RELEASE_N
  det = WindshieldRain()
  wet = _heavy_bokeh_windshield()
  dry = _dry_windshield()
  _acquire_y(det, wet)
  assert det._hold_n >= MIN_HOLD_N
  _expire_wipe(det)
  assert not det.hold
  _expire_wait(det)
  assert not det.update_from_y(dry)
  assert not det.hold
  assert windshield_obstruction_score(dry) < HOLD_OFF
  rest = _rest()
  assert apply_stw_wiper_beam_nibbles(rest, False, False) == rest


def test_dry_overcast_low_bokeh_does_not_stay_held():
  """Dry overcast / weak false bokeh must not acquire, and must stay off after a wipe."""
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import (
    _BOKEH_ROWS, _COLS, _band, _near_features,
  )

  overcast = _overcast_windshield()
  low = _low_bokeh_overcast()
  false_bokeh = _false_bokeh_overcast()
  dry = _dry_windshield()
  wet = _heavy_bokeh_windshield()
  for y in (overcast, low, false_bokeh, dry):
    obs = windshield_obstruction_score(y)
    _blob, _speckle, _sparse, _sat, _structure, bokeh_e = _near_features(_band(y, _BOKEH_ROWS, _COLS))
    assert obs < HOLD_ON
    assert bokeh_e < BOKEH_ON or obs < HOLD_ON
    assert not windshield_looks_rainy(y)
    det_dry = WindshieldRain()
    for _ in range(CLEAR_RELEASE_N + MIN_HOLD_N + 8):
      assert not det_dry.update_from_y(y)

  # After one wipe + 2 s wait, overcast residual must not re-wipe.
  for residual_y in (overcast, low, false_bokeh, dry):
    det = WindshieldRain()
    _acquire_y(det, wet)
    _expire_wipe(det)
    _expire_wait(det)
    assert not det.update_from_y(residual_y)
    assert not det.hold


def test_false_bokeh_below_bar_is_not_rain():
  """Clear/overcast texture in the old 1.5 bokeh band must not look rainy."""
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import (
    _BOKEH_ROWS, _COLS, _band, _near_features,
  )

  y = _false_bokeh_overcast()
  _blob, speckle, _sparse, _sat, _structure, bokeh_e = _near_features(_band(y, _BOKEH_ROWS, _COLS))
  assert speckle < 0.012
  # In the old false-positive band, but under the raised rain bar.
  assert bokeh_e < BOKEH_ON
  assert windshield_rain_score(y) < SCORE_ON
  assert windshield_obstruction_score(y) < HOLD_ON
  assert not windshield_looks_rainy(y)
  rest = _rest()
  assert apply_stw_wiper_beam_nibbles(rest, False, False) == rest


def test_absurd_bokeh_scale_is_invalid_dry_and_releases():
  """Live clear glass logged bokeh=49165. Garbage scale must not HOLD."""
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import _near_features, _band, _BOKEH_ROWS, _COLS

  dry = _dry_windshield()
  wet = _heavy_bokeh_windshield()
  # 16-bit-ish / exploded float plane (Justin ~49165).
  huge = dry.astype(np.float32) * 200.0
  u16 = (dry.astype(np.float32) * 256.0)
  assert float(huge.max()) > 20000.0
  for y in (huge, u16):
    _blob, _speckle, _sparse, _sat, _structure, bokeh_e = _near_features(_band(y, _BOKEH_ROWS, _COLS))
    assert bokeh_e <= BOKEH_ABSURD
    assert windshield_rain_score(y) < SCORE_ON
    assert windshield_obstruction_score(y) < HOLD_ON
    assert windshield_obstruction_score(y) < SCORE_ABSURD
    assert not windshield_looks_rainy(y)
    det_dry = WindshieldRain()
    for _ in range(CLEAR_RELEASE_N + MIN_HOLD_N + 8):
      assert not det_dry.update_from_y(y)
      assert det_dry.last_bokeh <= BOKEH_ABSURD
      assert det_dry.last_score < HOLD_ON

  # Already held from real rain: pulse ends, then garbage is dry, not "rain returned".
  det = WindshieldRain()
  _acquire_y(det, wet)
  _expire_wipe(det)
  _expire_wait(det)
  assert not det.update_from_y(huge)
  assert not det.hold
  assert det.last_bokeh <= BOKEH_ABSURD

  # Direct garbage obstruction must not latch or keep HOLD.
  det2 = WindshieldRain()
  for _ in range(CLEAR_RELEASE_N + 8):
    assert not det2._update_score(49165.0)
  det3 = WindshieldRain()
  _acquire_score(det3, HEAVY_ON + 0.8)
  assert det3._update_score(0.0)
  assert det3.hold
  _expire_wipe(det3)
  _expire_wait(det3)
  assert not det3._update_score(0.0)
  assert not det3.hold
  assert not det3._update_score(49165.0)
  assert not det3.hold


def test_elevated_residual_below_rewipe_exits_loop():
  """After one wipe + settle, residual / marginal / HOLD_ON must not re-wipe."""
  assert WIPE_CLEAR_N < CLEAR_RELEASE_N
  assert HOLD_OFF < HOLD_ON
  assert HOLD_ON < ACQUIRE_ON <= REWIPE_ON
  assert REWIPE_ON == HEAVY_ON

  for residual in (0.0, 0.45, 0.70, 0.95, 1.05, HOLD_ON, 1.7, 3.16, 4.42, 6.1, 6.77, 8.27):
    det = WindshieldRain()
    _acquire_score(det, HEAVY_ON + 0.8)
    _expire_wipe(det)
    _expire_wait(det)
    assert not det._update_score(residual), residual
    assert not det.hold
    assert not det._post_wipe


def test_light_sprinkle_never_enters_wipe_loop():
  """Very little sprinkles / residual beads: idle 4 s watch, no nibble-1."""
  for s in (HOLD_ON, 1.7, 3.16, 4.42, ACQUIRE_ON - 0.05):
    assert s < ACQUIRE_ON
    det = WindshieldRain()
    _skip_warmup(det)
    for _ in range(MIN_HOLD_N + 8):
      assert not det._update_score(s), s
    assert not det.hold
    assert det._wipe_t0 == 0.0


def test_heavy_rain_one_wipe_then_wait_then_reacquires():
  """Driveway/heavy: one sweep, cancel, settle, then still-clearly-wet wipes again."""
  heavy = HEAVY_ON + 0.8
  det = WindshieldRain()
  _acquire_score(det, heavy)
  assert det._wipe_t0 > 0.0
  assert det._update_score(heavy)
  assert det.hold
  _expire_wipe(det)
  assert not det.hold
  assert not det._update_score(heavy)
  assert not det.hold
  _expire_wait(det)
  assert det._update_score(heavy)
  assert det.hold
  assert det._wipe_t0 > 0.0


def test_heavy_wipe_end_auto_rest_cancels(monkeypatch):
  """Pulse end must drop wipe so Auto extra-forwards rest (Pre-AP Int cancel)."""
  from openpilot.selfdrive.car.tesla import preap_body_controls as body
  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain

  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  reset_windshield_rain()
  reset_auto_gates()
  det = WindshieldRain()
  _acquire_score(det, HEAVY_ON + 0.8)
  det._helper_started = True
  rain._detector = det
  try:
    set_auto_gates(True, "drive")
    assert body.rain_wiper_needed()
    assert not wiper_rest_tx_needed(True)
    _expire_wipe(det)
    assert not det.hold
    assert not body.rain_wiper_needed()
    assert wiper_rest_tx_needed(False)
  finally:
    set_rain_wiper_needed(None)
    reset_auto_gates()
    reset_windshield_rain()


def test_wipe_pulse_wall_clock_drops_hold_between_score_ticks(monkeypatch):
  """If ROAD lags, poll() must still end the one-sweep pulse at WIPE_PULSE_S."""
  import time as time_mod

  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain

  now = {"t": 1000.0}
  monkeypatch.setattr(time_mod, "monotonic", lambda: now["t"])
  det = WindshieldRain()
  _acquire_score(det, HEAVY_ON + 0.8)
  det._helper_started = True
  assert det.poll() is True
  now["t"] += rain.WIPE_PULSE_S - 0.05
  assert det.poll() is True
  now["t"] += 0.10
  assert det.poll() is False
  assert not det.hold
  assert det._wait_t0 > 0.0
  now["t"] += rain.CLEAR_WAIT_S - 0.05
  assert not det._update_score(HEAVY_ON + 0.8)
  assert not det.hold
  now["t"] += 0.10
  assert det._update_score(HEAVY_ON + 0.8)
  assert det.hold


def test_blade_spike_during_clear_wait_does_not_acquire():
  """Blade FOV folds into the post-wipe wait. Spikes there must not nibble-1."""
  det = WindshieldRain()
  _acquire_score(det, HEAVY_ON + 0.8)
  assert det._update_score(0.0)
  assert det.hold
  _expire_wipe(det)
  assert not det._update_score(HEAVY_ON + 3.0)
  assert not det.hold
  _expire_wait(det)
  assert not det._update_score(0.0)
  assert not det.hold


def test_mist_on_glass_acquires_and_fog_does_not():
  """Highway mist/streaks should wipe. Distant fog and dry garage must not."""
  mist = _mist_windshield()
  fine = _fine_mist_windshield()
  fog = _atmos_fog_windshield()
  dry = _dry_windshield()
  assert windshield_mist_score(mist) >= ACQUIRE_ON
  assert windshield_obstruction_score(mist) >= ACQUIRE_ON
  assert windshield_looks_rainy(mist)
  assert windshield_mist_score(fine) >= ACQUIRE_ON
  assert windshield_obstruction_score(fine) >= ACQUIRE_ON
  assert windshield_looks_rainy(fine)
  assert windshield_mist_score(fog) < HOLD_ON
  assert windshield_obstruction_score(fog) < HOLD_ON
  assert not windshield_looks_rainy(fog)
  assert windshield_mist_score(dry) < HOLD_ON
  assert windshield_obstruction_score(dry) < HOLD_ON

  tiny = y_plane_from_nv12(_nv12_buf(mist), max_side=Y_COPY_SIDE)
  assert tiny is not None
  assert windshield_obstruction_score(tiny) >= ACQUIRE_ON
  tiny_fine = y_plane_from_nv12(_nv12_buf(fine), max_side=Y_COPY_SIDE)
  assert tiny_fine is not None
  assert windshield_obstruction_score(tiny_fine) >= ACQUIRE_ON

  det = WindshieldRain()
  _skip_warmup(det)
  saw = False
  for _ in range(MIN_HOLD_N + 2):
    if det.update_from_y(mist):
      saw = True
      break
  assert saw
  assert det.hold
  _expire_wipe(det)
  _expire_wait(det)
  # Light mist must not easy-rewipe every 3 s — back to idle 4 s assess.
  assert not det.update_from_y(mist)
  assert not det.hold
  assert not det.update_from_y(mist)
  assert det.update_from_y(mist)
  assert det.hold

  fog_det = WindshieldRain()
  _skip_warmup(fog_det)
  for _ in range(MIN_HOLD_N + 8):
    assert not fog_det.update_from_y(fog)
  assert not fog_det.hold


def test_light_bokeh_and_wet_drop_fixtures_do_not_acquire():
  """Light sprinkle / residual-drop fixtures stay below acquire. Heavy still wipes."""
  light_y = _bokeh_windshield()
  wet_y = _wet_windshield()
  heavy_y = _heavy_bokeh_windshield()
  assert windshield_obstruction_score(light_y) < ACQUIRE_ON
  assert windshield_obstruction_score(wet_y) < ACQUIRE_ON
  assert windshield_obstruction_score(heavy_y) >= ACQUIRE_ON

  for y in (light_y, wet_y):
    det = WindshieldRain()
    _skip_warmup(det)
    for _ in range(MIN_HOLD_N + 8):
      assert not det.update_from_y(y)
    assert not det.hold

  det = WindshieldRain()
  _skip_warmup(det)
  saw = False
  for _ in range(MIN_HOLD_N + 2):
    if det.update_from_y(heavy_y):
      saw = True
      break
  assert saw
  assert det.hold


def test_single_frame_bokeh_score_does_not_acquire_hold():
  """Sprinkle / residual 3.16–4.42 must not HOLD. First 1–2 frames never acquire."""
  det = WindshieldRain()
  for _ in range(20):
    assert not det._update_score(0.0)
  assert not det.hold
  assert det.ema < HOLD_ON

  flicker = WindshieldRain()
  assert not flicker._update_score(3.16)
  assert not flicker.hold
  assert not flicker._update_score(0.0)
  assert not flicker.hold
  assert flicker._hold_n == 0

  sprinkle = WindshieldRain()
  _skip_warmup(sprinkle)
  for _ in range(MIN_HOLD_N + 8):
    assert not sprinkle._update_score(4.42)
    assert not sprinkle.hold

  latch = WindshieldRain()
  _skip_warmup(latch)
  wet = HEAVY_ON + 0.8
  for _ in range(MIN_HOLD_N - 1):
    assert not latch._update_score(wet)
    assert not latch.hold
  assert latch._update_score(wet)
  assert latch.hold


def test_clear_glass_after_wipe_stays_off_and_auto_cancels(monkeypatch):
  """One wipe then dry assess: HOLD stays off; Auto rest-cancels the Pre-AP Int latch."""
  from openpilot.selfdrive.car.tesla import preap_body_controls as body
  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain

  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  monkeypatch.setattr(body, "RAIN_HELPER_START_DELAY_S", 0.0)
  reset_windshield_rain()
  reset_auto_gates()
  det = WindshieldRain()
  wet = _heavy_bokeh_windshield()
  dry = _dry_windshield()
  saw = False
  for _ in range(16):
    if det.update_from_y(wet):
      saw = True
      break
  assert saw
  assert det.hold
  det._helper_started = True
  rain._detector = det
  try:
    set_auto_gates(True, "drive")
    assert body.rain_wiper_needed()
    assert body.requested_wiper_test()
    assert not wiper_rest_tx_needed(True)
    assert det.update_from_y(dry)
    assert det.hold
    _expire_wipe(det)
    assert not det.hold
    assert not body.rain_wiper_needed()
    assert not body.requested_wiper_test()
    assert wiper_rest_tx_needed(False)
    _expire_wait(det)
    assert not det.update_from_y(dry)
    assert not det.hold
  finally:
    set_rain_wiper_needed(None)
    reset_auto_gates()
    reset_windshield_rain()


def test_y_plane_from_nv12_crops_stride():
  class _Buf:
    width, height, stride = 16, 32, 24
    data = bytes([i % 256 for i in range(24 * 32)])
  y = y_plane_from_nv12(_Buf())
  assert y is not None
  assert y.shape == (32, 16)
  assert y[0, 15] == 15
  assert y[1, 0] == 24


def test_helper_score_period_is_every_few_seconds():
  """Justin: idle assess ~once per 4 s; post-wipe settle then assess."""
  assert 3.5 <= SCORE_PERIOD_S <= 4.5
  assert 0.20 <= SCORE_HZ <= 0.30
  assert STALE_S > SCORE_PERIOD_S
  assert abs(SCORE_HZ - 1.0 / SCORE_PERIOD_S) < 1e-6
  assert 16 <= Y_COPY_SIDE <= 64
  assert MIN_HOLD_N == 2
  assert WARMUP_N == 2
  assert CLEAR_RELEASE_N == 2
  assert WIPE_CLEAR_N == 1
  assert MIN_HOLD_N * SCORE_PERIOD_S <= 8.0
  assert HOLD_ON < ACQUIRE_ON
  assert ACQUIRE_ON <= REWIPE_ON
  assert REWIPE_ON == HEAVY_ON
  assert 1.0 <= WIPE_PULSE_S <= 2.0
  assert 2.5 <= CLEAR_WAIT_S <= 4.0
  assert CLEAR_WAIT_S < SCORE_PERIOD_S


def test_idle_watch_then_wipe_loop_cadence():
  """First look is assess-now (not wipe-first). Idle is ~4 s. Post-wipe is ~3 s."""
  det = WindshieldRain()
  t0 = time.monotonic()
  assert det._next_score_at(0.0, t0) == t0
  assert abs(det._next_score_at(t0, t0) - (t0 + SCORE_PERIOD_S)) < 1e-6
  _acquire_score(det, HEAVY_ON + 0.8)
  t_wipe = det._wipe_t0
  assert t_wipe > 0.0
  assert abs(det._next_score_at(t_wipe, t_wipe) - (t_wipe + WIPE_PULSE_S + CLEAR_WAIT_S)) < 1e-6
  _expire_wipe(det)
  assert not det.hold
  assert abs(det._next_score_at(t_wipe, det._wait_t0) - (det._wait_t0 + CLEAR_WAIT_S)) < 1e-6
  _expire_wait(det)
  now = time.monotonic()
  # Dry assess exits the wipe loop: next look is idle SCORE_PERIOD_S, not 2 s.
  assert not det._update_score(0.0)
  assert not det.hold
  assert det._wait_t0 == 0.0
  assert abs(det._next_score_at(now, now) - (now + SCORE_PERIOD_S)) < 1e-6


def test_dry_fixtures_never_enter_wipe_loop():
  """Bone-dry / clear overcast Auto must not nibble-1. 0987b1a7d wiped immediately."""
  dry_ys = (
    _dry_windshield(),
    _overcast_windshield(),
    _low_bokeh_overcast(),
    _false_bokeh_overcast(),
  )
  for y in dry_ys:
    assert windshield_obstruction_score(y) < ACQUIRE_ON
    det = WindshieldRain()
    for _ in range(MIN_HOLD_N + 12):
      assert not det.update_from_y(y)
    assert not det.hold
    assert det._wipe_t0 == 0.0
    assert not det._post_wipe


def test_live_dry_blob_bokeh_status_is_not_rain():
  """Justin 5f32c450b: dry and light-sprinkle look the same. Must not wipe."""
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import (
    _mist_film_from_feats, _rain_from_feats,
  )

  # Same session: sprinkle / wiping, then confirmed bone-dry last line.
  lives = (
    (7.40, 0.046, 4.7, 0.0, 0.013, 3.01),  # score=4.42 wiping
    (8.83, 0.081, 9.7, 0.0, 0.045, 4.22),  # score=5.44
    (7.00, 0.068, 9.6, 0.0, 0.015, 3.14),  # score=3.19
    (7.55, 0.069, 6.7, 0.0, 0.022, 3.85),  # score=6.29 BONE DRY
  )
  det = WindshieldRain()
  _skip_warmup(det)
  for feats in lives:
    rain = _rain_from_feats(*feats)
    mist = _mist_film_from_feats(*feats)
    obs = max((rain / SCORE_ON) if rain else 0.0, mist)
    assert rain < SCORE_ON, feats
    assert mist < HOLD_ON, (feats, mist)
    assert obs < ACQUIRE_ON, (feats, obs)
    for _ in range(MIN_HOLD_N + 2):
      assert not det._update_score(obs)
      assert not det._update_score(6.29)
  assert not det.hold


def test_live_highway_mist_status_acquires():
  """Justin 2db9e6c30 highway mist: score=2.15 bokeh=1.42 blob=3.93 sparse=3.2."""
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import _mist_film_from_feats

  mist = _mist_film_from_feats(3.93, 0.0, 3.2, 0.0, 0.0, 1.42)
  assert mist >= ACQUIRE_ON
  # Uniform fine film: same blob/sparse, weaker bokeh (ROAD looks through drizzle).
  fine = _mist_film_from_feats(3.93, 0.0, 3.2, 0.0, 0.0, 0.55)
  assert fine >= ACQUIRE_ON
  piled = _mist_film_from_feats(7.90, 0.11, 3.6, 0.0, 0.0, 0.71)
  assert piled >= ACQUIRE_ON
  det = WindshieldRain()
  _skip_warmup(det)
  assert not det._update_score(mist)
  assert not det.hold
  assert det._update_score(mist)
  assert det.hold


def test_warmup_looks_do_not_acquire():
  """First WARMUP_N helper scores never wipe, even if clearly wet."""
  det = WindshieldRain()
  for _ in range(WARMUP_N):
    assert not det._update_score(REWIPE_ON + 1.0)
    assert not det.hold
    assert det._hold_n == 0
  assert det._warm_n == WARMUP_N
  assert not det._update_score(REWIPE_ON + 1.0)
  assert not det.hold
  assert det._update_score(REWIPE_ON + 1.0)
  assert det.hold


def test_after_wipe_dry_score_exits_loop_and_stays_idle():
  """One dry/marginal post-wipe assess ends the loop. Do not wipe forever."""
  det = WindshieldRain()
  _acquire_score(det, HEAVY_ON + 0.8)
  _expire_wipe(det)
  _expire_wait(det)
  assert not det._update_score(0.0)
  assert not det.hold
  for s in (0.0, 0.8, HOLD_ON, ACQUIRE_ON - 0.05):
    assert not det._update_score(s), s
    assert not det.hold
  # Back on idle: one wet look is not a wipe (need two consecutive).
  assert not det._update_score(REWIPE_ON)
  assert not det.hold


def test_light_mist_score_after_wipe_needs_full_reacquire():
  """Justin: light misty rain acquired, then wipe→3s→rewipe over-fired."""
  mist = 6.7
  assert ACQUIRE_ON <= mist < REWIPE_ON
  det = WindshieldRain()
  _skip_warmup(det)
  assert not det._update_score(mist)
  assert det._update_score(mist)
  assert det.hold
  _expire_wipe(det)
  _expire_wait(det)
  assert not det._update_score(mist)
  assert not det.hold
  assert not det._update_score(mist)
  assert det._update_score(mist)
  assert det.hold


def test_clearly_wet_still_wipes_and_rewipes():
  """Real rain still acquires (two idle looks) and re-wipes if still clearly wet."""
  heavy = REWIPE_ON + 0.8
  det = WindshieldRain()
  _skip_warmup(det)
  assert not det._update_score(heavy)
  assert not det.hold
  assert det._update_score(heavy)
  assert det.hold
  _expire_wipe(det)
  _expire_wait(det)
  assert det._update_score(heavy)
  assert det.hold


def test_helper_keeps_road_client_between_score_ticks():
  """Resubscribe-every-4s left frames=1–2 and age_ms=28s on stale first Y."""
  import inspect
  src = inspect.getsource(WindshieldRain._helper_loop)
  assert src.count("_release_vision") == 1
  assert "unsubscribe ROAD between ticks" not in src


def test_helper_numpy_scores_on_period_not_every_road_frame(monkeypatch):
  """Fake ROAD can deliver ~50 Hz. Numpy HOLD ticks must follow SCORE_PERIOD_S."""
  import time as time_mod

  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain

  monkeypatch.setattr(rain, "SCORE_PERIOD_S", 0.30)
  dry = _dry_windshield()
  client = _CountingVisionClient(_nv12_buf(dry))
  det = WindshieldRain()
  det._client = client
  det.start_helper()
  try:
    time_mod.sleep(0.75)
    # Scoring every 20 ms recv would be ~35 frames. Period 0.30 s → ~3.
    assert 1 <= det.n_frames <= 4
  finally:
    det.stop_helper()


def test_stale_does_not_drop_hold_between_score_periods(monkeypatch):
  """poll() between idle ~4 s ticks must not fail-closed; STALE_S is longer than the period."""
  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain

  monkeypatch.setattr(rain, "WIPE_PULSE_S", 60.0)
  det = WindshieldRain()
  _skip_warmup(det)
  for _ in range(MIN_HOLD_N):
    det._update_score(HEAVY_ON + 0.8)
  assert det.hold
  det._helper_started = True
  det._last_frame_t = time.monotonic() - SCORE_PERIOD_S
  assert det.poll() is True
  assert det.hold
  det._last_frame_t = time.monotonic() - (STALE_S + 0.05)
  assert det.poll() is False
  assert not det.hold


def test_y_plane_live_copy_is_tiny_and_still_rainy():
  """Helper must not memcpy full ROAD. Tiny light sprinkle stays idle; tiny heavy wipes."""
  light = _bokeh_windshield()
  light_buf = _nv12_buf(light)
  full = y_plane_from_nv12(light_buf)
  tiny_light = y_plane_from_nv12(light_buf, max_side=Y_COPY_SIDE)
  assert full is not None and tiny_light is not None
  assert full.shape == light.shape
  assert min(tiny_light.shape) <= Y_COPY_SIDE + 8
  assert tiny_light.size < full.size
  assert windshield_looks_rainy(full)
  assert windshield_obstruction_score(tiny_light) < ACQUIRE_ON
  det_light = WindshieldRain()
  _skip_warmup(det_light)
  for _ in range(MIN_HOLD_N + 8):
    assert not det_light.update_from_y(tiny_light)
  assert not det_light.hold

  heavy = _heavy_bokeh_windshield()
  tiny_heavy = y_plane_from_nv12(_nv12_buf(heavy), max_side=Y_COPY_SIDE)
  assert tiny_heavy is not None
  assert windshield_rain_score(tiny_heavy) >= SCORE_ON
  assert windshield_obstruction_score(tiny_heavy) >= ACQUIRE_ON
  det = WindshieldRain()
  _skip_warmup(det)
  saw = False
  for _ in range(MIN_HOLD_N + 4):
    if det.update_from_y(tiny_heavy):
      saw = True
      break
  assert saw


def test_rain_defaults_dry_without_camera(monkeypatch):
  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain

  set_rain_wiper_needed(None)
  monkeypatch.setattr(rain, "windshield_rain_needed", lambda: False)
  assert not rain_wiper_needed()
  monkeypatch.setattr(rain, "windshield_rain_needed", lambda: True)
  assert rain_wiper_needed()
  set_rain_wiper_needed(False)
  assert not rain_wiper_needed()
  set_rain_wiper_needed(None)


def test_auto_only_in_drive_or_reverse(monkeypatch):
  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  set_rain_wiper_needed(True)
  try:
    set_auto_gates(True, "drive")
    assert body.requested_wiper_test()
    set_auto_gates(True, "reverse")
    assert body.requested_wiper_test()
    set_auto_gates(True, "park")
    assert not body.requested_wiper_test()
    set_auto_gates(True, "neutral")
    assert not body.requested_wiper_test()
    set_auto_gates(False, "drive")
    assert not body.requested_wiper_test()
  finally:
    set_rain_wiper_needed(None)
    reset_auto_gates()


def test_auto_gear_aliases_and_enum_name(monkeypatch):
  """Live CS may stringify as GearShifter.drive or D — still Auto-wipe."""
  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  set_rain_wiper_needed(True)

  class _Enumish:
    name = "drive"

    def __str__(self):
      return "CarState.GearShifter.drive"

  try:
    for gear in ("drive", "DRIVE", "D", "reverse", "R", _Enumish()):
      set_auto_gates(True, gear)
      assert body.in_drive_gear(), gear
      assert body.requested_wiper_test(), gear
      rest = _rest()
      held = apply_stw_wiper_beam_nibbles(rest, True, False)
      assert _byte(held) == STW_WIPER_ON
    set_auto_gates(True, "park")
    assert not body.requested_wiper_test()
    set_auto_gates(True, "unknown")
    assert not body.requested_wiper_test()
    line = body._auto_status_line(3, True, True, True, True)
    assert "setting=3" in line
    assert "gear=" in line
    assert "gear_src=" in line
    assert "wipe=1" in line
    assert "collar=1" in line
    assert "wash=0" in line
    assert "installed=" in line
    assert line.startswith("nap wiper auto")
  finally:
    set_rain_wiper_needed(None)
    reset_auto_gates()


def test_auto_reads_preap_inner_cs_out_gear(monkeypatch):
  """StockCCSpoofer CS is the inner Tesla parser. Drive is on CS.out, not CS.gearShifter."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  set_rain_wiper_needed(True)
  reset_auto_gates()
  try:
    inner = SimpleNamespace(
      msg_stw_actn_req={"SpdCtrlLvr_Stat": 0},
      out=SimpleNamespace(gearShifter="drive"),
    )
    assert not hasattr(inner, "gearShifter")
    body.update_live_car_state(inner)
    assert body.vehicle_is_on()
    assert body.in_drive_gear()
    assert body.requested_wiper_test()
    line = body._auto_status_line(3, True, True, True, True)
    assert "gear=drive" in line
    assert "gear_src=out" in line
    assert "wipe=1" in line
    rest = _rest()
    held = apply_stw_wiper_beam_nibbles(rest, True, False)
    assert _byte(held) == STW_WIPER_ON

    inner.out.gearShifter = "park"
    assert not body.in_drive_gear()
    assert not body.requested_wiper_test()
  finally:
    set_rain_wiper_needed(None)
    reset_auto_gates()


def test_auto_gear_int_enum_and_di_gear_tokens(monkeypatch):
  """Live Drive may be GearShifter int 2, Tesla DI_gear 4, or DI_GEAR_D."""
  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  set_rain_wiper_needed(True)
  try:
    for gear in (2, 4, "DI_GEAR_D", "DI_GEAR_R"):
      set_auto_gates(True, gear)
      assert body.in_drive_gear(), gear
      assert body.requested_wiper_test(), gear
    set_auto_gates(True, 1)
    assert not body.in_drive_gear()
    assert not body.requested_wiper_test()
    set_auto_gates(True, 3)
    assert not body.in_drive_gear()
    assert not body.requested_wiper_test()
  finally:
    set_rain_wiper_needed(None)
    reset_auto_gates()


class _DynamicEnum:
  """cereal / opendbc capnp gear: str is 'drive', .name is None, != structs enum."""
  name = None

  def __str__(self):
    return "drive"

  def __repr__(self):
    return "_DynamicEnum"

  def __eq__(self, other):
    return False


def test_auto_gear_dynamic_enum_name_none_str_drive(monkeypatch):
  """Justin: cereal gearShifter is drive _DynamicEnum None — Auto must still wipe."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  set_rain_wiper_needed(True)
  gear = _DynamicEnum()
  assert gear.name is None
  assert str(gear) == "drive"
  try:
    set_auto_gates(True, gear)
    assert body._gear_name(gear) == "drive"
    assert body.in_drive_gear()
    assert body.requested_wiper_test()
    line = body._auto_status_line(3, True, True, True, True)
    assert "gear=drive" in line
    assert "gear_type=_DynamicEnum" in line
    assert "wipe=1" in line
    rest = _rest()
    held = apply_stw_wiper_beam_nibbles(rest, True, False)
    assert _byte(held) == STW_WIPER_ON

    inner = SimpleNamespace(
      gearShifter=0,
      msg_stw_actn_req={"SpdCtrlLvr_Stat": 0},
      out=SimpleNamespace(gearShifter=gear),
    )
    reset_auto_gates()
    set_rain_wiper_needed(True)
    body.update_live_car_state(inner)
    assert body.vehicle_is_on()
    assert body.in_drive_gear()
    assert body.requested_wiper_test()
    line = body._auto_status_line(3, True, True, True, True)
    assert "gear_src=out" in line
    assert "gear=drive" in line
    monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
    monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
    fake = _FakeSpoofer()
    out = body.stock_cc_update_with_overlay(fake, inner, 10, None, 0)
    assert len(out) == 1
    assert out[0][0] == STW_ACTN_RQ_ADDR
  finally:
    set_rain_wiper_needed(None)
    reset_auto_gates()


def test_auto_gear_alias_without_gearShifter(monkeypatch):
  """Inner CS may expose Drive as gear / gear_shifter, not gearShifter."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  set_rain_wiper_needed(True)
  reset_auto_gates()
  try:
    inner = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0}, gear="drive")
    assert not hasattr(inner, "gearShifter")
    body.update_live_car_state(inner)
    assert body.in_drive_gear()
    assert body.requested_wiper_test()
    line = body._auto_status_line(3, True, True, True, True)
    assert "gear=drive" in line
    assert "wipe=1" in line
    inner.gear = "park"
    assert not body.in_drive_gear()
    assert not body.requested_wiper_test()
  finally:
    set_rain_wiper_needed(None)
    reset_auto_gates()


def test_auto_cereal_fallback_when_cs_gearshifter_missing(monkeypatch):
  """Live miss: stock-cc CS has no gearShifter; cereal carState is drive _DynamicEnum."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  set_rain_wiper_needed(True)
  reset_auto_gates()
  gear = _DynamicEnum()
  try:
    inner = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0})
    assert not hasattr(inner, "gearShifter")
    assert not hasattr(inner, "out")
    body.update_live_car_state(inner)
    set_cereal_gear(gear)
    assert body.vehicle_is_on()
    assert body.in_drive_gear()
    assert body.requested_wiper_test()
    line = body._auto_status_line(3, True, True, True, True)
    assert line.startswith("nap wiper auto")
    assert "gear=drive" in line
    assert "gear_src=cereal" in line
    assert "gear_type=_DynamicEnum" in line
    assert "drive=1" in line
    assert "wipe=1" in line
    rest = _rest()
    held = apply_stw_wiper_beam_nibbles(rest, True, False)
    assert _byte(held) == STW_WIPER_ON

    class _Fake:
      def __init__(self):
        self.sent = []

      def _send(self, CS, tesla_can, bus, button):
        msg = (STW_ACTN_RQ_ADDR, bytes([button]), bus)
        self.sent.append(msg)
        return msg

    monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
    monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
    out = body.stock_cc_update_with_overlay(_Fake(), inner, 10, None, 0)
    assert len(out) == 1
    assert out[0][0] == STW_ACTN_RQ_ADDR

    set_cereal_gear("park")
    assert not body.in_drive_gear()
    assert not body.requested_wiper_test()
    park_line = body._auto_status_line(3, True, False, True, False)
    assert "gear=park" in park_line
    assert "wipe=0" in park_line
  finally:
    set_rain_wiper_needed(None)
    reset_auto_gates()


def test_auto_cs_park_wins_over_cereal_drive(monkeypatch):
  """Once CS gear is known Park, stale cereal Drive must not Auto-wipe."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  set_rain_wiper_needed(True)
  reset_auto_gates()
  try:
    inner = SimpleNamespace(gearShifter="park", msg_stw_actn_req={"SpdCtrlLvr_Stat": 0})
    body.update_live_car_state(inner)
    set_cereal_gear(_DynamicEnum())
    assert not body.in_drive_gear()
    assert not body.requested_wiper_test()
    line = body._auto_status_line(3, True, False, True, False)
    assert "gear=park" in line
    assert "gear_src=cs" in line
    assert "wipe=0" in line
  finally:
    set_rain_wiper_needed(None)
    reset_auto_gates()


def test_auto_status_param_is_full_gate_line_not_short_rain(monkeypatch):
  """Justin cat'd hold= ema= because rain _debug overwrote the Auto gates."""
  from openpilot.selfdrive.car.tesla import preap_body_controls as body
  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain

  captured = {}
  monkeypatch.setattr(body, "_put_wiper_status", lambda line: captured.__setitem__("NAPWiperRainStatus", line))
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  det = WindshieldRain()
  det.hold = True
  det.ema = 4.83
  det.last_score = 5.03
  det.last_bokeh = 4.48
  det.last_sparse = 5.4
  det.connected = True
  det._failed = False
  det.n_frames = 8865
  det.stream = "ROAD"
  rain._detector = det
  set_rain_wiper_needed(True)
  try:
    set_auto_gates(True, "drive")
    body._last_auto_log_t = 0.0
    body._last_status_put_t = 0.0
    body._last_status_gate = None
    assert body.requested_wiper_test()
    line = captured["NAPWiperRainStatus"]
    assert line.startswith("nap wiper auto")
    assert "setting=3" in line
    assert "on=1" in line
    assert "gear=drive" in line
    assert "drive=1" in line
    assert "rain=1" in line
    assert "wipe=1" in line
    assert "cancel=0" in line
    assert "installed=" in line
    assert "gear_type=" in line
    assert "hold=1" in line
    assert "holdn=" in line
    assert "warm=" in line
    assert "struct=" in line
    assert "speckle=" in line
    assert "frames=8865" in line
    assert "helper=" in line
    assert "period_s=" in line
    assert "hz=" in line
    assert "clear=" in line
    assert "heavy=" in line
    assert "pulse=" in line
    assert "wait=" in line
    assert "score=" in line
    assert "bokeh=" in line
    assert not line.startswith("hold=")
  finally:
    set_rain_wiper_needed(None)
    reset_auto_gates()
    reset_windshield_rain()


def test_auto_status_put_is_rate_limited_not_every_10ms(monkeypatch):
  """100 Hz Params.put on stock_cc.update lagged engage. 1 Hz or gate change only."""
  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  puts = []
  monkeypatch.setattr(body, "_put_wiper_status", lambda line: puts.append(line))
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  reset_auto_gates()
  set_rain_wiper_needed(True)
  try:
    set_auto_gates(True, "drive")
    for _ in range(20):
      assert body.requested_wiper_test()
    assert 1 <= len(puts) <= 2
    set_rain_wiper_needed(False)
    assert not body.requested_wiper_test()
    assert len(puts) >= 2
  finally:
    set_rain_wiper_needed(None)
    reset_auto_gates()


def test_windshield_rain_needed_does_not_start_helper():
  """poll/needed must not subscribe ROAD. stock_cc starts the helper while Auto."""
  from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain
  reset_windshield_rain()
  assert not rain.windshield_rain_needed()
  assert rain._detector is None


def test_rain_debug_does_not_put_status_param():
  """Short hold= line must not overwrite NAPWiperRainStatus."""
  import inspect
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import WindshieldRain
  src = inspect.getsource(WindshieldRain._debug)
  assert "Params" not in src
  assert ".put(" not in src
  assert "_log_auto_status" not in src
  assert "preap_body_controls" not in src


def test_int_on_ignore_gear_and_camera(monkeypatch):
  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  set_rain_wiper_needed(False)
  set_auto_gates(True, "park")
  try:
    monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
      WIPER_SETTING_INTERMITTENT if key == NAP_WIPER_SPEED else default
    ))
    assert body.requested_wiper_test()
    monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
      WIPER_SETTING_ON if key == NAP_WIPER_SPEED else default
    ))
    assert body.requested_wiper_test()
    monkeypatch.setattr(body, "_param_int", lambda key, default=0: default)
    assert not body.requested_wiper_test()
  finally:
    set_rain_wiper_needed(None)
    reset_auto_gates()


def test_register_defaults_stay_off():
  from opendbc.car.tesla.preap.nap_params import DEFAULTS, NAPParamKeys
  register_nap_body_params()
  assert NAPParamKeys.WIPER_SPEED == NAP_WIPER_SPEED
  assert NAPParamKeys.HIGH_LOW_BEAM == NAP_HIGH_LOW_BEAM
  assert DEFAULTS[NAP_WIPER_SPEED] == 0
  assert DEFAULTS[NAP_HIGH_LOW_BEAM] == 0


def test_das_body_controls_stays_zero_when_settings_on():
  """DAS wiper/beam fields caused a controls mismatch. Leave them at 0."""
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  _, dat, _ = tc.create_body_controls_message(1, 0, CANBUS.party, 1)
  assert (dat[0] >> 4) & 0x0F == 0  # DAS_wiperSpeed
  assert (dat[1] >> 2) & 0x03 == 0  # DAS_highLowBeamDecision
  assert dat[0] & 0x03 == 0         # DAS_headlightRequest


def test_das_body_controls_still_requests_turn_indicator():
  """ALC keep-alive uses DAS_turnIndicatorRequest. Do not zero it with wipers."""
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  _, none, _ = tc.create_body_controls_message(0, 0, CANBUS.party, 1)
  _, left, _ = tc.create_body_controls_message(1, 0, CANBUS.party, 1)
  _, right, _ = tc.create_body_controls_message(2, 0, CANBUS.party, 1)
  assert left != none
  assert right != none
  assert left != right
  for dat in (none, left, right):
    assert (dat[0] >> 4) & 0x0F == 0  # DAS_wiperSpeed stays 0
    assert (dat[1] >> 2) & 0x03 == 0  # DAS_highLowBeamDecision stays 0


def test_create_action_request_overlay_holds_4_and_valid_crc(monkeypatch):
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS, CruiseButtons

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  msg_stw = {
    "MC_STW_ACTN_RQ": 5,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 1,
    "WprSw6Posn": 3,
    "WprWashSw_Psd": 0,
    "HiBmLvr_Stat": 0,
  }
  stock = tc.create_action_request(CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
  monkeypatch.setattr(body, "requested_wiper_test", lambda: True)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: True)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", TeslaCANPreAP.create_action_request)
  addr, dat, bus = body.create_action_request_with_overlay(
    tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
  assert addr == STW_ACTN_RQ_ADDR == stock[0]
  assert bus == stock[2]
  assert _byte(dat) == (STW_WIPER_ON | STW_HIGH_BEAM)
  assert dat[6] & 0x07 == 3  # WprSw6Posn preserved
  assert dat[:2] == stock[1][:2]
  assert dat[3:7] == stock[1][3:7]
  assert dat[7] == tc.stw_crc(dat[:7])
  assert _byte(dat) != STW_WASHER_SPRAY
  # Holding High keeps nibble 4. Rest/IDLE and SNA are not what goes out.
  for _ in range(8):
    _, held, _ = body.create_action_request_with_overlay(
      tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
    assert _byte(held) == (STW_WIPER_ON | STW_HIGH_BEAM)
    assert hibm_nibble(held) == STW_HIGH_BEAM
    assert hibm_nibble(held) != 0
    assert (hibm_nibble(held) & STW_HIBM_MASK) != STW_HIBM_MASK
    assert held[7] == tc.stw_crc(held[:7])


class _FakeSpoofer:
  def __init__(self):
    self.sent = []

  def _send(self, CS, tesla_can, bus, button):
    msg = (STW_ACTN_RQ_ADDR, bytes([button]), bus)
    self.sent.append((button, msg))
    return msg


def test_stock_cc_overlay_forwards_once_and_never_a_second_0x45(monkeypatch):
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0})
  monkeypatch.setattr(body, "requested_wiper_test", lambda: True)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
  assert len(out) == 1
  assert out[0][0] == STW_ACTN_RQ_ADDR
  assert fake.sent[0][0] == 0

  already = [(STW_ACTN_RQ_ADDR, b"\x00" * 8, 0)]
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: already)
  fake.sent.clear()
  out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
  assert out == already
  assert fake.sent == []


def test_disengaged_idle_stalk_still_forwards_when_on(monkeypatch):
  """Car on, NAP not engaged, no stalk pull: On/High must still forward 0x45."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  fake = _FakeSpoofer()
  cs = SimpleNamespace(
    cruiseEnabled=False,
    enableLongControl=False,
    enableJustCC=False,
    latActive=False,
    msg_stw_actn_req={"SpdCtrlLvr_Stat": 0},  # IDLE — no stalk pull
  )
  monkeypatch.setattr(body, "requested_wiper_test", lambda: True)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: True)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
  assert len(out) == 1
  assert out[0][0] == STW_ACTN_RQ_ADDR
  assert fake.sent[0][0] == 0  # forwarded idle lever, not a cruise press


def test_high_holds_4_and_extra_forwards_until_off(monkeypatch):
  """Holding High keeps TXing 0x45 with nibble 4 so rest/IDLE cannot cancel."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0, "MC_STW_ACTN_RQ": 9})
  monkeypatch.setattr(body, "requested_wiper_test", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: True)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])

  forwarded = 0
  for slot in range(1, 8):
    fake.sent.clear()
    out = body.stock_cc_update_with_overlay(fake, cs, slot * 10, None, 0)
    assert len(out) == 1
    assert out[0][0] == STW_ACTN_RQ_ADDR
    forwarded += 1
  assert forwarded == 7
  # Between 10 Hz slots too — bus-0 IDLE cannot sit unopposed.
  fake.sent.clear()
  out = body.stock_cc_update_with_overlay(fake, cs, 11, None, 0)
  assert len(out) == 1
  assert extra_stw_forward_needed([], 11, False, True) is True

  # Leave High — extra-forward stops, real stalk returns.
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  fake.sent.clear()
  out = body.stock_cc_update_with_overlay(fake, cs, 70, None, 0)
  assert out == []
  assert extra_stw_forward_needed([], 70, False, False) is False
  # Come back — hold 4 again, not a one-shot tap.
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: True)
  fake.sent.clear()
  out = body.stock_cc_update_with_overlay(fake, cs, 80, None, 0)
  assert len(out) == 1
  fake.sent.clear()
  out = body.stock_cc_update_with_overlay(fake, cs, 81, None, 0)
  assert len(out) == 1


def test_relayed_bus0_rest_is_replaced_with_held_4():
  """candump bus 0 00ff00 is the live stalk. Replace that payload in place."""
  rest = bytes.fromhex("00ff000000090e80")
  assert hibm_nibble(rest) == 0
  patched = replace_relayed_stw(rest, False, True, crc_fn=lambda payload: 0xAA)
  assert patched[:3] == bytes.fromhex("00ff04")
  assert hibm_nibble(patched) == STW_HIGH_BEAM
  assert hibm_nibble(patched) != 0
  assert hibm_nibble(patched) != STW_HIGH_BEAM_FLASH
  assert (hibm_nibble(patched) & STW_HIBM_MASK) != STW_HIBM_MASK
  # Same live frame — counter and neighboring bytes stay. Only HiBm + CRC.
  assert patched[:2] == rest[:2]
  assert patched[3:7] == rest[3:7]
  assert patched[7] == 0xAA
  # Off does not rewrite rest/IDLE.
  assert replace_relayed_stw(rest, False, False) == rest
  # Holding High keeps replacing with 4, not rest.
  for _ in range(8):
    again = replace_relayed_stw(rest, False, True)
    assert hibm_nibble(again) == STW_HIGH_BEAM
    assert again[:3] != rest[:3]


def test_live_stw_counter_is_not_plus_one():
  assert live_stw_counter({"MC_STW_ACTN_RQ": 9}) == 9
  assert live_stw_counter({"MC_STW_ACTN_RQ": 15}) == 15
  assert live_stw_counter({"MC_STW_ACTN_RQ": 0}) == 0


def test_send_replaced_live_stw_uses_live_counter_not_plus_one():
  from types import SimpleNamespace

  class _Rec:
    def __init__(self):
      self.counter = None

    def create_action_request(self, button, bus, counter, msg_stw=None):
      self.counter = counter
      return (STW_ACTN_RQ_ADDR, b"\x00" * 8, bus)

  rec = _Rec()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0, "MC_STW_ACTN_RQ": 9})
  out = send_replaced_live_stw(_FakeSpoofer(), cs, rec, 0)
  assert out[0] == STW_ACTN_RQ_ADDR
  assert rec.counter == 9  # live MC, not 10


def test_replace_relayed_rest_on_packed_stw_holds_4_same_mc():
  """Edit the packed live 0x45 — same MC, held 4, rest/IDLE gone, CRC re-signed."""
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS, CruiseButtons

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  msg_stw = {
    "MC_STW_ACTN_RQ": 9,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 0,
    "HiBmLvr_Stat": 0,
  }
  _, rest, _ = tc.create_action_request(CruiseButtons.IDLE, CANBUS.party, 9, msg_stw)
  assert rest[:3] == bytes.fromhex("00ff00")
  patched = replace_relayed_stw(rest, False, True, crc_fn=tc.stw_crc)
  assert patched[:3] == bytes.fromhex("00ff04")
  assert hibm_nibble(patched) == STW_HIGH_BEAM
  assert (patched[6] >> 4) & 0x0F == (rest[6] >> 4) & 0x0F  # MC nibble
  assert patched[7] == tc.stw_crc(patched[:7])
  assert patched[7] != rest[7]


def test_high_extra_forward_keeps_sending_00ff04(monkeypatch):
  """High extra-forward keeps 00ff04 on the live-counter 0x45. Never SNA or rest."""
  from types import SimpleNamespace

  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  orig = TeslaCANPreAP.create_action_request
  monkeypatch.setattr(TeslaCANPreAP, "create_action_request", body.create_action_request_with_overlay)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", orig)
  monkeypatch.setattr(body, "requested_wiper_test", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: True)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])

  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={
    "SpdCtrlLvr_Stat": 0,
    "MC_STW_ACTN_RQ": 9,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 0,
    "HiBmLvr_Stat": 0,
  })
  for frame in range(20):
    fake.sent.clear()
    out = body.stock_cc_update_with_overlay(fake, cs, frame, tc, CANBUS.party)
    assert len(out) == 1
    addr, dat, bus = out[0]
    assert addr == STW_ACTN_RQ_ADDR
    assert bus == CANBUS.party
    assert (dat[6] >> 4) & 0x0F == 9
    assert dat[:3] == bytes.fromhex("00ff04")
    assert hibm_nibble(dat) == STW_HIGH_BEAM
    assert (hibm_nibble(dat) & STW_HIBM_MASK) != STW_HIBM_MASK
    assert dat[7] == tc.stw_crc(dat[:7])
    assert fake.sent == []  # High uses send_replaced_live_stw, not _send MC+1


def test_stock_cc_off_does_not_change_forwarding(monkeypatch):
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  reset_auto_gates()
  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0})
  monkeypatch.setattr(body, "requested_wiper_test", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  assert body.stock_cc_update_with_overlay(fake, cs, 10, None, 0) == []
  assert fake.sent == []
  assert extra_stw_forward_needed([], 10, False, False) is False
  reset_auto_gates()


def test_create_action_request_off_matches_stock(monkeypatch):
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS, CruiseButtons

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  msg_stw = {
    "MC_STW_ACTN_RQ": 5,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 1,
    "WprSw6Posn": 2,
  }
  stock = tc.create_action_request(CruiseButtons.SET_ACCEL, CANBUS.party, 6, msg_stw)
  monkeypatch.setattr(body, "requested_wiper_test", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", TeslaCANPreAP.create_action_request)
  test = body.create_action_request_with_overlay(
    tc, CruiseButtons.SET_ACCEL, CANBUS.party, 6, msg_stw)
  assert test == stock


def test_auto_overlay_holds_interval1_on_rain_and_releases_when_dry(monkeypatch):
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS, CruiseButtons

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  msg_stw = {
    "MC_STW_ACTN_RQ": 5,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 1,
    "WprSw6Posn": 0,
    "WprWashSw_Psd": 0,
    "HiBmLvr_Stat": 0,
  }
  stock = tc.create_action_request(CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
  rain = {"on": True}
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  monkeypatch.setattr(body, "rain_wiper_needed", lambda: rain["on"])
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", TeslaCANPreAP.create_action_request)
  set_auto_gates(True, "drive")
  try:
    addr, dat, bus = body.create_action_request_with_overlay(
      tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
    assert addr == STW_ACTN_RQ_ADDR == stock[0]
    assert bus == stock[2]
    assert _byte(dat) != STW_WIPER_ON
    assert _byte(dat) != STW_WASHER_SPRAY
    assert stw_wash(dat) == 0
    assert stw_collar_posn(dat) == STW_COLLAR_INTERVAL1
    assert (dat[6] >> 4) & 0x0F == (stock[1][6] >> 4) & 0x0F  # live MC
    assert dat[7] == tc.stw_crc(dat[:7])
    for _ in range(8):
      _, held, _ = body.create_action_request_with_overlay(
        tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
      assert stw_collar_posn(held) == STW_COLLAR_INTERVAL1
      assert stw_wash(held) == 0
      assert _byte(held) != STW_WIPER_ON
      assert _byte(held) != STW_WASHER_SPRAY

    rain["on"] = False
    _, released, _ = body.create_action_request_with_overlay(
      tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
    assert stw_collar_posn(released) == 0
    assert stw_wash(released) == 0
    assert _byte(released) != STW_WIPER_ON
    assert released[7] == tc.stw_crc(released[:7])

    rain["on"] = True
    set_auto_gates(True, "park")
    _, parked, _ = body.create_action_request_with_overlay(
      tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
    assert stw_collar_posn(parked) == 0
    assert stw_wash(parked) == 0
    assert _byte(parked) != STW_WIPER_ON
  finally:
    reset_auto_gates()


def test_auto_stock_cc_forwards_on_rain_and_cancels_when_dry(monkeypatch):
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  reset_auto_gates()
  fake = _FakeSpoofer()
  cs = SimpleNamespace(
    cruiseEnabled=False,
    latActive=False,
    msg_stw_actn_req={"SpdCtrlLvr_Stat": 0},
  )
  rain = {"on": True}
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  monkeypatch.setattr(body, "requested_wiper_test", lambda: wiper_test_requested(
    WIPER_SETTING_AUTO, rain["on"]))
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])

  out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
  assert len(out) == 1
  assert out[0][0] == STW_ACTN_RQ_ADDR
  fake.sent.clear()
  out = body.stock_cc_update_with_overlay(fake, cs, 11, None, 0)
  assert len(out) == 1
  assert extra_stw_forward_needed([], 11, True, False, collar_hold=True) is True

  rain["on"] = False
  fake.sent.clear()
  # Falling edge must extra-forward rest immediately, not wait for the 10 Hz slot.
  out = body.stock_cc_update_with_overlay(fake, cs, 11, None, 0)
  assert len(out) == 1
  assert out[0][0] == STW_ACTN_RQ_ADDR
  assert extra_stw_forward_needed([], 11, False, False, wiper_cancel=True, cancel_now=True) is True
  assert extra_stw_forward_needed([], 20, False, False) is False
  reset_auto_gates()


def test_auto_stock_cc_forwards_when_gear_only_on_cs_out(monkeypatch):
  """Live miss: inner CS has no gearShifter, Drive is on CS.out. Still TX nibble 1."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  fake = _FakeSpoofer()
  cs = SimpleNamespace(
    msg_stw_actn_req={"SpdCtrlLvr_Stat": 0},
    out=SimpleNamespace(gearShifter="drive"),
  )
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  monkeypatch.setattr(body, "rain_wiper_needed", lambda: True)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  reset_auto_gates()
  try:
    out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
    assert len(out) == 1
    assert out[0][0] == STW_ACTN_RQ_ADDR
    assert extra_stw_forward_needed([], 10, True, False) is True
    cs.out.gearShifter = "park"
    fake.sent.clear()
    out = body.stock_cc_update_with_overlay(fake, cs, 20, None, 0)
    assert len(out) == 1
    assert out[0][0] == STW_ACTN_RQ_ADDR
    assert extra_stw_forward_needed([], 20, False, False, wiper_cancel=True) is True
  finally:
    reset_auto_gates()
    reset_windshield_rain()


def test_auto_stock_cc_releases_when_shifted_to_park(monkeypatch):
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  fake = _FakeSpoofer()
  cs = SimpleNamespace(
    gearShifter="drive",
    msg_stw_actn_req={"SpdCtrlLvr_Stat": 0},
  )
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  monkeypatch.setattr(body, "rain_wiper_needed", lambda: True)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  reset_auto_gates()
  try:
    out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
    assert len(out) == 1
    assert out[0][0] == STW_ACTN_RQ_ADDR
    cs.gearShifter = "park"
    fake.sent.clear()
    out = body.stock_cc_update_with_overlay(fake, cs, 20, None, 0)
    assert len(out) == 1
    assert out[0][0] == STW_ACTN_RQ_ADDR
    assert extra_stw_forward_needed([], 20, False, False, wiper_cancel=True) is True
  finally:
    reset_auto_gates()
    reset_windshield_rain()


def test_auto_does_not_change_high_beam_hold_path(monkeypatch):
  """Auto rain must not switch High off the live-counter 00ff04 hold."""
  from types import SimpleNamespace

  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  orig = TeslaCANPreAP.create_action_request
  monkeypatch.setattr(TeslaCANPreAP, "create_action_request", body.create_action_request_with_overlay)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", orig)
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  monkeypatch.setattr(body, "requested_wiper_test", lambda: wiper_test_requested(
    WIPER_SETTING_AUTO, True))
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: True)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])

  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={
    "SpdCtrlLvr_Stat": 0,
    "MC_STW_ACTN_RQ": 9,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 0,
    "HiBmLvr_Stat": 0,
  })
  for frame in range(5):
    fake.sent.clear()
    out = body.stock_cc_update_with_overlay(fake, cs, frame, tc, CANBUS.party)
    assert len(out) == 1
    addr, dat, bus = out[0]
    assert addr == STW_ACTN_RQ_ADDR
    assert dat[:3] == bytes.fromhex("00ff04")  # High nibble 4, no TIPWIPE
    assert hibm_nibble(dat) == STW_HIGH_BEAM
    assert stw_collar_posn(dat) == STW_COLLAR_INTERVAL1
    assert stw_wash(dat) == 0
    assert _byte(dat) != STW_WASHER_SPRAY
    assert (dat[6] >> 4) & 0x0F == 9
    assert fake.sent == []  # High still uses send_replaced_live_stw, not _send MC+1


def test_auto_does_not_touch_das_body_controls():
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  _, dat, _ = tc.create_body_controls_message(1, 0, CANBUS.party, 1)
  assert (dat[0] >> 4) & 0x0F == 0  # DAS_wiperSpeed
  assert (dat[1] >> 2) & 0x03 == 0  # DAS_highLowBeamDecision
  assert dat[0] & 0x03 == 0         # DAS_headlightRequest


def test_auto_dry_extra_forwards_rest_without_forcing_wipe(monkeypatch):
  """Auto selected + wipe=False must still TX rest so Pre-AP drops ~32s Int."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  reset_auto_gates()
  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0})
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  monkeypatch.setattr(body, "rain_wiper_needed", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  set_auto_gates(True, "drive")
  try:
    assert not body.requested_wiper_test()
    assert wiper_rest_tx_needed(False) is True
    fake.sent.clear()
    out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
    assert len(out) == 1
    assert out[0][0] == STW_ACTN_RQ_ADDR
    assert extra_stw_forward_needed([], 10, False, False, wiper_cancel=True) is True
    assert extra_stw_forward_needed([], 11, False, False, wiper_cancel=True) is False
  finally:
    reset_auto_gates()


def test_auto_rain_rising_edge_stops_rest_cancel(monkeypatch):
  """Cancel while wipe=0 must not keep rest-TX after HOLD/wipe becomes true."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  rain = {"on": False}
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  monkeypatch.setattr(body, "rain_wiper_needed", lambda: rain["on"])
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0})
  reset_auto_gates()
  set_auto_gates(True, "drive")
  try:
    assert not body.requested_wiper_test()
    assert wiper_rest_tx_needed(False) is True
    rest = _rest()
    dry = apply_stw_wiper_beam_nibbles(rest, False, False, clear_wiper=True)
    assert _byte(dry) != STW_WIPER_ON
    assert extra_stw_forward_needed([], 10, False, False, wiper_cancel=True) is True

    rain["on"] = True
    assert body.requested_wiper_test() is True
    assert wiper_rest_tx_needed(True) is False
    wet = apply_stw_wiper_beam_nibbles(rest, True, False, clear_wiper=wiper_rest_tx_needed(True))
    assert _byte(wet) == STW_WIPER_ON
    assert _byte(wet) != STW_WASHER_SPRAY
    assert extra_stw_forward_needed([], 10, True, False, wiper_cancel=False) is True
    fake.sent.clear()
    out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
    assert len(out) == 1
    assert out[0][0] == STW_ACTN_RQ_ADDR
  finally:
    reset_auto_gates()


def test_int_to_off_sends_rest_cancel_burst_then_leaves_stalk(monkeypatch):
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  reset_auto_gates()
  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0})
  on = {"v": True}
  monkeypatch.setattr(body, "requested_wiper_test", lambda: on["v"])
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  try:
    out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
    assert len(out) == 1
    on["v"] = False
    forwarded = 0
    for frame in range(11, 11 + STW_CANCEL_BURST_N):
      fake.sent.clear()
      out = body.stock_cc_update_with_overlay(fake, cs, frame, None, 0)
      assert len(out) == 1
      assert out[0][0] == STW_ACTN_RQ_ADDR
      forwarded += 1
    assert forwarded == STW_CANCEL_BURST_N
    fake.sent.clear()
    out = body.stock_cc_update_with_overlay(fake, cs, 10 + STW_CANCEL_BURST_N * 10, None, 0)
    assert out == []
    assert extra_stw_forward_needed([], 10, False, False) is False
  finally:
    reset_auto_gates()


def test_auto_overlay_dry_clears_held_nibble_not_identity(monkeypatch):
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS, CruiseButtons

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  msg_stw = {
    "MC_STW_ACTN_RQ": 5,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 1,
    "WprSw6Posn": 3,
    "WprWashSw_Psd": 0,
    "HiBmLvr_Stat": 0,
  }
  stock = tc.create_action_request(CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  monkeypatch.setattr(body, "rain_wiper_needed", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", TeslaCANPreAP.create_action_request)
  set_auto_gates(True, "drive")
  try:
    addr, dat, bus = body.create_action_request_with_overlay(
      tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
    assert addr == STW_ACTN_RQ_ADDR == stock[0]
    assert bus == stock[2]
    assert _byte(dat) != STW_WIPER_ON
    assert _byte(dat) != STW_WASHER_SPRAY
    assert stw_wash(dat) == 0
    assert stw_collar_posn(dat) == 0  # Auto dry forces Off over live leftover
    assert (dat[6] >> 4) & 0x0F == (stock[1][6] >> 4) & 0x0F
    assert dat != stock
    held = bytes.fromhex("00ff10") + stock[1][3:]
    cleared = replace_relayed_stw(held, False, False, crc_fn=tc.stw_crc,
                                  clear_wiper=True, collar_posn=0)
    assert _byte(cleared) == 0
    assert stw_collar_posn(cleared) == 0
    assert stw_wash(cleared) == 0
    assert cleared[7] == tc.stw_crc(cleared[:7])
  finally:
    reset_auto_gates()


def test_apply_stw_collar_interval1_clears_tipwipe_keeps_mc():
  rest = _rest()
  held = apply_stw_collar(rest, STW_COLLAR_INTERVAL1)
  assert stw_collar_posn(held) == STW_COLLAR_INTERVAL1
  assert stw_wash(held) == 0
  assert _byte(held) != STW_WIPER_ON
  assert _byte(held) != STW_WASHER_SPRAY
  assert apply_stw_collar(rest, None) == rest
  tip = apply_stw_wiper_beam_nibbles(rest, True, False)
  assert _byte(tip) == STW_WIPER_ON
  auto = apply_stw_collar(tip, STW_COLLAR_INTERVAL1)
  assert stw_wash(auto) == 0
  assert _byte(auto) & STW_WASH_MASK == 0
  assert stw_collar_posn(auto) == STW_COLLAR_INTERVAL1
  live = bytearray(rest)
  live[6] = 0x50  # MC=5, collar Off
  forced = apply_stw_collar(bytes(live), STW_COLLAR_INTERVAL1)
  assert stw_collar_posn(forced) == STW_COLLAR_INTERVAL1
  assert (forced[6] >> 4) & 0x0F == 5
  dry = apply_stw_collar(forced, 0)
  assert stw_collar_posn(dry) == 0
  assert (dry[6] >> 4) & 0x0F == 5


def test_auto_collar_last_wins_live_off(monkeypatch):
  """Live stalk Off is collar=0. Auto wipe must overlay INTERVAL1 on that frame."""
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS, CruiseButtons

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  msg_stw = {
    "MC_STW_ACTN_RQ": 9,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 1,
    "WprSw6Posn": 0,  # live Off
    "WprWashSw_Psd": 0,
    "HiBmLvr_Stat": 0,
  }
  stock = tc.create_action_request(CruiseButtons.IDLE, CANBUS.party, 9, msg_stw)
  assert stw_collar_posn(stock[1]) == 0
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  monkeypatch.setattr(body, "rain_wiper_needed", lambda: True)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", TeslaCANPreAP.create_action_request)
  orig = TeslaCANPreAP.create_action_request
  monkeypatch.setattr(TeslaCANPreAP, "create_action_request", body.create_action_request_with_overlay)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  set_auto_gates(True, "drive")
  try:
    assert body.requested_auto_collar_posn(True) == STW_COLLAR_INTERVAL1
    _, dat, _ = body.create_action_request_with_overlay(
      tc, CruiseButtons.IDLE, CANBUS.party, 9, msg_stw)
    assert stw_collar_posn(dat) == STW_COLLAR_INTERVAL1
    assert stw_wash(dat) == 0
    assert _byte(dat) != STW_WIPER_ON
    assert (dat[6] >> 4) & 0x0F == 9
    assert dat[7] == tc.stw_crc(dat[:7])
    last_mile = overlay_collar_on_can_msg(stock, tc, STW_COLLAR_INTERVAL1)
    assert stw_collar_posn(last_mile[1]) == STW_COLLAR_INTERVAL1
    assert last_mile[1][7] == tc.stw_crc(last_mile[1][:7])

    from types import SimpleNamespace
    fake = _FakeSpoofer()
    cs = SimpleNamespace(msg_stw_actn_req={
      "SpdCtrlLvr_Stat": 0,
      "MC_STW_ACTN_RQ": 9,
      "CRC_STW_ACTN_RQ": 0,
      "DTR_Dist_Rq": 255,
      "VSL_Enbl_Rq": 1,
      "WprSw6Posn": 0,
      "WprWashSw_Psd": 0,
      "HiBmLvr_Stat": 0,
    })
    for frame in range(5):
      fake.sent.clear()
      out = body.stock_cc_update_with_overlay(fake, cs, frame, tc, CANBUS.party)
      assert len(out) == 1
      addr, held, bus = out[0]
      assert addr == STW_ACTN_RQ_ADDR
      assert stw_collar_posn(held) == STW_COLLAR_INTERVAL1
      assert stw_wash(held) == 0
      assert _byte(held) != STW_WIPER_ON
      assert (held[6] >> 4) & 0x0F == 9
      assert held[7] == tc.stw_crc(held[:7])
      assert fake.sent == []  # live-MC replace, not _send +1
  finally:
    monkeypatch.setattr(TeslaCANPreAP, "create_action_request", orig)
    reset_auto_gates()


def test_int_on_still_uses_tipwipe_and_leaves_collar(monkeypatch):
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS, CruiseButtons

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  msg_stw = {
    "MC_STW_ACTN_RQ": 5,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 1,
    "WprSw6Posn": 2,
    "WprWashSw_Psd": 0,
    "HiBmLvr_Stat": 0,
  }
  stock = tc.create_action_request(CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_INTERMITTENT if key == NAP_WIPER_SPEED else default
  ))
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", TeslaCANPreAP.create_action_request)
  addr, dat, bus = body.create_action_request_with_overlay(
    tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
  assert addr == STW_ACTN_RQ_ADDR == stock[0]
  assert _byte(dat) == STW_WIPER_ON
  assert stw_collar_posn(dat) == 2
  assert body.requested_auto_collar_posn(True) is None

