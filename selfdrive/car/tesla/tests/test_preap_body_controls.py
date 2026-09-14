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
  BOKEH_ABSURD,
  BOKEH_ON,
  CLEAR_RELEASE_N,
  CONNECT_RETRY_S,
  FROST_ON,
  HOLD_OFF,
  HOLD_ON,
  ICE_ON,
  MIN_HOLD_N,
  SCORE_ABSURD,
  SCORE_OFF,
  SCORE_ON,
  STREAM_FALLBACK_S,
  WIPE_CLEAR_N,
  WindshieldRain,
  reset_windshield_rain,
  windshield_frost_score,
  windshield_ice_score,
  windshield_looks_rainy,
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
  # Auto + rain is the same 10 Hz Int hold, not the High 10 ms path.
  auto_rain = wiper_test_requested(WIPER_SETTING_AUTO, True)
  auto_dry = wiper_test_requested(WIPER_SETTING_AUTO, False)
  assert extra_stw_forward_needed([], 10, auto_rain, False) is True
  assert extra_stw_forward_needed([], 11, auto_rain, False) is False
  assert extra_stw_forward_needed([], 10, auto_dry, False) is False
  assert extra_stw_forward_needed(existing, 10, auto_rain, False) is False
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
  """Smooth near-glass sky over a textured road — a dry 3X ROAD frame stand-in."""
  rng = np.random.RandomState(seed)
  y = np.full((h, w), 128, np.uint8)
  y[:int(h * 0.32)] = np.linspace(70, 150, int(h * 0.32), dtype=np.uint8)[:, None]
  far = y[int(h * 0.55):]
  far[:] = np.clip(110 + rng.randint(-25, 26, far.shape), 0, 255)
  return y


def _wet_windshield(h=240, w=320, n=25, seed=1) -> np.ndarray:
  """Same dry scene with soft near-field blobs (drops on the glass)."""
  y = _dry_windshield(h, w, seed=0)
  rng = np.random.RandomState(seed)
  r0, r1 = int(h * 0.08), int(h * 0.32)
  c0, c1 = int(w * 0.12), int(w * 0.88)
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
    cy = rng.uniform(h * 0.18, h * 0.62)
    cx = rng.uniform(w * 0.15, w * 0.85)
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
  """Heavier rain: overlapping milky defocus patches over a driveway/tree scene.

  Matches Justin's later live 3X ROAD UI (more rain, large soft blobs over
  the cars — not sharp beads).
  """
  rng = np.random.RandomState(seed)
  y = np.zeros((h, w), np.float32)
  sky_h = int(h * 0.28)
  y[:sky_h] = np.linspace(95, 125, sky_h, dtype=np.float32)[:, None]
  y[sky_h:int(h * 0.55)] = rng.randint(45, 95, (int(h * 0.55) - sky_h, w))
  y[int(h * 0.55):] = np.clip(100 + rng.randint(-18, 19, (h - int(h * 0.55), w)), 0, 255)
  yy, xx = np.mgrid[0:h, 0:w]
  for _ in range(16):
    cy = rng.uniform(h * 0.16, h * 0.68)
    cx = rng.uniform(w * 0.12, w * 0.88)
    sig = rng.uniform(min(h, w) * 0.07, min(h, w) * 0.18)
    amp = rng.uniform(28.0, 55.0) * rng.choice([1.0, 1.0, 0.9, -0.25])
    y += amp * np.exp(-((yy - cy) ** 2 + (xx - cx) ** 2) / (2.0 * sig * sig))
  return np.clip(y, 0, 255).astype(np.uint8)


def test_windshield_camera_wet_holds_and_dry_releases():
  dry = _dry_windshield()
  wet = _wet_windshield()
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

  det = WindshieldRain()
  saw = False
  for _ in range(16):
    if det.update_from_y(bokeh):
      saw = True
      break
  assert saw
  released = False
  for _ in range(CLEAR_RELEASE_N + 40):
    if not det.update_from_y(dry):
      released = True
      break
  assert released


def test_heavy_soft_bokeh_over_driveway_holds():
  """Later live 3X ROAD UI: more rain, large overlapping milky blobs over cars."""
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import (
    _BOKEH_ROWS, _COLS, _band, _near_features,
  )

  dry = _dry_windshield()
  heavy = _heavy_bokeh_windshield()
  _blob, speckle, _sparse, _sat, _structure, bokeh_e = _near_features(_band(heavy, _BOKEH_ROWS, _COLS))
  assert speckle < 0.15
  # Soft defocus is present; rain score may also come from the speckle path.
  assert bokeh_e >= 1.5
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
  released = False
  for _ in range(CLEAR_RELEASE_N + 40):
    if not det.update_from_y(dry):
      released = True
      break
  assert released


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
  foliage[:int(h * 0.56)] = rng.randint(40, 160, (int(h * 0.56), w)).astype(np.uint8)
  lamps = np.full((h, w), 30, np.uint8)
  lamps[20:50, 40:80] = 250
  lamps[20:50, 240:280] = 250
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


def test_helper_holds_soft_bokeh_and_poll_does_not_recv():
  """stock_cc / card is CTRL_HIGH: poll must not drain VisionIpc. Soft bokeh HOLD."""
  import time as time_mod

  bokeh = _bokeh_windshield()
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
    assert det.last_bokeh >= BOKEH_ON
    n_poll = det._poll_recv
    for _ in range(25):
      assert det.poll() is True
    assert det._poll_recv == n_poll
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


def test_dense_bead_sparse_below_8_is_rain():
  """Justin's upper droplet crop was sparse≈6.85 — old SPARSE_MIN=8 rejected it."""
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import _SPARSE_MIN, _SPARSE_RAIN_MIN
  assert _SPARSE_RAIN_MIN <= 6.85 < _SPARSE_MIN
  wet = _wet_windshield(n=40, seed=9)
  assert windshield_rain_score(wet) >= SCORE_ON
  assert windshield_looks_rainy(wet)


def test_windshield_latch_holds_then_releases():
  det = WindshieldRain()
  wet = _wet_windshield()
  dry = _dry_windshield()
  saw_wet = False
  for _ in range(16):
    if det.update_from_y(wet):
      saw_wet = True
      break
  assert saw_wet
  released = False
  for _ in range(CLEAR_RELEASE_N + 40):
    if not det.update_from_y(dry):
      released = True
      break
  assert released
  frost_det = WindshieldRain()
  saw_frost = False
  for _ in range(16):
    if frost_det.update_from_y(_frost_windshield()):
      saw_frost = True
      break
  assert saw_frost


def test_hold_rides_brief_wipe_clear_then_releases_on_sustained_dry():
  """Wet latches. A swipe-clear (few dry frames) must keep HOLD. Long dry releases."""
  det = WindshieldRain()
  wet = _bokeh_windshield()
  dry = _dry_windshield()
  saw = False
  for _ in range(16):
    if det.update_from_y(wet):
      saw = True
      break
  assert saw
  assert det.hold

  assert WIPE_CLEAR_N < CLEAR_RELEASE_N
  for _ in range(WIPE_CLEAR_N):
    assert det.update_from_y(dry)
    assert det.hold
  assert 0 < det._clear_n < CLEAR_RELEASE_N

  # Drops return: clear counter resets, HOLD stays.
  assert det.update_from_y(wet)
  assert det.hold
  assert det._clear_n == 0

  released_at = None
  for n in range(1, CLEAR_RELEASE_N + MIN_HOLD_N + 1):
    if not det.update_from_y(dry):
      released_at = n
      break
  assert released_at is not None
  assert released_at <= CLEAR_RELEASE_N
  assert not det.hold
  assert det._clear_n == 0


def test_sustained_dry_releases_within_bounded_frames():
  """Already-held + truly dry scores must drop HOLD within CLEAR_RELEASE_N frames."""
  assert MIN_HOLD_N <= CLEAR_RELEASE_N
  det = WindshieldRain()
  wet = _bokeh_windshield()
  dry = _dry_windshield()
  for _ in range(16):
    det.update_from_y(wet)
  assert det.hold
  assert det._hold_n >= MIN_HOLD_N

  released_at = None
  for n in range(1, CLEAR_RELEASE_N + 1):
    if not det.update_from_y(dry):
      released_at = n
      break
  assert released_at == CLEAR_RELEASE_N
  assert not det.hold
  assert windshield_obstruction_score(dry) < HOLD_OFF
  rest = _rest()
  assert apply_stw_wiper_beam_nibbles(rest, False, False) == rest


def test_dry_overcast_low_bokeh_does_not_stay_held():
  """Dry overcast / weak false bokeh must not acquire, and must release if held."""
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import (
    _BOKEH_ROWS, _COLS, _band, _near_features,
  )

  overcast = _overcast_windshield()
  low = _low_bokeh_overcast()
  false_bokeh = _false_bokeh_overcast()
  dry = _dry_windshield()
  wet = _bokeh_windshield()
  for y in (overcast, low, false_bokeh, dry):
    obs = windshield_obstruction_score(y)
    _blob, _speckle, _sparse, _sat, _structure, bokeh_e = _near_features(_band(y, _BOKEH_ROWS, _COLS))
    assert obs < HOLD_ON
    assert bokeh_e < BOKEH_ON or obs < HOLD_ON
    assert not windshield_looks_rainy(y)
    det_dry = WindshieldRain()
    for _ in range(CLEAR_RELEASE_N + MIN_HOLD_N + 8):
      assert not det_dry.update_from_y(y)

  # After real rain, overcast residual must release — not stick forever.
  for residual_y in (overcast, low, false_bokeh, dry):
    det = WindshieldRain()
    for _ in range(16):
      det.update_from_y(wet)
    assert det.hold
    released_at = None
    for n in range(1, CLEAR_RELEASE_N + 1):
      if not det.update_from_y(residual_y):
        released_at = n
        break
    assert released_at is not None
    assert released_at <= CLEAR_RELEASE_N
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
  wet = _bokeh_windshield()
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

  # Already held from real rain: exploded frames are dry, not "rain returned".
  det = WindshieldRain()
  for _ in range(16):
    det.update_from_y(wet)
  assert det.hold
  released_at = None
  for n in range(1, CLEAR_RELEASE_N + 1):
    if not det.update_from_y(huge):
      released_at = n
      break
  assert released_at is not None
  assert released_at <= CLEAR_RELEASE_N
  assert not det.hold
  assert det.last_bokeh <= BOKEH_ABSURD

  # Direct garbage obstruction must not latch or keep HOLD.
  det2 = WindshieldRain()
  for _ in range(CLEAR_RELEASE_N + 8):
    assert not det2._update_score(49165.0)
  det3 = WindshieldRain()
  for _ in range(16):
    det3._update_score(HOLD_ON + 0.7)
  assert det3.hold
  for _ in range(WIPE_CLEAR_N):
    assert det3._update_score(0.0)
    assert det3.hold
  released = False
  for _ in range(CLEAR_RELEASE_N):
    if not det3._update_score(49165.0):
      released = True
      break
  assert released
  assert not det3.hold


def test_elevated_residual_below_hold_on_releases_and_does_not_stick():
  """Scores below rain-level HOLD_ON must finish CLEAR_RELEASE_N and drop HOLD."""
  assert WIPE_CLEAR_N < CLEAR_RELEASE_N
  assert HOLD_OFF < HOLD_ON

  def _latch(det):
    for _ in range(16):
      det._update_score(HOLD_ON + 0.7)
    assert det.hold
    assert det._hold_n >= MIN_HOLD_N

  # Residual that sat forever above the old 0.38 HOLD_OFF, including just-shy of rain.
  for residual in (0.0, 0.45, 0.70, 0.95):
    det = WindshieldRain()
    _latch(det)
    for _ in range(WIPE_CLEAR_N):
      assert det._update_score(residual)
      assert det.hold
    released_at = None
    for n in range(1, CLEAR_RELEASE_N + 1):
      if not det._update_score(residual):
        released_at = n
        break
    assert released_at is not None, residual
    assert released_at <= CLEAR_RELEASE_N - WIPE_CLEAR_N
    assert not det.hold

  # Light rain at/above HOLD_ON must keep HOLD (not a false residual release).
  det = WindshieldRain()
  _latch(det)
  for _ in range(CLEAR_RELEASE_N + MIN_HOLD_N + 8):
    assert det._update_score(1.05)
    assert det.hold

  # Brief wipe-clear of zeros, then rain returns: still HOLD.
  det = WindshieldRain()
  _latch(det)
  for _ in range(WIPE_CLEAR_N):
    assert det._update_score(0.0)
    assert det.hold
  assert det._update_score(1.7)
  assert det.hold
  assert det._clear_n == 0


def test_single_frame_bokeh_score_does_not_acquire_hold():
  """Live clear glass was bokeh~3.16 score=0. A one-frame 3.16 must not HOLD."""
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

  latch = WindshieldRain()
  for _ in range(MIN_HOLD_N - 1):
    assert not latch._update_score(3.16)
    assert not latch.hold
  assert latch._update_score(3.16)
  assert latch.hold


def test_y_plane_from_nv12_crops_stride():
  class _Buf:
    width, height, stride = 16, 32, 24
    data = bytes([i % 256 for i in range(24 * 32)])
  y = y_plane_from_nv12(_Buf())
  assert y is not None
  assert y.shape == (32, 16)
  assert y[0, 15] == 15
  assert y[1, 0] == 24


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
    assert "frames=8865" in line
    assert "helper=" in line
    assert "clear=" in line
    assert not line.startswith("hold=")
  finally:
    set_rain_wiper_needed(None)
    reset_auto_gates()
    reset_windshield_rain()


def test_rain_debug_does_not_put_status_param():
  """Short hold= line must not overwrite NAPWiperRainStatus."""
  import inspect
  from openpilot.selfdrive.car.tesla.preap_windshield_rain import WindshieldRain
  src = inspect.getsource(WindshieldRain._debug)
  assert "Params" not in src
  assert ".put(" not in src


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


def test_auto_overlay_holds_nibble_1_on_rain_and_releases_when_dry(monkeypatch):
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
    assert _byte(dat) == STW_WIPER_ON
    assert _byte(dat) != STW_WASHER_SPRAY
    assert dat[6] & 0x07 == 3  # WprSw6Posn preserved
    assert dat[7] == tc.stw_crc(dat[:7])
    for _ in range(8):
      _, held, _ = body.create_action_request_with_overlay(
        tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
      assert _byte(held) == STW_WIPER_ON
      assert _byte(held) != STW_WASHER_SPRAY

    rain["on"] = False
    released = body.create_action_request_with_overlay(
      tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
    assert released == stock

    rain["on"] = True
    set_auto_gates(True, "park")
    parked = body.create_action_request_with_overlay(
      tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
    assert parked == stock
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
  monkeypatch.setattr(body, "requested_wiper_test", lambda: wiper_test_requested(
    WIPER_SETTING_AUTO, rain["on"]))
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])

  out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
  assert len(out) == 1
  assert out[0][0] == STW_ACTN_RQ_ADDR
  assert extra_stw_forward_needed([], 11, True, False) is False

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
    assert dat[:3] == bytes.fromhex("00ff14")  # nibble 1 held with nibble 4
    assert hibm_nibble(dat) == STW_HIGH_BEAM
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
    assert dat == stock
    held = bytes.fromhex("00ff10") + stock[1][3:]
    cleared = replace_relayed_stw(held, False, False, crc_fn=tc.stw_crc, clear_wiper=True)
    assert _byte(cleared) == 0
    assert cleared[7] == tc.stw_crc(cleared[:7])
  finally:
    reset_auto_gates()
