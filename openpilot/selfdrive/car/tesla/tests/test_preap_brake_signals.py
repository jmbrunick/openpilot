"""Pre-AP has no analog brake pressure on buses we already parse.

Long drops on the car's digital Applied flags only. Do not invent a
travel/pressure threshold — we cannot half a boolean the car computed.
"""

from pathlib import Path

from opendbc import DBC_PATH


_PREAP_DBC = Path(DBC_PATH) / "tesla_preap.dbc"


def test_preap_brake_signals_are_digital_only():
  text = _PREAP_DBC.read_text()

  assert "SG_ DI_brakePedal :" in text
  assert "SG_ DI_brakePedalState :" in text
  assert "SG_ driverBrakeStatus :" in text
  assert 'VAL_ 280 DI_brakePedal 1 "Applied" 0 "Not_applied"' in text
  assert 'VAL_ 522 driverBrakeStatus 2 "APPLIED" 1 "NOT_APPLIED"' in text
  assert 'VAL_ 264 DI_pedalPos 255 "SNA"' in text

  # Analog / iBooster signals exist on HW3 / Raven, not Pre-AP.
  assert "IBST_" not in text
  assert "ESP_brakeTorqueTarget" not in text
  assert "IBST_sInputRodDriver" not in text
  assert "ESP_pEstMax" not in text

  # BrakeMessage carries only the 2-bit driver apply enum. Remaining bytes
  # are undefined; do not decode them as pressure or travel.
  brake_msg = text.split("BO_ 522 BrakeMessage:", 1)[1].split("BO_", 1)[0]
  assert "SG_ driverBrakeStatus" in brake_msg
  assert brake_msg.count("SG_") == 1

  # DI_pedalPos is accelerator, not brake travel.
  assert "SG_ DI_pedalPos :" in text
