"""Tipped lane-change confirm vs emergency: replays of logged nudges.

Sep 28 2026 (Scallywag, nap-dev 1119c0f) qlog carState at 10 Hz:

* E1 10:40:08 CT, route 123 seg 16, right tip: -1.88 Nm ~100 ms after
  +0.10 Nm. Old #249 fast rise (1.65 Nm / 150 ms) -> steerDisengage.
* E3 11:06:00 CT, route 124 seg 6, left tip: +0.03 -> +0.69 -> +1.77 Nm.
  Same failure.
* 11:07:15 CT, route 124 seg 7-8, left tip: a confirm that worked
  (peaked +1.17 Nm over ~400 ms).
* Sep 22 22:44:08 CT, route 104 seg 7, left tip: a first nudge at
  +0.73/+0.80/+0.67 Nm never reached steeringPressed; the driver pushed
  again 1.9 s later.

Full Sep 28 review (rlog carState at 20 Hz; R124 ran with unified ON):

* Confirms that worked and must stay confirms: 10:37:55 (R123, right,
  -1.32 Nm), 11:01:35 (left, +1.40), 11:11:26 (right, -1.25).
* 11:10:25 (right tip, |tq| <= 0.3 Nm the whole time): a clean 7 s
  preLaneChange timeout. Must not soft-confirm.
* 11:13:18 (the "emergency cancel", -2.92 Nm) is a sustained right turn
  with a latched stalk, replayed in test_lane_change_turn.py.

Hands-on level is not in these logs; it is taken as 1 while
steeringPressed, else 0. Torque is linearly interpolated to 100 Hz (card
/ controlsd) and 20 Hz (DesireHelper); discrete signals are held.
"""

from cereal import car, log

from openpilot.common.constants import CV
from openpilot.selfdrive.car.car_specific import CarSpecificEvents
from openpilot.selfdrive.controls.lib.desire_helper import (
  LANE_CHANGE_SPEED_MIN,
  DesireHelper,
  LaneChangeDirection,
  LaneChangeState,
)
from openpilot.selfdrive.controls.lib.lane_change_nudge import (
  EMERGENCY_RISE_NM,
  EMERGENCY_RISE_WINDOW_S,
  EMERGENCY_SUSTAIN_S,
  EMERGENCY_TORQUE_NM,
  SOFT_CONFIRM_NM,
  SOFT_CONFIRM_SUSTAIN_S,
  SOFT_YIELD_TRIGGER_NM,
  EmergencyYankTracker,
  TAKEOVER_SUSTAIN_S,
  SoftConfirmTracker,
  TippedLaneChangeTorque,
  is_emergency_yank,
)

EventName = log.OnroadEvent.EventName
DT_CTRL = 0.01
MODEL_EVERY = 5  # 20 Hz DesireHelper inside the 100 Hz loop
MPH = CV.MPH_TO_MS

# (torque Nm, steeringPressed, stalk, left lamp, right lamp, hands-on, vEgo m/s)
E1_RIGHT_1040 = [
  (+0.08, 0, 0, 0, 0, 0, 16.42),
  (+0.15, 0, 0, 0, 0, 0, 16.40),
  (+0.19, 0, 2, 0, 0, 0, 16.41),   # 10:40:08.599 stalk tip right
  (+0.23, 0, 0, 0, 1, 0, 16.45),
  (+0.31, 0, 0, 0, 1, 0, 16.46),
  (+0.10, 0, 0, 0, 1, 0, 16.45),
  (-1.88, 1, 0, 0, 1, 0, 16.45),   # 10:40:09.002 confirm nudge
  (-1.02, 1, 0, 0, 1, 0, 16.48),
  (+0.13, 0, 0, 0, 1, 0, 16.48),
  (-0.00, 0, 0, 0, 1, 0, 16.47),
  (+0.03, 0, 0, 0, 1, 0, 16.45),
]
E3_LEFT_1106 = [
  (-0.07, 0, 0, 0, 0, 0, 14.40),
  (-0.00, 0, 1, 0, 0, 0, 14.39),   # 11:06:00.238 stalk tip left
  (-0.05, 0, 1, 1, 0, 0, 14.39),
  (+0.07, 0, 0, 1, 0, 0, 14.37),
  (+0.03, 0, 0, 1, 0, 0, 14.36),
  (+0.69, 0, 0, 1, 0, 0, 14.36),
  (+1.77, 1, 0, 1, 0, 0, 14.34),   # 11:06:00.736 confirm nudge
  (+1.01, 1, 0, 1, 0, 0, 14.33),
  (-0.00, 0, 0, 1, 0, 0, 14.32),
  (+0.11, 0, 0, 1, 0, 0, 14.30),
  (+0.13, 0, 0, 1, 0, 0, 14.28),
]
OK_LEFT_1107 = [
  (+0.02, 0, 0, 0, 0, 0, 19.16),
  (+0.05, 0, 1, 0, 0, 0, 19.16),   # 11:07:15.439 stalk tip left
  (+0.01, 0, 1, 1, 0, 0, 19.17),
  (+0.03, 0, 0, 1, 0, 0, 19.17),
  (+0.02, 0, 0, 1, 0, 0, 19.18),
  (+0.05, 0, 0, 1, 0, 0, 19.18),
  (+0.20, 0, 0, 1, 0, 0, 19.18),
  (+0.27, 0, 0, 1, 0, 0, 19.18),
  (+0.56, 0, 0, 1, 0, 0, 19.19),
  (+0.78, 0, 0, 1, 0, 0, 19.19),
  (+1.00, 0, 0, 1, 0, 0, 19.19),
  (+1.17, 1, 0, 1, 0, 1, 19.18),   # 11:07:16.435 steeringPressed
  (+1.11, 1, 0, 1, 0, 1, 19.19),
  (+1.01, 1, 0, 1, 0, 1, 19.18),
  (+0.10, 0, 0, 1, 0, 0, 19.19),
  (-0.08, 0, 0, 1, 0, 0, 19.18),
  (+0.20, 0, 0, 1, 0, 0, 19.19),
  (-0.30, 0, 0, 1, 0, 0, 19.19),
  (+0.18, 0, 0, 1, 0, 0, 19.19),
]
PRESSED_1107_IDX = 11
FIRST_TRY_LEFT_0922 = [
  (+0.07, 0, 0, 0, 0, 0, 29.0),
  (+0.02, 0, 1, 0, 0, 0, 29.0),    # stalk tip left
  (-0.11, 0, 1, 1, 0, 0, 29.0),
  (-0.03, 0, 0, 1, 0, 0, 29.0),
  (+0.73, 0, 0, 1, 0, 0, 29.0),    # first nudge, never steeringPressed
  (+0.80, 0, 0, 1, 0, 0, 29.0),
  (+0.67, 0, 0, 1, 0, 0, 29.0),
  (+0.12, 0, 0, 1, 0, 0, 29.0),
  (+0.01, 0, 0, 1, 0, 0, 29.0),
  (+0.10, 0, 0, 1, 0, 0, 29.0),
]

# 20 Hz (rlog). 10:37:55 CT, route 123, right tip.
OK_RIGHT_1037 = [
  (+0.02, 0, 0, 0, 0, 0, 18.01),
  (+0.07, 0, 0, 0, 0, 0, 18.03),
  (+0.13, 0, 0, 0, 0, 0, 18.04),
  (+0.14, 0, 2, 0, 0, 0, 18.06),
  (+0.12, 0, 2, 0, 0, 0, 18.08),
  (+0.12, 0, 0, 0, 1, 0, 18.10),
  (+0.12, 0, 0, 0, 1, 0, 18.12),
  (+0.12, 0, 0, 0, 1, 0, 18.14),
  (+0.12, 0, 0, 0, 1, 0, 18.17),
  (+0.08, 0, 0, 0, 1, 0, 18.19),
  (+0.02, 0, 0, 0, 1, 0, 18.20),
  (-0.07, 0, 0, 0, 1, 0, 18.22),
  (-0.17, 0, 0, 0, 1, 0, 18.25),
  (-0.25, 0, 0, 0, 1, 0, 18.26),
  (-0.32, 0, 0, 0, 1, 0, 18.28),
  (-0.63, 0, 0, 0, 1, 0, 18.30),
  (-1.05, 0, 0, 0, 1, 0, 18.32),
  (-1.24, 0, 0, 0, 1, 0, 18.34),
  (-1.32, 0, 0, 0, 1, 0, 18.37),
  (-1.18, 1, 0, 0, 1, 1, 18.39),
  (-0.93, 1, 0, 0, 1, 1, 18.41),
  (-0.41, 1, 0, 0, 1, 1, 18.42),
  (+0.24, 1, 0, 0, 1, 1, 18.44),
  (+0.24, 0, 0, 0, 1, 0, 18.46),   # 10:37:56.363 laneChangeStarting
  (-0.06, 0, 0, 0, 1, 0, 18.48),
  (-0.03, 0, 0, 0, 1, 0, 18.50),
  (+0.18, 0, 0, 0, 1, 0, 18.52),
  (+0.20, 0, 0, 0, 1, 0, 18.54),
  (+0.14, 0, 0, 0, 1, 0, 18.58),
  (+0.15, 0, 0, 0, 1, 0, 18.61),
  (+0.20, 0, 0, 0, 1, 0, 18.63),
  (+0.19, 0, 0, 0, 1, 0, 18.65),
  (+0.14, 0, 0, 0, 1, 0, 18.66),
  (+0.11, 0, 0, 0, 1, 0, 18.67),
]
# 20 Hz. 11:01:35 CT, route 124, left tip. The stalk shows LEFT in 8 resampled
# samples (0.35-0.40 s); the car's DesireHelper counted it under the 0.40 s
# tip limit (it armed), so the last LEFT sample is taken as released.
OK_LEFT_1101 = [
  (+0.06, 0, 0, 0, 0, 0, 18.45),
  (+0.04, 0, 0, 0, 0, 0, 18.48),
  (+0.03, 0, 1, 0, 0, 0, 18.51),
  (+0.02, 0, 1, 0, 0, 0, 18.54),
  (+0.03, 0, 1, 1, 0, 0, 18.56),
  (+0.04, 0, 1, 1, 0, 0, 18.58),
  (+0.02, 0, 1, 1, 0, 0, 18.61),
  (+0.00, 0, 1, 1, 0, 0, 18.64),
  (+0.00, 0, 1, 1, 0, 0, 18.67),
  (+0.01, 0, 0, 1, 0, 0, 18.70),   # stalk released (see note)
  (+0.02, 0, 0, 1, 0, 0, 18.73),
  (+0.03, 0, 0, 1, 0, 0, 18.76),
  (+0.04, 0, 0, 1, 0, 0, 18.79),
  (+0.06, 0, 0, 1, 0, 0, 18.82),
  (+0.05, 0, 0, 1, 0, 0, 18.84),
  (+0.04, 0, 0, 1, 0, 0, 18.87),
  (+0.01, 0, 0, 1, 0, 0, 18.89),
  (-0.02, 0, 0, 1, 0, 0, 18.91),
  (-0.02, 0, 0, 1, 0, 0, 18.92),
  (-0.00, 0, 0, 1, 0, 0, 18.94),
  (+0.05, 0, 0, 1, 0, 0, 18.97),
  (+0.10, 0, 0, 1, 0, 0, 19.00),
  (+0.20, 0, 0, 1, 0, 0, 19.03),
  (+0.31, 0, 0, 1, 0, 0, 19.06),
  (+0.81, 0, 0, 1, 0, 0, 19.08),
  (+1.38, 0, 0, 1, 0, 0, 19.09),
  (+1.40, 1, 0, 1, 0, 1, 19.12),
  (+1.31, 1, 0, 1, 0, 1, 19.15),   # 11:01:35.738 laneChangeStarting
  (+0.58, 1, 0, 1, 0, 1, 19.17),
  (-0.26, 1, 0, 1, 0, 1, 19.19),
  (-0.14, 0, 0, 1, 0, 0, 19.21),
  (+0.15, 0, 0, 1, 0, 0, 19.23),
  (+0.02, 0, 0, 1, 0, 0, 19.26),
  (-0.21, 0, 0, 1, 0, 0, 19.30),
  (-0.10, 0, 0, 1, 0, 0, 19.31),
  (+0.07, 0, 0, 1, 0, 0, 19.32),
  (+0.05, 0, 0, 1, 0, 0, 19.34),
  (+0.01, 0, 0, 1, 0, 0, 19.35),
  (-0.02, 0, 0, 1, 0, 0, 19.37),
]
# 20 Hz. 11:11:26 CT, route 124, right tip.
OK_RIGHT_1111 = [
  (-0.01, 0, 0, 0, 0, 0, 16.44),
  (+0.06, 0, 0, 0, 0, 0, 16.45),
  (+0.05, 0, 2, 0, 1, 0, 16.47),
  (+0.01, 0, 2, 0, 1, 0, 16.50),
  (+0.04, 0, 0, 0, 1, 0, 16.52),
  (+0.08, 0, 0, 0, 1, 0, 16.54),
  (+0.09, 0, 0, 0, 1, 0, 16.56),
  (+0.10, 0, 0, 0, 1, 0, 16.58),
  (+0.09, 0, 0, 0, 1, 0, 16.59),
  (+0.07, 0, 0, 0, 1, 0, 16.59),
  (+0.06, 0, 0, 0, 1, 0, 16.60),
  (+0.05, 0, 0, 0, 1, 0, 16.61),
  (+0.08, 0, 0, 0, 1, 0, 16.62),
  (+0.12, 0, 0, 0, 1, 0, 16.62),
  (+0.03, 0, 0, 0, 1, 0, 16.63),
  (-0.09, 0, 0, 0, 1, 0, 16.65),
  (-0.17, 0, 0, 0, 1, 0, 16.65),
  (-0.24, 0, 0, 0, 1, 0, 16.66),
  (-0.24, 0, 0, 0, 1, 0, 16.66),
  (-0.23, 0, 0, 0, 1, 0, 16.67),
  (-0.29, 0, 0, 0, 1, 0, 16.68),
  (-0.36, 0, 0, 0, 1, 0, 16.68),
  (-0.78, 0, 0, 0, 1, 0, 16.68),
  (-1.25, 0, 0, 0, 1, 0, 16.67),
  (-1.02, 1, 0, 0, 1, 1, 16.67),
  (-0.66, 1, 0, 0, 1, 1, 16.68),
  (-0.30, 1, 0, 0, 1, 1, 16.69),
  (+0.07, 1, 0, 0, 1, 1, 16.70),
  (+0.26, 0, 0, 0, 1, 0, 16.70),
  (+0.42, 0, 0, 0, 1, 0, 16.71),   # 11:11:27.238 laneChangeStarting
  (+0.33, 0, 0, 0, 1, 0, 16.71),
  (+0.19, 0, 0, 0, 1, 0, 16.71),
  (+0.15, 0, 0, 0, 1, 0, 16.71),
  (+0.12, 0, 0, 0, 1, 0, 16.71),
  (+0.14, 0, 0, 0, 1, 0, 16.71),
  (+0.16, 0, 0, 0, 1, 0, 16.71),
  (+0.18, 0, 0, 0, 1, 0, 16.71),
  (+0.20, 0, 0, 0, 1, 0, 16.70),
  (+0.26, 0, 0, 0, 1, 0, 16.71),
  (+0.32, 0, 0, 0, 1, 0, 16.71),
  (+0.23, 0, 0, 0, 1, 0, 16.72),
]
# 10 Hz. 11:10:25 CT, route 124, right tip; |tq| <= 0.3 Nm; 7 s timeout.
TIMEOUT_RIGHT_1110 = [
  (+0.17, 0, 0, 0, 0, 0, 18.86),
  (+0.20, 0, 0, 0, 0, 0, 18.86),
  (+0.22, 0, 0, 0, 0, 0, 18.86),
  (+0.23, 0, 2, 0, 0, 0, 18.86),
  (+0.20, 0, 0, 0, 1, 0, 18.86),
  (+0.15, 0, 0, 0, 1, 0, 18.86),
  (+0.22, 0, 0, 0, 1, 0, 18.86),
  (+0.22, 0, 0, 0, 1, 0, 18.87),
  (+0.14, 0, 0, 0, 1, 0, 18.88),
  (+0.12, 0, 0, 0, 1, 0, 18.89),
  (+0.16, 0, 0, 0, 1, 0, 18.91),
  (+0.22, 0, 0, 0, 1, 0, 18.91),
  (+0.21, 0, 0, 0, 1, 0, 18.91),
  (+0.16, 0, 0, 0, 1, 0, 18.90),
  (+0.16, 0, 0, 0, 1, 0, 18.92),
  (+0.13, 0, 0, 0, 1, 0, 18.93),
  (+0.12, 0, 0, 0, 1, 0, 18.93),
  (+0.17, 0, 0, 0, 1, 0, 18.92),
  (+0.15, 0, 0, 0, 1, 0, 18.92),
  (+0.15, 0, 0, 0, 1, 0, 18.93),
  (+0.15, 0, 0, 0, 1, 0, 18.93),
  (+0.19, 0, 0, 0, 1, 0, 18.92),
  (+0.21, 0, 0, 0, 1, 0, 18.92),
  (+0.18, 0, 0, 0, 1, 0, 18.93),
  (+0.19, 0, 0, 0, 1, 0, 18.93),
  (+0.20, 0, 0, 0, 1, 0, 18.93),
  (+0.21, 0, 0, 0, 1, 0, 18.92),
  (+0.26, 0, 0, 0, 1, 0, 18.91),
  (+0.26, 0, 0, 0, 1, 0, 18.90),
  (+0.24, 0, 0, 0, 1, 0, 18.92),
  (+0.29, 0, 0, 0, 1, 0, 18.92),
  (+0.21, 0, 0, 0, 1, 0, 18.91),
  (+0.17, 0, 0, 0, 1, 0, 18.91),
  (+0.19, 0, 0, 0, 1, 0, 18.91),
  (+0.18, 0, 0, 0, 1, 0, 18.91),
  (+0.15, 0, 0, 0, 1, 0, 18.90),
  (+0.15, 0, 0, 0, 1, 0, 18.90),
  (+0.19, 0, 0, 0, 1, 0, 18.90),
  (+0.19, 0, 0, 0, 1, 0, 18.90),
  (+0.17, 0, 0, 0, 1, 0, 18.89),
  (+0.21, 0, 0, 0, 1, 0, 18.89),
  (+0.19, 0, 0, 0, 1, 0, 18.89),
  (+0.17, 0, 0, 0, 1, 0, 18.87),
  (+0.19, 0, 0, 0, 1, 0, 18.86),
  (+0.15, 0, 0, 0, 1, 0, 18.86),
  (+0.15, 0, 0, 0, 1, 0, 18.87),
  (+0.12, 0, 0, 0, 1, 0, 18.86),
  (+0.10, 0, 0, 0, 1, 0, 18.85),
  (+0.08, 0, 0, 0, 1, 0, 18.85),
  (+0.08, 0, 0, 0, 1, 0, 18.86),
  (+0.07, 0, 0, 0, 1, 0, 18.86),
  (+0.06, 0, 0, 0, 1, 0, 18.85),
  (+0.07, 0, 0, 0, 1, 0, 18.84),
  (+0.12, 0, 0, 0, 1, 0, 18.84),
  (+0.14, 0, 0, 0, 1, 0, 18.85),
  (+0.05, 0, 0, 0, 1, 0, 18.84),
  (+0.02, 0, 0, 0, 1, 0, 18.83),
  (+0.06, 0, 0, 0, 1, 0, 18.82),
  (+0.09, 0, 0, 0, 1, 0, 18.82),
  (+0.10, 0, 0, 0, 1, 0, 18.81),
  (+0.17, 0, 0, 0, 1, 0, 18.80),
  (+0.21, 0, 0, 0, 1, 0, 18.79),
  (+0.13, 0, 0, 0, 1, 0, 18.80),
  (+0.11, 0, 0, 0, 1, 0, 18.79),
  (+0.20, 0, 0, 0, 1, 0, 18.78),
  (+0.18, 0, 0, 0, 1, 0, 18.78),
  (+0.16, 0, 0, 0, 1, 0, 18.77),
  (+0.17, 0, 0, 0, 1, 0, 18.75),
  (+0.17, 0, 0, 0, 1, 0, 18.73),
  (+0.20, 0, 0, 0, 1, 0, 18.72),
  (+0.20, 0, 0, 0, 1, 0, 18.72),
  (+0.18, 0, 0, 0, 1, 0, 18.72),
  (+0.15, 0, 0, 0, 1, 0, 18.72),
  (+0.16, 0, 0, 0, 1, 0, 18.70),
  (+0.17, 0, 0, 0, 1, 0, 18.69),
  (+0.19, 0, 0, 0, 1, 0, 18.68),
  (+0.20, 0, 0, 0, 1, 0, 18.68),
  (+0.18, 0, 0, 0, 1, 0, 18.67),
  (+0.17, 0, 0, 0, 0, 0, 18.66),
  (+0.20, 0, 0, 0, 0, 0, 18.65),
]


class _Line:
  def __init__(self, y):
    self.y = [y, y, y]


class _Model:
  """Centered in a 3.7 m lane with confident lines (target lock needs them)."""

  def __init__(self):
    self.laneLines = [_Line(y) for y in (-5.55, -1.85, 1.85, 5.55)]
    self.laneLineProbs = [0.9] * 4


def _cp():
  cp = car.CarParams.new_message()
  cp.carFingerprint = "TESLA_MODEL_S_PREAP"
  cp.brand = "tesla"
  cp.pcmCruise = True
  cp.openpilotLongitudinalControl = False
  return cp


def _cs(tq, pressed, stalk, left, right, hands, v):
  cs = car.CarState.new_message()
  cs.cruiseState.enabled = True
  cs.cruiseState.available = True
  cs.gearShifter = "drive"
  cs.vEgo = float(v)
  cs.steeringTorque = float(tq)
  cs.steeringPressed = bool(pressed)
  cs.turnSignalStalkState = int(stalk)
  cs.leftBlinker = bool(left)
  cs.rightBlinker = bool(right)
  cs.steeringTorqueEps = float(hands)
  cs.steeringDisengage = int(hands) >= 2
  return cs


def _upsample(trace, hz=100, src_hz=10):
  """``src_hz`` rows -> ``hz`` rows. Torque linear, discrete signals held."""
  step = int(round(hz / src_hz))
  out = []
  for i, row in enumerate(trace):
    nxt = trace[i + 1] if i + 1 < len(trace) else row
    for k in range(step):
      f = k / step
      out.append((row[0] + (nxt[0] - row[0]) * f,) + tuple(row[1:]))
  return out


class Replay:
  """card (CarSpecificEvents) + controlsd emergency + DesireHelper at 20 Hz."""

  def __init__(self):
    self.cse = CarSpecificEvents(_cp())
    self.dh = DesireHelper()
    self.yank = EmergencyYankTracker()
    self.lct = TippedLaneChangeTorque()
    self.takeover_t: list[float] = []
    self.model = _Model()
    self.prev = _cs(0.0, 0, 0, 0, 0, 0, 20.0)
    self.frame = 0
    self.t = 0.0
    self.disengage_t: list[float] = []
    self.emergency_t: list[float] = []
    self.states: list[tuple[float, int]] = []

  def _dir(self):
    if self.dh.lane_change_state == LaneChangeState.off:
      return 0
    return 1 if self.dh.lane_change_direction == LaneChangeDirection.left else 2

  def step(self, row):
    cs = _cs(*row)
    cc = car.CarControl.new_message()
    cc.enabled = True
    cc.latActive = True
    cc.leftBlinker, cc.rightBlinker = self.dh.lane_change_keep_blinker(
      self.dh.lane_change_state, self.dh.lane_change_direction)
    events = self.cse.update(cs, self.prev, cc)
    if EventName.steerDisengage in events.names:
      self.disengage_t.append(self.t)
    # controlsd lane-change emergency (same inputs as controlsd.py)
    fast = self.yank.update(cs.steeringTorque, DT_CTRL)
    if is_emergency_yank(torque_nm=cs.steeringTorque, hands_on_level=int(row[5]), fast_rise=fast,
                         over_torque=self.yank.over_torque, alc_direction=self._dir()):
      self.emergency_t.append(self.t)
    d = self._dir()
    self.lct.update(torque_nm=cs.steeringTorque, hands_on_level=int(row[5]), direction=d,
                    tipped=d != 0, dt=DT_CTRL)
    if self.lct.takeover:
      self.takeover_t.append(self.t)
    if self.frame % MODEL_EVERY == 0:
      self.dh.update(cs, True, 0.0, model=self.model, engaged=True)
      self.states.append((self.t, self.dh.lane_change_state))
    self.prev = cs
    self.frame += 1
    self.t += DT_CTRL

  def run(self, trace, src_hz=10):
    for row in _upsample(trace, src_hz=src_hz):
      self.step(row)
    return self

  def first_t(self, state):
    for t, s in self.states:
      if s == state:
        return t
    return None


def _legacy_fast_rise(trace):
  """The #249 line: 1.65 Nm within 150 ms of being under 0.55 Nm."""
  rows = _upsample(trace)
  for i, row in enumerate(rows):
    if abs(row[0]) >= 3.0 * SOFT_YIELD_TRIGGER_NM:
      back = rows[max(0, i - 15):i + 1]
      if any(abs(r[0]) < SOFT_YIELD_TRIGGER_NM for r in back):
        return True
  return False


def test_new_thresholds():
  assert EMERGENCY_RISE_NM == 2.2
  assert EMERGENCY_RISE_WINDOW_S == 0.25
  assert EMERGENCY_TORQUE_NM == 2.5
  assert EMERGENCY_SUSTAIN_S == 0.10
  assert TAKEOVER_SUSTAIN_S == 0.30
  assert SOFT_CONFIRM_NM == 0.65
  assert SOFT_CONFIRM_SUSTAIN_S == 0.15
  assert LANE_CHANGE_SPEED_MIN == 20 * MPH


def test_fixtures_reproduce_the_old_misclassification():
  assert _legacy_fast_rise(E1_RIGHT_1040)
  assert _legacy_fast_rise(E3_LEFT_1106)
  assert not _legacy_fast_rise(OK_LEFT_1107)


def test_e1_right_nudge_is_a_confirm_not_an_emergency():
  r = Replay().run(E1_RIGHT_1040)
  assert r.disengage_t == []
  assert r.emergency_t == []
  assert r.first_t(LaneChangeState.laneChangeStarting) is not None
  assert r.dh.lane_change_direction == LaneChangeDirection.right


def test_e3_left_nudge_is_a_confirm_not_an_emergency():
  r = Replay().run(E3_LEFT_1106)
  assert r.disengage_t == []
  assert r.emergency_t == []
  assert r.first_t(LaneChangeState.laneChangeStarting) is not None
  assert r.dh.lane_change_direction == LaneChangeDirection.left


def test_1107_successful_confirm_still_confirms_no_later():
  r = Replay().run(OK_LEFT_1107)
  assert r.disengage_t == []
  assert r.emergency_t == []
  start = r.first_t(LaneChangeState.laneChangeStarting)
  assert start is not None
  # Soft confirm: no later than the steeringPressed sample (+ one model frame).
  assert start <= PRESSED_1107_IDX * 0.1 + 0.05 + 1e-9


def test_first_nudge_below_steering_pressed_now_confirms():
  r = Replay().run(FIRST_TRY_LEFT_0922)
  assert r.disengage_t == []
  assert r.first_t(LaneChangeState.laneChangeStarting) is not None


def _armed_left(v=20.0):
  """Replay the E3 tip only (no nudge): armed preLaneChange, left."""
  r = Replay()
  for row in _upsample([(0.0, 0, 0, 0, 0, 0, v), (0.0, 0, 1, 0, 0, 0, v), (0.0, 0, 1, 1, 0, 0, v),
                        (0.0, 0, 0, 1, 0, 0, v), (0.0, 0, 0, 1, 0, 0, v)]):
    r.step(row)
  assert r.dh.lane_change_state == LaneChangeState.preLaneChange
  assert r.dh.lane_change_direction == LaneChangeDirection.left
  return r


def _hold(r, seconds, tq, pressed=0, hands=0, v=20.0, left=1):
  for _ in range(int(round(seconds / DT_CTRL))):
    r.step((tq, pressed, 0, left, 0, hands, v))


def test_hard_same_direction_yank_releases_promptly():
  r = _armed_left()
  onset = r.t
  for k in range(60):
    tq = min(3.5, 3.5 * (k + 1) * DT_CTRL / 0.05)
    r.step((tq, 1, 0, 1, 0, 3 if tq > 2.0 else 1, 20.0))
  assert r.disengage_t, "hard yank must steerDisengage"
  assert r.disengage_t[0] - onset <= 0.15
  assert r.emergency_t and r.emergency_t[0] - onset <= 0.15
  assert r.dh.lane_change_state == LaneChangeState.off


def test_hard_opposite_yank_releases_promptly():
  r = _armed_left()
  onset = r.t
  for k in range(60):
    tq = -min(3.5, 3.5 * (k + 1) * DT_CTRL / 0.05)
    r.step((tq, 1, 0, 1, 0, 1, 20.0))
  assert r.disengage_t and r.disengage_t[0] - onset <= 0.15
  assert r.dh.lane_change_state == LaneChangeState.off


def _slow_pull(sign):
  # No fast rise: climbs 0.3 -> 3.0 Nm over 0.6 s, then held.
  r = _armed_left()
  for k in range(60):
    r.step((sign * (0.3 + 2.7 * (k + 1) / 60), 1, 0, 1, 0, 1, 20.0))
  crossed = r.t
  _hold(r, 0.5, sign * 3.0, pressed=1, hands=1)
  return r, crossed


def test_slow_hard_pull_against_the_change_releases():
  # Opposite direction: the sustained 2.5 Nm line is an emergency.
  r, crossed = _slow_pull(-1)
  assert r.disengage_t, "sustained opposite pull must steerDisengage"
  assert r.disengage_t[0] <= crossed + 0.02
  assert r.dh.lane_change_state == LaneChangeState.off


def test_sustained_same_direction_pull_is_a_takeover_not_an_emergency():
  # Same direction, sustained: the driver steering through. Lateral
  # yields (takeover) but nothing disengages.
  r, crossed = _slow_pull(+1)
  assert r.disengage_t == []
  assert r.emergency_t == []
  assert r.takeover_t
  first = r.takeover_t[0]
  # 2.5 Nm crossed ~0.07 s before the ramp ends; takeover 0.30 s later.
  assert crossed - 0.1 + TAKEOVER_SUSTAIN_S - 0.02 <= first <= crossed + TAKEOVER_SUSTAIN_S + 0.02
  # Hands off for TAKEOVER_HANDS_OFF_S: the takeover yield ends.
  _hold(r, 0.3, 0.0)
  assert not r.lct.takeover


def test_hands_on_level_3_same_direction_confirm_is_not_an_emergency():
  # Logged firm confirms reach level 3 at ~1.9-2.3 Nm.
  r = _armed_left()
  _hold(r, 0.1, 0.9)
  _hold(r, 0.4, 1.95, pressed=1, hands=3)
  assert r.disengage_t == []
  assert r.emergency_t == []
  assert r.first_t(LaneChangeState.laneChangeStarting) is not None
  # Same torque, opposite direction, level 3: still an emergency.
  assert is_emergency_yank(torque_nm=-1.0, hands_on_level=3, fast_rise=False,
                           over_torque=False, alc_direction=1)
  # Outside a tipped lane change level 3 is an emergency as before.
  assert is_emergency_yank(torque_nm=1.0, hands_on_level=3, fast_rise=False,
                           over_torque=False, alc_direction=0)


def test_one_frame_spike_over_the_lines_is_not_an_emergency():
  r = _armed_left()
  _hold(r, 0.05, 0.3)
  _hold(r, EMERGENCY_SUSTAIN_S - 0.03, 2.8, pressed=1)   # 70 ms above 2.5 Nm
  _hold(r, 0.3, 1.2, pressed=1)
  assert r.emergency_t == []
  assert r.disengage_t == []
  assert r.dh.lane_change_state == LaneChangeState.laneChangeStarting
  # Opposite direction: 70 ms over the torque line is not an emergency either.
  r = _armed_left()
  _hold(r, 0.05, -0.3)
  _hold(r, EMERGENCY_SUSTAIN_S - 0.03, -2.8, pressed=1)
  _hold(r, 0.3, -0.3)
  assert r.emergency_t == []
  assert r.disengage_t == []


def test_hard_confirm_peak_is_not_a_takeover():
  # Logged hard confirms (2.3-3.3 Nm) stay over 2.5 Nm for <= ~0.1 s;
  # 0.2 s is still a confirm, not a takeover.
  r = _armed_left()
  for k in range(50):   # 0.3 -> 2.8 Nm over 0.5 s (not a fast rise)
    r.step((0.3 + 2.5 * (k + 1) / 50, 1, 0, 1, 0, 1, 20.0))
  _hold(r, 0.2, 2.8, pressed=1, hands=2)   # ~0.26 s over 2.5 Nm in total
  _hold(r, 0.3, 0.2)
  assert r.takeover_t == []
  assert r.emergency_t == []
  assert r.disengage_t == []
  assert r.first_t(LaneChangeState.laneChangeStarting) is not None


def test_road_bumps_and_crown_do_not_confirm():
  r = _armed_left(v=45 * MPH)
  v = 45 * MPH
  # Hands-off lane keep: |tq| p99.9 ~0.5 Nm; crown bias held.
  _hold(r, 1.0, 0.50, v=v)
  # Single-sample bumps up to 1.0 Nm (one 10 Hz sample, 100 ms interpolated).
  for _ in range(5):
    for row in _upsample([(0.1, 0, 0, 1, 0, 0, v), (1.0, 0, 0, 1, 0, 0, v), (0.1, 0, 0, 1, 0, 0, v)]):
      r.step(row)
  # Washboard: +/-0.8 Nm alternating every 50 ms.
  for k in range(40):
    _hold(r, 0.05, 0.8 if k % 2 == 0 else -0.8, v=v)
  # Just under the line, held.
  _hold(r, 1.0, SOFT_CONFIRM_NM - 0.03, v=v)
  assert r.dh.lane_change_state == LaneChangeState.preLaneChange
  assert r.first_t(LaneChangeState.laneChangeStarting) is None
  assert r.disengage_t == []


def test_soft_confirm_needs_the_sustain_time():
  r = _armed_left(v=45 * MPH)
  _hold(r, SOFT_CONFIRM_SUSTAIN_S - 0.06, 0.8, v=45 * MPH)
  _hold(r, 0.2, 0.1, v=45 * MPH)
  assert r.dh.lane_change_state == LaneChangeState.preLaneChange
  _hold(r, SOFT_CONFIRM_SUSTAIN_S + 0.06, 0.8, v=45 * MPH)
  assert r.dh.lane_change_state == LaneChangeState.laneChangeStarting


def test_soft_confirm_only_same_direction_and_at_speed():
  r = _armed_left(v=45 * MPH)
  _hold(r, 1.0, -0.9, v=45 * MPH)   # opposite, below steeringPressed
  assert r.dh.lane_change_state == LaneChangeState.preLaneChange
  slow = 19 * MPH
  r = _armed_left(v=slow)
  _hold(r, 1.0, 0.9, v=slow)
  assert r.dh.lane_change_state == LaneChangeState.preLaneChange
  t = SoftConfirmTracker()
  assert not any(t.update(torque_nm=0.9, direction=1, armed=False, dt=0.05) for _ in range(10))
  assert not any(t.update(torque_nm=-0.9, direction=1, armed=True, dt=0.05) for _ in range(10))
  assert not any(t.update(torque_nm=0.9, direction=0, armed=True, dt=0.05) for _ in range(10))
  assert any(t.update(torque_nm=-0.9, direction=2, armed=True, dt=0.05) for _ in range(10))


def test_opposite_pull_still_cancels_after_soft_confirm():
  r = _armed_left(v=45 * MPH)
  _hold(r, 0.25, 0.8, v=45 * MPH)
  assert r.dh.lane_change_state == LaneChangeState.laneChangeStarting
  assert r.dh.target_locked
  _hold(r, 0.1, -1.1, pressed=1, hands=1, v=45 * MPH)
  assert r.dh.lane_change_state == LaneChangeState.laneChangeStarting   # spring-back length
  _hold(r, 0.2, -1.1, pressed=1, hands=1, v=45 * MPH)
  assert r.dh.lane_change_state == LaneChangeState.off


def test_unarmed_nudge_unchanged():
  # No stalk tip: the E3 nudge on its own is ordinary lane-keep override.
  r = Replay()
  trace = [(tq, p, 0, 0, 0, h, v) for tq, p, _s, _l, _r, h, v in E3_LEFT_1106]
  r.run(trace)
  assert r.first_t(LaneChangeState.preLaneChange) is None
  assert r.first_t(LaneChangeState.laneChangeStarting) is None
  assert r.disengage_t == []
  # Light held torque with no lane change armed does nothing.
  r = Replay()
  _hold(r, 1.0, 0.9, left=0, v=45 * MPH)
  assert r.dh.lane_change_state == LaneChangeState.off
  # Hands-on level 2 edge with no blinker still disengages as before.
  r = Replay()
  _hold(r, 0.1, 0.2, left=0)
  _hold(r, 0.05, 1.5, pressed=1, hands=2, left=0)
  assert r.disengage_t


def _assert_logged_confirm(trace, direction):
  r = Replay().run(trace, src_hz=20)
  assert r.disengage_t == []
  assert r.emergency_t == []
  assert r.takeover_t == []
  assert r.first_t(LaneChangeState.laneChangeStarting) is not None
  assert r.dh.lane_change_direction == direction
  return r


def test_sep28_1037_right_confirm_still_confirms():
  _assert_logged_confirm(OK_RIGHT_1037, LaneChangeDirection.right)


def test_sep28_1101_left_confirm_still_confirms():
  _assert_logged_confirm(OK_LEFT_1101, LaneChangeDirection.left)


def test_sep28_1111_right_confirm_still_confirms():
  _assert_logged_confirm(OK_RIGHT_1111, LaneChangeDirection.right)


def test_sep28_1110_light_torque_timeout_does_not_soft_confirm():
  r = Replay().run(TIMEOUT_RIGHT_1110)
  assert r.first_t(LaneChangeState.preLaneChange) is not None
  assert r.first_t(LaneChangeState.laneChangeStarting) is None
  assert r.disengage_t == []
  assert r.emergency_t == []
  assert r.takeover_t == []
